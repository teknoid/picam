#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>

#include "gpio.h"
#include "rfsniffer.h"
#include "utils.h"

#define BUFFER				0xff
#define BLOCKSIZE			64

#define PULSE_TOP_X			5

// stream_decoder: ring buffers
static uint8_t lstream[0xffff], hstream[0xffff];
static uint16_t streampointer;

// pulse counters: index equals pulse length x 100, e.g. 1=100µs, 2=200µs ... 255=25500µs
static uint16_t lpulse_counter[BUFFER];
static uint16_t hpulse_counter[BUFFER];

extern rfsniffer_config_t *rfcfg;

static void dump_pulse_counters() {
	printf("\nLCOUNTER   ");
	for (int i = 0; i < BUFFER; i++)
		if (lpulse_counter[i])
			printf("%d:%d ", i, lpulse_counter[i]);
	printf("\n");

	printf("HCOUNTER   ");
	for (int i = 0; i < BUFFER; i++)
		if (hpulse_counter[i])
			printf("%d:%d ", i, hpulse_counter[i]);
	printf("\n");
}

static void dump_stream(uint16_t start, uint16_t stopp) {
	uint8_t l, h;
	uint16_t count = (stopp > start) ? stopp - start : 0xffff - start + stopp;
	printf("DUMP %05u+%u :: ", start, count);
	while (start != stopp) {
		l = lstream[start];
		h = hstream[start];
		start++;
		printf("H%03u L%03u ", h, l);
	}
	printf("\n");
}

//
// short low or long low pulse
//   _   _     _     _
//    |_|       |___|
//      0           1
//
// this signal can be encoded as 01 10 sequences: clock pulse + data pulse:
// either short or long distance from clock to data pulse:
//     _   _               _      _
//    | |_| |____|        | |____| |_|
//        0      1               1   0
// =            01                  10
// =             0                   1
//
// this may require a dummy bit at end of transmission
//
static uint64_t stream_probe_low(uint16_t pos, uint8_t bits, uint16_t divider) {
	uint64_t code = 0;
	uint8_t l, h;
	for (int i = 0; i < bits; i++) {
		l = lstream[pos];
		h = hstream[pos];

		if (!l && !h)
			return 0;

		if (l > divider)
			code++;

		code <<= 1;
		pos++;
	}
	return code;
}

//
// short high pulse followed by long low pulse or long high + short low pulse, no clock
//     _           ___
//   _| |___     _|   |_
//      0             1
static uint64_t stream_probe_high(uint16_t pos, uint8_t bits, uint16_t divider) {
	uint64_t code = 0;
	uint8_t l, h;
	for (int i = 0; i < bits; i++) {
		l = lstream[pos];
		h = hstream[pos];

		if (!l && !h)
			return 0;

		if (h > divider)
			code++;

		code <<= 1;
		pos++;
	}
	return code;
}

// rewind stream_write position
static uint16_t stream_rewind(uint16_t start, uint16_t positions) {
	for (int i = 0; i < positions; i++)
		start--;
	return start;
}

// forward stream to position
static uint16_t stream_forward(uint16_t start, uint16_t positions) {
	for (int i = 0; i < positions; i++)
		start++;
	return start;
}

// counts the distance between the two stream positions
static uint16_t stream_distance(uint16_t start, uint16_t end) {
	uint16_t distance = 0;
	while (start++ != end)
		distance++;
	return distance;
}

static void stream_probe(uint8_t *stream, uint16_t start, uint16_t stopp) {
	if (rfcfg->verbose)
		printf("DECODER probe [%05u:%05u] %u samples\n", start, stopp, stream_distance(start, stopp));

	uint16_t count = (stopp > start) ? stopp - start : 0xffff - start + stopp;
	if (count < 16)
		return; // noise

//	if (stream == lstream)
//		printf("L-");
//	else
//		printf("H-");
//	stream_dump(start, stopp);

	uint64_t l = stream_probe_low(stopp - 64, 64, 15);
	uint64_t h = stream_probe_high(stopp - 64, 64, 6);
	printf("probe_low 0x%016llx == %s probe_high 0x%016llx == %s\n", l, printbits64(l, SPACEMASK64), h, printbits64(h, SPACEMASK64));
}

static uint16_t stream_start_sequence(uint8_t *stream, uint16_t start, uint16_t stopp, uint8_t top1, uint8_t top2, uint8_t top3) {
	int match = 0;
	while (start != stopp) {
		uint8_t pulse = stream[start++];
		if (pulse == top1 || pulse == top2 || pulse == top3)
			match++;
		else
			match = 0;

		if (match == 3)
			return start - 3; // 3 matches in sequence
	}
	return 0;
}

static uint16_t stream_stopp_sequence(uint8_t *stream, uint16_t start, uint16_t stopp, uint8_t top1, uint8_t top2, uint8_t top3) {
	int mismatch = 0;
	while (start != stopp) {
		uint8_t pulse = stream[start++];
		if (pulse == top1 || pulse == top2 || pulse == top3)
			mismatch = 0;
		else
			mismatch++;

		if (mismatch == 3)
			return start - 3; // 3 mismatches in sequence
	}
	return 0;
}

static void stream_count_pulses(uint16_t start, uint16_t stopp) {
	uint16_t lmaxc, hmaxc, ltop_count[PULSE_TOP_X], htop_count[PULSE_TOP_X];
	uint8_t l, h, lmaxp, hmaxp, ltop_pulse[PULSE_TOP_X], htop_pulse[PULSE_TOP_X];

	// clear pulse counters
	memset(lpulse_counter, 0, sizeof(lpulse_counter));
	memset(hpulse_counter, 0, sizeof(hpulse_counter));

	// count pulses - pulse length / 100 is the array index
	for (uint16_t i = start; i < stopp; i++) {
		l = lstream[i];
		h = hstream[i];
		if (l == 0 || h == 0)
			continue;
		lpulse_counter[l]++;
		hpulse_counter[h]++;
	}

	// now find the top-x pulses
	for (int t = 0; t < PULSE_TOP_X; t++) {
		hmaxc = lmaxc = hmaxp = lmaxp = 0;
		for (int i = 0; i < BUFFER; i++) {
			if (lpulse_counter[i] > lmaxc) {
				lmaxc = lpulse_counter[i];
				lmaxp = i;
			}
			if (hpulse_counter[i] > hmaxc) {
				hmaxc = hpulse_counter[i];
				hmaxp = i;
			}
		}
		ltop_count[t] = lmaxc;
		ltop_pulse[t] = lmaxp;
		htop_count[t] = hmaxc;
		htop_pulse[t] = hmaxp;
		lpulse_counter[lmaxp] = 0;
		hpulse_counter[hmaxp] = 0;
	}

	// put top pulse length into odd and top pulse count into even index
	for (int t = 0; t < PULSE_TOP_X; t++) {
		lpulse_counter[2 * t] = ltop_pulse[t];
		lpulse_counter[2 * t + 1] = ltop_count[t];
		hpulse_counter[2 * t] = htop_pulse[t];
		hpulse_counter[2 * t + 1] = htop_count[t];
	}
}

void* stream_decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	if (!rfcfg->quiet)
		printf("DECODER run every %d µs, %s\n", rfcfg->decoder_delay, rfcfg->collect_identical_codes ? "collect identical codes" : "process each code separately");

	uint16_t start, stopp, count, i, streampointer_last = 0;
	uint8_t l, h;

	while (1) {
		msleep(rfcfg->decoder_delay);

		// if we see ~8 pulses between 300-5000µs we assume receiving is in progress, so wait another 100ms
		while (1) {
			start = stream_rewind(streampointer, 8);
			int valid = 0;
			for (i = 0; i < 8; i++) {
				l = lstream[start];
				if (3 < l && l < 50)
					valid++;

				h = hstream[start];
				if (3 < h && h < 50)
					valid++;
			}

			// start decoding
			if (valid < 10)
				break;

			if (rfcfg->verbose)
				printf("DECODER receiving in progress, wait\n");

			// wait and check again
			msleep(100);
		}

		// this is the block we have to analyze in this round
		start = streampointer_last;
		stopp = streampointer;

		if (rfcfg->verbose)
			printf("DECODER analyze [%05u:%05u] %u samples\n", start, stopp, stream_distance(start, stopp));

		while (1) {
			count = stream_distance(start, stopp);

			// not enough samples available -> next round
			if (count < BLOCKSIZE)
				break;

			uint16_t block = stream_forward(start, BLOCKSIZE);
			stream_count_pulses(start, block);

			uint8_t l1p = lpulse_counter[0], l2p = lpulse_counter[2], l3p = lpulse_counter[4];
			uint8_t l1c = lpulse_counter[1], l2c = lpulse_counter[3], l3c = lpulse_counter[5];
			if ((l1c + l2c + l3c) < BLOCKSIZE / 2) {
				// we found only noise -> forward half blocksize and analyze next block
				start = stream_forward(start, BLOCKSIZE / 2);
				continue;
			}

			if (rfcfg->verbose)
				for (int t = 0; t < 3; t++)
					printf("TOP%d: L %03d:%02d H %03d:%02d\n", t + 1, lpulse_counter[2 * t], lpulse_counter[2 * t + 1], hpulse_counter[2 * t], hpulse_counter[2 * t + 1]);

			// find the block with all top-3 pulses
			uint16_t block_start = 0, block_stopp = 0;
			block_start = stream_start_sequence(lstream, start, stopp, l1p, l2p, l3p);
			block_stopp = stream_stopp_sequence(lstream, block_start, stopp, l1p, l2p, l3p);

			// validate block window
			if (block_start == 0 || block_stopp == 0) {
				if (rfcfg->verbose)
					printf("DECODER stream start/stopp sequence error: start=%05u stopp=%05u\n", block_start, block_stopp);
				start = block;
				break;
			}

			// analyze this block + forward stream to find next
			stream_probe(lstream, block_start, block_stopp);
			start = block_stopp;
			continue;
		}

		// next round
		streampointer_last = start;
	}
	return (void*) 0;
}

void* stream_sampler(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

// elevate realtime priority for sampler thread
	if (elevate_realtime(3) < 0)
		return (void*) 0;

	const char *name = rfcfg->rx;
	while (*name >= 'A')
		name++;

	char buf[32];
	snprintf(buf, 32, "/usr/bin/gpio edge %d both", atoi(name));
	system(buf);

	// poll setup
	snprintf(buf, 32, "/sys/class/gpio/gpio%d/value", atoi(name));
	struct pollfd fdset[1];
	fdset[0].fd = open(buf, O_RDONLY);
	fdset[0].events = POLLPRI;
	fdset[0].revents = 0;

	uint32_t pulse, last;
	uint8_t pin, dummy;

	// initialize tLast for correct 1st pulse calculation
	last = gpio_micros();

	// initialize the matrix
	matrix_init();

	streampointer = 0;
	while (1) {
		// wait for interrupt
		poll(fdset, 1, -1);

		// sample time + pin state
		pulse = gpio_micros_since(&last);
		pin = gpio_get(rfcfg->rx);

		// rewind & clear for next poll
		lseek(fdset[0].fd, 0, SEEK_SET);
		read(fdset[0].fd, &dummy, 1);

		// validate
		if (pulse < (rfcfg->noise) || pulse > 0xff * 100)
			continue;

		// round pulse length to multiples of 100 and divide by 100
		if ((pulse % 100) < 50)
			pulse = pulse / 100;
		else
			pulse = (pulse / 100) + 1;

		if (pin) {
			// that was a LOW pulse
			lstream[streampointer] = (uint8_t) pulse;
			if (rfcfg->sample_on_0)
				streampointer++;
		} else {
			// that was a HIGH pulse
			hstream[streampointer] = (uint8_t) pulse;
			if (rfcfg->sample_on_1)
				streampointer++;
		}
	}
	return (void*) 0;
}

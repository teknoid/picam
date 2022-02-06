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
#define SYMBOLS				8

// ring buffers for low and high pulses
static uint8_t lstream[0xffff], hstream[0xffff];
static uint16_t streampointer;

extern rfsniffer_config_t *rfcfg;

// rewind stream to position
static uint16_t backward(uint16_t start, uint16_t positions) {
	for (int i = 0; i < positions; i++)
		start--;
	return start;
}

// forward stream to position
static uint16_t forward(uint16_t start, uint16_t positions) {
	for (int i = 0; i < positions; i++)
		start++;
	return start;
}

// counts the distance between the two stream positions
static uint16_t distance(uint16_t start, uint16_t end) {
	uint16_t distance = 0;
	while (start++ != end)
		distance++;
	return distance;
}

static void dump(uint8_t *stream, uint16_t start, uint16_t stopp, uint16_t marker) {
	// noise marker
	uint16_t n = stream == lstream ? lstream[marker] : hstream[marker];

	// the opposite stream
	uint8_t *xstream = stream == lstream ? hstream : lstream;

	printf("DUMP %05u+%2u marker=%05u N=noise L=low H=high\n", start, distance(start, stopp), marker);

	uint16_t p = start;
	printf("N ");
	while (p++ != stopp)

		if (p == marker)
			printf("  ▼");
		else
			printf("   ", stream[p] - n > 0 ? stream[p] - n : n - stream[p]);
	printf("\n");

	p = start;
	printf("%s ", stream == lstream ? "L" : "H");
	while (p++ != stopp)
		printf("%3d", stream[p]);
	printf("\n");

	p = start;
	printf("%s ", xstream == lstream ? "L" : "H");
	while (p++ != stopp)
		printf("%3d", xstream[p]);
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
static uint64_t probe_low(uint16_t pos, uint8_t bits, uint16_t divider) {
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
static uint64_t probe_high(uint16_t pos, uint8_t bits, uint16_t divider) {
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

static void probe(uint16_t start, uint16_t stopp) {
	if (rfcfg->verbose) {
		printf("DECODER probing [%05u:%05u] %u samples\n", start, stopp, distance(start, stopp));
		dump(lstream, start - 16, start + 32, start);
		dump(lstream, stopp - 32, stopp + 16, stopp);
	}

	uint64_t l = probe_low(stopp - 64, 64, 15);
	uint64_t h = probe_high(stopp - 64, 64, 6);
	printf("probe_low 0x%016llx == %s probe_high 0x%016llx == %s\n", l, printbits64(l, SPACEMASK64), h, printbits64(h, SPACEMASK64));

	printf("\n");
}

static int pattern(uint8_t *stream, uint16_t start) {
	// fill pattern window
	uint8_t p0 = stream[start], p1 = stream[start + 1], p2 = stream[start + 2], p3 = stream[start + 3];

	// contains zeros
	if (p0 == 0 || p1 == 0 || p2 == 0 || p3 == 0)
		return 0;

	// [3 3 3 3] detect identical values with tolerance
	if (p0 == p1 && p1 == p2 && p2 == p3)
		return 1;

	// [9 9 19 19] [9 19 19 9] symmetric identical values
	if (p0 == p1 && p2 == p3)
		return 1;
	if (p0 == p3 && p1 == p2)
		return 1;

	// [5 10 5 10] alternating identical values
	if (p0 == p2 && p1 == p3)
		return 1;

	// [19 19 19 9] 3 identical values
	if (p0 == p1 && p1 == p2)
		return 1;
	if (p0 == p1 && p1 == p3)
		return 1;
	if (p0 == p2 && p2 == p3)
		return 1;
	if (p1 == p2 && p2 == p3)
		return 1;

	// [6  5  7  6] square sum max +- 1
	int q = (p0 + p1 + p2 + p3) / 4;
	int qmin = q - 1, qmax = q + 1;
	int tolerate = p0 == q || p0 == qmin || p0 == qmax;
	tolerate &= p1 == q || p1 == qmin || p1 == qmax;
	tolerate &= p2 == q || p2 == qmin || p2 == qmax;
	tolerate &= p3 == q || p3 == qmin || p3 == qmax;
	if (tolerate)
		return 1;

	// [5 10 5 15] multiples of smallest
	uint8_t min = p0;
	if (p1 < min)
		min = p1;
	if (p2 < min)
		min = p2;
	if (p3 < min)
		min = p3;
	int modulos = (p0 % min) + (p1 % min) + (p2 % min) + (p3 % min);
	if (min > 2 && !modulos)
		return 1;

	return 0;
}

void* stream_decoder(void *arg) {
	uint16_t start, stopp, streampointer_last = 0;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	if (!rfcfg->quiet)
		printf("DECODER run every %d ms, noise level %d µs, %s\n", rfcfg->decoder_delay, rfcfg->noise,
				rfcfg->collect_identical_codes ? "collect identical codes" : "process each code separately");

	while (1) {
		msleep(rfcfg->decoder_delay);

		// if we see ~8 pulses between 200-5000µs we assume receiving is in progress, so wait another 100ms
		while (1) {
			start = backward(streampointer, 8);

			int valid = 0;
			for (int i = 0; i < 8; i++) {
				uint8_t l = lstream[start];
				if (2 < l && l < 50)
					valid++;

				uint8_t h = hstream[start];
				if (2 < h && h < 50)
					valid++;
			}

			// start decoding
			if (valid < 10)
				break;

			if (rfcfg->verbose)
				printf("DECODER receiving\n");

			// wait and check again
			msleep(100);
		}

		// this is the block we have to analyze in this round
		start = streampointer_last;
		stopp = streampointer;

		if (rfcfg->verbose)
			printf("DECODER analyzing [%05u:%05u] %u samples\n", start, stopp, distance(start, stopp));

		while (1) {

			// not enough samples available -> next round
			int count = distance(start, stopp);
			if (count < 16)
				break;

			// find code start pattern
			while (++start != stopp)
				if (pattern(hstream, start))
					break;

			if (start == stopp)
				break; // nothing

			uint16_t cstart = start;
			printf("DECODER start pattern [%d, %d, %d, %d] at %05u\n", hstream[cstart], hstream[cstart + 1], hstream[cstart + 2], hstream[cstart + 3], cstart);

			// find code stopp pattern
			while (++start != stopp)
				if (!pattern(hstream, start))
					break;

			if (start == stopp)
				break; // nothing

			uint16_t cstopp = start + 3;
			printf("DECODER stopp pattern [%d, %d, %d, %d] at %05u\n", hstream[cstopp - 3], hstream[cstopp - 2], hstream[cstopp - 1], hstream[cstopp], cstopp);

			// validate code window
			int dist = distance(cstart, cstopp);
			if (dist < 16) {
				for (int i = 0; i < 5; i++)
					if (start != stopp)
						start++;
				if (rfcfg->verbose)
					printf("DECODER code start/stopp distance too small [%05d:%05d] %d, forward to %05d\n", cstart, cstopp, dist, start);
				continue;
			}

			// analyze this code and forward stream to find next code
			probe(cstart, cstopp);
			start = cstopp;
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

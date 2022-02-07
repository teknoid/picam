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

// print in green
static void green() {
	printf("\x1b[32m");
}

// reset colors
static void attroff() {
	printf("\x1b[m");
}

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

static void dump(uint8_t *stream, uint16_t start, uint16_t stopp, int mstart, int mstopp) {
	uint16_t p;

	// the opposite stream
	uint8_t *xstream = stream == lstream ? hstream : lstream;
	printf("DUMP %05u+%2u marker=%05u N=noise L=low H=high\n", start, distance(start, stopp), mstart > 0 ? mstart : mstopp);

	p = start;
	printf("%s ", stream == lstream ? "L" : "H");
	if (mstart == -1)
		green();
	while (p++ != stopp) {
		if (p == mstart)
			green();
		printf("%3d", stream[p]);
		if (p == mstopp)
			attroff();
	}
	if (mstopp == -1)
		attroff();
	printf("\n");

	p = start;
	printf("%s ", xstream == lstream ? "L" : "H");
	while (p++ != stopp) {
		printf("%3d", xstream[p]);
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
	// select the stream to sample on
	uint8_t *stream;
	if (rfcfg->sample_on_0)
		stream = lstream;
	else
		stream = hstream;

	if (rfcfg->verbose) {
		printf("\n");
		printf("DECODER probing [%05u:%05u] %u samples,", start, stopp, distance(start, stopp));
		printf(" start pattern [%d, %d, %d, %d],", stream[start], stream[start + 1], stream[start + 2], stream[start + 3]);
		printf(" stopp pattern [%d, %d, %d, %d]\n", stream[stopp - 3], stream[stopp - 2], stream[stopp - 1], stream[stopp]);
		dump(stream, start - 16, start + 32, start, -1);
		dump(stream, stopp - 32, stopp + 16, -1, stopp);
		printf("\n");
	}

	uint64_t l = probe_low(stopp - 64, 64, 15);
	uint64_t h = probe_high(stopp - 64, 64, 6);
	printf("probe_low 0x%016llx == %s probe_high 0x%016llx == %s\n", l, printbits64(l, SPACEMASK64), h, printbits64(h, SPACEMASK64));
}

// dumb 4-block symbol pattern matching
static int match4(uint8_t *stream, uint16_t start) {
	uint8_t p0 = stream[start], p1 = stream[start + 1], p2 = stream[start + 2], p3 = stream[start + 3];

	// contains zeros
	if (p0 == 0 || p1 == 0 || p2 == 0 || p3 == 0)
		return 0;

	// [3 3 3 3] full identical
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

	// TODO
	// [5, 10, 5, 15] 3 symbols
	// [19, 9, 19, 10] symbol tolerance

	return 0;
}

// determine symbols and count them
static int match8(uint8_t *stream, uint16_t start, int allowed) {
	// parameter entscheidet wieviel unterschiedliche symbole erlaubt sind
	return 0;
}

void* stream_decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	if (!rfcfg->quiet)
		printf("DECODER run every %d ms, noise level %d µs, %s\n", rfcfg->decoder_delay, rfcfg->noise,
				rfcfg->collect_identical_codes ? "collect identical codes" : "process each code separately");

	// select the stream to sample on
	uint8_t *stream, l, h;
	if (rfcfg->sample_on_0)
		stream = lstream;
	else
		stream = hstream;

	// block pointer and validation pointer
	uint16_t bptr = 0, vptr = 0;

	// decoder main loop: pause, then decode block
	while (1) {
		msleep(rfcfg->decoder_delay);

		// if we see ~8 pulses between 200-5000µs we assume receiving is in progress, so wait another 100ms
		while (1) {
			vptr = backward(streampointer, 8);

			int valid = 0;
			for (int i = 0; i < 8; i++) {
				l = lstream[vptr++];
				if (2 < l && l < 50)
					valid++;

				h = hstream[vptr++];
				if (2 < h && h < 50)
					valid++;
			}

			// if we see valid pulses - wait and check again
			if (valid > 10) {
				if (rfcfg->verbose)
					printf("DECODER receiving\n");
				msleep(100);
			} else {
				break; // start decoding
			}
		}

		// this is the block we have to analyze in this round
		uint16_t block = distance(bptr, streampointer);
		if (rfcfg->verbose)
			printf("DECODER analyzing [%05u:%05u] %u samples\n", bptr, streampointer, block);

		// block loop: do not overtake streampointer: start +4 and stopp +4
		while (block > 8) {

			// find code start pattern by 4-block jumps
			if (!match4(stream, bptr)) {
				bptr = forward(bptr, 4);
				block -= 4;
				continue;
			}

			// adjust left edge of code window
			uint16_t cstart = bptr;
			if (match4(stream, cstart - 1))
				cstart--;
			if (match4(stream, cstart - 1))
				cstart--;
			if (match4(stream, cstart - 1))
				cstart--;

			// find code stopp pattern
			while (block > 4 && match4(stream, bptr)) {
				bptr = forward(bptr, 4);
				block -= 4;
			}

			// adjust right edge of code window
			uint16_t cstopp = bptr;
			if (!match4(stream, cstopp - 1))
				cstopp--;
			if (!match4(stream, cstopp - 1))
				cstopp--;
			if (!match4(stream, cstopp - 1))
				cstopp--;

			// now this is the last matching position
			cstopp += 3;

			// validate and analyze code
			if (distance(cstart, cstopp) > 16)
				probe(cstart, cstopp); // right edge

			// this is the next position we have to match
			bptr = cstopp + 1;
		}
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

	// sampler main loop
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

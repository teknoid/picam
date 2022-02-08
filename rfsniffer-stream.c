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
static uint16_t back(uint16_t start, uint16_t positions) {
	for (int i = 0; i < positions; i++)
		start--;
	return start;
}

// forward stream to position
static uint16_t forw(uint16_t start, uint16_t positions) {
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

static void dump_stream(uint8_t *stream, uint16_t start, uint16_t stop, int mstart, int mstop) {
	uint16_t p;

	// the opposite stream
	uint8_t *xstream = stream == lstream ? hstream : lstream;
	printf("DUMP %05u+%2u marker=%05u N=noise L=low H=high\n", start, distance(start, stop), mstart > 0 ? mstart : mstop);

	p = start;
	printf("%s ", stream == lstream ? "L" : "H");
	if (mstart == -1)
		green();
	while (p++ != stop) {
		if (p == mstart)
			green();
		printf("%3d", stream[p]);
		if (p == mstop)
			attroff();
	}
	if (mstop == -1)
		attroff();
	printf("\n");

	p = start;
	printf("%s ", xstream == lstream ? "L" : "H");
	while (p++ != stop) {
		printf("%3d", xstream[p]);
	}
	printf("\n");
}

static void dump(uint16_t start, uint16_t stop) {
	uint8_t *stream;
	if (rfcfg->sample_on_0)
		stream = lstream;
	else
		stream = hstream;
	printf("DECODER probing [%05u:%05u] %u samples,", start, stop, distance(start, stop));
	printf(" start pattern [%d, %d, %d, %d],", stream[start], stream[start + 1], stream[start + 2], stream[start + 3]);
	printf(" stop  pattern [%d, %d, %d, %d]\n", stream[stop - 3], stream[stop - 2], stream[stop - 1], stream[stop]);
	dump_stream(stream, start - 16, start + 32, start, -1);
	dump_stream(stream, stop - 32, stop + 16, -1, stop);
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

static uint16_t probe(uint16_t start, uint16_t stop) {

	// this is crap
	if (distance(start, stop) < 16)
		return stop + 1;

	if (rfcfg->verbose)
		dump(start, stop);

	// TODO hier das volle programm:
	// - symbole sammeln
	// - fehlerhafte symbole korrigieren
	// - start/stop verschieben nach links/rechts
	// - sync pulse ermitteln
	// - code länge ermitteln
	// - probe_high + probe_low

	// [5, 10, 5, 15] 3 symbols

	// [19, 9, 19, 10] symbol tolerance

	uint64_t l = probe_low(stop - 64, 64, 15);
	uint64_t h = probe_high(stop - 64, 64, 6);
	if (rfcfg->verbose)
		printf("DECODER probe_low 0x%016llx == %s probe_high 0x%016llx == %s\n", l, printbits64(l, SPACEMASK64), h, printbits64(h, SPACEMASK64));

	return stop + 1;
}

// dumb 4-block symbol pattern matching
static int pattern(uint8_t *stream, uint16_t start) {
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

	return 0;

// alles in probe() --> brauchen wir dannn nicht mehr
//	// [6  5  7  6] square sum max +- 1
//	int q = (p0 + p1 + p2 + p3) / 4;
//	int qmin = q - 1, qmax = q + 1;
//	int tolerate = p0 == q || p0 == qmin || p0 == qmax;
//	tolerate &= p1 == q || p1 == qmin || p1 == qmax;
//	tolerate &= p2 == q || p2 == qmin || p2 == qmax;
//	tolerate &= p3 == q || p3 == qmin || p3 == qmax;
//	if (tolerate)
//		return 1;
//
}

// check the last received pulses - if between 200-5000µs we assume receiving is in progress
static int receiving() {
	uint16_t ptr = back(streampointer, 10);

	int valid = 0;
	for (int i = 0; i < 8; i++) {
		uint8_t l = lstream[ptr];
		uint8_t h = hstream[ptr];

		if (2 < l && l < 50)
			valid++;

		if (2 < h && h < 50)
			valid++;

		ptr++;
	}

	// not enough valid pulses - start decoding
	if (valid < 12)
		return 0;

	if (rfcfg->verbose)
		printf("DECODER receiving\n");

	return 1;
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
	uint8_t *stream;
	if (rfcfg->sample_on_0)
		stream = lstream;
	else
		stream = hstream;

	// decoder main loop: pause, then decode block
	uint16_t ptr = 0;
	while (1) {
		msleep(rfcfg->decoder_delay);

		// wait if receiving in progress
		while (receiving())
			msleep(100);

		// this is the block we have to analyze in this round
		uint16_t block = distance(ptr, streampointer);
		if (rfcfg->verbose)
			printf("DECODER analyzing [%05u:%05u] %u samples\n", ptr, streampointer, block);

		// pattern detector loop
		while (block > 8) {

			// catch pattern by jumping 4-block-wise
			if (!pattern(stream, ptr)) {
				ptr = forw(ptr, 4);
				block -= 4;
				continue;
			}

			// rewind till we find the first match
			uint16_t start = ptr;
			for (int i = 0; i < 6; i++)
				if (pattern(stream, start - 1))
					start--;

			// jump forward till no match
			while (block > 4 && pattern(stream, ptr)) {
				ptr = forw(ptr, 4);
				block -= 4;
			}

			// rewind till it again matches
			uint16_t stop = ptr;
			for (int i = 0; i < 4; i++)
				if (!pattern(stream, stop - 1))
					stop--;

			// deeper analyze, right edge of pattern window
			ptr = probe(start, stop + 3);
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

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
#define SYMBOLS				16

// ring buffers for low and high pulses
static uint8_t *stream, lstream[0xffff], hstream[0xffff];
static uint16_t streampointer;

// valid symbol table and count
uint16_t lcounter[SYMBOLS], hcounter[SYMBOLS];
uint8_t lsymbols[SYMBOLS], hsymbols[SYMBOLS];

extern rfsniffer_config_t *rfcfg;

// print in green
static void green() {
	printf("\x1b[32m");
}

// reset colors
static void attroff() {
	printf("\x1b[m");
}

// clear the line
static void clearline() {
	printf("\x1b[1F\x1b[K");
}

// select the LOW stream
static void select_lstream() {
	stream = lstream;
}

// select the HIGH stream
static void select_hstream() {
	stream = hstream;
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

// clears both streams between the two positions
static void clear_stream(uint16_t start, uint16_t stop) {
	uint16_t p = start;
	while (p++ != stop) {
		lstream[p] = 0;
		hstream[p] = 0;
	}
}

static void dump_stream(uint16_t start, uint16_t stop, int mstart, int mstop) {
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
	printf("DECODER dump [%05u:%05u] %u samples,", start, stop, distance(start, stop));
	printf(" start pattern [%d, %d, %d, %d],", stream[start], stream[start + 1], stream[start + 2], stream[start + 3]);
	printf(" stop  pattern [%d, %d, %d, %d]\n", stream[stop - 3], stream[stop - 2], stream[stop - 1], stream[stop]);
	dump_stream(start - 16, start + 32, start, -1);
	dump_stream(stop - 32, stop + 16, -1, stop);
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

// returns the symbol matching given distance, otherwise return -1
static int symbol_match_dist(uint8_t s, int d) {
	uint8_t *symbols;
	if (stream == lstream)
		symbols = lsymbols;
	else
		symbols = hsymbols;

	for (int i = 0; i < SYMBOLS; i++) {
		int m = symbols[i];
		if (!m)
			break; // table end
		if (m > d && ((m - d) < s && s < (m + d)))
			return m;
	}
	return -1; // not matching symbol for distance found
}

// returns 1 if symbol ..., otherwise 0
static int symbol_match_edge(uint16_t pos) {
	uint8_t s, l = lstream[pos], h = hstream[pos];
	int lmatch = 0, hmatch = 0;

	for (int i = 0; i < SYMBOLS; i++) {
		s = lsymbols[i];
		if (!s)
			break; // table end
		lmatch |= l == s || l == (s - 1) || l == (s + 1);
	}

	for (int i = 0; i < SYMBOLS; i++) {
		s = hsymbols[i];
		if (!s)
			break; // table end
		hmatch |= h == s || h == (s - 1) || h == (s + 1);
	}

	return lmatch && hmatch;
}

// returns the symbol matching distance max +- 1, otherwise return -1
static int symbol_match(uint8_t s) {
	uint8_t *symbols;
	if (stream == lstream)
		symbols = lsymbols;
	else
		symbols = hsymbols;

	for (int i = 0; i < SYMBOLS; i++) {
		uint8_t m = symbols[i];
		if (!m)
			break; // table end
		if (s == m)
			return m;
		if (s == (m - 1))
			return m;
		if (s == (m + 1))
			return m;
	}
	return -1;
}

// returns 1 if symbol is exact valid, otherwise 0
static int symbol_valid(uint8_t s) {
	uint8_t *symbols;

	if (stream == lstream)
		symbols = lsymbols;
	else
		symbols = hsymbols;

	for (int i = 0; i < SYMBOLS; i++) {
		uint8_t v = symbols[i];
		if (!v)
			break; // table end
		if (s == v)
			return 1;
	}
	return 0;
}

// returns the occurrence count of a symbol
static int symbol_count(uint8_t s) {
	uint16_t *counter;
	uint8_t *symbols;
	if (stream == lstream) {
		symbols = lsymbols;
		counter = lcounter;
	} else {
		symbols = hsymbols;
		counter = hcounter;
	}

	for (int i = 0; i < SYMBOLS; i++)
		if (s == symbols[i])
			return counter[i];

	return 0;
}

// remove symbols below minimum occurrence
static void symbols_clean(int min) {
	if (rfcfg->verbose)
		printf("DECODER clean symbols ");

	// merge both symbol tables and counters
	uint16_t counter[SYMBOLS];
	uint8_t symbols[SYMBOLS];
	memset(symbols, 0, SYMBOLS * sizeof(uint8_t));
	memset(counter, 0, SYMBOLS * sizeof(uint16_t));
	for (int i = 0; i < SYMBOLS; i++) {
		if (lsymbols[i] && lcounter[i] < min) {
			if (rfcfg->verbose)
				printf("L%d ", lsymbols[i]);
			lsymbols[i] = 0;
			lcounter[i] = 0;
		}
		if (hsymbols[i] && hcounter[i] < min) {
			if (rfcfg->verbose)
				printf("H%d ", hsymbols[i]);
			hsymbols[i] = 0;
			hcounter[i] = 0;
		}
	}
	if (rfcfg->verbose) {
		printf("\nDECODER symbol table after clean   L = ");
		for (int i = 0; i < SYMBOLS; i++)
			if (lsymbols[i])
				printf("%d:%d ", lsymbols[i], lcounter[i]);
		printf("   H = ");
		for (int i = 0; i < SYMBOLS; i++)
			if (hsymbols[i])
				printf("%d:%d ", hsymbols[i], hcounter[i]);
		printf("\n");
	}
}

// collect valid symbols
static void symbols_collect(uint16_t start, uint16_t stop) {
	uint16_t *counter;
	uint8_t *symbols;
	if (stream == lstream) {
		symbols = lsymbols;
		counter = lcounter;
	} else {
		symbols = hsymbols;
		counter = hcounter;
	}

	// clear symbols + count
	memset(symbols, 0, SYMBOLS * sizeof(uint8_t));
	memset(counter, 0, SYMBOLS * sizeof(uint16_t));

	// pulse counters: index equals pulse length x 100, e.g. 1=100µs, 2=200µs ... 255=25500µs
	uint16_t c[BUFFER];
	memset(c, 0, BUFFER * sizeof(uint16_t));
	uint16_t p = start;
	while (p != stop) {
		uint8_t s = stream[p];
		c[s]++;
		p++;
	}

	// collect the symbols in order of frequency
	for (int s = 0; s < SYMBOLS; s++) {
		uint16_t cmax = 0;
		uint8_t pmax = 0;
		for (int i = 0; i < BUFFER; i++) {
			if (c[i] > cmax) {
				cmax = c[i];
				pmax = i;
			}
		}
		counter[s] = cmax;
		symbols[s] = pmax;
		c[pmax] = 0; // not detect again
	}

	if (rfcfg->verbose) {
		printf("DECODER %s ", stream == lstream ? "L" : "H");
		for (int i = 0; i < SYMBOLS; i++)
			if (symbols[i])
				printf(" %d:%d", symbols[i], counter[i]);
		printf("\n");
	}
}

// sampling error correction
static void error_correction(uint16_t start, uint16_t stop) {
	uint16_t p;

	// do initial symbol determination
	symbols_collect(start, stop);

	// soft ironing symbols with a distance of +- 1 over max 3 rounds
	for (int r = 0; r < 3; r++) {
		if (rfcfg->verbose)
			printf("DECODER %s soft correction #%d: ", stream == lstream ? "L" : "H", r + 1);

		p = start;
		int fixed = 0;
		while (p++ != stop) {
			uint8_t s = stream[p];
			int m = symbol_match(s);
			if (m > 0 && m != s) {
				if (rfcfg->verbose)
					printf("%d->%d ", s, m);
				stream[p] = m;
				fixed++;
			}
		}

		if (rfcfg->verbose) {
			printf("\n");
			if (!fixed)
				clearline();
		}

		if (!fixed)
			break;

		// do symbol determination again
		symbols_collect(start, stop);
	}

	// special error handling:
	// correct the symbol if it occurs very rare (<5%) and the distance to a valid symbol is max 5
	uint16_t rare = distance(start, stop) / 20;

	if (rfcfg->verbose)
		printf("DECODER %s hard correction ", stream == lstream ? "L" : "H");

	p = start;
	int fixed = 0;
	int unfixed = 0;
	while (p++ != stop) {
		uint8_t s = stream[p];
		if (symbol_count(s) < rare) {
			int d = symbol_match_dist(s, 5);
			if (d > 0 && d != s) {
				if (rfcfg->verbose)
					printf(" %d->%d ", s, d);
				stream[p] = d;
				fixed++;
			} else {
				if (rfcfg->verbose)
					printf(" %d->X ", s);
				unfixed++;
			}
		}
	}

	if (rfcfg->verbose) {
		printf("\n");
		if (fixed || unfixed)
			printf("DECODER %s %d fixed symbols, %d unfixed symbols\n", stream == lstream ? "L" : "H", fixed, unfixed);
		else
			clearline();
	}

	// do final symbol determination
	symbols_collect(start, stop);
}

// returns the count of non matching symbols in a block (having a distance > requested)
static int errors_in_block(uint16_t start, uint16_t stop, int d) {
	uint16_t p = start;
	int errors = 0;
	while (p++ != stop) {
		uint8_t s = stream[p];
		if (s == 0)
			return 99; // definitive an error
		if (symbol_match_dist(s, d) < 0)
			errors++;
	}
	return errors;
}

static uint16_t probe(uint16_t start, uint16_t stop) {
	uint16_t estart, estop, dist, offset, reserve;

	// this is crap
	dist = distance(start, stop);
	if (dist < 16)
		return stop + 1;

	if (rfcfg->verbose)
		printf("DECODER probing [%05u:%05u] %u samples\n", start, stop, dist);

	// do initial error correction
	select_lstream();
	error_correction(start, stop);
	select_hstream();
	error_correction(start, stop);

	// right code window expansion, here signal is very reliable
	offset = 8;
	estop = stop;
	reserve = distance(stop, streampointer);
	select_lstream();
	while (reserve > 8) {
		int errors = errors_in_block(start + offset, stop + offset, 2);
		if (rfcfg->verbose)
			printf("DECODER right expand [%05u:%05u], errors=%d, reserve to streampointer[%05u]=%d\n", start, stop + offset, errors, streampointer, reserve);
		if (errors > 2)
			break;
		reserve = distance(stop + offset, streampointer);
		estop = stop + offset;
		offset += 8;
	}

	dist = distance(start, estop);
	if (rfcfg->verbose)
		printf("DECODER after right expanding   [%05u:%05u] %u samples\n", start, estop, dist);

	// this is crap
	if (dist < 16)
		return stop + 1;

	// do error correction again
	select_lstream();
	error_correction(start, estop);
	select_hstream();
	error_correction(start, estop);

	// now delete all codes with occurrence < 1%, then we have a good symbol table
	symbols_clean(dist > 100 ? dist / 100 : 1);

	// fine tune right edge of code window - be very restrictive
	estop -= 8;
	while (symbol_match_edge(estop + 1))
		estop++;

	dist = distance(start, estop);
	if (rfcfg->verbose)
		printf("DECODER after right edge tuning [%05u:%05u] %u samples\n", start, estop, dist);

	// left code window expansion - difficult because the signal slowly goes into noise on this side
	offset = 8;
	estart = start;
	select_lstream();
	while (1) {
		int errors = errors_in_block(start - offset, stop - offset, 4);
		if (rfcfg->verbose)
			printf("DECODER  left expand [%05u:%05u], errors=%d\n", start - offset, stop, errors);
		if (errors > 4)
			break;
		estart = start - offset;
		offset += 8;
	}

	dist = distance(estart, estop);
	if (rfcfg->verbose)
		printf("DECODER after left expanding   [%05u:%05u] %u samples\n", estart, estop, dist);

	// fine tune left edge of code window - be very tolerant
	estart += 8;
	while (1) {
		select_lstream();
		int lmatch = symbol_match(stream[estart - 1]);
		select_hstream();
		int hmatch = symbol_match(stream[estart - 1]);
		if (lmatch < 0 && hmatch < 0)
			break; // first mismatch on both streams
		estart--;
	}

	dist = distance(estart, estop);
	if (rfcfg->verbose)
		printf("DECODER after left edge tuning [%05u:%05u] %u samples\n", estart, estop, dist);

	// this is crap
	if (dist < 16)
		return stop + 1;

//	// do final error correction
//	select_lstream();
//	error_correction(estart, estop);
//	select_hstream();
//	error_correction(estart, estop);

	select_lstream();
	if (rfcfg->verbose)
		dump(estart, estop);

	uint64_t lcode = probe_low(stop - 64, 64, 15);
	uint64_t hcode = probe_high(stop - 64, 64, 6);
	if (rfcfg->verbose)
		printf("DECODER probe_low 0x%016llx == %s probe_high 0x%016llx == %s\n", lcode, printbits64(lcode, SPACEMASK64), hcode, printbits64(hcode, SPACEMASK64));

	// avoid re-catching this code
	clear_stream(estart, estop);
	return estop + 1;
}

// dumb 4-block symbol pattern matching
static int pattern(uint16_t start) {
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
	if (rfcfg->sample_on_0)
		select_lstream();
	else
		select_hstream();

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
			if (!pattern(ptr)) {
				ptr = forw(ptr, 4);
				block -= 4;
				continue;
			}

			// rewind till we find the first match
			uint16_t start = ptr;
			for (int i = 0; i < 6; i++)
				if (pattern(start - 1))
					start--;

			// jump forward till no match
			while (block > 4 && pattern(ptr)) {
				ptr = forw(ptr, 4);
				block -= 4;
			}

			// rewind till it again matches
			uint16_t stop = ptr;
			for (int i = 0; i < 4; i++)
				if (!pattern(stop - 1))
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

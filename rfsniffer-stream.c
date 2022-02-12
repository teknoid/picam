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

#define BLOCKSIZE			64
#define SYMBOLS				16

static const char L = 'L';
static const char H = 'H';

// ring buffers for low and high pulses
static uint8_t *stream, lstream[UINT16_MAX], hstream[UINT16_MAX];
static uint16_t streampointer;

static int receiving_counter = 0;

// tables for symbol and count
uint16_t *counter, lcounter[SYMBOLS], hcounter[SYMBOLS], mcounter[SYMBOLS];
uint8_t *symbols, lsymbols[SYMBOLS], hsymbols[SYMBOLS], msymbols[SYMBOLS];

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
	counter = lcounter;
	symbols = lsymbols;
}

// select the HIGH stream
static void select_hstream() {
	stream = hstream;
	counter = hcounter;
	symbols = hsymbols;
}

// select the counter table
static uint16_t* select_counter(uint8_t *xsymbols) {
	if (xsymbols == lsymbols)
		return lcounter;
	if (xsymbols == hsymbols)
		return hcounter;
	return NULL;
}

// select the symbols table
static uint8_t* select_symbols(uint8_t *xstream) {
	if (xstream == lstream)
		return lsymbols;
	if (xstream == hstream)
		return hsymbols;
	return NULL;
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

// calculate the distance between two stream positions
static uint16_t distance(uint16_t start, uint16_t stop) {
	if (stop < start)
		return (UINT16_MAX - start) + stop;
	else
		return stop - start;
}

// clears both streams between the positions
static void clear_streams(uint16_t start, uint16_t stop) {
	uint16_t p = start - 1;
	while (p++ != stop) {
		lstream[p] = 0;
		hstream[p] = 0;
	}
}

static void dump_stream(uint16_t start, uint16_t stop, int mstart, int mstop) {
	uint16_t p = start - 1;

	// the opposite stream
	uint8_t *xstream = stream == lstream ? hstream : lstream;

	printf("DUMP %05u+%2u marker=%05u N=noise L=low H=high\n", start, distance(start, stop), mstart > 0 ? mstart : mstop);

	printf("%c ", stream == lstream ? L : H);
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

	p = start - 1;
	printf("%c ", xstream == lstream ? L : H);
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

// calculates the size of symbol table
static uint8_t size_symbols(uint8_t *xsymbols) {
	int i = 0;
	while (i < SYMBOLS && xsymbols[i])
		i++;
	return i;
}

static void clear_tables() {
	memset(lsymbols, 0, SYMBOLS * sizeof(uint8_t));
	memset(lcounter, 0, SYMBOLS * sizeof(uint16_t));

	memset(hsymbols, 0, SYMBOLS * sizeof(uint8_t));
	memset(hcounter, 0, SYMBOLS * sizeof(uint16_t));
}

// prints the symbol tables
static void dump_tables(const char *message) {
	printf("DECODER %s L(%d) = ", message, size_symbols(lsymbols));
	for (int i = 0; i < SYMBOLS; i++)
		if (lsymbols[i])
			printf("%d:%d ", lsymbols[i], lcounter[i]);
	printf("   H(%d) = ", size_symbols(hsymbols));
	for (int i = 0; i < SYMBOLS; i++)
		if (hsymbols[i])
			printf("%d:%d ", hsymbols[i], hcounter[i]);
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

// determine minimum symbol distance
static uint8_t symbol_distance(uint8_t *xsymbols) {
	uint8_t m, min = 0xff;
	for (int x = 0; x < SYMBOLS; x++) {
		if (!xsymbols[x])
			break;
		for (int y = x + 1; y < SYMBOLS; y++) {
			if (!xsymbols[y])
				break;
			m = xsymbols[x] < xsymbols[y] ? (xsymbols[y] - xsymbols[x]) : (xsymbols[x] - xsymbols[y]);
			if (m < min)
				min = m;
		}
	}
	return min == 0xff ? xsymbols[0] : min; // only one symbol?
}

// compare symbol against table, if valid in given distance return the matching symbol else 0
static int match(uint8_t *xsymbols, uint8_t s, uint8_t d) {
	for (int i = 0; i < SYMBOLS; i++) {
		uint8_t t = xsymbols[i];
		if (!t)
			break; // table end
		if (s == t)
			return t;
		if (t < d)
			continue; // reached min distance
		if ((t - d) <= s && s <= (t + d))
			return t;
	}
	return 0;
}

// same as match() but additionally increments hit counter
static int valid(uint8_t *xsymbols, uint8_t s, uint8_t d) {
	uint16_t *xcounter = select_counter(xsymbols);

	for (int i = 0; i < SYMBOLS; i++) {
		uint8_t t = xsymbols[i];
		if (!t)
			break; // table end
		if (s == t) {
			xcounter[i]++;
			return s;
		}
		if (t < d)
			continue;
		if ((t - d) < s && s < (t + d)) {
			xcounter[i]++;
			return t;
		}
	}
	return 0;
}

static void learn(uint8_t *xsymbols, uint8_t l) {
	uint16_t *xcounter = select_counter(xsymbols);

	// check first 2 entries: symbols only allowed to grow, not shrink, allow max -1
	if (xsymbols[0] && (l < xsymbols[0] - 1) && xsymbols[1] && (l < xsymbols[1] - 1))
		return;

	// find next free entry
	int i = 0;
	while (i < SYMBOLS && xsymbols[i])
		i++;
	if (i == SYMBOLS)
		return; // table full

	xsymbols[i] = l;
	xcounter[i] = 1;
}

// move all symbols to the left without gaps between
static void shrink(uint8_t *xsymbols) {
	uint16_t *xcounter = select_counter(xsymbols);

	int repeat;
	do {
		repeat = 0;
		for (int i = 0; i < (SYMBOLS - 1); i++)
			if (!xsymbols[i] && xsymbols[i + 1]) {
				xsymbols[i] = xsymbols[i + 1];
				xsymbols[i + 1] = 0;
				xcounter[i] = xcounter[i + 1];
				xcounter[i + 1] = 0;
				repeat = 1;
			}
	} while (repeat);
}

// sort by highest occurrence
static void sort(uint8_t *xsymbols) {
	uint16_t c, *xcounter = select_counter(xsymbols);
	uint8_t s;

	int repeat;
	do {
		repeat = 0;
		for (int i = 0; i < (SYMBOLS - 1); i++)
			if (xcounter[i + 1] > xcounter[i]) {
				c = xcounter[i];
				s = xsymbols[i];
				xcounter[i] = xcounter[i + 1];
				xsymbols[i] = xsymbols[i + 1];
				xcounter[i + 1] = c;
				xsymbols[i + 1] = s;
				repeat = 1;
			}
	} while (repeat);
}

// remove symbols below minimum occurrence
static void filter(uint8_t min) {

	if (rfcfg->verbose)
		dump_tables("symbol tables ");

	// sort
	sort(lsymbols);
	sort(hsymbols);

	// clean
	for (int i = 0; i < SYMBOLS; i++) {
		if (lcounter[i] <= min)
			lsymbols[i] = 0;
		if (hcounter[i] <= min)
			hsymbols[i] = 0;
	}

	// shrink
	shrink(lsymbols);
	shrink(hsymbols);

	if (rfcfg->verbose)
		dump_tables("after filter  ");
}

// determine valid symbols and merge adjacent together
static void emphase(uint8_t *xsymbols) {
	uint16_t *xcounter = select_counter(xsymbols);

	if (rfcfg->verbose)
		printf("DECODER %c emphasing ", xsymbols == lsymbols ? L : H);

	// find minimum symbol distance
	uint8_t d, dmin = symbol_distance(xsymbols);
	uint8_t s = size_symbols(xsymbols);

	// minimum symbol distance is greater/equal table size --> we would loose information, so stop merging
	while (dmin < s) {

		if (rfcfg->verbose)
			printf("   d=%d s=%d ", dmin, s);

		// do the merge for current distance
		for (int x = 0; x < SYMBOLS; x++) {
			if (!xsymbols[x])
				break;
			for (int y = x + 1; y < SYMBOLS; y++) {
				if (!xsymbols[y])
					break;
				d = xsymbols[x] < xsymbols[y] ? (xsymbols[y] - xsymbols[x]) : (xsymbols[x] - xsymbols[y]);
				if (d <= dmin) {
					printf("%d<-%d ", xsymbols[x], xsymbols[y]);
					xcounter[x] += xcounter[y];
					xcounter[y] = 0;
					xsymbols[y] = 0;
				}
				shrink(xsymbols);
			}
		}

		// recalculate size and min symbol distance
		dmin = symbol_distance(xsymbols);
		s = size_symbols(xsymbols);
	}

	if (rfcfg->verbose)
		printf("\n");
}

// find the smallest symbol in given window
static uint16_t smallest(uint16_t start, uint16_t stop) {
	uint16_t pmin = start, p = start;
	uint8_t l, h, min = 0xff;

	while (p++ != stop) {
		l = lstream[p];
		h = hstream[p];
		if (l && h && (l + h) < min) {
			min = l + h;
			pmin = p;
		}
	}

	return pmin;
}

// ironing stream up to minimum symbol distance
static void iron(uint8_t *xstream, uint16_t start, uint16_t stop) {
	uint16_t p;
	uint8_t s, m, *xsymbols = select_symbols(xstream);
	uint8_t dmin = symbol_distance(xsymbols);

	for (int i = 0; i < dmin; i++) {

		if (rfcfg->verbose)
			printf("DECODER %s ironing d=%d ", xstream == lstream ? "L" : "H", i);

		p = start - 1;
		int fixed = 0;
		while (p++ != stop) {

			s = xstream[p];
			if (!s)
				continue;

			m = match(xsymbols, s, i);
			if (!m)
				continue;
			if (m != s) {
				if (rfcfg->verbose)
					printf("%d->%d ", s, m);
				xstream[p] = m;
				fixed++;
			}
		}

		if (rfcfg->verbose) {
			printf("\n");
			if (!fixed)
				clearline();
		}
	}
}

static uint16_t tune(uint16_t pos, const char direction) {
	uint16_t e = 0;
	uint8_t l, h, lv, hv, dist = 5;

	l = lstream[pos];
	h = hstream[pos];

	if (!l && !h)
		return 999; // dead stream

	lv = valid(lsymbols, l, dist);
	hv = valid(hsymbols, h, dist);

	if (!l || !h)
		e = 10; // TODO heavy sampling error but we must tolerate

	if (lv)
		e += lv > l ? (lv - l) : (l - lv);
	if (!lv)
		e += l;

	if (hv)
		e += hv > h ? (hv - h) : (h - hv);
	if (!hv)
		e += h;

	if (rfcfg->verbose) {
		printf("DECODER tune ");
		if (direction == 'l')
			printf("◄%05u  ", pos);
		if (direction == 'r')
			printf(" %05u► ", pos);
		printf("   L %3d(%2d)   H %3d(%2d)   E %d\n", l, lv, h, hv, e);
	}

	return e;
}

static uint16_t probe_right(uint16_t start, uint16_t stop) {
	uint16_t p = start - 1;
	uint8_t l, lv, h, hv;

	int e = 0;
	while (p++ != streampointer - 1) {
		l = lstream[p];
		h = hstream[p];

		if (!l && !h)
			break; // dead stream

		lv = valid(lsymbols, l, 1);
		hv = valid(hsymbols, h, 1);

		if (lv)
			e -= l;
		else {
			learn(lsymbols, l);
			e += l;
		}

		if (hv)
			e -= h;
		else {
			learn(hsymbols, h);
			e += h;
		}

		if (e < 0)
			e = 0;

		printf("DECODER probe  %05d►  %3d(%2d) L   %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));

		// too much errors - jump out
		if (e > 100) // TODO how calculate ?
			break;
	}
	return p;
}

static uint16_t probe_left(uint16_t start) {
	uint16_t p = start + 1;
	uint8_t l, lv, h, hv;

	int e = 0;
	while (p-- != streampointer + 1) {
		l = lstream[p];
		h = hstream[p];

		if (!l && !h)
			break; // dead stream

		lv = valid(lsymbols, l, 1);
		hv = valid(hsymbols, h, 1);

		if (lv)
			e -= l;
		else {
			learn(lsymbols, l);
			e += l;
		}

		if (hv)
			e -= h;
		else {
			learn(hsymbols, h);
			e += h;
		}

		if (e < 0)
			e = 0;

		printf("DECODER probe ◄%05d   %3d(%2d) L   %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));

		// too much errors - jump out
		if (e > 50) // TODO how calculate ?
			break;
	}
	return p;
}

// consume the signal
static void eat(uint16_t start, uint16_t stop) {
	select_hstream();
	if (rfcfg->verbose)
		dump(start, stop);

	uint64_t lcode = probe_low(stop - 64, 64, 15);
	uint64_t hcode = probe_high(stop - 64, 64, 6);
	if (rfcfg->verbose)
		printf("DECODER probe_low 0x%016llx == %s probe_high 0x%016llx == %s\n", lcode, printbits64(lcode, SPACEMASK64), hcode, printbits64(hcode, SPACEMASK64));
}

// find begin and end of a signal by analyzing symbols
static uint16_t probe(uint16_t start, uint16_t stop) {
	uint16_t p, estart, estop, dist;

	// this is crap
	dist = distance(start, stop);
	if (dist < 16)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER probing [%05u:%05u] %u samples\n", start, stop, dist);

	if (rfcfg->verbose)
		printf("DECODER L=low stream, H=high stream, symbol(valid), E=error counter, D=distance to stream head\n");

	// find the smallest symbol - here we start symbol learning
	// TODO eigentlich das am häufigsten auftretende (?)
	clear_tables();
	p = smallest(start, stop);

	if (rfcfg->verbose)
		printf("DECODER smallest symbol L%d H%d at %05u \n", lstream[p], hstream[p], p);

	// expand window to right, then left - right is more reliable due to signal goes into noise on left side
	estop = probe_right(p, stop);
	estart = probe_left(p);

	// this is crap
	dist = distance(estart, estop);
	if (dist < 16)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER probe window [%05u:%05u] %u samples\n", estart, estop, dist);

	// remove all symbols with occurrence below 1%
	filter(dist > 100 ? dist / 100 : 1);

	// merge similar symbols together
	emphase(lsymbols);
	emphase(hsymbols);
	if (rfcfg->verbose)
		dump_tables("after emphase ");

	// fine tune right edge of window
	estop -= 16;
	while (tune(estop, 'r') < 10) // TODO größtes symbol
		estop++;

	// fine tune left edge of window
	estart = estart + 16;
	while (tune(estart, 'l') < 30) // TODO größtes symbol * 3
		estart--;

	// this is again crap
	dist = distance(estart, estop);
	if (dist < 16)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER tuned window [%05u:%05u] %u samples\n", estart, estop, dist);

	// symbol soft error correction
	iron(lstream, estart, estop);
	iron(hstream, estart, estop);

	// hammer()
	// die übrig gebliebenen fehler versuchen zu korrigieren:
	// wenn in H nur ein einziges vorkommt dann das nehmen, wenn mehrere das mit der größten häufigkeit

	// consume this stream window
	eat(estart, estop);

	// avoid re-catching this code
	clear_streams(estart, estop);

	return estop;
}

// dumb 4-block symbol pattern matching
static int sniff(uint16_t start) {
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
	if (valid < 12) {
		if (rfcfg->verbose && receiving_counter)
			printf("\n");
		receiving_counter = 0;
		return 0;
	}

	if (rfcfg->verbose) {
		if (!receiving_counter++)
			printf("DECODER receiving .");
		else
			printf(".");
	}

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

		// pattern sniffer loop
		while (block > 8) {

			// catch pattern by jumping 4-block-wise
			if (!sniff(ptr)) {
				ptr = forw(ptr, 4);
				block -= 4;
				continue;
			}

			uint16_t start = ptr;

			// jump forward till no match
			while (sniff(ptr + 4) && block > 4) {
				ptr = forw(ptr, 4);
				block -= 4;
			}

			uint16_t stop = ptr;

			// detailed symbol analyzing
			ptr = probe(start, stop);

			block = distance(++ptr, streampointer);
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

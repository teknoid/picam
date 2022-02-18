#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "gpio.h"
#include "rfsniffer.h"
#include "utils.h"

#define SYMBOLS				32

static const char L = 'L';
static const char H = 'H';
static const char R = 'R';

// ring buffers for low and high pulses, 16bit = original samples, 8bit = compressed to symbols
static uint16_t lsamples[UINT16_MAX], hsamples[UINT16_MAX], streampointer;
static uint8_t lstream[UINT16_MAX], hstream[UINT16_MAX], *stream;

// tables for symbol and count
uint16_t *counter, lcounter[SYMBOLS], hcounter[SYMBOLS];
uint8_t *symbols, lsymbols[SYMBOLS], hsymbols[SYMBOLS];

static int receiving_counter = 0;

extern rfsniffer_config_t *rfcfg;

// print in green
static void red() {
	printf("\x1b[31m");
}

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
		return (UINT16_MAX - start) + stop + 1;
	else
		return stop - start + 1;
}

// clears both streams between the positions
static void clear_streams(uint16_t start, uint16_t stop) {
	uint16_t p = start - 1;
	while (p++ != stop) {
		lstream[p] = 0;
		hstream[p] = 0;
	}
}

static void dump_stream(uint8_t *xstream, uint16_t start, uint16_t stopp, int overhead) {
	uint16_t p;
	int cols = 80;

#ifdef TIOCGSIZE
    struct ttysize ts;
    ioctl(STDIN_FILENO, TIOCGSIZE, &ts);
    cols = ts.ts_cols;
#elif defined(TIOCGWINSZ)
	struct winsize ts;
	ioctl(STDIN_FILENO, TIOCGWINSZ, &ts);
	cols = ts.ws_col;
#endif

	int xstart = start - overhead;
	int xstopp = stopp + overhead;
	int places = cols / 3 - 5;

	if (distance(xstart, xstopp) > places) {

		// screen to small - skip stream in the middle
		int skip1 = xstart + places / 2;
		int skip2 = xstopp - places / 2;

		p = xstart - 1;
		printf("%c ", xstream == lstream ? L : H);
		while (p++ != skip1) {
			if (p == start)
				green();
			printf("%3d", xstream[p]);
		}

		printf("  ...");

		p = skip2 - 1;
		while (p++ != xstopp) {
			printf("%3d", xstream[p]);
			if (p == stopp)
				attroff();
		}
		printf("\n");

	} else {

		// all fit to screen
		p = xstart - 1;
		printf("%c ", xstream == lstream ? L : H);
		while (p++ != xstopp) {
			if (p == start)
				green();
			printf("%3d", xstream[p]);
			if (p == stopp)
				attroff();
		}
		printf("\n");
	}
}

static void dump(uint16_t start, uint16_t stop, int overhead) {
	printf("DECODER dump [%05u:%05u] %u samples,", start, stop, distance(start, stop));
	printf(" start pattern [%d, %d, %d, %d],", stream[start], stream[start + 1], stream[start + 2], stream[start + 3]);
	printf(" stop  pattern [%d, %d, %d, %d]\n", stream[stop - 3], stream[stop - 2], stream[stop - 1], stream[stop]);
	dump_stream(hstream, start, stop, overhead);
	dump_stream(lstream, start, stop, overhead);
}

// calculates the size of symbol table
static uint8_t size_symbols(uint8_t *xsymbols) {
	int i = 0;
	while (i < SYMBOLS && xsymbols[i])
		i++;
	return i;
}

// finds the smallest symbol in table
static uint8_t smallest_symbol(uint8_t *xsymbols) {
	uint8_t min = xsymbols[0];
	for (int i = 0; i < SYMBOLS; i++)
		if (xsymbols[i] && xsymbols[i] < min)
			min = xsymbols[i];
	return min;
}

// finds the biggest symbol in table
static uint8_t biggest_symbol(uint8_t *xsymbols) {
	uint8_t max = xsymbols[0];
	for (int i = 0; i < SYMBOLS; i++)
		if (xsymbols[i] && xsymbols[i] > max)
			max = xsymbols[i];
	return max;
}

// find the smallest sample in given window and return it's position
static uint16_t smallest_sample(uint8_t *xstream, uint16_t start, uint16_t stop) {
	uint16_t pmin = start - 1, p = start - 1;
	uint8_t s, min = UINT8_MAX;
	while (p++ != stop) {
		s = xstream[p];
		if (s && (s < min)) {
			min = s;
			pmin = p;
		}
	}
	return pmin;
}

// find the biggest sample in given window and return it's position
static uint16_t biggest_sample(uint8_t *xstream, uint16_t start, uint16_t stop) {
	uint16_t pmax = start - 1, p = start - 1;
	uint8_t s, max = 0;
	while (p++ != stop) {
		s = xstream[p];
		if (s && (s > max)) {
			max = s;
			pmax = p;
		}
	}
	return pmax;
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
static uint64_t decode_low(uint16_t pos, int bits) {
	uint64_t code = 0;
	uint8_t l, lmin = lsymbols[0] < lsymbols[1] ? lsymbols[0] : lsymbols[1];

	for (int i = 0; i < bits; i++) {
		code <<= 1;
		l = lstream[pos++];
		if (l != UINT8_MAX && l > lmin)
			code++;
	}
	return code;
}

//
// short high pulse followed by long low pulse or long high + short low pulse, no clock
//     _           ___
//   _| |___     _|   |_
//      0             1
static uint64_t decode_high(uint16_t pos, uint8_t bits, uint16_t divider) {
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

static uint16_t find(uint8_t *xstream, uint16_t start, uint16_t stop, uint8_t s) {
	start--;
	while (start++ != stop)
		if (xstream[start] == s)
			return start;
	return stop;
}

// find the SYNC symbol - min 3 repeats with identical distances > 8
static void find_sync(uint8_t *xstream, uint16_t start, uint16_t stop, uint8_t *sync, uint8_t *dist) {
	uint16_t p, positions[SYMBOLS];
	uint8_t s, *xsymbols = select_symbols(xstream);

	memset(positions, 0, SYMBOLS * sizeof(uint16_t));
	for (int i = 0; i < SYMBOLS; i++) {
		s = xsymbols[i];
		if (!s)
			break;

		int j = 0;
		p = start;
		do {
			p = find(xstream, p, stop, s);
			positions[j++] = p;
		} while (p++ != stop && j < SYMBOLS);

		int d0 = distance(positions[0], positions[1]);
		int d1 = distance(positions[1], positions[2]);
		int d2 = distance(positions[2], positions[3]);

		if (d0 > 8 && d0 == d1 && d1 == d2) {
			*sync = s;
			*dist = d0 - 2;
			return;
		}
	}
}

// consume the signal
static void eat(uint16_t start, uint16_t stop) {
	uint64_t code;
	uint16_t p = 0, count = 0;
	uint8_t sync = 0, dist = 0;

	if (rfcfg->verbose)
		dump(start, stop, 16);

	find_sync(lstream, start, stop, &sync, &dist);
	if (!sync)
		return;

	if (rfcfg->verbose)
		printf("DECODER found SYNC L%2d code length %2d bits\n", sync, dist);

	p = start;
	do {
		p = find(lstream, p, stop, sync);
		// the code before the first sync
		if (!count++) {
			dump(p - dist, p - 1, 2);
			code = decode_low(p - dist, dist);
			printf("%s\n", printbits64(code, 0x1011001101));
			matrix_store(P_NEXUS, code);
		}
		// the code after the sync
		dump(p + 1, p + dist, 2);
		code = decode_low(p + 1, dist);
		printf("%s\n", printbits64(code, 0x1011001101));
		matrix_store(P_NEXUS, code);
	} while (p++ != stop);
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

// returns the best matching symbol for given distance
static uint8_t valid(uint8_t *xsymbols, uint8_t s, uint8_t d) {
	uint8_t t, td, tdmin = UINT8_MAX, tmin = 0;
	if (!s)
		return 0;
	// 1st round: try to find equals match
	for (int i = 0; i < SYMBOLS; i++) {
		t = xsymbols[i];
		if (!t)
			break; // table end
		if (s == t)
			return t;
	}

	if (!d)
		return 0;

	// 2nd round: find the best match with minimal distance
	for (int i = 0; i < SYMBOLS; i++) {
		t = xsymbols[i];
		if (!t)
			break; // table end
		td = t > s ? t - s : s - t;
		if (td > d)
			continue;
		if (td < tdmin) {
			tdmin = td;
			tmin = t;
		}
	}
	return tmin;
}

// same as valid() but additionally increments the hit counter of the matching symbol
static uint8_t valid_hit(uint8_t *xsymbols, uint8_t s, uint8_t d) {
	uint16_t *xcounter = select_counter(xsymbols);
	uint8_t t, td, tdmin = UINT8_MAX, tmin = 0;
	if (!s)
		return 0;
	for (int i = 0; i < SYMBOLS; i++) {
		t = xsymbols[i];
		if (!t)
			break; // table end
		if (s == t) {
			xcounter[i]++;
			return t;
		}
	}

	if (!d)
		return 0;

	for (int i = 0; i < SYMBOLS; i++) {
		t = xsymbols[i];
		if (!t)
			break; // table end
		td = t > s ? t - s : s - t;
		if (td > d)
			continue;
		if (td < tdmin) {
			tdmin = td;
			tmin = t;
			xcounter[i]++;
		}
	}
	return tmin;
}

static void learn(uint8_t *xsymbols, uint8_t s, const char direction) {
	uint16_t *xcounter = select_counter(xsymbols);

	// never learn 0, 1 and the biggest
	if (!s || s == 1 || s == UINT8_MAX)
		return;

	// right: symbols only allowed to grow, not shrink, also learn big SYNC pulses
	uint8_t lmax = biggest_symbol(lsymbols), hmax = biggest_symbol(hsymbols);
//	if (direction == R && s > UINT8_MAX / 2)
	if (direction == R && lmax && hmax && s > (lmax + hmax) * 2)
		return;

	// left: symbols only allowed to shrink, not grow, except multiples of the smallest
	uint8_t lmin = smallest_symbol(lsymbols), hmin = smallest_symbol(hsymbols);
	int multiple = 0, m1 = 1, m2 = 1, m3 = 1;
	if (xsymbols == lsymbols) {
		if (lmin)
			m1 = (s - 1) % lmin, m2 = s % lmin, m3 = (s + 1) % lmin;
		multiple = !m1 || !m2 || !m3;
	} else {
		if (lmax)
			m1 = (s - 1) % lmax, m2 = s % lmax, m3 = (s + 1) % lmax;
		multiple = !m1 || !m2 || !m3;
	}
	if (direction == L && s > lmin && s > hmin && !multiple)
		return;

	// find next free entry
	int i = 0;
	while (i < SYMBOLS && xsymbols[i])
		i++;
	if (i == SYMBOLS)
		return; // table full

	xsymbols[i] = s;
	xcounter[i] = 1;
}

// move all symbols to the left without gaps between
static void condense(uint8_t *xsymbols) {
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

// straighten symbols
static void align(uint8_t *xsymbols) {
	uint16_t *xcounter = select_counter(xsymbols);

	if (rfcfg->verbose)
		printf("DECODER %c aligning ", xsymbols == lsymbols ? L : H);

	// find minimum symbol distance
	uint8_t d, dmin = symbol_distance(xsymbols);
	uint8_t s = size_symbols(xsymbols);

	// minimum symbol distance is greater/equal table size --> we would loose information, so stop merging
	while (dmin < s) {

		if (rfcfg->verbose)
			printf("    d=%d s=%d ", dmin, s);

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
				condense(xsymbols);
			}
		}

		// recalculate size and min symbol distance
		dmin = symbol_distance(xsymbols);
		s = size_symbols(xsymbols);
	}

	if (rfcfg->verbose)
		printf("\n");
}

// remove symbols below minimum occurrence
static void filter() {
	if (rfcfg->verbose)
		dump_tables("symbol tables ");

	// sort
	sort(lsymbols);
	sort(hsymbols);

	// condense
	condense(lsymbols);
	condense(hsymbols);

	// align
	align(lsymbols);
	align(hsymbols);

	if (rfcfg->verbose)
		dump_tables("after filter  ");
}

// move the stream to the right without gaps between
static void melt_condense(uint16_t start, uint16_t stop) {
	uint16_t p, pgap;

	int gaps;
	do {
		gaps = 0;
		p = stop + 1;
		while (--p != start) {

			if (lstream[p] && hstream[p])
				continue;

			// close this gap
			pgap = p + 1;
			while (--pgap != start) {
				lstream[pgap] = lstream[pgap - 1];
				hstream[pgap] = hstream[pgap - 1];
			}

			// create a gap at the beginning
			lstream[start] = 0;
			hstream[start] = 0;
			gaps = 1;
			start++;
			break;
		}
	} while (gaps);
}

// melt small spikes (L0, L1, ...) into next L symbol
static void melt(uint16_t start, uint16_t stop) {
	uint16_t p = start - 1;
	uint8_t d, dmin = lsymbols[0] < lsymbols[1] ? lsymbols[0] : lsymbols[1];
	if (dmin)
		dmin--;
	while (p++ != stop) {
		// melt and create a gap
		d = lstream[p] + hstream[p];
		if (d < dmin) {
			lstream[p + 1] += d;
			lstream[p] = 0;
			hstream[p] = 0;
		}
	}
}

// soft ironing stream up to minimum symbol distance
static void iron(uint8_t *xstream, uint16_t start, uint16_t stop) {
	uint16_t p;
	uint8_t s, m, *xsymbols = select_symbols(xstream);
	uint8_t dmin = symbol_distance(xsymbols);

	for (int i = 0; i <= dmin; i++) {

		if (rfcfg->verbose)
			printf("DECODER %s ironing d=%d ", xstream == lstream ? "L" : "H", i);

		p = start - 1;
		int fixed = 0;
		while (p++ != stop) {

			s = xstream[p];
			if (!s)
				continue;

			m = valid(xsymbols, s, i);
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

// TODO
// die übrig gebliebenen fehler versuchen zu korrigieren:
// wenn in H nur ein einziges vorkommt dann das nehmen, wenn mehrere das mit der größten häufigkeit

// brutal straighten invalid symbols
static void hammer(uint16_t start, uint16_t stop) {
	// do not try to fix errors on the left side - we do not know if they belong to the signal
	start += 8;
	// and of course not the last position which indicates the EOT
	stop -= 1;

	uint16_t p = start - 1;
	uint8_t l, lv, h, hv;

	if (rfcfg->verbose) {
		printf("DECODER invalid symbols ");
		red();
	}

	int invalid = 0;
	while (p++ != stop - 1) {
		l = lstream[p];
		h = hstream[p];

		lv = valid_hit(lsymbols, l, 0);
		hv = valid_hit(hsymbols, h, 0);

		if (!lv) {
			invalid++;
			if (rfcfg->verbose)
				printf("L%d[%u] ", l, p);
		}

		if (!hv) {
			invalid++;
			if (rfcfg->verbose)
				printf("H%d[%u] ", h, p);
			if (!h) {
				// TODO - an dieser stelle wurde der Empfang durch ein zu langes H unterbrochen - das sind genau die fehlenden bits
			}
		}
	}

	if (rfcfg->verbose) {
		attroff();
		printf("\n");
		if (!invalid)
			clearline();
	}
}

static int tune(uint16_t pos, const char direction) {
	int e = 0;
	uint8_t l, h, ll, hl, lh, hh;

	// smallest + biggest symbols
	// uint8_t lmin = smallest_symbol(lsymbols), hmin = smallest_symbol(hsymbols);
	uint8_t lmax = biggest_symbol(lsymbols), hmax = biggest_symbol(hsymbols);

	l = lstream[pos];
	h = hstream[pos];

	if (!l && !h)
		e = -1; // error or dead stream

	// bigger than biggest is not allowed
	if (l > (lmax + 3) || h > (hmax + 3))
		e = UINT8_MAX;

	// allow more distance to be fault tolerant on a single position
	ll = valid(lsymbols, l, 3);
	hl = valid(hsymbols, l, 3);
	lh = valid(lsymbols, h, 3);
	hh = valid(hsymbols, h, 3);

	// l not valid in l/h
	if (!ll && !hl)
		e = 1;

	// h not valid in l/h
	if (!lh & !hh)
		e = 2;

	if (rfcfg->verbose) {
		printf("DECODER tune ");
		if (direction == L)
			printf("◄%05u  ", pos);
		if (direction == R)
			printf(" %05u► ", pos);
		printf("%3d(%2d,%2d) L   %3d(%2d,%2d) H   %3d E\n", l, ll, hl, h, lh, hh, e);
	}

	return !e;
}

static uint16_t probe_left(uint16_t start) {
	uint16_t p = start + 1;
	uint8_t l, lv, h, hv;

	// to the right we already collected symbols - use them as error indicator
	uint8_t lmin = smallest_symbol(lsymbols), hmin = smallest_symbol(hsymbols);
	uint8_t lmax = biggest_symbol(lsymbols), hmax = biggest_symbol(hsymbols);
	uint16_t ex = 5 * (lmin + hmin);

	printf("DECODER probe  ◄        {%2d,%2d} L      {%2d,%2d} H   %3d Ex\n", lmin, lmax, hmin, hmax, ex);

	int e = 0; // floating error counter
	while (p-- != streampointer + 1) {
		l = lstream[p];
		h = hstream[p];

		if (!l && !h)
			break; // dead stream

		// biggest on left - might be a sync, but bigger are not allowed
		if (l > (lmax + hmax) || h > (lmax + hmax)) {
			printf("DECODER probe |◄%05d   %3d(%2d) L      %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));
			break;
		}

		// null distance because we want to learn
		lv = valid_hit(lsymbols, l, 0);
		hv = valid_hit(hsymbols, h, 0);

		if (lv)
			e -= l;
		else {
			learn(lsymbols, l, L);
			e += l < lmin ? lmin : l;
		}

		if (hv)
			e -= h;
		else {
			learn(hsymbols, h, L);
			e += h < hmin ? hmin : h;
		}

		if (e < 0)
			e = 0;

		// tolerate sampling errors, but too much -> jump out
		if (e > ex) {
			printf("DECODER probe |◄%05d   %3d(%2d) L      %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));
			break;
		}

		printf("DECODER probe  ◄%05d   %3d(%2d) L      %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));
	}
	return p;
}

static uint16_t probe_right(uint16_t start, uint16_t stop) {
	uint16_t p;
	uint8_t l, lv, h, hv, lmin, hmin, lmax, hmax;

	// find the biggest/smallest H sample and learn it
	p = biggest_sample(hstream, start, stop);
	hmax = hstream[p];
	p = smallest_sample(hstream, start, stop);
	hmin = hstream[p];

	// find the smallest L sample and learn them - also here we start expanding to right
	p = biggest_sample(lstream, start, stop);
	lmax = lstream[p];
	p = smallest_sample(lstream, start, stop);
	lmin = lstream[p];

	// initially learn them
	lsymbols[0] = lmin;
	lcounter[0] = 1;
	hsymbols[0] = hmin;
	hcounter[0] = 1;
	lsymbols[1] = lmax;
	lcounter[1] = 1;
	hsymbols[1] = hmax;
	hcounter[1] = 1;

	if (rfcfg->verbose)
		printf("DECODER probe        ►  {%2d,%2d} L      {%2d,%2d} H\n", lmin, lmax, hmin, hmax);

	p--;
	int e = 0; // floating error counter
	while (p++ != streampointer - 1) {
		l = lstream[p];
		h = hstream[p];

		if (!l && !h)
			break; // dead stream

		// null distance because we want to learn
		lv = valid_hit(lsymbols, l, 0);
		hv = valid_hit(hsymbols, h, 0);

		if (lv)
			e -= l;
		else {
			learn(lsymbols, l, R);
			e += l < lmin ? lmin : l;
		}

		if (hv)
			e -= h;
		else {
			learn(hsymbols, h, R);
			e += h < hmin ? hmin : h;
		}

		if (e < 0)
			e = 0;

		// reached EOT
		if (e >= UINT8_MAX - 10) {
			printf("DECODER probe   %05d►| %3d(%2d) L      %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));
			break;
		}

		printf("DECODER probe   %05d►  %3d(%2d) L      %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));
	}
	return p;
}

// find begin and end of a signal by analyzing symbols
static uint16_t probe(uint16_t start, uint16_t stop) {
	uint16_t estart, estop, dist;

	// this is crap
	dist = distance(start, stop);
	if (dist < 16)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER probing [%05u:%05u] %u samples\n", start, stop, dist);

	// clear tables before learning new symbols
	clear_tables();

	if (rfcfg->verbose)
		printf("DECODER L=low stream, H=high stream, symbol(valid), E=error counter, D=distance to stream head\n");

	// expand window to right, then left - right is more reliable due to signal goes into noise on left side
	estop = probe_right(start, stop);
	estart = probe_left(start);

	// this is crap
	dist = distance(estart, estop);
	if (dist < 16 || dist > 2048)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER probe window  [%05u:%05u] %u samples\n", estart, estop, dist);

	// filter, sort and align symbol tables
	filter();

	// melt small L+H spikes into next L - if we do it initially on lsamples/hsamples its hard to do the probe_left
	melt(estart, estop);
	melt_condense(estart, estop);
	while (!lstream[estart] && !hstream[estart] && estart != estop) // adjust start
		estart++;

	// this is again crap
	dist = distance(estart, estop);
	if (dist < 16 || dist > 2048)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER after melting [%05u:%05u] %u samples\n", estart, estop, dist);

	// fine tune right edge of window
	estop -= 8;
	while (tune(estop, R))
		estop++;
	estop--;

	// fine tune left edge of window
	estart += 16;
	while (tune(estart, L))
		estart--;
	estart++;

	// this is again crap
	dist = distance(estart, estop);
	if (dist < 16 || dist > 2048)
		return stop;

	// EOT - the receiver is adjusting it's sensitivity back to noise level
	// estimate signal strength from time to next l+h samples
	uint8_t ln = lstream[estop + 1], hn = hstream[estop + 1];
	uint16_t lsn = lsamples[estop + 1], hsn = hsamples[estop + 1];
	uint32_t strenth = (lsn + hsn) * 100 / (UINT16_MAX * 2);

	if (rfcfg->verbose)
		printf("DECODER tuned  [%05u:%05u] %u samples, signal %u%% estimated from L+1 %d(%05u) H+1 %d(%05u) after EOT\n", estart, estop, dist, strenth, ln, lsn, hn, hsn);

	// symbol soft error correction
	iron(lstream, estart, estop);
	iron(hstream, estart, estop);

	// symbol hard error correction
	hammer(estart, estop);

	// consume this stream window
	eat(estart, estop);

	// avoid re-catching this code
	clear_streams(estart, estop);

	return estop;
}

// round pulse length to multiples of 100 and divide by 100
static void scale(uint16_t start, uint16_t stop) {
	uint16_t sl, sh, p = start + 1, smax = UINT8_MAX * 100;
	uint8_t l, h;

	while (p++ != stop) {
		sl = lsamples[p];
		sh = hsamples[p];

		// too big - ? multiple signals at same time, receiver sensitivity was overridden by the stronger
		if (sl > smax)
			sl = smax;
		if (sh > smax)
			sh = smax;

		// scale down and round
		if ((sl % 100) < 50)
			l = sl / 100;
		else
			l = (sl / 100) + 1;
		if ((sh % 100) < 50)
			h = sh / 100;
		else
			h = (sh / 100) + 1;

		lstream[p] = l;
		hstream[p] = h;
	}
}

// dumb 4-block symbol pattern matching
static int sniff(uint16_t start) {
	uint8_t p0 = stream[start], p1 = stream[start + 1], p2 = stream[start + 2], p3 = stream[start + 3];

	// contains zeros
	if (!p0 || !p1 || !p2 || !p3)
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
		uint16_t l = lsamples[ptr];
		uint16_t h = hsamples[ptr];

		if (200 < l && l < 5000)
			valid++;

		if (200 < h && h < 5000)
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
	uint16_t start = 0, stop = 0;
	while (1) {
		msleep(rfcfg->decoder_delay);

		// wait if receiving in progress
		while (receiving())
			msleep(100);

		// this is the block we have to analyze in this round
		stop = streampointer;

		if (rfcfg->verbose)
			printf("DECODER analyzing [%05u:%05u] %u samples\n", start, stop, distance(start, stop));

		// shrink samples to max 256 symbols
		scale(start, stop);

		// pattern sniffer loop
		uint16_t block = distance(start, stop);
		while (block > 8) {

			// catch pattern by jumping 4-block-wise
			if (!sniff(start)) {
				start = forw(start, 4);
				block -= 4;
				continue;
			}

			uint16_t p1 = start;

			// jump forward till no match
			while (sniff(start + 4) && block > 4) {
				start = forw(start, 4);
				block -= 4;
			}

			uint16_t p2 = start;

			// detailed symbol analyzing
			if (p1 != p2)
				start = probe(p1, p2);

			block = distance(++start, stop);
		}

		matrix_decode();
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

	// initialize stream
	streampointer = 0;
	last = gpio_micros();

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

		// pulse too long - store as maximum
		if (pulse > UINT16_MAX)
			pulse = UINT16_MAX;

		if (pin) {
			// that was a LOW pulse
			lsamples[streampointer] = (uint16_t) pulse;
			if (rfcfg->sample_on_0)
				streampointer++;
		} else {
			// that was a HIGH pulse
			hsamples[streampointer] = (uint16_t) pulse;
			if (rfcfg->sample_on_1)
				streampointer++;
		}
	}
	return (void*) 0;
}

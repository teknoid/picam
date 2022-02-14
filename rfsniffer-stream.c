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
static uint64_t decode_low(uint16_t pos, uint8_t bits, uint16_t divider) {
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

// consume the signal
static void eat(uint16_t start, uint16_t stop) {
	select_hstream();
	if (rfcfg->verbose)
		dump(start, stop);

	uint64_t lcode = decode_low(stop - 64, 64, 15);
	uint64_t hcode = decode_high(stop - 64, 64, 6);
	if (rfcfg->verbose)
		printf("DECODER probe_low 0x%016llx == %s probe_high 0x%016llx == %s\n", lcode, printbits64(lcode, SPACEMASK64), hcode, printbits64(hcode, SPACEMASK64));
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

	// right: symbols only allowed to grow, not shrink
	uint8_t lmax = biggest_symbol(lsymbols), hmax = biggest_symbol(hsymbols);
	if (direction == R && s < lmax && s < hmax)
		return;

	// left: symbols only allowed to shrink, not grow
	uint8_t lmin = smallest_symbol(lsymbols), hmin = smallest_symbol(hsymbols);
	if (direction == L && s > lmin && s > hmin)
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
static void filter(uint16_t samples) {
	// int min = samples > 100 ? samples / 100 : 1; !!! hier gehen informationen verloren !!! --> 20 kommt nur 1x vor ist aber gültig !!!
	int min = samples > 100 ? samples / 100 : 0; // TODO was bei Radiohead > 100 samples ???

	if (rfcfg->verbose)
		dump_tables("symbol tables ");

	// sort
	sort(lsymbols);
	sort(hsymbols);

	// clean
	// TODO ist es ein vielfaches vom kleinsten symbol? dann nicht löschen !!! --> modulo max +-1
	for (int i = 0; i < SYMBOLS; i++) {
		if (lcounter[i] <= min)
			lsymbols[i] = 0;
		if (hcounter[i] <= min)
			hsymbols[i] = 0;
	}

	// condense
	condense(lsymbols);
	condense(hsymbols);

	if (rfcfg->verbose)
		dump_tables("after filter  ");

	// melt
	align(lsymbols);
	align(hsymbols);

	if (rfcfg->verbose)
		dump_tables("after align   ");
}

static void melt(uint16_t start, uint16_t stop) {
	if (rfcfg->verbose)
		printf("DECODER melting ");
}

// soft ironing stream up to minimum symbol distance
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

	if (rfcfg->verbose)
		printf("DECODER invalid symbols ");

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
		printf("\n");
		if (!invalid)
			clearline();
	}
}

static int tune(uint16_t pos, const char direction) {
	uint16_t e = 0;
	uint8_t l, h, ll, hl, lh, hh;

	// smallest + biggest symbols
	// uint8_t lmin = smallest_symbol(lsymbols), hmin = smallest_symbol(hsymbols);
	uint8_t lmax = biggest_symbol(lsymbols), hmax = biggest_symbol(hsymbols);

	l = lstream[pos];
	h = hstream[pos];

	if (!l && !h)
		e = UINT8_MAX; // error or dead stream

	// border hard right hit -> EOT
	if (direction == R)
		if (l == UINT8_MAX || h == UINT8_MAX)
			e = UINT8_MAX;

	// bigger than biggest on the left is not allowed
	if (direction == L)
		if (l > (lmax + 5) || h > (hmax + 5))
			e = UINT8_MAX;

	// allow more distance to be fault tolerant on a single position
	ll = valid(lsymbols, l, 5);
	hl = valid(hsymbols, l, 5);
	lh = valid(lsymbols, h, 5);
	hh = valid(hsymbols, h, 5);

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
		printf("   L %3d(%2d,%2d)   H %3d(%2d,%2d)   E %d\n", l, ll, hl, h, lh, hh, e);
	}

	return !e;
}

static uint16_t probe_left(uint16_t start) {
	uint16_t p = start + 1;
	uint8_t l, lv, h, hv;

	// to the right we already collected symbols - use them as error indicator
	uint8_t lmin = smallest_symbol(lsymbols), hmin = smallest_symbol(hsymbols);
	uint8_t lmax = biggest_symbol(lsymbols), hmax = biggest_symbol(hsymbols);

	int e = 0; // floating error counter
	while (p-- != streampointer + 1) {
		l = lstream[p];
		h = hstream[p];

		if (!l && !h)
			break; // dead stream

		// biggest on left - might be a sync, but bigger are not allowed
		if (l > (lmax + hmax) || h > (lmax + hmax))
			e = UINT8_MAX;

		// take the next left as error - if its valid then error is healed on next position
		if (!l)
			e += lstream[p - 1];
		if (!h)
			e += hstream[p - 1];

		// small distance because we want to learn
		lv = valid_hit(lsymbols, l, 1);
		hv = valid_hit(hsymbols, h, 1);

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

		printf("DECODER probe ◄%05d   %3d(%2d) L   %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));

		// tolerate sampling errors, but too much -> jump out
		if (e > 5 * (lmin + hmin))
			break;
	}
	return p;
}

static uint16_t probe_right(uint16_t start, uint16_t stop) {
	uint16_t p;
	uint8_t l, lv, h, hv, lmin, hmin;

	// find the smallest H sample and learn it
	p = smallest_sample(hstream, start, stop);
	hmin = hstream[p];
	learn(hsymbols, hmin, R);

	// find the smallest L sample and learn them - also her we start expanding to right
	p = smallest_sample(lstream, start, stop);
	lmin = lstream[p];
	learn(lsymbols, lmin, R);

	if (rfcfg->verbose)
		printf("DECODER initially learned smallest symbols   L%d   H%d\n", lmin, hmin);

	p--;
	int e = 0; // floating error counter
	while (p++ != streampointer - 1) {
		l = lstream[p];
		h = hstream[p];

		if (!l && !h)
			break; // dead stream

		// take the next right as error - if its valid then error is healed on next position
		if (!l)
			e += lstream[p + 1];
		if (!h)
			e += hstream[p + 1];

		// small distance because we want to learn
		lv = valid_hit(lsymbols, l, 1);
		hv = valid_hit(hsymbols, h, 1);

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

		printf("DECODER probe  %05d►  %3d(%2d) L   %3d(%2d) H   %3d E   %05d D\n", p, l, lv, h, hv, e, distance(p, streampointer));

		// reached EOT
		if (e >= UINT8_MAX)
			break;
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
	if (dist < 16)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER probe window [%05u:%05u] %u samples\n", estart, estop, dist);

	// filter, sort and melt the symbol tables
	filter(dist);

	// fine tune right edge of window
	estop -= 8;
	while (tune(estop, R))
		estop++;

	// fine tune left edge of window
	estart += 16;
	while (tune(estart, L))
		estart--;

	// this is again crap
	dist = distance(estart, estop);
	if (dist < 16)
		return stop;

	if (rfcfg->verbose)
		printf("DECODER tuned window [%05u:%05u] %u samples\n", estart, estop, dist);

	// melt short spikes (H0, H1) to the next position
	melt(estart, estop);

	// symbol soft error correction
	iron(lstream, estart, estop);
	iron(hstream, estart, estop);

	// symbol hard error correction
	hammer(estart, estop);

	// EOT - the receiver is adjusting it's sensitivity back to noise level
	// estimate signal strength based on time to next sample
	uint8_t lnext = lstream[estop], hnext = rfcfg->sample_on_0 ? hstream[estop + 1] : hstream[estop];
	uint16_t lsnext = lsamples[estop], hsnext = rfcfg->sample_on_0 ? hsamples[estop + 1] : hsamples[estop];
	uint32_t strength = (lsnext + hsnext) * 100 / (UINT16_MAX * 2);
	if (rfcfg->verbose)
		printf("DECODER signal strength   %u%%   estimated from EOT   L+1 %d(%u)   H+1 %d(%u)\n", strength, lnext, lsnext, hnext, hsnext);

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

		// TODO noise ?

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
	uint16_t p1 = 0, p2 = 0;
	while (1) {
		msleep(rfcfg->decoder_delay);

		// wait if receiving in progress
		while (receiving())
			msleep(100);

		// this is the block we have to analyze in this round
		p2 = streampointer;
		uint16_t block = distance(p1, p2);

		if (rfcfg->verbose)
			printf("DECODER analyzing [%05u:%05u] %u samples\n", p1, p2, block);

		// shrink samples to max 256 symbols
		scale(p1, p2);

		// pattern sniffer loop
		while (block > 8) {

			// catch pattern by jumping 4-block-wise
			if (!sniff(p1)) {
				p1 = forw(p1, 4);
				block -= 4;
				continue;
			}

			uint16_t start = p1;

			// jump forward till no match
			while (sniff(p1 + 4) && block > 4) {
				p1 = forw(p1, 4);
				block -= 4;
			}

			uint16_t stop = p1;

			// detailed symbol analyzing
			if (start != stop)
				p1 = probe(start, stop);

			block = distance(++p1, p2);
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

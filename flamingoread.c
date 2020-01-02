#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <wiringPi.h>

#include "flamingo.h"

static struct timeval tNow, tLast;

static int bitcount;
static char bit, state, newstate, edge;
static unsigned long code, pulse, lastPulse;

static char *printBits() {
	char *out = malloc(sizeof(long) * 8) + 1;
	char *p = out;
	for (unsigned long mask = (1 << (sizeof(long) * 8 - 1)); mask > 0; mask >>= 1) {
		if (code & mask) {
			*p++ = '1';
		} else {
			*p++ = '0';
		}
	}
	*p++ = '\0';
	return out;
}

static void sample_hot() {
	bit = pulse > 660 ? 1 : 0;
	code = code << 1;
	code += bit;
	bitcount++;
//	printf("H %04lu/%04lu %04ld %02d %s 0x%08lx\n", lastPulse, pulse, lastPulse + pulse, bitcount, printBits(code), code);
	if (bitcount == 28) {
		printf("got code 0x%08lx\n", code);
	}
}

static void sample_cold() {
//	printf("C %04lu/%04lu %04ld %02d %s 0x%08lx\n", lastPulse, pulse, lastPulse + pulse, bitcount, printBits(code), code);
}

static void sync_1() {
	code = 0;
	bitcount = 0;
	if (newstate == 1) {
//		printf("detected lo sync 1\n");
		edge = 0;
	}
}

static void sync_2() {
	code = 0;
	bitcount = 0;
	if (newstate == 1) {
//		printf("detected lo sync 2\n");
		edge = 1;
	}
}

static void sync_3() {
	code = 0;
	bitcount = 0;
	if (newstate == 1) {
//		printf("detected lo sync 3\n");
		edge = 1;
	}
}

static void next() {
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;
	lastPulse = pulse;
	state = newstate;
}

static void poll() {

	// initialize
	pulse = lastPulse = 9999;
	state = digitalRead(RX);
	bitcount = 0;

	while (1) {
		// wait for next rising or falling edge
		do {
			newstate = digitalRead(RX);
		} while (state == newstate);

		// immediately calculate pulse length
		gettimeofday(&tNow, NULL);
		pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);

		// check for sync pulses
		if (4500 < pulse && pulse < 5000) {
			sync_1();
			next();
			continue;
		} else if (2500 < pulse && pulse < 3000) {
//			sync_2();
			next();
			continue;
		} else if (10000 < pulse && pulse < 11000) {
//			sync_3();
			next();
			continue;
		}

		// depending on encoded sequence - sample rising or falling edge
		if (newstate == edge) {
			sample_hot();
		} else {
			sample_cold();
		}

		next();
	}
}

int main(int argc, char **argv) {
	if (wiringPiSetup() == -1) {
		printf("Unable to start wiringPi");
		return -1;
	}

	// Set our thread to MAX priority
	struct sched_param sp;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (sched_setscheduler(0, SCHED_FIFO, &sp)) {
		return -2;
	}

	// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT)) {
		return -3;
	}

	pinMode(RX, INPUT);
	pullUpDnControl(RX, PUD_DOWN);

	code = (1L << 28);
	printf("test: bit 28 (must be printed as 0x10000000) --> 0x%08lx\n", code);

	state = digitalRead(RX);
	printf("pin state: %i\n", state);

	poll();
}

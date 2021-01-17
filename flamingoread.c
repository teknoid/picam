#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <wiringPi.h>

#include "flamingo.h"

static struct timeval tNow, tLast;

static unsigned char bits, state, clock, databits;
static unsigned long code, pulse;

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

// short high pulse followed by long low pulse or long high + short low pulse, no clock
//       _              ___
// 0 = _| |___    1 = _|   |_
//
// https://forum.arduino.cc/index.php?topic=201771.0
//
static void isr2824() {
	// calculate pulse length; store timer value for next calculation; get pin state
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// measure HIGH pulse length: short=0 long=1
	if (bits && (state == 0)) {
		code = code << 1;
		code += pulse > P2824X2 ? 1 : 0;
		if (--bits == 0) {
			printf("0x%08lx\n", code);
			code = 0;
		}
		return;
	}

	// ignore noise
	if (pulse < 100) {
		return;
	}

	// check for sync LOW pulses
	if (state == 1) {
		// a rising edge
		if (SYNCF28MIN < pulse && pulse < SYNCF28MAX) {
			bits = 28;
			printf("28bit LOW sync %lu :: ", pulse);
		} else if (SYNCF24MIN < pulse && pulse < SYNCF24MAX) {
			bits = 24;
			printf("24bit LOW sync %lu :: ", pulse);
		}
		// printf("LOW sync %lu\n", pulse);
	}
}

// clock pulse + data pulse, either short or long distance from clock to data pulse
//       _   _               _      _
// 0 = _| |_| |____    1 = _| |____| |_
//
// https://forum.pilight.org/showthread.php?tid=1110&page=12
//
static void isr32_short() {
	// calculate pulse length; store timer value for next calculation; get pin state
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// measure LOW pulses, skip clock and the check space between next pulse
	if (bits && (state == 1)) {
		// clock pulse -> skip
		if (clock) {
			clock = 0;
			code = code << 1;
			return;
		}

		// data pulse, check space between previous clock pulse: short=0 long=1
		code += pulse > P32X2 ? 1 : 0;
		if (--bits == 0) {
			printf("0x%08lx\n", code);
			code = 0;
		}
		clock = 1; // next is a clock pulse
		return;
	}

	// ignore noise
	if (pulse < 100) {
		return;
	}

	// check for sync LOW pulses
	if (state == 1) {
		// a rising edge
		if (SYNCF32MIN < pulse && pulse < SYNCF32MAX) {
			clock = 0; // this is the first clock pulse
			bits = 32;
			printf("32bit LOW short sync %lu :: ", pulse);
		}
	}
}

// similar to isr32_short but with longer sync
// looks like a clock pulse but more than one data pulse follow - encoding not yet known
static void isr32_long() {
	// calculate pulse length; store timer value for next calculation; get pin state
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// measure LOW pulses and decide if it was a data bit or a clock bit
	if (bits && (state == 1)) {
		if (pulse < P32X2) {
			// simply count the data bits
			databits++;
		} else {
			// next clock bit, store data bits
			printf("%d", databits);
			if (--bits == 0) {
				printf("\n");
			}
			databits = 0;
		}
		return;
	}

	// ignore noise
	if (pulse < 100) {
		return;
	}

	// check for sync LOW pulses
	if (state == 1) {
		// a rising edge
		if (9200 < pulse && pulse < 9300) {
			bits = 32;
			printf("32bit LOW long sync %lu :: ", pulse);
		}
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

	// initialize
	state = digitalRead(RX);
	printf("pin state: %i\n", state);

	bits = 0;

	// parse command line arguments
	int c = getopt(argc, argv, "123");
	switch (c) {
	case '1':
		printf("listening on 28bit & 24bit codes...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr2824) < 0) {
			return -4;
		}
		break;
	case '2':
		printf("listening on 32bit codes short sync...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32_short) < 0) {
			return -4;
		}
		break;
	case '3':
		printf("listening on 32bit codes long sync...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32_long) < 0) {
			return -4;
		}
		break;
	default:
		printf("Usage: flamingoread -1 | -2 | -3\n");
		printf("  -1 ... detect 28bit & 24bit codes, rc pattern 1 and 4\n");
		printf("  -2 ... detect 32bit codes with short sync, rc pattern 2\n");
		printf("  -3 ... detect 32bit codes with long sync, rc pattern 3, encoding still unknown\n");
		return -5;
	}

	while (1) {
		sleep(1);
	}
}

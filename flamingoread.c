#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>

#include <wiringPi.h>

#include "utils.h"
#include "flamingo.h"
#include "flamingocrypt.h"

static struct timeval tNow, tLast;

static unsigned int txid;

static unsigned long code, message, pulse;

static unsigned char bits, state, clock, databits, command, channel, payload;

// short high pulse followed by long low pulse or long high + short low pulse, no clock
//       _              ___
// 0 = _| |___    1 = _|   |_
//
// https://forum.arduino.cc/index.php?topic=201771.0
//
static void isr28() {
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
			code = 0;
			bits = 28;
			printf("28bit LOW sync %lu :: ", pulse);
		}
		// printf("LOW sync %lu\n", pulse);
	}
}

static void isr24() {
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
		if (SYNCF24MIN < pulse && pulse < SYNCF24MAX) {
			code = 0;
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
// 32 bit pattern: 00000000 1000001101011010 0001 0001
//                          1                2    3
//
// 1=Transmitter ID, 16 bit
// 2=Command, 4 bit, 0=OFF, 1=ON
// 3=Channel, 4 bit
//
// https://forum.pilight.org/showthread.php?tid=1110&page=12
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
			channel = code & 0x0f;
			command = code >> 4 & 0x0f;
			txid = code >> 8 & 0xffff;
			printf("0x%08lx Transmitter=0x%04x Unit=%d Command=%i\n", code, txid, channel, command);
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
			code = 0;
			bits = 32;
			clock = 0; // this is the first clock pulse
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
			code = 0;
			bits = 32;
			printf("32bit LOW long sync %lu :: ", pulse);
		}
	}
}

int main(int argc, char **argv) {
	char loop_decode = 0;

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
	int c = getopt(argc, argv, "1234");
	switch (c) {
	case '1':
		printf("listen to 28bit codes...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr28) < 0) {
			return -4;
		}
		loop_decode = 1;
		break;
	case '2':
		printf("listen to 32bit codes short sync...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32_short) < 0) {
			return -4;
		}
		break;
	case '3':
		printf("listen to 32bit codes long sync...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32_long) < 0) {
			return -4;
		}
		break;
	case '4':
		printf("listen to 24bit codes...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr24) < 0) {
			return -4;
		}
		break;
	default:
		printf("Usage: flamingoread -1 | -2 | -3 | -4\n");
		printf("  -1 ... detect 28bit rolling codes, rc pattern 1\n");
		printf("  -2 ... detect 32bit codes with short sync, rc pattern 2\n");
		printf("  -3 ... detect 32bit codes with long sync, rc pattern 3, encoding still unknown\n");
		printf("  -4 ... detect 24bit codes, rc pattern 4\n");
		return -5;
	}

	while (1) {
		sleep(3);

		if (loop_decode && !bits && code) {
			// try to decrypt received code if we have a rc pattern 1 code
			message = decrypt(code);

			printf("\n");
			printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(message, 0x01000110), message, code);

			txid = decode_txid(message);
			channel = decode_channel(message);
			command = decode_command(message);
			payload = decode_payload(message);
			printf("Transmitter-Id = 0x%x    channel = %d    command = %d    payload = 0x%02x\n", txid, channel, command, payload);
		}
	}
}

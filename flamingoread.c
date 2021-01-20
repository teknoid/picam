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

static unsigned int xmitter;

static unsigned long code, message, pulse;

static unsigned char bits, state, clock, databits, command, channel, payload;

//
// 4x rc1 pattern
//
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
		code += pulse > T1X2 ? 1 : 0;
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
		if (T1SMIN < pulse && pulse < T1SMAX) {
			code = 0;
			bits = 28;
			printf("28bit LOW sync %lu :: ", pulse);
		}
		// printf("LOW sync %lu\n", pulse);
	}
}

//
// 5x rc4 pattern
//
// similar to is28 but only 24bit code without encryption
//
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
		code += pulse > T1X2 ? 1 : 0;
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
		if (T4SMIN < pulse && pulse < T4SMAX) {
			code = 0;
			bits = 24;
			printf("24bit LOW sync %lu :: ", pulse);
		}
		// printf("LOW sync %lu\n", pulse);
	}
}

//
// 3x rc2 pattern
//
// clock pulse + data pulse, either short or long distance from clock to data pulse
//       _   _               _      _
// 0 = _| |_| |____    1 = _| |____| |_
//
// 32 bit pattern: 00000000 1000001101011010 0001 0001
//                          1                2    3
// 1=Transmitter ID, 16 bit
// 2=Command, 4 bit, 0=OFF, 1=ON
// 3=Channel, 4 bit
//
// https://forum.pilight.org/showthread.php?tid=1110&page=12
static void isr32() {
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
		code += pulse < T2Y ? 0 : 1;
		if (--bits == 0) {
			channel = code & 0x0f;
			command = code >> 4 & 0x0f;
			xmitter = code >> 8 & 0xffff;
			// printf("0x%08lx Transmitter=0x%04x Unit=%d Command=%i\n", code, xmitter, channel, command);
			printf("%s\n", printbits(code, 0x01000110));
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
		if (T2SMIN < pulse && pulse < T2SMAX) {
			code = 0;
			bits = 32;
			clock = 0; // this is the first clock pulse
			printf("32bit LOW sync %lu :: ", pulse);
		}
	}
}

//
// 3x rc3 pattern
//
// similar to isr32_short but with longer sync
// looks like a clock pulse but more than one data pulses follow
// encoding not yet known, simply count the data bits after clock bit
static void isr32_multibit() {
	// calculate pulse length; store timer value for next calculation; get pin state
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// measure LOW pulses and decide if it was a data bit or a clock bit
	if (bits && (state == 1)) {
		if (pulse < T3Y) {
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
		if (T3SMIN < pulse && pulse < T3SMAX) {
			code = 0;
			bits = 32;
			printf("32bit LOW multibit sync %lu :: ", pulse);
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
		printf("listen to 28bit rc1 rolling codes...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr28) < 0) {
			return -4;
		}
		loop_decode = 1;
		break;
	case '2':
		printf("listen to 32bit rc2 codes...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32) < 0) {
			return -4;
		}
		break;
	case '3':
		printf("listen to 32bit rc3 multibit codes...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32_multibit) < 0) {
			return -4;
		}
		break;
	case '4':
		printf("listen to 24bit rc4 codes...\n");
		if (wiringPiISR(RX, INT_EDGE_BOTH, &isr24) < 0) {
			return -4;
		}
		break;
	default:
		printf("Usage: flamingoread -1 | -2 | -3 | -4\n");
		printf("  -1 ... detect 28bit rolling codes, rc1 pattern\n");
		printf("  -2 ... detect 32bit messages, rc2 pattern\n");
		printf("  -3 ... detect 32bit multibit messages, rc3 pattern, encoding still unknown\n");
		printf("  -4 ... detect 24bit messages, rc4 pattern\n");
		return -5;
	}

	while (1) {
		sleep(3);

		if (loop_decode && !bits && code) {
			// try to decrypt received code if we have a rc1 pattern code
			message = decrypt(code);

			printf("\n");
			printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(message, 0x01000110), message, code);

			xmitter = decode_xmitter(message);
			channel = decode_channel(message);
			command = decode_command(message);
			payload = decode_payload(message);
			printf("Transmitter-Id = 0x%x    channel = %d    command = %d    payload = 0x%02x\n", xmitter, channel, command, payload);
		}
	}
}

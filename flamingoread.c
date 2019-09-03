#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/mman.h>
#include <wiringPi.h>

#include "flamingo.h"

int bitcount = 0;
unsigned long timer = 0;
unsigned long lastTimer = 0;
unsigned long hiPulse = 0;
unsigned long loPulse = 0;
unsigned long pulse = 0;
long bit;
char state, record;

uint64_t code;

static void _trigger_flamingo() {
	timer = micros();
	pulse = timer - lastTimer;
	lastTimer = timer;

	state = digitalRead(RX);
	if (state == 1) {
		loPulse = pulse;
	} else {
		hiPulse = pulse;
	}

	if (record && state == 1) {
		bit = hiPulse - loPulse;
		if (bit > 0) {
			code++;
		}
		if (++bitcount == 28) {
			printf(" *** %02i bits received: 0x%08llx\n", bitcount, code);
			record = 0;
		}
		code = code << 1;
	}

	if (loPulse > 3000 && loPulse < 6000) { // first start condition on rising edge
		bitcount = 0;
		loPulse = 0;
		hiPulse = 0;
		code = 0;
		record = 1;
	}
}

//static void _trigger_generic() {
//	timer = micros();
//	pulse = timer - lastTimer;
//	lastTimer = timer;
//
//	state = digitalRead(RX);
//	if (state == 1) {
//		loPulse = pulse;
//	} else {
//		hiPulse = pulse;
//	}
//
//	if (loPulse > 3000 && loPulse < 15000) { // any start condition on rising edge
//		printf(" *** sync detected, lo=%05lu, hi=%05lu, %02i bits received: 0x%08llx\n", loPulse, hiPulse, bitcount, code);
//		bitcount = 0;
//		loPulse = 0;
//		hiPulse = 0;
//		code = 0;
//		return;
//	}
//
//	if (state == 1) {
//		code = code << 1;
//		bit = hiPulse - loPulse;
//		if (bit > 0) {
//			printf("1");
//			code++;
//		} else {
//			printf("0");
//		}
//		if (++bitcount % 8 == 0) {
//			printf(" ");
//		}
//	}
//}

int main(int argc, char **argv) {
	if (wiringPiSetup() == -1) {
		printf("Unable to start wiringPi");
		return -1;
	}

// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT)) {
		return -2;
	}

// Set our thread to real time priority
	struct sched_param sp;
	sp.sched_priority = SCHED_PRIO;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
		printf("Unable to set thread priority: %s\n", strerror(errno));
		return -3;
	}

	pinMode(RX, INPUT);
	pullUpDnControl(RX, PUD_UP);
	if (wiringPiISR(RX, INT_EDGE_BOTH, &_trigger_flamingo) < 0) {
		printf("Unable to setup ISR: %s\n", strerror(errno));
		return -4;
	}

	while (1) {
		sleep(1);
	}
}

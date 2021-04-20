/***
 *
 * Library for reading and decoding 433MHz messages
 *
 * Copyright (C) 04/2021 by teknoid
 *
 *
 * tested with following devices
 *
 * - ELRO Flamingo Switches
 * - NEXUS Weather sensor
 */

#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>

#include <wiringPi.h>

#include "flamingo.h"
#include "utils.h"

#define DEBUG

#define BUFFER		255
#define MAX_EDGES	128 // max 64 bit code = 128 high/low edges

// global variables used in GPIO interrupt handler
static struct timeval tNow, tLast;
static unsigned long long isr_code;
static unsigned long isr_pulse;
static unsigned char isr_pattern, isr_bits, isr_state, isr_kill, isr_index;
static unsigned char isr_pattern_buffer[BUFFER]; // ring buffer pattern type
static unsigned long isr_sync_buffer[BUFFER]; // ring buffer sync pulse
static unsigned long long isr_code_buffer[BUFFER]; // ring buffer codes

static pthread_t thread_decoder;
static void* decoder(void *arg);

static void isr() {
	// calculate isr_pulse length; store timer value for next calculation; get pin state
	gettimeofday(&tNow, NULL);
	isr_state = digitalRead(RX);
	isr_pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// ignore noise & crap
	if (isr_pulse < 111 || isr_pulse > 22222)
		return;

	// error detection
	if (--isr_kill == 0) {
		printf("sampling %d aborted\n", isr_pattern);
		isr_bits = 0;
		isr_pattern = 0;
		return;
	}

	switch (isr_pattern) {

	case 1: // nexus
		if (isr_bits && isr_state) {
			if (isr_pulse > 1500)
				isr_code++;

			if (--isr_bits == 0) {
				isr_code_buffer[isr_index++] = isr_code;
				isr_pattern = 0;
			} else {
				isr_code <<= 1;
			}
		}
		break;

	case 2: // flamingo 28bit
		if (isr_bits && !isr_state) {
			if (isr_pulse > T1X2)
				isr_code++;

			if (--isr_bits == 0) {
				isr_code_buffer[isr_index++] = isr_code;
				isr_pattern = 0;
			} else {
				isr_code <<= 1;
			}
		}
		break;

	case 3: // flamingo 24bit
		if (isr_bits && !isr_state) {
			if (isr_pulse > T1X2)
				isr_code++;

			if (--isr_bits == 0) {
				isr_code_buffer[isr_index++] = isr_code;
				isr_pattern = 0;
			} else {
				isr_code <<= 1;
			}
		}
		break;

	case 4: // flamingo 32bit
		if (isr_bits && isr_state) {
			isr_code += isr_pulse < T2Y ? 0 : 1;
			if (--isr_bits == 0) {
				isr_code_buffer[isr_index++] = isr_code;
				isr_pattern = 0;
			} else {
				isr_code <<= 1;
			}
		}
		break;

	case 99: // ??? test ???
		if (isr_bits) {
			isr_code += isr_pulse < 750 ? 0 : 1;

//			if (isr_state)
//				printf("L%04lu ", isr_pulse);
//			else
//				printf("H%04lu ", isr_pulse);

			if (--isr_bits == 0) {
				isr_code_buffer[isr_index++] = isr_code;
				isr_pattern = 0;
//				printf("\n");
			} else {
				isr_code <<= 1;
			}
		}
		break;

	default: // detect SYNC sequences and start pattern sampling
		isr_code = 0;
		isr_kill = MAX_EDGES + 1;

		if (isr_state == 1) {
			// LOW sync pulses
			if (3850 < isr_pulse && isr_pulse < 4000) {
				// not a real sync - it's the pause between 1st and 2nd message
				isr_bits = 36;
				isr_pattern = 1;
				isr_pattern_buffer[isr_index] = isr_pattern;
				isr_sync_buffer[isr_index] = isr_pulse;
			} else if (T1SMIN < isr_pulse && isr_pulse < T1SMAX) {
				isr_bits = 28;
				isr_pattern = 2;
				isr_pattern_buffer[isr_index] = isr_pattern;
				isr_sync_buffer[isr_index] = isr_pulse;
			} else if (T4SMIN < isr_pulse && isr_pulse < T4SMAX) {
				isr_bits = 24;
				isr_pattern = 3;
				isr_pattern_buffer[isr_index] = isr_pattern;
				isr_sync_buffer[isr_index] = isr_pulse;
			} else if (T2S1MIN < isr_pulse && isr_pulse < T2S1MAX) {
				isr_bits = 64;
				isr_pattern = 4;
				isr_pattern_buffer[isr_index] = isr_pattern;
				isr_sync_buffer[isr_index] = isr_pulse;
			}
		} else {
			// HIGH sync pulses
			if (800 < isr_pulse && isr_pulse < 900) {
				isr_bits = 32;
				isr_pattern = 99;
				isr_pattern_buffer[isr_index] = isr_pattern;
				isr_sync_buffer[isr_index] = isr_pulse;
			}
		}
	}
}

static unsigned long decode_0110(unsigned long long in) {
	unsigned long long shift;
	unsigned char mask = 0b11;
	unsigned char bit0 = 0b01;
	unsigned char bit1 = 0b10;
	unsigned char count = 64;
	unsigned long out = 0;

	// printf("\n0110 %s", printbits64(in, 0x0001000100010001));
	while (count > 0) {
		shift = (in >> 62) & mask;

		out <<= 1;
		if (shift == bit0) {
			out += 0;
		} else if (shift == bit1) {
			out += 1;
		} else {
			printf("\0110 decode error %s\n", printbits64(in, 0x0001000100010001));
			return 0;
		}
		count -= 2;
		in <<= 2;
	}
	// printf("--> %s\n", printbits(out, 0x00010001));
	return out;
}

static void decode_nexus(unsigned char i) {
	unsigned long long code = isr_code_buffer[i];
	unsigned long sync = isr_sync_buffer[i];
	printf("NEXUS sync %lu 0x%08llx", sync, code);

	if ((code & 0x0f00) != 0x0f00) {
		printf(" message verification failed\n");
		return;
	}
	// https://github.com/merbanan/rtl_433/blob/master/src/devices/nexus.c
	unsigned char humidity = code & 0xff;
	code >>= 8; // humidity
	code >>= 4; // const always 1111 - used for message verification
	int temp = (int) code & 0x0fff;
	code >>= 12; // temp
	unsigned char channel = code & 0x07;
	code >>= 3;
	unsigned char battery = code & 0x01;
	code >>= 1;
	unsigned char id = code & 0xff;
	printf(" :: Id = %d, Channel = %d, Battery = %d, Temperature = %02.1fC, Humidity = %d%%\n", id, channel, battery, ((float) temp * 0.1), humidity);
}

static void decode_flamingo28(unsigned char i) {
	unsigned long code = (unsigned long) isr_code_buffer[i];
	unsigned long sync = isr_sync_buffer[i];
	printf("FLAMINGO28 sync %lu 0x%04lx", sync, code);

	unsigned long message;
	unsigned int xmitter;
	unsigned char channel, command, payload, rolling;

	message = decrypt(code);
	decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);
	printf(" :: Transmitter-Id = 0x%x, channel = %d, command = %d, payload = 0x%02x, rolling = %d\n", xmitter, channel, command, payload, rolling);
}

static void decode_flamingo24(unsigned char i) {
	unsigned long code = (unsigned long) isr_code_buffer[i];
	unsigned long sync = isr_sync_buffer[i];
	printf("FLAMINGO24 sync %lu 0x%04lx", sync, code);
	printf(" %s\n", printbits(code, 0x01010101));
	// TODO decode
}

static void decode_flamingo32(unsigned char i) {
	unsigned long long code_raw = isr_code_buffer[i];
	unsigned long sync = isr_sync_buffer[i];
	printf("FLAMINGO32 sync %lu 0x%08llx", sync, code_raw);

	// decode code_raw into 01=0 and 10=1
	unsigned long code = decode_0110(code_raw);
	if (code == 0)
		return;

	printf(" :: decoded 0x%0lx", code);

	unsigned int xmitter;
	unsigned char channel, command, payload;
	channel = code & 0x0f;
	code >>= 4; // channel
	command = code & 0x0f;
	code >>= 4; // channel
	xmitter = code & 0xffff;
	code >>= 16; // xmitter
	payload = code & 0xff;
	printf(" :: Transmitter-Id = 0x%x, channel = %d, command = %d, payload = 0x%02x, \n", xmitter, channel, command, payload);
}

static void decode_test(unsigned char i) {
	unsigned long long code_raw = isr_code_buffer[i];
	unsigned long sync = isr_sync_buffer[i];
	printf("TEST sync %04lu 0x%08llx", sync, code_raw);
	printf(" %s\n", printbits64(code_raw, 0x0101010101010101));
}

static void decode(unsigned char index) {
	unsigned char pattern = isr_pattern_buffer[index];
	switch (pattern) {
	case 1:
		return decode_nexus(index);
	case 2:
		return decode_flamingo28(index);
	case 3:
		return decode_flamingo24(index);
	case 4:
		return decode_flamingo32(index);
	case 99:
		return decode_test(index);
	default:
		printf("no decoder configured for pattern %d sync %lu 0x%0llx\n", pattern, isr_sync_buffer[index], isr_code_buffer[index]);
	}
}

int main(int argc, char **argv) {
	// test
	printf("test 2x 0xdeadbeef = %s\n", printbits64(0xdeadbeefdeadbeef, 0x0101010101010101));

	// Set our thread to MAX priority
	struct sched_param sp;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (sched_setscheduler(0, SCHED_FIFO, &sp)) {
		return 0;
	}

	// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT)) {
		return 0;
	}

	// decoder thread
	if (pthread_create(&thread_decoder, NULL, &decoder, NULL)) {
		perror("Error creating decoder thread");
		return -1;
	}
	printf("started decoder thread\n");

	// initialize wiringPi, GPIO pin connected to 433MHz receiver module
	if (wiringPiSetup() < 0) {
		return -1;
	}
	pinMode(RX, INPUT);
	pullUpDnControl(RX, PUD_DOWN);

	// GPIO interrupt service routine
	isr_index = isr_bits = 0;
	isr_state = digitalRead(RX);
	if (wiringPiISR(RX, INT_EDGE_BOTH, &isr) < 0) {
		return -1;
	}
	printf("installed ISR for GPIO_GEN%i\n", RX);

	// wait
	pause();

	if (thread_decoder) {
		if (pthread_cancel(thread_decoder)) {
			perror("Error canceling thread");
		}
		if (pthread_join(thread_decoder, NULL)) {
			perror("Error joining thread");
		}
	}

	return 0;
}

static void* decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	unsigned char decoder_index = 0;
	while (1) {
		sleep(2);

		if (decoder_index == isr_index)
			continue; // nothing received

		int todo = (isr_index > decoder_index) ? (isr_index - decoder_index) : (decoder_index - isr_index);
		printf("DECODER [%d/%d] processing %i codes\n", decoder_index, isr_index, todo);

		if (decoder_index > isr_index)
			while (decoder_index > isr_index)
				decode(decoder_index++);

		else
			while (decoder_index < isr_index)
				decode(decoder_index++);

	}

	// Ideen

	// --- ring buffer ---
	// thread läuft alle 3 sekunden und überwacht value slots,
	// wenn < 5 drin sind: löschen und next
	// wenn >= 5 drin sind dann auswerten

	// --- nexus multimessages ---
	// ersten und letzten wegschmeissen
	// prüfen ob alle 3 identisch sind
	// nein --> löschen und next
	// ja --> dekodieren und ausgeben

	// --- decoder ---
	// mit semaphore
	// wenn sync empfangen wird sperren,
	// wenn erfolgreich beenden öffnen
	// wenn kill runter gezalt hat auch öffnen

	return (void *) 0;
}

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
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/time.h>

#include <bcm2835.h>

#include "flamingo.h"
#include "utils.h"

#define GPIO_PIN	27	//RPi2 Pin13 GPIO_GEN2

#define BUFFER		255
#define MAX_EDGES	128 // max 64 bit code = 128 high/low edges

// ring buffers
static unsigned char pattern_buffer[BUFFER];
static unsigned long sync_buffer[BUFFER];
static unsigned long long code_buffer[BUFFER];
static unsigned char sampler_index, decoder_index = 0;

static pthread_t thread_decoder;
static void* decoder(void *arg);

static pthread_t thread_sampler;
static void* sampler(void *arg);

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

int main(int argc, char **argv) {
	// test
	printf("test 2x 0xdeadbeef = %s\n", printbits64(0xdeadbeefdeadbeef, 0x0101010101010101));

	if (!bcm2835_init())
		return -1;

	// decoder thread
	if (pthread_create(&thread_decoder, NULL, &decoder, NULL)) {
		perror("Error creating decoder thread");
		return -1;
	}
	printf("started decoder thread\n");

	// sampler thread
	if (pthread_create(&thread_sampler, NULL, &sampler, NULL)) {
		perror("Error creating sampler thread");
		return -1;
	}
	printf("started sampler thread\n");

	// wait
	pause();

	if (thread_sampler) {
		if (pthread_cancel(thread_sampler)) {
			perror("Error canceling thread");
		}
		if (pthread_join(thread_sampler, NULL)) {
			perror("Error joining thread");
		}
	}

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
	unsigned long long code = code_buffer[i];
	unsigned long sync = sync_buffer[i];
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
	unsigned long code = (unsigned long) code_buffer[i];
	unsigned long sync = sync_buffer[i];
	printf("FLAMINGO28 sync %lu 0x%04lx", sync, code);

	unsigned long message;
	unsigned int xmitter;
	unsigned char channel, command, payload, rolling;

	message = decrypt(code);
	decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);
	printf(" :: Transmitter-Id = 0x%x, channel = %d, command = %d, payload = 0x%02x, rolling = %d\n", xmitter, channel, command, payload, rolling);
}

static void decode_flamingo24(unsigned char i) {
	unsigned long code = (unsigned long) code_buffer[i];
	unsigned long sync = sync_buffer[i];
	printf("FLAMINGO24 sync %lu 0x%04lx", sync, code);
	printf(" %s\n", printbits(code, 0x01010101));
	// TODO decode
}

static void decode_flamingo32(unsigned char i) {
	unsigned long long code_raw = code_buffer[i];
	unsigned long sync = sync_buffer[i];
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
	unsigned long long code_raw = code_buffer[i];
	unsigned long sync = sync_buffer[i];
	printf("TEST sync %04lu 0x%08llx", sync, code_raw);
	printf(" %s\n", printbits64(code_raw, 0x0101010101010101));
}

static void decode(unsigned char index) {
	unsigned char pattern = pattern_buffer[index];
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
		printf("no decoder configured for pattern %d sync %lu 0x%0llx\n", pattern, sync_buffer[index], code_buffer[index]);
	}
}

static void* sampler(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	// Set our thread to MAX priority
	struct sched_param sp;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (sched_setscheduler(0, SCHED_FIFO, &sp)) {
		return (void *) 0;
	}

	// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT)) {
		return (void *) 0;
	}

	// gpio pin setup
	bcm2835_gpio_fsel(GPIO_PIN, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(GPIO_PIN, BCM2835_GPIO_PUD_UP);
	bcm2835_gpio_fen(GPIO_PIN);
	bcm2835_gpio_ren(GPIO_PIN);

	// poll setup
	char buf[32];
	snprintf(buf, 32, "/sys/class/gpio/gpio%d/value", GPIO_PIN);
	struct pollfd fdset[1];
	fdset[0].fd = open(buf, O_RDONLY);
	fdset[0].events = POLLPRI;
	fdset[0].revents = 0;

	struct timeval tNow, tLast;
	unsigned long long code;
	unsigned long pulse;
	unsigned char pattern, bits, state, kill, dummy;

	while (1) {
		// wait for interrupt
		poll(fdset, 1, 33333);

		// sample time + pin state
		gettimeofday(&tNow, NULL);
		state = bcm2835_gpio_lev(GPIO_PIN);

		// calculate length of last pulse, store timer value for next calculation
		pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
		tLast.tv_sec = tNow.tv_sec;
		tLast.tv_usec = tNow.tv_usec;

		// rewind & clear for next poll
		lseek(fdset[0].fd, 0, SEEK_SET);
		read(fdset[0].fd, &dummy, 1);

		// ignore noise & crap
		if (pulse < 222 || pulse > 22222)
			continue;

		// error detection
		if (--kill == 0) {
			printf("sampling %d aborted\n", pattern);
			bits = 0;
			pattern = 0;
			continue;
		}

		switch (pattern) {

		case 1: // nexus - sample LOW pulses
			if (bits && state) {
				if (pulse > 1500)
					code++;

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					pattern = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 2: // flamingo 28bit - sample HIGH pulses
			if (bits && !state) {
				if (pulse > T1X2)
					code++;

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					pattern = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 3: // flamingo 24bit - sample HIGH pulses
			if (bits && !state) {
				if (pulse > T1X2)
					code++;

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					pattern = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 4: // flamingo 32bit - sample LOW pulses
			if (bits && state) {
				code += pulse < T2Y ? 0 : 1;
				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					pattern = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 99: // ??? test - sample both HIGH/LOW pulses
			if (bits) {
				code += pulse < 750 ? 0 : 1;

				if (state)
					printf("L%04lu ", pulse);
				else
					printf("H%04lu ", pulse);

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					pattern = 0;
					printf("\n");
				} else {
					code <<= 1;
				}
			}
			break;

		default: // detect SYNC sequences and start pattern sampling
			code = 0;
			kill = MAX_EDGES + 1;

			if (state == 1) {
				// LOW sync pulses
				if (3850 < pulse && pulse < 4000) {
					// not a real sync - it's the pause between 1st and 2nd message
					bits = 36;
					pattern = 1;
					pattern_buffer[sampler_index] = pattern;
					sync_buffer[sampler_index] = pulse;
				} else if (T1SMIN < pulse && pulse < T1SMAX) {
					bits = 28;
					pattern = 2;
					pattern_buffer[sampler_index] = pattern;
					sync_buffer[sampler_index] = pulse;
				} else if (T4SMIN < pulse && pulse < T4SMAX) {
					bits = 24;
					pattern = 3;
					pattern_buffer[sampler_index] = pattern;
					sync_buffer[sampler_index] = pulse;
				} else if (T2S1MIN < pulse && pulse < T2S1MAX) {
					bits = 64;
					pattern = 4;
					pattern_buffer[sampler_index] = pattern;
					sync_buffer[sampler_index] = pulse;
				}
			} else {
				// HIGH sync pulses
				if (850 < pulse && pulse < 1000) {
					bits = 32;
					pattern = 99;
					pattern_buffer[sampler_index] = pattern;
					sync_buffer[sampler_index] = pulse;
				}
			}
		}
	}

	return (void *) 0;
}

static void* decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	while (1) {
		sleep(2);

		if (decoder_index == sampler_index)
			continue; // nothing received

		int todo = (sampler_index > decoder_index) ? (sampler_index - decoder_index) : ((BUFFER - decoder_index) + sampler_index);
		printf("DECODER [%d/%d] processing %i %s\n", decoder_index, sampler_index, todo, todo == 1 ? "code" : "codes");

		// process till overflow
		if (decoder_index > sampler_index)
			while (decoder_index > sampler_index)
				decode(decoder_index++);

		// follow sampler_index
		while (decoder_index < sampler_index)
			decode(decoder_index++);

	}

	return (void *) 0;
}

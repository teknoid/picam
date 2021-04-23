/***
 *
 * sampling and decoding 433MHz messages
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

#define GPIO_PIN			27	//RPi2 Pin13 GPIO_GEN2

#define BUFFER				256

#define STATE_RESET			129 // max 64 bit code -> 128 high/low edges + 1

#define PULSE_COUNTER_MAX	BUFFER * BUFFER

static pthread_t thread_decoder;
static void* decoder(void *arg);

static pthread_t thread_sampler;
static void* sampler(void *arg);

// ring buffers
static unsigned char sampler_index, decoder_index = 0;
static unsigned char pattern_buffer[BUFFER];
static unsigned long sync_buffer[BUFFER];
static unsigned long long code_buffer[BUFFER];

// pulse counters: index equals pulse length, e.g. 100µs=1, 200µs=2, etc
static unsigned long lpulse_counter[PULSE_COUNTER_MAX];
static unsigned long hpulse_counter[PULSE_COUNTER_MAX];

// default configuration
static unsigned long sync_min = 1800;
static unsigned long sync_max = 2000;
static unsigned long bitdivider = 1500;
static unsigned char decoder_delay = 3;
static unsigned char sync_on_pulse = 0;
static unsigned char bits_to_sample = 32;
static unsigned char pulse_counter_active = 0;
static unsigned char analyzer_mode = 0;

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

static void usage() {
	printf("Usage: rfsniffer [-a] [-c] [-d X] [-l] [-h] [-x X] [-y X] [-z X]\n");
	printf("  void  normal mode, sync on known pulses, sample and decode messages\n");
	printf("  -a    analyzer mode\n");
	printf("  -b X  bits to sample (0 - 64) default 32\n");
	printf("  -c    activate pulse counter\n");
	printf("  -d X  decoder delay in seconds, default 3\n");
	printf("  -l    analyzer: sync on LOW pulses (default)\n");
	printf("  -h    analyzer: sync on HIGH pulses\n");
	printf("  -x X  analyzer: sync pulse min length in microseconds\n");
	printf("  -y X  analyzer: sync pulse max length in microseconds\n");
	printf("  -z X  analyzer: 0/1 bit divider pulse length in microseconds\n");
}

int main(int argc, char **argv) {

	// test
	printf("test 2x 0xdeadbeef = %s\n", printbits64(0xdeadbeefdeadbeef, 0x0101010101010101));

	// parse command line arguments
	if (argc > 0) {
		int c;
		while ((c = getopt(argc, argv, "ab:cd:lhx:y:z:")) != -1) {
			switch (c) {
			case 'a':
				analyzer_mode = 1;
				break;
			case 'b':
				bits_to_sample = atoi(optarg);
				break;
			case 'c':
				pulse_counter_active = 1;
				break;
			case 'd':
				decoder_delay = atoi(optarg);
				break;
			case 'l':
				sync_on_pulse = 0;
				break;
			case 'h':
				sync_on_pulse = 1;
				break;
			case 'x':
				sync_min = strtoul(optarg, NULL, 0);
				break;
			case 'y':
				sync_max = strtoul(optarg, NULL, 0);
				break;
			case 'z':
				bitdivider = strtoul(optarg, NULL, 0);
				break;
			default:
				usage();
				return EXIT_FAILURE;
			}
		}
	}

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

static void decode_anaylzer(unsigned char i) {
	unsigned long long code_raw = code_buffer[i];
	unsigned long sync = sync_buffer[i];
	printf("ANALYZER sync %04lu 0x%08llx", sync, code_raw);
	printf(" %s\n", printbits64(code_raw, 0x0101010101010101));
}

static void decode(unsigned char index) {
	unsigned long long code_raw = code_buffer[index];
	if (!code_raw)
		return;

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
	case 255:
		return decode_anaylzer(index);
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

	// gpio pin setup (not working - use wiringpi gpio program)
//	bcm2835_gpio_fsel(GPIO_PIN, BCM2835_GPIO_FSEL_INPT);
//	bcm2835_gpio_set_pud(GPIO_PIN, BCM2835_GPIO_PUD_UP);
//	bcm2835_gpio_fen(GPIO_PIN);
//	bcm2835_gpio_ren(GPIO_PIN);

	char command[BUFFER];
	sprintf(command, "/usr/bin/gpio edge %d both", GPIO_PIN);
	system(command);

	// poll setup
	char buf[32];
	snprintf(buf, 32, "/sys/class/gpio/gpio%d/value", GPIO_PIN);
	struct pollfd fdset[1];
	fdset[0].fd = open(buf, O_RDONLY);
	fdset[0].events = POLLPRI;
	fdset[0].revents = 0;

	struct timeval tNow, tLast;
	unsigned long long code;
	unsigned long pulse, p1, p2;
	unsigned int pulse_counter;
	unsigned char state, state_reset, bits, pin, dummy;

	// initialize tLast for correct 1st pulse calculation
	gettimeofday(&tLast, NULL);

	state = 0;
	if (analyzer_mode) {
		state = 254;
		printf("ANALYZER sync on %lu-%luµs %s pulses, sampling %d bits, 0/1 divider pulse length %luµs\n", sync_min, sync_max, sync_on_pulse ? "HIGH" : "LOW", bits_to_sample,
				bitdivider);
	}

	while (1) {
		// wait for interrupt
		poll(fdset, 1, 33333);

		// sample time + pin state
		gettimeofday(&tNow, NULL);
		pin = bcm2835_gpio_lev(GPIO_PIN);

		// calculate length of last pulse, store timer value for next calculation
		pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
		tLast.tv_sec = tNow.tv_sec;
		tLast.tv_usec = tNow.tv_usec;

		// rewind & clear for next poll
		lseek(fdset[0].fd, 0, SEEK_SET);
		read(fdset[0].fd, &dummy, 1);

		if (analyzer_mode || pulse_counter_active) {

			// round pulse length to multiples of 100
			p1 = pulse % 100;
			p2 = p1 >= 50 ? (pulse - p1 + 100) : (pulse - p1);
			pulse_counter = p2 / 100;

			// update pulse counters: 100th pulse length is the array index
			if (pulse_counter < PULSE_COUNTER_MAX) {
				if (pin)
					lpulse_counter[pulse_counter]++;
				else
					hpulse_counter[pulse_counter]++;
			} else {
				printf("warning: pulse_counter overflow %d\n", pulse_counter);
			}

		} else {

			// ignore noise & crap
			if (pulse < 222)
				continue;
			if (pulse > 22222)
				continue;

			// error detection
			if (--state_reset == 0) {
				printf("sampling %d aborted\n", state);
				state = 0;
				continue;
			}
		}

		switch (state) {

		case 0: // initial state: detect known SYNC pulses and define next sampling state
			code = 0;
			state_reset = STATE_RESET;

			if (pin) {
				// LOW sync pulses
				if (3800 < pulse && pulse < 4000) {
					// not a real sync - it's the pause between 1st and 2nd message
					bits = 36;
					state = 1;
					pattern_buffer[sampler_index] = state;
					sync_buffer[sampler_index] = pulse;
				} else if (T1SMIN < pulse && pulse < T1SMAX) {
					bits = 28;
					state = 2;
					pattern_buffer[sampler_index] = state;
					sync_buffer[sampler_index] = pulse;
				} else if (T4SMIN < pulse && pulse < T4SMAX) {
					bits = 24;
					state = 3;
					pattern_buffer[sampler_index] = state;
					sync_buffer[sampler_index] = pulse;
				} else if (T2S1MIN < pulse && pulse < T2S1MAX) {
					bits = 64;
					state = 4;
					pattern_buffer[sampler_index] = state;
					sync_buffer[sampler_index] = pulse;
				}
			} else {
				// HIGH sync pulses
			}
			break;

		case 1: // nexus - sample LOW pulses
			if (bits && pin) {
				if (pulse > 1500)
					code++;

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					state = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 2: // flamingo 28bit - sample HIGH pulses
			if (bits && !pin) {
				if (pulse > T1X2)
					code++;

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					state = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 3: // flamingo 24bit - sample HIGH pulses
			if (bits && !pin) {
				if (pulse > T1X2)
					code++;

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					state = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 4: // flamingo 32bit - sample LOW pulses
			if (bits && pin) {
				code += pulse < T2Y ? 0 : 1;
				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					state = 0;
				} else {
					code <<= 1;
				}
			}
			break;

		case 254: // analyzer sync mode: find the SYNC pulse
			code = 0;
			state_reset = STATE_RESET;

			if (pin) {
				// LOW sync pulses
				if (sync_on_pulse == 0) {
					if (sync_min < pulse && pulse < sync_max) {
						bits = bits_to_sample;
						state = 255; // next is analyzer sample mode
						pattern_buffer[sampler_index] = state;
						sync_buffer[sampler_index] = pulse;
						printf("\nS ");
					}
				}
			} else {
				// HIGH sync pulses
				if (sync_on_pulse == 1) {
					if (sync_min < pulse && pulse < sync_max) {
						bits = bits_to_sample;
						state = 255; // next is analyzer sample mode
						pattern_buffer[sampler_index] = state;
						sync_buffer[sampler_index] = pulse;
						printf("\nS ");
					}
				}
			}
			break;

		case 255: // analyzer sample mode: sample bits and print
			if (bits) {
				if (pulse > bitdivider)
					code++;

				// print
				if (pin)
					printf("L%02d ", pulse_counter);
				else
					printf("H%02d ", pulse_counter);

				if (--bits == 0) {
					code_buffer[sampler_index++] = code;
					state = 254; // back to analyzer sync mode
					printf("\n");
				} else {
					code <<= 1;
				}
			}
			break;

		default:
			printf("illegal state %d\n", state);
			state = 0;
		}
	}

	return (void *) 0;
}

static void* decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	unsigned long lmax, hmax;
	unsigned char lmaxi, hmaxi;

	printf("DECODER run every %d seconds\n", decoder_delay);

	while (1) {
		sleep(decoder_delay);

		// print pulse counters and top ten pulse lengths, then clear counters
		if (pulse_counter_active) {
			printf("\nLCOUNTER up to 9900µs\n");
			for (int i = 0; i < 100; i++) {
				printf("%02d   ", i);
			}
			printf("\n");
			for (int i = 0; i < 100; i++) {
				printf("%04lu ", lpulse_counter[i]);
			}
			printf("\n\nHCOUNTER up to 9900µs\n");
			for (int i = 0; i < 100; i++) {
				printf("%02d   ", i);
			}
			printf("\n");
			for (int i = 0; i < 100; i++) {
				printf("%04lu ", hpulse_counter[i]);
			}
			printf("\n\nTOPTEN (without noise)\n");
			lpulse_counter[0] = hpulse_counter[0] = 0;	// ignore pulses below 100µs
			lpulse_counter[1] = hpulse_counter[1] = 0;	// ignore pulses below 200µs
			hpulse_counter[2] = 0; 						// ignore HIGH pulses below 300µs
			for (int m = 0; m < 10; m++) {
				hmax = lmax = hmaxi = lmaxi = 0;
				for (int i = 0; i < BUFFER; i++) {
					if (lpulse_counter[i] > lmax) {
						lmax = lpulse_counter[i];
						lmaxi = i;
					}
					if (hpulse_counter[i] > hmax) {
						hmax = hpulse_counter[i];
						hmaxi = i;
					}
				}
				printf("L%03d %04lu   H%03d %04lu\n", lmaxi, lmax, hmaxi, hmax);
				lpulse_counter[lmaxi] = 0;
				hpulse_counter[hmaxi] = 0;
			}
			for (int i = 0; i < 100; i++) {
				lpulse_counter[i] = 0;
				hpulse_counter[i] = 0;
			}
		}

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

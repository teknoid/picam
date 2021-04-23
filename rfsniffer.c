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
#include "frozen.h"

#define GPIO_PIN			27	//RPi2 Pin13 GPIO_GEN2

#define BUFFER				256

#define STATE_RESET			129 // max 64 bit code -> 128 high/low edges + 1

#define PULSE_COUNTER_MAX	BUFFER * BUFFER

const static char *CLOW = "LOW\0";
const static char *CHIGH = "HIGH\0";
const static char *CEDGE = "EDGE\0";
const static char *CNONE = "NONE\0";

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
static unsigned char json = 0;
static unsigned char decoder_delay = 3;
static unsigned char sync_on_0 = 1;
static unsigned char sync_on_1 = 0;
static unsigned char sample_on_0 = 1;
static unsigned char sample_on_1 = 0;
static unsigned char bits_to_sample = 32;
static unsigned char pulse_counter_active = 0;
static unsigned char analyzer_mode = 0;
static unsigned char verbose = 0;

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
	printf("Usage: rfsniffer -v -j -c -dX -a -bX -sX -SX -xX -yX -zX\n");
	printf("  void  normal mode, sync on known pulses, sample and decode messages\n");
	printf("  -v    verbose output\n");
	printf("  -j    print messages as JSON\n");
	printf("  -c    activate pulse length counter\n");
	printf("  -d X  decoder delay in seconds, default 3\n");
	printf("  -a    analyzer mode\n");
	printf("  -b X  analyzer: bits to sample (0 - 64) default 32\n");
	printf("  -s X  analyzer: sync on pulse 0=LOW (default) 1=HIGH 2=EDGE)\n");
	printf("  -S X  analyzer: sample on pulse 0=LOW (default) 1=HIGH 2=EDGE)\n");
	printf("  -x X  analyzer: sync pulse min length in microseconds\n");
	printf("  -y X  analyzer: sync pulse max length in microseconds\n");
	printf("  -z X  analyzer: 0/1 bit divider pulse length in microseconds\n");
}

int main(int argc, char **argv) {

	// test
	if (verbose)
		printf("MAIN test 2x 0xdeadbeef = %s\n", printbits64(0xdeadbeefdeadbeef, 0x0101010101010101));

	// parse command line arguments
	if (argc > 0) {
		int c, i;
		while ((c = getopt(argc, argv, "ab:cd:js:S:vx:y:z:")) != -1) {
			switch (c) {
			case 'a':
				analyzer_mode = 1;
				verbose = 1;
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
			case 'j':
				json = 1;
				break;
			case 's':
				i = atoi(optarg);
				switch (i) {
				case 0:
					sync_on_0 = 1;
					sync_on_1 = 0;
					break;
				case 1:
					sync_on_0 = 0;
					sync_on_1 = 1;
					break;
				case 2:
					sync_on_0 = 1;
					sync_on_1 = 1;
					break;
				}
				break;
			case 'S':
				i = atoi(optarg);
				switch (i) {
				case 0:
					sample_on_0 = 1;
					sample_on_1 = 0;
					break;
				case 1:
					sample_on_0 = 0;
					sample_on_1 = 1;
					break;
				case 2:
					sample_on_0 = 1;
					sample_on_1 = 1;
					break;
				}
				break;
			case 'v':
				verbose = 1;
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
	if (verbose)
		printf("MAIN started decoder thread\n");

	// sampler thread
	if (pthread_create(&thread_sampler, NULL, &sampler, NULL)) {
		perror("Error creating sampler thread");
		return -1;
	}
	if (verbose)
		printf("MAIN started sampler thread\n");

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
			if (verbose)
				printf("0110 decode error %s\n", printbits64(in, 0x0001000100010001));
			return 0;
		}
		count -= 2;
		in <<= 2;
	}
	// printf("--> %s\n", printbits(out, 0x00010001));
	return out;
}

static void decode_nexus(unsigned char i) {
	unsigned long long raw = code_buffer[i];
	unsigned long sync = sync_buffer[i];

	if ((raw & 0x0f00) != 0x0f00) {
		if (verbose)
			printf("NEXUS message verification failed 0x%08llx = %s\n", raw, printbits64(raw, 0x1011001101));
		return;
	}

	// https://github.com/merbanan/rtl_433/blob/master/src/devices/nexus.c
	// 9 nibbles: [id0] [id1] [flags] [temp0] [temp1] [temp2] [const] [humi0] [humi1]
	unsigned long long code = raw;
	unsigned char hum = code & 0xff;
	code >>= 8;
	code >>= 4; // always 1111 - used for message verification
	float temp = 0.1 * (code & 0x0fff);
	code >>= 12;
	unsigned char channel = code & 0x07;
	code >>= 3;
	unsigned char battery = code & 0x01;
	code >>= 1;
	unsigned char id = code & 0xff;

	if (json) {
		char format[BUFFER], craw[12], ctemp[6];
		snprintf(craw, 12, "0x%08llx", raw);
		snprintf(ctemp, 12, "%02.1f", temp);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%d, %Q:%d, %Q:%d, %Q:%Q, %Q:%d }\n");
		json_printf(&jout, format, "type", "NEXUS", "raw", craw, "sync", sync, "id", id, "channel", channel, "battery", battery, "temp", ctemp, "hum", hum);
	} else
		printf("NEXUS (%lu, 0x%08llx) Id = %d, Channel = %d, Battery = %s, Temperature = %02.1fC, Humidity = %d%%\n", sync, raw, id, channel, battery ? "OK" : "LOW", temp, hum);
}

static void decode_flamingo28(unsigned char i) {
	unsigned long raw = (unsigned long) code_buffer[i];
	unsigned long sync = sync_buffer[i];

	unsigned long message;
	unsigned int xmitter;
	unsigned char channel, command, payload, rolling;

	message = decrypt(raw);
	decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);

	if (json) {
		char format[BUFFER], craw[12], cid[12];
		snprintf(craw, 12, "0x%04lx", raw);
		snprintf(cid, 12, "0x%04x", xmitter);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%Q, %Q:%d, %Q:%d, %Q:%d, %Q:%d }\n");
		json_printf(&jout, format, "type", "FLAMINGO28", "raw", craw, "sync", sync, "id", cid, "channel", channel, "command", command, "payload", payload, "rolling", rolling);
	} else
		printf("FLAMINGO28 (%lu, 0x%04lx) Id = 0x%x, Channel = %d, Command = %d, Payload = 0x%02x, Rolling = %d\n", sync, raw, xmitter, channel, command, payload, rolling);
}

static void decode_flamingo24(unsigned char i) {
	unsigned long raw = (unsigned long) code_buffer[i];
	unsigned long sync = sync_buffer[i];

	// TODO decode

	if (json) {
		char format[BUFFER], craw[12];
		snprintf(craw, 12, "0x%04lx", raw);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu }\n");
		json_printf(&jout, format, "type", "FLAMINGO24", "raw", craw, "sync", sync);
	} else
		printf("FLAMINGO24 (%lu, 0x%04lx) %s\n", sync, raw, printbits(raw, 0x01010101));
}

static void decode_flamingo32(unsigned char i) {
	unsigned long long raw = code_buffer[i];
	unsigned long sync = sync_buffer[i];

	unsigned long code = decode_0110(raw);
	if (code == 0)
		return;

	unsigned int xmitter;
	unsigned char channel, command, payload;
	unsigned long code_save = code;
	channel = code & 0x0f;
	code >>= 4; // channel
	command = code & 0x0f;
	code >>= 4; // channel
	xmitter = code & 0xffff;
	code >>= 16; // xmitter
	payload = code & 0xff;

	if (json) {
		char format[BUFFER], craw[12], cid[12];
		snprintf(craw, 12, "0x%04lx", code_save);
		snprintf(cid, 12, "0x%04x", xmitter);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%Q, %Q:%d, %Q:%d, %Q:%d }\n");
		json_printf(&jout, format, "type", "FLAMINGO32", "raw", craw, "sync", sync, "id", cid, "channel", channel, "command", command, "payload", payload);
	} else
		printf("FLAMINGO32 (%lu, 0x%04lx) Id = 0x%x, Channel = %d, Command = %d, Payload = 0x%02x\n", sync, code_save, xmitter, channel, command, payload);
}

static void decode_anaylzer(unsigned char i) {
	unsigned long long raw = code_buffer[i];
	unsigned long sync = sync_buffer[i];
	printf("ANALYZER (%lu, 0x%08llx) %s\n", sync, raw, printbits64(raw, 0x0101010101010101));
}

static void decode(unsigned char index) {
	unsigned long long code_raw = code_buffer[index];
	if (!code_raw) {
		if (verbose)
			printf("DECODER received empty message\n");
		return;
	}

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
		const char *csync, *csample;
		if (sync_on_0 && sync_on_1)
			csync = CEDGE;
		else if (sync_on_0 && !sync_on_1)
			csync = CLOW;
		else if (!sync_on_0 && sync_on_1)
			csync = CHIGH;
		else
			csync = CNONE;

		if (sample_on_0 && sample_on_1)
			csample = CEDGE;
		else if (sample_on_0 && !sample_on_1)
			csample = CLOW;
		else if (!sample_on_0 && sample_on_1)
			csample = CHIGH;
		else
			csample = CNONE;

		printf("ANALYZER sync on %lu-%luµs %s pulses, sampling %d %s bits, 0/1 divider pulse length %luµs\n", sync_min, sync_max, csync, bits_to_sample, csample, bitdivider);
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
				if (verbose)
					printf("sampling state %d aborted\n", state);
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
				if (sync_on_0) {
					if (sync_min < pulse && pulse < sync_max) {
						bits = bits_to_sample;
						state = 255; // next is analyzer sample mode
						pattern_buffer[sampler_index] = state;
						sync_buffer[sampler_index] = pulse;
						printf("\nSYN ");
					}
				}
			} else {
				// HIGH sync pulses
				if (sync_on_1) {
					if (sync_min < pulse && pulse < sync_max) {
						bits = bits_to_sample;
						state = 255; // next is analyzer sample mode
						pattern_buffer[sampler_index] = state;
						sync_buffer[sampler_index] = pulse;
						printf("\nSYN ");
					}
				}
			}
			break;

		case 255: // analyzer sample mode: sample bits and print
			if (bits) {
				if (pin) {
					// LOW sync pulses
					printf("L%02d ", pulse_counter);
					if (sample_on_0) {
						if (pulse > bitdivider)
							code++;
						if (--bits == 0) {
							code_buffer[sampler_index++] = code;
							state = 254; // back to analyzer sync mode
							printf("\n");
						} else {
							code <<= 1;
						}
					}
				} else {
					// HIGH sync pulses
					printf("H%02d ", pulse_counter);
					if (sample_on_1) {
						if (pulse > bitdivider)
							code++;
						if (--bits == 0) {
							code_buffer[sampler_index++] = code;
							state = 254; // back to analyzer sync mode
							printf("\n");
						} else {
							code <<= 1;
						}
					}
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

	if (verbose)
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
			printf("\n\nTOP-TEN (without noise)\n");
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
		if (verbose)
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

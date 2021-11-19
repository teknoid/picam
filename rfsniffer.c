/***
 *
 * sampling and decoding 433MHz messages

 * unfortunately this works only if the pi is idle
 * especially there should be no USB traffic during sampling
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <syslog.h>

//#include <bcm2835.h>
#include <wiringPi.h>

#include "rfsniffer.h"
#include "flamingo.h"
#include "utils.h"

// #define RFSNIFFER_MAIN

// #define RX				247
// #define TX				219
// #define GPIO_PIN			247	//OdroidC2

#define RX					2
#define TX					0
#define GPIO_PIN			27	//RPi2 Pin13 GPIO_GEN2

#define BUFFER				0xff

#define STATE_RESET			129 // max 64 bit code -> 128 high/low edges + 1

#define PULSE_COUNTER_MAX	BUFFER * BUFFER

const static char *CLOW = "LOW\0";
const static char *CHIGH = "HIGH\0";
const static char *CEDGE = "EDGE\0";
const static char *CNONE = "NONE\0";

static pthread_t thread_decoder;
static void* realtime_decoder(void *arg);
static void* stream_decoder(void *arg);

static pthread_t thread_sampler;
static void* realtime_sampler(void *arg);
static void* stream_sampler(void *arg);

// stream_decoder: ring buffers
static unsigned short lstream[0xffff], hstream[0xffff];
static unsigned short stream_write, stream_read;

// code matrix: x=protocol code, y=code number, [x][0] contains next free entry
static unsigned long long matrix[BUFFER][BUFFER];

// pulse counters: index equals pulse length, e.g. 10µs=1, 20µs=2, etc
static unsigned short lpulse_counter[PULSE_COUNTER_MAX];
static unsigned short hpulse_counter[PULSE_COUNTER_MAX];

// realtime_decoder: calculate age of last message
static unsigned long timestamp_last_code;

static rfsniffer_config_t *cfg;

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

// TODO
// - timestamp

#ifdef RFSNIFFER_MAIN
static int usage() {
	printf("Usage: rfsniffer -v -q -jX -fX -c -nX -dX -a -r -bX -sX -SX -xX -yX -zX\n");
	printf("  void  normal mode, sync on known pulses, sample and decode messages\n");
	printf("  -v    verbose console output\n");
	printf("  -q    no console output except errors\n");
	printf("  -j X  create JSON file\n");
	printf("  -f X  create sysfs-like entries in specified directory\n");
	printf("  -t    add timestamp to messages\n");
	printf("  -c    activate pulse length counter\n");
	printf("  -n X  pulse length counter noise level (default 100µs)\n");
	printf("  -e    process each single code (default collect)\n");
	printf("  -d X  decoder delay in seconds, default 1\n");
	printf("  -a    analyzer mode\n");
	printf("  -r    realtime mode\n");
	printf("  -b X  analyzer: bits to sample (0 - 64) default 32\n");
	printf("  -s X  analyzer: sync on pulse 0=LOW (default) 1=HIGH 2=EDGE)\n");
	printf("  -S X  analyzer: sample on pulse 0=LOW (default) 1=HIGH 2=EDGE)\n");
	printf("  -x X  analyzer: sync pulse min length in microseconds/10\n");
	printf("  -y X  analyzer: sync pulse max length in microseconds/10\n");
	printf("  -z X  analyzer: 0/1 bit divider pulse length in microseconds/10\n");
	return EXIT_FAILURE;
}

static int rfsniffer_main(int argc, char **argv) {
	// return rfcodec_test();

	// if (!bcm2835_init())
	if (wiringPiSetup() < 0)
		return -1;

	// gain RT
	if (elevate_realtime(3) < 0)
		return -2;

	// initialize a default configuration
	cfg = rfsniffer_default_config();

	if (argc > 0) {
		int c, i;
		while ((c = getopt(argc, argv, "ab:cd:ef:jn:qrs:S:tvx:y:z:")) != -1) {
			switch (c) {
			case 'a':
				cfg->analyzer_mode = 1;
				break;
			case 'b':
				cfg->bits_to_sample = atoi(optarg);
				break;
			case 'c':
				cfg->pulse_counter_active = 1;
				break;
			case 'd':
				cfg->decoder_delay = atoi(optarg);
				break;
			case 'e':
				cfg->collect_identical_codes = 0;
				break;
			case 'f':
				cfg->sysfslike = optarg;
				break;
			case 'j':
				cfg->json = 1;
				break;
			case 'n':
				cfg->noise = atoi(optarg);
				break;
			case 'q':
				cfg->quiet = 1;
				break;
			case 'r':
				cfg->realtime_mode = 1;
				break;
			case 's':
				i = atoi(optarg);
				switch (i) {
				case 0:
					cfg->sync_on_0 = 1;
					cfg->sync_on_1 = 0;
					break;
				case 1:
					cfg->sync_on_0 = 0;
					cfg->sync_on_1 = 1;
					break;
				case 2:
					cfg->sync_on_0 = 1;
					cfg->sync_on_1 = 1;
					break;
				}
				break;
			case 'S':
				i = atoi(optarg);
				switch (i) {
				case 0:
					cfg->sample_on_0 = 1;
					cfg->sample_on_1 = 0;
					break;
				case 1:
					cfg->sample_on_0 = 0;
					cfg->sample_on_1 = 1;
					break;
				case 2:
					cfg->sample_on_0 = 1;
					cfg->sample_on_1 = 1;
					break;
				}
				break;
			case 't':
				cfg->timestamp = 1; // TODO
				break;
			case 'v':
				cfg->verbose = 1;
				break;
			case 'x':
				cfg->sync_min = strtoul(optarg, NULL, 0);
				break;
			case 'y':
				cfg->sync_max = strtoul(optarg, NULL, 0);
				break;
			case 'z':
				cfg->bitdivider = strtoul(optarg, NULL, 0);
				break;
			default:
				return usage();
			}
		}
	}

	rfsniffer_init();
	pause();
	rfsniffer_close();
	return 0;
}
#endif

static void matrix_decode_protocol(unsigned char x) {
	unsigned long long code;
	unsigned char repeat, y = 1;

	if (cfg->collect_identical_codes) {
		// TODO find ALL identical codes, not only when differs
		code = matrix[x][y];
		repeat = 0;
		do {
			if (code == matrix[x][y])
				repeat++; // still the same code
			else {
				rfcodec_decode(x, code, repeat);
				code = matrix[x][y];
				repeat = 1; // new code
			}
		} while (++y < matrix[x][0]);
		// all identical or the last one
		rfcodec_decode(x, code, repeat);
	} else {
		do {
			code = matrix[x][y];
			rfcodec_decode(x, code, 0);
		} while (++y < matrix[x][0]);
	}
}

static void matrix_decode() {
	unsigned char x, y, filled = 0;

	// check if empty
	for (x = 0; x < BUFFER; x++) {
		y = matrix[x][0];
		if (y > 1)
			filled = 1;
	}
	if (!filled)
		return;

	// print summary
	if (cfg->verbose) {
		printf("MATRIX ");
		for (x = 0; x < BUFFER; x++) {
			y = matrix[x][0];
			if (y > 1)
				printf("[%u:%u] ", x, y - 1);
		}
		printf("\n");
	}

	// decode a column
	for (x = 0; x < BUFFER; x++) {
		y = matrix[x][0];
		if (y > 1)
			matrix_decode_protocol(x);
		matrix[x][0] = 1; // initialize next free entry
	}
}

static void matrix_store(unsigned char x, unsigned long long code) {
	unsigned char ynext = matrix[x][0];
	matrix[x][ynext] = code;
	matrix[x][0]++; // increment next free entry
}

static void matrix_init() {
	memset(matrix, 0, sizeof(matrix));
	for (unsigned char i = 0; i < BUFFER; i++) {
		matrix[i][0] = 1; // initialize next free entry
	}
}

static void dump_pulse_counters() {
	printf("\nLCOUNTER up to 2000µs\n");
	for (int i = 0; i < 200; i++) {
		printf("%02d   ", i);
	}
	printf("\n");
	for (int i = 0; i < 200; i++) {
		printf("%04u ", lpulse_counter[i]);
	}
	printf("\n\nHCOUNTER up to 2000µs\n");
	for (int i = 0; i < 200; i++) {
		printf("%02d   ", i);
	}
	printf("\n");
	for (int i = 0; i < 200; i++) {
		printf("%04u ", hpulse_counter[i]);
	}

	// calculate top ten pulse lengths
	unsigned long lmax, hmax;
	unsigned char lmaxi, hmaxi, i;
	for (i = 0; i < (cfg->noise / 10); i++) {
		lpulse_counter[i] = hpulse_counter[i] = 0;	// ignore noise pulses
	}
	printf("\n\nTOP-TEN (noise level = %uµs)\n", cfg->noise);
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

	// clear counter
	memset(lpulse_counter, 0, sizeof(lpulse_counter));
	memset(hpulse_counter, 0, sizeof(hpulse_counter));
}

static void* realtime_decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	if (!cfg->quiet)
		printf("DECODER run every %d seconds, %s\n", cfg->decoder_delay, cfg->collect_identical_codes ? "collect identical codes" : "process each code separately");

	while (1) {
		sleep(cfg->decoder_delay);

		// repeating transmission: wait until actual message is minimum 500ms old
		unsigned long age_last_message = _micros() - timestamp_last_code;
		while (age_last_message < 500) {
			msleep(100);
			age_last_message = _micros() - timestamp_last_code;
			if (cfg->verbose)
				printf("DECODER receiving in progress %lums\n", age_last_message);
		}

		// print pulse counters,  top ten pulse lengths, clear counters
		if (cfg->pulse_counter_active) {
			dump_pulse_counters();
		}

		matrix_decode();
	}
	return (void *) 0;
}

static void* realtime_sampler(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	// elevate realtime priority
	if (elevate_realtime(3) < 0)
		return (void *) 0;

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

	unsigned long long code;
	unsigned long pulse, tnow, tlast, divider, p1, p2;
	unsigned int pulse_counter;
	unsigned char protocol, state_reset, bits, pin, dummy;
	short state;

	// initialize tLast for correct 1st pulse calculation
	tlast = _micros();

	// initialize the matrix
	matrix_init();

	state = -1;
	if (cfg->analyzer_mode) {
		state = -2;

		const char *csync, *csample;
		if (cfg->sync_on_0 && cfg->sync_on_1)
			csync = CEDGE;
		else if (cfg->sync_on_0 && !cfg->sync_on_1)
			csync = CLOW;
		else if (!cfg->sync_on_0 && cfg->sync_on_1)
			csync = CHIGH;
		else
			csync = CNONE;

		if (cfg->sample_on_0 && cfg->sample_on_1)
			csample = CEDGE;
		else if (cfg->sample_on_0 && !cfg->sample_on_1)
			csample = CLOW;
		else if (!cfg->sample_on_0 && cfg->sample_on_1)
			csample = CHIGH;
		else
			csample = CNONE;

		if (cfg->verbose) {
			const char *fmt = "SAMPLER sync on %lu-%luµs %s pulses, sampling %d %s bits, 0/1 divider pulse length %luµs\n";
			printf(fmt, cfg->sync_min, cfg->sync_max, csync, cfg->bits_to_sample, csample, cfg->bitdivider);
		}
	}

	while (1) {
		// wait for interrupt
		poll(fdset, 1, -1);

		// sample time + pin state
		tnow = _micros();
		// pin = bcm2835_gpio_lev(GPIO_PIN);
		pin = digitalRead(cfg->rx);

		// rewind & clear for next poll
		lseek(fdset[0].fd, 0, SEEK_SET);
		read(fdset[0].fd, &dummy, 1);

		// calculate pulse length
		pulse = tnow - tlast;
		tlast = tnow;

		// ignore noise & crap
		if (pulse < cfg->noise)
			continue;
		if (pulse > 22222)
			continue;

		if (cfg->analyzer_mode || cfg->pulse_counter_active) {

			// round pulse length to multiples of 10
			p1 = pulse % 10;
			p2 = p1 >= 5 ? (pulse - p1 + 10) : (pulse - p1);
			pulse_counter = p2 / 10;

			// update pulse counters: 10th pulse length is the array index
			if (pulse_counter < PULSE_COUNTER_MAX) {
				if (pin)
					lpulse_counter[pulse_counter]++;
				else
					hpulse_counter[pulse_counter]++;
			} else if (cfg->verbose)
				printf("SAMPLER pulse_counter overflow %d\n", pulse_counter);

		} else {

			// error detection
			if (--state_reset == 0) {
				state = -1;
				if (cfg->verbose)
					printf("SAMPLER sampling on protocol %d aborted\n", state);
				continue;
			}
		}

		switch (state) {

		case -1: // initial state: detect known SYNC pulses and define next sampling state
			code = 0;
			state_reset = STATE_RESET;

			if (pin) {
				// LOW sync pulses
				if (3800 < pulse && pulse < 4000) {
					// not a real sync - it's the pause between 1st and 2nd message
					protocol = P_NEXUS;
					bits = 36;
					state = 0; // LOW pulses
					divider = 1500;
				} else if (T1SMIN < pulse && pulse < T1SMAX) {
					protocol = P_FLAMINGO28;
					bits = 28;
					state = 1; // HIGH pulses
					divider = T1X2;
				} else if (T4SMIN < pulse && pulse < T4SMAX) {
					bits = 24;
					protocol = P_FLAMINGO24;
					state = 1; // HIGH pulses
					divider = T1X2;
				} else if (2600 < pulse && pulse < 2800) {
					bits = 64;
					protocol = P_FLAMINGO32;
					state = 0; // LOW pulses
					divider = T2Y;
				}
			} else {
				// HIGH sync pulses
			}
			break;

		case 0: // sample LOW pulses
			if (bits)
				if (pin) {
					if (pulse > divider)
						code++;
					if (--bits == 0) {
						matrix_store(protocol, code);
						timestamp_last_code = tnow;
						state = -1;
					} else
						code <<= 1;
				}
			break;

		case 1: // sample HIGH pulses
			if (bits)
				if (!pin) {
					if (pulse > divider)
						code++;
					if (--bits == 0) {
						matrix_store(protocol, code);
						timestamp_last_code = tnow;
						state = -1;
					} else
						code <<= 1;
				}
			break;

		case -2: // analyzer sync mode: find the SYNC pulse
			code = 0;
			state_reset = STATE_RESET;

			if (pin) {
				// LOW pulse
				if (cfg->sync_on_0)
					if (cfg->sync_min < pulse && pulse < cfg->sync_max) {
						bits = cfg->bits_to_sample;
						state = 127; // next is analyzer sample mode
						printf("\nSYN ");
					}

			} else {
				// HIGH pulse
				if (cfg->sync_on_1)
					if (cfg->sync_min < pulse && pulse < cfg->sync_max) {
						bits = cfg->bits_to_sample;
						state = 127; // next is analyzer sample mode
						printf("\nSYN ");
					}
			}
			break;

		case 127: // analyzer sample mode: sample bits and print
			if (bits) {
				if (pin) {
					// LOW pulse
					printf("L%02d ", pulse_counter);
					if (cfg->sample_on_0) {
						if (pulse > cfg->bitdivider)
							code++;
						if (--bits == 0) {
							matrix_store(P_ANALYZE, code);
							timestamp_last_code = tnow;
							state = -2; // back to analyzer sync mode
							printf("\n");
						} else
							code <<= 1;
					}
				} else {
					// HIGH pulse
					printf("H%02d ", pulse_counter);
					if (cfg->sample_on_1) {
						if (pulse > cfg->bitdivider)
							code++;
						if (--bits == 0) {
							matrix_store(P_ANALYZE, code);
							timestamp_last_code = tnow;
							state = -2; // back to analyzer sync mode
							printf("\n");
						} else
							code <<= 1;
					}
				}
			}
			break;

		default:
			printf("illegal state %d\n", state);
			state = -1;
		}
	}
	return (void *) 0;
}

static void stream_dump(unsigned short start, unsigned short positions) {
	unsigned short i, l, h;
	printf("DUMP %05u+%u :: ", start, positions);
	for (i = 0; i < positions; i++) {
		l = lstream[start];
		h = hstream[start];
		start++;
		printf("H%03u L%03u ", h, l);
	}
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
static unsigned long long stream_probe_low(unsigned short pos, unsigned char bits, unsigned short divider) {
	unsigned long long code = 0;
	unsigned long l, h;
	for (int i = 0; i < bits; i++) {
		l = lstream[pos] * 10;
		h = hstream[pos] * 10;

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
static unsigned long long stream_probe_high(unsigned short pos, unsigned char bits, unsigned short divider) {
	unsigned long long code = 0;
	unsigned long l, h;
	for (int i = 0; i < bits; i++) {
		l = lstream[pos] * 10;
		h = hstream[pos] * 10;

		if (!l && !h)
			return 0;

		if (h > divider)
			code++;

		code <<= 1;
		pos++;
	}
	return code;
}

// rewind stream_write position
static unsigned short stream_rewind(int positions) {
	unsigned short current = stream_write;
	for (int i = 0; i < positions; i++)
		current--;
	return current;
}

// stream_read follows stream_write without overtake
static void stream_consume(int positions) {
	while (positions && (stream_read != stream_write)) {
		stream_read++;
		positions--;
	}
}

static void* stream_decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	if (!cfg->quiet)
		printf("DECODER run every %d seconds, %s\n", cfg->decoder_delay, cfg->collect_identical_codes ? "collect identical codes" : "process each code separately");

	unsigned short delta, stream_last;
	unsigned short i, lshort, hshort, rewind, progress;
	unsigned long l;
	// unsigned long h;
	unsigned long long code;

	stream_read = 0;
	while (1) {
		sleep(cfg->decoder_delay);

		// if we see 8 bit valid pulses then wait (assuming receiving in progress)
		do {
			rewind = stream_rewind(8);
			for (i = 0; i < 8; i++) {
				progress = 0;
				lshort = lstream[rewind];
				if (!lshort || lshort > 222)
					break;
				hshort = hstream[rewind];
				if (!hshort || hshort > 222)
					break;
				progress = 1;
				rewind++;
			}
			if (progress) {
				if (cfg->verbose)
					printf("DECODER receiving in progress, wait\n");
				msleep(100);
			}
		} while (progress);

		delta = (stream_write > stream_last) ? stream_write - stream_last : 0xffff - stream_last + stream_write;
		if (cfg->verbose)
			printf("DECODER stream [%05u:%04u]\n", stream_write, delta);

		while (1) {

			// update pulse counters: 10th pulse length is the array index
			if (cfg->pulse_counter_active) {
				lshort = lstream[stream_read];
				hshort = hstream[stream_read];
				if (lshort < PULSE_COUNTER_MAX)
					lpulse_counter[lshort]++;
				else
					printf("SAMPLER LOW pulse_counter overflow %d\n", lshort);

				if (hshort < PULSE_COUNTER_MAX)
					hpulse_counter[hshort]++;
				else
					printf("SAMPLER HIGH pulse_counter overflow %d\n", hshort);
			}

			l = lstream[stream_read] * 10;
			// h = hstream[stream_read] * 10;
			if (3800 < l && l < 4000) {
				// not a real sync - it's the pause between 1st and 2nd message - but 1st message is always blurred
				if (cfg->verbose)
					printf("DECODER %lu SYNC on NEXUS\n", l);
				code = stream_probe_low(stream_read, 36, 1500);
				if (code) {
					matrix_store(P_NEXUS, code);
					stream_consume(36);
				}

			} else if (T1SMIN < l && l < T1SMAX) {
				if (cfg->verbose)
					printf("DECODER %lu SYNC on FLAMINGO28\n", l);
				code = stream_probe_high(stream_read, 28, T1X2);
				if (code) {
					matrix_store(P_FLAMINGO28, code);
					stream_consume(28);
				}

			} else if (T4SMIN < l && l < T4SMAX) {
				if (cfg->verbose)
					printf("DECODER %lu SYNC on FLAMINGO24\n", l);
				code = stream_probe_high(stream_read, 24, T1X2);
				if (code) {
					matrix_store(P_FLAMINGO24, code);
					stream_consume(24);
				}

			} else if (2600 < l && l < 2800) {
				if (cfg->verbose)
					printf("DECODER %lu SYNC on FLAMINGO32\n", l);
				code = stream_probe_low(stream_read, 64, T2Y);
				if (code) {
					matrix_store(P_FLAMINGO32, code);
					stream_consume(64);
				}

			} else if (cfg->sync_min < l && l < cfg->sync_max) {
				if (cfg->verbose) {
					printf("DECODER %lu SYNC on ?, ", l);
					stream_dump(stream_read, 8);
				}
				code = stream_probe_low(stream_read, cfg->bits_to_sample, cfg->bitdivider);
				if (code) {
					matrix_store(P_ANALYZE, code);
					// stream_consume(cfg->bits_to_sample);
				}
				code = stream_probe_high(stream_read, cfg->bits_to_sample, cfg->bitdivider);
				if (code) {
					matrix_store(P_ANALYZE, code);
					// stream_consume(cfg->bits_to_sample);
				}
			}

			if (stream_read++ == stream_write)
				break; // reached stream_write position
		}

		// print pulse counters,  top ten pulse lengths, clear counters
		if (cfg->pulse_counter_active)
			dump_pulse_counters();

		// process received codes
		matrix_decode();

		// store for next round
		stream_last = stream_read;
	}
	return (void *) 0;
}

static void* stream_sampler(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	// elevate realtime priority
	if (elevate_realtime(3) < 0)
		return (void *) 0;

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

	unsigned long pulse, p1, p2, tnow, tlast;
	unsigned char pin, dummy;

	// initialize tLast for correct 1st pulse calculation
	tlast = _micros();

	// initialize the matrix
	matrix_init();

	stream_write = 0;
	while (1) {
		// wait for interrupt
		poll(fdset, 1, -1);

		// sample time + pin state
		tnow = _micros();
		// pin = bcm2835_gpio_lev(GPIO_PIN);
		pin = digitalRead(cfg->rx);

		// calculate length of last pulse, store timer value for next calculation
		pulse = tnow - tlast;
		tlast = tnow;

		// rewind & clear for next poll
		lseek(fdset[0].fd, 0, SEEK_SET);
		read(fdset[0].fd, &dummy, 1);

		if (pin) {
			// LOW pulse

			// clear
			lstream[stream_write] = 0;

			// validate
			if (pulse < cfg->noise)
				continue;
			if (pulse > 655360)
				continue;

			// round pulse length to multiples of 10
			p1 = pulse % 10;
			p2 = p1 >= 5 ? (pulse - p1 + 10) : (pulse - p1);

			// sample
			lstream[stream_write] = (unsigned short) p2 / 10;
			if (cfg->sample_on_0)
				stream_write++;

		} else {
			// HIGH pulse

			// clear
			hstream[stream_write] = 0;

			// validate
			if (pulse < cfg->noise)
				continue;
			if (pulse > 655360)
				continue;

			// round pulse length to multiples of 10
			p1 = pulse % 10;
			p2 = p1 >= 5 ? (pulse - p1 + 10) : (pulse - p1);

			// sample
			hstream[stream_write] = (unsigned short) p2 / 10;
			if (cfg->sample_on_1)
				stream_write++;
		}
	}

	return (void *) 0;
}

void rfsniffer_stdout_handler(rfsniffer_event_t *e) {
	if (e->message)
		// print formatted string
		printf(e->message);
	else {
		// print raw values
		const char *fmt1 = "HANDLER Protocol = %d, Raw = 0x%llx, Repeat = %d, Device = 0x%x, Channel = %d, Event = %d, Value = 0x%02x\n";
		const char *fmt2 = "HANDLER Protocol = %d, Raw = 0x%llx, Repeat = %d, Device = 0x%x, Channel = %d, Event = %d, Value = %02.1f\n";
		if (e->key)
			printf(fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->key, e->value);
		if (e->ikey1)
			printf(fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey1, e->ivalue1);
		if (e->ikey2)
			printf(fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey2, e->ivalue2);
		if (e->ikey3)
			printf(fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey3, e->ivalue3);
		if (e->fkey1)
			printf(fmt2, e->protocol, e->raw, e->repeat, e->device, e->channel, e->fkey1, e->fvalue1);
	}
	free(e);
}

void rfsniffer_syslog_handler(rfsniffer_event_t *e) {
	if (e->message)
		// print formatted string
		syslog(LOG_NOTICE, e->message);
	else {
		// print raw values
		const char *fmt1 = "HANDLER Protocol = %d, Raw = 0x%llx, Repeat = %d, Device = 0x%x, Channel = %d, Event = %d, Value = 0x%02x\n";
		const char *fmt2 = "HANDLER Protocol = %d, Raw = 0x%llx, Repeat = %d, Device = 0x%x, Channel = %d, Event = %d, Value = %02.1f\n";
		if (e->key)
			syslog(LOG_NOTICE, fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->key, e->value);
		if (e->ikey1)
			syslog(LOG_NOTICE, fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey1, e->ivalue1);
		if (e->ikey2)
			syslog(LOG_NOTICE, fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey2, e->ivalue2);
		if (e->ikey3)
			syslog(LOG_NOTICE, fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey3, e->ivalue3);
		if (e->fkey1)
			syslog(LOG_NOTICE, fmt2, e->protocol, e->raw, e->repeat, e->device, e->channel, e->fkey1, e->fvalue1);
	}
	free(e);
}

rfsniffer_config_t *rfsniffer_default_config() {
	cfg = malloc(sizeof(*cfg));
	memset(cfg, 0, sizeof(*cfg));

	// default config
	cfg->rx = RX;
	cfg->tx = TX;
	cfg->analyzer_mode = 0;
	cfg->realtime_mode = 0;
	cfg->timestamp = 0;
	cfg->pulse_counter_active = 0;
	cfg->decoder_delay = 1;
	cfg->bits_to_sample = 32;
	cfg->collect_identical_codes = 1;
	cfg->sync_on_0 = 1;
	cfg->sync_on_1 = 0;
	cfg->sample_on_0 = 1;
	cfg->sample_on_1 = 0;
	cfg->sync_min = 1800;
	cfg->sync_max = 2000;
	cfg->bitdivider = 1500;
	cfg->noise = 100;
	cfg->verbose = 0;
	cfg->quiet = 0;
	cfg->syslog = 0;
	cfg->json = 0;
	cfg->sysfslike = 0;
	cfg->rfsniffer_handler = &rfsniffer_stdout_handler;

	// hand-over to rfcodec module
	rfcodec_set_config(cfg);

	return cfg;
}

int rfsniffer_init() {
	if (!cfg->quiet)
		printf("INIT test 2x 0xdeadbeef = %s\n", printbits64(0xdeadbeefdeadbeef, 0x0101010101010101));

	if (init_micros())
		return -1;

	void *sampler, *decoder;
	if (cfg->realtime_mode) {
		sampler = &realtime_sampler;
		decoder = &realtime_decoder;
		if (!cfg->quiet)
			printf("INIT using realtime_sampler and realtime_decoder\n");
	} else {
		sampler = &stream_sampler;
		decoder = &stream_decoder;
		if (!cfg->quiet)
			printf("INIT using stream_sampler and stream_decoder\n");
	}

	// decoder thread
	if (pthread_create(&thread_decoder, NULL, decoder, NULL)) {
		perror("Error creating decoder thread");
		return -1;
	}
	if (!cfg->quiet)
		printf("INIT started decoder thread\n");

	// sampler thread
	if (pthread_create(&thread_sampler, NULL, sampler, NULL)) {
		perror("Error creating sampler thread");
		return -1;
	}
	if (!cfg->quiet)
		printf("INIT started sampler thread\n");

	return 0;
}

int rfsniffer_close() {
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
	free(cfg);
	return 0;
}

#ifdef RFSNIFFER_MAIN
int main(int argc, char *argv[]) {
	return rfsniffer_main(argc, argv);
}
#endif

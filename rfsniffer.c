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

#include "rfsniffer.h"
#include "flamingo.h"
#include "utils.h"
#include "frozen.h"

#define RFSNIFFER_MAIN

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

// ring buffers + code matrix
static unsigned short lstream[0xffff], hstream[0xffff];
static unsigned short stream_write;
static unsigned long long matrix[BUFFER][BUFFER];

// pulse counters: index equals pulse length, e.g. 10µs=1, 20µs=2, etc
static unsigned short lpulse_counter[PULSE_COUNTER_MAX];
static unsigned short hpulse_counter[PULSE_COUNTER_MAX];

// default configuration
static unsigned char analyzer_mode = 0;
static unsigned char realtime_mode = 0;
static unsigned char pulse_counter_active = 0;
static unsigned char decoder_delay = 1;
static unsigned char bits_to_sample = 32;
static unsigned char collect_identical_codes = 1;
static unsigned char json = 0;
static unsigned char sync_on_0 = 1;
static unsigned char sync_on_1 = 0;
static unsigned char sample_on_0 = 1;
static unsigned char sample_on_1 = 0;
static unsigned long sync_min = 1800;
static unsigned long sync_max = 2000;
static unsigned long bitdivider = 1500;
static unsigned short noise = 100;

static unsigned long timestamp_last_code;

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

static int usage() {
	printf("Usage: rfsniffer -j -c -nX -dX -a -r -bX -sX -SX -xX -yX -zX\n");
	printf("  void  normal mode, sync on known pulses, sample and decode messages\n");
	printf("  -j    print messages as JSON\n");
	printf("  -c    activate pulse length counter\n");
	printf("  -n X  pulse length counter noise level (default 100µs)\n");
	printf("  -e    process each single code (default collect)\n");
	printf("  -d X  decoder delay in seconds, default 1\n");
	printf("  -a    analyzer mode\n");
	printf("  -r    realtime mode\n");
	printf("  -b X  analyzer: bits to sample (0 - 64) default 32\n");
	printf("  -s X  analyzer: sync on pulse 0=LOW (default) 1=HIGH 2=EDGE)\n");
	printf("  -S X  analyzer: sample on pulse 0=LOW (default) 1=HIGH 2=EDGE)\n");
	printf("  -x X  analyzer: sync pulse min length in microseconds\n");
	printf("  -y X  analyzer: sync pulse max length in microseconds\n");
	printf("  -z X  analyzer: 0/1 bit divider pulse length in microseconds\n");
	return EXIT_FAILURE;
}

static int rfsniffer_main(int argc, char **argv) {
	if (argc > 0) {
		int c, i;
		while ((c = getopt(argc, argv, "ab:cd:ejn:rs:S:x:y:z:")) != -1) {
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
			case 'e':
				collect_identical_codes = 0;
				break;
			case 'j':
				json = 1;
				break;
			case 'n':
				noise = atoi(optarg);
				break;
			case 'r':
				realtime_mode = 1;
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
				return usage();
			}
		}
	}

	rfsniffer_init();
	pause();
	rfsniffer_close();
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
#ifdef RFSNIFFER_MAIN
			printf("0110 decode error %s\n", printbits64(in, 0x0001000100010001));
#endif
			return 0;
		}
		count -= 2;
		in <<= 2;
	}
	// printf("--> %s\n", printbits(out, 0x00010001));
	return out;
}

static void decode_nexus(unsigned long long raw, unsigned char repeat) {

	// sensor transmits 10 messages
	// 1st message get's lost due to sync on pause between repeats
	// we want at least 3 identical messages
	if (1 <= repeat && repeat <= 3) {
#ifdef RFSNIFFER_MAIN
		printf("NEXUS {%d} too few repeats, discard message 0x%08llx = %s\n", repeat, raw, printbits64(raw, 0x1011001101));
#endif
		return;
	}

	if ((raw & 0x0f00) != 0x0f00) {
#ifdef RFSNIFFER_MAIN
		printf("NEXUS message verification failed 0x%08llx = %s\n", raw, printbits64(raw, 0x1011001101));
#endif
		return;
	}

	// https://github.com/merbanan/rtl_433/blob/master/src/devices/nexus.c
	// 9 nibbles: [id0] [id1] [flags] [temp0] [temp1] [temp2] [const] [humi0] [humi1]
	unsigned long long code = raw;
	unsigned char h = code & 0xff;
	code >>= 8;
	code >>= 4; // always 1111 - used for message verification
	short t_raw = (short) (code & 0x0fff);
	code >>= 12;
	unsigned char c = code & 0x07;
	code >>= 3;
	unsigned char b = code & 0x01;
	code >>= 1;
	unsigned char i = code & 0xff;

	// calculate float temperature value
	float t = (t_raw & 0x0800) ? -0.1 * (0x0fff - t_raw) : 0.1 * t_raw;

	// TODO timestamp via utils

	if (json) {
		char format[BUFFER], craw[12], ctemp[6];
		snprintf(craw, 12, "0x%08llx", raw);
		snprintf(ctemp, 6, "%02.1f", t);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%d, %Q:%d, %Q:%d, %Q:%d, %Q:%Q, %Q:%d }\n");
		json_printf(&jout, format, "type", "NEXUS", "raw", craw, "repeat", repeat, "id", i, "channel", c, "battery", b, "temp", ctemp, "hum", h);
	} else
		printf("NEXUS {%d} 0x%08llx Id = %d, Channel = %d, Battery = %s, Temp = %02.1fC, Hum = %d%%\n", repeat, raw, i, c, b ? "OK" : "LOW", t, h);
}

static void decode_flamingo28(unsigned long long raw, unsigned char r) {
	unsigned long message;
	unsigned int xmitter;
	unsigned char channel, command, payload, rolling;

	message = decrypt(raw);
	decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);

	if (json) {
		char format[BUFFER], craw[12], cid[12];
		snprintf(craw, 12, "0x%08llx", raw);
		snprintf(cid, 12, "0x%04x", xmitter);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%Q, %Q:%d, %Q:%d, %Q:%d, %Q:%d }\n");
		json_printf(&jout, format, "type", "FLAMINGO28", "raw", craw, "sync", 0, "id", cid, "channel", channel, "command", command, "payload", payload, "rolling", rolling);
	} else
		printf("FLAMINGO28 0x%08llx Id = 0x%x, Channel = %d, Command = %d, Payload = 0x%02x, Rolling = %d\n", raw, xmitter, channel, command, payload, rolling);
}

static void decode_flamingo24(unsigned long long raw, unsigned char r) {

	// TODO decode

	if (json) {
		char format[BUFFER], craw[12];
		snprintf(craw, 12, "0x%08llx", raw);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu }\n");
		json_printf(&jout, format, "type", "FLAMINGO24", "raw", craw, "sync", sync);
	} else
		printf("FLAMINGO24 0x%08llx %s\n", raw, printbits(raw, 0x01010101));
}

static void decode_flamingo32(unsigned long long raw, unsigned char r) {

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
		json_printf(&jout, format, "type", "FLAMINGO32", "raw", craw, "sync", 0, "id", cid, "channel", channel, "command", command, "payload", payload);
	} else
		printf("FLAMINGO32 0x%04lx Id = 0x%x, Channel = %d, Command = %d, Payload = 0x%02x\n", code_save, xmitter, channel, command, payload);
}

static void decode_anaylzer(unsigned long long raw, unsigned char r) {
	printf("ANALYZER 0x%08llx %s\n", raw, printbits64(raw, 0x0101010101010101));
}

static void decode(unsigned char pattern, unsigned long long code, unsigned char repeat) {
	if (!code) {
#ifdef RFSNIFFER_MAIN
		printf("DECODE received empty message\n");
#endif
		return;
	}

	// dispatch to corresponding pattern decoder
	switch (pattern) {
	case 1:
		return decode_nexus(code, repeat);
	case 2:
		return decode_flamingo28(code, repeat);
	case 3:
		return decode_flamingo24(code, repeat);
	case 4:
		return decode_flamingo32(code, repeat);
	case 255:
		return decode_anaylzer(code, repeat);
	default:
		printf("DECODE no decoder configured for pattern %d 0x%0llx\n", pattern, code);
	}
}

static void matrix_store(unsigned char pattern, unsigned long long code) {
	unsigned char entries = matrix[pattern][0];
	matrix[pattern][entries] = code;
	matrix[pattern][0]++;
}

static void matrix_decode_pattern(unsigned char pattern) {
	unsigned char entries = matrix[pattern][0];
	unsigned char repeat, index = 0;
	unsigned long long code;

	if (collect_identical_codes) {
		// TODO find ALL identical codes, not only when differs
		code = matrix[pattern][index];
		repeat = 0;
		do {
			if (code == matrix[pattern][index])
				repeat++; // still the same code
			else {
				decode(pattern, code, repeat);
				code = matrix[pattern][index];
				repeat = 1; // new code
			}
		} while (++index < entries);
		// rewind once, decode and increment again
		index--;
		decode(pattern, code, repeat);
		index++;
	} else {
		do {
			code = matrix[pattern][index];
			decode(pattern, code, 0);
		} while (++index < entries);
	}
}

static void matrix_decode() {
	unsigned char pattern, index, filled = 0;

	// check if empty
	for (pattern = 0; pattern < 0xff; pattern++) {
		index = matrix[pattern][0];
		if (index)
			filled = 1;
	}
	if (!filled)
		return;

	printf("MATRIX codes ");
	for (pattern = 0; pattern < 0xff; pattern++) {
		index = matrix[pattern][0];
		if (index)
			printf("%d:%d ", pattern, index);
	}
	printf("\n");

	for (pattern = 0; pattern < 0xff; pattern++) {
		index = matrix[pattern][0];
		if (index)
			matrix_decode_pattern(pattern);

		matrix[pattern][0] = 0;
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
	for (i = 0; i < (noise / 10); i++) {
		lpulse_counter[i] = hpulse_counter[i] = 0;	// ignore noise pulses
	}
	printf("\n\nTOP-TEN (noise level = %uµs)\n", noise);
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

int rfsniffer_init() {
#ifdef RFSNIFFER_MAIN
	printf("INIT test 2x 0xdeadbeef = %s\n", printbits64(0xdeadbeefdeadbeef, 0x0101010101010101));
#endif

	if (!bcm2835_init())
		return -1;

	void *sampler, *decoder;
	if (realtime_mode) {
		sampler = &realtime_sampler;
		decoder = &realtime_decoder;
#ifdef RFSNIFFER_MAIN
		printf("INIT using realtime_sampler and realtime_decoder\n");
#endif
	} else {
		sampler = &stream_sampler;
		decoder = &stream_decoder;
#ifdef RFSNIFFER_MAIN
		printf("INIT using stream_sampler and stream_decoder\n");
#endif
	}

	// decoder thread
	if (pthread_create(&thread_decoder, NULL, decoder, NULL)) {
		perror("Error creating decoder thread");
		return -1;
	}
#ifdef RFSNIFFER_MAIN
	printf("INIT started decoder thread\n");
#endif

	// sampler thread
	if (pthread_create(&thread_sampler, NULL, sampler, NULL)) {
		perror("Error creating sampler thread");
		return -1;
	}
#ifdef RFSNIFFER_MAIN
	printf("INIT started sampler thread\n");
#endif

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
	return 0;
}

static void* realtime_decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

#ifdef RFSNIFFER_MAIN
	printf("DECODER run every %d seconds, %s\n", decoder_delay, collect_identical_codes ? "collect identical codes" : "process each code separately");
#endif

	struct timeval tNow;
	unsigned long age_last_message;

	while (1) {
		sleep(decoder_delay);

		// repeating transmission: wait until actual message is minimum 500ms old
		gettimeofday(&tNow, NULL);
		age_last_message = (((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000) - timestamp_last_code;
		while (age_last_message < 500) {
			msleep(100);
			gettimeofday(&tNow, NULL);
			age_last_message = (((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000) - timestamp_last_code;
#ifdef RFSNIFFER_MAIN
			printf("DECODER receiving in progress %lums\n", age_last_message);
#endif
		}

		// print pulse counters,  top ten pulse lengths, clear counters
		if (pulse_counter_active) {
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

	// clear matrix
	memset(matrix, 0, sizeof(matrix));

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

		printf("SAMPLER sync on %lu-%luµs %s pulses, sampling %d %s bits, 0/1 divider pulse length %luµs\n", sync_min, sync_max, csync, bits_to_sample, csample, bitdivider);
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

		// ignore noise & crap
		if (pulse < noise)
			continue;
		if (pulse > 22222)
			continue;

		if (analyzer_mode || pulse_counter_active) {

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
			} else {
				printf("SAMPLER pulse_counter overflow %d\n", pulse_counter);
			}

		} else {

			// error detection
			if (--state_reset == 0) {
#ifdef RFSNIFFER_MAIN
				printf("SAMPLER sampling on pattern %d aborted\n", state);
#endif
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
				} else if (T1SMIN < pulse && pulse < T1SMAX) {
					bits = 28;
					state = 2;
				} else if (T4SMIN < pulse && pulse < T4SMAX) {
					bits = 24;
					state = 3;
				} else if (T2S1MIN < pulse && pulse < T2S1MAX) {
					bits = 64;
					state = 4;
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
					matrix_store(state, code);
					timestamp_last_code = ((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000;
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
					matrix_store(state, code);
					timestamp_last_code = ((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000;
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
					matrix_store(state, code);
					timestamp_last_code = ((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000;
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
					matrix_store(state, code);
					timestamp_last_code = ((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000;
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
				// LOW pulse
				if (sync_on_0) {
					if (sync_min < pulse && pulse < sync_max) {
						bits = bits_to_sample;
						state = 255; // next is analyzer sample mode
						printf("\nSYN ");
					}
				}
			} else {
				// HIGH pulse
				if (sync_on_1) {
					if (sync_min < pulse && pulse < sync_max) {
						bits = bits_to_sample;
						state = 255; // next is analyzer sample mode
						printf("\nSYN ");
					}
				}
			}
			break;

		case 255: // analyzer sample mode: sample bits and print
			if (bits) {
				if (pin) {
					// LOW pulse
					printf("L%02d ", pulse_counter);
					if (sample_on_0) {
						if (pulse > bitdivider)
							code++;
						if (--bits == 0) {
							matrix_store(state, code);
							timestamp_last_code = ((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000;
							state = 254; // back to analyzer sync mode
							printf("\n");
						} else {
							code <<= 1;
						}
					}
				} else {
					// HIGH pulse
					printf("H%02d ", pulse_counter);
					if (sample_on_1) {
						if (pulse > bitdivider)
							code++;
						if (--bits == 0) {
							matrix_store(state, code);
							timestamp_last_code = ((tNow.tv_sec * 1000000) + tNow.tv_usec) / 1000;
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

unsigned long long stream_probe_low(unsigned short pos, unsigned char bits, unsigned short divider) {
	unsigned long long code = 0;
	unsigned short l, h;
	for (int i = 0; i < bits; i++) {
		l = lstream[pos];
		h = hstream[pos];

		if (!l && !h)
			return 0;

		if (l > divider)
			code++;

		code <<= 1;
		pos++;
	}
	return code;
}

unsigned long long stream_probe_high(unsigned short pos, unsigned char bits, unsigned short divider) {
	unsigned long long code = 0;
	unsigned short l, h;
	for (int i = 0; i < bits; i++) {
		l = lstream[pos];
		h = hstream[pos];

		if (!l && !h)
			return 0;

		if (h > divider)
			code++;

		code <<= 1;
		pos++;
	}
	return code;
}

static unsigned short stream_rewind(unsigned short current, unsigned short positions) {
	for (int i = 0; i < positions; i++)
		current--;
	return current;
}

static void stream_dump(unsigned short start, unsigned short positions) {
	unsigned short i, l, h;
	printf("REWIND %u %u\n", start, positions);
	for (i = 0; i < positions; i++) {
		l = lstream[start];
		h = hstream[start];
		start++;
		printf("H%03u L%03u ", l, h);
	}
	printf("\n");
}

static void* stream_decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

#ifdef RFSNIFFER_MAIN
	printf("DECODER run every %d seconds, %s\n", decoder_delay, collect_identical_codes ? "collect identical codes" : "process each code separately");
	unsigned short delta, stream_last;
#endif

	unsigned short i, l, h, rewind, progress, stream_read = 0;
	unsigned long long code;

	while (1) {
		sleep(decoder_delay);

		// wait till we see 8 bits of noise (<50µs)
		do {
			progress = 0;
			rewind = stream_rewind(stream_write, 8);
			// stream_dump(rewind, 8);
			for (i = 0; i < 8; i++) {
				l = lstream[rewind];
				h = hstream[rewind];
				if (l && h && (l > 50 || h > 50)) {
					progress = 1;
				}
				rewind++;
			}
			if (progress) {
#ifdef RFSNIFFER_MAIN
				printf("DECODER receiving in progress, wait\n");
#endif
				msleep(100);
			}
		} while (progress);

#ifdef RFSNIFFER_MAIN
		delta = (stream_write > stream_last) ? stream_write - stream_last : 0xffff - stream_last + stream_write;
		printf("DECODER stream [%05u:%04u]\n", stream_write, delta);
#endif

		do {
			l = lstream[stream_read];
			h = hstream[stream_read];

			// update pulse counters: 10th pulse length is the array index
			if (pulse_counter_active) {
				if (l < PULSE_COUNTER_MAX)
					lpulse_counter[l]++;
				else
					printf("SAMPLER LOW pulse_counter overflow %d\n", l);

				if (h < PULSE_COUNTER_MAX)
					hpulse_counter[h]++;
				else
					printf("SAMPLER HIGH pulse_counter overflow %d\n", h);
			}

			if (380 < l && l < 400) {
				// not a real sync - it's the pause between 1st and 2nd message - but 1st message is always blurred
				printf("DECODER %u SYNC on NEXUS\n", l);
				code = stream_probe_low(stream_read, 36, 150);
				if (code)
					matrix_store(1, code);

			} else if (T1SMIN < l && l < T1SMAX) {
				printf("DECODER %u SYNC on FLAMINGO28\n", l);
				code = stream_probe_high(stream_read, 28, T1X2);
				if (code)
					matrix_store(2, code);

			} else if (T4SMIN < l && l < T4SMAX) {
				printf("DECODER %u SYNC on FLAMINGO24\n", l);
				code = stream_probe_high(stream_read, 24, T1X2);
				if (code)
					matrix_store(3, code);

			} else if (T2S1MIN < l && l < T2S1MAX) {
				printf("DECODER %u SYNC on FLAMINGO32\n", l);
				code = stream_probe_low(stream_read, 64, T2Y);
				if (code)
					matrix_store(4, code);

			}

		} while (++stream_read != stream_write); // follow the stream_write position

#ifdef RFSNIFFER_MAIN
		stream_last = stream_write;
#endif

		// print pulse counters,  top ten pulse lengths, clear counters
		if (pulse_counter_active) {
			dump_pulse_counters();
		}

		matrix_decode();
	}
	return (void *) 0;
}

static void* stream_sampler(void *arg) {
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
	unsigned long pulse, p1, p2;
	unsigned char pin, dummy;

	// initialize tLast for correct 1st pulse calculation
	gettimeofday(&tLast, NULL);

	// clear matrix
	memset(matrix, 0, sizeof(matrix));

	stream_write = 0;
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

		if (pin) {
			// LOW pulse

			// clear
			lstream[stream_write] = 0;

			// validate
			if (pulse < noise)
				continue;
			if (pulse > 655360)
				continue;

			// round pulse length to multiples of 10
			p1 = pulse % 10;
			p2 = p1 >= 5 ? (pulse - p1 + 10) : (pulse - p1);

			// sample
			lstream[stream_write] = (unsigned short) p2 / 10;
			if (sample_on_0)
				stream_write++;

		} else {
			// HIGH pulse

			// clear
			hstream[stream_write] = 0;

			// validate
			if (pulse < noise)
				continue;
			if (pulse > 655360)
				continue;

			// round pulse length to multiples of 10
			p1 = pulse % 10;
			p2 = p1 >= 5 ? (pulse - p1 + 10) : (pulse - p1);

			// sample
			hstream[stream_write] = (unsigned short) p2 / 10;
			if (sample_on_1)
				stream_write++;
		}
	}

	return (void *) 0;
}

#ifdef RFSNIFFER_MAIN
int main(int argc, char *argv[]) {
	return rfsniffer_main(argc, argv);
}
#endif

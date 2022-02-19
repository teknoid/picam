/***
 *
 * sampling and decoding 433MHz messages

 * unfortunately this works only if the pi is idle
 * especially there should be no USB traffic during sampling
 *
 * Copyright (C) 02/2022 by teknoid
 *
 *
 * tested with following devices
 *
 * - ELRO Flamingo Switches
 * - NEXUS Weather sensor
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <syslog.h>
#include <pthread.h>

#include "rfsniffer.h"
#include "rfcodec.h"
#include "utils.h"
#include "flamingo.h"
#include "gpio.h"

// odroid
// #define RX				247
// #define TX				219

// picam
#define RX					"GPIO27"
#define TX					"GPIO17"

#define BUFFER				0xff

static const char *handler_fmt1 = "HANDLER Protocol = %d, Raw = 0x%llx, Repeat = %d, Device = 0x%x, Channel = %d, Event = %d, Value = 0x%02x\n";
static const char *handler_fmt2 = "HANDLER Protocol = %d, Raw = 0x%llx, Repeat = %d, Device = 0x%x, Channel = %d, Event = %d, Value = %02.1f\n";

static pthread_t thread_decoder;
static pthread_t thread_sampler;

// code matrix: x=protocol code, y=code number, [x][0] contains next free entry
static uint64_t matrix[BUFFER][BUFFER];

rfsniffer_config_t *rfcfg;

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
	printf("Usage: rfsniffer -v -q -jX -fX -c -nX -dX -a -r -0 -1 -Tc -bX -sX -SX -xX -yX -zX\n");
	printf("  void  normal mode, sync on known pulses, sample and decode messages\n");
	printf("  -v    verbose console output\n");
	printf("  -q    no console output except errors\n");
	printf("  -j X  create JSON file\n");
	printf("  -f X  create sysfs-like entries in specified directory\n");
	printf("  -t    add timestamp to messages\n");
	printf("  -c    activate pulse length counter\n");
	printf("  -n X  pulse length counter noise level (default 100µs)\n");
	printf("  -e    process each single code (default collect)\n");
	printf("  -d X  decoder delay in milli seconds, default 1000\n");
	printf("  -a    analyzer mode\n");
	printf("  -r    realtime mode\n");
	printf("  -0    sample on 0\n");
	printf("  -1    sample on 1 (default)\n");
	printf("  -T c  run unit tests and decode <c>");
	printf("  -b X  analyzer: bits to sample (0 - 64) default 32\n");
	printf("  -s X  analyzer: sync on pulse 0=LOW (default) 1=HIGH 2=EDGE)\n");
	printf("  -x X  analyzer: sync pulse min length in microseconds/10\n");
	printf("  -y X  analyzer: sync pulse max length in microseconds/10\n");
	printf("  -z X  analyzer: 0/1 bit divider pulse length in microseconds/10\n");
	return EXIT_FAILURE;
}

void rfsniffer_test(int argc, char **argv) {
	uint16_t test = 0;

	test += 4;
	printf("test uint overflow +4 %05u\n", test);
	test -= 8;
	printf("test uint overflow -8 %05u\n", test);

	test = UINT16_MAX - 4;
	printf("test uint overflow MAX -4 %05u\n", test);
	test += 8;
	printf("test uint overflow MAX +8 %05u\n", test);

	rfcodec_test(argc, argv);
}

static int rfsniffer_main(int argc, char **argv) {
	if (gpio_init() < 0)
		return -1;

	// initialize a default configuration
	rfcfg = rfsniffer_default_config();

	if (argc > 0) {
		int c, i;
		while ((c = getopt(argc, argv, "ab:cd:ef:jn:qr01s:S:tTvx:y:z:")) != -1) {
			switch (c) {
			case 'a':
				rfcfg->analyzer_mode = 1;
				break;
			case 'b':
				rfcfg->bits_to_sample = atoi(optarg);
				break;
			case 'c':
				rfcfg->pulse_counter_active = 1;
				break;
			case 'd':
				rfcfg->decoder_delay = atoi(optarg);
				break;
			case 'e':
				rfcfg->collect_identical_codes = 0;
				break;
			case 'f':
				rfcfg->sysfslike = optarg;
				break;
			case 'j':
				rfcfg->json = 1;
				break;
			case 'n':
				rfcfg->noise = atoi(optarg);
				break;
			case 'q':
				rfcfg->quiet = 1;
				break;
			case 'r':
				rfcfg->stream_mode = 0;
				rfcfg->realtime_mode = 1;
				break;
			case '0':
				rfcfg->sample_on_0 = 1;
				rfcfg->sample_on_1 = 0;
				break;
			case '1':
				rfcfg->sample_on_0 = 0;
				rfcfg->sample_on_1 = 1;
				break;
			case 's':
				i = atoi(optarg);
				switch (i) {
				case 0:
					rfcfg->sync_on_0 = 1;
					rfcfg->sync_on_1 = 0;
					break;
				case 1:
					rfcfg->sync_on_0 = 0;
					rfcfg->sync_on_1 = 1;
					break;
				case 2:
					rfcfg->sync_on_0 = 1;
					rfcfg->sync_on_1 = 1;
					break;
				}
				break;
			case 't':
				rfcfg->timestamp = 1; // TODO
				break;
			case 'T':
				rfsniffer_test(argc, argv);
				return 0;
			case 'v':
				rfcfg->verbose = 1;
				break;
			case 'x':
				rfcfg->sync_min = strtoul(optarg, NULL, 0);
				break;
			case 'y':
				rfcfg->sync_max = strtoul(optarg, NULL, 0);
				break;
			case 'z':
				rfcfg->bitdivider = strtoul(optarg, NULL, 0);
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

void matrix_decode_protocol(uint8_t x) {
	uint64_t code;
	uint8_t repeat, y = 1;

	if (rfcfg->collect_identical_codes) {
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

void matrix_decode() {
	uint8_t x, y, filled = 0;

	// check if empty
	for (x = 0; x < BUFFER; x++) {
		y = matrix[x][0];
		if (y > 1)
			filled = 1;
	}
	if (!filled)
		return;

	// print summary
	if (rfcfg->verbose) {
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

void matrix_store(uint8_t x, uint64_t code) {
	uint8_t ynext = matrix[x][0];
	matrix[x][ynext] = code;
	matrix[x][0]++; // increment next free entry
}

void matrix_init() {
	memset(matrix, 0, sizeof(matrix));
	for (uint8_t i = 0; i < BUFFER; i++)
		matrix[i][0] = 1; // initialize next free entry
}

void rfsniffer_stdout_handler(rfsniffer_event_t *e) {
	if (e->message)
		// print formatted string
		printf(e->message);
	else {
		// print raw values
		if (e->key)
			printf(handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->key, e->value);
		if (e->ikey1)
			printf(handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey1, e->ivalue1);
		if (e->ikey2)
			printf(handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey2, e->ivalue2);
		if (e->ikey3)
			printf(handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey3, e->ivalue3);
		if (e->fkey1)
			printf(handler_fmt2, e->protocol, e->raw, e->repeat, e->device, e->channel, e->fkey1, e->fvalue1);
	}
	free(e);
}

void rfsniffer_syslog_handler(rfsniffer_event_t *e) {
	if (e->message)
		// print formatted string
		syslog(LOG_NOTICE, e->message);
	else {
		// print raw values
		if (e->key)
			syslog(LOG_NOTICE, handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->key, e->value);
		if (e->ikey1)
			syslog(LOG_NOTICE, handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey1, e->ivalue1);
		if (e->ikey2)
			syslog(LOG_NOTICE, handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey2, e->ivalue2);
		if (e->ikey3)
			syslog(LOG_NOTICE, handler_fmt1, e->protocol, e->raw, e->repeat, e->device, e->channel, e->ikey3, e->ivalue3);
		if (e->fkey1)
			syslog(LOG_NOTICE, handler_fmt2, e->protocol, e->raw, e->repeat, e->device, e->channel, e->fkey1, e->fvalue1);
	}
	free(e);
}

rfsniffer_config_t* rfsniffer_default_config() {
	rfcfg = malloc(sizeof(*rfcfg));
	memset(rfcfg, 0, sizeof(*rfcfg));

	// default config
	rfcfg->rx = RX;
	rfcfg->tx = TX;
	rfcfg->analyzer_mode = 0;
	rfcfg->realtime_mode = 0;
	rfcfg->stream_mode = 1;
	rfcfg->timestamp = 0;
	rfcfg->pulse_counter_active = 0;
	rfcfg->decoder_delay = 1000;
	rfcfg->bits_to_sample = 32;
	rfcfg->collect_identical_codes = 1;
	rfcfg->sync_on_0 = 1;
	rfcfg->sync_on_1 = 0;
	rfcfg->sample_on_0 = 0;
	rfcfg->sample_on_1 = 1;
	rfcfg->sync_min = 3800;
	rfcfg->sync_max = 4200;
	rfcfg->bitdivider = 3000;
	rfcfg->noise = 100;
	rfcfg->verbose = 0;
	rfcfg->validate = 1;
	rfcfg->quiet = 0;
	rfcfg->syslog = 0;
	rfcfg->json = 0;
	rfcfg->sysfslike = 0;
	// rfcfg->rfsniffer_handler = &rfsniffer_stdout_handler;

	return rfcfg;
}

int rfsniffer_init() {
	if (gpio_init())
		return -1;

	// initialize the matrix
	matrix_init();

	void *sampler = 0, *decoder = 0;
	if (rfcfg->realtime_mode) {
		sampler = &realtime_sampler;
		decoder = &realtime_decoder;
		if (!rfcfg->quiet)
			printf("INIT using realtime_sampler and realtime_decoder\n");
	}
	if (rfcfg->stream_mode) {
		sampler = &stream_sampler;
		decoder = &stream_decoder;
		if (!rfcfg->quiet)
			printf("INIT using stream_sampler and stream_decoder\n");
	}

	// decoder thread
	if (decoder != 0) {
		if (pthread_create(&thread_decoder, NULL, decoder, NULL)) {
			perror("Error creating decoder thread");
			return -1;
		}
		if (!rfcfg->quiet)
			printf("INIT started decoder thread\n");
	}

	// sampler thread
	if (sampler != 0) {
		if (pthread_create(&thread_sampler, NULL, sampler, NULL)) {
			perror("Error creating sampler thread");
			return -1;
		}
		if (!rfcfg->quiet)
			printf("INIT started sampler thread\n");
	}

	return 0;
}

int rfsniffer_close() {
	if (thread_sampler) {
		if (pthread_cancel(thread_sampler))
			perror("Error canceling thread");

		if (pthread_join(thread_sampler, NULL))
			perror("Error joining thread");
	}

	if (thread_decoder) {
		if (pthread_cancel(thread_decoder))
			perror("Error canceling thread");

		if (pthread_join(thread_decoder, NULL))
			perror("Error joining thread");
	}

	free(rfcfg);
	return 0;
}

#ifdef RFSNIFFER_MAIN
int main(int argc, char *argv[]) {
	return rfsniffer_main(argc, argv);
}
#endif

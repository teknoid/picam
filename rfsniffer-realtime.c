#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <unistd.h>

#include "gpio.h"
#include "rfsniffer.h"
#include "utils.h"

#include "flamingo.h" // for the timings

#define BUFFER				0xff

#define STATE_RESET			129 // max 64 bit code -> 128 high/low edges + 1

#define PULSE_COUNTER_MAX	BUFFER * BUFFER

static const char *CLOW = "LOW\0";
static const char *CHIGH = "HIGH\0";
static const char *CEDGE = "EDGE\0";
static const char *CNONE = "NONE\0";

// realtime_decoder: calculate age of last message
static uint32_t timestamp_last_code;

// pulse counters: index equals pulse length x 100, e.g. 1=100µs, 2=200µs ... 255=25500µs
static uint16_t lpulse_counter[BUFFER];
static uint16_t hpulse_counter[BUFFER];

extern rfsniffer_config_t *rfcfg;

static void dump_pulse_counters() {
	printf("\nLCOUNTER   ");
	for (int i = 0; i < BUFFER; i++)
		if (lpulse_counter[i])
			printf("%d:%d ", i, lpulse_counter[i]);
	printf("\n");

	printf("HCOUNTER   ");
	for (int i = 0; i < BUFFER; i++)
		if (hpulse_counter[i])
			printf("%d:%d ", i, hpulse_counter[i]);
	printf("\n");
}

void* realtime_decoder(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	if (!rfcfg->quiet)
		printf("DECODER run every %d µs, %s\n", rfcfg->decoder_delay, rfcfg->collect_identical_codes ? "collect identical codes" : "process each code separately");

	while (1) {
		msleep(rfcfg->decoder_delay);

		// repeating transmission: wait until actual message is minimum 500ms old
		uint32_t age_last_message = gpio_micros() - timestamp_last_code;
		while (age_last_message < 500) {
			msleep(100);
			age_last_message = gpio_micros() - timestamp_last_code;
			if (rfcfg->verbose)
				printf("DECODER receiving in progress %ums\n", age_last_message);
		}

		// print pulse counters,  top ten pulse lengths, clear counters
		if (rfcfg->pulse_counter_active)
			dump_pulse_counters();

		matrix_decode();
	}
	return (void*) 0;
}

void* realtime_sampler(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	// elevate realtime priority for sampler thread
	if (elevate_realtime(3) < 0)
		return (void*) 0;

	const char *name = rfcfg->rx;
	while (*name >= 'A')
		name++;

	char buf[32];
	snprintf(buf, 32, "/usr/bin/gpio edge %d both", atoi(name));
	system(buf);

	// poll setup
	snprintf(buf, 32, "/sys/class/gpio/gpio%d/value", atoi(name));
	struct pollfd fdset[1];
	fdset[0].fd = open(buf, O_RDONLY);
	fdset[0].events = POLLPRI;
	fdset[0].revents = 0;

	uint64_t code;
	uint32_t pulse, last, divider, p1, p2;
	uint16_t pulse_counter;
	uint8_t protocol, state_reset, bits, pin, dummy;
	int state;

	// initialize tLast for correct 1st pulse calculation
	last = gpio_micros();

	state = -1;
	if (rfcfg->analyzer_mode) {
		state = -2;

		const char *csync, *csample;
		if (rfcfg->sync_on_0 && rfcfg->sync_on_1)
			csync = CEDGE;
		else if (rfcfg->sync_on_0 && !rfcfg->sync_on_1)
			csync = CLOW;
		else if (!rfcfg->sync_on_0 && rfcfg->sync_on_1)
			csync = CHIGH;
		else
			csync = CNONE;

		if (rfcfg->sample_on_0 && rfcfg->sample_on_1)
			csample = CEDGE;
		else if (rfcfg->sample_on_0 && !rfcfg->sample_on_1)
			csample = CLOW;
		else if (!rfcfg->sample_on_0 && rfcfg->sample_on_1)
			csample = CHIGH;
		else
			csample = CNONE;

		if (rfcfg->verbose) {
			const char *fmt = "SAMPLER sync on %u-%uµs %s pulses, sampling %d %s bits, 0/1 divider pulse length %uµs\n";
			printf(fmt, rfcfg->sync_min, rfcfg->sync_max, csync, rfcfg->bits_to_sample, csample, rfcfg->bitdivider);
		}
	}

	while (1) {
		// wait for interrupt
		poll(fdset, 1, -1);

		// sample time + pin state
		pulse = gpio_micros_since(&last);
		pin = gpio_get(rfcfg->rx);

		// rewind & clear for next poll
		lseek(fdset[0].fd, 0, SEEK_SET);
		read(fdset[0].fd, &dummy, 1);

		// ignore noise & crap
		if (pulse < rfcfg->noise)
			continue;
		if (pulse > 22222)
			continue;

		if (rfcfg->analyzer_mode || rfcfg->pulse_counter_active) {

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
			} else if (rfcfg->verbose)
				printf("SAMPLER pulse_counter overflow %d\n", pulse_counter);

		} else {

			// error detection
			if (--state_reset == 0) {
				state = -1;
				if (rfcfg->verbose)
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
						timestamp_last_code = last;
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
						timestamp_last_code = last;
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
				if (rfcfg->sync_on_0)
					if (rfcfg->sync_min < pulse && pulse < rfcfg->sync_max) {
						bits = rfcfg->bits_to_sample;
						state = 127; // next is analyzer sample mode
						printf("\nSYN ");
					}

			} else {
				// HIGH pulse
				if (rfcfg->sync_on_1)
					if (rfcfg->sync_min < pulse && pulse < rfcfg->sync_max) {
						bits = rfcfg->bits_to_sample;
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
					if (rfcfg->sample_on_0) {
						if (pulse > rfcfg->bitdivider)
							code++;
						if (--bits == 0) {
							matrix_store(P_ANALYZE, code);
							timestamp_last_code = last;
							state = -2; // back to analyzer sync mode
							printf("\n");
						} else
							code <<= 1;
					}
				} else {
					// HIGH pulse
					printf("H%02d ", pulse_counter);
					if (rfcfg->sample_on_1) {
						if (pulse > rfcfg->bitdivider)
							code++;
						if (--bits == 0) {
							matrix_store(P_ANALYZE, code);
							timestamp_last_code = last;
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
	return (void*) 0;
}

void rfsniffer_realtime_test(int argc, char **argv) {
}
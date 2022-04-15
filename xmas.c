#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#include "flamingo.h"
#include "sensors.h"
#include "utils.h"
#include "xmas.h"

// TODO define channel status for each remote control unit
static char channel_status[128];

static pthread_t xmas_thread;

static void* xmas_loop(void *arg);

static void send_on(const timing_t *timing) {
	int index = timing->channel - 'A';
	if (!channel_status[index]) {
		xlog("flamingo_send_FA500 %d %c 1\n", timing->remote, timing->channel);
		flamingo_send_FA500(timing->remote, timing->channel, 1, -1);
		channel_status[index] = 1;
	}
}

static void send_off(const timing_t *timing) {
	int index = timing->channel - 'A';
	if (channel_status[index]) {
		xlog("flamingo_send_FA500 %d %c 0\n", timing->remote, timing->channel);
		flamingo_send_FA500(timing->remote, timing->channel, 0, -1);
		channel_status[index] = 0;
	}
}

static void process(struct tm *now, const timing_t *timing) {
	int afternoon = now->tm_hour < 12 ? 0 : 1;
	int curr = now->tm_hour * 60 + now->tm_min;
	int from = timing->on_h * 60 + timing->on_m;
	int to = timing->off_h * 60 + timing->off_m;
	int on = channel_status[(int) timing->channel - 'A'];

	if (from <= curr && curr <= to) {

		// ON time frame
		if (!on) {
			if (afternoon) {
				// evening: check if sundown is reached an switch on
				// xlog("in ON time, waiting for XMAS_SUNDOWN");
				if (sensors->bh1750_raw2 < XMAS_SUNDOWN) {
					xlog("reached XMAS_SUNDOWN at bh1750_raw2=%d", sensors->bh1750_raw2);
					send_on(timing);
				}
			} else {
				// morning: switch on
				xlog("reached ON trigger at %02d:%02d", timing->on_h, timing->on_m);
				send_on(timing);
			}
		}
	} else {

		// OFF time frame
		if (on) {
			if (afternoon) {
				// evening: switch off
				xlog("reached OFF trigger at %02d:%02d", timing->off_h, timing->off_m);
				send_off(timing);
			} else {
				// morning: check if sunrise is reached an switch off
				// xlog("in OFF time, waiting for XMAS_SUNRISE");
				if (sensors->bh1750_raw2 > XMAS_SUNRISE) {
					xlog("reached XMAS_SUNRISE at bh1750_raw2=%d", sensors->bh1750_raw2);
					send_off(timing);
				}
			}
		}
	}
}

int xmas_init() {
	ZERO(channel_status);
	if (pthread_create(&xmas_thread, NULL, &xmas_loop, NULL))
		xlog("Error creating thread");

	return 0;
}

void xmas_close() {
	if (pthread_cancel(xmas_thread))
		xlog("Error canceling thread");

	if (pthread_join(xmas_thread, NULL))
		xlog("Error joining thread");
}

static void* xmas_loop(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		xlog("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	// elevate realtime priority for our thread
	if (elevate_realtime(3) < 0) {
		xlog("elevate_realtime() error ?");
		return (void*) 0;
	}

	while (1) {
		time_t now_ts = time(NULL);
		struct tm *now = localtime(&now_ts);

		for (int i = 0; i < ARRAY_SIZE(timings); i++) {
			const timing_t *timing = &timings[i];

			if (!timing->active)
				continue;

			if (now->tm_wday != timing->wday)
				continue;

			// xlog("processing timing[%i]", i);
			process(now, timing);
		}

		sleep(60);
	}
}

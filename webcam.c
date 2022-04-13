#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#include "utils.h"
#include "sensors.h"
#include "webcam.h"

static int webcam_on;

static pthread_t webcam_thread;

static void* webcam_loop(void *arg);

static void start() {
	system(WEBCAM_START);
	webcam_on = 1;
	xlog("executed %s", WEBCAM_START);
}

static void start_reset() {
	system(WEBCAM_START_RESET);
	webcam_on = 1;
	xlog("executed %s", WEBCAM_START_RESET);
}

static void stop() {
	system(WEBCAM_STOP);
	webcam_on = 0;
	xlog("executed %s", WEBCAM_STOP);
}

static void stop_timelapse() {
	system(WEBCAM_STOP_TIMELAPSE);
	webcam_on = 0;
	xlog("executed %s", WEBCAM_STOP_TIMELAPSE);
}

int webcam_init() {
	webcam_on = 0;
	if (pthread_create(&webcam_thread, NULL, &webcam_loop, NULL)) {
		xlog("Error creating thread");
	}
	return 0;
}

void webcam_close() {
	if (pthread_cancel(webcam_thread)) {
		xlog("Error canceling thread");
	}
	if (pthread_join(webcam_thread, NULL)) {
		xlog("Error joining thread");
	}
	stop();
}

static void* webcam_loop(void *arg) {
	time_t now_ts;
	struct tm *now;
	int afternoon;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		xlog("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	// wait till network & nfs available
	sleep(15);

	// state unknown (e.g. system startup) --> check need for switching on
	if (sensors->bh1750_raw > WEBCAM_SUNRISE)
		start();
	else
		stop();

	while (1) {
		now_ts = time(NULL);
		now = localtime(&now_ts);
		afternoon = now->tm_hour < 12 ? 0 : 1;

		if (afternoon && webcam_on) {
			// evening and on --> check if need to switch off
			// xlog("awaiting WEBCAM_SUNDOWN at %i", value);
			if (sensors->bh1750_raw < WEBCAM_SUNDOWN) {
				xlog("reached WEBCAM_SUNDOWN at bh1750_raw=%d", sensors->bh1750_raw);
				stop_timelapse();
			}
		}

		if (!afternoon && !webcam_on) {
			// morning and off --> check if need to switch on
			// xlog("awaiting WEBCAM_SUNRISE at %i", value);
			if (sensors->bh1750_raw > WEBCAM_SUNRISE) {
				xlog("reached WEBCAM_SUNRISE at bh1750_raw=%d", sensors->bh1750_raw);
				start_reset();
			}
		}

		sleep(60);
	}
}

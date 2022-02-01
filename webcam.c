#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#include "utils.h"
#include "mcp3204.h"
#include "webcam.h"

static int webcam_on;

static pthread_t thread_webcam;

static void* webcam(void *arg);

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
	if (pthread_create(&thread_webcam, NULL, &webcam, NULL)) {
		xlog("Error creating thread");
	}
	return 0;
}

void webcam_close() {
	if (pthread_cancel(thread_webcam)) {
		xlog("Error canceling thread");
	}
	if (pthread_join(thread_webcam, NULL)) {
		xlog("Error joining thread");
	}
	stop();
}

static void* webcam(void *arg) {
	time_t now_ts;
	struct tm *now;
	int afternoon;
	int value;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		xlog("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	// wait till network & nfs available
	sleep(15);

	// state unknown (e.g. system startup) --> check need for switching on
	value = mcp3204_read();
	if (value == 0) {
		xlog("mcp3204_read returned 0 ?");
		return (void *) 0;
	} else if (value < WEBCAM_SUNRISE) {
		start();
	} else {
		stop();
	}

	while (1) {
		now_ts = time(NULL);
		now = localtime(&now_ts);
		afternoon = now->tm_hour < 12 ? 0 : 1;
		value = mcp3204_read();

		if (afternoon && webcam_on) {
			// evening and on --> check if need to switch off
			// xlog("awaiting WEBCAM_SUNDOWN at %i", value);
			if (value > WEBCAM_SUNDOWN) {
				xlog("reached WEBCAM_SUNDOWN at %i, wait 15 minutes before stopping webcam", value);
				sleep(60 * 15);
				stop_timelapse();
			}
		}

		if (!afternoon && !webcam_on) {
			// morning and off --> check if need to switch on
			// xlog("awaiting WEBCAM_SUNRISE at %i", value);
			if (value < WEBCAM_SUNRISE) {
				xlog("reached WEBCAM_SUNRISE at %i", value);
				start_reset();
			}
		}

		sleep(60);
	}
}

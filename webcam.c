#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <syslog.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#include "mcp3204.h"
#include "webcam.h"

static pthread_t thread_webcam;

static void* webcam(void *arg);

static void start() {
	system(WEBCAM_START);
	syslog(LOG_NOTICE, "executed %s", WEBCAM_START);
}

static void start_reset() {
	system(WEBCAM_START_RESET);
	syslog(LOG_NOTICE, "executed %s", WEBCAM_START_RESET);
}

static void stop() {
	system(WEBCAM_STOP);
	syslog(LOG_NOTICE, "executed %s", WEBCAM_STOP);
}

static void stop_timelapse() {
	syslog(LOG_NOTICE, "workaround: wait 15 minutes before stopping webcam...");
	sleep(60 * 15);
	system(WEBCAM_STOP_TIMELAPSE);
	syslog(LOG_NOTICE, "executed %s", WEBCAM_STOP_TIMELAPSE);
}

int webcam_init() {
	if (pthread_create(&thread_webcam, NULL, &webcam, NULL)) {
		syslog(LOG_WARNING, "Error creating thread");
	}

	return 0;
}

void webcam_close() {
	if (pthread_cancel(thread_webcam)) {
		syslog(LOG_WARNING, "Error canceling thread");
	}
	if (pthread_join(thread_webcam, NULL)) {
		syslog(LOG_WARNING, "Error joining thread");
	}
}

static void* webcam(void *arg) {
	time_t now;
	struct tm *now_tm;
	int state = -1;
	int value;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		syslog(LOG_ERR, "Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	// wait till network & nfs available
	sleep(15);

	// state unknown (e.g. system startup) --> check need for switching on
	value = mcp3204_read();
	if (value == 0) {
		syslog(LOG_NOTICE, "mcp3204_read returned 0 ?");
		return (void *) 0;
	} else if (value < WEBCAM_SUNRISE) {
		start();
		state = 1;
	} else {
		stop();
		state = 0;
	}

	while (1) {
		now = time(NULL);
		now_tm = localtime(&now);
		value = mcp3204_read();

		if (now_tm->tm_hour < 9 && state == 0) {
			// morning and off --> check if need to switch on
			// syslog(LOG_NOTICE, "awaiting WEBCAM_SUNRISE at %i", value);
			if (value < WEBCAM_SUNRISE) {
				syslog(LOG_NOTICE, "reached WEBCAM_SUNRISE at %i", value);
				start_reset();
				state = 1;
			}
		} else if (now_tm->tm_hour > 15 && state == 1) {
			// evening and on --> check if need to switch off
			// syslog(LOG_NOTICE, "awaiting WEBCAM_SUNDOWN at %i", value);
			if (value > WEBCAM_SUNDOWN) {
				syslog(LOG_NOTICE, "reached WEBCAM_SUNDOWN at %i", value);
				stop_timelapse();
				state = 0;
			}
		}

		sleep(60);
	}
}

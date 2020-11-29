#include "xmas.h"

#include <bits/types/struct_tm.h>
#include <bits/types/time_t.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include "mcp3204.h"

static char status[128];

static pthread_t thread_xmas;

static void* xmas(void *arg);

static void flamingosend(char *cmd) {
	char command[128];
	int i;

	strcpy(command, FLAMINGOSEND);
	strcat(command, " ");
	strcat(command, cmd);
	for (i = 0; i < 5; i++) {
		int ret = system(command);
		syslog(LOG_NOTICE, "executed %s returned %d", command, ret);
		sleep(1);
	}
}

static void send_on(char code) {
	if (!status[(int) code]) {
		char cmd[6] = "1 x 1\0";
		cmd[2] = code;
		flamingosend(cmd);
		status[(int) code] = 1;
	}
}

static void send_off(char code) {
	if (status[(int) code]) {
		char cmd[6] = "1 x 0\0";
		cmd[2] = code;
		flamingosend(cmd);
		status[(int) code] = 0;
	}
}

static void process(struct tm *now, const timing_t *timing) {
	int afternoon = now->tm_hour < 12 ? 0 : 1;
	int curr = now->tm_hour * 60 + now->tm_min;
	int from = timing->on_h * 60 + timing->on_m;
	int to = timing->off_h * 60 + timing->off_m;
	int on = status[(int) timing->code];
	int value;

	if (from <= curr && curr <= to) {
		// ON time frame
		if (!on) {
			if (afternoon) {
				// evening: check if sundown is reached an switch on
				// syslog(LOG_NOTICE, "in ON time, waiting for XMAS_SUNDOWN");
				value = mcp3204_read();
				if (value > XMAS_SUNDOWN) {
					syslog(LOG_NOTICE, "reached XMAS_SUNDOWN at %i", value);
					send_on(timing->code);
				}
			} else {
				// morning: switch on
				syslog(LOG_NOTICE, "reached ON trigger at %i:%i", timing->on_h, timing->on_m);
				send_on(timing->code);
			}
		}
	} else {
		// OFF time frame
		if (on) {
			if (afternoon) {
				// evening: switch off
				syslog(LOG_NOTICE, "reached OFF trigger at %i:%i", timing->off_h, timing->off_m);
				send_off(timing->code);
			} else {
				// morning: check if sunrise is reached an switch off
				// syslog(LOG_NOTICE, "in OFF time, waiting for XMAS_SUNRISE");
				value = mcp3204_read();
				if (value < XMAS_SUNRISE) {
					syslog(LOG_NOTICE, "reached XMAS_SUNRISE at %i", value);
					send_off(timing->code);
				}
			}
		}
	}
}

int xmas_init() {
	if (pthread_create(&thread_xmas, NULL, &xmas, NULL)) {
		syslog(LOG_WARNING, "Error creating thread");
	}
	ZERO(status);
	return 0;
}

void xmas_close() {
	if (pthread_cancel(thread_xmas)) {
		syslog(LOG_WARNING, "Error canceling thread");
	}
	if (pthread_join(thread_xmas, NULL)) {
		syslog(LOG_WARNING, "Error joining thread");
	}
}

static void* xmas(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		syslog(LOG_ERR, "Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	int value = mcp3204_read();
	if (value == 0) {
		syslog(LOG_NOTICE, "mcp3204_read returned 0 ?");
		return (void *) 0;
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

			// syslog(LOG_NOTICE, "processing timing[%i]", i);
			process(now, timing);
		}

		sleep(60);
	}
}

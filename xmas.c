#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <syslog.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>

#include "mcp3204.h"
#include "xmas.h"

static pthread_t thread_xmas;
static state_t a_state, b_state;

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

static void a_on() {
	if (a_state != on) {
		flamingosend("1 A 1");
		a_state = on;
	}
}

static void a_off() {
	if (a_state != off) {
		flamingosend("1 A 0");
		a_state = off;
	}
}

static void b_on() {
	if (b_state != on) {
		flamingosend("1 B 1");
		b_state = on;
	}
}

static void b_off() {
	if (b_state != off) {
		flamingosend("1 B 0");
		b_state = off;
	}
}

static bool is_active(int active, int fh, int fm, int th, int tm) {
	time_t now;
	struct tm *now_tm;

	now = time(NULL);
	now_tm = localtime(&now);

	int mod_start = fh * 60 + fm;
	int mod_end = th * 60 + tm;
	int mod = now_tm->tm_hour * 60 + now_tm->tm_min;

	if (active && mod_start <= mod && mod <= mod_end) {
		return true;
	} else {
		return false;
	}
}

int xmas_init() {
	a_state = unknown;
	b_state = unknown;

	if (pthread_create(&thread_xmas, NULL, &xmas, NULL)) {
		syslog(LOG_WARNING, "Error creating thread");
	}

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
	bool active_a, active_b;
	bool check_a, check_b;
	int value;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		syslog(LOG_ERR, "Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	value = mcp3204_read();
	if (value == 0) {
		syslog(LOG_NOTICE, "mcp3204_read returned 0 ?");
		return (void *) 0;
	}

	check_a = true;
	check_b = true;
	while (1) {
		active_a = is_active(A_ACTIVE, A_FROM_H, A_FROM_M, A_TO_H, A_TO_M);
		active_b = is_active(B_ACTIVE, B_FROM_H, B_FROM_M, B_TO_H, B_TO_M);

		if (active_a == true) {
			if (check_a == true) {
				value = mcp3204_read();
				if (value > XMAS_SUNDOWN) {
					syslog(LOG_NOTICE, "reached XMAS_SUNDOWN at %i", value);
					a_on();
					check_a = false; // once on stay on
				} else {
					a_off();
				}
			}
		} else if (active_b == true) {
			if (check_b == true) {
				value = mcp3204_read();
				if (value < XMAS_SUNRISE) {
					syslog(LOG_NOTICE, "reached XMAS_SUNRISE at %i", value);
					b_off();
					check_b = false; // once off stay off
				} else {
					b_on();
				}
			}
		} else {
			a_off();
			b_off();
			check_a = true;
			check_b = true;
		}

		sleep(60);
	}
}

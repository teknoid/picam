#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>

#include "mcp3204.h"
#include "utils.h"
#include "lumi.h"

static pthread_t thread_lumi;

static void* lumi(void *arg);

int lumi_init() {
	if (pthread_create(&thread_lumi, NULL, &lumi, NULL)) {
		syslog(LOG_WARNING, "Error creating thread");
	}
	return 0;
}

void lumi_close() {
	if (pthread_cancel(thread_lumi)) {
		syslog(LOG_WARNING, "Error canceling thread");
	}
	if (pthread_join(thread_lumi, NULL)) {
		syslog(LOG_WARNING, "Error joining thread");
	}
}

static void* lumi(void *arg) {
	char cvalue[5];
	int value;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		syslog(LOG_ERR, "Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	value = mcp3204_read();
	if (value == 0) {
		syslog(LOG_NOTICE, "mcp3204_read returned 0 ?");
		return (void*) 0;
	}

	while (1) {
		value = mcp3204_read();
		snprintf(cvalue, 4, "%d", value);

		create_sysfslike(DIRECTORY, "lumi", cvalue, "%s", "MCP3204");

		sleep(60);
	}
}

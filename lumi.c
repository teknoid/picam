#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>

#include "utils.h"
#include "mcp3204.h"
#include "lumi.h"

static pthread_t thread_lumi;

static void* lumi(void *arg);

int lumi_init() {
	if (pthread_create(&thread_lumi, NULL, &lumi, NULL))
		xlog("Error creating thread");

	return 0;
}

void lumi_close() {
	if (pthread_cancel(thread_lumi))
		xlog("Error canceling thread");

	if (pthread_join(thread_lumi, NULL))
		xlog("Error joining thread");
}

static void* lumi(void *arg) {
	char cvalue[5];
	uint16_t value;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		xlog("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	value = mcp3204_read();
	if (value == 0) {
		xlog("mcp3204_read returned 0 ?");
		return (void*) 0;
	}

	while (1) {
		value = mcp3204_read();
		snprintf(cvalue, 5, "%u", value);
		create_sysfslike(DIRECTORY, "lum_raw", cvalue, "%s", "MCP3204");

		value = ((ADC_MAX - value) * 100) / ADC_MAX;
		snprintf(cvalue, 5, "%u", value);
		create_sysfslike(DIRECTORY, "lum_percent", cvalue, "%s", "MCP3204");

		sleep(60);
	}
}

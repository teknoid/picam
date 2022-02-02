#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "rfsniffer.h"
#include "rfcodec.h"
#include "utils.h"
#include "frozen.h"

#define BUFFER		128

const char *fmt_message = "NEXUS {%d} 0x%08llx id=%d, channel=%d, battery=%s, temp=%02.1fC, hum=%d%%\n";

static rfsniffer_config_t *cfg;

void nexus_decode(uint8_t protocol, uint64_t raw, uint8_t repeat) {

	// sensor transmits 10 messages
	// 1st message get's lost due to sync on pause between repeats
	// we want at least 3 identical messages
	if (1 <= repeat && repeat < 3) {
		if (cfg->verbose)
			printf("NEXUS {%d} too few repeats, discard message 0x%08llx = %s\n", repeat, raw, printbits64(raw, 0x1011001101));
		return;
	}

	// https://github.com/merbanan/rtl_433/blob/master/src/devices/nexus.c
	// 9 nibbles: [id0] [id1] [flags] [temp0] [temp1] [temp2] [const] [humi0] [humi1]
	uint64_t code = raw;
	uint8_t h = code & 0xff;
	code >>= 8;
	code >>= 4; // always 1111 - used for message verification
	int t_raw = (int) (code & 0x0fff);
	code >>= 12;
	uint8_t c = code & 0x07;
	code >>= 3;
	uint8_t b = code & 0x01;
	code >>= 1;
	uint8_t i = code & 0xff;

	// calculate float temperature value
	float t = (t_raw & 0x0800) ? -0.1 * (0x0fff - t_raw) : 0.1 * t_raw;

	if (cfg->validate) {
		if ((raw & 0x0f00) != 0x0f00) {
			if (cfg->verbose)
				printf("NEXUS message verification failed 0x%08llx = %s\n", raw, printbits64(raw, 0x1011001101));
			return;
		}
	}

	// TODO timestamp via utils

	// create strings
	char craw[12], ctemp[6], chumi[3], cbatt[2];
	snprintf(craw, 12, "0x%08llx", raw);
	snprintf(ctemp, 6, "%02.1f", t);
	snprintf(chumi, 3, "%u", h);
	snprintf(cbatt, 2, "%u", b);

	if (!cfg->quiet)
		printf(fmt_message, repeat, raw, i, c, b ? "OK" : "LOW", t, h);

	if (cfg->json) {
		struct json_out jout = JSON_OUT_FILE(stdout);
		const char *fmt_json = "{ %Q:%Q, %Q:%Q, %Q:%d, %Q:%d, %Q:%d, %Q:%d, %Q:%Q, %Q:%d }\n";
		json_printf(&jout, fmt_json, "type", "NEXUS", "raw", craw, "repeat", repeat, "id", i, "channel", c, "battery", b, "temp", ctemp, "humi", h);
	}

	if (cfg->sysfslike) {
		// e.g. creates file entries like /tmp/NEXUS/231/0/temp
		create_sysfslike(cfg->sysfslike, "temp", ctemp, "%s%d%d", "NEXUS", i, c);
		create_sysfslike(cfg->sysfslike, "humi", chumi, "%s%d%d", "NEXUS", i, c);
		create_sysfslike(cfg->sysfslike, "batt", cbatt, "%s%d%d", "NEXUS", i, c);
	}

	if (cfg->rfsniffer_handler) {
		rfsniffer_event_t *e = malloc(sizeof(*e));
		memset(e, 0, sizeof(*e));

		e->raw = raw;
		e->protocol = protocol;
		e->repeat = repeat;
		e->device = i;
		e->channel = c;
		e->key = E_BATTERY;
		e->value = b;
		e->ikey1 = E_HUMIDITY;
		e->ivalue1 = h;
		e->fkey1 = E_TEMPERATURE;
		e->fvalue1 = t;

		char cmessage[BUFFER];
		snprintf(cmessage, BUFFER, fmt_message, repeat, raw, i, c, b ? "OK" : "LOW", t, h);
		e->message = cmessage;

		(cfg->rfsniffer_handler)(e);
	}
}

int nexus_test(int argc, char **argv) {
	return 0;
}

void nexus_cfg(rfsniffer_config_t *master) {
	cfg = master;
}

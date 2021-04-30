/***
 *
 * decoder + encoder routines for rfsniffer
 *
 * Copyright (C) 04/2021 by teknoid
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rfsniffer.h"
#include "utils.h"
#include "flamingo.h"
#include "frozen.h"

#define BUFFER		128

static rfsniffer_config_t *cfg;

static unsigned long decode_0110(unsigned long long in) {
	unsigned long long shift;
	unsigned char mask = 0b11;
	unsigned char bit0 = 0b01;
	unsigned char bit1 = 0b10;
	unsigned char count = 64;
	unsigned long out = 0;

	// printf("\n0110 %s", printbits64(in, 0x0001000100010001));
	while (count > 0) {
		shift = (in >> 62) & mask;

		out <<= 1;
		if (shift == bit0) {
			out += 0;
		} else if (shift == bit1) {
			out += 1;
		} else {
			if (cfg->verbose)
				printf("0110 decode error %s\n", printbits64(in, 0x0001000100010001));
			return 0;
		}
		count -= 2;
		in <<= 2;
	}
	// printf("--> %s\n", printbits(out, 0x00010001));
	return out;
}

static void decode_nexus(unsigned char protocol, unsigned long long raw, unsigned char repeat) {

	// sensor transmits 10 messages
	// 1st message get's lost due to sync on pause between repeats
	// we want at least 3 identical messages
	if (1 <= repeat && repeat < 3) {
		if (cfg->verbose)
			printf("NEXUS {%d} too few repeats, discard message 0x%08llx = %s\n", repeat, raw, printbits64(raw, 0x1011001101));
		return;
	}

	if ((raw & 0x0f00) != 0x0f00) {
		if (cfg->verbose)
			printf("NEXUS message verification failed 0x%08llx = %s\n", raw, printbits64(raw, 0x1011001101));
		return;
	}

	// https://github.com/merbanan/rtl_433/blob/master/src/devices/nexus.c
	// 9 nibbles: [id0] [id1] [flags] [temp0] [temp1] [temp2] [const] [humi0] [humi1]
	unsigned long long code = raw;
	unsigned char h = code & 0xff;
	code >>= 8;
	code >>= 4; // always 1111 - used for message verification
	short t_raw = (short) (code & 0x0fff);
	code >>= 12;
	unsigned char c = code & 0x07;
	code >>= 3;
	unsigned char b = code & 0x01;
	code >>= 1;
	unsigned char i = code & 0xff;

	// calculate float temperature value
	float t = (t_raw & 0x0800) ? -0.1 * (0x0fff - t_raw) : 0.1 * t_raw;

	// TODO timestamp via utils

	// create strings
	char cmessage[BUFFER], craw[12], ctemp[6], chumi[3], cbatt[2];
	snprintf(craw, 12, "0x%08llx", raw);
	snprintf(ctemp, 6, "%02.1f", t);
	snprintf(chumi, 3, "%u", h);
	snprintf(cbatt, 2, "%u", b);
	const char *fmt_message = "NEXUS {%d} 0x%08llx Id = %d, Channel = %d, Battery = %s, Temp = %02.1fC, Hum = %d%%\n";
	snprintf(cmessage, BUFFER, fmt_message, repeat, raw, i, c, b ? "OK" : "LOW", t, h);

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
		e->message = cmessage;
		e->protocol = protocol;
		e->repeat = repeat;
		e->raw = raw;
		e->device = i;
		e->channel = c;
		e->key = E_BATTERY;
		e->value = b;
		e->ikey1 = E_HUMIDITY;
		e->ivalue1 = h;
		e->fkey1 = E_TEMPERATURE;
		e->fvalue1 = t;
		(cfg->rfsniffer_handler)(e);
	}
}

static void decode_flamingo28(unsigned char protocol, unsigned long long raw, unsigned char repeat) {
	unsigned long code;
	unsigned int xmitter;
	unsigned char channel, command, payload, rolling;

	code = decrypt(raw);
	decode_FA500(code, &xmitter, &channel, &command, &payload, &rolling);

	// create strings
	char cmessage[BUFFER];
	const char *fmt_message = "FLAMINGO28 0x%08llx Id = 0x%x, Channel = %d, Command = %d, Payload = 0x%02x, Rolling = %d\n";
	snprintf(cmessage, BUFFER, fmt_message, raw, xmitter, channel, command, payload, rolling);

	if (cfg->json) {
		char format[BUFFER], craw[12], cid[12];
		snprintf(craw, 12, "0x%08llx", raw);
		snprintf(cid, 12, "0x%04x", xmitter);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%Q, %Q:%d, %Q:%d, %Q:%d, %Q:%d }\n");
		json_printf(&jout, format, "type", "FLAMINGO28", "raw", craw, "id", cid, "channel", channel, "command", command, "payload", payload, "rolling", rolling);
	}

	if (cfg->rfsniffer_handler) {
		rfsniffer_event_t *e = malloc(sizeof(*e));
		e->message = cmessage;
		e->protocol = protocol;
		e->repeat = repeat;
		e->raw = raw;
		e->device = xmitter;
		e->channel = channel;
		e->key = E_BUTTON;
		e->value = command;
		e->ikey1 = E_PAYLOAD;
		e->ivalue1 = payload;
		e->ikey2 = E_ROLLING;
		e->ivalue2 = rolling;
		(cfg->rfsniffer_handler)(e);
	}
}

static void decode_flamingo24(unsigned char protocol, unsigned long long raw, unsigned char repeat) {

	// TODO decode

	if (cfg->json) {
		char format[BUFFER], craw[12];
		snprintf(craw, 12, "0x%08llx", raw);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu }\n");
		json_printf(&jout, format, "type", "FLAMINGO24", "raw", craw);
	}

	if (cfg->rfsniffer_handler) {
		rfsniffer_event_t *e = malloc(sizeof(*e));
		e->protocol = protocol;
		e->repeat = repeat;
		e->raw = raw;
	}
}

static void decode_flamingo32(unsigned char protocol, unsigned long long raw, unsigned char repeat) {
	unsigned long code = decode_0110(raw);
	if (code == 0)
		return;

	unsigned int xmitter;
	unsigned char channel, command, payload;
	unsigned long code_save = code;
	channel = code & 0x0f;
	code >>= 4; // channel
	command = code & 0x0f;
	code >>= 4; // channel
	xmitter = code & 0xffff;
	code >>= 16; // xmitter
	payload = code & 0xff;

	// create strings
	char cmessage[BUFFER];
	const char *fmt_message = "FLAMINGO32 0x%08llx Id = 0x%x, Channel = %d, Command = %d, Payload = 0x%02x\n";
	snprintf(cmessage, BUFFER, fmt_message, raw, xmitter, channel, command, payload);

	if (cfg->json) {
		char format[BUFFER], craw[12], cid[12];
		snprintf(craw, 12, "0x%04lx", code_save);
		snprintf(cid, 12, "0x%04x", xmitter);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%Q, %Q:%d, %Q:%d, %Q:%d }\n");
		json_printf(&jout, format, "type", "FLAMINGO32", "raw", craw, "id", cid, "channel", channel, "command", command, "payload", payload);
	}

	if (cfg->rfsniffer_handler) {
		rfsniffer_event_t *e = malloc(sizeof(*e));
		e->message = cmessage;
		e->protocol = protocol;
		e->repeat = repeat;
		e->raw = raw;
		e->device = xmitter;
		e->channel = channel;
		e->key = E_BUTTON;
		e->value = command;
		e->ikey1 = E_PAYLOAD;
		e->ivalue1 = payload;
		(cfg->rfsniffer_handler)(e);
	}
}

static void decode_anaylzer(unsigned char protocol, unsigned long long raw, unsigned char r) {
	if (!cfg->quiet)
		printf("ANALYZER 0x%08llx %s\n", raw, printbits64(raw, 0x0101010101010101));
}

void rfcodec_decode(unsigned char protocol, unsigned long long code, unsigned char repeat) {
	if (!code) {
		if (cfg->verbose)
			printf("DECODE received empty message\n");
		return;
	}

	// dispatch to corresponding protocol decoder
	switch (protocol) {
	case P_NEXUS:
		return decode_nexus(protocol, code, repeat);
	case P_FLAMINGO28:
		return decode_flamingo28(protocol, code, repeat);
	case P_FLAMINGO24:
		return decode_flamingo24(protocol, code, repeat);
	case P_FLAMINGO32:
		return decode_flamingo32(protocol, code, repeat);
	case P_ANALYZE:
		return decode_anaylzer(protocol, code, repeat);
	default:
		if (!cfg->quiet)
			printf("DECODE no decoder configured for protocol %d 0x%0llx\n", protocol, code);
	}
}

void rfcodec_set_config(rfsniffer_config_t *master_cfg) {
	cfg = master_cfg;
}

/***
 *
 * Library for encoding and decding of 433MHz ELRO Flamingo home device messages
 *
 * (C) Copyright 2022 Heiko Jehmlich <hje@jecons.de>
 *
 * tested with following devices
 *
 * FA500R REMOTE
 * FA500S WIRELESS SWITCH UNIT
 * SF-500R CONTROL
 * SF-500P SWITCH
 *
 *
 * FA500R 28bit message pattern:
 *
 * 0000 0000000000000000 0000 XXXX	channel
 * 0000 0000000000000000 00XX 0000	command
 * 0000 0000000000000000 XX00 0000	rolling code id
 * 0000 XXXXXXXXXXXXXXXX 0000 0000	transmitter id
 * XXXX 0000000000000000 0000 0000	payload
 *
 *
 * SF-500R 32bit message pattern - guessed - looks like transmitter id is moved left
 *
 * 0000000000000000 00000000 0000 XXXX	channel
 * 0000000000000000 00000000 XXXX 0000	command
 * 0000000000000000 XXXXXXXX 0000 0000	payload
 * XXXXXXXXXXXXXXXX 00000000 0000 0000	transmitter id
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "rfsniffer.h"
#include "rfcodec.h"
#include "utils.h"
#include "frozen.h"
#include "flamingo.h"

#define BUFFER				128

// flamingo encryption key
static const uint8_t CKEY[16] = { 9, 6, 3, 8, 10, 0, 2, 12, 4, 14, 7, 5, 1, 15, 11, 13 };

// flamingo decryption key (invers encryption key - exchanged index & value)
static const uint8_t DKEY[16] = { 5, 12, 6, 2, 8, 11, 1, 10, 3, 0, 4, 14, 7, 15, 9, 13 };

static const char *fmt_message28 = "FLAMINGO28 {%d} 0x%08lx id=%04x, chan=%02d, cmd=%d, pay=0x%02x, roll=%d\n";
static const char *fmt_message32 = "FLAMINGO32 {%d} 0x%08lx id=%04x, chan=%02d, cmd=%d, pay=0x%02x\n";

extern rfsniffer_config_t *rfcfg;

static void flamingo_test_handler(rfsniffer_event_t *e) {
	// re-encode the message for visual compare
	if (e->protocol == P_FLAMINGO28)
		flamingo28_encode(e->device, e->channel, e->value, e->ivalue1, e->ivalue2);
	if (e->protocol == P_FLAMINGO32)
		flamingo32_encode(e->device, e->channel, e->value, e->ivalue1);
	free(e);
}

static uint32_t encrypt(uint32_t message) {
	uint32_t code = 0;
	uint8_t n[7];
	int i, r, idx;

	// split into nibbles
	for (i = 0; i < sizeof(n); i++)
		n[i] = message >> (4 * i) & 0x0F;

	// XOR encryption 2 rounds
	for (r = 0; r <= 1; r++) {					// 2 encryption rounds
		idx = (n[0] - r + 1) & 0x0F;
		n[0] = CKEY[idx];						// encrypt first nibble
		for (i = 1; i <= 5; i++) {				// encrypt 4 nibbles
			idx = ((n[i] ^ n[i - 1]) - r + 1) & 0x0F;
			n[i] = CKEY[idx];					// crypted with predecessor & key
		}
	}
	n[6] = n[6] ^ 9;							// no  encryption

	// build encrypted message
	code = (n[6] << 24) | (n[5] << 20) | (n[4] << 16) | (n[3] << 12) | (n[2] << 8) | (n[1] << 4) | n[0];

	// shift 2 bits right & copy lowest 2 bits of n[0] in msg bit 27/28
	code = (code >> 2) | ((code & 3) << 0x1A);

	return code;
}

static uint32_t decrypt(uint32_t code) {
	uint32_t message = 0;
	uint8_t n[7];
	int i, r;

	//shift 2 bits left & copy bit 27/28 to bit 1/2
	code = ((code << 2) & 0x0FFFFFFF) | ((code & 0xC000000) >> 0x1A);

	// split into nibbles
	for (i = 0; i < sizeof(n); i++)
		n[i] = code >> (4 * i) & 0x0F;

	n[6] = n[6] ^ 9;							// no decryption
	// XOR decryption 2 rounds
	for (r = 0; r <= 1; r++) {					// 2 decryption rounds
		for (i = 5; i >= 1; i--) {					// decrypt 4 nibbles
			n[i] = ((DKEY[n[i]] - r) & 0x0F) ^ n[i - 1];	// decrypted with predecessor & key
		}
		n[0] = (DKEY[n[0]] - r) & 0x0F;			// decrypt first nibble
	}

	// build message
	for (i = sizeof(n) - 1; i >= 0; i--) {
		message |= n[i];
		if (i)
			message <<= 4;
	}

	return message;
}

void flamingo28_decode(uint8_t protocol, uint64_t raw, uint8_t repeat) {
	uint32_t code = (uint32_t) raw;
	uint32_t message = decrypt(code);

	uint8_t payload = (message >> 24) & 0x0F;
	uint16_t xmitter = (message >> 8) & 0xFFFF;
	uint8_t rolling = (message >> 6) & 0x03;
	uint8_t command = (message >> 4) & 0x03;
	uint8_t channel = message & 0x0F;

	if (rfcfg->verbose)
		printf("F28 %04x %02d %d 0 %s <= 0x%08x <= 0x%08x\n", xmitter, channel, command, printbits(message, SPACEMASK_FA500), message, code);

	if (rfcfg->validate) {
		char valid = 0;
		for (int i = 0; i < ARRAY_SIZE(REMOTES); i++)
			if (xmitter == REMOTES[i])
				valid = 1;

		if (!valid)
			if (rfcfg->verbose)
				printf("FLAMINGO28 discard message, unknown transmitter ID 0x%02x\n", xmitter);

		if (!valid)
			return;
	}

	if (!rfcfg->quiet)
		printf(fmt_message28, repeat, message, xmitter, channel, command, payload, rolling);

	if (rfcfg->json) {
		char format[BUFFER], craw[12], cid[12];
		snprintf(craw, 12, "0x%08llx", raw);
		snprintf(cid, 12, "0x%04x", xmitter);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%Q, %Q:%d, %Q:%d, %Q:%d, %Q:%d }\n");
		json_printf(&jout, format, "type", "FLAMINGO28", "raw", craw, "id", cid, "channel", channel, "command", command, "payload", payload, "rolling", rolling);
	}

	if (rfcfg->rfsniffer_handler) {
		rfsniffer_event_t *e = malloc(sizeof(*e));
		memset(e, 0, sizeof(*e));

		e->raw = raw;
		e->code = message;
		e->protocol = protocol;
		e->repeat = repeat;
		e->device = xmitter;
		e->channel = channel;
		e->key = E_BUTTON;
		e->value = command;
		e->ikey1 = E_PAYLOAD;
		e->ivalue1 = payload;
		e->ikey2 = E_ROLLING;
		e->ivalue2 = rolling;

		char cmessage[BUFFER]; // TODO malloc
		snprintf(cmessage, BUFFER, fmt_message28, repeat, message, xmitter, channel, command, payload, rolling);
		e->message = cmessage;

		(rfcfg->rfsniffer_handler)(e);
	}
}

void flamingo32_decode(uint8_t protocol, uint64_t raw, uint8_t repeat) {
	uint32_t message = (uint32_t) raw;

	uint8_t payload = (message >> 24) & 0x0F;
	uint16_t xmitter = (message >> 8) & 0xFFFF;
	uint8_t command = (message >> 4) & 0x0F;
	uint8_t channel = message & 0x0F;

	if (rfcfg->verbose)
		printf("F32 %04x %02d %d %s <= 0x%08x\n", xmitter, channel, command, printbits(message, SPACEMASK_FA500), message);

	if (rfcfg->validate) {
		char valid = 0;
		for (int i = 0; i < ARRAY_SIZE(REMOTES); i++)
			if (xmitter == REMOTES[i])
				valid = 1;

		if (!valid)
			if (rfcfg->verbose)
				printf("FLAMINGO32 discard message, unknown transmitter ID 0x%02x\n", xmitter);

		if (!valid)
			return;
	}

	if (!rfcfg->quiet)
		printf(fmt_message32, repeat, message, xmitter, channel, command, payload);

	if (rfcfg->json) {
		char format[BUFFER], craw[12], cid[12];
		snprintf(craw, 12, "0x%08x", message);
		snprintf(cid, 12, "0x%04x", xmitter);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu, %Q:%Q, %Q:%d, %Q:%d, %Q:%d }\n");
		json_printf(&jout, format, "type", "FLAMINGO32", "raw", craw, "id", cid, "channel", channel, "command", command, "payload", payload);
	}

	if (rfcfg->rfsniffer_handler) {
		rfsniffer_event_t *e = malloc(sizeof(*e));
		memset(e, 0, sizeof(*e));

		e->raw = raw;
		e->code = message;
		e->protocol = protocol;
		e->repeat = repeat;
		e->device = xmitter;
		e->channel = channel;
		e->key = E_BUTTON;
		e->value = command;
		e->ikey1 = E_PAYLOAD;
		e->ivalue1 = payload;

		char cmessage[BUFFER]; // TODO malloc
		snprintf(cmessage, BUFFER, fmt_message32, repeat, message, xmitter, channel, command, payload);
		e->message = cmessage;

		(rfcfg->rfsniffer_handler)(e);
	}
}

void flamingo24_decode(uint8_t protocol, uint64_t raw, uint8_t repeat) {

	// TODO decode

	if (rfcfg->json) {
		char format[BUFFER], craw[12];
		snprintf(craw, 12, "0x%08llx", raw);
		struct json_out jout = JSON_OUT_FILE(stdout);
		snprintf(format, BUFFER, "%s", "{ %Q:%Q, %Q:%Q, %Q:%lu }\n");
		json_printf(&jout, format, "type", "FLAMINGO24", "raw", craw);
	}

	if (rfcfg->rfsniffer_handler) {
		rfsniffer_event_t *e = malloc(sizeof(*e));
		e->protocol = protocol;
		e->repeat = repeat;
		e->raw = raw;
	}
}

uint32_t flamingo28_encode(uint16_t xmitter, char channel, char command, char payload, char rolling) {
	uint32_t message = (payload & 0x0F) << 24 | xmitter << 8 | (rolling << 6 & 0xC0) | (command & 0x03) << 4 | (channel & 0x0F);
	uint32_t code = encrypt(message);

	if (rfcfg->verbose)
		printf("F28 %04x %02d %d %d %s => 0x%08x => 0x%08x\n", xmitter, channel, command, rolling, printbits(message, SPACEMASK_FA500), message, code);

	if (!rfcfg->quiet)
		printf(fmt_message28, 0, code, xmitter, channel, command, payload, rolling);

	return code;
}

uint32_t flamingo32_encode(uint16_t xmitter, char channel, char command, char payload) {
	uint32_t message = (payload & 0x0F) << 24 | xmitter << 8 | (command & 0x0F) << 4 | (channel & 0x0F);

	if (rfcfg->verbose)
		printf("F32 %04x %02d %d %s => 0x%08x\n", xmitter, channel, command, printbits(message, SPACEMASK_FA500), message);

	if (!rfcfg->quiet)
		printf(fmt_message32, 0, message, xmitter, channel, command, payload);

	return message;
}

uint32_t flamingo24_encode(uint16_t xmitter, char channel, char command, char payload) {
	// TODO
//	if (!cfg->quiet)
//		printf("SF500 %d %d %d => 0x%08x %s", xmitter, channel, command, message, printbits(message, SPACEMASK_SF500));
	return 0;
}

int flamingo_test(int argc, char **argv) {
	uint32_t bruteforce[4] = { 0x0e6bd68d, 0x0e7be29d, 0x0e7be29d, 0x0e763e15 };
	uint32_t deadbeef[3] = { 0x0000dead, 0x000beef0, 0x0affe000 };
	uint32_t code, message;

	// modify configuration for our test procedure
	rfcfg->rfsniffer_handler = &flamingo_test_handler;
	rfcfg->validate = 0;

	printf("\n*** test message encode + decode + re-encode ***\n");
	code = flamingo28_encode(REMOTES[0], 2, 1, 0x05, 0);
	flamingo28_decode(P_FLAMINGO28, code, 1);
	code = flamingo32_encode(REMOTES[0], 2, 1, 0x05);
	flamingo32_decode(P_FLAMINGO32, code, 1);

	printf("\n*** test rolling code encryption & decryption ***\n");
	for (int r = 0; r < 4; r++) {
		code = flamingo28_encode(REMOTES[0], 2, 0, 0, r);
		flamingo28_decode(P_FLAMINGO28, code, 1);
	}

	printf("\n*** test brute-force encryption ***\n");
	for (int y = 0; y < 0x0F; y++)
		for (int x = 0; x < 0xFF; x++) {
			message = y << 24 | REMOTES[0] << 8 | x;
			code = encrypt(message);
			if (code == bruteforce[0] || code == bruteforce[1] || code == bruteforce[2] || code == bruteforce[4])
				printf("%s => 0x%08x => 0x%08x\n", printbits(message, SPACEMASK_FA500), message, code);
		}

	printf("\n*** test dead+beef+affe ***\n");
	for (int i = 0; i < ARRAY_SIZE(deadbeef); i++) {
		code = deadbeef[i];
		flamingo28_decode(P_FLAMINGO28, code, 1);
	}

	if (argc > 2)
		for (int i = 2; i < argc; i++)
			if (strlen(argv[i]) > 5) {
				printf("\n*** decode command line argument %s ***\n", argv[i]);
				flamingo28_decode(P_FLAMINGO28, strtoul(argv[i], NULL, 0), 1);
			}

	return EXIT_SUCCESS;
}

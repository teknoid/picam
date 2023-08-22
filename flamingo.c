/***
 *
 * Library for sending 433MHz ELRO Flamingo home device messages
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
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "gpio.h"
#include "utils.h"
#include "flamingo.h"

// flamingo encryption key
static const uint8_t CKEY[16] = { 9, 6, 3, 8, 10, 0, 2, 12, 4, 14, 7, 5, 1, 15, 11, 13 };

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

static uint32_t flamingo28_encode(uint16_t xmitter, char channel, char command, char payload, char rolling) {
	uint32_t message = (payload & 0x0F) << 24 | xmitter << 8 | (rolling << 6 & 0xC0) | (command & 0x03) << 4 | (channel & 0x0F);
	uint32_t code = encrypt(message);
	return code;
}

static uint32_t flamingo32_encode(uint16_t xmitter, char channel, char command, char payload) {
	uint32_t message = (payload & 0x0F) << 24 | xmitter << 8 | (command & 0x0F) << 4 | (channel & 0x0F);
	return message;
}

// TODO
static uint32_t flamingo24_encode(uint16_t xmitter, char channel, char command, char payload) {
	return 0;
}

#ifdef FLAMINGO_MAIN
static int usage() {
	printf("Usage: flamingo <remote> <channel> <command> [rolling]\n");
	printf("    <remote>  1, 2, 3, ...\n");
	printf("    <channel> A, B, C, D\n");
	printf("    <command> 0 - off, 1 - on\n");
	printf("    [rolling]  rolling code index, 0...3\n");
	return EXIT_FAILURE;
}

static int flamingo_main(int argc, char **argv) {
	if (argc < 1)
		return usage();

	if (argc >= 4) {

		// SEND mode

		// remote 1, 2, 3, ...
		int remote = atoi(argv[1]);
		if (remote < 1 || remote > sizeof(REMOTES)) {
			printf("unknown remote %i\n", remote);
			usage();
			return EINVAL;
		}

		// channel A, B, C, D
		char *c = argv[2];
		char channel = toupper(c[0]);
		if (channel < 'A' || channel > 'D') {
			printf("channel not supported %c\n", channel);
			usage();
			return EINVAL;
		}

		// command 0 = off, 1 = on
		int command = atoi(argv[3]);
		if (!(command == 0 || command == 1)) {
			printf("wrong command %i\n", command);
			usage();
			return EINVAL;
		}

		// optional: send rolling code index
		int rolling = -1;
		if (argv[4] != NULL) {
			rolling = atoi(argv[4]);
			if (rolling < 0 || rolling > 3) {
				printf("wrong rolling code index %i\n", rolling);
				usage();
				return EINVAL;
			}
		}

		// initialize without receive support (clear pattern + default handler)
		flamingo_init();
		flamingo_send_FA500(remote, channel, command, rolling);
		flamingo_close();
		return EXIT_SUCCESS;

	} else
		return usage();
}
#endif

void flamingo_send_FA500(int remote, char channel, int command, int rolling) {
	if (remote < 1 || remote > ARRAY_SIZE(REMOTES))
		return;

	if (channel < 'A' || channel > 'P')
		return;

	uint16_t transmitter = REMOTES[remote - 1];
	if (0 <= rolling && rolling <= 4) {
		// send specified rolling code
		uint32_t c28 = flamingo28_encode(transmitter, channel - 'A' + 1, command ? 2 : 0, 0, rolling);
		gpio_flamingo_v1(TX, c28, 28, 4, T1);

		uint32_t m32 = flamingo32_encode(transmitter, channel - 'A' + 1, command, 0);
		gpio_flamingo_v2(TX, m32, 32, 3, T2H, T2L);
	} else {
		// send all rolling codes in sequence
		for (int r = 0; r < 4; r++) {
			uint32_t c28 = flamingo28_encode(transmitter, channel - 'A' + 1, command ? 2 : 0, 0, r);
			gpio_flamingo_v1(TX, c28, 28, 4, T1);

			uint32_t m32 = flamingo32_encode(transmitter, channel - 'A' + 1, command, 0);
			gpio_flamingo_v2(TX, m32, 32, 3, T2H, T2L);
			sleep(1);
		}
	}
}

// TODO
void flamingo_send_SF500(int remote, char channel, int command) {
	if (remote < 1 || remote > ARRAY_SIZE(REMOTES))
		return;

	uint16_t transmitter = REMOTES[remote - 1];
	uint32_t message = flamingo24_encode(transmitter, channel - 'A' + 1, command, 0);
	gpio_flamingo_v2(TX, message, 32, 5, T2H, T2L);
}

int flamingo_init() {
	// elevate realtime priority for sending thread
	if (elevate_realtime(3) < 0)
		return -2;

	// GPIO pin connected to 433MHz receiver+sender module
	gpio_configure(RX, 0, 0, 0);
	gpio_configure(TX, 1, 0, 0);

	return 0;
}

void flamingo_close() {
}

#ifdef FLAMINGO_MAIN
int main(int argc, char *argv[]) {
	return flamingo_main(argc, argv);
}
#endif

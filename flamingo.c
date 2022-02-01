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
#include "rfsniffer.h"
#include "rfcodec.h"
#include "flamingo.h"

#define RX	"GPIO27"
#define TX	"GPIO17"

static rfsniffer_config_t *cfg;

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
		gpio_flamingo_v1(cfg->tx, c28, 28, 4, T1);

		uint32_t m32 = flamingo32_encode(transmitter, channel - 'A' + 1, command, 0);
		gpio_flamingo_v2(cfg->tx, m32, 32, 3, T2H, T2L);
	} else {
		// send all rolling codes in sequence
		for (int r = 0; r < 4; r++) {
			uint32_t c28 = flamingo28_encode(transmitter, channel - 'A' + 1, command ? 2 : 0, 0, r);
			gpio_flamingo_v1(cfg->tx, c28, 28, 4, T1);

			uint32_t m32 = flamingo32_encode(transmitter, channel - 'A' + 1, command, 0);
			gpio_flamingo_v2(cfg->tx, m32, 32, 3, T2H, T2L);
			sleep(1);
		}
	}
}

void flamingo_send_SF500(int remote, char channel, int command) {
	if (remote < 1 || remote > ARRAY_SIZE(REMOTES))
		return;

	uint16_t transmitter = REMOTES[remote - 1];
	uint32_t message = flamingo24_encode(transmitter, channel - 'A' + 1, command, 0);
	gpio_flamingo_v2(cfg->tx, message, 32, 5, T2H, T2L);
}

int flamingo_init() {
	if (gpio_init() < 0)
		return -1;

	// elevate realtime priority for sending thread
	if (elevate_realtime(3) < 0)
		return -2;

	// initialize a default configuration if not yet done
	if (cfg == NULL)
		cfg = rfsniffer_default_config();

	// GPIO pin connected to 433MHz sender module
	gpio_configure(cfg->tx, 1, 0, 0);

	return 0;
}

void flamingo_close() {
	free(cfg);
}

void flamingo_config(void *master) {
	cfg = (rfsniffer_config_t*) master;
}

#ifdef FLAMINGO_MAIN
int main(int argc, char *argv[]) {
	return flamingo_main(argc, argv);
}
#endif

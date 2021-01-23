#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "flamingo.h"

static void usage() {
	printf("Usage: flamingosend <remote> <channel> <command> [rolling]\n");
	printf("  <remote>  1, 2, 3, ...\n");
	printf("  <channel> A, B, C, D\n");
	printf("  <command> 0 - off, 1 - on\n");
	printf("  [rolling]  rolling code index, 0...3\n");
}

int main(int argc, char *argv[]) {
	if (argc < 4) {
		usage();
		return EXIT_FAILURE;
	}

	// initialize without receive support
	flamingo_init(0, 0);

	// remote 1, 2, 3, ...
	int remote = atoi(argv[1]);
	if (remote < 1 || remote > sizeof(REMOTES)) {
		printf("unknown remote %i\n", remote);
		usage();
		return EINVAL;
	}

	// channel A, B, C, D
	char* c = argv[2];
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

	flamingo_send_FA500(remote, channel, command, rolling);
}

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>

#include "flamingo.h"

static void usage() {
	printf("Usage: flamingoread -1 | -2 | -3 | -4\n");
	printf("  -1 ... detect 28bit rolling codes, rc1 pattern\n");
	printf("  -2 ... detect 32bit messages, rc2 pattern\n");
	printf("  -3 ... detect 32bit multibit messages, rc3 pattern (encoding still unknown)\n");
	printf("  -4 ... detect 24bit messages, rc4 pattern\n");
}

void handler(unsigned int xmitter, unsigned char channel, unsigned char command, unsigned char payload) {
	printf("received Transmitter-Id = 0x%x, channel = %d, command = %d, payload = 0x%02x\n", xmitter, channel, command, payload);
}

int main(int argc, char **argv) {
	int pattern;

	// parse command line arguments
	int c = getopt(argc, argv, "1234");
	switch (c) {
	case '1':
		pattern = 1;
		break;
	case '2':
		pattern = 2;
		break;
	case '3':
		pattern = 3;
		break;
	case '4':
		pattern = 4;
		break;
	default:
		usage();
		return EXIT_FAILURE;
	}

	// initialize with receive support: pattern to listen on and a handler routine
	flamingo_init(pattern, &handler);

	pause();

	flamingo_close();
	return EXIT_SUCCESS;
}

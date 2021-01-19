#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <wiringPi.h>

#include "utils.h"
#include "flamingo.h"
#include "flamingocrypt.h"

static struct timeval tNow, tLong, tEnd;

static void _delay(unsigned int millis) {
	gettimeofday(&tNow, NULL);
	tLong.tv_sec = millis / 1000000;
	tLong.tv_usec = millis % 1000000;
	timeradd(&tNow, &tLong, &tEnd);
	while (timercmp(&tNow, &tEnd, <)) {
		gettimeofday(&tNow, NULL);
	}
}

static void send28(unsigned int txid, unsigned char channel, unsigned char command, unsigned char rolling) {
	unsigned long message = encode(txid, channel, command, 0);
	unsigned long code = encrypt(message, rolling);

	printf("sending %s => 0x%08lx => 0x%08lx\n", printbits(message, 0x01000110), message, code);

	for (int repeat = 1; repeat <= 4; repeat++) {
		unsigned long mask = 1 << 27;

		// sync
		digitalWrite(TX, 1);
		_delay(P2824);
		digitalWrite(TX, 0);
		_delay(P2824X15);

		while (mask) {
			if (code & mask) {
				// 1
				digitalWrite(TX, 1);
				_delay(P2824X3);
				digitalWrite(TX, 0);
				_delay(P2824);
			} else {
				// 0
				digitalWrite(TX, 1);
				_delay(P2824);
				digitalWrite(TX, 0);
				_delay(P2824X3);
			}

			mask = mask >> 1;
		}
	}
}

static void send24(char remote, char channel, char command) {
	unsigned long code = 0x00144114;

	for (int repeat = 1; repeat <= 5; repeat++) {
		unsigned long mask = 1 << 23;

		// sync
		digitalWrite(TX, 1);
		_delay(P2824);
		digitalWrite(TX, 0);
		_delay(P2824X31);

		while (mask) {
			if (code & mask) {
				// 1
				digitalWrite(TX, 1);
				_delay(P2824X3);
				digitalWrite(TX, 0);
				_delay(P2824);
			} else {
				// 0
				digitalWrite(TX, 1);
				_delay(P2824);
				digitalWrite(TX, 0);
				_delay(P2824X3);
			}

			mask = mask >> 1;
		}
	}
}

static void send32_short(char remote, char channel, char command) {
}

static void send32_long(char remote, char channel, char command) {
}

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
		return -1;
	}

	// initialize wiringPi
	if (wiringPiSetup() == -1) {
		printf("Unable to start wiringPi");
		return -2;
	}

	// Set our thread to MAX priority
	struct sched_param sp;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (sched_setscheduler(0, SCHED_FIFO, &sp)) {
		return -3;
	}

	// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT)) {
		return -4;
	}

	pinMode(TX, OUTPUT);

	// remote 1, 2, 3, ...
	int remote = atoi(argv[1]);
	if (remote < 1 || remote > REMOTES) {
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
	int rolling = 0;
	if (argv[4] != NULL) {
		rolling = atoi(argv[4]);
		if (rolling < 0 || rolling > 3) {
			printf("wrong rolling code index %i\n", rolling);
			usage();
			return EINVAL;
		}
	}

	send28(TRANSMITTER[remote - 1], channel - 'A' + 1, command ? 2 : 0, rolling);
	// send24(remote, channel, command);
}

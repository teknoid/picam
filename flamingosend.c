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

#include "flamingo.h"

static void _delay(unsigned int millis) {
	struct timeval tNow, tLong, tEnd;

	gettimeofday(&tNow, NULL);
	tLong.tv_sec = millis / 1000000;
	tLong.tv_usec = millis % 1000000;
	timeradd(&tNow, &tLong, &tEnd);

	while (timercmp(&tNow, &tEnd, <)) {
		gettimeofday(&tNow, NULL);
	}
}

static void send28(char remote, char channel, char command, char offset) {
	unsigned int x = (remote - 1) * 4 + (channel - 'A');
	unsigned int y = command * 4 + offset;
	unsigned long code = FLAMINGO[x][y];

//	printf("remote %i channel %i command %i offset %i\n", remote, channel, command, offset);
//	printf("sending FLAMINGO[%i][%i] code 0x%08lx %i\n", x, y, code, PULSE);

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

static void send24(char remote, char channel, char command, char offset) {
	unsigned int x = (remote - 1) * 4 + (channel - 'A');
	unsigned int y = command * 4 + offset;
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

static void usage() {
	printf("Usage: flamingosend <remote> <channel> <command> [offset]\n");
	printf("  <remote>  1, 2, 3, ...\n");
	printf("  <channel> A, B, C, D\n");
	printf("  <command> 0 - off, 1 - on\n");
	printf("  [offset]  which one of the 4 codes to be sent\n");
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
	char remote = atoi(argv[1]);
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

	// optional: send one of the 4 control codes
	int offset = 0;
	if (argv[4] != NULL) {
		offset = atoi(argv[4]);
		if (offset < 0 || offset > 3) {
			printf("wrong offset %i\n", offset);
			usage();
			return EINVAL;
		}
	}

	send24(remote, channel, command, offset);
}

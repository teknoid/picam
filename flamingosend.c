#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <time.h>
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

static void _send(uint32_t code) {
	uint32_t msb = 0b1000000000000000000000000000;	// the 28th MSB bit	(Flamingo command is only 28 bits)
	uint32_t codeRepeat = code;
	uint16_t repeat, i;

	for (repeat = 0; repeat < 4; repeat++) {
		// send sync
		digitalWrite(TX, HIGH);
		_delay(PULSE * 1);
		digitalWrite(TX, LOW);
		_delay(PULSE * 15);
		code = codeRepeat;
		for (i = 0; i < 28; i++) {
			if ((code & msb) == 0) {
				// send 0
				digitalWrite(TX, HIGH);
				_delay(PULSE * 1);
				digitalWrite(TX, LOW);
				_delay(PULSE * 3);
			} else {
				// send 1
				digitalWrite(TX, HIGH);
				_delay(PULSE * 3);
				digitalWrite(TX, LOW);
				_delay(PULSE * 1);
			}
			code = code << 1;
		}
	}
}

static void send(char remote, char channel, char command, char offset) {
	uint16_t x = (remote - 1) * 4 + (channel - 'A');
	uint16_t y = command * 4 + offset;
	uint32_t code = FLAMINGO[x][y];

	// printf("%i %i %i %i\n", remote, channel, command, offset);
	// printf("sending FLAMINGO[%i][%i] code 0x%08x\n", x, y, code);
	_send(code);
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

	// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT)) {
		return -3;
	}

	// Set our thread to real time priority
	struct sched_param sp;
	sp.sched_priority = SCHED_PRIO;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
		printf("Unable to set thread priority: %s\n", strerror(errno));
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

	send(remote, channel, command, offset);
}

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

	printf("rc1 sending %s => 0x%08lx => 0x%08lx\n", printbits(message, 0x01000110), message, code);

	for (int repeat = 1; repeat <= 4; repeat++) {
		unsigned long mask = 1 << 27;

		// sync
		digitalWrite(TX, 1);
		_delay(T1);
		digitalWrite(TX, 0);
		_delay(T1X15);

		while (mask) {
			if (code & mask) {
				// 1
				digitalWrite(TX, 1);
				_delay(T1X3);
				digitalWrite(TX, 0);
				_delay(T1);
			} else {
				// 0
				digitalWrite(TX, 1);
				_delay(T1);
				digitalWrite(TX, 0);
				_delay(T1X3);
			}

			mask = mask >> 1;
		}
	}
	usleep(REPEAT_PAUSE1);
}

static void send24(unsigned int txid, unsigned char channel, unsigned char command) {
	unsigned long message = 0x00144114;

	printf("rc4 sending %s => 0x%08lx\n", printbits(message, 0x01000110), message);

	for (int repeat = 1; repeat <= 5; repeat++) {
		unsigned long mask = 1 << 23;

		// sync
		digitalWrite(TX, 1);
		_delay(T1);
		digitalWrite(TX, 0);
		_delay(T1X31);

		while (mask) {
			if (message & mask) {
				// 1
				digitalWrite(TX, 1);
				_delay(T1X3);
				digitalWrite(TX, 0);
				_delay(T1);
			} else {
				// 0
				digitalWrite(TX, 1);
				_delay(T1);
				digitalWrite(TX, 0);
				_delay(T1X3);
			}

			mask = mask >> 1;
		}
	}
	usleep(REPEAT_PAUSE2);
}

static void send32(unsigned int txid, unsigned char channel, unsigned char command) {
	unsigned long message = encode(txid, channel, command, 0);

	printf("rc2 sending %s => 0x%08lx\n", printbits(message, 0x01000110), message);

	for (int repeat = 1; repeat <= 3; repeat++) {
		unsigned long mask = 1 << 31;

		// sync
		digitalWrite(TX, 1);
		_delay(T2H);
		digitalWrite(TX, 0);
		_delay(T2S);

		while (mask) {
			if (message & mask) {
				// 1
				digitalWrite(TX, 1);
				_delay(T2H);
				digitalWrite(TX, 0);
				_delay(T2X);
				digitalWrite(TX, 1);
				_delay(T2H);
				digitalWrite(TX, 0);
				_delay(T2L);
			} else {
				// 0
				digitalWrite(TX, 1);
				_delay(T2H);
				digitalWrite(TX, 0);
				_delay(T2L);
				digitalWrite(TX, 1);
				_delay(T2H);
				digitalWrite(TX, 0);
				_delay(T2X);
			}

			mask = mask >> 1;
		}

		// a clock or parity (?) bit terminates the message
		digitalWrite(TX, 1);
		_delay(T2H);
		digitalWrite(TX, 0);
		_delay(T2L);

		// wait before sending next sync
		_delay(4 * T2S);
	}
	usleep(REPEAT_PAUSE2);
}

static void send32_multibit(unsigned int txid, unsigned char channel, unsigned char command) {
	unsigned char bits[32];

	char m1[] = "00300030001011200111104002100210"; // on
	char m2[] = "00300030001011200111104003000210"; // off
	char *c = command ? m1 : m2;

	// create multibit array from string
	for (int i = 0; i < 32; i++) {
		bits[i] = *c++ - '0';
	}

	printf("rc3 sending ");
	for (int i = 0; i < 32; i++) {
		printf("%d", bits[i]);
	}
	printf("\n");

	for (int repeat = 1; repeat <= 3; repeat++) {

		// sync
		digitalWrite(TX, 1);
		_delay(T3H);
		digitalWrite(TX, 0);
		_delay(T3S);

		for (int i = 0; i < 32; i++) {

			// clock bit
			digitalWrite(TX, 1);
			_delay(T2H);
			digitalWrite(TX, 0);
			_delay(T2L);

			// data bits
			for (int j = 0; j < bits[i]; j++) {
				digitalWrite(TX, 1);
				_delay(T2H);
				digitalWrite(TX, 0);
				_delay(T2L);
			}

			// wait to next clock bit
			_delay(T2X);
		}
	}
	usleep(REPEAT_PAUSE2);
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
	send32(TRANSMITTER[remote - 1], channel - 'A' + 1, command);
	send32_multibit(0, 0, 0);
	send24(0, 0, 0);
}

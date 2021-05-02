/***
 *
 * Library for sending and receiving 433MHz ELRO Flamingo home device messages
 *
 * Copyright (C) 01/2021 by teknoid
 *
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
#include <ctype.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <wiringPi.h>

#include "flamingo.h"
#include "utils.h"

#define RX					2
#define TX					0

// #define FLAMINGO_MAIN

// global variables used in GPIO interrupt handler
static unsigned long code, pulse, tnow, tlast;
static unsigned char bits, state, clockbit, databits, decryption;

static pthread_t thread_flamingo;
static void* flamingo(void *arg);

static flamingo_config_t *cfg;

#ifdef FLAMINGO_MAIN
static int usage() {
	printf("Usage: flamingo -t | -12345 | <remote> <channel> <command> [rolling]\n");
	printf("  RECEIVE and TEST mode\n");
	printf("    -t ... execute unit test functions\n");
	printf("    -v ... verbose output\n");
	printf("    -1 ... detect 28bit rolling codes, rc1 pattern\n");
	printf("    -2 ... detect 32bit messages, rc2 pattern\n");
	printf("    -3 ... detect 32bit multibit messages, rc3 pattern (encoding still unknown)\n");
	printf("    -4 ... detect 24bit messages, rc4 pattern\n");
	printf("    -5 ... detect 32bit messages, rc2 pattern SF-500R message coding\n");
	printf("  SEND mode\n");
	printf("    <remote>  1, 2, 3, ...\n");
	printf("    <channel> A, B, C, D\n");
	printf("    <command> 0 - off, 1 - on\n");
	printf("    [rolling]  rolling code index, 0...3\n");
	return EXIT_FAILURE;
}

static void flamingo_test_general(unsigned int xmitter) {
	unsigned long code, message, msg1, msg2;
	unsigned char channel, command, payload, rolling;

	printf("\n*** test message encoding & decoding ***\n");
	message = encode_FA500(xmitter, 2, 1, 0x05, 0);
	code = encrypt(message);
	printf("encrypt %s => 0x%08lx => 0x%08lx\n", printbits(message, SPACEMASK_FA500), message, code);
	message = decrypt(code);
	decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);
	printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(message, SPACEMASK_FA500), message, code);
	printf("  xmitter = 0x%x\n  channel = %d\n  command = %d\n  payload = 0x%02x\n", xmitter, channel, command, payload);

	printf("\n*** test rolling code encryption & decryption ***\n");
	for (int r = 0; r < 4; r++) {
		msg1 = encode_FA500(xmitter, 2, 0, 0, r);
		code = encrypt(msg1);
		printf("encrypt %s => 0x%08lx => 0x%08lx\n", printbits(msg1, SPACEMASK_FA500), msg1, code);
		msg2 = decrypt(code);
		printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(msg2, SPACEMASK_FA500), msg2, code);
	}
}

static void flamingo_test_decrypt(unsigned long code) {
	unsigned long message;
	unsigned char channel, command, payload, rolling;
	unsigned int xmitter;

	printf("\n*** test decryption & decoding ***\n");
	message = decrypt(code);
	decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);
	printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(message, SPACEMASK_FA500), message, code);
	printf("  xmitter = 0x%x\n  channel = %d\n  command = %d\n  payload = 0x%02x\n", xmitter, channel, command, payload);

	// validate against defined transmitter id's
	for (int i = 0; i < ARRAY_SIZE(REMOTES); i++) {
		if (xmitter == REMOTES[i]) {
			printf("  Transmitter valid, index=%d\n", i);
		}
	}
}

static void flamingo_test_brute_force(unsigned int xmitter, unsigned long c1, unsigned long c2, unsigned long c3, unsigned long c4) {
	unsigned long code, message;

	printf("\n*** test brute-force encryption ***\n");
	for (int y = 0; y < 0x0F; y++) {
		for (int x = 0; x < 0xFF; x++) {
			message = y << 24 | xmitter << 8 | x;
			code = encrypt(message);
			if (code == c1 || code == c2 || code == c3 || code == c4) {
				printf("%s => 0x%08lx => 0x%08lx\n", printbits(message, SPACEMASK_FA500), message, code);
			}
		}
	}
}

static void flamingo_test_deadbeef() {
	unsigned int transmitter;
	unsigned long code, message;
	unsigned char channel, command, payload, rolling;
	unsigned long testcodes[3] = { 0x0000dead, 0x000beef0, 0x0affe000 };

	printf("\n*** test dead+beef+affe ***\n");
	for (int i = 0; i < ARRAY_SIZE(testcodes); i++) {
		code = testcodes[i];
		message = decrypt(code);
		decode_FA500(message, &transmitter, &channel, &command, &payload, &rolling);
		printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(message, SPACEMASK_FA500), message, code);
		message = encode_FA500(transmitter, channel, command, payload, rolling);
		code = encrypt(message);
		printf("encrypt %s => 0x%08lx => 0x%08lx\n", printbits(message, SPACEMASK_FA500), message, code);
	}
}

static int flamingo_test(int argc, char *argv[]) {

	// transmitter
	flamingo_test_general(REMOTES[0]);

	// code
	flamingo_test_decrypt(0x0e5afff5);

	// transmitter + 4x rolling codes (to see corresponding source messages)
	flamingo_test_brute_force(REMOTES[0], 0x0e6bd68d, 0x0e7be29d, 0x0e70a7f5, 0x0e763e15);

	// test deadbeef
	flamingo_test_deadbeef();

	// decrypt all codes from command line
	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
			unsigned long code = strtoul(argv[i], NULL, 0);
			flamingo_test_decrypt(code);
		}
	}
	return EXIT_SUCCESS;
}

static int flamingo_main(int argc, char **argv) {
	if (argc < 1)
		return usage();

	// initialize a default configuration
	cfg = flamingo_default_config();

	if (argc >= 4) {

		// SEND mode

		// elevate realtime priority
		if (elevate_realtime(3) < 0)
			return -1;

		if (init_micros())
			return -2;

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

		// initialize without receive support
		flamingo_init(0, 0);
		flamingo_send_FA500(remote, channel, command, rolling);
		return EXIT_SUCCESS;

	} else {

		// RECEIVE or TEST mode

		int c = getopt(argc, argv, "t12345");
		switch (c) {
		case 't':
			return flamingo_test(argc, argv);
		case '1':
			cfg->pattern = 1;
			break;
		case '2':
			cfg->pattern = 2;
			break;
		case '3':
			cfg->pattern = 3;
			break;
		case '4':
			cfg->pattern = 4;
			break;
		case '5':
			cfg->pattern = 5;
			break;
		default:
			return usage();
		}

		// initialize with receive support: pattern to listen on and a handler routine
		flamingo_init();
		pause();
		flamingo_close();
		return EXIT_SUCCESS;
	}
}
#endif

//
// rc1 pattern
//
// short high pulse followed by long low pulse or long high + short low pulse, no clock
//       _              ___
// 0 = _| |___    1 = _|   |_
//
// https://forum.arduino.cc/index.php?topic=201771.0
//
static void isr28() {
	// calculate pulse length; store timer value for next calculation; get pin state
	tnow = _micros();
	state = digitalRead(cfg->rx);
	pulse = tnow - tlast;
	tlast = tnow;

	// measure HIGH pulse length: short=0 long=1
	if (bits && (state == 0)) {
		code = code << 1;
		code += pulse > T1X2 ? 1 : 0;
		if (--bits == 0) {
			if (!cfg->quiet)
				printf("0x%08lx\n", code);
		}
		return;
	}

	// ignore noise
	if (pulse < 100) {
		return;
	}

	// check for sync LOW pulses
	if (state == 1) {
		// a rising edge
		if (T1SMIN < pulse && pulse < T1SMAX) {
			code = 0;
			bits = 28;
			if (!cfg->quiet)
				printf("28bit LOW sync %lu :: ", pulse);
		}
	}
}

//
// rc4 pattern
//
// similar to isr28 but only 24bit code without encryption
//
static void isr24() {
	// calculate pulse length; store timer value for next calculation; get pin state
	tnow = _micros();
	state = digitalRead(cfg->rx);
	pulse = tnow - tlast;
	tlast = tnow;

	// measure HIGH pulse length: short=0 long=1
	if (bits && (state == 0)) {
		code = code << 1;
		code += pulse > T1X2 ? 1 : 0;
		if (--bits == 0) {
			if (!cfg->quiet)
				printf("0x%08lx\n", code);
		}
		return;
	}

	// ignore noise
	if (pulse < 100) {
		return;
	}

	// check for sync LOW pulses
	if (state == 1) {
		// a rising edge
		if (T4SMIN < pulse && pulse < T4SMAX) {
			code = 0;
			bits = 24;
			if (!cfg->quiet)
				printf("24bit LOW sync %lu :: ", pulse);
		}
	}
}

//
// rc2 pattern
//
// clock pulse + data pulse, either short or long distance from clock to data pulse
//       _   _               _      _
// 0 = _| |_| |____    1 = _| |____| |_
//
// 32 bit pattern: 00000000 1000001101011010 0001 0001
//                          1                2    3
// 1=Transmitter ID, 16 bit
// 2=Command, 4 bit, 0=OFF, 1=ON
// 3=Channel, 4 bit
//
// https://forum.pilight.org/showthread.php?tid=1110&page=12
static void isr32() {
	// calculate pulse length; store timer value for next calculation; get pin state
	tnow = _micros();
	state = digitalRead(cfg->rx);
	pulse = tnow - tlast;
	tlast = tnow;

	// measure LOW pulses, skip clock and the check space between next pulse
	if (bits && (state == 1)) {
		// clock pulse -> skip
		if (clockbit) {
			clockbit = 0;
			code = code << 1;
			return;
		}

		// data pulse, check space between previous clock pulse: short=0 long=1
		code += pulse < T2Y ? 0 : 1;
		if (--bits == 0)
			if (!cfg->quiet)
				printf("0x%08lx\n", code);

		clockbit = 1; // next is a clock pulse
		return;
	}

	// ignore noise
	if (pulse < 100) {
		return;
	}

	// check for sync LOW pulses
	if (state == 1) {
		// a rising edge
		if (T2S1MIN < pulse && pulse < T2S1MAX) {
			code = 0;
			bits = 32;
			clockbit = 0; // this is the first clock pulse
			if (!cfg->quiet)
				printf("32bit LOW sync %lu :: ", pulse);

		} else if (T2S2MIN < pulse && pulse < T2S2MAX) {
			code = 0;
			bits = 32;
			clockbit = 0; // this is the first clock pulse
			if (!cfg->quiet)
				printf("32bit LOW sync %lu :: ", pulse);

		}
	}
}

//
// rc3 pattern
//
// similar to isr32_short but with longer sync
// looks like a clock pulse with more than one data pulses follow
// encoding not yet known, simply count and print the data bits after clock bit
static void isr32_multibit() {
	// calculate pulse length; store timer value for next calculation; get pin state
	tnow = _micros();
	state = digitalRead(cfg->rx);
	pulse = tnow - tlast;
	tlast = tnow;

	// measure LOW pulses and decide if it was a data bit or a clock bit
	if (bits && (state == 1)) {
		if (pulse < T3Y) {
			// simply count the data bits
			databits++;
		} else {
			// next clock bit, store data bits
			bits--;
			if (!cfg->quiet) {
				printf("%d", databits);
				if (bits == 0) {
					printf("\n");
				}
			}
			databits = 0;
		}
		return;
	}

	// ignore noise
	if (pulse < 100) {
		return;
	}

	// check for sync LOW pulses
	if (state == 1) {
		// a rising edge
		if (T3SMIN < pulse && pulse < T3SMAX) {
			code = 0;
			bits = 32;
			if (!cfg->quiet)
				printf("32bit LOW multibit sync %lu :: ", pulse);

		}
	}
}

// rc1 pattern
static void send28(unsigned long m, int r) {
	while (r--) {
		unsigned long mask = 1 << 27;

		// sync
		digitalWrite(cfg->tx, 1);
		delay_micros(T1);
		digitalWrite(cfg->tx, 0);
		delay_micros(T1X15);

		while (mask) {
			if (m & mask) {
				// 1
				digitalWrite(cfg->tx, 1);
				delay_micros(T1X3);
				digitalWrite(cfg->tx, 0);
				delay_micros(T1);
			} else {
				// 0
				digitalWrite(cfg->tx, 1);
				delay_micros(T1);
				digitalWrite(cfg->tx, 0);
				delay_micros(T1X3);
			}

			mask = mask >> 1;
		}
	}
	usleep(REPEAT_PAUSE1);
}

// rc4 pattern
static void send24(unsigned long m, int r) {
	while (r--) {
		unsigned long mask = 1 << 23;

		// sync
		digitalWrite(cfg->tx, 1);
		delay_micros(T1);
		digitalWrite(cfg->tx, 0);
		delay_micros(T1X31);

		while (mask) {
			if (m & mask) {
				// 1
				digitalWrite(cfg->tx, 1);
				delay_micros(T1X3);
				digitalWrite(cfg->tx, 0);
				delay_micros(T1);
			} else {
				// 0
				digitalWrite(cfg->tx, 1);
				delay_micros(T1);
				digitalWrite(cfg->tx, 0);
				delay_micros(T1X3);
			}

			mask = mask >> 1;
		}
	}
	usleep(REPEAT_PAUSE2);
}

// rc2
static void send32(unsigned long m, int r) {
	while (r--) {
		unsigned long mask = 1 << 31;

		// sync
		digitalWrite(cfg->tx, 1);
		delay_micros(T2H);
		digitalWrite(cfg->tx, 0);
		delay_micros(T2S1);

		while (mask) {
			if (m & mask) {
				// 1
				digitalWrite(cfg->tx, 1);
				delay_micros(T2H);
				digitalWrite(cfg->tx, 0);
				delay_micros(T2X);
				digitalWrite(cfg->tx, 1);
				delay_micros(T2H);
				digitalWrite(cfg->tx, 0);
				delay_micros(T2L);
			} else {
				// 0
				digitalWrite(cfg->tx, 1);
				delay_micros(T2H);
				digitalWrite(cfg->tx, 0);
				delay_micros(T2L);
				digitalWrite(cfg->tx, 1);
				delay_micros(T2H);
				digitalWrite(cfg->tx, 0);
				delay_micros(T2X);
			}

			mask = mask >> 1;
		}

		// a clock or parity (?) bit terminates the message
		digitalWrite(cfg->tx, 1);
		delay_micros(T2H);
		digitalWrite(cfg->tx, 0);
		delay_micros(T2L);

		// wait before sending next sync
		delay_micros(4 * T2S1);
	}
	usleep(REPEAT_PAUSE2);
}

// rc3
static void send32_multibit(char *m, int r) {
	unsigned char bits[32];

	char m1[] = "00300030001011200111104002100210"; // on
	//char m2[] = "00300030001011200111104003000210"; // off
	m = m1;

	// create multibit array from string
	char *c = m;
	for (int i = 0; i < 32; i++) {
		bits[i] = *c++ - '0';
	}

//#ifdef FLAMINGO_MAIN
//	printf("rc3 sending ");
//	for (int i = 0; i < 32; i++) {
//		printf("%d", bits[i]);
//	}
//	printf("\n");
//#endif

	while (r--) {

		// sync
		digitalWrite(cfg->tx, 1);
		delay_micros(T3H);
		digitalWrite(cfg->tx, 0);
		delay_micros(T3S);

		for (int i = 0; i < 32; i++) {

			// clock bit
			digitalWrite(cfg->tx, 1);
			delay_micros(T3H);
			digitalWrite(cfg->tx, 0);
			delay_micros(T3L);

			// data bits
			for (int j = 0; j < bits[i]; j++) {
				digitalWrite(cfg->tx, 1);
				delay_micros(T3H);
				digitalWrite(cfg->tx, 0);
				delay_micros(T3L);
			}

			// wait to next clock bit
			delay_micros(T3X);
		}

		// wait before sending next sync
		delay_micros(4 * T3S);
	}
	usleep(REPEAT_PAUSE2);
}

unsigned long encrypt(unsigned long message) {
	unsigned long code = 0;
	unsigned char n[7];
	int i, r, idx;

	// split into nibbles
	for (i = 0; i < sizeof(n); i++) {
		n[i] = message >> (4 * i) & 0x0F;
	}

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

unsigned long decrypt(unsigned long code) {
	unsigned long message = 0;
	unsigned char n[7];
	int i, r;

	//shift 2 bits left & copy bit 27/28 to bit 1/2
	code = ((code << 2) & 0x0FFFFFFF) | ((code & 0xC000000) >> 0x1A);

	// split into nibbles
	for (i = 0; i < sizeof(n); i++) {
		n[i] = code >> (4 * i) & 0x0F;
	}

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
		if (i) {
			message <<= 4;
		}
	}

	return message;
}

unsigned long encode_FA500(unsigned int xmitter, unsigned char channel, unsigned char command, unsigned char payload, unsigned char rolling) {
	unsigned long message = (payload & 0x0F) << 24 | xmitter << 8 | (rolling << 6 & 0xC0) | (command & 0x03) << 4 | (channel & 0x0F);
	return message;
}

unsigned long encode_SF500(unsigned int xmitter, unsigned char channel, unsigned char command, unsigned char payload) {
	// TODO
	return 0;
}

void decode_FA500(unsigned long message, unsigned int *xmitter, unsigned char *channel, unsigned char *command, unsigned char *payload, unsigned char *rolling) {
	*payload = message >> 24 & 0x0F;
	*xmitter = message >> 8 & 0xFFFF;
	*rolling = message >> 6 & 0x03;
	*command = message >> 4 & 0x03;
	*channel = message & 0x0F;
}

void decode_SF500(unsigned long message, unsigned int *xmitter, unsigned char *channel, unsigned char *command, unsigned char *payload) {
	*xmitter = message >> 16 & 0xFFFF;
	*payload = message >> 8 & 0x0F;
	*command = message >> 4 & 0x03;
	*channel = message & 0x0F;
}

void flamingo_send_FA500(int remote, char channel, int command, int rolling) {
	if (remote < 1 || remote > ARRAY_SIZE(REMOTES)) {
		return;
	}
	if (channel < 'A' || channel > 'P') {
		return;
	}

	unsigned int transmitter = REMOTES[remote - 1];

	if (0 <= rolling && rolling <= 4) {

		// send specified rolling code
		unsigned long m28 = encode_FA500(transmitter, channel - 'A' + 1, command ? 2 : 0, 0, rolling);
		unsigned long m32 = encode_FA500(transmitter, channel - 'A' + 1, command, 0, 0);
		unsigned long c28 = encrypt(m28);
		if (!cfg->quiet)
			printf("FA500 %d %c %d %d => 0x%08lx %s => 0x%08lx\n", remote, channel, command, rolling, m28, printbits(m28, SPACEMASK_FA500), c28);
		send28(c28, 4);
		send32(m32, 3);
		send32_multibit(0, 3); // TODO
		send24(0x00144114, 5); // TODO

	} else {

		// send all rolling codes in sequence
		for (int r = 0; r < 4; r++) {

			unsigned long m28 = encode_FA500(transmitter, channel - 'A' + 1, command ? 2 : 0, 0, r);
			unsigned long m32 = encode_FA500(transmitter, channel - 'A' + 1, command, 0, 0);
			unsigned long c28 = encrypt(m28);
			if (!cfg->quiet)
				printf("FA500 %d %c %d %d => 0x%08lx %s => 0x%08lx\n", remote, channel, command, r, m28, printbits(m28, SPACEMASK_FA500), c28);
			send28(c28, 4);
			send32(m32, 3);
			send32_multibit(0, 3); // TODO
			send24(0x00144114, 5); // TODO
			sleep(1);
		}
	}
}

void flamingo_send_SF500(int remote, char channel, int command) {
	if (remote < 1 || remote > ARRAY_SIZE(REMOTES)) {
		return;
	}

	unsigned int transmitter = REMOTES[remote - 1];
	unsigned long message = encode_SF500(transmitter, channel - 'A' + 1, command, 0);
	if (!cfg->quiet)
		printf("SF500 %d %d %d => 0x%08lx %s\n", remote, channel, command, message, printbits(message, SPACEMASK_SF500));
	send32(message, 5);
}

static void* flamingo(void *arg) {
	unsigned long message;
	unsigned int xmitter;
	unsigned char channel, command, payload, rolling;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	// elevate realtime priority
	if (elevate_realtime(3) < 0)
		return (void *) 0;

	if (init_micros())
		return (void *) 0;

	while (1) {
		msleep(500);

		if (bits)
			continue; // receiving in progress

		if (!code)
			continue; // no new code received

		// TODO find a synchronization mechanism, e.g. semaphores
		if (decryption) {
			message = decrypt(code);
		} else {
			message = code;
		}

		if (cfg->pattern == 5) {
			decode_SF500(message, &xmitter, &channel, &command, &payload);
		} else {
			decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);
		}

		if (!cfg->quiet) {
			unsigned long spacemask = cfg->pattern == 5 ? SPACEMASK_FA500 : SPACEMASK_SF500;
			printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(message, spacemask), message, code);
			printf("  xmitter = 0x%x\n  channel = %d\n  command = %d\n  payload = 0x%02x\n", xmitter, channel, command, payload);
		}

		// validate received message against defined transmitter id's
		for (int i = 0; i < ARRAY_SIZE(REMOTES); i++) {
			if (xmitter == REMOTES[i]) {
				// call handler
				(cfg->flamingo_handler)(xmitter, channel, command, payload);
			}
		}

		code = 0;
	}

	return (void *) 0;
}

static void flamingoread_default_handler(unsigned int xmitter, unsigned char channel, unsigned char command, unsigned char payload) {
	printf("received Transmitter-Id = 0x%x, channel = %d, command = %d, payload = 0x%02x\n", xmitter, channel, command, payload);
}

flamingo_config_t *flamingo_default_config() {
	cfg = malloc(sizeof(*cfg));
	memset(cfg, 0, sizeof(*cfg));

	// default config
	cfg->quiet = 0;
	cfg->rx = RX;
	cfg->tx = TX;
	cfg->pattern = 1;
	cfg->flamingo_handler = &flamingoread_default_handler;

	return cfg;
}

int flamingo_init() {
	// initialize wiringPi
	if (wiringPiSetup() < 0) {
		return -1;
	}

	// GPIO pin connected to 433MHz sender module
	pinMode(cfg->tx, OUTPUT);

	// for code receiving start a thread to decrypt and handle codes
	if (cfg->pattern && cfg->flamingo_handler) {
		if (pthread_create(&thread_flamingo, NULL, &flamingo, NULL)) {
			perror("Error creating thread");
			return -1;
		}

		// GPIO pin connected to 433MHz receiver module
		pinMode(cfg->rx, INPUT);
		pullUpDnControl(cfg->rx, PUD_DOWN);

		bits = 0;
		decryption = 0;
		state = digitalRead(cfg->rx);

		if (!cfg->quiet)
			printf("pin state: %i\n", state);

		switch (cfg->pattern) {
		case 1:
			decryption = 1;
			if (wiringPiISR(cfg->rx, INT_EDGE_BOTH, &isr28) < 0) {
				return -1;
			}
			if (!cfg->quiet)
				printf("listen to 28bit rc1 rolling codes...\n");
			break;

		case 2:
			if (wiringPiISR(cfg->rx, INT_EDGE_BOTH, &isr32) < 0) {
				return -1;
			}
			if (!cfg->quiet)
				printf("listen to 32bit rc2 codes...\n");
			break;

		case 3:
			if (wiringPiISR(cfg->rx, INT_EDGE_BOTH, &isr32_multibit) < 0) {
				return -1;
			}
			if (!cfg->quiet)
				printf("listen to 32bit rc3 multibit codes...\n");
			break;

		case 4:
			if (wiringPiISR(cfg->rx, INT_EDGE_BOTH, &isr24) < 0) {
				return -1;
			}
			if (!cfg->quiet)
				printf("listen to 24bit rc4 codes...\n");
			break;
		case 5:
			if (wiringPiISR(cfg->rx, INT_EDGE_BOTH, &isr32) < 0) {
				return -1;
			}
			if (!cfg->quiet)
				printf("listen to 32bit rc2 with SF-500R message coding...\n");
			break;
		}
	}

	return 0;
}

void flamingo_close() {
	if (thread_flamingo) {
		if (pthread_cancel(thread_flamingo)) {
			perror("Error canceling thread");
		}
		if (pthread_join(thread_flamingo, NULL)) {
			perror("Error joining thread");
		}
	}
	free(cfg);
}

#ifdef FLAMINGO_MAIN
int main(int argc, char *argv[]) {
	return flamingo_main(argc, argv);
}
#endif

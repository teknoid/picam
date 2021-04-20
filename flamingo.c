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

#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>

#include <wiringPi.h>

#include "flamingo.h"
#include "utils.h"

#define DEBUG

// global variables used in GPIO interrupt handler
static unsigned long code, pulse;
static unsigned char bits, state, clockbit, databits, decryption;
static struct timeval tNow, tLong, tLast, tEnd;

static pthread_t thread_flamingo;

static void* flamingo(void *arg);

int flamingoread_pattern;
flamingo_handler_t flamingoread_handler;

static void _delay(unsigned int millis) {
	gettimeofday(&tNow, NULL);
	tLong.tv_sec = millis / 1000000;
	tLong.tv_usec = millis % 1000000;
	timeradd(&tNow, &tLong, &tEnd);
	while (timercmp(&tNow, &tEnd, <)) {
		gettimeofday(&tNow, NULL);
	}
}

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
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// measure HIGH pulse length: short=0 long=1
	if (bits && (state == 0)) {
		code = code << 1;
		code += pulse > T1X2 ? 1 : 0;
		if (--bits == 0) {
#ifdef DEBUG
			printf("0x%08lx\n", code);
#endif
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
#ifdef DEBUG
			printf("28bit LOW sync %lu :: ", pulse);
#endif
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
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// measure HIGH pulse length: short=0 long=1
	if (bits && (state == 0)) {
		code = code << 1;
		code += pulse > T1X2 ? 1 : 0;
		if (--bits == 0) {
#ifdef DEBUG
			printf("0x%08lx\n", code);
#endif
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
#ifdef DEBUG
			printf("24bit LOW sync %lu :: ", pulse);
#endif
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
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

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
		if (--bits == 0) {
#ifdef DEBUG
			printf("0x%08lx\n", code);
#endif
		}
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
#ifdef DEBUG
			printf("32bit LOW sync %lu :: ", pulse);
#endif
		} else if (T2S2MIN < pulse && pulse < T2S2MAX) {
			code = 0;
			bits = 32;
			clockbit = 0; // this is the first clock pulse
#ifdef DEBUG
			printf("32bit LOW sync %lu :: ", pulse);
#endif
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
	gettimeofday(&tNow, NULL);
	state = digitalRead(RX);
	pulse = ((tNow.tv_sec * 1000000) + tNow.tv_usec) - ((tLast.tv_sec * 1000000) + tLast.tv_usec);
	tLast.tv_sec = tNow.tv_sec;
	tLast.tv_usec = tNow.tv_usec;

	// measure LOW pulses and decide if it was a data bit or a clock bit
	if (bits && (state == 1)) {
		if (pulse < T3Y) {
			// simply count the data bits
			databits++;
		} else {
			// next clock bit, store data bits
			bits--;
#ifdef DEBUG
			printf("%d", databits);
			if (bits == 0) {
				printf("\n");
			}
#endif
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
#ifdef DEBUG
			printf("32bit LOW multibit sync %lu :: ", pulse);
#endif
		}
	}
}

// rc1 pattern
static void send28(unsigned long m, int r) {
	while (r--) {
		unsigned long mask = 1 << 27;

		// sync
		digitalWrite(TX, 1);
		_delay(T1);
		digitalWrite(TX, 0);
		_delay(T1X15);

		while (mask) {
			if (m & mask) {
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

// rc4 pattern
static void send24(unsigned long m, int r) {
	while (r--) {
		unsigned long mask = 1 << 23;

		// sync
		digitalWrite(TX, 1);
		_delay(T1);
		digitalWrite(TX, 0);
		_delay(T1X31);

		while (mask) {
			if (m & mask) {
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

// rc2
static void send32(unsigned long m, int r) {
	while (r--) {
		unsigned long mask = 1 << 31;

		// sync
		digitalWrite(TX, 1);
		_delay(T2H);
		digitalWrite(TX, 0);
		_delay(T2S1);

		while (mask) {
			if (m & mask) {
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
		_delay(4 * T2S1);
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

//#ifdef DEBUG
//	printf("rc3 sending ");
//	for (int i = 0; i < 32; i++) {
//		printf("%d", bits[i]);
//	}
//	printf("\n");
//#endif

	while (r--) {

		// sync
		digitalWrite(TX, 1);
		_delay(T3H);
		digitalWrite(TX, 0);
		_delay(T3S);

		for (int i = 0; i < 32; i++) {

			// clock bit
			digitalWrite(TX, 1);
			_delay(T3H);
			digitalWrite(TX, 0);
			_delay(T3L);

			// data bits
			for (int j = 0; j < bits[i]; j++) {
				digitalWrite(TX, 1);
				_delay(T3H);
				digitalWrite(TX, 0);
				_delay(T3L);
			}

			// wait to next clock bit
			_delay(T3X);
		}

		// wait before sending next sync
		_delay(4 * T3S);
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
#ifdef DEBUG
		printf("FA500 %d %c %d %d => 0x%08lx %s => 0x%08lx\n", remote, channel, command, rolling, m28, printbits(m28, SPACEMASK_FA500), c28);
#endif
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
#ifdef DEBUG
			printf("FA500 %d %c %d %d => 0x%08lx %s => 0x%08lx\n", remote, channel, command, r, m28, printbits(m28, SPACEMASK_FA500), c28);
#endif
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
#ifdef DEBUG
	printf("SF500 %d %d %d => 0x%08lx %s\n", remote, channel, command, message, printbits(message, SPACEMASK_SF500));
#endif
	send32(message, 5);
}

int flamingo_init(int pattern, flamingo_handler_t handler) {
	// initialize wiringPi
	if (wiringPiSetup() < 0) {
		return -1;
	}

	// GPIO pin connected to 433MHz sender module
	pinMode(TX, OUTPUT);

	// for code receiving start a thread to decrypt and handle codes
	if (pattern && handler) {
		flamingoread_pattern = pattern;
		flamingoread_handler = handler;
		if (pthread_create(&thread_flamingo, NULL, &flamingo, NULL)) {
			perror("Error creating thread");
			return -1;
		}

		// GPIO pin connected to 433MHz receiver module
		pinMode(RX, INPUT);
		pullUpDnControl(RX, PUD_DOWN);

		bits = 0;
		decryption = 0;
		state = digitalRead(RX);

#ifdef DEBUG
		printf("pin state: %i\n", state);
#endif

		switch (pattern) {
		case 1:
			decryption = 1;
			if (wiringPiISR(RX, INT_EDGE_BOTH, &isr28) < 0) {
				return -1;
			}
#ifdef DEBUG
			printf("listen to 28bit rc1 rolling codes...\n");
#endif
			break;

		case 2:
			if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32) < 0) {
				return -1;
			}
#ifdef DEBUG
			printf("listen to 32bit rc2 codes...\n");
#endif
			break;

		case 3:
			if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32_multibit) < 0) {
				return -1;
			}
#ifdef DEBUG
			printf("listen to 32bit rc3 multibit codes...\n");
#endif
			break;

		case 4:
			if (wiringPiISR(RX, INT_EDGE_BOTH, &isr24) < 0) {
				return -1;
			}
#ifdef DEBUG
			printf("listen to 24bit rc4 codes...\n");
#endif
			break;
		case 5:
			if (wiringPiISR(RX, INT_EDGE_BOTH, &isr32) < 0) {
				return -1;
			}
#ifdef DEBUG
			printf("listen to 32bit rc2 with SF-500R message coding...\n");
#endif
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
}

static void* flamingo(void *arg) {
	unsigned long message;
	unsigned int xmitter;
	unsigned char channel, command, payload, rolling;

	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		perror("Error setting pthread_setcancelstate");
		return (void *) 0;
	}

	// Set our thread to MAX priority
	struct sched_param sp;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (sched_setscheduler(0, SCHED_FIFO, &sp)) {
		return (void *) 0;
	}

	// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT)) {
		return (void *) 0;
	}

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

		if (flamingoread_pattern == 5) {
			decode_SF500(message, &xmitter, &channel, &command, &payload);
		} else {
			decode_FA500(message, &xmitter, &channel, &command, &payload, &rolling);
		}

#ifdef DEBUG
		unsigned long spacemask = flamingoread_pattern == 5 ? SPACEMASK_FA500 : SPACEMASK_SF500;
		printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(message, spacemask), message, code);
		printf("  xmitter = 0x%x\n  channel = %d\n  command = %d\n  payload = 0x%02x\n", xmitter, channel, command, payload);
#endif

		// validate received message against defined transmitter id's
		for (int i = 0; i < ARRAY_SIZE(REMOTES); i++) {
			if (xmitter == REMOTES[i]) {
				// call handler
				(flamingoread_handler)(xmitter, channel, command, payload);
			}
		}

		code = 0;
	}

	return (void *) 0;
}

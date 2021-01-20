#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "flamingocrypt.h"

/**
 *
 * based on coding from Fuchks
 * https://forum.fhem.de/index.php/topic,36399.msg503530.html#msg503530
 *
 * encoded message pattern
 *
 * 0000 0000000000000000 0000 XXXX	channel
 * 0000 0000000000000000 00XX 0000	command
 * 0000 0000000000000000 XX00 0000	rolling code id
 * 0000 000000000000XXXX 0000 0000	transmitter id
 * 0000 00000000XXXX0000 0000 0000	transmitter id
 * 0000 0000XXXX00000000 0000 0000	transmitter id
 * 0000 XXXX000000000000 0000 0000	transmitter id
 * XXXX 0000000000000000 0000 0000	payload
 */

unsigned long encode(unsigned int xmitter, unsigned char channel, unsigned char command, unsigned char payload) {
	unsigned long message = 0;
	unsigned char n[7];							// 7 nibbles = 28bit
	int i;

	// encode into nibbles
	n[0] = channel & 0x0F;
	n[1] = command & 0x03;
	n[2] = xmitter & 0x0F;
	n[3] = xmitter >> 4 & 0x0F;
	n[4] = xmitter >> 8 & 0x0F;
	n[5] = xmitter >> 12 & 0x0F;
	n[6] = payload & 0x0F;

	// build message
	for (i = sizeof(n) - 1; i >= 0; i--) {
		message |= n[i];
		if (i) {
			message <<= 4;
		}
	}

	return message;
}

unsigned long encrypt(unsigned long message, unsigned char rolling) {
	unsigned long code = 0;
	unsigned char n[7];
	int i, r, idx;

	// insert rolling code into message
	message = message | (rolling << 6 & 0xC0);

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
	n[0] = code & 0x0000000F;
	n[1] = (code & 0x000000F0) >> 4;
	n[2] = (code & 0x00000F00) >> 8;
	n[3] = (code & 0x0000F000) >> 12;
	n[4] = (code & 0x000F0000) >> 16;
	n[5] = (code & 0x00F00000) >> 20;
	n[6] = (code & 0x0F000000) >> 24;
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

unsigned char decode_channel(unsigned long message) {
	unsigned char channel = message & 0x0F;
	return channel;
}

unsigned char decode_command(unsigned long message) {
	unsigned char command = message >> 4 & 0x03;
	return command;
}

unsigned int decode_xmitter(unsigned long message) {
	unsigned int xmitter = message >> 8 & 0xFFFF;
	return xmitter;
}

unsigned char decode_payload(unsigned long message) {
	unsigned char payload = message >> 24 & 0x0F;
	return payload;
}

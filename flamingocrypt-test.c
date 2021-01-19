#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "utils.h"
#include "flamingo.h"
#include "flamingocrypt.h"

static void brute_force(unsigned int xmitter, unsigned long m1, unsigned long m2, unsigned long m3, unsigned long m4) {
	unsigned char n[7], x, y;
	unsigned long msg, source;

	for (x = 0; x < 0xFF; x++) {
		for (y = 0; y < 0x0F; y++) {

			// encode message
			n[0] = x & 0x0F;
			n[1] = x >> 4 & 0x0F;
			n[2] = xmitter & 0x0F;
			n[3] = xmitter >> 0x04 & 0x0F;
			n[4] = xmitter >> 0x08 & 0x0F;
			n[5] = xmitter >> 0x0C & 0x0F;
			n[6] = y & 0x0F;

			source = (n[6] << 24) | (n[5] << 20) | (n[4] << 16) | (n[3] << 12) | (n[2] << 8) | (n[1] << 4) | n[0];

			// XOR encryption 2 rounds
			for (uint8_t r = 0; r <= 1; r++) {
				n[0] = CKEY[(n[0] - r + 1) & 0x0F];
				for (uint8_t i = 1; i <= 5; i++) {
					n[i] = CKEY[((n[i] ^ n[i - 1]) - r + 1) & 0x0F];
				}
			}
			n[6] = n[6] ^ 9;
			msg = (n[6] << 24) | (n[5] << 20) | (n[4] << 16) | (n[3] << 12) | (n[2] << 8) | (n[1] << 4) | n[0];
			msg = (msg >> 2) | ((msg & 3) << 0x1A);

			if (msg == m1 || msg == m2 || msg == m3 || msg == m4) {
				printf("%s => 0x%08lx => 0x%08lx\n", printbits(source, 0x01000110), source, msg);
			}
		}
	}
}

static void test() {
	printf("\ntest rolling code encryption & decryption\n");
	for (int r = 0; r < 4; r++) {
		unsigned long msg1 = encode(TRANSMITTER[0], 2, 0, 0);
		unsigned long code = encrypt(msg1, r);
		printf("encrypt %s => 0x%08lx => 0x%08lx\n", printbits(msg1, 0x01000110), msg1, code);
		unsigned long msg2 = decrypt(code);
		printf("decrypt %s <= 0x%08lx <= 0x%08lx\n", printbits(msg2, 0x01000110), msg2, code);
	}

	printf("\ntest message decoding\n");
	unsigned long msg = encode(TRANSMITTER[1], 1, 1, 0x05);
	unsigned int xmitter = decode_xmitter(msg);
	unsigned char channel = decode_channel(msg);
	unsigned char command = decode_command(msg);
	unsigned char payload = decode_payload(msg);
	printf("%s\n", printbits(msg, 0x01000110));
	printf("Transmitter-Id = 0x%x    channel = %d    command = %d    payload = 0x%02x\n", xmitter, channel, command, payload);

	// put in here transmitter id and captured codes to see original message to these codes
	printf("\ntest brute-force encryption\n");
	brute_force(TRANSMITTER[0], 0x02796056, 0x025469de, 0x02779c76, 0x0274b462);

}

int main(int argc, char *argv[]) {
	test();
}
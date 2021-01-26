#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "flamingo.h"
#include "utils.h"

static void flamingo_test(unsigned int xmitter) {
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

int main(int argc, char *argv[]) {

	// transmitter
	flamingo_test(REMOTES[0]);

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
}


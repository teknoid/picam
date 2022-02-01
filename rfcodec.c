/***
 *
 * decoder + encoder routines for rfsniffer
 *
 * Copyright (C) 02/2022 by teknoid
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "rfsniffer.h"
#include "rfcodec.h"
#include "utils.h"
#include "frozen.h"

static rfsniffer_config_t *cfg;

static void anaylzer_decode(uint8_t protocol, uint64_t raw, uint8_t r) {
	if (!cfg->quiet)
		xlog("ANALYZER 0x%08llx %s", raw, printbits64(raw, 0x0101010101010101));
}

uint32_t rfcodec_decode_0110(uint64_t in) {
	uint64_t mask;
	uint32_t out = 0;
	int count = 62;

	mask = 0b11;
	mask <<= count; 					// 11000000...00000000

	while (count > 0) {
		int x = (in & mask) >> count;	// 00000000...000000XX
		switch (x) {
		case 0b01:
			out += 0;
			break;
		case 0b10:
			out += 1;
			break;
		default:
			if (cfg->verbose)
				xlog("01/10 decoder error %s", printbits64(in, 0x0001000100010001));
			return 0;
		}

		mask >>= 2;
		count -= 2;
		out <<= 1;
	}
	return out;
}

void rfcodec_decode(uint8_t protocol, uint64_t code, uint8_t repeat) {
	if (!code)
		if (cfg->verbose)
			xlog("DECODE received empty message");

	if (!code)
		return;

	// dispatch to corresponding protocol decoder
	switch (protocol) {
	case P_NEXUS:
		return nexus_decode(protocol, code, repeat);
	case P_FLAMINGO28:
		return flamingo28_decode(protocol, code, repeat);
	case P_FLAMINGO24:
		return flamingo24_decode(protocol, code, repeat);
	case P_FLAMINGO32:
		code = rfcodec_decode_0110(code);
		if (code == 0)
			return;
		return flamingo32_decode(protocol, code, repeat);
	case P_ANALYZE:
		return anaylzer_decode(protocol, code, repeat);
	default:
		if (!cfg->quiet)
			xlog("DECODE no decoder configured for protocol %d 0x%0llx", protocol, code);
	}
}

void rfcodec_test(int argc, char **argv) {
	printf("\n*** test 2x 0xdeadbeef ***\n");
	uint64_t deadbeef = 0xdeadbeefdeadbeef;
	printf("%s\n", printbits64(deadbeef, 0x0101010101010101));

	printf("\n*** test 01/10 decoding ***\n");
	uint64_t c1 = 0x5555955a66995556;
	printf("%s\n", printbits64(c1, 0x0101010101010101));
	uint64_t c2 = rfcodec_decode_0110(c1);
	printf("%s\n", printbits(c2, 0x01010101));

	// test sub modules
	nexus_test(argc, argv);
	flamingo_test(argc, argv);
}

void rfcodec_cfg(rfsniffer_config_t *master) {
	cfg = master;
}

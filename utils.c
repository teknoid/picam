#include <stdlib.h>

#include "utils.h"

char *printbits64(unsigned long long code, unsigned long long spacemask) {
	unsigned int bits = 64;
	char *out = malloc(bits * 2 + 1);
	char *p = out;
	unsigned long long mask = 0x8000000000000000;
	while (mask) {
		if (code & mask) {
			*p++ = '1';
		} else {
			*p++ = '0';
		}
		if (mask & spacemask) {
			*p++ = ' ';
		}
		mask >>= 1;
	}
	*p++ = '\0';
	return out;
}

char *printbits(unsigned long code, unsigned long spacemask) {
	unsigned int bits = 32;
	char *out = malloc(bits * 2 + 1);
	char *p = out;
	unsigned long mask = 0x80000000;
	while (mask) {
		if (code & mask) {
			*p++ = '1';
		} else {
			*p++ = '0';
		}
		if (mask & spacemask) {
			*p++ = ' ';
		}
		mask >>= 1;
	}
	*p++ = '\0';
	return out;
}

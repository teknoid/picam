#include <stdlib.h>

#include "utils.h"

char *printbits(unsigned long code, unsigned long spacemask) {
	char *out = malloc(sizeof(long) * 8 + sizeof(long) + 1);
	char *p = out;

	unsigned long mask = 1 << (sizeof(long) * 8 - 1);
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

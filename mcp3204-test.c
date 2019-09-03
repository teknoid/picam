#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "mcp3204.h"

int main(int argc, char **argv) {

	mcp3204_init();

	int i = 100;
	while (i-- > 0) {
		uint16_t adc12bit = mcp3204_read();
		uint16_t mVolt = mcp3204_millivolt(adc12bit);
		printf("MCP3204: %d --> %dmV \n", adc12bit, mVolt);
		sleep(1);
	}
	return 0;
}

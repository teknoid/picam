#include <stdint.h>

#define VCC_REF			5120	// reference voltage used for A/D conversion
#define SAMPLES			100		// take SAMPLES from MCP3204 and calculate average

#define SPI_DEVICE		"/dev/spidev0.1"
#define SPI_SPEED		10000000
#define SPI_BITS		8
#define SPI_DELAY		0

int mcp3204_init();
uint16_t mcp3204_read();
uint16_t mcp3204_millivolt(uint16_t adc12bit);

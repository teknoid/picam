#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "mcp3204.h"

static pthread_mutex_t mcp3204_lock;
static int spi_fd;

static int mcp3204_main(int argc, char **argv) {
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

static int spi_init() {
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = SPI_BITS;
	uint32_t speed = SPI_SPEED;

	spi_fd = open(SPI_DEVICE, O_RDWR);
	if (spi_fd < 0) {
		printf("error open: %s\n", strerror(errno));
		return -1;
	}

	if (ioctl(spi_fd, SPI_IOC_RD_MODE, &mode) < 0) {
		printf("SPI_IOC_RD_MODE Change failure: %s\n", strerror(errno));
		return -1;
	}
	if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
		printf("SPI_IOC_WR_MODE Change failure: %s\n", strerror(errno));
		return -1;
	}

	if (ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
		printf("SPI_IOC_RD_BITS_PER_WORD Change failure: %s\n", strerror(errno));
		return -1;
	}
	if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		printf("SPI_IOC_WR_BITS_PER_WORD Change failure: %s\n", strerror(errno));
		return -1;
	}

	if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		printf("SPI_IOC_RD_MAX_SPEED_HZ Change failure: %s\n", strerror(errno));
		return -1;
	}
	if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		printf("SPI_IOC_WR_MAX_SPEED_HZ Change failure: %s\n", strerror(errno));
		return -1;
	}

	return 0;
}

static int spi(uint8_t *data, int len) {
	struct spi_ioc_transfer spi;
	memset(&spi, 0, sizeof(spi));
	spi.tx_buf = (unsigned long) data;
	spi.rx_buf = (unsigned long) data;
	spi.len = len;
	spi.speed_hz = SPI_SPEED;
	spi.bits_per_word = SPI_BITS;
	spi.delay_usecs = SPI_DELAY;
	int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
	if (ret < 0) {
		printf("SPI_IOC_MESSAGE ioctl error: %s\n", strerror(errno));
	}
	return ret;
}

uint16_t mcp3204_read() {
	uint32_t sum = 0;
	uint8_t data[3];
	int i, ret;

	pthread_mutex_lock(&mcp3204_lock);
	for (i = 0; i < SAMPLES; i++) {
		data[0] = 0x06;
		data[1] = 0x00;
		data[2] = 0x00;

		ret = spi(data, 3);
		if (ret < 0) {
			return -1;
		}

		uint16_t adc12bit = ((data[1] << 8) & 0x0f00) | (data[2] & 0x00ff);
		sum += adc12bit;
		usleep(100);
	}
	pthread_mutex_unlock(&mcp3204_lock);

	uint16_t avg = sum / SAMPLES;
	return avg;
}

uint16_t mcp3204_millivolt(uint16_t adc12bit) {
	float scale = VCC_REF / (float) 4096;
	uint16_t mVolt = (uint16_t) adc12bit * scale;
	return mVolt;
}

int mcp3204_init() {
	if (spi_init() < 0) {
		return -1;
	}
	if (mcp3204_read() < 0) {
		return -1;
	}
	return 0;
}

#ifdef MCP3204_MAIN
int main(int argc, char **argv) {
	return mcp3204_main(argc, argv);
}
#endif

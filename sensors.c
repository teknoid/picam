#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <mqtt.h>
#include <posix_sockets.h>

#include "sensors.h"
#include "smbus.h"
#include "utils.h"

#define SWAP(X) ((X<<8) & 0xFF00) | ((X>>8) & 0xFF)

// TODO config
static const char *clientid = "picam-mcp";
static const char *addr = "tron";
static const char *port = "1883";
static const char *topic = "sensor";

static pthread_t thread_sensors;
static int i2cfd;
static int mqttfd;
static struct mqtt_client client;
static uint8_t sendbuf[2048];
static uint8_t recvbuf[1024];

static void* sensors_loop(void *arg);

sensors_t *sensors;

// bisher gefühlt bei 100 Lux (19:55)
// Straßenlampe Eisenstraße 0x40 = XX Lux
// Straßenlampe Wiesenstraße bei 0x08 = 6 Lux (20:15)
// Kamera aus bei 0x02 = 1 Lux (20:22)
// 0x00 (20:25) aber noch deutlich hell am Westhorizont

// https://forums.raspberrypi.com/viewtopic.php?t=38023
static void read_bh1750() {
	__u8 buf[2];

	if (ioctl(i2cfd, I2C_SLAVE, BH1750_ADDR) < 0)
		return;

	// powerup
	i2c_smbus_write_byte(i2cfd, BH1750_POWERON);

	// continuous high mode (resolution 1lx)
	i2c_smbus_write_byte(i2cfd, BH1750_CHM);
	msleep(180);
	if (read(i2cfd, buf, 2) != 2)
		return;
	sensors->bh1750_raw = buf[0] << 8 | buf[1];

	// continuous high mode 2 (resolution 0.5lx)
	i2c_smbus_write_byte(i2cfd, BH1750_CHM2);
	msleep(180);
	if (read(i2cfd, buf, 2) != 2)
		return;
	sensors->bh1750_raw2 = buf[0] << 8 | buf[1];

	// sleep
	i2c_smbus_write_byte(i2cfd, BH1750_POWERDOWN);

	if (sensors->bh1750_raw2 == UINT16_MAX)
		sensors->bh1750_lux = sensors->bh1750_raw / 1.2;
	else
		sensors->bh1750_lux = sensors->bh1750_raw2 / 2.4;

	sensors->bh1750_prc = (sqrt(sensors->bh1750_raw) * 100) / UINT8_MAX;
}

// https://forums.raspberrypi.com/viewtopic.php?t=16968
static void read_bmp085() {
	__u8 buf[3];
	int x1, x2, x3, b3, b6, p;
	unsigned int b4, b7;

	if (ioctl(i2cfd, I2C_SLAVE, BMP085_ADDR) < 0)
		return;

	short int ac1 = sensors->bmp085_ac1;
	short int ac2 = sensors->bmp085_ac2;
	short int ac3 = sensors->bmp085_ac3;
	unsigned short int ac4 = sensors->bmp085_ac4;
	unsigned short int ac5 = sensors->bmp085_ac5;
	unsigned short int ac6 = sensors->bmp085_ac6;
	short int b1 = sensors->bmp085_b1;
	short int b2 = sensors->bmp085_b2;
	// short int mb = sensors->bmp085_mb;
	short int mc = sensors->bmp085_mc;
	short int md = sensors->bmp085_md;

	// temperature
	i2c_smbus_write_byte_data(i2cfd, 0xF4, 0x2E);
	msleep(5);
	sensors->bmp085_utemp = SWAP(i2c_smbus_read_word_data(i2cfd, 0xF6));
	x1 = (((int) sensors->bmp085_utemp - (int) ac6) * (int) ac5) >> 15;
	x2 = ((int) mc << 11) / (x1 + md);
	int b5 = x1 + x2;
	sensors->bmp085_temp = ((b5 + 8) >> 4) / 10.0;

	// pressure
	i2c_smbus_write_byte_data(i2cfd, 0xF4, 0x34 + (BMP085_OVERSAMPLE << 6));
	msleep(2 + (3 << BMP085_OVERSAMPLE));
	i2c_smbus_read_i2c_block_data(i2cfd, 0xF6, 3, buf);
	sensors->bmp085_ubaro = (((unsigned int) buf[0] << 16) | ((unsigned int) buf[1] << 8) | (unsigned int) buf[2]) >> (8 - BMP085_OVERSAMPLE);
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6) >> 12) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int) ac1) * 4 + x3) << BMP085_OVERSAMPLE) + 2) >> 2;
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned int) (x3 + 32768)) >> 15;
	b7 = ((unsigned int) (sensors->bmp085_ubaro - b3) * (50000 >> BMP085_OVERSAMPLE));
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;
	sensors->bmp085_baro = p / 100.0;
}

// read BMP085 calibration data
static void init_bmp085() {
	if (ioctl(i2cfd, I2C_SLAVE, BMP085_ADDR) < 0)
		return;

	sensors->bmp085_ac1 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xAA));
	sensors->bmp085_ac2 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xAC));
	sensors->bmp085_ac3 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xAE));
	sensors->bmp085_ac4 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xB0));
	sensors->bmp085_ac5 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xB2));
	sensors->bmp085_ac6 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xB4));
	sensors->bmp085_b1 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xB6));
	sensors->bmp085_b2 = SWAP(i2c_smbus_read_word_data(i2cfd, 0xB8));
	sensors->bmp085_mb = SWAP(i2c_smbus_read_word_data(i2cfd, 0xBA));
	sensors->bmp085_mc = SWAP(i2c_smbus_read_word_data(i2cfd, 0xBC));
	sensors->bmp085_md = SWAP(i2c_smbus_read_word_data(i2cfd, 0xBE));
	xlog("read BMP085 calibration data");
}

static void init_mqtt() {
	if (mqttfd > 0)
		close(mqttfd);

	mqttfd = open_nb_socket(addr, port);
	if (mqttfd < 0) {
		xlog("Failed to open MQTT socket");
		return;
	}

	uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;
	mqtt_init(&client, mqttfd, sendbuf, sizeof(sendbuf), recvbuf, sizeof(recvbuf), NULL);
	mqtt_connect(&client, clientid, NULL, NULL, 0, NULL, NULL, connect_flags, 400);

	if (client.error != MQTT_OK)
		xlog("MQTT Error: %s\n", mqtt_error_str(client.error));
	else
		xlog("connected to MQTT Broker on %s:%s", addr, port);
}

static void publish_mqtt_sensor(const char *sensor, const char *name, const char *value) {
	char subtopic[64];
	snprintf(subtopic, sizeof(subtopic), "%s/%s/%s", topic, sensor, name);
	mqtt_publish(&client, subtopic, value, strlen(value), MQTT_PUBLISH_QOS_0);
}

static void publish_mqtt() {
	char cvalue[8];

	if (mqttfd < 0)
		init_mqtt();
	if (mqttfd < 0)
		return;

	mqtt_sync(&client);
	if (client.error != MQTT_OK) {
		xlog("MQTT pre sync error: %s, trying reconnect on next publishing\n", mqtt_error_str(client.error));
		close(mqttfd);
		mqttfd = -1;
		return;
	}

	snprintf(cvalue, 6, "%u", sensors->bh1750_raw);
	publish_mqtt_sensor(BH1750, "lum_raw", cvalue);

	snprintf(cvalue, 6, "%u", sensors->bh1750_raw2);
	publish_mqtt_sensor(BH1750, "lum_raw2", cvalue);

	snprintf(cvalue, 6, "%u", sensors->bh1750_lux);
	publish_mqtt_sensor(BH1750, "lum_lux", cvalue);

	snprintf(cvalue, 4, "%u", sensors->bh1750_prc);
	publish_mqtt_sensor(BH1750, "lum_percent", cvalue);

	snprintf(cvalue, 5, "%2.1f", sensors->bmp085_temp);
	publish_mqtt_sensor(BMP085, "temp", cvalue);

	snprintf(cvalue, 8, "%4.1f", sensors->bmp085_baro);
	publish_mqtt_sensor(BMP085, "baro", cvalue);

	mqtt_sync(&client);
	if (client.error != MQTT_OK)
		xlog("MQTT post sync error: %s\n", mqtt_error_str(client.error));
}

static void write_sysfslike() {
	char cvalue[8];

	snprintf(cvalue, 6, "%u", sensors->bh1750_raw);
	create_sysfslike(DIRECTORY, "lum_raw", cvalue, "%s", BH1750);

	snprintf(cvalue, 6, "%u", sensors->bh1750_raw2);
	create_sysfslike(DIRECTORY, "lum_raw2", cvalue, "%s", BH1750);

	snprintf(cvalue, 6, "%u", sensors->bh1750_lux);
	create_sysfslike(DIRECTORY, "lum_lux", cvalue, "%s", BH1750);

	snprintf(cvalue, 4, "%u", sensors->bh1750_prc);
	create_sysfslike(DIRECTORY, "lum_percent", cvalue, "%s", BH1750);

	snprintf(cvalue, 5, "%2.1f", sensors->bmp085_temp);
	create_sysfslike(DIRECTORY, "temp", cvalue, "%s", BMP085);

	snprintf(cvalue, 8, "%4.1f", sensors->bmp085_baro);
	create_sysfslike(DIRECTORY, "baro", cvalue, "%s", BMP085);
}

int sensors_init() {
	sensors = malloc(sizeof(*sensors));
	memset(sensors, 0, sizeof(*sensors));

	// TODO config
	i2cfd = open(I2CBUS, O_RDWR);
	if (i2cfd < 0)
		xlog("I2C BUS error");

	init_bmp085();
	init_mqtt();

#ifndef SENSORS_MAIN
	if (pthread_create(&thread_sensors, NULL, &sensors_loop, NULL))
		xlog("Error creating thread");
#endif

	return 0;
}

void sensors_close() {
	if (thread_sensors) {
		if (pthread_cancel(thread_sensors))
			xlog("Error canceling thread");
		if (pthread_join(thread_sensors, NULL))
			xlog("Error joining thread");
	}

	if (i2cfd > 0)
		close(i2cfd);

	if (mqttfd > 0)
		close(mqttfd);
}

static void* sensors_loop(void *arg) {
	if (pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL)) {
		xlog("Error setting pthread_setcancelstate");
		return (void*) 0;
	}

	while (1) {
		read_bh1750();
		read_bmp085();
		// write_sysfslike();
		publish_mqtt();
		sleep(60);
	}
}

#ifdef SENSORS_MAIN
int main(int argc, char **argv) {
	sensors_init();

	read_bh1750();
	read_bmp085();
	publish_mqtt();

	printf("BH1750 raw  %d\n", sensors->bh1750_raw);
	printf("BH1750 raw2 %d\n", sensors->bh1750_raw2);
	printf("BH1750 lux  %d lx\n", sensors->bh1750_lux);
	printf("BH1750 prc  %d %%\n", sensors->bh1750_prc);

	printf("BMP085 temp %d (raw)\n", sensors->bmp085_utemp);
	printf("BMP085 baro %d (raw)\n", sensors->bmp085_ubaro);

	printf("BMP085 temp %0.1f °C\n", sensors->bmp085_temp);
	printf("BMP085 baro %0.1f hPa\n", sensors->bmp085_baro);

	sensors_close();
	return 0;
}
#endif


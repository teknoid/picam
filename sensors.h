// TODO config
#define DIRECTORY			"/ram"
#define I2CBUS				"/dev/i2c-0"

#define BH1750				"BH1750"
#define BH1750_ADDR			0x23
#define BH1750_POWERDOWN	0x00
#define BH1750_POWERON		0x01
#define BH1750_RESET		0x07
#define BH1750_CONTINUHIGH0	0x10
#define BH1750_CONTINUHIGH1	0x11
#define BH1750_CONTINULOW	0x13
#define BH1750_ONETIMEHIGH	0x20
#define BH1750_ONETIMELOW	0x23

#define BMP085				"BMP085"
#define BMP085_ADDR			0x77
#define BMP085_OVERSAMPLE	3

typedef struct sensors_t {
	// BH1750 luminousity
	unsigned int bh1750_raw;
	unsigned int bh1750_lux;
	unsigned int bh1750_prc;

	// BMP085 calibration data
	int bmp085_ac1;
	int bmp085_ac2;
	int bmp085_ac3;
	unsigned int bmp085_ac4;
	unsigned int bmp085_ac5;
	unsigned int bmp085_ac6;
	int bmp085_b1;
	int bmp085_b2;
	int bmp085_mb;
	int bmp085_mc;
	int bmp085_md;

	// BMP085 temperature + barometric pressure
	float bmp085_temp;
	unsigned int bmp085_utemp;
	float bmp085_baro;
	unsigned int bmp085_ubaro;
} sensors_t;

extern sensors_t *sensors;

int sensors_init(void);
void sensors_close(void);

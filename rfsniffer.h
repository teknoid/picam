#define GPIO_PIN			27	//RPi2 Pin13 GPIO_GEN2

#define BUFFER				256

#define STATE_RESET			129 // max 64 bit code -> 128 high/low edges + 1

#define PULSE_COUNTER_MAX	BUFFER * BUFFER

int rfsniffer_init();
int rfsniffer_close();

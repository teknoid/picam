// A: time frame in the evening
#define A_FROM_H		15
#define A_FROM_M		00
#define A_TO_H			22
#define A_TO_M			00
#define A_ACTIVE		1

// B: time frame in the morning
#define B_FROM_H		6
#define B_FROM_M		30
#define B_TO_H			8
#define B_TO_M			00
#define B_ACTIVE		0

#define FLAMINGOSEND	"/usr/local/bin/flamingosend"

// light on: ↑later ↓earlier
#define XMAS_SUNDOWN	3100

// light off: ↑earlier, ↓later
#define XMAS_SUNRISE	3000

typedef enum {
	false, true
} bool;

typedef enum {
	unknown, off, on
} state_t;

int xmas_init(void);
void xmas_close(void);

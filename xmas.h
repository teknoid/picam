typedef struct timing_t {
	int active;								// enabled / disabled
	int wday;								// weekday
	int on_h;								// soonest switch on hour
	int on_m;								// soonest switch on minute
	int off_h;								// latest switch off hour
	int off_m;								// latest switch off minute
	int remote;								// index of remote control unit
	char channel;							// channel of remote control unit
} timing_t;

// light on: ↑later ↓earlier
#define XMAS_SUNDOWN	2500

// light off: ↑earlier, ↓later
#define XMAS_SUNRISE	3000

// the remote control unit
#define WHITE1				1

static const timing_t timings[] = {
	{ 1, 1, 15, 00, 22, 00, WHITE1, 'A' }, // Monday
	{ 1, 2, 15, 00, 22, 00, WHITE1, 'A' },
	{ 1, 3, 15, 00, 22, 00, WHITE1, 'A' },
	{ 1, 4, 15, 00, 22, 00, WHITE1, 'A' },
	{ 1, 5, 15, 00, 23, 00, WHITE1, 'A' },
	{ 1, 6, 15, 00, 23, 00, WHITE1, 'A' },
	{ 1, 0, 15, 00, 23, 00, WHITE1, 'A' }, // Sunday
};

int xmas_init(void);
void xmas_close(void);

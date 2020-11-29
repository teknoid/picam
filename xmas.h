typedef struct timing_t {
	int active;
	int wday;
	int on_h;
	int on_m;
	int off_h;
	int off_m;
	char code;
} timing_t;

static const timing_t timings[] = {
	{ 1, 1, 15, 00, 22, 00, 'A' }, // Monday
	{ 1, 2, 15, 00, 22, 00, 'A' },
	{ 1, 3, 15, 00, 22, 00, 'A' },
	{ 1, 4, 15, 00, 22, 00, 'A' },
	{ 1, 5, 15, 00, 23, 00, 'A' },
	{ 1, 6, 15, 00, 23, 00, 'A' },
	{ 1, 0, 15, 00, 23, 00, 'A' }, // Sunday
};

// light on: ↑later ↓earlier
#define XMAS_SUNDOWN	2500

// light off: ↑earlier, ↓later
#define XMAS_SUNRISE	3000

#define FLAMINGOSEND	"/usr/local/bin/flamingosend"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define ZERO(a) memset(a, 0, sizeof(*a));

int xmas_init(void);
void xmas_close(void);

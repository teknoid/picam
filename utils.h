#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define ZERO(a) memset(a, 0, sizeof(*a));

#define msleep(x) usleep(x*1000)

unsigned long _micros();

int init_micros();

void delay_micros(unsigned int us);

int elevate_realtime(int cpu);

char *printbits64(unsigned long long code, unsigned long long spacemask);

char *printbits(unsigned long code, unsigned long spacemask);

void create_sysfslike(char *dir, char *fname, char *fvalue, const char *fmt, ...);

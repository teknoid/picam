#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define ZERO(a) memset(a, 0, sizeof(*a));

#define msleep(x) usleep(x*1000)

char *printbits(unsigned long code, unsigned long spacemask);

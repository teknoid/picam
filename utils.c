#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "utils.h"

static volatile unsigned long *systReg = 0;

//
// The RT scheduler problem
//
// https://www.raspberrypi.org/forums/viewtopic.php?t=228727
// https://www.codeblueprint.co.uk/2019/10/08/isolcpus-is-deprecated-kinda.html
// https://www.iot-programmer.com/index.php/books/22-raspberry-pi-and-the-iot-in-c/chapters-raspberry-pi-and-the-iot-in-c/33-raspberry-pi-iot-in-c-almost-realtime-linux
//

int init_micros() {
	// based on pigpio source; simplified and re-arranged
	int fdMem = open("/dev/mem", O_RDWR | O_SYNC);
	if (fdMem < 0) {
		perror("Cannot map memory (need sudo?)\n");
		return -1;
	}
	// figure out the address
	FILE *f = fopen("/proc/cpuinfo", "r");
	char buf[1024];
	fgets(buf, sizeof(buf), f); // skip first line
	fgets(buf, sizeof(buf), f); // model name
	unsigned long phys = 0;
	if (strstr(buf, "ARMv6")) {
		phys = 0x20000000;
	} else if (strstr(buf, "ARMv7")) {
		phys = 0x3F000000;
	} else if (strstr(buf, "ARMv8")) {
		phys = 0x3F000000;
	} else {
		perror("Unknown CPU type\n");
		return -1;
	}
	fclose(f);
	systReg = (unsigned long *) mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fdMem, phys + 0x3000);
	return 0;
}

unsigned long _micros() {
	return systReg[1];
}

void delay_micros(unsigned int us) {
	// usleep() on its own gives latencies 20-40 us; this combination gives < 25 us.
	unsigned long start = systReg[1];
	if (us >= 100)
		usleep(us - 50);
	while ((systReg[1] - start) < us)
		;
}

int elevate_realtime(int cpu) {
	// Set our thread to MAX priority
	struct sched_param sp;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	if (sched_setscheduler(0, SCHED_FIFO, &sp))
		return -1;

	// Lock memory to ensure no swapping is done.
	if (mlockall(MCL_FUTURE | MCL_CURRENT))
		return -2;

	// pin thread to CPU
	// add this argument to /boot/cmdline.txt: isolcpus=2,3
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(cpu, &cpuset);
	if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset))
		return -3;

	// This permits realtime processes to use 100% of a CPU, but on a
	// RPi that starves the kernel. Without this there are latencies
	// up to 50 MILLISECONDS.
	system("echo -1 >/proc/sys/kernel/sched_rt_runtime_us");

	return 0;
}

char *printbits64(unsigned long long code, unsigned long long spacemask) {
	unsigned int bits = 64;
	char *out = malloc(bits * 2 + 1);
	char *p = out;
	unsigned long long mask = 0x8000000000000000;
	while (mask) {
		if (code & mask) {
			*p++ = '1';
		} else {
			*p++ = '0';
		}
		if (mask & spacemask) {
			*p++ = ' ';
		}
		mask >>= 1;
	}
	*p++ = '\0';
	return out;
}

char *printbits(unsigned long code, unsigned long spacemask) {
	unsigned int bits = 32;
	char *out = malloc(bits * 2 + 1);
	char *p = out;
	unsigned long mask = 0x80000000;
	while (mask) {
		if (code & mask) {
			*p++ = '1';
		} else {
			*p++ = '0';
		}
		if (mask & spacemask) {
			*p++ = ' ';
		}
		mask >>= 1;
	}
	*p++ = '\0';
	return out;
}

void create_sysfslike(char *dir, char *fname, char *fvalue, const char *fmt, ...) {
	const char *p;
	struct stat st = { 0 };
	char path[128], cp[10], *c;
	int i;
	FILE *fp;
	va_list va;

	// configured directory (remove trailing slash if necessary)
	strcpy(path, dir);
	if (path[strlen(path) - 1] == '/')
		path[strlen(path) - 1] = '\0';
	if (stat(path, &st) == -1)
		if (mkdir(path, 0755))
			perror(strerror(errno));

	// paths from varargs with format string
	va_start(va, fmt);
	for (p = fmt; *p != '\0'; p++) {
		if (*p != '%')
			continue;
		strcat(path, "/");
		switch (*++p) {
		case 'c':
			i = va_arg(va, int);
			sprintf(cp, "%c", i);
			strcat(path, cp);
			break;
		case 'd':
			i = va_arg(va, int);
			sprintf(cp, "%d", i);
			strcat(path, cp);
			break;
		case 's':
			c = va_arg(va, char *);
			strcat(path, c);
			break;
		}
		if (stat(path, &st) == -1)
			if (mkdir(path, 0755))
				perror(strerror(errno));
	}
	va_end(va);

	// file
	strcat(path, "/");
	strcat(path, fname);
	if ((fp = fopen(path, "w")) == NULL)
		perror(strerror(errno));
	while (*fvalue)
		fputc(*fvalue++, fp);
	fputc('\n', fp);
	fclose(fp);
}


#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <syslog.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "utils.h"

static int xlog_output = 0;
static FILE *xlog_file;

//
// The RT scheduler problem
//
// https://www.raspberrypi.org/forums/viewtopic.php?t=228727
// https://www.codeblueprint.co.uk/2019/10/08/isolcpus-is-deprecated-kinda.html
// https://www.iot-programmer.com/index.php/books/22-raspberry-pi-and-the-iot-in-c/chapters-raspberry-pi-and-the-iot-in-c/33-raspberry-pi-iot-in-c-almost-realtime-linux
//

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

void xlog_init(int output, const char *filename) {
	xlog_output = output;

	// print to stdout
	if (output == XLOG_STDOUT)
		return;

	// write to syslog
	if (output == XLOG_SYSLOG)
		return;

	// write to a logfile
	if (output == XLOG_FILE) {
		if (xlog_file == 0) {
			xlog_file = fopen(filename, "a");
			if (xlog_file == 0) {
				perror("error opening logfile!");
				exit(EXIT_FAILURE);
			}
		}
		return;
	}
}

void xlog(const char *format, ...) {
	char BUFFER[256];
	va_list vargs;

	if (xlog_output == XLOG_STDOUT) {
		va_start(vargs, format);
		vsprintf(BUFFER, format, vargs);
		va_end(vargs);
		printf(BUFFER);
		printf("\n");
		return;
	}

	if (xlog_output == XLOG_SYSLOG) {
		va_start(vargs, format);
		vsprintf(BUFFER, format, vargs);
		va_end(vargs);
		syslog(LOG_NOTICE, BUFFER);
		return;
	}

	if (xlog_output == XLOG_FILE) {
		time_t timer;
		struct tm *tm_info;

		time(&timer);
		tm_info = localtime(&timer);
		strftime(BUFFER, 26, "%d.%m.%Y %H:%M:%S", tm_info);

		fprintf(xlog_file, "%s: ", BUFFER);
		va_start(vargs, format);
		vfprintf(xlog_file, format, vargs);
		va_end(vargs);
		fprintf(xlog_file, "\n");
		fflush(xlog_file);
	}
}

void xlog_close() {
	if (xlog_file) {
		fflush(xlog_file);
		fclose(xlog_file);
	}
}
char* printbits64(uint64_t code, uint64_t spacemask) {
	uint64_t mask = 0x8000000000000000;
	uint16_t bits = 64;
	char *out = malloc(bits * 2 + 1);
	char *p = out;
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

char* printbits(uint32_t code, uint32_t spacemask) {
	uint32_t mask = 0x80000000;
	uint16_t bits = 32;
	char *out = malloc(bits * 2 + 1);
	char *p = out;
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
			c = va_arg(va, char*);
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


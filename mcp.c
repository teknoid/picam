#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "utils.h"
#include "flamingo.h"
#include "sensors.h"
#include "webcam.h"
#include "xmas.h"
#include "gpio.h"
#include "mcp.h"

static mcp_config_t *cfg;

static void sig_handler(int signo) {
	xlog("MCP received signal %d", signo);
}

static void daemonize() {
	pid_t pid;

	/* Fork off the parent process */
	pid = fork();
	if (pid < 0)
		exit(EXIT_FAILURE);

	if (pid > 0)
		exit(EXIT_SUCCESS);

	if (setsid() < 0)
		exit(EXIT_FAILURE);

	/* Catch, ignore and handle signals */
	signal(SIGCHLD, SIG_IGN);
	signal(SIGHUP, SIG_IGN);

	/* Fork off for the second time*/
	pid = fork();
	if (pid < 0)
		exit(EXIT_FAILURE);

	if (pid > 0)
		exit(EXIT_SUCCESS);

	/* Set new file permissions, set new root, close standard file descriptors */
	umask(0);
	chdir("/");

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	xlog("MCP forked into background");
}

static void mcp_init() {
	if (gpio_init() < 0)
		exit(EXIT_FAILURE);

	if (flamingo_init() < 0)
		exit(EXIT_FAILURE);

	if (sensors_init() < 0)
		exit(EXIT_FAILURE);

	if (webcam_init() < 0)
		exit(EXIT_FAILURE);

	if (xmas_init() < 0)
		exit(EXIT_FAILURE);

	xlog("all modules initialized");
}

static void mcp_close() {
	xmas_close();
	webcam_close();
	sensors_close();
	flamingo_close();
	gpio_close();

	xlog("all modules closed");
}

int main(int argc, char **argv) {
	xlog_init(XLOG_SYSLOG, NULL);
	xlog("MCP initializing");

	cfg = malloc(sizeof(*cfg));
	memset(cfg, 0, sizeof(*cfg));

	// parse command line arguments
	int c;
	while ((c = getopt(argc, argv, "d")) != -1)
		switch (c) {
		case 'd':
			cfg->daemonize = 1;
			break;
		}

	// fork into background
	// not necessary anymore, see http://jdebp.eu/FGA/unix-daemon-design-mistakes-to-avoid.html
	if (cfg->daemonize)
		daemonize();

	// install signal handler
	if (signal(SIGINT, sig_handler) == SIG_ERR) {
		xlog("can't catch SIGINT");
		exit(EXIT_FAILURE);
	}
	if (signal(SIGTERM, sig_handler) == SIG_ERR) {
		xlog("can't catch SIGTERM");
		exit(EXIT_FAILURE);
	}
	if (signal(SIGHUP, sig_handler) == SIG_ERR) {
		xlog("can't catch SIGHUP");
		exit(EXIT_FAILURE);
	}

	// initialize modules
	mcp_init();

	xlog("MCP online");
	pause();

	// close modules
	mcp_close();

	xlog("MCP terminated");
	return EXIT_SUCCESS;
}

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "mcp.h"

static mcp_config_t *cfg;

static void sig_handler(int signo) {
	syslog(LOG_NOTICE, "MCP received signal %d", signo);
}

static void daemonize() {
	pid_t pid;

	/* Fork off the parent process */
	pid = fork();
	if (pid < 0) {
		exit(EXIT_FAILURE);
	}
	if (pid > 0) {
		exit(EXIT_SUCCESS);
	}
	if (setsid() < 0) {
		exit(EXIT_FAILURE);
	}

	/* Catch, ignore and handle signals */
	signal(SIGCHLD, SIG_IGN);
	signal(SIGHUP, SIG_IGN);

	/* Fork off for the second time*/
	pid = fork();
	if (pid < 0) {
		exit(EXIT_FAILURE);
	}
	if (pid > 0) {
		exit(EXIT_SUCCESS);
	}

	/* Set new file permissions, set new root, close standard file descriptors */
	umask(0);
	chdir("/");

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	syslog(LOG_NOTICE, "MCP forked into background");
}

static void mcp_init() {
	cfg->flamingocfg = flamingo_default_config();
	cfg->flamingocfg->flamingo_handler = 0;
	cfg->flamingocfg->pattern = 0;
	cfg->flamingocfg->quiet = 1;
	if (flamingo_init(0, 0) < 0) {
		exit(EXIT_FAILURE);
	}

//	cfg->rfsniffercfg = rfsniffer_default_config();
//	cfg->rfsniffercfg->rfsniffer_handler = &rfsniffer_syslog_handler;
//	cfg->rfsniffercfg->quiet = 1;
//	cfg->rfsniffercfg->sysfslike = "/ram";
//	cfg->rfsniffercfg->realtime_mode = 1;
//	cfg->rfsniffercfg->noise = 300;
//	if (rfsniffer_init() < 0) {
//		exit(EXIT_FAILURE);
//	}

	if (mcp3204_init() < 0) {
		exit(EXIT_FAILURE);
	}

	if (webcam_init() < 0) {
		exit(EXIT_FAILURE);
	}

	if (xmas_init() < 0) {
		exit(EXIT_FAILURE);
	}

	if (lumi_init() < 0) {
		exit(EXIT_FAILURE);
	}

	syslog(LOG_NOTICE, "all modules successfully initialized");
}

static void mcp_close() {
	lumi_close();
	webcam_close();
	xmas_close();
//	rfsniffer_close();
	flamingo_close();

	syslog(LOG_NOTICE, "all modules successfully closed");
}

int main(int argc, char **argv) {
	syslog(LOG_NOTICE, "MCP initializing");

	cfg = malloc(sizeof(*cfg));
	memset(cfg, 0, sizeof(*cfg));

	// parse command line arguments
	int c;
	while ((c = getopt(argc, argv, "d")) != -1) {
		switch (c) {
		case 'd':
			cfg->daemonize = 1;
			break;
		}
	}

	// fork into background
	// not necessary anymore, see http://jdebp.eu/FGA/unix-daemon-design-mistakes-to-avoid.html
	if (cfg->daemonize) {
		daemonize();
	}

	// install signal handler
	if (signal(SIGINT, sig_handler) == SIG_ERR) {
		syslog(LOG_NOTICE, "can't catch SIGINT");
		exit(EXIT_FAILURE);
	}
	if (signal(SIGTERM, sig_handler) == SIG_ERR) {
		syslog(LOG_NOTICE, "can't catch SIGTERM");
		exit(EXIT_FAILURE);
	}
	if (signal(SIGHUP, sig_handler) == SIG_ERR) {
		syslog(LOG_NOTICE, "can't catch SIGHUP");
		exit(EXIT_FAILURE);
	}

	// initialize modules
	mcp_init();

	syslog(LOG_NOTICE, "MCP online");
	pause();

	// close modules
	mcp_close();

	syslog(LOG_NOTICE, "MCP terminated");
	return EXIT_SUCCESS;
}

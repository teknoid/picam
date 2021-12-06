#include "flamingo.h"
#include "rfsniffer.h"
#include "mcp3204.h"
#include "webcam.h"
#include "xmas.h"
#include "lumi.h"

typedef struct mcp_config_t {
	int daemonize;
	flamingo_config_t *flamingocfg;
	rfsniffer_config_t *rfsniffercfg;
} mcp_config_t;

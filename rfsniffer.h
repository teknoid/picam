// Events
#define E_PAYLOAD			0
#define E_BUTTON			1
#define E_ROLLING			2
#define E_TEMPERATURE		3
#define E_HUMIDITY			4
#define E_BATTERY			5

// Protocols
#define P_SYNC				0
#define P_ANALYZE			255
#define P_ANALYZE_SYNC		254

#define P_NEXUS				1
#define P_FLAMINGO28		2
#define P_FLAMINGO24		3
#define P_FLAMINGO32		4
#define P_FLAMINGO32M		5

typedef struct rfsniffer_event_t {
	unsigned short protocol;
	unsigned long long raw;
	int device;
	int channel;
	int repeat;
	unsigned char key;
	int value;
	unsigned char ikey1;
	int ivalue1;
	unsigned char ikey2;
	int ivalue2;
	unsigned char ikey3;
	int ivalue3;
	unsigned char fkey1;
	float fvalue1;
	char *message;
} rfsniffer_event_t;

typedef void (*rfsniffer_handler_t)(rfsniffer_event_t *event);

typedef struct rfsniffer_config_t {
	unsigned char rx;
	unsigned char tx;
	unsigned char analyzer_mode;
	unsigned char realtime_mode;
	unsigned char timestamp;
	unsigned char pulse_counter_active;
	unsigned char decoder_delay;
	unsigned char bits_to_sample;
	unsigned char collect_identical_codes;
	unsigned char sync_on_0;
	unsigned char sync_on_1;
	unsigned char sample_on_0;
	unsigned char sample_on_1;
	unsigned long sync_min;
	unsigned long sync_max;
	unsigned long bitdivider;
	unsigned short noise;
	unsigned char verbose;
	unsigned char quiet;
	unsigned char syslog;
	unsigned char json;
	char *sysfslike;
	rfsniffer_handler_t rfsniffer_handler;
} rfsniffer_config_t;

rfsniffer_config_t *rfsniffer_default_config();
void rfsniffer_syslog_handler(rfsniffer_event_t *event);
void rfsniffer_stdout_handler(rfsniffer_event_t *event);
int rfsniffer_init();
int rfsniffer_close();

void rfcodec_decode(unsigned char protocol, unsigned long long code, unsigned char repeat);
void rfcodec_set_config(rfsniffer_config_t *cfg);

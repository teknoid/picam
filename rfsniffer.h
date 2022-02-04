// Events
#define E_PAYLOAD			0
#define E_BUTTON			1
#define E_ROLLING			2
#define E_TEMPERATURE		3
#define E_HUMIDITY			4
#define E_BATTERY			5

// Protocols
#define P_NEXUS				1
#define P_FLAMINGO28		2
#define P_FLAMINGO24		3
#define P_FLAMINGO32		4
#define P_FLAMINGO32M		5
#define P_ANALYZE			255

#define NEXUS_PULSE			500
#define NEXUS_SYNC_MIN		3800
#define NEXUS_SYNC_MAX		4000
#define NEXUS_DIVIDER		NEXUS_PULSE * 3

typedef struct rfsniffer_event_t {
	uint64_t raw;
	uint32_t code;
	uint16_t device;
	uint8_t protocol;
	uint8_t channel;
	uint8_t repeat;
	uint8_t key;
	int16_t value;
	uint8_t ikey1;
	int16_t ivalue1;
	uint8_t ikey2;
	int16_t ivalue2;
	uint8_t ikey3;
	int16_t ivalue3;
	uint8_t fkey1;
	float fvalue1;
	char *message;
} rfsniffer_event_t;

typedef void (*rfsniffer_handler_t)(rfsniffer_event_t *event);

typedef struct rfsniffer_config_t {
	const char *rx;
	const char *tx;
	uint32_t sync_min;
	uint32_t sync_max;
	uint32_t bitdivider;
	uint16_t noise;
	uint16_t decoder_delay;
	uint8_t analyzer_mode;
	uint8_t realtime_mode;
	uint8_t stream_mode;
	uint8_t timestamp;
	uint8_t pulse_counter_active;
	uint8_t bits_to_sample;
	uint8_t collect_identical_codes;
	uint8_t sync_on_0;
	uint8_t sync_on_1;
	uint8_t sample_on_0;
	uint8_t sample_on_1;
	uint8_t verbose;
	uint8_t validate;
	uint8_t quiet;
	uint8_t syslog;
	uint8_t json;
	char *sysfslike;
	rfsniffer_handler_t rfsniffer_handler;
} rfsniffer_config_t;

rfsniffer_config_t* rfsniffer_default_config();
int rfsniffer_init();
int rfsniffer_close();

void rfsniffer_syslog_handler(rfsniffer_event_t *event);
void rfsniffer_stdout_handler(rfsniffer_event_t *event);

void* realtime_decoder(void *arg);
void* stream_decoder(void *arg);

void* realtime_sampler(void *arg);
void* stream_sampler(void *arg);

void matrix_init();
void matrix_store(uint8_t x, uint64_t code);
void matrix_decode();
void matrix_decode_protocol(uint8_t x);

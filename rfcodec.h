// Events
#define E_PAYLOAD			0
#define E_BUTTON			1
#define E_TEMPERATURE		2
#define E_HUMIDITY			3
#define E_BATTERY			4

// Protocols
#define P_SYNC				0
#define P_ANALYZE			255
#define P_ANALYZE_SYNC		254

#define P_NEXUS				1
#define P_FLAMINGO28		2
#define P_FLAMINGO24		3
#define P_FLAMINGO32		4
#define P_FLAMINGO32M		5

typedef void (*rfsniffer_handler_t)(unsigned short protocol, unsigned long long raw, int device, int channel, int event, int value);

void decode(unsigned char protocol, unsigned long long code, unsigned char repeat);

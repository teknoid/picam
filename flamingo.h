#define RX					2
#define TX					0

// transmitter id's of our known remote control units
const static unsigned long REMOTES[5] = { 0x53cc, 0x835a, 0x31e2, 0x295c, 0x272d };
//											White1	White2	White3	Black	SF500

// timings for 28bit rc1 and 24bit rc4 patterns
// tested with 1 C 1: min 180 max 350 --> 330 is closest to the original remote
#define T1					330
const static unsigned int T1X2 = T1 * 2;
const static unsigned int T1X3 = T1 * 3;
const static unsigned int T1X15 = T1 * 15;
const static unsigned int T1X31 = T1 * 31;
const static unsigned int T1SMIN = T1X15 - 80;
const static unsigned int T1SMAX = T1X15 + 80;
const static unsigned int T4SMIN = T1X31 - 100;
const static unsigned int T4SMAX = T1X31 + 100;

// timings for 32bit rc2 patterns
#define T2H					200
#define T2L					330
const static unsigned int T2X = (T2H + T2L) * 2 + T2L; // 1390 - low data bit delay
const static unsigned int T2Y = T2X / 2; // 695 - decides 0 / 1
const static unsigned int T2S1 = T2X * 2; // 2780 - low sync delay (FA500R)
const static unsigned int T2S1MIN = T2S1 - 50;
const static unsigned int T2S1MAX = T2S1 + 50;
const static unsigned int T2S2 = T2L * 8; // 2640 - low sync delay (SF500R)
const static unsigned int T2S2MIN = T2S2 - 50;
const static unsigned int T2S2MAX = T2S2 + 50;

// timings for 32bit rc3 multibit patterns
#define T3H					220
#define T3L					330
const static unsigned int T3X = T3H + T3L + T3L; // 880 - low delay to next clock
const static unsigned int T3Y = T3H + T3L; // 550 - decides if clock or data bit
const static unsigned int T3S = 9250; // don't know how to calculate
const static unsigned int T3SMIN = T3S - 50;
const static unsigned int T3SMAX = T3S + 50;

#define REPEAT_PAUSE1		5555
#define REPEAT_PAUSE2		9999

#define SPACEMASK_FA500		0x01000110
#define SPACEMASK_SF500		0x00010110

// encryption key
const static unsigned char CKEY[16] = { 9, 6, 3, 8, 10, 0, 2, 12, 4, 14, 7, 5, 1, 15, 11, 13 };

// decryption key (invers encryption key - exchanged index & value)
const static unsigned char DKEY[16] = { 5, 12, 6, 2, 8, 11, 1, 10, 3, 0, 4, 14, 7, 15, 9, 13 };

typedef void (*flamingo_handler_t)(unsigned int, unsigned char, unsigned char, unsigned char);

int flamingo_init(int pattern, flamingo_handler_t handler);
void flamingo_close();

void flamingo_send_FA500(int remote, char channel, int command, int rolling);
void flamingo_send_SF500(int remote, char channel, int command);

unsigned long encrypt(unsigned long message);
unsigned long decrypt(unsigned long code);

unsigned long encode_FA500(unsigned int xmitter, unsigned char channel, unsigned char command, unsigned char payload, unsigned char rolling);
unsigned long encode_SF500(unsigned int xmitter, unsigned char channel, unsigned char command, unsigned char payload);

void decode_FA500(unsigned long message, unsigned int *xmitter, unsigned char *channel, unsigned char *command, unsigned char *payload, unsigned char *rolling);
void decode_SF500(unsigned long message, unsigned int *xmitter, unsigned char *channel, unsigned char *command, unsigned char *payload);

/*

 not necessary anymore to store codes here as we now able to calculate and encrypt them, left here only for debugging purposes

 const static unsigned long FLAMINGO[REMOTES * 4][8] = {
 //	Remote Name
 //	{ , , , , , , ,  }, // 4 codes for OFF + 4 codes for ON


 White 1
 OFF												ON
 { 0x0251b29a, 0x0253174e, 0x02498346, 0x0252e83a, 0x02796056, 0x025469de, 0x02779c76, 0x0274b462 },
 { 0x02783f5d, 0x024f5b55, 0x0256408d, 0x025a5c29, 0x0261b985, 0x0246c8d1, 0x025ab199, 0x025f0bf9 },
 { 0x0e6bd68d, 0x0e7be29d, 0x0e70a7f5, 0x0e763e15, 0x0e738cb9, 0x0e7432d1, 0x0e7ff619, 0x0e4e8e31 },
 { 0x06492136, 0x065c85ba, 0x064bdeb2, 0x0667d10a, 0x06467c9a, 0x066e7082, 0x064e6086, 0x0653b3aa },

 White 2
 OFF												ON
 { 0x02665c0e, 0x0250b446, 0x02503f7a, 0x027d905a, 0x027e94d6, 0x0248379e, 0x025674b6, 0x02773962 },
 { 0x02412f1d, 0x024a5315, 0x024ca84d, 0x027b1769, 0x02404f59, 0x024469f9, 0x0272c385, 0x025d2111 },
 { 0x0e5951cd, 0x0e42161d, 0x0e795435, 0x0e42fb95, 0x0e6ffaf9, 0x0e7c8891, 0x0e6f9799, 0x0e7093b1 },
 { 0x0656c8f6, 0x064de17a, 0x06696072, 0x065d7e4a, 0x066726da, 0x067cc842, 0x0661d506, 0x066cd2ea },

 White 3
 OFF												ON
 { 0x0266c31a, 0x0261fbce, 0x024173c6, 0x02469eba, 0x02455f62, 0x02797896, 0x025fd19e, 0x024acf76 },
 { 0x0244b4e9, 0x0256b11d, 0x027ebad5, 0x024f420d, 0x027d6b91, 0x0278c8d9, 0x02589339, 0x025e1685 },
 { 0x0e75ed15, 0x0e5afb0d, 0x0e6f76dd, 0x0e648135, 0x0e678ab1, 0x0e4101b9, 0x0e573751, 0x0e44f499 },
 { 0x064d478a, 0x066c4ff6, 0x06726cba, 0x066fd632, 0x0656062a, 0x06493a1a, 0x066d83c2, 0x06624006 },

 Black
 OFF												ON
 { 0x024f4746, 0x02437c3a, 0x026e6e9a, 0x026ddf4e, 0x025b2c56, 0x0255f9de, 0x02699876, 0x02488c62 },
 { 0x024dcb5d, 0x02586b55, 0x027a288d, 0x024b4029, 0x02712d99, 0x0259e3f9, 0x02756985, 0x026ab8d1 },
 { 0x0e5e4e8d, 0x0e764a9d, 0x0e5afff5, 0x0e678e15, 0x0e70b8b9, 0x0e640ed1, 0x0e4cee19, 0x0e4e7e31 },
 { 0x0649cd36, 0x067321ba, 0x066edab2, 0x066fbd0a, 0x066d249a, 0x0642d082, 0x067a9c86, 0x06658faa }
 };
 */

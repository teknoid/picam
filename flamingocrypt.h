// encryption key
const static unsigned char CKEY[16] = { 9, 6, 3, 8, 10, 0, 2, 12, 4, 14, 7, 5, 1, 15, 11, 13 };

// decryption key (invers encryption key - exchanged index & value)
const static unsigned char DKEY[16] = { 5, 12, 6, 2, 8, 11, 1, 10, 3, 0, 4, 14, 7, 15, 9, 13 };

unsigned long encode(unsigned int txid, unsigned char channel, unsigned char command, unsigned char payload);

unsigned long encrypt(unsigned long message, unsigned char rolling);

unsigned long decrypt(unsigned long code);

unsigned char decode_channel(unsigned long message);

unsigned char decode_command(unsigned long message);

unsigned char decode_payload(unsigned long message);

unsigned int decode_txid(unsigned long message);

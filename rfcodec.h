/*
 *  main
 */
void rfcodec_decode(uint8_t protocol, uint64_t code, uint8_t repeat);
void rfcodec_test(int argc, char **argv);
uint32_t rfcodec_decode_0110(uint64_t in);

/*
 *  nexus part
 */
int nexus_test(int argc, char **argv);
void nexus_decode(uint8_t protocol, uint64_t raw, uint8_t repeat);

/*
 *  flamingo part
 */
int flamingo_test(int argc, char **argv);

void flamingo28_decode(uint8_t protocol, uint64_t raw, uint8_t repeat);
void flamingo24_decode(uint8_t protocol, uint64_t raw, uint8_t repeat);
void flamingo32_decode(uint8_t protocol, uint64_t raw, uint8_t repeat);

uint32_t flamingo28_encode(uint16_t xmitter, char channel, char command, char payload, char rolling);
uint32_t flamingo24_encode(uint16_t xmitter, char channel, char command, char payload);
uint32_t flamingo32_encode(uint16_t xmitter, char channel, char command, char payload);

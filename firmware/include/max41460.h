#include <stdint.h> // for uint8_t, uint16_t, etc.

void configSPI();
uint8_t transferSPI(uint8_t byte);
void startTransmitter();
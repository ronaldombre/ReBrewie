#include <Arduino.h>

uint8_t TWIReadACK(void);

uint8_t TWIReadNACK(void);

void TWIWrite(uint8_t);

void TWIStart(void);
//send stop signal
void TWIStop(void);

uint8_t TWIGetStatus(void);

void TWIInit(void);

uint8_t PressureReadAll(int16_t*, uint16_t*, uint8_t);

uint8_t ScanTWI();

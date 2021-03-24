#include "Pressure.h"

uint8_t TWIReadACK(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    uint32_t timeout = millis();
    while ((TWCR & (1<<TWINT)) == 0) {
      if (millis() - timeout > 20) {
        break;
      }
    }
    return TWDR;
}

uint8_t TWIReadNACK(void) {
    TWCR = (1<<TWINT)|(1<<TWEN);
    uint32_t timeout = millis();
    while ((TWCR & (1<<TWINT)) == 0) {
      if (millis() - timeout > 20) {
        break;
      }
    }
    return TWDR;
}

void TWIWrite(uint8_t u8data) {
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    uint32_t timeout = millis();
    while ((TWCR & (1<<TWINT)) == 0) {
      if (millis() - timeout > 20) {
        break;
      }
    }
}

void TWIStart(void) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}
//send stop signal
void TWIStop(void) {
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

uint8_t TWIGetStatus(void) {
    uint8_t status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}

void TWIInit(void) {
    //set SCL to 400kHz
    TWCR = 0;
    delay(10);
    TWSR = 0x00;
    TWBR = 72;
    //enable TWI
    TWCR = (1<<TWEN);
}

uint8_t PressureReadAll(int16_t *pressure, uint16_t *temperature, uint8_t address) {
    TWIStart();
    if (TWIGetStatus() != 0x08)
        return 1;
    //select devise and send A2 A1 A0 address bits
    TWIWrite(address << 1);
    
    if (TWIGetStatus() != 0x18)
        return 1;
    //send the rest of address
    TWIWrite(0);
    
    if (TWIGetStatus() != 0x28)
        return 1;
    //send start
    TWIStart();

    if (TWIGetStatus() != 0x10)
        return 1;
    //select devise and send read bit
    TWIWrite((address << 1)|1);
    
    if (TWIGetStatus() != 0x40)
        return 1;
    *pressure = TWIReadACK() << 8;
    *pressure |= TWIReadACK();
    *temperature = TWIReadACK() << 8;
    *temperature |= TWIReadNACK();
    if (TWIGetStatus() != 0x58)
        return 1;
    TWIStop();
    return 0;
}

uint8_t ScanTWI() {
  uint8_t address = 255;
  for (uint8_t x = 0; x<129; x++) {
    TWIStart();
    //if (TWIGetStatus() != 0x08)
        //return 1;
    //select devise and send A2 A1 A0 address bits
    TWIWrite(x << 1);    
    if (TWIGetStatus() == 0x18) {
      address = x;
      break;
    }
    TWIStop();
  }
  return address;
}

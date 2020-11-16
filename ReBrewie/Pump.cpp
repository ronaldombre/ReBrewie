#include "Pump.h"

#define PUMP_DRY      220
#define PUMP_DRY_RPM  140

Pump::Pump(uint8_t pin, bool channel) {
  _pumpPin = pin;
  pinMode(_pumpPin, OUTPUT);
  digitalWrite(_pumpPin, LOW);
  _pumpChannel = channel;
}

void Pump::Pump_Speed_Control(uint16_t current) {
  _pumpCurrent = current;
  // Calculate pump RPM and flow
  _pumpTach = (uint8_t)((uint16_t)*pumpTicks*100/((millis() - _pumpTime)/10));
  _pumpFlow += (uint16_t)(*pumpTicks);
  *pumpTicks = 0;
  if (_pumpEnable) {
    _pumpTime = millis();
    writeDAC();

    if (_pumpSpeed == 255) {
      if (_pumpCount++ > 4) {
        _pumpDiag = 1;
      }
      if (_pumpCurrent < PUMP_DRY && _pumpSpeed == 255) {
        // Pump may be running dry
        _dryRun++;
      } else {
        if (_dryRun > 0) {
          _dryRun--;
        }
      }
      if (_dryRun > 5) {
        _pumpDiag = 2;
        if (_dryRun > 10  && !_pumpDry) {
          _pumpDry = true;
          _pumpDiag = 0;
          digitalWrite(_pumpPin, LOW);
          //setPumpSpeed(0);
          _running = false;
        }
      }
    }
    
    // Restart pump and try again
    /*if (_dryRun > 8 && !_pumpDry) {
      _dryRun = 0;
      digitalWrite(_pumpPin, HIGH);
      setPumpSpeed(130);
      _pumpDiag = 255;
      _running = true;
      _pumpTries++;
    }
    if (_pumpTries > 30) {
      //_pumpDry = true;
      _dryRun = 0;
      _wetRun = 0;
      //_pumpDiag = 2;
    }
    if (_pumpSpeed < 255) {
      _pumpTries++;
    }*/
  } else {
    _dryRun = 0;
    _running = false;
    _pumpDiag = 0;
  }
}

void Pump::setPumpSpeed(uint16_t pumpSpeed) {
  _pumpSpeed = pumpSpeed;
  _pumpCount = 0;
  writeDAC();
  if (_pumpSpeed == 0) {
    _dryRun = 0;
    _running = false;
    _pumpDiag = 0;
    _pumpEnable = false;
  } else {
    _running = true;
    _pumpDiag = pumpSpeed;
    digitalWrite(_pumpPin, HIGH);
    _pumpEnable = true;
    _pumpDry = false;
    _pumpTries = 0;
    //_pumpSpeed = 255;
    //writeDAC();
    //_pumpSpeed = pumpSpeed;
  }
}

void Pump::writeDAC() {
  uint16_t dataPack = _pumpSpeed << 4;
  if (_pumpChannel) {
    dataPack |= 0x8000;
  } else {
    dataPack &= ~0x8000;
  }
  dataPack |= 0x1000;
  uint8_t data1 = (uint16_t)dataPack >> 8;
  uint8_t data2 = dataPack & 0x00FF;
 
  //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  PORTB &= ~0x10;
  //digitalWrite(10, LOW);
  SPI.transfer(data1);
  SPI.transfer(data2);
  //digitalWrite(10, HIGH);
  PORTB |= 0x10;
  SPI.endTransaction();
  //digitalWrite(9, LOW);
  PORTH &= ~0x40;
  delayMicroseconds(100);
  PORTH |= 0x40;
  //digitalWrite(9, HIGH);
}

bool Pump::isRunning() {
  return _running;
}

uint8_t Pump::pumpTach() {
  return _pumpTach;
}

uint16_t Pump::pumpFlow() {
  return _pumpFlow;
}

uint8_t Pump::pumpDiag() {
  return _pumpDiag;
}

bool Pump::pumpIsDry() {
  return _pumpDry;
}

void Pump::pumpFlowReset() {
  _pumpFlow = 0;
}

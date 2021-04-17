#include "Pump.h"
#include "Valves.h"

// Pump L/min = 0.04 * _pumpSpeed + 0.8

Pump::Pump(uint8_t pin, bool channel) {
  _pumpPin = pin;
  pinMode(_pumpPin, OUTPUT);
  digitalWrite(_pumpPin, LOW);
  _pumpChannel = channel;
  _pumpIsClogged = false;
  _pumpDry = false;
}

void Pump::Pump_Speed_Control(uint16_t current) {
  _pumpCurrent = current;
  // Calculate pump RPM and flow
  _pumpTach = (uint8_t)((uint16_t)*pumpTicks*100/((millis() - _pumpTime)/10));
  _pumpTime = millis();
  _pumpFlow += (uint16_t)(*pumpTicks);
  *pumpTicks = 0;
  // Expected current for speed:
  _expectedCurrent = 0.007155*_pumpSpeed*_pumpSpeed - 0.346564*_pumpSpeed + 28.425309;
  // Expected RPM for speed:
  _expectedRPM = -0.001234*_pumpSpeed*_pumpSpeed + 0.932027*_pumpSpeed - 10.832309;
  _percentLoad = ((_expectedCurrent - current)/_expectedCurrent + (_expectedRPM - _pumpTach)/_expectedRPM)*100.0;
  if (_pumpEnable) {
    switch(_pumpState) {
      case 0: // Running
        if (_pumpCount++ > 4) {
          _pumpDiag = 1;
          if (_pumpTach == 0) {
            _cloggedCount++;
            if (_cloggedCount > 4) {
              _pumpIsClogged = true;
            }
          } else {
            _pumpIsClogged = false;
          }
        }
        if (_pumpCurrent < (_expectedCurrent*.7)) {
          // Pump may be running dry
          _dryRun++;
          if (_pumpCurrent < 35) {
            // Not on? Resend DAC value
            writeDAC();
          }
        } else {
          if (_dryRun > 0) {
            _dryRun--;
          }
        }
        if (_dryRun > 5) {
          _pumpDiag = 2;
          if (_dryRun > 15  && !_pumpDry) {
            _pumpState++;
            _pumpDiag = 0;
            _pumpSpeed = 0;
            writeDAC();
            _running = false;
            _pumpCount = 0;
          }
        }
        break;
      case 1: // Sensing Dry
        if (_pumpOut) {
          // Pump out special workflow
          setValve(VALVE_BOIL_RET, VALVE_PINCH_ANGLE);
          setValve(VALVE_MASH_RET, VALVE_PINCH_ANGLE);
          setValve(VALVE_BOIL_IN, VALVE_CLOSE);
          setValve(VALVE_MASH_IN, VALVE_CLOSE);
          _pumpState++;
        } else {
          _pumpDry = true;
          _pumpState = 0;
        }
        break;
      case 2: // Pump again to remove every drop of water
        if (_pumpCount++ > 4) {
          setPumpSpeed(110);
          _pumpState++;
          _pumpCount = 0;
          _dryRun = 0;
        } else if (_pumpCount > 1) {
          setValve(VALVE_BOIL_RET, VALVE_PINCH_ANGLE);
          setValve(VALVE_MASH_RET, VALVE_CLOSE);
        }
        break;
      case 3: // Stop pumping
        if (_pumpCount++ > 3) {
          if (_pumpCurrent < (_expectedCurrent*.80)) {
            // Pump may be running dry
            _pumpDiag = 2;
            _dryRun++;
          } else {
            if (_dryRun > 0) {
              _dryRun--;
            }
          }
          if (_pumpCount > 20 || _dryRun > 5) {
            _pumpState = 0;
            _pumpCount = 0;
            _pumpDry = true;
            _pumpOut = false;
            setValve(VALVE_MASH_RET, VALVE_CLOSE);
            setValve(VALVE_BOIL_RET, VALVE_CLOSE);
          }
        }
        break;
      default: 
        _pumpState = 0;
        break;
    }
  } else {
    _dryRun = 0;
    _running = false;
    _pumpDiag = 0;
  }
}

void Pump::setPumpSpeed(uint16_t pumpSpeed) {
  if (pumpSpeed > 220) {
    pumpSpeed = 220;
  }
  _pumpSpeed = pumpSpeed;
  _pumpCount = 0;
  writeDAC();
  _pumpIsClogged = false;
  _cloggedCount = 0;
  if (_pumpSpeed == 0) {
    _dryRun = 0;
    _running = false;
    _pumpDiag = 0;
    _pumpEnable = false;
    _pumpOut = false;
    //digitalWrite(_pumpPin, LOW);
  } else {
    _running = true;
    _pumpDiag = 255;
    digitalWrite(_pumpPin, HIGH);
    _pumpEnable = true;
    _pumpDry = false;
    _pumpTries = 0;
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
 
  PORTB &= ~0x10;
  SPI.transfer(data1);
  SPI.transfer(data2);
  PORTB |= 0x10;
  SPI.endTransaction();
}

void Pump::setPumpOut() {
  _pumpOut = true;
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

bool Pump::pumpIsClogged() {
  bool clogged = _pumpIsClogged;
  _pumpIsClogged = false;
  return clogged;
}

void Pump::pumpFlowReset() {
  _pumpFlow = 0;
}
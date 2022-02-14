#include "Heater.h"

Heater::Heater(uint8_t pinHeater) {
  _heatMask = digitalPinToBitMask(pinHeater);
  uint8_t port = digitalPinToPort(pinHeater);

  _heatDir = portModeRegister(port);
  _heatPort = portOutputRegister(port);
  *_heatDir |= _heatMask;
  *_heatPort &= ~_heatMask;
  _heatActive = false;
}

Heater::Heater(volatile uint8_t* pinPort, volatile uint8_t* pinDir, uint8_t pin) {
  _heatPort = pinPort;
  _heatMask = (1 << pin);
  _heatDir = pinDir;

  *_heatDir |= _heatMask;
  *_heatPort &= ~_heatMask;
}

void Heater::turnOn() {
  if (!_heatActive) {
    *_heatPort |= _heatMask;
    _heatActive = true;
  }
}

void Heater::turnOff() {
  if (_heatActive) {
    *_heatPort &= ~_heatMask;
    _heatActive = false;
  }
}

bool Heater::isActive() {
  return _heatActive;
}


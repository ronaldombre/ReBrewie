#ifndef Heater_h
#define Heater_h 

#include <Arduino.h>

class Heater {
  public:
    Heater(uint8_t pinHeater);
    Heater(uint8_t* pinPort, uint8_t* pinDir, uint8_t pin);
    void turnOn();
    void turnOff();
    bool isActive();

  private:
    bool _heatActive;

    volatile uint8_t *_heatPort;
    volatile uint8_t *_heatDir;
    uint8_t _heatMask;
};

#endif

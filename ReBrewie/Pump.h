#ifndef Pump_h
#define Pump_h 

#include <Arduino.h>
#include <SPI.h>

class Pump {
  public:
    Pump(uint8_t pin, bool channel);
    void Pump_Speed_Control(uint16_t current);
    void setPumpSpeed(uint16_t pumpSpeed);
    void writeDAC();
    uint8_t pumpTach();
    //float pumpDelta();
    uint8_t pumpDiag();
    bool pumpIsDry();
    void pumpFlowReset();
    uint16_t pumpFlow();
    bool isRunning();

    volatile uint8_t* pumpTicks;

  private:
    uint8_t _pumpPin;
    bool _pumpChannel;
    uint8_t _pumpTach;
    uint8_t _pumpSpeed;
    uint8_t _pumpDiag;
    uint16_t _pumpCurrent;
    uint16_t _pumpFlow;
    uint8_t _dryRun;
    bool _pumpDry;
    uint16_t _pumpCount;
    uint8_t _pumpTries;
    //float _pumpDelta;
    bool _pumpEnable;
    bool _running;

    uint32_t _pumpTime;
};

#endif

#ifndef Pump_h
#define Pump_h 

#include <Arduino.h>
#include <SPI.h>

class Pump {
  public:
    Pump(uint8_t pin, bool channel);
    void Pump_Speed_Control(float current);
    void setPumpSpeed(uint8_t pumpSpeed);
    void writeDAC();
    void setPumpOut();
    float pumpTach();
    uint8_t pumpDiag();
    bool pumpIsDry();
    bool pumpIsClogged();
    void pumpFlowReset();
    uint16_t pumpFlow();
    float flowRate();
    float flowTotal();
    void setFlowScale(float);
    bool isRunning();

    volatile uint8_t* pumpTicks;

  private:
    void _setPumpSpeed();
    void _stopPump();
    
    uint8_t _pumpPin;
    bool _pumpChannel;
    float _pumpTach;
    uint8_t _pumpSpeed;
    uint8_t _pumpSpeedRestart;
    uint8_t _pumpDiag;
    float _pumpCurrent;
    uint16_t _pumpFlow;
    uint8_t _dryRun;
    uint8_t _cloggedCount;
    bool _pumpDry;
    bool _pumpIsClogged;
    uint16_t _pumpCount;
    bool _pumpEnable;
    bool _running;
    bool _pumpOut;
    uint8_t _pumpState;

    float _expectedRPM;
    float _expectedCurrent;
    float _flowRate;
    float _flowTotal;
    float _flowScale = 1.0;

    uint32_t _pumpTime;
};

#endif

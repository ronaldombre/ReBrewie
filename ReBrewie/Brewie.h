#include <Arduino.h>
#include "Heater.h"

class Brewie {
  public:
    Brewie();
    void setTemperatures(float, float);
    void setTemperatureSensors(float*, float*);
    void setPowerSensor(float*);
    void Temperature_Control();
    void Control_Calculation();
    bool MashHeaterOn();
    bool BoilHeaterOn();
    bool MashTempReached();
    bool BoilTempReached();
    void SetMashMass(float);
    void SetBoilMass(float);
    void Reset();

  private:
    Heater* BoilHeater;
    Heater* MashHeater;

    float* _mashTemp;
    float* _boilTemp;
    float _mashTempDelta;
    float _boilTempDelta;
    float _mashSetTemp;
    float _boilSetTemp;
    float _mashStartTemp;
    float _boilStartTemp;
    bool _mashTempReached;
    bool _boilTempReached;
    
    bool _mashHeatSet;
    bool _boilHeatSet;
    bool _mashHeaterEnable;
    bool _boilHeaterEnable;
    float _mashHeaterEnergy;
    float _boilHeaterEnergy;
    float _lastEnergy;

    int32_t _mashHeaterControl;
    int32_t _boilHeaterControl;
    int32_t _heaterControlTime;
    
    float _mashTempPrevious;
    float _boilTempPrevious;

    uint32_t _heaterTime;
    uint32_t _mashTimer;
    uint32_t _boilTimer;

    float* _powerMeasure;
    float _mashMass;
    float _boilMass;
    float _mashRequiredEnergy;
    float _boilRequiredEnergy;
};

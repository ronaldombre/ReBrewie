#include <Arduino.h>
#include "Heater.h"

class Brewie {
  public:
    Brewie();
    void setTemperatures(float, float);
    void setTemperatureSensors(float*, float*);
    void setPowerSensor(float*);
    void Temperature_Average();
    void Temperature_Control();
    void Control_Calculation();
    bool MashHeaterOn();
    bool BoilHeaterOn();
    bool MashTempReached();
    bool BoilTempReached();
    void SetCooling();
    void SetHeating();
    void Reset();
    void Start();

    void SetMashMaxDuty(float);
    void SetBoilMaxDuty(float);

    bool ReadMashError();
    bool ReadBoilError();
    bool ReadCoolingError();

  private:
    void PrintDiagnostics();
  
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
    float _mashIntegralError;
    float _boilIntegralError;
    
    float _mashTempPrevious;
    float _boilTempPrevious;

    uint32_t _heaterTime;
    uint32_t _mashTimer;
    uint32_t _boilTimer;

    float* _powerMeasure;

    uint32_t _boilDeltaTimer;
    float _boilDeltaAverage;

    float _mashTempSum;
    float _boilTempSum;
    float _deltaMashTempSum;
    float _deltaBoilTempSum;
    float _mashTempMax;
    float _boilTempMax;
    float _mashTempMin;
    float _boilTempMin;
    float _mashTempAverage;
    float _boilTempAverage;
    float _mashTempAverageLast;
    float _boilTempAverageLast;
    uint8_t _tempSamples;
    uint8_t _boilingCount;
    bool _boilDetect;

    bool _setBoiling;
    bool _boilCooling;
    bool _coolingError;

    float _maxMashDuty = 1.0;
    float _maxBoilDuty = 1.0;

    bool _mashError;
    bool _boilError;
};

#include "Brewie.h"
#include "B20Plus.h"

Brewie::Brewie() {
  MashHeater = new Heater(MASH_HEATER);
  BoilHeater = new Heater(&BOIL_PORT, &BOIL_DIR, BOIL_PIN);
  _heaterControlTime = 20000;
  _boilingPoint = 100.0;
  _mashHeaterEnergy = 0.0;
  _boilHeaterEnergy = 0.0;
  _lastEnergy = 0.0;
}

void Brewie::setTemperatures(float mashSetTemp, float boilSetTemp) {
  _mashError = false;
  _boilError = false;
  if (mashSetTemp >= 0.0) {
    _mashSetTemp = mashSetTemp;
  }
  if (boilSetTemp >= 0.0) {
    _boilSetTemp = boilSetTemp;
  }
  _mashStartTemp = *_mashTemp;
  _boilStartTemp = *_boilTemp;
  _heaterControlTime = 20000;
  _mashHeaterControl = 0;
  _boilHeaterControl = 0;
  _boilingCount = 0;

  // Handle fresh boot a little more graciously
  if (_mashTempPrevious < 1.0) {
    _mashTempPrevious = *_mashTemp;
    _mashTempAverageLast = *_mashTemp;
  }
  if (_boilTempPrevious < 1.0) {
    _boilTempPrevious = *_boilTemp;
    _boilTempAverageLast = *_boilTemp;
  }

  _heaterTime = 0;
  
  if (_mashSetTemp > 10.0) {
    _mashHeatSet = true;
    _mashIntegralError  = 0;
    _mashTempReached = false;
  } else {
    _mashHeatSet = false;
    MashHeater->turnOff();
  }
  if (_boilSetTemp > 10.0) {
    _boilHeatSet = true;
    _boilIntegralError  = 0;
    _boilingCount = 0;
    _boilTempReached = false;
    if (_boilSetTemp >= 93.0) {
      _heaterControlTime = 60000;
    }
  } else {
    _boilHeatSet = false;
    BoilHeater->turnOff();
  }

  if (_boilCooling) {
    _heaterControlTime = 5000;
  }
}

void Brewie::setTemperatureSensors(float* mashTemp, float* boilTemp) {
  _mashTemp = mashTemp;
  _boilTemp = boilTemp;
}

void Brewie::setPowerSensor(float* power) {
  _powerMeasure = power;
}

void Brewie::Reset() {
  _mashTempDelta = 0.0;
  _boilTempDelta = 0.0;
  _boilCooling = false;
  _mashTempSum = 0;
  _boilTempSum = 0;
  _tempSamples = 0;
  _mashTempAverageLast = 0;
  _boilTempAverageLast = 0;
  _boilingCount = 0;
  _boilDetect = false;
  _coolingError = false;
  setTemperatures(0,0);
}

void Brewie::Start() {
  _boilCooling = false;
  _boilDetect = false;
  _mashHeaterEnergy = 0.0;
  _boilHeaterEnergy = 0.0;
  _lastEnergy = 0.0;
}

void Brewie::Temperature_Control() {
  // Process temperature control on a minute-by-minute basis
  if (millis() - _heaterTime > _heaterControlTime || _heaterTime == 0) {
    _heaterTime = millis();
    _mashTimer = _heaterTime;
    _boilTimer = _heaterTime; 

    // Deal with averages and such
    _mashTempAverage = *_mashTemp;
    _boilTempAverage = *_boilTemp;
    if (_tempSamples > 0) {
      _mashTempAverage = _mashTempSum/_tempSamples;
      _boilTempAverage = _boilTempSum/_tempSamples;
      _mashTempSum = 0;
      _boilTempSum = 0;
    } else {
      // Shouldn't happen, but just in case
      _mashTempAverage = *_mashTemp;
      _boilTempAverage = *_boilTemp;
    }
    _mashTempDelta = (_mashTempAverage - _mashTempAverageLast)*60000.0/(float)_heaterControlTime;
    _boilTempDelta = (_boilTempAverage - _boilTempAverageLast)*60000.0/(float)_heaterControlTime;
    _mashTempAverageLast = _mashTempAverage;
    _boilTempAverageLast = _boilTempAverage;
    _mashTempSpread = _mashTempMax - _mashTempMin;
    _boilTempSpread = _boilTempMax - _boilTempMin;
    _mashTempMax = 0;
    _mashTempMin = 110;
    _boilTempMax = 0;
    _boilTempMin = 110;
    _tempSamples = 0;
    _mashTempPrevious = *_mashTemp;
    _boilTempPrevious = *_boilTemp;

    // If heaters didn't shut off via control timing, add power here
    float deltaEnergy = *_powerMeasure - _lastEnergy;
    if (_mashHeaterEnable) {
      _mashHeaterEnergy += deltaEnergy;

      if (*_mashTemp < (_mashSetTemp - 0.5)) {
        if (deltaEnergy < (float)(0.35*_mashHeaterControl/_heaterControlTime)) {
          _mashError = true;
        } else {
          _mashError = false;
        }
      }
    } else if (_boilHeaterEnable) {
      _boilHeaterEnergy += deltaEnergy;

      if (*_boilTemp < (_boilSetTemp - 0.5)) {
        if (deltaEnergy < (float)(0.35*_boilHeaterControl/_heaterControlTime)) {
          _boilError = true;
        } else {
          _boilError = false;
        }
      }
    }
    _lastEnergy = *_powerMeasure;

    // Determine temperature reached
    if (_boilCooling) {
      Cooling_Calculation();
      if (_boilTempDelta > -0.25 && *_boilTemp < (_boilSetTemp+5)) {
        //_boilTempReached = true;
        _boilingCount += 3;
      } else if (_boilTempDelta > -0.25 && *_boilTemp < 35.0) {
        //_boilTempReached = true;
        _boilingCount += 2;
      } else if (*_boilTemp <= _boilSetTemp) {
        _boilingCount += 30;
      } else {
        if (_boilingCount > 0) {
          _boilingCount--;
        }
        _boilTempReached = false;
        if (_boilTempDelta > -0.25) {
          _boilingCount += 3;
        }
      }
      if (_boilingCount >= 30) {
        _boilTempReached = true;
        //_coolingError = true;
      }
    } else {
      Control_Calculation();
      if (_mashSetTemp-*_mashTemp <= 0.25 && _mashTempDelta <= 0.25 && _mashHeatSet) {
        _mashTempReached = true;
      } else if (*_mashTemp > _mashSetTemp) {
        _mashTempReached = true;
      } else {
        _mashTempReached = false;
      }
      // Three-Stage Boil Detection:
      //  - Close to 100C, just detect a single delta of ~0C
      //  - Not too close to 100C, detect consecutive delta's of ~0C
      //  - Very far from 100C (High altitude?), detect multiple consecutive ~0C deltas
      if (_boilSetTemp >= 100.0) {
        if (_boilTempDelta < -0.25) {
          // Partially reset if the temperature isn't going up
          _boilingCount >>= 1;
          // Restore full power
          SetBoilMaxDuty(1.0);
        } else if (_boilingPoint-*_boilTemp <= 0.5 && _boilTempDelta <= 0.125) {
          _boilingCount += 6;
        } else if (_boilingPoint-*_boilTemp <= 2.5 && _boilTempDelta <= 0.125) {
          _boilingCount += 4;
        } else if (_boilingPoint-*_boilTemp <= 13.0 && _boilTempDelta <= 0.0625) {
          _boilingCount += 2;
        } else {
          if (_boilingCount > 0) {
            _boilingCount--;
          }
          _boilTempReached = false;
        }
        
        if (_boilingCount >= 12) {
          if (_boilTempSpread < 0.25) {
            _boilTempReached = true;
          }
        }
        if (!_boilDetect) {
          if (_maxBoilDuty < 1.0) {
            // Restore heat if delta is dropping
            if (_boilTempDelta < 0.125) {
              SetBoilMaxDuty(1.0);
            }
          }
          if (_boilingCount >= 4 && _boilingPoint != 100.0) {
            // Potentially approaching boiling, scale back heat
            SetBoilMaxDuty(0.667);
          }
        }
        if (_boilTempReached && !_boilDetect) {
          Serial.println("Boiling Point Detected: ");
          Serial.println(_boilTempAverage);
          _boilDetect = true;
          SetBoilMaxDuty(1.0);
        }
      } else {
        if (_boilSetTemp-*_boilTemp <= 0.25 && _boilTempDelta <= 0.25 && _boilHeatSet) {
          _boilTempReached = true;
        } else if (*_boilTemp > _boilSetTemp) {
          if (_boilSetTemp > 10.0) {
            _boilTempReached = true;
          }
        } else {
          _boilTempReached = false;
        }
      }
    }

    // Diagnostic Information
    if (_mashHeatSet || _boilHeatSet) {
      PrintDiagnostics();
    }
  } 

  // Disable heaters if past control time
  uint32_t currentTime = millis();
  if (_mashHeaterEnable && (currentTime - _mashTimer > _mashHeaterControl)) {
    _mashHeaterEnable = false;
    _mashHeaterEnergy += *_powerMeasure - _lastEnergy;
    _lastEnergy = *_powerMeasure;
    _boilTimer = currentTime;                                // Add back what the mash heater took from the boil heater. Boil heater gets remaining time
  } else if (_boilHeaterEnable && (currentTime - _boilTimer > _boilHeaterControl)) {
    _boilHeaterEnable = false;
    _boilHeaterEnergy += *_powerMeasure - _lastEnergy;
    _lastEnergy = *_powerMeasure;
  } else if (_boilCoolingEnable && (currentTime - _boilTimer > _boilCoolingControl)) {
    _boilCoolingEnable = false;
  }
  
  if (_boilCooling) {
    if (_boilCoolingEnable) {
      digitalWrite(INLET_2, HIGH);
    } else {
      digitalWrite(INLET_2, LOW);
    }
  } else {
    if (digitalRead(INLET_2) == HIGH) {
      digitalWrite(INLET_2, LOW);
    }
  }

  // Turn on heaters if permitted
  if (_mashHeatSet) {
    if (_mashHeaterEnable) {
        MashHeater->turnOn();
    } else {
      MashHeater->turnOff();
    }
  } else {
    MashHeater->turnOff();
  }
  if (_boilHeatSet && !_boilCooling) {
    if (_boilHeaterEnable && !_mashHeaterEnable) {
        BoilHeater->turnOn();
    } else {
      BoilHeater->turnOff();
    }
  } else {
    BoilHeater->turnOff();
  }
}

void Brewie::Control_Calculation() {
  // Enable or disable heaters based on current temperature
  if (_mashHeatSet) {
    if (*_mashTemp >= (_mashSetTemp+0.625)) {
      _mashHeaterEnable = false;
    } else {
      _mashHeaterEnable = true;
    }
  } else {
    _mashHeaterEnable = false;
  }
  if (_boilHeatSet) {
    if (*_boilTemp >= (_boilSetTemp+0.625)) {
      _boilHeaterEnable = false;
    } else {
      _boilHeaterEnable = true;
    }
    if (_boilSetTemp >= 100.0) {
      _boilHeaterEnable = true;
    }
  } else {
    _boilHeaterEnable = false;
  }
  
  // MASH
  float mashError = _mashSetTemp - *_mashTemp;
  float mashDerivative = (_mashTempDelta*1.0*(float)_heaterControlTime);
  float mashIntegral = 0.0;
  if (mashError < 1.00 && mashError > -0.25) {
    _mashIntegralError += mashError;
    if (_mashIntegralError > 5.0) {
      _mashIntegralError = 5.0;  
    } else if (_mashIntegralError < -2.0) {
      _mashIntegralError = -2.0;
    }
    mashIntegral = _mashIntegralError * 0.1 * (float)_heaterControlTime;
  } else {
    _mashIntegralError  = 0;
  }
  if (_mashHeaterEnable) {
    _mashHeaterControl = (int32_t)((mashError * 1.2) * (float)_heaterControlTime);
    _mashHeaterControl -= (int32_t)mashDerivative;
    _mashHeaterControl += (int32_t)mashIntegral;
    if (_mashHeaterControl > (float)_heaterControlTime*_maxMashDuty) {
      _mashHeaterControl = (float)_heaterControlTime * _maxMashDuty;
    } else if (_mashHeaterControl < _heaterControlTime/10) {
      _mashHeaterControl = 0;
    }
  } else {
    _mashHeaterControl = 0;
  }

  // BOIL
  float boilError = _boilSetTemp - *_boilTemp;
  float boilDerivative = (_boilTempDelta*1.0*(float)_heaterControlTime);
  float boilIntegral = 0.0;
  if (boilError < 1.00 && boilError > -0.25) {
    _boilIntegralError += boilError;
    if (_boilIntegralError > 5.0) {
      _boilIntegralError = 5.0;  
    } else if (_boilIntegralError < -2.0) {
      _boilIntegralError = -2.0;
    }
    boilIntegral = _boilIntegralError * 0.1 * (float)_heaterControlTime;
  } else {
    _boilIntegralError  = 0;
  }
  if (_boilHeaterEnable) {
    _boilHeaterControl = (int32_t)((boilError * 1.2) * (float)_heaterControlTime);
    _boilHeaterControl -= (int32_t)boilDerivative;
    _boilHeaterControl += (int32_t)boilIntegral;

    if (_boilHeaterControl > ((float)_heaterControlTime*_maxBoilDuty)) {
      _boilHeaterControl = (float)_heaterControlTime * _maxBoilDuty;
    } else if (_boilHeaterControl < _heaterControlTime/10) {
      _boilHeaterControl = 0;
    }

    if (_boilSetTemp >= 100.0) {
      _boilHeaterControl = (float)_heaterControlTime * _maxBoilDuty;
    }
    if (_boilHeaterControl > (_heaterControlTime-_mashHeaterControl)) {
      _boilHeaterControl = _heaterControlTime - _mashHeaterControl;
    }
  } else {
    _boilHeaterControl = 0;
  }
  OCR3A = _boilHeaterControl*3;
}

void Brewie::Cooling_Calculation() {
  // Enable or disable heaters based on current temperature
  if (_boilCooling) {
    if (*_boilTemp <= _boilSetTemp) {
      _boilCoolingEnable = false;
    } else {
      _boilCoolingEnable = true;
    }
  } else {
    _boilCoolingEnable = false;
  }

  float boilError = *_boilTemp -_boilSetTemp;
  float boilDerivative = (_boilTempDelta*(float)_heaterControlTime/20.0);
  float boilIntegral = 0.0;
  if (boilError <= 1.20 && boilError > 0.00) {
    _boilIntegralError += boilError;
    if (_boilIntegralError > 10.0) {
      _boilIntegralError = 10.0;  
    } else if (_boilIntegralError < -1.0) {
      _boilIntegralError = -1.0;
    }
    boilIntegral = _boilIntegralError * 0.15 * (float)_heaterControlTime;
  } else {
    _boilIntegralError  = 0;
  }
  if (_boilCooling && _boilCoolingEnable) {
    _boilCoolingControl = (int32_t)((boilError * (float)_heaterControlTime)*0.5);
    _boilCoolingControl += (int32_t)boilDerivative;
    _boilCoolingControl += (int32_t)boilIntegral;

    if (_boilCoolingControl > ((float)_heaterControlTime*_maxBoilDuty)) {
      _boilCoolingControl = (float)_heaterControlTime * _maxBoilDuty;
    } else if (_boilCoolingControl < _heaterControlTime/10) {
      _boilCoolingControl = 0;
    }
  } else {
    _boilCoolingControl = 0;
  }
}

// Runs every second in main control loop
void Brewie::Temperature_Average() {
  // Add to sum
  _mashTempSum += *_mashTemp;
  _boilTempSum += *_boilTemp;
  _tempSamples++;

  // Find max and min temperatures
  if (*_mashTemp > _mashTempMax) {
    _mashTempMax = *_mashTemp;
  } else if (*_mashTemp < _mashTempMin) {
    _mashTempMin = *_mashTemp;
  }

  if (*_boilTemp > _boilTempMax) {
    _boilTempMax = *_boilTemp;
  } else if (*_boilTemp < _boilTempMin) {
    _boilTempMin = *_boilTemp;
  }
}

bool Brewie::MashHeaterOn() {
  return MashHeater->isActive();
}

bool Brewie::BoilHeaterOn() {
  return BoilHeater->isActive();
}

bool Brewie::MashTempReached() {
  return _mashTempReached;
}

bool Brewie::BoilTempReached() {
  return _boilTempReached;
}

float Brewie::GetBoilingPoint() {
  return _boilingPoint;
}

void Brewie::SetBoilingPoint(float bPoint) {
  _boilingPoint = bPoint;
}

void Brewie::SetCooling() {
  _boilCooling = true;
  _coolingError = false;
  _heaterControlTime = 5000;
}

void Brewie::SetHeating() {
  _boilCooling = false;
  _heaterControlTime = 20000;
}

void Brewie::SetMashMaxDuty(float duty) {
  _maxMashDuty = duty;
}

void Brewie::SetBoilMaxDuty(float duty) {
  _maxBoilDuty = duty;
}

bool Brewie::ReadMashError() {
  bool error = _mashError;
  _mashError = false;
  return error;
}

bool Brewie::ReadBoilError() {
  bool error = _boilError;
  _boilError = false;
  return error;
}

bool Brewie::ReadCoolingError() {
  bool error = _coolingError;
  _coolingError = false;
  return error;
}

void Brewie::PrintDiagnostics() {
  Serial.print("!Mash Delta: ");
  Serial.print(_mashTempDelta);
  Serial.print(" Mash Ave: ");
  Serial.print(_mashTempAverage);
  Serial.print(" Mash Spread: ");
  Serial.print(_mashTempSpread);
  Serial.print(" Mash Energy Use: ");
  Serial.print(_mashHeaterEnergy);
  
  Serial.print(" Boil Delta: ");
  Serial.print(_boilTempDelta);
  Serial.print(" Boil Ave: ");
  Serial.print(_boilTempAverage);
  Serial.print(" Boil Spread: ");
  Serial.print(_mashTempSpread);
  Serial.print(" Boil Energy Use: ");
  Serial.println(_boilHeaterEnergy);

  uint32_t controlPercent = _mashHeaterControl*100/_heaterControlTime;
  Serial.print("!Temp Control - Mash Duty: ");
  Serial.print(controlPercent);
  Serial.print("% ");
  if (_mashTempReached && _mashHeatSet) {
    Serial.print("At Temp");
  } else if (_mashHeatSet) {
    Serial.print("Heating");
  } else {
    Serial.print("Idle");
  }
  if (_boilCooling) {
    controlPercent = _boilCoolingControl*100/_heaterControlTime;
  } else {
    controlPercent = _boilHeaterControl*100/_heaterControlTime;
  }
  Serial.print("       Boil Duty: ");
  Serial.print(controlPercent);
  Serial.print("% ");
  Serial.print("/");
  Serial.print(_maxBoilDuty*100.0, 0);
  Serial.print("% ");
  if (_boilTempReached && _boilHeatSet) {
    Serial.println("At Temp");
  } else if (_boilCooling) {
    Serial.println("Cooling");
  } else if (_boilHeatSet) {
    Serial.println("Heating");
  } else {
    Serial.println("Idle");
  }
}


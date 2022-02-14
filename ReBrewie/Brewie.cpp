#include "Brewie.h"
#include "B20Plus.h"

int32_t heaterMinimumCycleTime = 2000;

Brewie::Brewie() {
  MashHeater = new Heater(MASH_HEATER);
  BoilHeater = new Heater(&BOIL_PORT, &BOIL_DIR, BOIL_PIN);
  _heaterControlTime = 20000;
  _boilingPoint = 100.0;
  _mashHeaterEnergy = 0.0;
  _boilHeaterEnergy = 0.0;
  _lastEnergy = 0.0;
  _heaterTime = 0;
}

void Brewie::setTemperatures(float mashSetTemp, float boilSetTemp) {
  _boilTempReached = false;
  _mashTempReached = false;
  _mashError = false;
  _boilError = false;
  if (mashSetTemp <= 0.0 && boilSetTemp <= 0.00) {
    // Don't disturb heater control for turning it off
  } else {
    _heaterTime = 0;
  }
  if (mashSetTemp >= 0.0) {
    _mashSetTemp = mashSetTemp;
  }
  if (boilSetTemp >= 0.0) {
    _boilSetTemp = boilSetTemp;
  }
  _mashHeaterControl = 0;
  _boilHeaterControl = 0;
  _boilingCount = 0;
  
  if (_mashSetTemp > 10.0) {
    _mashHeatSet = true;
    //TIMSK3 |= _BV(OCIE3A);
  } else {
    _mashHeaterControl = 0;
    _mashHeatSet = false;
    //MashHeater->turnOff();
    //TIMSK3 &= ~_BV(OCIE3A);
  }

  if (_boilCooling) {
    _heaterControlTime = 5000;
  } else {
    if (_boilSetTemp >= 88.0) {
      _heaterControlTime = 40000;
    } else {
      _heaterControlTime = 20000;
    }
  }
    
  if (_boilSetTemp > 10.0) {
    _boilHeatSet = true;
    _boilingCount = 0;
    //TIMSK3 |= _BV(OCIE3B);
  } else {
    _boilHeaterControl = 0;
    _boilHeatSet = false;
    //BoilHeater->turnOff();
    //TIMSK3 &= ~_BV(OCIE3B);
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
  setTemperatures(0,0);
  _mashTempDelta = 0.0;
  _boilTempDelta = 0.0;
  _mashTempSum = 0;
  _boilTempSum = 0;
  _tempSamples = 0;
  _mashTempAverageLast = 0;
  _boilTempAverageLast = 0;
  _heaterTime = 0;
  Start();
}

void Brewie::Start() {
  _boilCooling = false;
  _boilDetect = false;
  _boilTempReached = false;
  _boilingCount = 0;
  _coolingError = false;
  _mashHeaterEnergy = 0.0;
  _boilHeaterEnergy = 0.0;
  _lastEnergy = 0.0;
  _maxMashDuty = 1.0;
  _maxBoilDuty = 1.0;
}

void Brewie::Temperature_Control() {
  // Process temperature control on a minute-by-minute basis
  if ((millis() - _heaterTime) > _heaterControlTime && (!_mashHeaterEnable && !_boilHeaterEnable) || _heaterTime == 0) {
    _heaterTime = millis();
    _mashTimer = _heaterTime;
    _boilTimer = _heaterTime; 

    // Deal with averages and such
    if (_tempSamples > 0) {
      _mashTempAverage = _mashTempSum/(float)_tempSamples;
      _boilTempAverage = _boilTempSum/(float)_tempSamples;
      _tempSamples = 0;
      _mashTempSum = 0;
      _boilTempSum = 0;
    } else {
      // Shouldn't happen, but just in case
      _mashTempAverage = *_mashTemp;
      _boilTempAverage = *_boilTemp;
    }
    _mashTempDelta = (_mashTempAverage - _mashTempAverageLast)*60000.0/(float)_heaterControlTime;
    _boilTempDelta = (_boilTempAverage - _boilTempAverageLast)*60000.0/(float)_heaterControlTime;

    // If heaters didn't shut off via control timing, add power here
    float deltaEnergy = *_powerMeasure - _lastEnergy;
    if (_mashHeaterEnable) {
      _mashHeaterEnergy += deltaEnergy;
    } else if (_boilHeaterEnable) {
      _boilHeaterEnergy += deltaEnergy;
    }
    _lastEnergy = *_powerMeasure;

    // Determine temperature reached
    if (_boilCooling) {
      Cooling_Calculation();
      if (_boilTempDelta > -0.25 && *_boilTemp < (_boilSetTemp+5.0)) {
        _boilingCount += 3;
      } else if (_boilTempDelta > -0.25 && *_boilTemp < 35.0) {
        _boilingCount += 1;
      } else if (*_boilTemp <= _boilSetTemp+0.125) {
        _boilingCount += 30;
      } else {
        if (_boilTempDelta > -0.25) {
          _boilingCount += 2;
        } else {
          if (_boilingCount > 0) {
            _boilingCount--;
          }
        }
      }
      if (_boilingCount >= 30) {
        _boilTempReached = true;
        if (*_boilTemp > _boilSetTemp+5.0) {
          _coolingError = true;
        }
      }
    } else {
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
          //SetBoilMaxDuty(1.0);
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
        
        if (_boilingCount >= 16) {
          _boilTempReached = true;
        }

        if (_boilTempReached && !_boilDetect) {
          _boilingPoint = _boilTempAverage;
          _boilDetect = true;
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

      Control_Calculation();
    }

    _mashTempAverageLast = _mashTempAverage;
    _boilTempAverageLast = _boilTempAverage;

    // Diagnostic Information
    if (_mashHeatSet || _boilHeatSet) {
      PrintDiagnostics();
    }
  } 

  // Disable heaters if past control time
  uint32_t currentTime = millis();
  float energyCheck = 0;
  if (_mashHeaterEnable && (currentTime - _mashTimer > _mashHeaterControl)) {
    _mashHeaterEnable = false;
    energyCheck = *_powerMeasure - _lastEnergy;
    _mashHeaterEnergy += energyCheck;
    _lastEnergy = *_powerMeasure;
    _boilTimer = currentTime;                                // Add back what the mash heater took from the boil heater. Boil heater gets remaining time

    // Perform mash heater power on check
    if (energyCheck < (float)(0.1*_mashHeaterControl/_heaterControlTime)) {
      _mashError = true;
      Serial.println("!Mash Heater Error Detected");
    } else {
      _mashError = false;
    }
  } else if (_boilHeaterEnable && (currentTime - _boilTimer > _boilHeaterControl)) {
    _boilHeaterEnable = false;
    energyCheck = *_powerMeasure - _lastEnergy;
    _boilHeaterEnergy += energyCheck;
    _lastEnergy = *_powerMeasure;

    // Perform boil heater power on check
    if (energyCheck < (float)(0.1*_boilHeaterControl/_heaterControlTime)) {
      _boilError = true;
    } else {
      _boilError = false;
    }
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
  //if (TIMSK3 & _BV(OCIE3A) == 0) {
    if (_mashHeatSet) {
      if (_mashHeaterEnable) {
          MashHeater->turnOn();
      } else {
        MashHeater->turnOff();
      }
    } else {
      MashHeater->turnOff();
    }
  //}
  //if (TIMSK3 & _BV(OCIE3B) == 0) {
    if (_boilHeatSet && !_boilCooling) {
      if (_boilHeaterEnable && !_mashHeaterEnable) {
          BoilHeater->turnOn();
      } else {
        BoilHeater->turnOff();
      }
    } else {
      BoilHeater->turnOff();
    }
  //}
}

void Brewie::Control_Calculation() {
  // Enable or disable heaters based on current temperature
  if (_mashHeatSet) {
    if (*_mashTemp >= (_mashSetTemp+0.5)) {
      _mashHeaterEnable = false;
    } else {
      _mashHeaterEnable = true;
    }
  } else {
    _mashHeaterEnable = false;
  }
  if (_boilHeatSet) {
    if (_boilSetTemp >= 100.0) {
      _boilHeaterEnable = true;
    } else {
      if (*_boilTemp >= (_boilSetTemp+0.5)) {
        _boilHeaterEnable = false;
      } else {
        _boilHeaterEnable = true;
      }
    }
  } else {
    _boilHeaterEnable = false;
  }

  float heaterControlTime = (float)_heaterControlTime;
  float pGain = 0.75;
  
  // MASH
  float mashError = _mashSetTemp - *_mashTemp;
  //float mashTraj = (*_mashTemp - _mashTempAverageLast)*60000.0/heaterControlTime;
  float mashJerk = (*_mashTemp - _mashTempAverage)*60000.0/heaterControlTime;
  float mashDerivative = 0.4*heaterControlTime*((_mashTempDelta*0.4)+(mashJerk*0.6));
  float mashIntegral = 0.0;
  if (mashError < 1.00 && mashError > -0.25) {
    _mashIntegralError += mashError;
    if (_mashIntegralError > 5) {
      _mashIntegralError = 5;  
    } else if (_mashIntegralError < -2.5) {
      _mashIntegralError = -2.5;
    }
    mashIntegral = _mashIntegralError * 0.05 * heaterControlTime;
  } else {
    _mashIntegralError  = 0;
  }
  if (_mashHeaterEnable) {
    pGain = 0.2 + (_mashSetTemp/100.0);
    _mashHeaterControl = (int32_t)((mashError * pGain) * heaterControlTime);
    _mashHeaterControl -= (int32_t)mashDerivative;
    _mashHeaterControl += (int32_t)mashIntegral;

    /*Serial.print("!Control Parameters: ");
    Serial.print(mashError);
    Serial.print(", ");
    Serial.print(mashDerivative);
    Serial.print(", ");
    Serial.print(mashIntegral);
    Serial.print(" - ");
    Serial.print(mashTraj);
    Serial.print(", ");
    Serial.println(mashJerk);*/

    if (_mashHeaterControl > heaterControlTime*_maxMashDuty) {
      _mashHeaterControl = heaterControlTime * _maxMashDuty;
    } else if (_mashHeaterControl < heaterMinimumCycleTime) {
      _mashHeaterControl = 0;
    }
  } else {
    _mashHeaterControl = 0;
  }

  // BOIL
  float boilError = _boilSetTemp - *_boilTemp;
  //float boilTraj = (*_boilTemp - _boilTempAverageLast)*60000.0/heaterControlTime;
  float boilJerk = (*_boilTemp - _boilTempAverage)*60000.0/heaterControlTime;
  float boilDerivative = 0.2*heaterControlTime*((_boilTempDelta*0.4)+(boilJerk*0.6));
  float boilIntegral = 0.0;
  if (boilError < 1.00 && boilError > -0.25) {
    _boilIntegralError += boilError;
    if (_boilIntegralError > 5) {
      _boilIntegralError = 5;  
    } else if (_boilIntegralError < -2.5) {
      _boilIntegralError = -2.5;
    }
    boilIntegral = _boilIntegralError * 0.05 * heaterControlTime;
  } else {
    _boilIntegralError  = 0;
  }
  if (_boilHeaterEnable) {
    pGain = 0.2 + (_boilSetTemp/80.0);
    _boilHeaterControl = (int32_t)((boilError * pGain) * heaterControlTime);
    _boilHeaterControl -= (int32_t)boilDerivative;
    _boilHeaterControl += (int32_t)boilIntegral;

    /*Serial.print("!Control Parameters: ");
    Serial.print(boilError);
    Serial.print(", ");
    Serial.print(boilDerivative);
    Serial.print(", ");
    Serial.print(boilIntegral);
    Serial.print(" - ");
    Serial.print(boilTraj);
    Serial.print(", ");
    Serial.println(boilJerk);*/

    if (_boilHeaterControl > (int32_t)(heaterControlTime*_maxBoilDuty)) {
      _boilHeaterControl = (int32_t)(heaterControlTime * _maxBoilDuty);
    } else if (_boilHeaterControl < heaterMinimumCycleTime) {
      _boilHeaterControl = 0;
    } 

    if (_boilHeaterControl > (_heaterControlTime-_mashHeaterControl)) {
      _boilHeaterControl = _heaterControlTime - _mashHeaterControl;
    }
    if (_boilSetTemp >= 100.0) {
      _boilHeaterControl = (int32_t)(heaterControlTime * _maxBoilDuty);
    }
  } else {
    _boilHeaterControl = 0;
  }
  //OCR3B = 65535 - (uint16_t)((float)_boilHeaterControl*3.27675);
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

  float heaterControlTime = (float)_heaterControlTime;

  float boilError = *_boilTemp - _boilSetTemp;
  float boilTraj = *_boilTemp - _boilTempAverageLast;
  float boilJerk = *_boilTemp - _boilTempAverage;
  float boilDerivative = (_boilTempDelta*0.3*heaterControlTime)+(boilTraj*0.1*heaterControlTime)+(boilJerk*0.5*heaterControlTime);
  float boilIntegral = 0.0;
  if (boilError < 1.00 && boilError > -0.25) {
    if (_boilTempDelta > 0.125 || _boilTempDelta < -0.125) {
      _boilIntegralError += boilError;
    }
    if (_boilIntegralError > 2.5) {
      _boilIntegralError = 2.5;  
    } else if (_boilIntegralError < -1.0) {
      _boilIntegralError = -1.0;
    }
    boilIntegral = _boilIntegralError * 0.05 * heaterControlTime;
  } else {
    _boilIntegralError  = 0;
  }
  if (_boilCooling && _boilCoolingEnable) {
    _boilCoolingControl = (int32_t)(boilError * 1.0 * heaterControlTime);
    _boilCoolingControl += (int32_t)boilDerivative;
    _boilCoolingControl += (int32_t)boilIntegral;

    if (_boilCoolingControl > (int32_t)(heaterControlTime)) {
      _boilCoolingControl = (int32_t)(heaterControlTime);
    } else if (_boilCoolingControl < (int32_t)(heaterControlTime/5.0)) {
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
  /*if (*_mashTemp > _mashTempMax) {
    _mashTempMax = *_mashTemp;
  } else if (*_mashTemp < _mashTempMin) {
    _mashTempMin = *_mashTemp;
  }

  if (*_boilTemp > _boilTempMax) {
    _boilTempMax = *_boilTemp;
  } else if (*_boilTemp < _boilTempMin) {
    _boilTempMin = *_boilTemp;
  }*/
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
  Serial.print(" Mash Energy Use: ");
  Serial.print(_mashHeaterEnergy);
  
  Serial.print(" Boil Delta: ");
  Serial.print(_boilTempDelta);
  Serial.print(" Boil Ave: ");
  Serial.print(_boilTempAverage);
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


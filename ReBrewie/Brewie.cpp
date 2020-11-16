#include "Brewie.h"
#include "B20Plus.h"

#define HEATER_CONTROL_TIME   30000

Brewie::Brewie() {
  MashHeater = new Heater(MASH_HEATER);
  BoilHeater = new Heater(&BOIL_PORT, &BOIL_DIR, BOIL_PIN);
}

void Brewie::setTemperatures(float mashSetTemp, float boilSetTemp) {
  _mashSetTemp = mashSetTemp;
  _boilSetTemp = boilSetTemp;
  _mashStartTemp = *_mashTemp;
  _boilStartTemp = *_boilTemp;

  // Handle fresh boot a little more graciously
  if (_mashTempPrevious == 0) {
    _mashTempPrevious = *_mashTemp;
  }
  if (_boilTempPrevious == 0) {
    _boilTempPrevious = *_boilTemp;
  }

  _heaterTime = 0;
  _heaterControlTime = 30000;
  
  if (_mashSetTemp > 0.0) {
    _mashHeatSet = true;
    _mashTimer = 0;
  } else {
    _mashHeatSet = false;
    MashHeater->turnOff();
  }

  if (_boilSetTemp > 0.0) {
    _boilHeatSet = true;
    _boilTimer = 0;
  } else {
    _boilHeatSet = false;
    BoilHeater->turnOff();
  }

  // Estimated energy
  if (_mashHeatSet) {
    _mashTempReached = false;
    if (_mashMass > 0) {
      _mashRequiredEnergy = _mashMass*4200*(_mashSetTemp-_mashStartTemp);
      Serial.print("Required Mash Heater Energy: ");
      Serial.println(_mashRequiredEnergy);
    }
  }
  if (_boilHeatSet) {
    _boilTempReached = false;
    if (_boilMass > 0) {
      _boilRequiredEnergy = _boilMass*4200*(_boilSetTemp-_boilStartTemp);
      Serial.print("Required Boil Heater Energy: ");
      Serial.println(_boilRequiredEnergy);
    }
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
  _mashHeaterEnergy = 0.0;
  _boilHeaterEnergy = 0.0;
  _mashTempDelta = 0.0;
  _boilTempDelta = 0.0;
  setTemperatures(0,0);
}

void Brewie::Temperature_Control() {
  // Process temperature control on a minute-by-minute basis
  if (millis() - _heaterTime > _heaterControlTime) {
    _heaterTime = millis();
    _mashTimer = _heaterTime;
    _boilTimer = _heaterTime; 

    // Calculate control values
    Control_Calculation();

    // If heaters didn't shut off via control timing, add power here
    float _deltaEnergy = *_powerMeasure - _lastEnergy;
    if (_mashHeaterEnable) {
      _mashHeaterEnergy += _deltaEnergy;
      Serial.print("Estimated Mass: ");
      float massEst = _deltaEnergy*3600.0/_mashTempDelta/4200.0;
      Serial.println(massEst);
    } else if (_boilHeaterEnable) {
      _boilHeaterEnergy += _deltaEnergy;
      Serial.print("Estimated Mass: ");
      float massEst = _deltaEnergy*3600.0/_boilTempDelta/4200.0;
      Serial.println(massEst);
    }
    _lastEnergy = *_powerMeasure;

    // Determine temperature reached
    if (_mashSetTemp-*_mashTemp < 0.125 && _mashTempDelta < 0.35 && !_mashTempReached) {
      _mashTempReached = true;
      Serial.print("Total Energy Required for Mash: ");
      Serial.println(_mashHeaterEnergy);
      Serial.print("Estimated Energy Required for Mash: ");
      Serial.println(_mashRequiredEnergy);
    }
    if (_boilSetTemp-*_boilTemp < 0.125 && _boilTempDelta < 0.35 && !_boilTempReached) {
      _boilTempReached = true;
      Serial.print("Total Energy Required for Boil: ");
      Serial.println(_boilHeaterEnergy);
      Serial.print("Estimated Energy Required for Boil: ");
      Serial.println(_boilRequiredEnergy);
    }

    // Enable or disable heaters based on current temperature
    if (_mashHeatSet) {
      if (*_mashTemp >= _mashSetTemp) {
        _mashHeaterEnable = false;
        _mashHeaterControl = 0;
      } else {
        _mashHeaterEnable = true;
      }
      //if (_mashHeaterControl == _heaterControlTime) {
        /*_heaterControlTime += (0.5-_mashTempDelta)*_heaterControlTime;
        if (_heaterControlTime > 60000) {
          _heaterControlTime = 60000;
        } else if (_heaterControlTime < 10000) {
          _heaterControlTime = 10000;
        }*/
      //}
    } else {
      _mashHeaterEnable = false;
      _mashHeaterControl = 0;
    }
    if (_boilHeatSet) {
      if (*_boilTemp >= _boilSetTemp && _boilSetTemp < 100.0) {
        _boilHeaterEnable = false;
        _boilHeaterControl = 0;
      } else {
        _boilHeaterEnable = true;
      }
      //if (!_mashHeatSet) {
        //if (_boilHeaterControl == _heaterControlTime) {
          /*_heaterControlTime += (0.5-_boilTempDelta)*_heaterControlTime;
          if (_heaterControlTime > 60000) {
            _heaterControlTime = 60000;
          } else if (_heaterControlTime < 10000) {
            _heaterControlTime = 10000;
          }*/
        //}
      //}
    } else {
      _boilHeaterEnable = false;
      _boilHeaterControl = 0;
    }

    Serial.print("Mash Temp Delta: ");
    Serial.print(_mashTempDelta);
    _mashTempPrevious = *_mashTemp;
    Serial.print(" Mash Energy Use: ");
    Serial.print(_mashHeaterEnergy);
    
    Serial.print(" Boil Temp Delta: ");
    Serial.print(_boilTempDelta);
    _boilTempPrevious = *_boilTemp;
    Serial.print(" Boil Energy Use: ");
    Serial.println(_boilHeaterEnergy);

    uint32_t controlPercent = _mashHeaterControl*100.0/_heaterControlTime;
    Serial.print("Temp Control - Mash Set: ");
    Serial.print(controlPercent);
    Serial.print("%");
    if (_mashTempReached && _mashHeatSet) {
      Serial.print("At Temp");
    } else if (_mashHeatSet) {
      Serial.print("Heating");
    } else {
      Serial.print("Idle");
    }
    controlPercent = _boilHeaterControl*100.0/_heaterControlTime;
    Serial.print(" Boil Set: ");
    Serial.print(controlPercent);
    Serial.print("% ");
    if (_boilTempReached && _boilHeatSet) {
      Serial.print("At Temp");
    } else if (_boilHeatSet) {
      Serial.print("Heating");
    } else {
      Serial.print("Idle");
    }
    Serial.println("");
  }

  // Disable heaters if past control time
  uint32_t currentTime = millis();
  if (_mashHeaterEnable && (currentTime - _mashTimer > _mashHeaterControl)) {
    _mashHeaterEnable = false;
    //_boilTimer += _mashHeaterControl;                   // Add back what the mash heater took from the boil heater. Boil heater gets remaining time
    _mashHeaterEnergy += *_powerMeasure - _lastEnergy;
    _lastEnergy = *_powerMeasure;
    _boilTimer = millis();
  } else if (_boilHeaterEnable && (currentTime - _boilTimer > _boilHeaterControl)) {
    _boilHeaterEnable = false;
    _boilHeaterEnergy += *_powerMeasure - _lastEnergy;
    _lastEnergy = *_powerMeasure;
  }

  // Turn on heaters if permitted
  if (_mashHeatSet) {
    if (_mashHeaterEnable) {
        MashHeater->turnOn();
        digitalWrite(A1, HIGH);
    } else {
      MashHeater->turnOff();
      digitalWrite(A1, LOW);
    }
  } else {
    MashHeater->turnOff();
    digitalWrite(A1, LOW);
  }
  if (_boilHeatSet) {
    if (_boilHeaterEnable && !_mashHeaterEnable) {
        BoilHeater->turnOn();
        digitalWrite(A2, HIGH);
    } else {
      BoilHeater->turnOff();
      digitalWrite(A2, LOW);
    }
  } else {
    BoilHeater->turnOff();
    digitalWrite(A2, LOW);
  }
}

void Brewie::Control_Calculation() {
  _mashTempDelta = *_mashTemp - _mashTempPrevious;
  _boilTempDelta = *_boilTemp - _boilTempPrevious;
  
  // MASH
  if (_mashHeatSet) {
    _mashHeaterControl = (int32_t)((_mashSetTemp - *_mashTemp ) * _heaterControlTime/2);
    if (_mashHeaterControl > _heaterControlTime) {
      //_mashHeaterControl = _heaterControlTime;
    } /*else if (_mashHeaterControl < HEATER_CONTROL_TIME/6) {
      _mashHeaterControl = 0;
    }*/
    _mashHeaterControl -= (int32_t)((_mashTempDelta-0.25)*(_heaterControlTime/3/*-_mashHeaterControl*/));
    if (_mashHeaterControl > _heaterControlTime) {
      _mashHeaterControl = _heaterControlTime;
    } else if (_mashHeaterControl < _heaterControlTime/10) {
      _mashHeaterControl = 0;
    }
  } else {
    _mashHeaterControl = 0;
  }

  // BOIL
  if (_boilHeatSet) {
    _boilHeaterControl = (int32_t)((_boilSetTemp - *_boilTemp) * _heaterControlTime/2);
    /*if (_boilHeaterControl > _heaterControlTime) {
      _boilHeaterControl = _heaterControlTime;
    } else if (_boilHeaterControl < 0) {
      //_boilHeaterControl = 0;
    }*/
    _boilHeaterControl -= (int32_t)((_boilTempDelta-0.25)*_heaterControlTime/3);
    if (_boilHeaterControl > _heaterControlTime) {
      _boilHeaterControl = _heaterControlTime;
    } else if (_boilHeaterControl < _heaterControlTime/10) {
      _boilHeaterControl = 0;
    }
    if (_boilHeaterControl > (_heaterControlTime-_mashHeaterControl)) {
      _boilHeaterControl = _heaterControlTime - _mashHeaterControl;
    }
    if (_boilSetTemp >= 100.0) {
      _boilHeaterControl = _heaterControlTime;
    }
  } else {
    _boilHeaterControl = 0;
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

void Brewie::SetMashMass(float mass) {
  _mashMass = mass;
}

void Brewie::SetBoilMass(float mass) {
  _boilMass = mass;
}

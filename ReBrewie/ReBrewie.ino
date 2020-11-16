#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>

//#define B20

#include "Brewie.h"
#include "Pump.h"
#include "Heater.h"
#include "Valves.h"
#include "B20Plus.h"

const uint8_t mcuVersion = 7;

// Brewie Command variables
int8_t currStep = -1;
uint8_t brewieData[80];
uint8_t brewieCommand[24][2];             // Store received information position and length
uint8_t errorMessage[4][4];
uint8_t brewieMessage[4][4];
uint8_t echoMessage[80];
uint8_t errorCount = 0;
uint8_t messageCount = 0;
uint8_t echoCount = 0;

// Flags and indicators
bool powerOn = false;
bool newStep = false;
bool newCommand = false;
bool brewiePause = false;
bool stepActive = false;
bool stepFinished = false;
bool spargeOverride = false;
bool brewieWaterFill = false;
bool waterFilled = false;
bool brewing = false;
bool fanOverride = false;
bool safetyShutdown = false;
bool errorOverride = true;
bool waterOverride = false;
bool fastWaterReadings = false;

// MCU step variables
int16_t brewieStepBuffer[22];
int16_t nextStep[22];
int16_t brewieStep[22];
uint16_t totalTime = 0;
uint16_t stepTime = 0;
int16_t leftTime = 0;

// Calibration to be loaded by P80 command
float toLiter = 267.000;
float toLiterNull = 2358.0;
float mashDelta = 0.5;
float boilDelta = 0.5;

// Sensors
OneWire MashTempPin(MASH_TEMP);
OneWire BoilTempPin(BOIL_TEMP);
DallasTemperature MashTemp(&MashTempPin);
DallasTemperature BoilTemp(&BoilTempPin);
DeviceAddress mashAddress;
DeviceAddress boilAddress;

uint8_t massAddress = 25;

int16_t massSensor = 1000;
int16_t massFast = 0;
float acCurrent = 0;
float acMeasure  = 0;
float boardTemp = 0;
float inlet1A = 0;
float servosA = 0;
float boilPumpA = 460;
float mashPumpA = 480;
float pumpTemp = 0;

// Heater Control Variables
float mashTemp = 25.00;
float boilTemp = 25.00;

uint32_t debugTime = millis();
uint32_t waterTime = millis();
uint32_t leftOverTime = 0;

// Interrupt Variables
volatile uint8_t mashTicks = 0;
volatile uint8_t boilTicks = 0;
volatile bool powerButton = false;
volatile bool drainButton = false;

Pump mashPump(MASH_PUMP, 1);
Pump boilPump(BOIL_PUMP, 0);
Brewie Brewie;

void setup() {
  Initialize_2560();
}

void loop() {
  if (powerOn) {
    Process_Sensors();
    Brewie.Temperature_Control();
    if (digitalRead(POWER_BUTTON) == HIGH) {
      powerButton = true;
    }
    if (Brewie_Read() == false) {
      // If no command over UART, Process
      Fill_Boil_Tank();
      Control_Loop();
      Step_Complete();
      Run_Sparge();
      Fast_Water_Readings();
    }
  } else {
    if (digitalRead(POWER_BUTTON) == HIGH) {
      Serial.println("Button Press");
      digitalWrite(POWER_LIGHT, HIGH);
      PORTH |= 0x80;
      delay(500);
      digitalWrite(PWR_EN_5V, HIGH);
      delay(500);
      digitalWrite(PWR_EN_ARM, HIGH);
      delay(500);
      //Close_All_Valves();
      powerOn = true;
      Serial.println("Turned On");
    }
    Brewie_Read();
  }
}

bool Brewie_Read() {
  static uint8_t charCount;
  static uint32_t serialTimeout;
  static uint8_t fieldPos;
  static uint8_t fieldLen;
  static bool incomingCommand = false;

  if (incomingCommand) {
    if ((millis() - serialTimeout) > 100 && incomingCommand) {
      incomingCommand = false;
    } 
    if (Serial.available()) {
      brewieData[charCount] = Serial.peek();
      serialTimeout = millis();
      if (brewieData[charCount] == 0x20) {
        Serial.read();
        fieldPos++;
        brewieCommand[fieldPos][0] = charCount+1;
        brewieCommand[fieldPos-1][1] = fieldLen;
        fieldLen = 0;
      } else if (brewieData[charCount] == '$') {
        incomingCommand = false;
      } else {
        Serial.read();
        fieldLen++;
      }
      if (charCount-2 > brewieData[1]) {
        if (brewieData[charCount] == 0x2A) {
          //Serial.println("end char received");
          brewieCommand[0][0] = fieldPos;
          brewieCommand[fieldPos][1] = fieldLen-2;
          brewieData[charCount-1] = 0x00;
          incomingCommand = false;
          newCommand = true;
          Brewie_ACK_Write();
          Brewie_Command_Decode();
        }
      }
      if (charCount > 79) {
        incomingCommand = false;
      }
      charCount++;
    }
  } else if (Serial.available()) {
    if (Serial.read() == '$') {
      incomingCommand = true;
      serialTimeout = millis();
      // Command is being sent, extract data
      while(Serial.available() < 2) {
        if (millis() - serialTimeout > 100) {
          break;
        }
      }
      brewieData[0] = Serial.read();  // Command number
      brewieData[1] = Serial.read();  // Command length
      charCount = 2;
      fieldPos = 0;
      fieldLen = 0;
      for (uint8_t i = 0; i<24; i++) {
        brewieCommand[i][0] = 0;
        brewieCommand[i][1] = 0;
      }
    } else {
      Serial.read();
    }
  }

  return incomingCommand;
}

void Brewie_ACK_Write() {
  Serial.print("$");
  Serial.write(1);
  Serial.write(brewieData[0]);
  Serial.write(0x2A);
  Serial.write(0x0D);
  Serial.write(0x0A);
}

void Brewie_Status_Write() {
  // Build up status string
  uint8_t brewieLength = 0;
  char statusMessage[255];
  
  if (currStep == -1) {
    brewieLength  = sprintf(statusMessage, "%d\t%d\t\t\t", currStep, totalTime);
  } else {
    brewieLength  = sprintf(statusMessage, "%d\t%d\t%d\t%d\t", currStep, totalTime, stepTime, leftTime);
  }
  if (brewieMessage[0][0] != 0) {
    brewieLength += sprintf(&statusMessage[brewieLength], "%s,", brewieMessage);
  }
  if (errorMessage[0][0] != 0) {
    brewieLength += sprintf(&statusMessage[brewieLength], "%s,", errorMessage);
  }
  brewieLength += sprintf(&statusMessage[brewieLength], "V%d\t", mcuVersion);
  brewieLength += sprintf(&statusMessage[brewieLength], "%d\t", (uint16_t)massSensor);
  float floatTemp = (float)massSensor/toLiter;
  brewieLength += floatToStringAppend(&floatTemp, &statusMessage[brewieLength]);
  brewieLength += floatToStringAppend(&mashTemp, &statusMessage[brewieLength]);
  brewieLength += floatToStringAppend(&boilTemp, &statusMessage[brewieLength]);
  floatTemp = mashTemp*10.0;
  brewieLength += floatToStringAppend(&floatTemp, &statusMessage[brewieLength]);
  floatTemp = boilTemp*10.0;
  brewieLength += floatToStringAppend(&floatTemp, &statusMessage[brewieLength]);
  floatTemp = (float)(mashPump.pumpTach());
  brewieLength += floatToStringAppend(&floatTemp, &statusMessage[brewieLength]);
  floatTemp = (float)(boilPump.pumpTach());
  brewieLength += floatToStringAppend(&floatTemp, &statusMessage[brewieLength]);
  brewieLength += sprintf(&statusMessage[brewieLength], "%d\t\%d\t%d\t\%d\t", mashPump.pumpDiag(), (uint16_t)mashPumpA, boilPump.pumpDiag(), (uint16_t)boilPumpA);
  brewieLength += sprintf(&statusMessage[brewieLength], "%d\t\%d\t%d\t", (uint8_t)Brewie.MashHeaterOn(), (uint8_t)Brewie.BoilHeaterOn(), (uint16_t)boardTemp);
  if (newCommand) {
    brewieLength += sprintf(&statusMessage[brewieLength], echoMessage);
  }
  brewieLength += sprintf(&statusMessage[brewieLength], "\t");
  floatTemp = acMeasure;
  brewieLength += floatToStringAppend(&floatTemp, &statusMessage[brewieLength]);
  brewieLength += sprintf(&statusMessage[brewieLength], "\r\n");
  
  Serial.print(statusMessage);
}

void Brewie_Command_Decode() {
  static uint8_t hopTanksOpen = 0;
  if (echoCount > 0) {
    echoCount += sprintf(&echoMessage[echoCount], ",");
  }
  echoCount += sprintf(&echoMessage[echoCount], &brewieData[2]);
  
  if (brewieData[2] == 'P') {
    uint8_t commandNum = (brewieData[4]-0x30)*10+(brewieData[5]-0x30);
  
    if (brewieData[3] == '1') {
      if (commandNum == 3) {
        Brewie_Step();
      } else if (commandNum == 10) {
        digitalWrite(INLET_1, HIGH);
      } else if (commandNum == 11) {
        digitalWrite(INLET_1, LOW);
      } else if (commandNum == 24) {
        mashPump.setPumpSpeed(255);
      } else if (commandNum == 25) {
        mashPump.setPumpSpeed(0);
      } else if (commandNum == 26) {
        boilPump.setPumpSpeed(255);
      } else if (commandNum == 27) {
        boilPump.setPumpSpeed(0);
      } else if (commandNum == 28) {
        digitalWrite(INLET_2, HIGH);
      } else if (commandNum == 29) {
        digitalWrite(INLET_2, LOW);
      } else if (commandNum == 12) {
        setValve(VALVE_MASH_IN, VALVE_OPEN);
      } else if (commandNum == 13) {
        setValve(VALVE_MASH_IN, VALVE_CLOSE);
      } else if (commandNum == 14) {
        setValve(VALVE_BOIL_IN, VALVE_OPEN);
      } else if (commandNum == 15) {
        setValve(VALVE_BOIL_IN, VALVE_CLOSE);
      } else if (commandNum == 16) {
        setValve(VALVE_HOP_1, VALVE_OPEN);
        hopTanksOpen++;
      } else if (commandNum == 17) {
        setValve(VALVE_HOP_1, VALVE_CLOSE);
        hopTanksOpen--;
      } else if (commandNum == 18) {
        setValve(VALVE_HOP_2, VALVE_OPEN);
        hopTanksOpen++;
      } else if (commandNum == 19) {
        setValve(VALVE_HOP_2, VALVE_CLOSE);
        hopTanksOpen--;
      } else if (commandNum == 20) {
        setValve(VALVE_HOP_3, VALVE_OPEN);
        hopTanksOpen++;
      } else if (commandNum == 21) {
        setValve(VALVE_HOP_3, VALVE_CLOSE);
        hopTanksOpen--;
      } else if (commandNum == 22) {
        setValve(VALVE_HOP_4, VALVE_OPEN);
        hopTanksOpen++;
      } else if (commandNum == 23) {
        setValve(VALVE_HOP_4, VALVE_CLOSE);
        hopTanksOpen--;
      } else if (commandNum == 30) {
        setValve(VALVE_COOL, VALVE_OPEN);
      } else if (commandNum == 31) {
        setValve(VALVE_COOL, VALVE_CLOSE);
      } else if (commandNum == 32) {
        setValve(VALVE_OUTLET, VALVE_OPEN);
      } else if (commandNum == 33) {
        setValve(VALVE_OUTLET, VALVE_CLOSE);
      } else if (commandNum == 34) {
        setValve(VALVE_MASH_RET, VALVE_OPEN);
      } else if (commandNum == 35) {
        setValve(VALVE_MASH_RET, VALVE_CLOSE);
      } else if (commandNum == 36) {
        setValve(VALVE_BOIL_RET, VALVE_OPEN);
      } else if (commandNum == 37) {
        setValve(VALVE_BOIL_RET, VALVE_CLOSE);
      } else if (commandNum == 50) {
        Brewie.setTemperatures(atof(&brewieData[brewieCommand[1][0]])/10.0,atof(&brewieData[brewieCommand[1][0]])/10.0);
        safetyShutdown = false;
      } else if (commandNum == 51) {
        Brewie.setTemperatures(0,atof(&brewieData[brewieCommand[1][0]])/10.0);
        safetyShutdown = false;
      }
    } else if (brewieData[3] == '2') {
      if (commandNum == 0) {            // Start
        debugTime = millis();
        totalTime = 0;
        brewing = true;
        acMeasure = 0;
        digitalWrite(VENT_FAN, HIGH);
        digitalWrite(POWER_FAN, HIGH);
        Step_Change();
      } else if (commandNum == 1) {     // Pause
        Brewie_Pause();
      } else if (commandNum == 2) {     // Continue
        brewing = true;
        brewiePause = false;
        if (safetyShutdown) {
          // Possible error has occured, let user bypass
          errorOverride = true;
          waterOverride = true;
        }
        safetyShutdown = false;
        Process_Step();
        digitalWrite(VENT_FAN, HIGH);
        digitalWrite(POWER_FAN, HIGH);
      } else if (commandNum == 4) {     // Skip step
        totalTime += leftTime;
        stepTime = 0;
        leftTime = 0;
        Request_Next_Step();
        currStep++;
        // Necessary? Was stepComplete
        stepActive = false;
      } else if (commandNum == 5) {     // Developer Mode/Fast water readings
        if (brewieData[brewieCommand[1][0]] == '1') {
          brewing = true;
          powerOn = true;
          fanOverride = true;
          fastWaterReadings = true;
          digitalWrite(VENT_FAN, HIGH);
          digitalWrite(POWER_FAN, HIGH);
        } else {
          fanOverride = false;
          brewing = false;
          fastWaterReadings = false;
        }
      }
    } else if (brewieData[3] == '8') {
      // Load calibration constants to MCU
      digitalWrite(POWER_LIGHT, HIGH);
      toLiter = atof(&brewieData[brewieCommand[1][0]]);
      toLiterNull = atof(&brewieData[brewieCommand[2][0]]);
      mashDelta = atof(&brewieData[brewieCommand[3][0]]);
      boilDelta = atof(&brewieData[brewieCommand[4][0]]);
      PORTH |= 0x80;
      digitalWrite(PWR_EN_5V, HIGH);
      digitalWrite(PWR_EN_ARM, HIGH);
      Close_All_Valves();
      Brewie_Reset();
      powerOn = true;
    } else if (brewieData[3] == '9') {
      if (commandNum == 98) {
        if (currStep != -1) {
          if (brewieCommand[0][0] == 0) {
            brewiePause = true; 
            Close_All_Valves();
          } 
        } 
        if (brewieData[brewieCommand[1][0]] == '1') {
          digitalWrite(DRAIN_LIGHT, HIGH);
        } else if (brewieData[brewieCommand[1][0]] == '0') {
          digitalWrite(DRAIN_LIGHT, LOW);
        }
      }
      if (commandNum == 99) {
        if (brewieCommand[0][0] == 1) {
          Power_Off();
        } else {
          Close_All_Valves();
          Brewie_Reset();
        }
      }
    }
  }

  if (boilPump.isRunning()) {
    if (hopTanksOpen > 0) {
      boilPump.setPumpSpeed(80+50*(hopTanksOpen-1));
    }
  }
}

uint8_t floatToStringAppend(float* data, uint8_t* str) {
  dtostrf(*data, 6, 6, str);
  uint8_t start = 6;
  start += sprintf(&str[start], "\t");
  return start;
}

bool setValve(uint8_t valve, bool openNotClosed) {
  static uint8_t samps = 0;
  static uint32_t servoTime = 0;
  bool valveMoved = false;
  if ((valveState[valve] == 0 && openNotClosed) || (valveState[valve] == 1 && !openNotClosed)) {
    valvePWM = valve;
    uint16_t pwm = VALVE_CLOSE_ANGLE;
    if (openNotClosed) {
      pwm = VALVE_OPEN_ANGLE;
    }
    OCR4A = 1000 + (uint16_t)(pwm*17);
    digitalWrite(PWR_EN_SERVO, HIGH);
    TIMSK4 |= _BV(TOIE4) + _BV(OCIE4A);
    servoTime = millis();
    servosA = 0;
    uint16_t samps = 0;
    while(millis() - servoTime < ((VALVE_CLOSE_ANGLE - VALVE_OPEN_ANGLE)*6)) {
      servosA += analogRead(I_VALVES);
      samps++;
    }
    servosA /= samps;
    digitalWrite(PWR_EN_SERVO, LOW);
    delay(20);
    TIMSK4 &= ~(_BV(TOIE4) + _BV(OCIE4A));
    valveState[valve] = openNotClosed;
  }
  return valveMoved;
}

void Brewie_Step() {
  for (uint8_t go = 0; go < 22; go++) {
    brewieStepBuffer[go] = 0;
  }
  for (uint8_t commands = 1; commands <= brewieCommand[0][0]; commands++) {
    for (uint8_t len = 0; len < brewieCommand[commands][1]; len++) {
      brewieStepBuffer[commands-1] += brewieData[brewieCommand[commands][0]+len] - 0x30;
      if (brewieCommand[commands][1] - len > 1) {
        brewieStepBuffer[commands-1] *= 10;
      }
    }
  }
  newStep = true;
  brewing = true;
}

void Step_Change() {
  if (brewieStepBuffer[0] >= 0) {
    for (uint8_t go = 0; go < 22; go++) {
      brewieStep[go] = brewieStepBuffer[go];
      //brewieStepBuffer[go] = 0;
    }
    //brewieStepBuffer[0] = -1;
    leftTime = brewieStep[STEP_TIME];
    stepTime = 0;
    // Step command was received, enter brewing mode if it's the first one
    brewiePause = false;
    Process_Step();
  }
  errorOverride = false;
  waterOverride = false;
}

void Process_Step() {
  // Figure out where the step should go
  bool zeroTimeStep = false;
  currStep = brewieStep[0];

  float mashSetTemp = (float)brewieStep[STEP_MASH_TEMP]/10.0;
  float boilSetTemp = (float)brewieStep[STEP_BOIL_TEMP]/10.0;

  digitalWrite(INLET_1, LOW);
  digitalWrite(INLET_2, LOW);
  spargeOverride = false;
  brewieWaterFill = false;

  switch(brewieStep[STEP_PRIMARY]) {
    case 1: // Water inlet
      // Run "tank has water routine"
      brewieWaterFill = true;
      break;
    case 2: // Mash Temp
      break;
    case 3: // Boil Temp
      brewieStep[STEP_BOIL_PUMP] = 100;
      brewieStep[STEP_BOIL_INLET] = 1;
      brewieStep[STEP_BOIL_RETURN] = 1;
      //boilHeaterRequiredEnergy = (boilSetTemp - boilTemp)*4200.0*(massSensor/toLiter)/3600.00;
      break;
    case 4: // Mash Pump Empty
      //mashTankVolume = (float)brewieStep[STEP_WATER]/10.0;
      break;
    case 5: // Boil Pump Empty
      //mashTankVolume = (float)brewieStep[STEP_WATER]/10.0;
      break;
    case 6: // Run for time
      // If boil step, circulate
      if (brewieStep[STEP_BOIL_TEMP] == 930) {
        brewieStep[STEP_BOIL_PUMP] = 100;
        brewieStep[STEP_BOIL_INLET] = 1;
        brewieStep[STEP_BOIL_RETURN] = 1;
      }
      if (brewieStep[STEP_TIME] == 0) {
        zeroTimeStep = true;
      }
      break;
    case 7: // Hard Boil
      brewieStep[STEP_BOIL_PUMP] = 100;
      brewieStep[STEP_BOIL_INLET] = 1;
      brewieStep[STEP_BOIL_RETURN] = 1;
      brewieStep[STEP_BOIL_TEMP] = 1000;
      boilSetTemp = ((float)brewieStep[STEP_BOIL_TEMP])/10.0;
      break;
    case 8: // Last Step?
      break;
    case 9: // Unknown!
      break;
    case 10: // Cool Step
      break;
    default: 
      break;
  }
  switch(brewieStep[STEP_SECONDARY]) {
    case 1: // 
      break;
    case 2: // Sparge 
    {
      spargeOverride = true;
      // Override sparge valve and pump settings since they change on their own
      brewieStep[STEP_MASH_PUMP] = 0;
      brewieStep[STEP_BOIL_PUMP] = 0;
      brewieStep[STEP_MASH_INLET] = 0;
      brewieStep[STEP_BOIL_RETURN] = 0;
      brewieStep[STEP_MASH_RETURN] = 0;
      brewieStep[STEP_BOIL_INLET] = 0;
      break;
    }
    case 3: // Hop Additions
    {
      uint8_t hopTanks = brewieStep[STEP_HOP_1] + brewieStep[STEP_HOP_2] + brewieStep[STEP_HOP_3] + brewieStep[STEP_HOP_4];
      brewieStep[STEP_BOIL_PUMP] = 30 + (50*hopTanks);
      brewieStep[STEP_BOIL_TEMP] = 1000;
      boilSetTemp = ((float)brewieStep[STEP_BOIL_TEMP])/10.0;
      if (zeroTimeStep) {
        boilPump.setPumpSpeed(30);
      }
      break;
    }
    case 4:
      break;
    case 5: // Boil initial fill?
      break;
    case 6: // Boil sparge fill?
      break;
    default: 
      break;
  }

  if (zeroTimeStep == false) {
    // Valves
    setValve(VALVE_MASH_RET, brewieStep[STEP_MASH_RETURN]);
    setValve(VALVE_BOIL_RET, brewieStep[STEP_BOIL_RETURN]);
    setValve(VALVE_MASH_IN, brewieStep[STEP_MASH_INLET]);
    setValve(VALVE_BOIL_IN, brewieStep[STEP_BOIL_INLET]);
    setValve(VALVE_HOP_1, brewieStep[STEP_HOP_1]);
    setValve(VALVE_HOP_2, brewieStep[STEP_HOP_2]);
    setValve(VALVE_HOP_3, brewieStep[STEP_HOP_3]);
    setValve(VALVE_HOP_4, brewieStep[STEP_HOP_4]);
    setValve(VALVE_COOL, brewieStep[STEP_COOL_VALVE]);
    setValve(VALVE_OUTLET, brewieStep[STEP_OUTLET]); 
  }

  // Pumps
  // Mash Pump speed override, to slow it down for mashing
  if (brewieStep[STEP_MASH_PUMP] > 0) {
    if (brewieStep[STEP_MASH_INLET] == 0) {
      brewieStep[STEP_MASH_PUMP] = 255;
    } else if (brewieStep[STEP_MASH_INLET] == 1 && brewieStep[STEP_BOIL_INLET] == 1) {
      brewieStep[STEP_MASH_PUMP] = 255;
    } else {
      brewieStep[STEP_MASH_PUMP] = 100;
    }
  }
  mashPump.setPumpSpeed(brewieStep[STEP_MASH_PUMP]);
  boilPump.setPumpSpeed(brewieStep[STEP_BOIL_PUMP]);

  // Turn on water, for automatic cleaning
  if (brewieStep[STEP_WATER] == 0) {
    if (brewieStep[STEP_WATER_INLET] == 1) {
      digitalWrite(INLET_1, HIGH);
    }
  }

  Brewie.SetBoilMass((float)brewieStep[STEP_WATER]/10.0);
  Brewie.setTemperatures(mashSetTemp, boilSetTemp);
  
  newStep = false;
  stepActive = true;
}

bool Step_Complete() {
  bool stepComplete = false;
  if (currStep >= 0) {
    if (stepActive) {
      switch(brewieStep[STEP_PRIMARY]) {
        case 1: // Water inlet, fill to volume, check for water
          if (waterFilled) {
            digitalWrite(INLET_1, LOW);
            stepActive = false;
            brewieWaterFill = false;
            waterFilled = false;
            //boilTankVolume = (float)massSensor/toLiter;
          }
          break;
        case 2: // Run to Mash Temperature
          if (Brewie.MashTempReached()) {
            stepActive = false;
          }
          break;
        case 3: // Run to Boil Temperature
          if (Brewie.BoilTempReached()) {
            stepActive = false;
          }
          break;
        case 4: // Run until Mash Pump empty
          if (mashPump.pumpIsDry()) {
            stepActive = false;
            setValve(VALVE_MASH_RET, VALVE_CLOSE);
            //mashTankEmpty = true;
            //boilTankVolume = (float)massSensor/toLiter;
            //mashTankVolume = 0.0;
          }
          break;
        case 5: // Run until Boil Pump empty
          if (boilPump.pumpIsDry()) {
            stepActive = false;
            setValve(VALVE_BOIL_RET, VALVE_CLOSE);
            //boilTankEmpty = true;
            //boilTankVolume = 0.0;
            //mashTankVolume = (float)massSensor/toLiter;
          }
          break;
        case 6: // Run until time complete
          if (leftTime == 0) {
            stepActive = false;
            spargeOverride = false;
          }
          break;
        case 7: // Something to do with total boil time? Force boil? No boil over?
          if (Brewie.BoilTempReached()) {
            stepActive = false;
          }
          break;
        case 8: // Sedimentation
          if (leftTime == 0) {
            stepActive = false;
          }
          break;
      }
      if (stepActive == false) {
        //if (brewieStep[STEP_PRIMARY] == 8) {
        if (brewieStepBuffer[0] > -1) {
          Request_Next_Step();
          //Step_Change();
        }
        leftTime = 0;
        stepComplete = true;
      }
    }
  }
  return stepComplete;
}

void Process_Temperature() {
  //Process 1-Wire Temperatures
  if (powerOn) {
    if (MashTemp.isConnected(mashAddress)) {
      mashTemp = MashTemp.getTempC(mashAddress);
    }
    if (BoilTemp.isConnected(boilAddress)) {
      boilTemp = BoilTemp.getTempC(boilAddress);
    }
    if (mashTemp < -40) {
      mashTemp = -40;
      sprintf(errorMessage[errorCount][0], "E%d", 115);
      errorCount++;
      MashTemp.begin();
      MashTemp.getAddress(mashAddress, 0);
      MashTemp.setCheckForConversion(true);
      MashTemp.setWaitForConversion(false);
    }
    if (boilTemp < -40) {
      boilTemp = -40;
      sprintf(errorMessage[errorCount][0], "E%d", 116);
      errorCount++;
      BoilTemp.begin();
      BoilTemp.getAddress(mashAddress, 0);
      BoilTemp.setCheckForConversion(true);
      BoilTemp.setWaitForConversion(false);
    }
    MashTemp.requestTemperaturesByAddress(mashAddress);
    BoilTemp.requestTemperaturesByAddress(boilAddress);
  }
}

void Process_Sensors() {
  static uint32_t sensorTime = millis();
  static uint32_t massTime = millis();
  static uint32_t sensorArray[4] = { 0, 0, 0, 0 };
  static uint16_t samples = 0;
  static uint32_t massAve = 0;
  static uint8_t massSamp = 0;

  float timeDiff = millis() - sensorTime;
  if (timeDiff > 500) {
    sensorTime = millis();
    //Serial.println(samples);

    if (samples > 100) {
      acCurrent = (float)(sensorArray[0]/samples)/12.0;
      acMeasure += acCurrent/3600.0*120.0*(timeDiff/1000.0);             // Convert back to per sample period basis
      boardTemp = (uint16_t)((float)analogRead(BOARD_TEMP)*5.0/(6.1));
      inlet1A   = (float)(sensorArray[1]/samples);
      boilPumpA = (float)(sensorArray[2]/samples);
      mashPumpA = (float)(sensorArray[3]/samples);
  
      for (uint8_t clr = 0; clr < 4; clr++) {
        sensorArray[clr] = 0;
      }
      
      samples = 0;
    }
  }
  
  // Process Analog Inputs
  uint16_t acTemp = analogRead(AC_MEAS);
  if (acTemp > 7) {
    acTemp -= 8;
  } else {
    acTemp = 0;
  }
  sensorArray[0] += acTemp;
  sensorArray[1] += analogRead(I_INLET1);
  sensorArray[2] += analogRead(I_BOIL_PUMP);
  sensorArray[3] += analogRead(I_MASH_PUMP);

  if (millis() - massTime > 150) {
    // Process I2C Weight/Pressure Sensors
    massTime = millis();
    if (powerOn) {
      Wire.write(0x00);
      Wire.requestFrom(massAddress,2);
      if (Wire.available() == 2) {
        massFast = (uint16_t)((Wire.read() << 8) + Wire.read());
        massFast -= toLiterNull;
          
        if (massFast > 0) {
          massAve += massFast;
        } else {
          massAve += massSensor;
        }
        massSamp++;
      } else {
        sprintf(errorMessage[0][0], "E%d", 116);
        errorCount++;
      }
    }
  }
  
  if (boilPump.pumpTach() == 0) {
    if (massSamp > 6) {
      massSensor = (int16_t)massAve/massSamp;
      massAve = 0;
      massSamp = 0;
    }
  } else {
    massSamp = 0;
  }

  samples++;
}

void Close_All_Valves() {
  Brewie.Reset();
  digitalWrite(INLET_1, LOW);
  digitalWrite(INLET_2, LOW);
  mashPump.setPumpSpeed(0);
  boilPump.setPumpSpeed(0);
  setValve(VALVE_OUTLET, VALVE_CLOSE);
  setValve(VALVE_MASH_RET, VALVE_CLOSE);
  setValve(VALVE_BOIL_RET, VALVE_CLOSE);
  setValve(VALVE_BOIL_IN, VALVE_CLOSE);
  setValve(VALVE_MASH_IN, VALVE_CLOSE);
  setValve(VALVE_HOP_1, VALVE_CLOSE);
  setValve(VALVE_HOP_2, VALVE_CLOSE);
  setValve(VALVE_HOP_3, VALVE_CLOSE);
  setValve(VALVE_HOP_4, VALVE_CLOSE);
  setValve(VALVE_COOL, VALVE_CLOSE);
}

void Power_Off() {
  Brewie_Reset();
  setValve(VALVE_OUTLET, VALVE_CLOSE);
  setValve(VALVE_MASH_IN, VALVE_OPEN);
  setValve(VALVE_BOIL_IN, VALVE_OPEN);
  setValve(VALVE_HOP_1, VALVE_OPEN);
  setValve(VALVE_HOP_2, VALVE_OPEN);
  setValve(VALVE_HOP_3, VALVE_OPEN);
  setValve(VALVE_HOP_4, VALVE_OPEN);
  setValve(VALVE_COOL, VALVE_OPEN); 
  setValve(VALVE_MASH_RET, VALVE_OPEN);
  setValve(VALVE_BOIL_RET, VALVE_OPEN);
  digitalWrite(PWR_EN_SERVO, LOW);
  PORTH &= ~0x80;
  digitalWrite(PWR_EN_ARM, LOW);
  digitalWrite(PWR_EN_5V, LOW);
  digitalWrite(POWER_LIGHT, LOW);
  Serial.println("Power Off");
  powerOn = false;
}

bool Run_Sparge() {
  static uint16_t spargeNull = 0;
  static uint16_t boilHigh  = 0;
  static uint16_t boilLow = 0;
  static uint16_t mashSide = 2000;
  static uint8_t spargeStep = 0;
  static uint32_t spargeTime = 0;
  static uint16_t flowResult = 0;
  if (spargeOverride) {
    switch (spargeStep) {
      case 0:
        mashSide = 2000;
        mashPump.pumpFlowReset();
        boilPump.pumpFlowReset();
        spargeTime = millis();
        spargeStep++;
        break;
      case 1:
        if (millis() - spargeTime > 4000) {
          spargeNull = massSensor;
          boilLow = spargeNull;
          // Worth it???
          if (spargeNull < 8000) {
            spargeStep++;
          } else {
            sprintf(&brewieMessage[messageCount][0], "E%d%d", 0, 12);
            messageCount++;
            //Serial.println("Too Much Water In Tank");
            Brewie_Pause();
          }
        }
        break;
      case 2:
        // Run mash pump to boil side
        setValve(VALVE_BOIL_RET, VALVE_CLOSE);
        setValve(VALVE_MASH_IN, VALVE_CLOSE);
        setValve(VALVE_MASH_RET, VALVE_OPEN);
        mashPump.setPumpSpeed(50);
        setValve(VALVE_BOIL_IN, VALVE_OPEN); 
        mashPump.setPumpSpeed(255);
        spargeTime = millis();
        spargeStep++;
        break;
      case 3:
        if (mashPump.pumpFlow() > mashSide) {
          spargeStep++;
          mashPump.setPumpSpeed(30);
          setValve(VALVE_MASH_RET, VALVE_CLOSE);
          mashPump.setPumpSpeed(0);
          setValve(VALVE_BOIL_IN, VALVE_CLOSE);
          spargeTime = millis();
        }
        break;
      case 4:
        // Run boil pump to mash side
        if (millis() - spargeTime > 4000) {
          boilHigh = massSensor;
          setValve(VALVE_BOIL_RET, VALVE_OPEN);
          boilPump.setPumpSpeed(50);
          setValve(VALVE_MASH_IN, VALVE_OPEN);
          boilPump.setPumpSpeed(255);
          spargeTime = millis();
          spargeStep++;
        }
        break;
      case 5:
        // Stop when pump ticks are equal
        if (boilPump.pumpFlow() > 2000) {
          spargeStep++;
        }
        break;
      case 6:
        // Stop
        boilPump.setPumpSpeed(30);
        setValve(VALVE_BOIL_RET, VALVE_CLOSE);
        boilPump.setPumpSpeed(0);
        setValve(VALVE_MASH_RET, VALVE_CLOSE);
        setValve(VALVE_BOIL_IN, VALVE_CLOSE);
        setValve(VALVE_MASH_IN, VALVE_CLOSE);
        spargeTime = millis();
        spargeStep++;
        break;
      case 7:
        // Wait for things to settle
        if (millis() - spargeTime > 5000) {
          spargeStep++;
        }
        break;
      case 8:
      {
        // Adjust sparge routine to even out water level
        int16_t tachDiff = (int16_t)(boilPump.pumpFlow() - (mashSide - mashPump.pumpFlow()));
        int16_t deltaBoil = (int16_t)(boilHigh - boilLow);
        boilLow = massSensor;
        int16_t deltaMash = (int16_t)(boilHigh - boilLow);
        mashSide = 2000 + (deltaMash - deltaBoil) + (int16_t)(spargeNull - massSensor) + (tachDiff/10);
        Serial.print("Sparge Null: ");
        Serial.print(spargeNull);
        Serial.print(" Mash Side: ");
        Serial.println(deltaMash);
        Serial.print(" Boil Side: ");
        Serial.println(deltaBoil);
        spargeStep = 2;
        mashPump.pumpFlowReset();
        boilPump.pumpFlowReset();
        break;
      }
      default:
        spargeStep = 0;
        break;
    } 
  } else {
    spargeStep = 0;
  }
  return spargeStep > 0;
}

void Fill_Boil_Tank() {
  static uint8_t fillStep = 0;
  static uint8_t fillAttempts = 0;
  static float initialMass = 0;
  static float fillTarget = 0;
  static uint32_t startTime = 0;
  uint32_t waterTimeout = (uint32_t)brewieStep[STEP_WATER]*2000;
  if (brewieWaterFill) {
    switch (fillStep) {
      case 0:
        // Initialize variables
        Serial.print("Target Volume: ");
        Serial.println(brewieStep[STEP_WATER]);
        fillAttempts = 0;
        fillStep++;
        initialMass = 0;
        waterTime = millis();
        startTime = millis();
        break;
      case 1:
        // Wait to let water settle, just in case
        if (millis() - waterTime > 5000) {
          initialMass = (float)massSensor/toLiter;
          fillStep++;
        }
        break;
      case 2:
        // Decide which path to follow based on initial conditions
        Serial.print("Initial Fill: ");
        initialMass = (float)massSensor/toLiter;
        Serial.println(initialMass);
        if (fillAttempts == 0) {
          fillTarget = 5.0;
        } else {
          fillTarget = ((float)brewieStep[STEP_WATER]/10.0);
        }
        if (initialMass < fillTarget) {
          digitalWrite(INLET_1, HIGH);
          fillStep++;
        } else if ((initialMass-0.5) > fillTarget) {
          fillStep = 13;
        } else {
          fillStep = 6;
        }
        waterTime = millis();
        break;
      case 3:
        if ((float)(massFast/toLiter) < fillTarget*0.95) {
          // Water level still low
        } else {
          // Water at level, shut down
          Serial.print("Level: ");
          Serial.println((float)(massSensor/toLiter));
          fillStep = 4;
        }
        if (millis() - waterTime > waterTimeout) {
          if ((float)(massSensor/toLiter) < fillTarget*0.65) {
            fillStep = 12;
          } else {
            fillStep = 4;
          }
        }
        break;
      case 4:
        // Shut down
        digitalWrite(INLET_1, LOW);
        fillStep++;
        waterTime = millis();
        break;
      case 5:
        // Wait 5 seconds, check water level, and adjust as needed
        if (millis() - waterTime > 5000) {
          // Restart if water level is still very low
          if ((float)(massSensor/toLiter) < ((float)brewieStep[STEP_WATER]/10.0)*0.85) {
            fillAttempts++;
            fillStep = 2;
          } else {
            fillStep = 6;
          }
          if (fillAttempts > 3) {
            fillStep = 12;
          }
          float flowRate = ((float)(massSensor/toLiter) - initialMass)/((float)(millis() - startTime))*60000.0;
          Serial.print("Fill Rate: ");
          Serial.print(flowRate);
          Serial.println("L/min");
        }
        break;
      case 6:
        setValve(VALVE_BOIL_RET, VALVE_OPEN);
        setValve(VALVE_BOIL_IN, VALVE_OPEN);
        boilPump.setPumpSpeed(255);
        waterTime = millis();
        fillStep++;
        break;
      case 7:
        if ((millis() - waterTime) > 6000) {
          if(boilPump.pumpIsDry()) {
            fillStep = 14;
          } else {
            fillStep++;
          }
          boilPump.setPumpSpeed(0);
          setValve(VALVE_BOIL_RET, VALVE_CLOSE);
          setValve(VALVE_BOIL_IN, VALVE_CLOSE);
          waterTime = millis();
        }
        break;
      case 8:
        // Passed all steps so far...
        if (millis() - waterTime > 8000) {
          //boilSetTemp = 30.0;
          initialMass = (float)massSensor/toLiter;
          if (initialMass < (float)brewieStep[STEP_WATER]/10.0) {
            digitalWrite(INLET_1, HIGH);
            fillStep++;
          } else if (initialMass - 0.5 > (float)brewieStep[STEP_WATER]/10.0) {
            fillStep = 13;
          } else {
            fillStep = 11;
          }
          waterTime = millis();
        }
        break;
      case 9:
        if ((float)(massFast/toLiter) < (float)brewieStep[STEP_WATER]/10.0) {
          // Water level still low
        } else {
          // Water at level, shut down
          fillStep++;
        }
        if (millis() - waterTime > (waterTimeout/4)) {
          fillStep++;
        }
        break;
      case 10:  
        // Shut down
        digitalWrite(INLET_1, LOW);
        fillStep++;
        waterTime = millis();
        break;
      case 11:
        // Wait 5 seconds, check water level, and adjust as needed
        if (millis() - waterTime > 5000) {
          // Should be > 3?
          if (fillAttempts == 3) {
            fillStep = 12;
          }
          if (((float)massSensor/toLiter)*1.01 < ((float)brewieStep[STEP_WATER]/10.0)) {
            fillStep = 8;
            fillAttempts++;
          } else {
            // Filled to volume
            waterFilled = true;
            brewieWaterFill = false;
            fillStep = 0;
          }
          waterTime = millis();
        }
        break;
      case 12:
        // No Inlet
        brewieWaterFill = false;
        Brewie_Pause();
        sprintf(&errorMessage[errorCount][0], "E005");
        errorCount++;
        fillStep = 0;
        currStep = -2;
        break;
      case 13: 
        // Too much water
        if (waterOverride) {
          waterFilled = true;
          brewieWaterFill = false;
          fillStep = 0;
        } else {
          safetyShutdown = true;
          brewieWaterFill = false;
          Brewie_Pause();
          sprintf(&errorMessage[errorCount][0], "E012");
          errorCount++;
          fillStep = 0;
          currStep = -2;
        }
        break;
      case 14:
        // No Water Detected
        brewieWaterFill = false;
        Brewie_Pause();
        sprintf(&errorMessage[errorCount][0], "E011");
        errorCount++;
        fillStep = 0;
        currStep = -2;
        break;
      default:
        fillStep = 0;
        break;
    }
  } else {
    fillStep = 0;
  }
}

void Safety_Check() {
  if (acCurrent > 16.0) {
    // Shut down heaters
    safetyShutdown = true;
  } else if (acCurrent < 10.0) {
    digitalWrite(DRAIN_LIGHT, LOW);
    digitalWrite(POWER_LIGHT, LOW);
    if (Brewie.MashHeaterOn()) {
      if (digitalRead(POWER_LIGHT) == LOW) {
        digitalWrite(POWER_LIGHT, HIGH);
      } else {
        digitalWrite(POWER_LIGHT, LOW);
      }
    } else if (Brewie.BoilHeaterOn()) {
      if (digitalRead(DRAIN_LIGHT) == LOW) {
        digitalWrite(DRAIN_LIGHT, HIGH);
      } else {
        digitalWrite(DRAIN_LIGHT, LOW);
      }
    }
  } else {
    if (Brewie.MashHeaterOn()) {
      digitalWrite(POWER_LIGHT, HIGH);
    } else {
      digitalWrite(POWER_LIGHT, LOW);
    }
    if (Brewie.BoilHeaterOn()) {
      digitalWrite(DRAIN_LIGHT, HIGH);
    } else {
      digitalWrite(DRAIN_LIGHT, LOW);
    }
  }
  // Turn off boil heater if no water detected
  if (!errorOverride) {
    if (Brewie.BoilHeaterOn()) {
      if (boilPump.pumpDiag() == 0) {
        if ((float)(massSensor/toLiter) < 4.0) {
          safetyShutdown = true;
        }
      } else if (boilPump.pumpIsDry()) {
        safetyShutdown = true;
      }
    }
    if (Brewie.MashHeaterOn()) {
      if (mashPump.pumpIsDry()) {
        safetyShutdown = true;
      }
    }

    // Pump running dry safety
    /*if (brewieStep[STEP_PRIMARY] != 4) {
      if (mashPump.pumpIsDry()) {
        safetyShutdown = true;
      }
    } else if (brewieStep[STEP_PRIMARY] != 5) {
      if (boilPump.pumpIsDry()) {
        safetyShutdown = true;
      }
    }*/
  }
  if (boardTemp > 400) {
    digitalWrite(VENT_FAN, HIGH);
    digitalWrite(POWER_FAN, HIGH);
    if (boardTemp > 500) {
      safetyShutdown = true;
    } else if (boardTemp < 450) {
      safetyShutdown = false;
    }
  } else if (!brewing && !fanOverride && boardTemp < 350) {
    digitalWrite(VENT_FAN, LOW);
    digitalWrite(POWER_FAN, LOW);
  }
  if (safetyShutdown && !brewiePause && !errorOverride) {
    Brewie_Pause();
    sprintf(&brewieMessage[messageCount][0], "E%d%d", 0, 20);
    messageCount++;
  }
}

void Control_Loop() {
  // Print out status info every second
  uint32_t currTime = millis();
  uint32_t timeDiff = currTime - debugTime;
  if (timeDiff >= 999) {
    debugTime = currTime;
    uint8_t extraTime = (timeDiff/999);
    leftOverTime += timeDiff % 999;
    if (leftOverTime > 1000) {
      extraTime++;
      leftOverTime = leftOverTime % 1000;
    }
    
    if (powerButton) {
      sprintf(&brewieMessage[messageCount][0], "P%d", 999);
      messageCount++;
      powerButton = false;
    }
    if (drainButton) {
      sprintf(&brewieMessage[messageCount][0], "P%d", 998);
      messageCount++;
      drainButton = false;
    }

    Process_Temperature();
    
    mashPump.Pump_Speed_Control((uint16_t)mashPumpA);
    boilPump.Pump_Speed_Control((uint16_t)boilPumpA);

    Safety_Check();

    Brewie_Status_Write();

    if (!stepActive) {
      if (!brewiePause) {
        if (brewieStep[STEP_PRIMARY] == 8) {
          if (brewieStep[0] > 10) {
            currStep = -1;
            totalTime = 0;
          } else {
            Step_Change();
          }
        } else {
          Step_Change();
        }
      }
      //Process_Step();
    }

    brewieMessage[0][0] = 0;
    errorMessage[0][0] = 0;
    echoCount = 0;
    newCommand = false;

    messageCount = 0;
    errorCount = 0;
    
    if (currStep > -1) {
      if (!brewiePause) {
        totalTime += extraTime;
        stepTime += extraTime;
        if (leftTime >= extraTime) {
          leftTime -= extraTime;
        }
      }
    }
  }
}

void Request_Next_Step() {
  sprintf(&brewieMessage[messageCount][0], "P%d", 103);
  messageCount++;
  //brewieStepBuffer[0] = -1;
}

void Initialize_2560() {
  //PRR0 = (1<<PRTWI);     // turn off TWI
  //PRR1 = /*(1<<PRTIM5) | (1<<PRTIM3) |*/ (1<<PRUSART1) | (1<<PRUSART2) | (1<<PRUSART3);
  DIDR2 = 0x7F; // ADC8-14 are analog ins
  PORTK |= 0x80;  // ADC15 input high
  /*DDRF  |=~0xF8;
  PORTF |= 0xF8;
  PORTA = 0;
  DDRA  |= 0xFF;
  DDRJ  |= 0x7C;
  PORTJ |=~0x7C;
  DDRC  |= 0xAA;
  PORTC |=~0xAA;
  DDRD  |= 0xB3;
  PORTD |=~0xB7;
  DDRL  |= 0x20;
  PORTL |=~0x20;
  DDRB  |= 0xF6;
  PORTB |=~0xF6;
  DDRH  |= 0xC0;
  PORTH |=~0xC0;
  DDRE  |= 0x05;
  PORTE |=~0x05;

  DDRG  |= 0x20;
  PORTG |=~0x23;*/
  
  
  // Set up Timer 4 for Servos
  TCCR4A = 0;
  TCCR4B =_BV(WGM43) |  _BV(CS41); 
  ICR4 = 20000;

  /*PORTA = 0xFF;
  PORTB = 0xFF;
  PORTC = 0xFF;
  PORTD = 0xFF;
  PORTE = 0xFF;
  PORTF = 0xFF;
  PORTG = 0xFF;
  PORTH = 0xFF;
  PORTJ = 0xFF;
  PORTK = 0xFF;
  PORTL = 0xFF;

  PORTA |= ~0xF8;
  DDRA  |= 0xF8;
  PORTC |= ~0x02;
  DDRC  |= 0x02;
  PORTJ |= ~0x7C;
  DDRJ  |= 0x7C;
  PORTB |= ~0x80;
  DDRB  |= 0x80;
  PORTB |= 0x80;*/

  pinMode(VENT_FAN, OUTPUT);
  pinMode(POWER_FAN, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED1, OUTPUT);

  // SPI for DAC
  pinMode(SPEED_CTRL_SCLK, OUTPUT);
  pinMode(SPEED_CTRL_MOSI, OUTPUT);
  pinMode(SPEED_CTRL_LDAC, OUTPUT);
  pinMode(SPEED_CTRL_CS, OUTPUT);
  digitalWrite(SPEED_CTRL_LDAC, HIGH);  // ~LDAC
  digitalWrite(SPEED_CTRL_CS, HIGH); // DAC ~CS
  pinMode(50, INPUT);   //MISO, DAC doesn't send any data though
  pinMode(53, OUTPUT);  //Main ~CS, needs to be output for SPI to work

  pinMode(PWR_EN_SERVO, OUTPUT);
  digitalWrite(PWR_EN_SERVO, LOW);
  
  // Power Enable 12V
  DDRH |= 0x80;
  PORTH &= ~0x80;
  delay(100);

  pinMode(PWR_EN_5V, OUTPUT);
  pinMode(PWR_EN_ARM, OUTPUT);
  
  digitalWrite(PWR_EN_5V, HIGH);
  pinMode(MASH_PUMP_TACH, INPUT_PULLUP);
  pinMode(BOIL_PUMP_TACH, INPUT_PULLUP);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();
  
  pinMode(POWER_BUTTON, INPUT);
  pinMode(DRAIN_BUTTON, INPUT);
  pinMode(POWER_LIGHT, OUTPUT);
  pinMode(DRAIN_LIGHT, OUTPUT);
  digitalWrite(POWER_LIGHT, LOW);
  digitalWrite(DRAIN_LIGHT, LOW); 
  
  Serial.begin(115200);
  Wire.begin();

  attachInterrupt(digitalPinToInterrupt(BOIL_PUMP_TACH), boilTachISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(MASH_PUMP_TACH), mashTachISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(DRAIN_BUTTON), drainButtonISR, RISING);

  MashTemp.begin();
  BoilTemp.begin();

  MashTemp.getAddress(mashAddress, 0);
  MashTemp.setCheckForConversion(true);
  MashTemp.setWaitForConversion(false);
  BoilTemp.getAddress(boilAddress, 0);
  BoilTemp.setCheckForConversion(true);
  BoilTemp.setWaitForConversion(false);

  MashTemp.requestTemperaturesByAddress(mashAddress);
  BoilTemp.requestTemperaturesByAddress(boilAddress);

  debugTime = millis();

  // Object data assignments
  Brewie_Reset();
  Brewie.setTemperatureSensors(&mashTemp, &boilTemp);
  Brewie.setPowerSensor(&acMeasure);

  mashPump.pumpTicks = &mashTicks;
  boilPump.pumpTicks = &boilTicks;
}

void Brewie_Reset() {
  currStep = -1;
  for (uint8_t go = 0; go < 22; go++) {
    brewieStep[go] = 0;
    brewieStepBuffer[go] = 0;
  }
  brewieStep[0] = -1;
  brewieStepBuffer[0] = -1;
  totalTime = 0;
  stepTime = 0;
  leftTime = 0;
  brewing = false;
  brewieWaterFill = false;
  spargeOverride = false;
  safetyShutdown = false;
  errorOverride = false;
  Brewie.Reset();
  mashPump.setPumpSpeed(0);
  boilPump.setPumpSpeed(0);
  digitalWrite(INLET_1, LOW);
  digitalWrite(INLET_2, LOW);
}

void Brewie_Pause() {
  brewiePause = true;
  if (currStep >= 0) {
    //brewiePause = true;
    brewieStep[STEP_TIME] = leftTime;
    currStep = -2;
    Close_All_Valves();
    brewieWaterFill = false;
    spargeOverride = false;
    brewing = false;
  }
}

void Fast_Water_Readings() {
  if (fastWaterReadings) {
    if (millis() % 200 == 0) {
      Serial.print("RL");
      Serial.println(massFast);
      delay(5);
    }
  }
}

ISR(TIMER4_COMPA_vect){
  *Valve_Port[valvePWM] &= ~Valve_Bitmask[valvePWM];
}

ISR(TIMER4_OVF_vect){
  *Valve_Port[valvePWM] |= Valve_Bitmask[valvePWM];
}

void boilTachISR() {
  boilTicks++;
}

void mashTachISR() {
  mashTicks++;
}

void drainButtonISR() {
  drainButton = true;
}

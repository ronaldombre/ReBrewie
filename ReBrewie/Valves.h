#ifndef Valves_h
#define Valves_h

#include <Arduino.h>
#include "B20Plus.h"
// Valves
#define VALVE_MASH_IN       0
#define VALVE_BOIL_RET      1
#define VALVE_OUTLET        2
#define VALVE_COOL          3
#define VALVE_HOP_4         4
#define VALVE_HOP_3         5
#define VALVE_HOP_2         6
#define VALVE_HOP_1         7
#define VALVE_MASH_RET      8
#define VALVE_BOIL_IN       9
#define VALVE_OPEN_ANGLE    10
#define VALVE_CLOSE_ANGLE   135
#define VALVE_PINCH_ANGLE   76
#define VALVE_OPEN          1
#define VALVE_CLOSE         0

// Valve structures
extern volatile uint8_t* Valve_Port[10];
extern uint8_t Valve_Bitmask[10];
extern uint8_t valveState[10];

extern uint8_t valvePWM;
extern uint8_t valveCount;

bool setValve(uint8_t, uint8_t);

#endif

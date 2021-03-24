#include "Valves.h"

#ifdef  B20
volatile uint8_t* Valve_Port[10]  = { &PORTA, &PORTA, &PORTJ, &PORTJ, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA };
uint8_t Valve_Bitmask[10]         = { 0x01,   0x08,   0x02,   0x01,   0x10,   0x20,   0x40,   0x80,   0x02,   0x04 };
uint8_t valveState[10]            = {    1,      1,      1,      1,      1,      1,      1,      1,      1,      1 };
#else
volatile uint8_t* Valve_Port[10]  = { &PORTJ, &PORTJ, &PORTJ, &PORTJ, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTC };
uint8_t Valve_Bitmask[10]         = { 0x04,   0x08,   0x20,   0x40,   0x20,   0x08,   0x80,   0x10,   0x40,   0x02 };
uint8_t valveState[10]            = {   10,     10,     10,     10,     10,     10,     10,     10,     10,     10 };
#endif
uint8_t valvePWM = 0;
uint8_t valveCount = 0;

bool setValve(uint8_t valve, uint8_t angle) {
  uint16_t setAngle = 0;

  // Use 0 and 1 to mean closed and open, but also allow for custom angles for higher values
  if (angle == 0) {
    // Valve Closed 
    setAngle = VALVE_CLOSE_ANGLE; 
  } else if (angle == 1) {
    setAngle = VALVE_OPEN_ANGLE;
  } else if (angle <= VALVE_CLOSE_ANGLE) {
    setAngle = angle;
  } else if (angle == 255) {
    setAngle = VALVE_OPEN_ANGLE;
  } else {
    Serial.println("!Bad Valve Setpoint");
  }

  if (valveState[valve] != setAngle) {
    valvePWM = valve;
    OCR4A = 1000 + (uint16_t)(setAngle*17);
    digitalWrite(PWR_EN_SERVO, HIGH);
    TIMSK4 |= _BV(TOIE4) + _BV(OCIE4A);
    // Servo seems to take 100ms to react to a change
    delay(100);
    uint32_t servoTime = millis();
    uint16_t valveSum = 0;
    while(millis() - servoTime < 500) {
      for (int x = 1; x < 64; x++) {
        valveSum += analogRead(I_VALVES);
      }
      uint16_t valveI = valveSum/64;
      valveSum = 0;
      if (valveI < 40) {
        break;
      }
    }
    //delay(50);
    digitalWrite(PWR_EN_SERVO, LOW);
    TIMSK4 &= ~(_BV(TOIE4) + _BV(OCIE4A));
    valveState[valve] = setAngle;
  }
  return valveState[valve] > VALVE_CLOSE_ANGLE;
}

ISR(TIMER4_COMPA_vect){
  *Valve_Port[valvePWM] &= ~Valve_Bitmask[valvePWM];
}

ISR(TIMER4_OVF_vect){
  *Valve_Port[valvePWM] |= Valve_Bitmask[valvePWM];
}

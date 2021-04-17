#include "Valves.h"

#ifdef  B20
volatile uint8_t* Valve_Port[10]  = { &PORTA, &PORTA, &PORTJ, &PORTJ, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA };
uint8_t Valve_Bitmask[10]         = { 0x01,   0x08,   0x02,   0x01,   0x10,   0x20,   0x40,   0x80,   0x02,   0x04 };
#else
volatile uint8_t* Valve_Port[10]  = { &PORTJ, &PORTJ, &PORTJ, &PORTJ, &PORTA, &PORTA, &PORTA, &PORTA, &PORTA, &PORTC };
uint8_t Valve_Bitmask[10]         = { 0x04,   0x08,   0x20,   0x40,   0x20,   0x08,   0x80,   0x10,   0x40,   0x02 };
#endif
uint8_t valveState[10]            = {    1,      1,      1,      1,      1,      1,      1,      1,      1,      1 };
uint8_t valveError[10]            = {    0,      0,      0,      0,      0,      0,      0,      0,      0,      0 };
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
    TCNT4 = ICR4;
    OCR4A = 1200 + (uint16_t)(setAngle*10);
    TIMSK4 |= _BV(TOIE4) + _BV(OCIE4A);
    digitalWrite(PWR_EN_SERVO, HIGH);  
    // Servo seems to take 100ms to react to a change
    delay(110);
    uint32_t servoTime = millis();
    uint32_t valveSum = 0;
    uint16_t valveSamples = 0;
    while(millis() - servoTime < 250) {
      valveSum += analogRead(I_VALVES);
      valveSamples++;
    }
    uint16_t valveI = (uint16_t)(valveSum/valveSamples);
    if (valveI > 100) {
      while(millis() - servoTime < 500) {
        valveSum += analogRead(I_VALVES);
        valveSamples++;
      }
    }
    TIMSK4 &= ~(_BV(TOIE4) + _BV(OCIE4A));
    digitalWrite(PWR_EN_SERVO, LOW); 
    valveI = (uint16_t)(valveSum/valveSamples);
    Serial.println(valveI);
    if (valveI < 30) {
      // Valve likely unplugged/broken
      valveError[valve]++;
    } else if (valveI < 90) {
      // Valve likely did not move, but is plugged in
    } else {
      valveError[valve] = 0;
    }
    valveState[valve] = setAngle;
  }
  return valveState[valve] > VALVE_CLOSE_ANGLE;
}

ISR(TIMER4_COMPA_vect){
  cli();
  *Valve_Port[valvePWM] &= ~Valve_Bitmask[valvePWM];
  sei();
}

ISR(TIMER4_OVF_vect){
  cli();
  *Valve_Port[valvePWM] |= Valve_Bitmask[valvePWM];
  sei();
}

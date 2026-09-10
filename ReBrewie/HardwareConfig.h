/*
 * Brewie AVR hardware selection.
 *
 * Select the physical controller board here.  The B20 uses the HX711 load
 * cell interface and the B20+ uses the original I2C pressure sensor.
 */
#ifndef BREWIE_HARDWARE_CONFIG_H
#define BREWIE_HARDWARE_CONFIG_H

#ifndef BREWIE_HARDWARE_B20
#define BREWIE_HARDWARE_B20 1
#endif

#include "B20Plus.h"

#if BREWIE_HARDWARE_B20
  #define BREWIE_USE_HX711 1

  #undef MASH_HEATER
  #define MASH_HEATER 13
  #undef BOIL_HEATER
  #define BOIL_HEATER 4

  // Original B20 controller I/O. B20Plus.h supplies the shared power and
  // heater definitions; these signals differ on the two board revisions.
  #undef LED1
  #define LED1 41
  #undef LED2
  #define LED2 40
  // PG0, ATmega2560 package pin 51, Arduino Mega digital pin 41.
  #undef POWER_LIGHT
  #define POWER_LIGHT 41
  #undef DRAIN_LIGHT
  #define DRAIN_LIGHT 40
  // The B20 wiring has the physical power button on AVR pin 19 and the
  // drain button on AVR pin 38.  B20Plus.h uses the opposite legacy map.
  #undef POWER_BUTTON
  #define POWER_BUTTON 19
  #undef DRAIN_BUTTON
  #define DRAIN_BUTTON 38
  // PJ2, ATmega2560 package pin 65.  PJ2 has no Arduino Mega digital-pin
  // alias in the standard variant; use direct PORTJ access.
  #define B20_VALVE_POWER_PORT PORTJ
  #define B20_VALVE_POWER_DDR DDRJ
  #define B20_VALVE_POWER_BIT PJ2
  #undef BOIL_PUMP
  #define BOIL_PUMP 2
  #undef MASH_PUMP
  #define MASH_PUMP 5
  // B20 LMV324 current-sense outputs. On the B20 controller wiring, PF0/A0
  // is the mash/pump-1 sense channel and PF1/A1 is the boil/pump-2 sense
  // channel. PF4/A4 is the servo/valve current channel.
  #undef I_BOIL_PUMP
  #define I_BOIL_PUMP A1
  #undef I_MASH_PUMP
  #define I_MASH_PUMP A0
  #undef I_VALVES
  #define I_VALVES A4
  #undef INLET_1
  #define INLET_1 7
  #undef INLET_2
  #define INLET_2 6
  #undef VENT_FAN
  #define VENT_FAN 46
  #undef POWER_FAN
  #define POWER_FAN 8

  #undef MASH_TEMP
  #define MASH_TEMP 11
  #undef BOIL_TEMP
  #define BOIL_TEMP 10
#else
  #define BREWIE_USE_HX711 0
#endif

#endif

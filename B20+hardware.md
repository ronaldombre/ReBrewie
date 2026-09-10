# Brewie B20+ hardware reference

This document describes the B20+ profile selected with:

```cpp
#define BREWIE_HARDWARE_B20 0
```

The B20+ profile is defined primarily by `ReBrewie/B20Plus.h`. The default
build currently selects the B20 profile through `HardwareConfig.h`; set the
flag to `0` only when building for a real B20+ controller.

## Main differences from B20

| Feature | B20+ | B20 |
|---|---|---|
| Level measurement | I2C pressure sensor | HX711 load cells |
| HX711 pins | Not used | PH2/PH7 |
| Pump outputs | D32/D34 | D2/D5 |
| Pump tachometers | D2/D3 | Board-specific B20 wiring |
| Valve power | D13 (`PWR_EN_SERVO`) | PJ2 |
| Cooling | Automatic cooling controller available | `P128/P129` directly operate cool inlet |
| Heater outputs | Port-register definitions for the B20+ board | Ordinary digital outputs |

## Power and control signals

Arduino pin numbers are the numbers used by the Arduino APIs. Port names are
the ATmega2560 electrical names; package pin numbers are included where they
are useful for probing.

| Symbol | Arduino pin | ATmega2560 signal | Function |
|---|---:|---|---|
| `PWR_EN_SERVO` | 13 | PB7, package pin 17 | Servo/valve power enable |
| `PWR_EN_ARM` | 12 | PB6, package pin 16 | ARM/MCU power enable |
| `PWR_EN_5V` | 11 | PB5, package pin 15 | 5 V power enable |
| `PWR_EN_12V` | — | PH7, package pin 27 | 12 V power latch (`PORTH bit 7`) |
| `PWR_12V_SENSE` | 53 | PB0, package pin 10 | 12 V sense/input |
| `POWER_BUTTON` | 38 | PD7, package pin 50 | Power button |
| `DRAIN_BUTTON` | 19 | PD2, package pin 45 | Drain button |
| `POWER_LIGHT` | 44 | PL5 | Power LED |
| `DRAIN_LIGHT` | 30 | PC7, package pin 42 | Drain LED |

## Heaters, pumps, and DAC

| Symbol | Arduino pin | ATmega2560 signal | Function |
|---|---:|---|---|
| `MASH_HEATER` | 4 | PG5, package pin 56 | Mash heater output |
| `BOIL_HEATER` | — | PE2, package pin 4 | Boil heater output (`PORTE bit 2`) |
| `BOIL_PUMP` | 32 | PC5 | Boil pump output (`P126/P127`) |
| `MASH_PUMP` | 34 | PC3 | Mash pump output (`P124/P125`) |
| `BOIL_PUMP_TACH` | 2 | PE4, package pin 6 | Boil pump tachometer |
| `MASH_PUMP_TACH` | 3 | PE5, package pin 7 | Mash pump tachometer |
| `SPEED_CTRL_MOSI` | 51 | PB2, package pin 12 | Pump-speed DAC SPI data |
| `SPEED_CTRL_CS` | 10 | PB4, package pin 14 | Pump-speed DAC chip select |
| `SPEED_CTRL_SCLK` | 52 | PB1, package pin 11 | Pump-speed DAC SPI clock |
| `SPEED_CTRL_LDAC` | 9 | PH6 | Pump-speed DAC latch |

The pump speed is set through the SPI DAC. `P124/P125` control the mash pump
and `P126/P127` control the boil pump. Pump-current monitoring uses the
analog channels listed below, not the B20 A0/A1 wiring.

## Sensors and analog inputs

| Symbol | Arduino input | ATmega2560 signal | Function |
|---|---:|---|---|
| `BOIL_TEMP` | 40 | PG1, package pin 52 | Boil-tank DS18B20 bus |
| `MASH_TEMP` | 41 | PG0, package pin 51 | Mash-tank DS18B20 bus |
| `AC_MEAS` | `A8` | PK0 / ADC8, package pin 89 | Total AC-current measurement |
| `BOARD_TEMP` | `A9` | PK1 / ADC9, package pin 88 | Board-temperature input |
| `I_INLET1` | `A10` | PK2 / ADC10 | Inlet diagnostic/current input |
| `I_INLET2` | `A11` | PK3 / ADC11 | Inlet diagnostic/current input |
| `I_VALVES` | `A12` | PK4 / ADC12 | Valve/servo current |
| `I_BOIL_PUMP` | `A13` | PK5 / ADC13 | Boil pump current |
| `I_MASH_PUMP` | `A14` | PK6 / ADC14 | Mash pump current |

The pressure sensor is read through the ATmega2560 TWI/I2C peripheral:

```text
SDA: PD1 / Arduino D20 / package pin 44
SCL: PD0 / Arduino D21 / package pin 43
```

The firmware scans for a responding I2C address and uses the discovered
device for pressure and temperature data. The initial address constant is
`25` decimal, but the scan result is authoritative.

`E210` belongs to this B20+ AC-current path. Its inherited limits are 20 A
when `machineVoltage` is not 240 and 12 A when it is 240.

## Inlets, valves, and fans

| Symbol | Arduino pin | ATmega2560 signal | Function |
|---|---:|---|---|
| `INLET_1` | 23 | PA1, package pin 23 | Water/cool inlet output |
| `INLET_2` | 24 | PA2, package pin 24 | Second inlet output |
| `VENT_FAN` | 22 | PA0, package pin 22 | Ventilation fan |
| `POWER_FAN` | `A0` | PF0 / ADC0, package pin 97 | Electronics/power fan |

The remaining valve outputs are defined in `Valves.cpp` through the B20+
port/bit table. B20+ uses its own valve table; do not apply the B20 PJ4/PJ5
corrections to a B20+ binary.

## Protocol behavior

The standard actuator commands remain compatible with the application:

| Command | Function |
|---|---|
| `P124/P125` | Mash pump start/stop |
| `P126/P127` | Boil pump start/stop |
| `P128/P129` | B20+ cooling-controller entry/exit behavior |
| `P150/P151` | Mash/boil heater targets |
| `P205` | Fan control/override |
| `P999` | Close all valves |

Unlike B20, the B20+ implementation can use automatic cooling in recipe
steps and through the cooling controller. The B20 build deliberately
compiles that behavior out and treats `P128/P129` as direct cool-inlet
commands.

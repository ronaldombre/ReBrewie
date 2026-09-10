# Brewie B20 hardware reference

This document describes the hardware selected by `BREWIE_HARDWARE_B20` in
`ReBrewie/HardwareConfig.h`. The source starts with the B20+ definitions and
overrides the signals that differ on the original B20 controller.

## Selecting the hardware

```cpp
#define BREWIE_HARDWARE_B20 1   // original B20
// #define BREWIE_HARDWARE_B20 0 // B20+
```

| Build value | Level sensor | Behavior |
|---|---|---|
| `1` | HX711 load-cell interface | Original B20 I/O and direct cooling commands |
| `0` | B20+ I2C pressure sensor | B20+ I/O and automatic cooling behavior |

This is a complete hardware selection, not just a feature switch. A binary
built for the wrong value can drive the wrong AVR pins and interpret the
wrong sensors.

## B20 signal map

Arduino pin numbers are the numbers passed to `pinMode()`, `digitalWrite()`,
and `analogRead()`. The ATmega2560 port name is the electrical reference.

| Symbol | Arduino pin | ATmega2560 signal | Function |
|---|---:|---|---|
| `MASH_HEATER` | 13 | PB7 | Mash heater |
| `BOIL_HEATER` | 4 | PG5 | Boil heater |
| `BOIL_PUMP` | 2 | PE4 | Logical boil pump (`P126/P127`) |
| `MASH_PUMP` | 5 | PE3 | Logical mash pump (`P124/P125`) |
| `INLET_1` | 7 | PH4 | Water-inlet output |
| `INLET_2` | 6 | PH3, package pin 15 | Cool-inlet output (`P128/P129`) |
| `POWER_LIGHT` | 41 | PG0, package pin 51 | Power LED |
| `DRAIN_LIGHT` | 40 | PG1, package pin 52 | Drain LED |
| `POWER_BUTTON` | 19 | PD2 | AVR-side power-button input |
| `DRAIN_BUTTON` | 38 | PD7 | AVR-side drain-button input |
| `VENT_FAN` | 46 | PL3 | Ventilation fan |
| `POWER_FAN` | 8 | PH5 | Electronics/power fan |
| `B20_VALVE_POWER_*` | — | PJ2, package pin 65 | Valve-converter enable |

### Sensors and analog inputs

| Symbol | Arduino input | ATmega2560 signal | Function |
|---|---:|---|---|
| `MASH_TEMP` | 11 | PB5 | Mash-tank DS18B20 bus |
| `BOIL_TEMP` | 10 | PB4 | Boil-tank DS18B20 bus |
| HX711 DOUT | — | PH2, package pin 14 | Load-cell data |
| HX711 SCK | — | PH7, package pin 27 | Load-cell clock |
| `I_MASH_PUMP` | `A0` | PF0 / ADC0, package pin 97 | Pump-1/mash current sense |
| `I_BOIL_PUMP` | `A1` | PF1 / ADC1, package pin 96 | Pump-2/boil current sense |
| `I_VALVES` | `A4` | PF4 / ADC4, package pin 93 | Valve/servo current sense |
| `AC_MEAS` | `A8` | PK0 / ADC8, package pin 89 | B20+: total-AC-current input; unused by original B20 |
| `BOARD_TEMP` | `A9` | PK1 / ADC9, package pin 88 | Current board-temperature input |
| `I_INLET1` | `A10` | PK2 / ADC10 | Inlet diagnostic input |
| `I_INLET2` | `A11` | PK3 / ADC11 | Inlet diagnostic input |

The two pump-current values in the status record are emitted as:

```text
mash/pump-1 diagnostic, mash/pump-1 current,
boil/pump-2 diagnostic, boil/pump-2 current
```

These are ADC-derived values, not calibrated amperes. The original B20 hex
was checked and samples only ADC0/A0 and ADC1/A1 for the pump-current paths;
it does not use ADC8/PK0. Consequently, the inherited B20+ total-AC-current
check and `E210` protection are compiled out for B20. B20+ retains the
original `AC_MEAS` behavior, with limits of 20 A when `machineVoltage` is not
240 and 12 A when it is 240.

The hardware has also been traced with a chassis DS18B20 on PF2. The current
source still defines `BOARD_TEMP` as `A9/PK1`; this hardware observation and
the firmware implementation should be reconciled before treating that path
as final.

## B20 valve and actuator commands

Valve 9 is unused on the B20 harness. Outlet and cooling are mapped to PJ4
and PJ5 respectively.

| Command | Function |
|---|---|
| `P110/P111` | Water inlet open/close |
| `P112/P113` | Mash inlet open/close |
| `P114/P115` | Boil inlet open/close |
| `P116/P117` through `P122/P123` | Hop cages 1 through 4 |
| `P124/P125` | Mash pump start/stop |
| `P126/P127` | Boil pump start/stop |
| `P128/P129` | Cool inlet open/close |
| `P130/P131` | Cool valve open/close, PJ5 |
| `P132/P133` | Outlet valve open/close, PJ4 |
| `P134/P135` | Mash return open/close |
| `P136/P137` | Boil return open/close |
| `P150/P151` | Mash/boil heater targets |
| `P999` | Close all valves |

## B20 versus B20+

- B20 uses HX711 load cells on PH2/PH7; B20+ uses the pressure-sensor path.
- B20 uses the B20 pump, valve, inlet, fan, heater, LED, and power mappings
  above; B20+ retains the definitions in `B20Plus.h`.
- B20 `P128/P129` directly operate the cooling inlet. Automatic B20+ cooling
  is disabled for B20.
- B20 uses PJ2 for valve-converter power instead of the B20+ servo-enable
  signal.
- B20 heater outputs are ordinary digital outputs; B20+ uses its differing
  port-register heater definitions.

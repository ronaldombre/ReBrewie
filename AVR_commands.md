# Brewie AVR Commands

This document describes the serial protocol implemented by the current
ReBrewie firmware. Commands are ASCII payloads sent over `/dev/ttyS1` at
115200 baud, 8N1.

## Serial framing

Host-to-AVR frames use this binary layout:

```text
$ <sequence> <payload-length> <ASCII payload> <checksum/reserved byte> *
```

`payload-length` covers only the ASCII payload. Stock B20 firmware expects a
CRC-8 byte calculated over the payload with polynomial `0x5e`. ReBrewie keeps
the byte position for compatibility but does not validate it; BrewieNext sends
ASCII space (`0x20`). Bytes after `*`, including CR/LF, are ignored.

Accepted commands are acknowledged as:

```text
$ 0x01 <sequence> * CR LF
```

The AVR also emits tab-separated status records approximately once per second.

## Direct and control commands

| Command | Function | Arguments |
|---|---|---|
| `P80` | Initialize calibration and power on | `toLiter toLiterNull mashDelta boilDelta [boilingPoint]` |
| `P103` | Buffer the next recipe step | 21 numeric fields; see below |
| `P110` / `P111` | Open / close water inlet | None |
| `P112` / `P113` | Open / close mash inlet valve | None |
| `P114` / `P115` | Open / close boil inlet valve | None |
| `P116` / `P117` | Open / close hop cage 1 | None |
| `P118` / `P119` | Open / close hop cage 2 | None |
| `P120` / `P121` | Open / close hop cage 3 | None |
| `P122` / `P123` | Open / close hop cage 4 | None |
| `P124` / `P125` | Start / stop mash pump | None; start selects full scale (`255`) |
| `P126` / `P127` | Start / stop boil pump | None; start selects full scale (`255`) |
| `P128` | Start automatic cooling to a target | Temperature in tenths of °C, for example `320` for 32.0 °C |
| `P129` | Stop automatic cooling and return to heating mode | None |
| `P130` / `P131` | Open / close wort cooling valve | None |
| `P132` / `P133` | Open / close outlet valve | None |
| `P134` / `P135` | Open / close mash return valve | None |
| `P136` / `P137` | Open / close boil return valve | None |
| `P150` | Set mash-heater target | Temperature in tenths of °C; `0` disables it |
| `P151` | Set boil-heater target | Temperature in tenths of °C; `0` disables it |
| `P200` | Start buffered recipe execution | None |
| `P201` | Pause execution | None; closes valves and disables pumps and heaters |
| `P202` | Continue execution | None; resumes the current step |
| `P204` | Request the next step | None |
| `P205` | Enable / disable developer mode and fast water readings | `1` enables; `0` disables |
| `P998` | Control drain indicator | `1` on; `0` off |
| `P999` | Safe reset | No argument; see below |

Targets are parsed as decimal numbers and divided by ten. Consequently,
`P150 398.2` represents 39.82 °C, although integer tenths are conventional.

## Initialization (`P80`)

`P80` loads the load-cell scale and zero, mash and boil temperature offsets,
and optional boiling point. A missing boiling point—or one below 86 °C—defaults
to 100 °C. The command then powers on and initializes the controller.

The legacy Linux application repeatedly sent `P80` until the AVR acknowledged
initialization. Its values came from `/usr/share/brewie/config.json`.

## Buffered recipe step (`P103`)

The AVR executes one step while retaining one next-step buffer. Sending another
`P103` while that buffer is occupied reports `E300` and pauses execution.

| Offset | Firmware symbol | Meaning |
|---:|---|---|
| 0 | — | Step number, starting at `0` |
| 1 | `STEP_WATER_INLET` | Water inlet: `0` closed, `1` open |
| 2 | `STEP_MASH_INLET` | Mash inlet valve |
| 3 | `STEP_BOIL_INLET` | Boil inlet valve |
| 4 | `STEP_MASH_TEMP` | Mash target in tenths of °C; `0` disables heating |
| 5 | `STEP_BOIL_TEMP` | Boil target in tenths of °C; `0` disables heating |
| 6–9 | `STEP_HOP_1` … `STEP_HOP_4` | Hop-cage valve states |
| 10 | `STEP_COOL_VALVE` | Wort cooling-valve state |
| 11 | `STEP_VALVE_11` | Reserved; currently unused |
| 12 | `STEP_OUTLET` | Outlet-valve state |
| 13 | `STEP_MASH_PUMP` | Mash-pump command, `0`–`255` |
| 14 | `STEP_BOIL_PUMP` | Boil-pump command, `0`–`255` |
| 15 | `STEP_WATER` | Fill quantity in tenths of a liter |
| 16 | `STEP_TIME` | Step duration in seconds |
| 17 | `STEP_PRIMARY` | Primary completion mode |
| 18 | `STEP_SECONDARY` | Secondary behavior mode |
| 19 | `STEP_MASH_RETURN` | Mash return-valve state |
| 20 | `STEP_BOIL_RETURN` | Boil return-valve state |

Primary modes currently implemented are: `1` fill to volume, `2` reach mash
temperature, `3` reach boil temperature, `4` run mash pump until dry, `5` run
boil pump until dry, `6` run for time, `7` hard boil, `8` final/prompt step,
and `10` cooling/unclogging.

Secondary modes include `2` sparging/water calibration, `3` hop addition,
`4` unrestricted water inlet, `5` initial boil fill, `6` sparge fill/timed
operation, and `8` the final unclogging step. Other values are reserved or not
fully understood.

## Safe reset (`P999`)

`P999` without arguments closes every valve and both inlets, disables both
heaters and pumps, clears the active and buffered steps, and resets execution
timers and state. This is the BrewieNext backend's safe-start command.

The firmware's `P999` path with an argument invokes the broader power-off
sequence; it should not be used as an ordinary actuator command.

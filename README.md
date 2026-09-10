# ReBrewie AVR firmware

Firmware for the ATmega2560 controller used in Brewie brewing systems. The
firmware controls sensors, heaters, pumps, valves, fans, LEDs, and the serial
protocol used by the Linux application.

The Linux operating system and graphical application are separate components;
this repository contains the AVR firmware only.

## Hardware profiles

The hardware profile is selected in `ReBrewie/HardwareConfig.h`:

```cpp
#define BREWIE_HARDWARE_B20 1   // original B20
// #define BREWIE_HARDWARE_B20 0 // B20+
```

This flag selects the complete hardware interface, including pin assignments,
sensor interfaces, actuator mappings, current channels, and cooling behavior.
It is not safe to flash a binary built for one controller variant onto the
other variant.

Detailed hardware references:

- [B20hardware.md](B20hardware.md)
- [B20+hardware.md](B20+hardware.md)
- [AVR_commands.md](../AVR_commands.md)

The current B20 profile uses HX711 load-cell measurement. The B20+ profile
uses the pressure-sensor interface and retains the B20+ cooling behavior.

Bring-up diagnostics and automatic power-on are disabled by default. They are
controlled by the `BREWIE_DIAGNOSTICS` and `BREWIE_AUTO_POWER_ON` definitions
in `ReBrewie.ino`. `BREWIE_CLOSE_VALVES_ON_BOOT` is enabled by default and
actively homes every valve to its closed position after an AVR reset. This
safe startup pass does not power on the controller or enter the AC/E210 path.

## Firmware architecture

The firmware communicates with the Linux application over `/dev/ttyS1` at
115200 baud, 8 data bits, no parity, and 1 stop bit. It receives framed
commands, maintains the brewing state machine, samples sensors, controls
actuators, and periodically emits tab-separated status records.

The main implementation is in `ReBrewie/ReBrewie.ino`; hardware and actuator
subsystems are implemented in the adjacent `.cpp` and `.h` files.

## Build requirements

- Arduino AVR toolchain or Arduino IDE
- Arduino Mega 2560 board support
- OneWire library
- DallasTemperature library

The target is:

```text
Arduino Mega or Mega 2560
ATmega2560
```

### Arduino IDE

Open `ReBrewie/ReBrewie.ino`, select the target above, and use **Sketch →
Export Compiled Binary**. The application binary will be generated as an
`.ino.mega.hex` file.

### Command-line build

With Buildroot's Arduino toolchain installed, an example command is:

```sh
arduino-builder -compile -logger=plain \
  -hardware /usr/share/arduino/hardware \
  -tools /usr/share/arduino/tools \
  -built-in-libraries /usr/share/arduino/hardware/arduino/avr/libraries \
  -libraries "$HOME/Arduino/libraries" \
  -fqbn arduino:avr:mega:cpu=atmega2560 \
  -build-path /tmp/rebrewie-build \
  ReBrewie/ReBrewie.ino
```

The resulting file is usually:

```text
/tmp/rebrewie-build/ReBrewie.ino.hex
```

## Flashing through the Brewie Linux system

The Linux system communicates with the AVR bootloader through `/dev/ttyS1`.
The repository's Linux-side upload helper is `brewie-upload-fw`; the exact
path depends on the Linux image being used.

Typical workflow:

```sh
scp ReBrewie.ino.hex root@brewie:/tmp/
ssh root@brewie
killall BrewieApplication
brewie-upload-fw /tmp/ReBrewie.ino.hex
reboot
```

The uploader resets the AVR and starts `avrdude` with the wiring programmer.
The reset timing and polarity are hardware-specific; use the B20-specific
upload helper and configuration supplied with the target Linux image.

An in-circuit programmer connected to the AVR ISP header can be used when the
serial bootloader is unavailable.

## Protocol

Commands and status fields are documented in [AVR_commands.md](../AVR_commands.md).
Important actuator commands include:

```text
P124/P125   mash pump on/off
P126/P127   boil pump on/off
P150/P151   mash/boil heater targets
P999        close all valves
```

The B20 cooling commands are intentionally different from the B20+ automatic
cooling behavior; see the hardware references for details.

## Safety

This firmware directly controls mains-powered heaters, pumps, valves, and
fans. Validate the selected hardware profile, pin map, sensor readings,
actuator polarity, and fail-safe behavior on the target hardware before
operating a machine unattended.

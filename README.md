# ReBrewie
Custom, Open-Source firmware for the Brewie B20+ (and maybe B20 eventually)

# Intro
I was planning on releasing this once it was a bit more buttoned-up, but seeing as how I haven't been able to do that I figured why not release it and maybe other 
people can help, or it can help someone else.

This firmware, for the most part, is fully compatible with the original stock Brewie software, but has been improved upon to allow more advanced features that were built into the B20+. Probably for lack of engineering time and for easy of code maintenance, Brewie chose to stick to the B20 firmware and not use the new features because they wouldn't be compatible. 

# Build instructions

## Encouragement and Disclaimer
The Brewie's touch display and wireless communication are controlled by a very small computer that runs Linux as an Operating System. This is why you can login to the machine. The operating system, and consequently all the graphical user interface, are not addressed by this update. Connected to the Linux system is a small chip that is a microcontroller. It receives its own code, i.e. the firmware. The microcontroller then knows how (at what I/O port), e.g., to control valves. The microcontroller also takes action over time, e.g. it knows how steer the heating to keep temperaturs while brewing, and knows how to determine the temperature in the first place, and how to communicate its sensors to the GUI. The order in which the microcontroller executes its activities is determined by the GUI and we call this a "Beer Brewing Recipe". The more expressive or detailed the firmware becomes in the activities it offers, the better the GUI (and us users) can express its culinary desires.

The firmware is built with Arduino. That is a very common platform that you may already have heard about. Kids from 12 years onwards can program that. There are plenty of tutorials out there and small Arduino kits with to learn cost less than grain you put into the Brewie for a brew. One may be tempted to think that one cannot break anything of value while tinkering with this firmware. Wrong. Any stupid error you make (or someone else made in the developer branch) may well ruin your Brewie's hardware or even your home when running unsupervised, e.g. by not closing your valve when letting in the water. Hence, by all means, if you decide to try rebuilding this, do so on your own risk. It also helps to have a more experienced person nearby with whom to share some beers (preferbly after the coding) as an extra pair of eyeballs.

As already expressed in the MIT License this code is shipping with, do not blame any other developer of this software for time you wasted or any mayhem happening on your end.

# Building and Flashing
The Arduino environment is most easily used via the graphical user interface from your regular Desktop. To retrieve the code of this repository, run
```
git clone https://github.com/ronaldombre/ReBrewie
```
To install Arduino please google for respective instructions for your Operating System. On Debian or Ubuntu Linux run
```
sudo apt update
sudo apt install arduino
```
Then start the environment with
```
arduino
```
There is a chance that you are requested to connect to a "dialout group" but since the device is not attached to your desktop but to the Brewie, you can ignore that. We will see later how to flash to the Brewie what we are compiling on the Desktop.

When the Arduino interface opens, in the File menu find "Open..." and navigate to the directory you checked out and in the subdirectory ReBrewie find the file ReBrewie.ino. In the "Tools" menu find the item "Board" and select "Arduino Mega or Mega 2560". The item below should then with it change (or manually be changed) to "ATmega 2560". Also in the "Tools" menu, select "Manage Libraries..." and install the "OneWire" and "DallasTemperature" libraries (which provides the respective cognate header files).

In the "Sketch" menu select "Export Binary", which will compile all source code and yield the files
```
ReBrewie.ino.with_bootloader.mega.hex
ReBrewie.ino.mega.hex
```
next to the ReBrewie.ino file.

If at some point you cannot log in to the Brewie, you can connect an in-circuit programmer and use the 6-pin connector to the Brewie PCB. But we happily use the software solution, instead.

Follow these steps:
 1. Make sure you have the Brewie switched on and connected to your local Wifi network. Consider assigning a permanent IP address on your router for the Brewie, so you can address it by name (brewie for me). Otherwise use the IP number, like 192.168.178.57, which will be different in your setup. The gateway will know. Later versions of the GUI will also present the local IP number.
 2. Copy ReBrewie.ino.mega.hex to the brewie: `scp ReBrewie.ino.mega.hex root@brewie:/tmp/`
 3. Log in to your brewie: `ssh root@brewie`, the default password is `terminat`
 3. Kill the GUI, aka the Brewie app: `killall BrewieApplication`.
 4. Upload the firmware to the microcontroller: `brewie-upload-fw /tmp/ReBrewie.ino.mega.hex` B20+ users will need to run it two or three times to be successful, or modify the upload script.
 5. Restart the application `BrewieApplication` or `reboot`, just type the respective commands.

Caveat: Doing this on a B20+ requires a change in the script so that the reset pin polarity is flipped. There is a define statement for before and after that can be changed.

# Request for Help
If someone has the time to try to port this over to the B20, that would be amazing. The main thing to figure out would be the pump control, since Brewie used valve controls to slow down the pump for hopping. Maybe PWM or some other sort of valve control would be suitable. The other piece would be the weight sensor, but all that would be required is to figure out where the data is and the rest should fit into the current flow.


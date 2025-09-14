# JBC Nano Solder
DIY controller for JBC Nano (NT115) hand piece and C115 tips.

This project can be found on Hackaday: https://hackaday.io/project/175580-diy-jbc-nano-controller

JBC makes awesome high-end soldering irons for professional use. The stations are quite expensive (especially the Nano system), however the tips and hand pieces are reasonably priced for hobby users. Therefore building controllers for these became kind of a sport among DIYers.

## State of the project
This project is in a working state, but is not actively worked on and there are a few unsolved issues, that can lead to the tip overheating if things go wrong. The issues stem from using a low-side gate driver to drive a P-channel FET on the high side. This causes the default state to be **ON**, which in turn means, the heater is powered when there is no signal telling it otherwise. This happens, if the CPU is in RESET or has no power.
Make sure to always disconnect the iron when flashing the firmware, debugging or making changes to the hardware.
**As a mitigation, consider adding a pull-up resistor to the gate driver input.**

## Why Nano and not T245/T210 as everyone else?
The JBC NT115 soldering iron is amazing, it's quite compact, has a slim cable and the grip-to-tip distance is really small. It's meant for super fine-pitched SMT parts, yet if you combine it with a larger tip, its powerful enough for most soldering needs. So, despite being a high precision tool, it is still use-able as a general purpose soldering iron.

Also, there is no real point in DIYing a controller for the larger irons from scratch, since there are already heaps of projects on the internet.

## Differences to the T245/T210
Both have similar tips with the heater and thermocouple embedded directly in the tip.
### Plug
The most obvious difference is the plug, this immediately suggests that there are some more fundamental difference between the two irons.
### Voltage
The NT115 system is designed for a much lower voltage. If I remember correctly, NT115 uses about 9V while T245 is more 20ish. Not sure how bad too much power is, but overloading the tip by a factor of 4 doesn't sound good. For reference, the C115 heaters have a (hot) resistance of about 5 Ohms, which would result in ~16W at 9V, so that's quite close to the nominal 15W of the tips.
### Heater and thermocouple are in series
Despite the cable having three wires and the tip having three contacts, two of the contacts seem to be internally connected inside the tip. So, the only way to power the heater is through the thermocouple. This sounds odd at first, but that's just how it it...

## Status of this project
Everything works pretty well, the heat-up times are JBC-like fast and the temperature is quite stable. I don't know the exact characteristics of the thermocouple, but from tests, I assume the sensitivity is about 9uV/K. It doesn't matter that much, since the temperature you need for a job depends highly on the tip and the job anyway anyway. The hardware supports to measure the supply voltage, but I'm not using it right now.
### Under-voltage self destruct
One issue with the current hardware is a potential self-destruct scenario, if the supply voltage is too low. Since I'm using a low-side driver to drive a high-side P-Channel FET, the undervoltage lockout (about 4V) of the driver will force the FET ON when the supply voltage is too low. This may cause the tip or the mosfet to overheat. So, be careful with the choice of power supply.

## Building the firmware
To build the firmware, you will need the ARM toolchain on your machine. I have only tested the build on Linux, so not sure how this would work on other OSes.
### Clone repo and submodules
```
git clone https://github.com/Winkelkatze/jbc_nano_solder.git
cd jbc_nano_solder
git submodule init
git submodule update
```
### Build libopencm3
```
cd firmware/libopencm3
make
```
### Build the firmware
```
cd firmware
make
```
### Flashing the firmware
You can either use your preferred tool to flash the .elf manually or, if you have openocd installed and the controller connected with a STLINK, the firmware can be flashed with
```
make flash
```

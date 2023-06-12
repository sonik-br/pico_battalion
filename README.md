# pico_battalion
Steel Battalion Controller USB adapter using a Pi Pico RP2040

Adapter outputs as USB HID.

The adapter is very simple to make.
Just requires soldering an xbox input connector to a Pico. Or a usb connector, and use a xbox to usb dongle.

## Features
Output as HID with 8 analog axis and 62 digital buttons.

It can control the device LEDs but there's no interface for it. I might do something in the future to receive commands via serial COM port.

## Building
Check the wiring guidance [here](https://github.com/sekigon-gonnoc/Pico-PIO-USB/discussions/7).

I'm not using any resistor and I don't think it's needed.

Pins can be changed.
Define the `D+` pin on sketch. `D-` will be `D+` + 1.

Check the arduino sketch file for required libs.

## Credits
SBC [docs](https://xboxdevwiki.net/Xbox_Input_Devices#bType_.3D_128:_Steel_Battalion) by xboxdevwiki

[Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) by sekigon-gonnoc

## Disclaimer

Code and wiring directions are provided to you 'as is' and without any warranties. Use at your own risk.
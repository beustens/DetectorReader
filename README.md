# Detector Reader
Short range, low cost UHF RFID reader based on RF detector

![](docs/detector_reader.jpg)

## Features
- 0.5m...2m read range depending on antenna and tag (8dBi patch antenna recommended)
- Monostatic or bistatic configuration
- ~15$ of production cost
- Compact size of 52x25mm
- Powersupply via 5V/USB
- EPCs are reported via UART or USB (serial) interface (115200 baud)

## Flashing
The firmware is written in AVR-C for the ATtiny814. To start programming, an extended AVR toolchain is needed, which can be found in the Arduino IDE (the GNU AVR toolchain does NOT support the ATtiny814):
- Go to https://www.arduino.cc/en/software and scroll down to "legacy IDE (1.8.X)" and download the zipped version for your OS
- Unzip and add *arduino-1.8.19-linux64.tar/arduino-1.8.19-linux64/arduino-1.8.19/hardware/tools/avr/bin* (**Linux**) or *arduino-1.8.19-windows\arduino-1.8.19\hardware\tools\avr\bin* (**Windows**) to your systems PATH variable (the rest of the IDE is not needed)
- Install [Make](https://www.gnu.org/software/make/#download) if you have it not already

To transfer the firmware to the microcontroller, install [pymcuprog](https://pypi.org/project/pymcuprog/) and build yourself a UPDI programmer from a simple USB-UART converter and a 1kOhm resistor by connecting the resistor between Rx und Tx and connecting Rx to UPDI (Reset pin):
```
                        Vcc                     Vcc
                        +-+                     +-+
                         |                       |
 +---------------------+ |                       | +--------------------+
 | Serial port         +-+                       +-+  AVR device        |
 |                     |     +----------+          |                    |
 |                  TX +-----+   1k     +-----+    |                    |
 |                     |     +----------+     |    |                    |
 |                     |                      |    |                    |
 |                  RX +----------------------+----+ UPDI               |
 |                     |                           |                    |
 |                     +--+                     +--+                    |
 +---------------------+  |                     |  +--------------------+
                         +-+                   +-+
                         GND                   GND
```

Edit the PROGRAMMER variable (serial port) in the [firmware/Makefile](Makefile) and run `make flash clean` to build, flash and remove the built files.

## Notes
- In the current design, the transmitter is activated and modulated by a simple TTL signal, but the power is limited to 13dBm. It would be possible to control it via SPI and increase its power to 16dBm (and tune frequency) though
- Due to the detector-based design (no I/Q, no phase information), the detection of tags varies periodically with the distance, so it works better when the tag or reader is moving
- Have a look in the branches for alternative LF amplification and filtering of the demodulated signal
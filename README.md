# Detector Reader
Short range UHF RFID reader based on RF detector

### Flashing
The firmware is written in AVR-C for ATtiny814 (tinyAVR 1-series with UPDI). An AVR-C toolchain for compiling is, e.g., included in the Arduino IDE. 
- Get the AVR-C toolchain and modify your systems `PATH` variable to look in its *bin/* directory
- Get a USB-UART adapter, connect its RX and TX with a 1k resistor and connect RX to the UPDI pin of the board, (and 5V and GND of course)
- Install [pyupdi](https://github.com/mraardvark/pyupdi)
- In the firmware directory, edit `PROGRAMMER = /dev/tty.usbserial-A50285BI` in the *Makefile* to match your USB-UART adapter, then run `make flash clean`
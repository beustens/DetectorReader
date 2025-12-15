#include "gpio.h"

void configGPIO() {
	LED_PORT.DIRSET = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN; // LED pins as output
	LED_PORT.OUTSET = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN; // low active, so initially set high to disable

	READER_CMD_PORT.DIRSET = READER_CMD_PIN; // reader command mask pin as output
	READER_CMD_PORT.OUTSET = READER_CMD_PIN; // initially set high to pass signal from detector to LF amplifier
}
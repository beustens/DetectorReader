#include <stdint.h> // for uint8_t, uint16_t, etc.
#include <util/delay.h> // for waiting
#include <stdio.h> // for sprintf
#include <string.h>	// for string operations like cmp and cat
#include <avr/interrupt.h>	// for sei(), cli() and ISR()
#include "gpio.h"
#include "main.h"
#include "rfid.h"
#include "max41460.h"


ISR(USART0_RXC_vect) {
	char nextChar = USART0.RXDATAL; // new char available

	// track received string
	static uint8_t charCnt = 0;
	if (nextChar != STR_TERM && charCnt < UART_MAX_LEN) {
		// still getting valid chars
		readStr[charCnt++] = nextChar;
	} else {
		// end of string
		readStr[charCnt] = '\0';
		charCnt = 0;
		gotUARTCmd = TRUE;
	}
}

int main() {
	// setup
	configClock();
	configGPIO();
	configSPI();
	configTCA();
	configTCB();
	configRTC();
	configUSART();
	sei(); // enable interrupts

	LED_PORT.OUTCLR = LED_RED_PIN; // enable red LED
	startTransmitter();
	enablePower(TRUE);
	LED_PORT.OUTSET = LED_RED_PIN; // disable LED

	// main program loop
	while (TRUE) {
		// parse incoming UART command
		if (gotUARTCmd) {
			parseCommand((char*)readStr);
			gotUARTCmd = FALSE;
		}
		
		// transmit Query periodicly
		if (RTC.INTFLAGS & RTC_CMP_bm) {
			// RTC interrupt bit set
			RTC.INTFLAGS |= RTC_CMP_bm; // clear interrupt bit
			RTC.CNT = 0; // restart RTC
			runSequence(queryPulses);
		}

		// receive tag responses
		trackTagPulses();
		if (gotEPC) {
			LED_PORT.OUTCLR = LED_GREEN_PIN; // tag EPC detected indicator
			sendEPC();
			gotEPC = FALSE;
		}
	}

	return 0;
}

void sendEPC() {
	// format and send EPC via UART
	static char hexStr[3];
	for (iEPCByte = NUM_EPC_PREFIX; iEPCByte < NUM_EPC_PREFIX+NUM_EPC_BYTES; iEPCByte++) {
		// format EPC bytes as hex ascii strings
		sprintf(hexStr, "%.2x", epc[iEPCByte]);
		sendStringRaw(hexStr);
	}
	sendChar(STR_TERM);
}

void parseCommand(const char* pStr) {
	// parse command
	if (strncmp(pStr, "POW", 3) == 0) {
		// enable/disable power
		pStr += 3;
		if (*pStr == '?') {
			// host wants to know if power (carrier, unmodulated) is enabled
			sendString((powerEnable) ? "ON" : "OFF");
		} else {
			// host wants to set power output
			pStr += 2; // skip " O"
			enablePower((*pStr == 'N'));
			sendString("1"); // confirm
		}
	} else {
		// no valid command
		sendString("Invalid command");
	}
}

void configClock() {
	// configure CPU frequency to 20MHz
	CCP = CCP_IOREG_gc; // disable protection to configure clock frequency
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc; // use 20 MHz internal clock as source
	CCP = CCP_IOREG_gc; // disable protection to configure clock frequency
	CLKCTRL.MCLKCTRLB = 0; // no prescale divider
}

void configUSART() {
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(115200); // set baudrate
	USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm; // enable RX | TX
	USART0.CTRLA |= USART_RXCIF_bm; // enable interrupt on receive complete
	//PORTMUX_CTRLB |= PORTMUX_USART0_bm; // enable alternative UART pins
	//PORTA.DIRSET = PIN1_bm; // PA1 is the alternative TxD pin
	PORTB.DIRSET = PIN2_bm; // PB2 is the default TxD pin
}

void sendChar(const char c) {
	while (!(USART0.STATUS & USART_DREIF_bm)); // wait until the transmit DATA register is empty
	USART0.TXDATAL = c; // put char into buffer
}

void sendString(const char* pStr) {
	// for all characters in string until STR_TERM
	while(*pStr && (*pStr != STR_TERM)) {
		sendChar(*pStr++);
	}
	// append string termination
	sendChar(STR_TERM);
}

void sendStringRaw(const char* pStr) {
	// for all characters in string
	while(*pStr) {
		sendChar(*pStr++);
	}
}

void configRTC() {
	// config realtime clock
	RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // use internal 32.768kHz clock
	RTC.CMP = 200; // compare to this value
	RTC.INTCTRL = RTC_CMP_bm; // enable interrupt on compare
	RTC.CTRLA = RTC_RTCEN_bm; // enable RTC
}

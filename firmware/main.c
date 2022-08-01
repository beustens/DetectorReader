#include <avr/io.h> // for microcontroller specific peripheral definitions
#include <stdint.h> // for uint8_t, uint16_t, etc.
#include <string.h>	// for string operations like cmp and cat
#include <stdio.h> // for sprintf
#include <avr/interrupt.h>	// for sei(), cli() and ISR()
#include <util/delay.h> // for waiting

// Command pulses output: PB2
// Command pulses progress: PB3
// Tag pulses input: PA4
// Tag detected indicator output: PA7
// USART (alternative pin) RX: PA2
// USART (alternative pin) TX: PA1

// definitions
// general
#define FALSE 0
#define TRUE 1
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU*64/(16*(float)BAUD_RATE))+0.5)
#define UART_MAX_LEN 16
#define STR_TERM '\n' // USART receive and transmit termination
// reader
#define PULSE_TERM 0 // pulse sequence termination
#define RS 10 // reader short pulse length in us
#define RL (3*RS) // reader long pulse length in us
#define FRAME_SYNC 12, RS, RS, (2*RS+RL), RS // reader frame sync pulse lenght in us
#define TR_CAL 150, RS // reader TRcal pulse length in us
#define RD0 RS, RS // reader data-0 pulse length
#define RD1 RL, RS // reader data-1 pulse length
#define ACK_CMD RD0, RD1 // reader ACK command bit (01) pulses in us
#define NUM_ACK_PREFIX 9 // pulses in ACK command before RN16 pulses
#define NUM_ACK_FULL 41 // pulses in ACK command after RN16 pulses
// tag
#define NUM_PREAMBLE 6 // tag preamble pulses - 1 because first not detected
#define TS 100 // tag FM0 short pulse (2x for 0-bit) length in us * 10 for 50kHz BLF
#define TL (2*TS) // tag FM0 long pulse (1x for 1-bit) length in us * 10 for 50kHz BLF
#define TAG_TOLERANCE 30 // tag pulse length tolerance in us * 10
#define RESP_DELAY 5*TL/10 // 3...20 * FM0 symbol period in us for reader to wait before react to tag response
#define NUM_EPC_PREFIX 2 // PC word (and actually CRC, why not 4?)
#define NUM_EPC_BYTES 12 // EPC bytes

// enumerations
enum tagReplies {RN16, EPC};
enum tagReplies tagResponse = RN16;

// variables
volatile char readStr[UART_MAX_LEN] = "";
volatile uint8_t gotUARTCmd = FALSE;
uint8_t powerEnable = FALSE;
const uint8_t queryPulses[] = {FRAME_SYNC, TR_CAL, RD1, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD1, RD0, RD1, RD0, RD0, RD0, RD0, RD0, RD1, RD1, RD1, RD0, RD1, PULSE_TERM}; // QUERY command with FM0, DR=8, no preamble, 20us Tari, 50kHz BLF
uint8_t ackPulses[] = {FRAME_SYNC, ACK_CMD, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, PULSE_TERM}; // ACK command with RN 0b0000000000000000
const uint16_t preamblePulses[NUM_PREAMBLE] = {TS, TS, TL, TS, TS+TL, TL}; // FM0 preamble without first pulse. 50kHz BLF * 10
uint8_t iPreamblePulse = 0;
uint8_t iACKPulse = 0;
uint8_t iTagData = 0;
uint8_t halfTagData0 = FALSE; // tracks tag data0 (consists of 2 pulses)
uint8_t epc[NUM_EPC_PREFIX+NUM_EPC_BYTES];
uint8_t iEPCBit = 0;
uint8_t iEPCByte = 0;

// prototypes
void configGPIO();
void configClock();
void configTCA();
void configTCB();
void configRTC();
void configUSART();
void sendChar(char c);
void sendString(const char* pStr);
void sendStringRaw(const char* pStr);
void parseCommand(const char* pStr);
void enablePower(uint8_t enable);
void runSequence(const uint8_t* pPulses);
uint8_t pulseMatch(uint16_t pulseLen1, uint16_t pulseLen2);
void resetParsing();
void modifyACK(uint8_t pulseWidth);
void modifyEPC(uint8_t bitVal);

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
	configTCA();
	configTCB();
	configRTC();
	configUSART();
	sei(); // enable interrupts
	
	// transmitter wake-up sequence
	enablePower(TRUE);
	_delay_ms(1);
	enablePower(FALSE);
	_delay_ms(1);
	
	// ensure carrier is active
	enablePower(TRUE);
	
	// main program loop
	while (TRUE) {
		// parse incoming
		if (gotUARTCmd) {
			parseCommand((char*)readStr);
			gotUARTCmd = FALSE;
		}
		
		// transmit query
		if ((RTC.INTFLAGS & RTC_CMP_bm) && (iPreamblePulse == 0)) {
			// RTC interrupt bit set
			RTC.INTFLAGS |= RTC_CMP_bm; // clear interrupt bit
			RTC.CNT = 0; // restart RTC
			runSequence(queryPulses);
		}
		
		// track tag pulses
		if (TCB0.INTFLAGS & TCB_CAPT_bm) {
			// poll interrupt flag
			uint16_t tagPulse = TCB0.CCMP; // get pulse length and clear interrupt flag
			TCB0.EVCTRL ^= TCB_EDGE_bm; // toggle edge to trigger next
			TCB0.CNT = 0; // reset counter
			
			if (iPreamblePulse < NUM_PREAMBLE) {
				// track preamble
				if (pulseMatch(tagPulse, preamblePulses[iPreamblePulse])) {
					// pulse in sequence matched
					iPreamblePulse++; // point to next element
					if (iPreamblePulse >= NUM_PREAMBLE) {
						// preamble finished
						PORTA.OUTSET = PIN7_bm; // tag detected indicator
						if (tagResponse == RN16) {
							iACKPulse = NUM_ACK_PREFIX;
						} else if (tagResponse == EPC) {
							/*
							_delay_ms(100); // for perceptual brighter indicator
							resetParsing();
							tagResponse = RN16;
							*/
							iEPCBit = 7;
							iEPCByte = 0;
							memset(epc, 0, NUM_EPC_PREFIX+NUM_EPC_BYTES);
						}
					}
				} else {
					// wrong sequence
					resetParsing();
				}
			} else {
				// track payload
				if (pulseMatch(tagPulse, TS)) {
					// half of tag data-0
					if (halfTagData0) {
						// second part of data-0
						halfTagData0 = FALSE;
						if (tagResponse == RN16) {
							modifyACK(RS);
						} else if (tagResponse == EPC) {
							modifyEPC(0);
						}
					} else {
						// first part of data-0
						halfTagData0 = TRUE;
					}
				} else if (pulseMatch(tagPulse, TL)) {
					// tag data-1
					halfTagData0 = FALSE;
					if (tagResponse == RN16) {
						modifyACK(RL);
					} else if (tagResponse == EPC) {
						modifyEPC(1);
					}
				} else {
					// invalid pulse
					resetParsing();
					tagResponse = RN16;
				}
			}
		}
	}
	return 0;
}

void resetParsing() {
	iPreamblePulse = 0;
	halfTagData0 = FALSE;
	PORTA.OUTCLR = PIN7_bm; // clear tag detected indicator
}

void modifyACK(uint8_t pulseWidth) {
	ackPulses[iACKPulse] = pulseWidth;
	iACKPulse += 2;
	if (iACKPulse >= NUM_ACK_FULL) {
		// reached end
		resetParsing();
		tagResponse = EPC; // next state
		_delay_us(RESP_DELAY);
		runSequence(ackPulses); // acknowledge tag to make it reply with EPC
	}
}

void modifyEPC(uint8_t bitVal) {
	epc[iEPCByte] |= (bitVal << iEPCBit);
	if (iEPCBit == 0) {
		iEPCBit = 7;
		iEPCByte++;
		if (iEPCByte >= NUM_EPC_PREFIX+NUM_EPC_BYTES) {
			// reached end
			static char hexStr[3];
			for (iEPCByte = NUM_EPC_PREFIX; iEPCByte < NUM_EPC_PREFIX+NUM_EPC_BYTES; iEPCByte++) {
				// format EPC bytes as hex ascii strings
				sprintf(hexStr, "%.2x", epc[iEPCByte]);
				sendStringRaw(hexStr);
			}
			sendChar(STR_TERM);
			//_delay_ms(100); // for perceptual brighter indicator
			resetParsing();
			tagResponse = RN16;
		}
	} else {
		iEPCBit--;
	}
}

inline void parseCommand(const char* pStr) {
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

void enablePower(uint8_t enable) {
	// enables or disables power
	if (enable) {
		PORTB.OUTSET = PIN2_bm; // enable
		powerEnable = TRUE;
	} else {
		PORTB.OUTCLR = PIN2_bm; // disable
		powerEnable = FALSE;
	}
}

void runSequence(const uint8_t* pPulses) {
	// indication
	PORTB.OUTSET = PIN3_bm;

	// starts the sequence
	TCA0.SINGLE.CNT = 0; // reset count
	uint8_t pulse = *pPulses++; // read first pulse length
	TCA0.SINGLE.PER = pulse*10; // set TOP
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // enable timer
	PORTB.OUTCLR = PIN2_bm; // first pulse is low level
	while (TRUE) {
		if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) {
			TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm; // clear flag
			PORTB.OUTTGL = PIN2_bm; // toggle output
			uint8_t pulse = *pPulses++; // read next pulse length
			if (pulse) {
				TCA0.SINGLE.PER = pulse*10; // set next TOP
			} else {
				break; // sequence end
			}
		}
	}
	// finish process
	if (powerEnable) PORTB.OUTSET = PIN2_bm; // enforce high level
	TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; // disable timer

	// indication
	PORTB.OUTCLR = PIN3_bm;
}

uint8_t pulseMatch(uint16_t pulseLen1, uint16_t pulseLen2) {
	// compares two pulse lengths wether they match within tolerance
	return ((pulseLen1 < pulseLen2+TAG_TOLERANCE) && (pulseLen1 > pulseLen2-TAG_TOLERANCE)) ? TRUE : FALSE;
}

void configGPIO() {
	PORTB.DIRSET = PIN3_bm | PIN2_bm; // PB2 and PB3 as output
	PORTA.DIRCLR = PIN4_bm; // PA4 as input
	PORTA.PIN4CTRL = PORT_PULLUPEN_bm; // enable pullup resistor for PA4
	PORTA.DIRSET = PIN7_bm; // PA7 as output
	PORTB.PIN3CTRL |= PORT_INVEN_bm; // invert cmd progress output to simplify circuit
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
	// use alternative UART pins
	PORTMUX_CTRLB |= PORTMUX_USART0_bm; // enable alternative UART pins
	PORTA.DIRCLR = PIN2_bm; // PA2 as input (RX)
	PORTA.DIRSET = PIN1_bm; // PA1 as output (TX)
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

void configTCA() {
	// set normal mode (default)
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; // enable interrupt on overflow
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc; // count with prescale 2
	//TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // enable timer
}

void configTCB() {
	// config event
	EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;
	EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_PORTA_PIN4_gc; // PA4 as event trigger
	// config pulse with capture
	TCB0.CTRLB = TCB_CNTMODE_PW_gc; // set timer to pulse width measurement mode
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc; // prescale timer clock by 2
	TCB0.EVCTRL = TCB_CAPTEI_bm; // enable input event
	TCB0.INTCTRL = TCB_CAPT_bm; // enable the capture interrupt
	TCB0.CTRLA |= TCB_ENABLE_bm; // enable timer
}

void configRTC() {
	// config realtime clock
	RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // use internal 32.768kHz clock
	RTC.CMP = 100; // compare to this value
	RTC.INTCTRL = RTC_CMP_bm; // enable interrupt on compare
	RTC.CTRLA = RTC_RTCEN_bm; // enable RTC
}
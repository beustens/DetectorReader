#include <util/delay.h> // for waiting
#include "gpio.h"
#include "rfid.h"


// variables
enum tagReplies tagResponse = RN16;
uint8_t powerEnable = FALSE;
const uint8_t queryPulses[] = {FRAME_SYNC, TR_CAL, RD1, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD1, RD0, RD1, RD0, RD0, RD0, RD0, RD0, RD1, RD1, RD1, RD0, RD1, PULSE_TERM}; // QUERY command with FM0, DR=8, no preamble, 20us Tari, 50kHz BLF
uint8_t ackPulses[] = {FRAME_SYNC, ACK_CMD, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, RD0, PULSE_TERM}; // ACK command with RN 0b0000000000000000
const uint16_t preamblePulses[NUM_PREAMBLE] = {TS, TS, TL, TS, TS+TL, TL}; // FM0 preamble without first pulse. 50kHz BLF * 10
uint8_t iPreamblePulse = 0;
uint8_t iACKPulse = 0;
uint8_t iTagData = 0;
uint8_t halfTagData0 = FALSE; // tracks tag data0 (consists of 2 pulses)
uint8_t iEPCBit = 0;
uint8_t iEPCByte = 0;
uint8_t gotEPC = FALSE;


void resetTracking() {
	iPreamblePulse = 0;
	halfTagData0 = FALSE;
	LED_PORT.OUTSET = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN; // clear tag indicators
}

void trackTagPulses() {
	if (!(TCB0.INTFLAGS & TCB_CAPT_bm)) return; // poll pulse capture interrupt flag
	
	uint16_t tagPulse = TCB0.CCMP; // get pulse length and clear interrupt flag
	TCB0.EVCTRL ^= TCB_EDGE_bm; // toggle edge to trigger next
	TCB0.CNT = 0; // reset counter
	
	if (iPreamblePulse < NUM_PREAMBLE) {
		// track preamble for any tag reply
		if (pulseMatch(tagPulse, preamblePulses[iPreamblePulse])) {
			// pulse in sequence matched
			iPreamblePulse++; // point to next element
			if (iPreamblePulse >= NUM_PREAMBLE) {
				// preamble finished
				LED_PORT.OUTCLR = LED_BLUE_PIN; // set tag preamble detected indicator
				if (tagResponse == RN16) {
					iACKPulse = NUM_ACK_PREFIX;
				} else if (tagResponse == EPC) {
					iEPCBit = 7;
					iEPCByte = 0;
					memset(epc, 0, NUM_EPC_PREFIX+NUM_EPC_BYTES);
				}
			}
		} else {
			// wrong sequence
			LED_PORT.OUTCLR = LED_RED_PIN; // set error indicator
			_delay_ms(5);

			resetTracking();
		}
	} else {
		// track payload
		if (pulseMatch(tagPulse, TS)) {
			// half of tag data-0
			if (halfTagData0) {
				// second part of data-0
				halfTagData0 = FALSE;
				if (tagResponse == RN16) {
					trackACK(RS);
				} else if (tagResponse == EPC) {
					trackEPC(0);
				}
			} else {
				// first part of data-0
				halfTagData0 = TRUE;
			}
		} else if (pulseMatch(tagPulse, TL)) {
			// tag data-1
			halfTagData0 = FALSE;
			if (tagResponse == RN16) {
				trackACK(RL);
			} else if (tagResponse == EPC) {
				trackEPC(1);
			}
		} else {
			// invalid pulse
			LED_PORT.OUTCLR = LED_RED_PIN; // set error indicator
			_delay_ms(5);

			resetTracking();
			tagResponse = RN16;
		}
	}
}

void trackACK(uint8_t pulseWidth) {
	ackPulses[iACKPulse] = pulseWidth;
	iACKPulse += 2;
	if (iACKPulse >= NUM_ACK_FULL) {
		// reached end
		resetTracking();
		tagResponse = EPC; // next state
		_delay_us(RESP_DELAY);
		RTC.CNT = 0; // restart RTC
		runSequence(ackPulses); // acknowledge tag to make it reply with EPC
	}
}

void trackEPC(uint8_t bitVal) {
	epc[iEPCByte] |= (bitVal << iEPCBit);
	if (iEPCBit == 0) {
		iEPCBit = 7;
		iEPCByte++;
		if (iEPCByte >= NUM_EPC_PREFIX+NUM_EPC_BYTES) {
			// reached end
			gotEPC = TRUE;
			// reset
			resetTracking();
			tagResponse = RN16;
			_delay_us(RESP_DELAY);
		}
	} else {
		iEPCBit--;
	}
}

void enablePower(uint8_t enable) {
	// enables or disables power
	if (enable) {
		READER_PORT.OUTSET = READER_PULSE_PIN; // enable
		powerEnable = TRUE;
	} else {
		READER_PORT.OUTCLR = READER_PULSE_PIN; // disable
		powerEnable = FALSE;
	}
}

void runSequence(const uint8_t* pPulses) {
	// mask command which disables detector signal amplification
	READER_CMD_PORT.OUTCLR = READER_CMD_PIN;

	// starts the sequence
	TCA0.SINGLE.CNT = 0; // reset count
	uint8_t pulse = *pPulses++; // read first pulse length
	TCA0.SINGLE.PER = pulse*10; // set TOP
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // enable timer
	READER_PORT.OUTCLR = READER_PULSE_PIN; // first pulse is low level
	while (TRUE) {
		if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) {
			TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm; // clear flag
			READER_PORT.OUTTGL = READER_PULSE_PIN; // toggle output
			uint8_t pulse = *pPulses++; // read next pulse length
			if (pulse) {
				TCA0.SINGLE.PER = pulse*10; // set next TOP
			} else {
				break; // sequence end
			}
		}
	}
	// finish process
	if (powerEnable) READER_PORT.OUTSET = READER_PULSE_PIN; // enforce high level
	TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; // disable timer

	// end mask
	READER_CMD_PORT.OUTSET = READER_CMD_PIN;
}

uint8_t pulseMatch(uint16_t pulseLen1, uint16_t pulseLen2) {
	// compares two pulse lengths wether they match within tolerance
	return ((pulseLen1 < pulseLen2+TAG_TOLERANCE) && (pulseLen1 > pulseLen2-TAG_TOLERANCE)) ? TRUE : FALSE;
}

void configTCA() {
	// set normal mode (default)
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; // enable interrupt on overflow
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc; // count with prescale 2
	//TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // enable timer
}

void configTCB() {
	// config event
	EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH1_gc; // pins of PORTB can be selected in ASYNCCH1
	EVSYS.ASYNCCH1 = EVSYS_ASYNCCH1_PORTB_PIN0_gc; // select PB0 as event trigger
	// config pulse with capture
	TCB0.CTRLB = TCB_CNTMODE_PW_gc; // set timer to pulse width measurement mode
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc; // prescale timer clock by 2
	TCB0.EVCTRL = TCB_CAPTEI_bm; // enable input event
	TCB0.INTCTRL = TCB_CAPT_bm; // enable the capture interrupt
	TCB0.CTRLA |= TCB_ENABLE_bm; // enable timer
}
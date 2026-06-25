#include <stdint.h> // for uint8_t, uint16_t, etc.

// general
#define FALSE 0
#define TRUE 1
// tag
#define NUM_PREAMBLE 6 // number of tag preamble pulses - 1 because first not detected
#define BLF 50 // tag backscatter frequency in kHz
#define TL (10000/BLF) // tag FM0 long pulse (1x for 1-bit) length in us * 10
#define TS (TL/2) // tag FM0 short pulse (2x for 0-bit) length in us * 10
#define TAG_TOLERANCE 40 // tag pulse length tolerance in us * 10
#define RESP_DELAY 5*TL/10 // 3...20 * FM0 symbol period in us for reader to wait before react to tag response
#define NUM_EPC_PREFIX 2 // PC word (and actually CRC, why not 4?)
#define NUM_EPC_BYTES 12 // EPC bytes
// reader
#define PULSE_TERM 0 // pulse sequence termination
#define TARI 20 // reader data-0 length in us
#define RS (TARI/2) // reader short pulse length in us
#define RL (3*RS) // reader long pulse length in us
#define RD0 RS, RS // reader data-0 pulses
#define RD1 RL, RS // reader data-1 pulses
#define RT_CAL ((RS+RS)+(RL+RS)-RS), RS // reader RTcal pulses in us. RTCal = RD0length+RD1length
#define FRAME_SYNC 12, RD0, RT_CAL // reader frame sync pulses in us
#define POPULATION (1 << 4) // tags respond on a random slot number of = 2^Q - 1 (must be same as in queryPulses!)
#define DR 8 // divide ratio (must be same as in queryPulses!)
#define TR_CAL ((1000*DR/BLF)-RS), RS // reader TRCal pulses in us. TRCal = DR/(BLF[MHz]), according to queryPulses
#define NUM_ACK_PREFIX 9 // number of pulses in ACK command before RN16 pulses
#define NUM_ACK_FULL 41 // number of pulses in ACK command after RN16 pulses

// enumerations
enum tagReplies {RN16, EPC};
extern enum tagReplies tagResponse;

// variables
extern uint8_t powerEnable;
extern const uint8_t queryPulses[]; // Query command
extern const uint8_t queryRepPulses[]; // QueryRep command
extern uint8_t ackPulses[]; // ACK command
extern const uint16_t preamblePulses[NUM_PREAMBLE]; // FM0 preamble without first pulse
extern uint8_t iPreamblePulse;
extern uint8_t iACKPulse;
extern uint8_t iTagData;
extern uint8_t halfTagData0; // tracks tag data0 (consists of 2 pulses)
extern uint8_t iEPCBit;
extern uint8_t iEPCByte;
extern uint8_t gotEPC;
uint8_t epc[NUM_EPC_PREFIX+NUM_EPC_BYTES];

// prototypes
void resetTracking();
void trackTagPulses();
void trackTagPayload(uint8_t bitVal);
void trackACK(uint8_t bitVal);
void trackEPC(uint8_t bitVal);
void enablePower(uint8_t enable);
void runSequence(const uint8_t* pPulses);
uint8_t pulseMatch(uint16_t pulseLen1, uint16_t pulseLen2);
void configTCA();
void configTCB();
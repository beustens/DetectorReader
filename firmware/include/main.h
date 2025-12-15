#include <stdint.h> // for uint8_t, uint16_t, etc.

// defines
#define FALSE 0
#define TRUE 1
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU*64/(16*(float)BAUD_RATE))+0.5)
#define UART_MAX_LEN 16
#define STR_TERM '\n' // USART receive and transmit termination

// variables
volatile char readStr[UART_MAX_LEN] = "";
volatile uint8_t gotUARTCmd = FALSE;

// prototypes
void configClock();
void configRTC();
void configSPI();
uint8_t transferSPI(uint8_t byte);
void configUSART();
void sendChar(char c);
void sendString(const char* pStr);
void sendStringRaw(const char* pStr);
void sendEPC();
void parseCommand(const char* pStr);
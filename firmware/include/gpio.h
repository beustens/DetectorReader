#include <avr/io.h> // for microcontroller specific peripheral definitions

// LEDs (low active)
#define LED_PORT            PORTA
#define LED_RED_PIN         PIN6_bm
#define LED_GREEN_PIN       PIN7_bm
#define LED_BLUE_PIN        PIN5_bm

// reader command
#define READER_PORT         PORTA
#define SPI_MOSI            PIN1_bm
#define SPI_MISO            PIN2_bm
#define SPI_SCK             PIN3_bm
#define SPI_CS              PIN4_bm
#define READER_PULSE_PIN    SPI_MOSI

#define READER_CMD_PORT     PORTB
#define READER_CMD_PIN      PIN1_bm

// tag response
// see rfid.c -> configTCB()
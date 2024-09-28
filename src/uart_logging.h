#ifndef UARTLOGH
#define UARTLOGH
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// adjustable parameters
#define UART_READBUFF_SIZE	1024
#define UART_BAUD	115200
#define UART_TX_PIN	1
#define UART_RX_PIN 2
#define FAULT_LED_PIN 3

typedef enum {
	LEVEL_DEBUG,
	LEVEL_INFO,
	LEVEL_WARN,
	LEVEL_ERROR
} LogLevel;

#define LOGLEVEL	LEVEL_DEBUG

void uart_setup();

void uart_send(char*);

void uart_log(LogLevel, char*);

bool uart_getline(char*);

void set_fault(bool);

bool get_fault();

#endif

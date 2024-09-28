#pragma ONCE
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

void uart_setup();

void uart_send(char*);

bool uart_getline(char*);

void set_fault(bool);

bool get_fault();

#endif

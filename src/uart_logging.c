#include "uart_logging.h"
#include <string.h>
#include <stdio.h>
#include <rcl/rcl.h>

void uart_setup(){
	uart_init(uart0, UART_BAUD);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
}

void uart_send(char *tosend){
	uart_puts(uart0, tosend);
}

void uart_log(LogLevel level, char *message){
	// do not process if log level of message is below set level
	if (level < LOGLEVEL){
		return;
	}
	uint32_t uptime_ms = time_us_32()/1000;
	char out[UART_DEBUG_MAXLEN];
	char *levelstr;
	switch (level){
		case LEVEL_DEBUG: {
			levelstr = "DEBUG";
			break;
		}
		case LEVEL_INFO: {
		    levelstr = "INFO";
		    break;
		}
		case LEVEL_WARN: {
		    levelstr = "WARN";
		    break;
		}
		case LEVEL_ERROR: {
		    levelstr = "ERROR";
		    break;
		}
		default: {
		    levelstr = "UNKNOWN";
		}
	}
	snprintf(out, UART_DEBUG_MAXLEN, "[%lu] %s: %s\r\n", uptime_ms, levelstr, message);
	uart_send(out);

}


/**
 * @brief  Nonblocking read checking for newline on UART
 * 
 * Polls for data on UART 
 * 
 * 
 * @param[in]  target  Pointer to destination buffer
 * 
 * @return  true if a new message has been written to target buffer, false otherwise
 */
bool uart_getline(char *target){
	static char recbuff[UART_READBUFF_SIZE];
	static int recv_idx = 0;
	if (!uart_is_readable(uart0)){
		return false;
	}
	char newchar = uart_getc(uart0);
	if (recv_idx >= UART_READBUFF_SIZE-2) {
		uart_send("UART RX FULL! Flushing...\n");
		recv_idx = 0;
		return false;
	}
	recbuff[recv_idx] = newchar;
	if (newchar == '\n'){
		// message over
		recbuff[recv_idx+1] = '\0'; // null terminate 
		memcpy(target, recbuff, recv_idx+2);
		recv_idx = 0;
		return true;
	}
	recv_idx++;
	return false;	
}

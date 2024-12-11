#include "uart_logging.h"
#include <string.h>
#include <stdio.h>
#include <rcl/rcl.h>
#define UART_COLOR

mutex_t uart_mutex;

void uart_setup(){
	uart_init(uart0, UART_BAUD);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    mutex_init(&uart_mutex);
    uart_puts(uart0, "\r\n");
}

void uart_send(char *tosend){
	uart_puts(uart0, tosend);
}

static void uart_log_internal(LogLevel level, char *message){
	// do not process if log level of message is below set level
	if (level < LOGLEVEL){
		return;
	}
	uint32_t uptime_ms = time_us_32()/1000;
	char out[UART_DEBUG_MAXLEN];
	char *levelstr;
	switch (level){
		#ifdef UART_COLOR
		case LEVEL_DEBUG: {
			levelstr = "\033[32mDEBUG\033[0m";
			break;
		}
		case LEVEL_INFO: {
		    levelstr = "\033[34mINFO\033[0m";
		    break;
		}
		case LEVEL_WARN: {
		    levelstr = "\033[33mWARN\033[0m";
		    break;
		}
		case LEVEL_ERROR: {
		    levelstr = "\033[31mERROR\033[0m";
		    break;
		}
		#else
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
		#endif
		default: {
		    levelstr = "UNKNOWN";
		}
	}
	snprintf(out, UART_DEBUG_MAXLEN, "[%lu] %s: %s\r\n", uptime_ms, levelstr, message);
	uart_send(out);

}

void uart_log(LogLevel level, char *message){
	mutex_enter_timeout_ms(&uart_mutex, UART_WRITE_TIMEOUT);
	uart_log_internal(level, message);
	mutex_exit(&uart_mutex);
}

void uart_log_nonblocking(LogLevel level, char *message){
	if(!mutex_try_enter(&uart_mutex, NULL)) {
		return;
	}
	uart_log_internal(level, message);
	mutex_exit(&uart_mutex);
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
		uart_log(LEVEL_WARN,"UART RX FULL! Flushing...");
		recv_idx = 0;
		return false;
	}
	recbuff[recv_idx] = newchar;
	if (newchar == '\r' || newchar == '\n'){
		// message over
		recbuff[recv_idx+1] = '\0'; // null terminate 
		memcpy(target, recbuff, recv_idx+2);
		recv_idx = 0;
		return true;
	}
	recv_idx++;
	return false;	
}

void rcl_check_error(rcl_ret_t retcode, const char *desc){
	char outstring[100];
	switch (retcode){
		case RCL_RET_OK:
			return;
		case RCL_RET_INVALID_ARGUMENT:
			snprintf(outstring, 100, "Invalid argument in %s call!",desc);
			break;
		case RCL_RET_PUBLISHER_INVALID:
			snprintf(outstring, 100, "%s failed with code invalid pub!",desc);
			break;
		case RCL_RET_NODE_INVALID:
			snprintf(outstring, 100, "%s failed with code invalid node!",desc);
			break;
		case RCL_RET_TIMEOUT:
			snprintf(outstring, 100, "%s timed out!",desc);
			break;
		default:
			snprintf(outstring, 100, "Operation %s returned unknown error code %d!",desc,retcode);
	}
	uart_log(LEVEL_ERROR, outstring);
}

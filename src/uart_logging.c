#include "uart_logging.h"

static bool fault = false;

void uart_setup(){
	gpio_init(FAULT_LED_PIN);
	gpio_set_dir(FAULT_LED_PIN, GPIO_OUT);
	uart_init(uart0, UART_BAUD);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
}

void uart_send(const char *tosend){
	uart_puts(uart0, tosend);
}


/**
 * @brief  Nonblocking read checking for newline on UART
 * 
 * Polls for 
 * other relevant information.
 * 
 * @param[in]  param1  Description of the first input parameter.
 * @param[in]  param2  Description of the second input parameter.
 * 
 * @return  Description of the return value.
 */
bool uart_getline(char *target){
	static char recbuff[UART_READBUFF_SIZE];
	static int recv_idx = 0;
	if (!uart_is_readable(uart0)){
		return false;
	}
	char newchar = uart_getc(uart0);
	if (recv_idx >= UART_READBUFF_SIZE-1 {
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

void set_fault(bool status){
	fault = status;
	gpio_put(FAULT_LED_PIN, status);
}

bool get_fault(){
	return fault;
}

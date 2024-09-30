#include "actuators.h"
#include "uart_logging.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

static encoder init_encoder(uint pinA, uint pinB){
	// each bit represents availability of nth state machine
	static char pioAvail = 0xff;
	if (pinA-pinB != 1){
		uart_log(LEVEL_ERROR, "Encoder pin A and B must be sequential! Aborting enc init");
		return NULL;
	}
	Encoder enc;
	// find an available state machine
	for (int i=0; i<8; i++){
		// ith pio state machine is available
		if ((pioAvail >> i) & 0x1){
			// mark SM as taken
			pioAvail &= ~(0x1 << i);
			if (i < 4){
				enc.pio = pio0;
				enc.sm = i;
			}
			else {
				enc.pio = pio1;
				enc.sm = i-4;
			}
		}
		else if (i == 7){
			uart_log(LEVEL_ERROR, "Cannot allocate PIO for encoder!!");
			return NULL;
		}
	}
	quadrature_encoder_program_init(enc.pio, enc.sm, pinA, 0);
	return enc;
}

void init_motors(){
	uart_log(LEVEL_INFO, "Starting motor init");
	// load pio program into both PIOs
	pio_add_program(pio0, &quadrature_encoder_program);
	pio_add_program(pio1, &quadrature_encoder_program);
	
}

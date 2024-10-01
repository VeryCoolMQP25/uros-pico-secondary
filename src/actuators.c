#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "quadrature_encoder.pio.h"
#include "actuators.h"
#include "uart_logging.h"

Motor drivetrain_left;
Motor drivetrain_right;
Motor lift_motor;

//return index of next unallocated PIO state machine
static int get_next_sm(){
	// each bit represents availability of nth state machine
	static char pioAvail = 0xff;
	for (int i=0; i<8; i++){
		// ith pio state machine is available
		if ((pioAvail >> i) & 0x1){
			// mark SM as taken
			pioAvail &= ~(0x1 << i);
			return i;
		}
	}
	return -1;
}

static Encoder *init_encoder(uint pinA, uint pinB){
	if (pinA-pinB != 1){
		uart_log(LEVEL_ERROR, "Encoder pin A and B must be sequential! Aborting enc init");
		return NULL;
	}
	Encoder *enc = malloc(sizeof(Encoder));
	// find an available state machine
	int sm_idx = get_next_sm();
	if (sm_idx == -1) {
		return NULL;
	}
	if (sm_idx < 4) {
		enc->pio = pio0;
		enc->sm = sm_idx;
	}
	else {
		enc->pio = pio1;
		enc->sm = sm_idx-4;
	}
	quadrature_encoder_program_init(enc->pio, enc->sm, pinA, 0);
	return enc;
}

static void init_motor(int pin, Motor *motor_struct){
	motor_struct->pin_num = pin;
	gpio_set_function(pin, GPIO_FUNC_PWM);
	uint32_t clk = clock_get_hz(clk_sys);
	uint32_t div = clk / (20000 * 50);
	
}

void init_motors(){
	uart_log(LEVEL_INFO, "Starting motor init");
	// load pio program into both PIOs
	pio_add_program(pio0, &quadrature_encoder_program);
	pio_add_program(pio1, &quadrature_encoder_program);
	
}

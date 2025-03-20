#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "actuators.h"
#include "pins.h"
#include "uart_logging.h"
#include "message_types.h"

Servo button_pusher_horiz;

float servo_angle_convert(int angle){
	if (abs(angle) > 90){
		uart_log(LEVEL_INFO, "Servo angle commanded out of range, correcting");
		if (angle > 0){
			angle = 90;
		} else {
			angle = -90;
		}
	}
	return ((float)angle+95.0)/SERVO_RANGE;
}

void init_servo(Servo *servo_struct, uint pin)
{
	servo_struct->pin_num = pin;
	gpio_set_function(pin, GPIO_FUNC_PWM);
	uint slice = pwm_gpio_to_slice_num(pin);
	servo_struct->slice_num = slice;

	float clkdiv = clock_get_hz(clk_sys) / (50.0f * 4096.0f);  // 4096 ticks per period
    pwm_set_wrap(slice, 4095);
    pwm_set_clkdiv(slice, clkdiv); // Set clock divider for accurate frequency
    pwm_set_enabled(slice, true);
	servo_struct->wrap = 4095;
	// set PWM to neutral before start
	// init and start PWM channel
	char debugbuff[60];
	snprintf(debugbuff, sizeof(debugbuff), "initialized servo with pin %d, slice %d", pin, slice);
	uart_log(LEVEL_INFO, debugbuff);
	set_servo_position(servo_struct, 90);
}

void set_servo_position(Servo *servo_struct, float position)
{
	pwm_set_enabled(servo_struct->slice_num, 1);
	uint setpoint = MIN_PULSE_WIDTH + position * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH);
	pwm_set_gpio_level(servo_struct->pin_num, setpoint);
	servo_struct->position = position;
	char debugbuff[100];
	snprintf(debugbuff, 100, "Setting servo to: %d deg. (setpoint %d)",position, setpoint);
	uart_log(LEVEL_DEBUG, debugbuff);
}

void pusher_servo_callback(const void *msgin){
	const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
	set_servo_position(&button_pusher_horiz, servo_angle_convert(msg->data));
}
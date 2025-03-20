#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "actuators.h"
#include "pins.h"
#include "uart_logging.h"
#include "message_types.h"

Servo button_pusher_horiz;

void init_servo(Servo *servo_struct, uint pin)
{
	servo_struct->pin_num = pin;
	gpio_set_function(pin, GPIO_FUNC_PWM);
	uint slice = pwm_gpio_to_slice_num(pin);
	servo_struct->slice_num = slice;

	float clkdiv = clock_get_hz(clk_sys) / (50.0f * 4096.0f);  // 4096 ticks per period
	pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 4095);
    pwm_config_set_clkdiv(&config, clkdiv); // Set clock divider for accurate frequency

    pwm_set_enabled(slice, true);
	servo_struct->wrap = 4095;
	// set PWM to neutral before start
	// init and start PWM channel
	pwm_init(slice, &config, true);
	char debugbuff[60];
	snprintf(debugbuff, sizeof(debugbuff), "initialized servo with pin %d, slice %d", pin, slice);
	uart_log(LEVEL_INFO, debugbuff);
	set_servo_position(servo_struct, 90);
}

uint servo_angle_convert(int angle){
	// 0 degrees commanded = 43 degrees actual
	if (abs(angle) > 45){
		uart_log(LEVEL_INFO, "Servo angle commanded out of range, correcting");
		if (angle > 0){
			angle = 45;
		} else {
			angle = -45;
		}
	}
	return angle + 43;
}

void set_servo_position(Servo *servo_struct, uint position)
{
    pwm_set_enabled(servo_struct->slice_num, 1);
	float pulse_width = MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * (position / (float)SERVO_RANGE));
    uint32_t level = (pulse_width * servo_struct->wrap) / 20000;  // Scale to wrap value
    pwm_set_gpio_level(servo_struct->pin_num, level);
    servo_struct->position = position;
    char debugbuff[100];
    snprintf(debugbuff, 100, "Setting servo to: %d deg. (setpoint %d)", position, level);
    uart_log(LEVEL_DEBUG, debugbuff);
}

void pusher_servo_callback(const void *msgin){
	const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
	set_servo_position(&button_pusher_horiz, servo_angle_convert(msg->data));
}
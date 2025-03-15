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
	gpio_init(pin);
	gpio_set_dir(pin, GPIO_OUT);
	gpio_put(pin, 0);
	uint slice = pwm_gpio_to_slice_num(pin);
	servo_struct->slice_num = slice;
	pwm_config config = pwm_get_default_config();
	pwm_config_set_clkdiv(&config, 100.0f);
	pwm_config_set_wrap(&config, SERVO_PWM_WRAP);
	// set PWM to neutral before start
	// init and start PWM channel
	pwm_init(slice, &config, true);
	char debugbuff[60];
	snprintf(debugbuff, sizeof(debugbuff), "initialized servo with pin %d, slice %d", pin, slice);
	uart_log(LEVEL_INFO, debugbuff);
	set_servo_position(servo_struct, 90);
}

void set_servo_position(Servo *servo_struct, uint position)
{
	if (position < SERVO_MIN_POS_DEG || position > SERVO_MAX_POS_DEG)
	{
		uart_log(LEVEL_WARN, "Invalid servo position commanded");
		return;
	}
	pwm_set_enabled(servo_struct->slice_num, 1);
	uint setpoint = SERVO_MIN_PWM + (position - SERVO_MIN_POS_DEG) * (SERVO_MAX_PWM - SERVO_MIN_PWM) / (SERVO_MAX_POS_DEG - SERVO_MIN_POS_DEG);
	pwm_set_gpio_level(servo_struct->pin_num, setpoint);
	servo_struct->position = position;
}

void pusher_servo_callback(const void *msgin){
	const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
	set_servo_position(&button_pusher_horiz, msg->data);
}
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "actuators.h"
#include "pins.h"
#include "uart_logging.h"
#include "message_types.h"
#include "hardware/watchdog.h"


Servo button_pusher_horiz;

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
	servo_struct->angle = -90;
	set_servo_position(servo_struct, 90);
}

float servo_angle_convert(int angle){
	if (abs(angle) > 90){
		uart_log(LEVEL_INFO, "Servo angle commanded out of range, correcting");
		if (angle > 0){
			angle = 90;
		} else {
			angle = -90;
		}
	}
	return (95.0-(float)angle)/SERVO_RANGE;
}

void set_servo_position(Servo *servo_struct, float position)
{
	pwm_set_enabled(servo_struct->slice_num, 1);
	uint setpoint = MIN_PULSE_WIDTH + position * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH);
	pwm_set_gpio_level(servo_struct->pin_num, setpoint);
	char debugbuff[100];
	snprintf(debugbuff, 100, "Setting servo to: %d deg. (setpoint %d)",position, setpoint);
	uart_log(LEVEL_DEBUG, debugbuff);
	watchdog_update();
}

void stop_servo(){
	pwm_set_enabled(button_pusher_horiz.slice_num, 0);
}

void pusher_servo_callback_absolute(const void *msgin){
	static long last_time = 0;
	long cur_time = time_us_64();
	// rate limit the servo command to 20
	if (cur_time - last_time < 50000){
		return;
	}
	last_time = cur_time;
	const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
	button_pusher_horiz.angle = msg->data;
	set_servo_position(&button_pusher_horiz, servo_angle_convert(msg->data));
}

void pusher_servo_callback_step(const void *msgin){
	static long last_time = 0;
	long cur_time = time_us_64();
	// rate limit the servo command to 20
	if (cur_time - last_time < 50000){
		return;
	}
	last_time = cur_time;
	const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
	int newpos = button_pusher_horiz.angle + msg->data;
	button_pusher_horiz.angle = newpos;
	set_servo_position(&button_pusher_horiz, servo_angle_convert(newpos));
}
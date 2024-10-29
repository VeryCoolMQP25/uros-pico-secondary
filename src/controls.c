#include <std_msgs/msg/int32_multi_array.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "uart_logging.h"
#include "controls.h"
#include "tunables.h"
//globals
static int left_power = 0;
static int right_power = 0;
static int lift_power = 0;
unsigned long dt_raw_last_update = 0;
static PIDController pid_left;
static PIDController pid_right;
static PIDController pid_lift;


void pid_setup(){
	pid_left = init_pid_control(PID_DT_KP, PID_DT_KI, PID_DT_KD);
	pid_right = init_pid_control(PID_DT_KP, PID_DT_KI, PID_DT_KD);
	pid_lift = init_pid_control(PID_LFT_KP, PID_LFT_KI, PID_LFT_KD);	
}

PIDController init_pid_control(float Kp, float Ki, float Kd){
	PIDController controller;
	controller.Kp = Kp;
	controller.Ki = Ki;
	controller.Kd = Kd;
	controller.previous_error = 0.0;
	controller.integral = 0.0;
	controller.last_tick_us = time_us_64();
	return controller;
}

void dt_power_callback(const void *indata) {
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)indata;
    if (msg->data.size != 3){
    	uart_log(LEVEL_WARN,"drivetrain power message wrong len! Discarded.");
    	return;
    }
	if (abs(msg->data.data[0]) > 100 || abs(msg->data.data[1]) > 100){
		uart_log(LEVEL_WARN,"bad drivetrain power message. Discarded.");
		return;
	}
    left_power = msg->data.data[0];
    right_power = msg->data.data[1];
    char debugbuff[60];
    snprintf(debugbuff, 60, "Drivetrain powers now (%d, %d)",left_power, right_power);
    uart_log(LEVEL_DEBUG, debugbuff);
    dt_raw_last_update = time_us_64();
}

int get_left_power(){
	return left_power;
}

int get_right_power(){
	return right_power;
}

int get_lift_power(){
	return lift_power;
}

bool get_lift_hardstop(){
	return gpio_get(LIFT_LIMIT_PIN);
}

void set_drivetrain_power(int l_power, int r_power){
	set_motor_power(&drivetrain_left, l_power);
	set_motor_power(&drivetrain_right, r_power);
}

void drivetrain_power_from_ros(){
	set_drivetrain_power(left_power, right_power);
}

void set_lift_power(int pwr){
	if (get_lift_hardstop() && pwr < 0){
		pwr = 0;
		uart_log(LEVEL_INFO, "Halted downward lift motion due to limit sw");
	}
	set_motor_power(&lift_motor, pwr);
}

void lift_power_from_ros(){
	set_lift_power(lift_power);
}

// Speed values in m/s
void set_drivetrain_speed(float l_speed, float r_speed){
	run_pid(&drivetrain_left, &pid_left, l_speed);
	run_pid(&drivetrain_right, &pid_right, r_speed);
}

void run_pid(Motor *motor, PIDController *pid, float target){
	uint64_t curtime = time_us_64();
	float delta_time_s = (curtime - pid->last_tick_us)/1000000.0;
	pid->last_tick_us = curtime;
	
	float error = target - motor->velocity;
	float P = pid->Kp * error;
	
	pid->integral += error * delta_time_s;
	float I = pid->Ki * pid->integral;

	float D = pid->Kd * ((error - pid->previous_error) / delta_time_s);

	pid->previous_error = error;

	// PID output
	int output = P + I + D;
	if (output > 100){
		output = 100;
	}
	else if (output < -100){
		output = -100;
	}
	set_motor_power(motor, output);
}

DriveMode drive_mode_from_ros(){
	// hardcoding raw mode for now, TODO
	if (time_us_64() - dt_raw_last_update > DRIVETRAIN_TIMEOUT){
		uart_log(LEVEL_WARN,"Drivetrain timeout exceeded!!");
		return dm_halt;
	}
	return dm_raw;
}

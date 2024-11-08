#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include "pico/stdlib.h"
#include "uart_logging.h"
#include "controls.h"
#include "tunables.h"
//globals
static int left_power = 0;
static int right_power = 0;
static int lift_power = 0;
unsigned long dt_raw_last_update = 0;
static PIDController pid_v_left;
static PIDController pid_v_right;
static PIDController pid_p_left;
static PIDController pid_p_right;
static PIDController pid_lift;
static DriveMode drive_mode_global = dm_halt;


void pid_setup(){
	pid_v_left = init_pid_control(PID_DT_V_KP, PID_DT_V_KI, PID_DT_V_KD, 0.0, pid_velocity);
	pid_v_right = init_pid_control(PID_DT_V_KP, PID_DT_V_KI, PID_DT_V_KD, 0.0, pid_velocity);
	pid_p_left = init_pid_control(PID_DT_KP, PID_DT_KI, PID_DT_KD, PID_DT_TOL, pid_position);
	pid_p_right = init_pid_control(PID_DT_KP, PID_DT_KI, PID_DT_KD, PID_DT_TOL, pid_position);
	pid_lift = init_pid_control(PID_LFT_KP, PID_LFT_KI, PID_LFT_KD, PID_LFT_TOL, pid_position);	
}

PIDController init_pid_control(float Kp, float Ki, float Kd, float tolerance, PIDMode pmode){
	PIDController controller;
	controller.Kp = Kp;
	controller.Ki = Ki;
	controller.Kd = Kd;
	controller.previous_error = 0.0;
	controller.integral = 0.0;
	controller.tolerance = tolerance;
	controller.mode = pmode;
	controller.last_tick_us = time_us_64();
	return controller;
}

void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    uart_log(LEVEL_ERROR, "Drivetrain Twist NOT IMPLEMENTED!");
    left_power = 0;
    right_power = 0;
    char debugbuff[60];
    snprintf(debugbuff, 60, "Drivetrain powers now (%d, %d)",left_power, right_power);
    uart_log(LEVEL_DEBUG, debugbuff);
    dt_raw_last_update = time_us_64();
}

void pid_k_callback(const void *msgin){
	const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
	uart_log(LEVEL_ERROR, "Live PID NOT IMPLEMENTED!");
	    
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
	run_pid_velocity(&drivetrain_left, &pid_v_left, l_speed);
	run_pid_velocity(&drivetrain_right, &pid_v_right, r_speed);
}

void run_pid(Motor *motor, PIDController *pid, float target){
	uint64_t curtime = time_us_64();
	float delta_time_s = (curtime - pid->last_tick_us)/1000000.0;
	pid->last_tick_us = curtime;
	float error;
	switch (pid->mode){
		case pid_velocity:
			error = target - motor->velocity;
			if (fabs(error) > pid->tolerance) {
				pid->integral += error * delta_time_s;
			} else {
				pid->integral = 0;  // Reset to avoid windup
			}
			break;
		case pid_position:
			error = target - motor->position;
			pid->integral += error * delta_time_s;
			break;
		default:
			uart_log(LEVEL_ERROR, "Invalid PID mode!");
			return;
	}

	float P = pid->Kp * error;
	float I = pid->Ki * pid->integral;
	float D = pid->Kd * ((error - pid->previous_error) / delta_time_s);

	// PID output
	int output = P + I + D;
	if (output > 100){
		output = 100;
	}
	else if (output < -100){
		output = -100;
	}
	pid->previous_error = error;
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

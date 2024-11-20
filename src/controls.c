#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "uart_logging.h"
#include "controls.h"
#include "tunables.h"
#include "message_types.h"

// globals
static PIDController pid_v_left;
static PIDController pid_v_right;
static PIDController pid_lift;
static DriveMode drive_mode_global = dm_halt;
unsigned long last_twist_msg = 0;
unsigned long last_lift_msg = 0;

void pid_setup()
{
	pid_v_left = init_pid_control(PID_DT_V_KP, PID_DT_V_KI, PID_DT_V_KD, PID_DT_TOL, pid_velocity);
	pid_v_right = init_pid_control(PID_DT_V_KP, PID_DT_V_KI, PID_DT_V_KD, PID_DT_TOL, pid_velocity);
	pid_lift = init_pid_control(PID_LFT_KP, PID_LFT_KI, PID_LFT_KD, PID_LFT_TOL, pid_position);
}

PIDController init_pid_control(float Kp, float Ki, float Kd, float tolerance, PIDMode pmode)
{
	PIDController controller;
	controller.Kp = Kp;
	controller.Ki = Ki;
	controller.Kd = Kd;
	controller.previous_error = 0.0;
	controller.integral = 0.0;
	controller.target = 0.0;
	controller.tolerance = tolerance;
	controller.mode = pmode;
	controller.last_tick_us = time_us_64();
	return controller;
}

void twist_callback(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
	float linear = msg->linear.x;	// m/s
	float angular = msg->angular.z; // rad/s
	pid_v_left.target = linear - (WHEELBASE_M * angular) / 2;
	pid_v_right.target = linear + (WHEELBASE_M * angular) / 2;
	last_twist_msg = time_us_64();
}

// calculate 'twist' messages based on observed encoder data
void populate_observed_twist(geometry_msgs__msg__TwistStamped *msg)
{
	float v_l = drivetrain_left.velocity;
	float v_r = drivetrain_right.velocity;
	float v_diff = v_l - v_r;
	unsigned long messagetime = time_us_64();
	msg->header.stamp.sec = messagetime / 1000000;
	msg->header.stamp.nanosec = messagetime % 1000000;
	msg->twist.angular.z = v_diff / WHEELBASE_M;
	msg->twist.linear.x = (v_l + v_r) / 2;
}

void raw_lift_callback(const void *msgin)
{
	const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
	set_lift_power((int)(msg->data * 100.0));
	last_lift_msg = time_us_64();
}

void set_lift_power(int pwr)
{
	if (get_lift_hardstop() && pwr < 0)
	{
		pwr = 0;
		uart_log(LEVEL_INFO, "Halted downward lift motion due to limit sw");
	}
	set_motor_power(&lift_motor, pwr);
}

void do_drivetrain_pid_v()
{
	run_pid(&drivetrain_left, &pid_v_left);
	run_pid(&drivetrain_right, &pid_v_right);
}

void run_pid(Motor *motor, PIDController *pid)
{
	static unsigned short printctr = 0;
	update_motor_encoders(motor);
	uint64_t curtime = time_us_64();
	float delta_time_s = (curtime - pid->last_tick_us) / 1000000.0;
	pid->last_tick_us = curtime;
	float error;
	switch (pid->mode)
	{
	case pid_velocity:
		error = pid->target - motor->velocity;
		if (fabs(error) > pid->tolerance)
		{
			pid->integral += error * delta_time_s;
		}
		else
		{
			pid->integral = 0; // Reset to avoid windup
		}
		if (pid->integral > PID_DT_KI_CAP)
		{
			uart_log(LEVEL_WARN, "Reset integral");
			pid->integral = 0;
		}
		break;
	case pid_position:
		error = pid->target - motor->position;
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
	int output = 100 * (P + I + D);
	if (output > 100)
	{
		output = 100;
	}
	else if (output < -100)
	{
		output = -100;
	}
	// if (printctr++ == 4){
	// 	char debugbuff[110];
	// 	snprintf(debugbuff, 110, "[%s] P:%f, I:%f, D:%f\ttgt: %f, act: %f, (%f) | out: %d",
	// 	motor->name, P, I, D, pid->target, motor->velocity, error, output);
	// 	uart_log_nonblocking(LEVEL_DEBUG, debugbuff);
	// 	printctr = 0;
	// }
	pid->previous_error = error;
	set_motor_power(motor, output);
}

DriveMode drive_mode_from_ros()
{
	static DriveMode last = dm_halt;
	if (time_us_64() - last_twist_msg > DRIVETRAIN_TIMEOUT)
	{
		if (last != dm_halt)
		{
			uart_log(LEVEL_WARN, "Drivetrain timeout exceeded!!");
			char asdf[50];
			snprintf(asdf, 50, "L: %f, R: %f", drivetrain_left.position, drivetrain_right.position);
			uart_log(LEVEL_DEBUG, asdf);
			drivetrain_left.position = 0.0;
			drivetrain_right.position = 0.0;
		}
		last = dm_halt;
		return dm_halt;
	}
	last = dm_twist;
	return dm_twist;
}

void die()
{
	// kill drivtrain control core (prevent WDT updates)
	multicore_lockout_start_blocking();
	while (1)
	{
		kill_all_actuators();
		uart_log(LEVEL_ERROR, "KILL ME");
	}
}

#ifndef CONTROLS_H
#define CONTROLS_H

#define DRIVETRAIN_TIMEOUT 200000 //microseconds

#include "pins.h"
#include "actuators.h"

typedef enum {
	dm_raw,
	dm_twist,
	dm_halt
} DriveMode;

typedef enum {
	pid_position,
	pid_velocity
} PIDMode;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float previous_error;
	float integral;
	float tolerance;
	PIDMode mode;
	uint64_t last_tick_us;
} PIDController;

void pid_setup();

PIDController init_pid_control(float Kp, float Ki, float Kd, float tolerance, PIDMode pmode);

void twist_callback(const void*);

void pid_k_callback(const void*);

int get_left_power();

int get_right_power();

int get_lift_power();

void set_drivetrain_power(int, int);

void drivetrain_power_from_ros();

void lift_power_from_ros();

void set_drivetrain_speed(float, float);

void run_pid(Motor*, PIDController*, float);

DriveMode drive_mode_from_ros();

#endif

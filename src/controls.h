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
	float target;
	PIDMode mode;
	uint64_t last_tick_us;
} PIDController;

void pid_setup();

PIDController init_pid_control(float Kp, float Ki, float Kd, float tolerance, PIDMode pmode);

void twist_callback(const void*);

void pid_k_callback(const void*);

void set_drivetrain_power(int, int);

void do_drivetrain_pid_v();

void run_pid(Motor*, PIDController*);

DriveMode drive_mode_from_ros();

#endif

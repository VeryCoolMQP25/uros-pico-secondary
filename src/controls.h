#ifndef CONTROLS_H
#define CONTROLS_H

typedef enum {
	dm_raw,
	dm_twist,
	dm_halt
} DriveMode;

void dt_power_callback(const void*);

int get_left_power();

int get_right_power();

void set_drivetrain_power(int, int);

void drivetrain_power_from_ros();

void set_drivetrain_speed(float, float);

#endif

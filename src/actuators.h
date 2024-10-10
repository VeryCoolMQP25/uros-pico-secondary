#ifndef ACTUATORS_H
#define ACTUATORS_H
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "quadrature_encoder.pio.h"

#define DT_L_ENCODER_A 6
#define DT_L_ENCODER_B 7
#define DT_R_ENCODER_A 8
#define DT_R_ENCODER_B 9

#define ENCODER_DIST_PER_PULSE	100 // arbitrary placehoder

#define DT_L_PWM 14
#define DT_R_PWM 15
#define LIFT_PWM 11

#define MOTOR_POWER_MAX	40

/** Talon SR speed controller specifics
 * 333Hz signal
 * 2.037ms = full "forward"
 * 1.539ms = the "high end" of the deadband range
 * 1.513ms = center of the deadband range (off)
 * 1.487ms = the "low end" of the deadband range
 * 0.989ms = full "reverse"
*/
#define TALON_PWM_FREQ 333
#define TALON_PWM_WRAP 15015
#define TALON_FULL_FWD 10184
#define TALON_FULL_REV 4945
#define TALON_DEADCTR  7565

typedef struct {
	PIO pio;
	uint sm; 
	uint prev_count;
	uint64_t prev_time_us;
} Encoder;

typedef struct {
	uint pin_num;
	uint slice_num;
	int curpower;
	Encoder *enc;
	float velocity;
} Motor;

extern Motor drivetrain_left;
extern Motor drivetrain_right;
extern Motor lift_motor;

void init_all_motors();
bool set_motor_power(Motor*, int);
void kill_all_actuators();

void update_motor_encoders(Motor*);

#endif

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "pins.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <stdint.h>

/* Servo PWM signaling:
* 50Hz signal
* 20ms period
* 500us = full "forward"
* 1500us = center of the deadband range (off)
* 2500us = full "reverse"
*/
#define PWM_FREQUENCY 50
#define MIN_PULSE_WIDTH 500   // 500us
#define MAX_PULSE_WIDTH 2500  // 2500us
#define SERVO_RANGE 180.0f       // Servo angle range (0-180 degrees)
#define SERVO_PWM_WRAP     24999

typedef struct {
    uint pin_num;
    uint slice_num;
    uint position;
    uint wrap;
} Servo;

extern Servo button_pusher_horiz;

void init_servo(Servo *servo_struct, uint pin_num);
void set_servo_position(Servo *servo_struct, float position);
void stop_servo();
void pusher_servo_callback(const void *msgin);
#endif

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
#define SERVO_PWM_FREQ     50
#define SERVO_PWM_WRAP     24999
#define SERVO_MIN_PWM      1250  // Servo 1000µs in counter ticks (1000 * 25000 / 20000)
#define SERVO_MAX_PWM      3125
#define SERVO_MIN_POS_DEG  0
#define SERVO_MAX_POS_DEG  180


typedef struct {
    uint pin_num;
    uint slice_num;
    uint position;
} Servo;

extern Servo button_pusher_horiz;

void init_servo(Servo *servo_struct, uint pin_num);
void set_servo_position(Servo *servo_struct, uint position);
void pusher_servo_callback(const void *msgin);
#endif

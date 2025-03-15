#ifndef SENSORS_H
#define SENSORS_H
#include "message_types.h"

void prepare_lift_height(sensor_msgs__msg__Range *height_message);
void update_lift_height(sensor_msgs__msg__Range *height_message);
void height_monitor_c1();
#endif
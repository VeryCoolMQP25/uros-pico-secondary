#ifndef NAV_H
#define NAV_H
#include "message_types.h"

typedef struct {
    pos_x: float,
    pos_y: float,
    yaw: float
} OdomState;

void populate_odometry(nav_msgs__msg__Odometry *msg);
void update_odometry();
#endif
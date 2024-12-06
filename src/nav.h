#ifndef NAV_H
#define NAV_H
#include "message_types.h"

typedef struct {
    double pos_x;
    double pos_y;
    float yaw;
    unsigned long timestamp;
} OdomState;

void init_odometry();
void populate_odometry(nav_msgs__msg__Odometry *msg);
void update_odometry();
#endif
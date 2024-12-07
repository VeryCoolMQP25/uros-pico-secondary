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
void populate_transform(geometry_msgs__msg__TransformStamped *tf, nav_msgs__msg__Odometry *odom);
void update_odometry();
void reset_odometry();
void init_odom_message(nav_msgs__msg__Odometry *odometry_message);
void init_tf_message(geometry_msgs__msg__TransformStamped *msg);
#endif
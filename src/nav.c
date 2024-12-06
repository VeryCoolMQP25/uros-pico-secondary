#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "uart_logging.h"
#include "controls.h"
#include "tunables.h"
#include "nav.h"

OdomState *ods_a;
OdomState *ods_b;
OdomState *ods_cur;

void init_odometry(nav_msgs__msg__Odometry *msg){
    ods_a = malloc(sizeof(OdomState));
    ods_b = malloc(sizeof(OdomState));
    ods_cur = ods_a;
}

void populate_odometry(nav_msgs__msg__Odometry *msg){
	msg->header.stamp.sec = ods_cur->timestamp / 1000000;
	msg->header.stamp.nanosec = ods_cur->timestamp % 1000000;

	// Pose
	msg->pose.pose.position.x = ods_cur->pos_x;
	msg->pose.pose.position.y = ods_cur->pos_y;

	// Convert yaw angle to quaternion
	msg->pose.pose.orientation.z = sin(ods_cur->yaw / 2.0);
	msg->pose.pose.orientation.w = cos(ods_cur->yaw / 2.0);

	// Twist
	float v_l = drivetrain_left.velocity;
	float v_r = drivetrain_right.velocity;
	float v_diff = v_r - v_l;
	msg->twist.twist.angular.z = v_diff / WHEELBASE_M;
	msg->twist.twist.linear.x = (v_l + v_r) / 2;
}

// update odometry, swap between two structs so all data updates are thread safe
void update_odometry(){
    // manipulate alternate ODS struct
    OdomState *ods_working = ods_a;
    if(ods_cur == ods_a){
        ods_working = ods_b;
    }
    // calculate odometry
    ods_working->timestamp = time_us_64();
    double dt = ((float)(ods_working->timestamp - ods_cur->timestamp))/1000000;
    double v_linear = (drivetrain_left.velocity + drivetrain_right.velocity)/2;
    double d_theta = (drivetrain_right.velocity - drivetrain_left.velocity)/WHEELBASE_M;
    ods_working->pos_x = ods_cur->pos_x + v_linear*cos(d_theta)*dt;
    ods_working->pos_y = ods_cur->pos_y + v_linear*sin(d_theta)*dt;
    ods_working->yaw = ods_cur->yaw + d_theta;

    // set current struct to the one we just updated
    ods_cur = ods_working;
}
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "uart_logging.h"
#include "controls.h"
#include "tunables.h"
#include "nav.c"

OdomState *ods_a = malloc(sizeof(OdomState));
OdomState *ods_b = malloc(sizeof(OdomState));
OdomState *ods_cur = ods_a;


void populate_odometry(nav_msgs__msg__Odometry *msg){
	unsigned long messagetime = time_us_64();
	msg->header.stamp.sec = messagetime / 1000000;
	msg->header.stamp.nanosec = messagetime % 1000000;
	msg->header.frame_id = "odom";
	msg->child_frame_id = "base_link";

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
    OdomState ods_working;
    if(ods_cur == ods_a){
        ods_working = ods_b;
    }
    else {
        ods_working = ods_a;
    }
    if (ods_working == NULL){
        uart_log(LEVEL_ERROR, "Odometry state null ptr!");
        return;
    }
    // calculate odometry
    

    // set current struct to the one we just updated
    ods_cur = ods_working;
}
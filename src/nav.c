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
#include "haw/MPU6050.h"

OdomState *ods_a;
OdomState *ods_b;
OdomState *ods_cur;
// mpu6050_t mpu6050;
// bool imu = true;
// mpu6050_vectorf_t *accel;
// mpu6050_vectorf_t *gyro;
// float imu_yaw = 0.0;

// void init_imu(){
// 	gpio_init(I2C_SDA);
//     gpio_init(I2C_SCL);
//     gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
//     gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
//     gpio_pull_up(I2C_SDA);
//     gpio_pull_up(I2C_SCL);

//     // Pass in the I2C driver (Important for dual-core operations). The second parameter is the address,
//     // which can change if you connect pin A0 to GND or to VCC.
//     mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_VCC);
// 	int imu_fail_ctr = 0;
// 	while (!mpu6050_begin(&mpu6050)) {
// 		uart_log(LEVEL_WARN, "BLOCKING: Waiting for IMU...");
// 		sleep_ms(100);
// 		if (imu_fail_ctr++ >= 8){
// 			uart_log(LEVEL_ERROR, "Failed to init IMU!");
// 			imu = false;
// 			return;
// 		}
// 	}
// 	// Set scale of gyroscope
// 	mpu6050_set_scale(&mpu6050, MPU6050_SCALE_250DPS);
// 	// Set range of accelerometer
// 	mpu6050_set_range(&mpu6050, MPU6050_RANGE_4G);

// 	// Disable temperature, enable gyroscope and accelerometer readings
// 	mpu6050_set_temperature_measuring(&mpu6050, false);
// 	mpu6050_set_gyroscope_measuring(&mpu6050, true);
// 	mpu6050_set_accelerometer_measuring(&mpu6050, true);
// 	imu = true;
// 	update_imu();
// 	uart_log(LEVEL_INFO,"MPU6050 Initialized");
// }

void init_odometry(){
    ods_a = malloc(sizeof(OdomState));
    ods_b = malloc(sizeof(OdomState));
    reset_odometry();
    ods_cur = ods_a;
}

void populate_odometry(nav_msgs__msg__Odometry *msg){
    float yaw = ods_cur->yaw;
	msg->header.stamp.sec = ods_cur->timestamp / 1000000;
	msg->header.stamp.nanosec = ods_cur->timestamp % 1000000;

	// Pose
	msg->pose.pose.position.x = ods_cur->pos_x;
	msg->pose.pose.position.y = ods_cur->pos_y;

	// Convert yaw angle to quaternion
	msg->pose.pose.orientation.z = sin(yaw / 2.0);
	msg->pose.pose.orientation.w = cos(yaw / 2.0);

	// Twist
	float v_l = drivetrain_left.velocity;
	float v_r = drivetrain_right.velocity;
	float v_diff = v_r - v_l;
	msg->twist.twist.angular.z = v_diff / WHEELBASE_M;
	msg->twist.twist.linear.x = (v_l + v_r) / 2;
    char debugbuff[25];
    snprintf(debugbuff, 25, "yaw: %f deg.", yaw*(180.0/3.1415));
    uart_log(LEVEL_DEBUG, debugbuff);
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
    double yaw = (drivetrain_right.position - drivetrain_left.position)/WHEELBASE_M;
    ods_working->pos_x = ods_cur->pos_x + v_linear*cos(yaw)*dt;
    ods_working->pos_y = ods_cur->pos_y + v_linear*sin(yaw)*dt;
    ods_working->yaw = yaw;

    // set current struct to the one we just updated
    ods_cur = ods_working;
}

// void update_imu(){
// 	if (imu){
// 		mpu6050_event(&mpu6050);
// 		accel = mpu6050_get_accelerometer(&mpu6050);
// 		gyro = mpu6050_get_gyroscope(&mpu6050);
// 	}
// }

void reset_odometry(){
    ods_a->pos_x = 0.0;
    ods_a->pos_y = 0.0;
    ods_a->yaw = 0.0;
    ods_b->pos_x = 0.0;
    ods_b->pos_y = 0.0;
    ods_b->yaw = 0.0;
    drivetrain_left.enc->prev_count=0;
    drivetrain_right.enc->prev_count=0;
    uart_log(LEVEL_INFO, "Reset odometry");
}

void init_odom_message(nav_msgs__msg__Odometry *odometry_message){
    	odometry_message->header.stamp.sec = 0;  // Set seconds part of the timestamp (replace with actual time)
	odometry_message->header.stamp.nanosec = 0;  // Set nanoseconds part of the timestamp (replace with actual time)
	odometry_message->header.frame_id.data = "odom";  // Set the frame of reference for the odometry
	odometry_message->header.frame_id.size = strlen(odometry_message->header.frame_id.data);
	odometry_message->header.frame_id.capacity = odometry_message->header.frame_id.size + 1;
	odometry_message->child_frame_id.data = "base_link";  // The child frame (typically the robot base or robot link)
	odometry_message->child_frame_id.size = strlen(odometry_message->child_frame_id.data);
	odometry_message->child_frame_id.capacity = odometry_message->child_frame_id.size + 1;


	// Initialize position (pose)
	odometry_message->pose.pose.position.x = 0.0;  // Initial X position
	odometry_message->pose.pose.position.y = 0.0;  // Initial Y position
	odometry_message->pose.pose.position.z = 0.0;  // Initial Z position

	// Initialize orientation (quaternion)
	odometry_message->pose.pose.orientation.x = 0.0;  // X component of the quaternion
	odometry_message->pose.pose.orientation.y = 0.0;  // Y component of the quaternion
	odometry_message->pose.pose.orientation.z = 0.0;  // Z component of the quaternion
	odometry_message->pose.pose.orientation.w = 1.0;  // W component of the quaternion (identity quaternion)

	// Initialize linear velocity (twist)
	odometry_message->twist.twist.linear.x = 0.0;  // Linear velocity in X direction (m/s)
	odometry_message->twist.twist.linear.y = 0.0;  // Linear velocity in Y direction (m/s)
	odometry_message->twist.twist.linear.z = 0.0;  // Linear velocity in Z direction (m/s)

	// Initialize angular velocity (twist)
	odometry_message->twist.twist.angular.x = 0.0;  // Angular velocity around X axis (rad/s)
	odometry_message->twist.twist.angular.y = 0.0;  // Angular velocity around Y axis (rad/s)
	odometry_message->twist.twist.angular.z = 0.0;  // Angular velocity around Z axis (rad/s)
}

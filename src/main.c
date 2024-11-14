#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"
#include "controls.h"
#include "pins.h"
#include "message_types.h"

// globals
const char *namespace = "";
DriveMode drive_mode = dm_halt;


/// support for encoder publisher
rcl_publisher_t encoder_publisher;
geometry_msgs__msg__TwistStamped observed_twist_msg;
// callback to publish encoder data (processed into timestamped twists)

void publish_encoder(rcl_timer_t *timer, int64_t last_call_time){
	populate_observed_twist(&observed_twist_msg);
	// Publish message
	if(rcl_publish(&encoder_publisher, &observed_twist_msg, NULL)){
		uart_log(LEVEL_WARN,"Encoder publish failed!");
	}
}

// checks if we have comms with serial agent
void check_connectivity(rcl_timer_t *timer, int64_t last_call_time){
	//uart_log(LEVEL_DEBUG, "connectivity CB run");
	bool ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	if (!ok){
		drive_mode = dm_halt;
	}
	watchdog_update();
}

// creates and returns a timer, configuring it to call specified callback. Returns timer handle
rcl_timer_t *create_timer_callback(rclc_executor_t *executor, rclc_support_t *support, uint period_ms, rcl_timer_callback_t cb)
{
	rcl_timer_t *timer = malloc(sizeof(rcl_timer_t));
    rclc_timer_init_default(timer, support, RCL_MS_TO_NS(period_ms), cb);
	rclc_executor_add_timer(executor, timer);
	uart_log(LEVEL_DEBUG, "registered timer cb");
	return timer;
}

void core1task(){
	uart_log(LEVEL_DEBUG, "Started core 1 task");
	while(true){
		watchdog_update();
		drive_mode = drive_mode_from_ros();
		switch(drive_mode){
			case dm_halt:
				set_motor_power(&drivetrain_left, 0);
				set_motor_power(&drivetrain_right, 0);
				break;
			case dm_twist:
				do_drivetrain_pid_v();
				break;
			default:
				uart_log(LEVEL_WARN, "Invalid drive state!");
				drive_mode = dm_halt;
		}
		sleep_ms(4);
	}
	uart_log(LEVEL_ERROR, "Exiting core1 task!");
	kill_all_actuators();	
}

int main()
{	
	// init uart0 debugging iface
    uart_setup();
    uart_log(LEVEL_DEBUG, "Started UART comms");
    if (watchdog_caused_reboot()){
    	uart_log(LEVEL_WARN, "Rebooted by watchdog!");
    }
    else {
    	uart_log(LEVEL_INFO, "Boot was clean.");
    }
    uart_log(LEVEL_INFO, "Starting watchdog...");
    watchdog_enable(300, 1);
	// in case of unclean boot, make sure actuators are off
	kill_all_actuators();
	// init USB serial comms
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
	
	// setup on-board status LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    uart_log(LEVEL_INFO, "Waiting for agent...");
    
    // try 50 times to ping, 50ms timeout each ping
	for (int i = 0; i < 50; i++){
		if (rmw_uros_ping_agent(1, 50) == RCL_RET_OK){
			uart_log(LEVEL_INFO,"Connected to host.");
			watchdog_update();
			break;
		}
		char outbuff[20];
		snprintf(outbuff, 20, "Ping #%d failed.", i);
		uart_log(LEVEL_DEBUG, outbuff);
		if (i == 49){
			uart_log(LEVEL_ERROR, "Cannot contact USB Serial Agent! Bailing!");
			//wait for watchdog to reset board
			while(1);
		}
		watchdog_update();
	}

	// --init uros--
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();	
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", namespace, &support);
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    
    // --create timed events--
    create_timer_callback(&executor, &support, 50, publish_encoder);
    create_timer_callback(&executor, &support, 200, check_connectivity);
	watchdog_update();
	
	// --create publishers--
	rclc_publisher_init_default(
	    &encoder_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
	    "twist_observed"
	);
	// setup static components of message
	observed_twist_msg.header.frame_id.data = "base_link";
	observed_twist_msg.header.frame_id.size = strlen(observed_twist_msg.header.frame_id.data);
	observed_twist_msg.header.frame_id.capacity = observed_twist_msg.header.frame_id.size + 1;
	observed_twist_msg.twist.linear.x = 0.0;
	observed_twist_msg.twist.linear.y = 0.0;
	observed_twist_msg.twist.linear.z = 0.0;
	observed_twist_msg.twist.angular.x = 0.0;
	observed_twist_msg.twist.angular.y = 0.0;
	observed_twist_msg.twist.angular.z = 0.0;
	
    // --create subscribers--
    // twist command subscriber
    rcl_subscription_t twist_subscriber;
    geometry_msgs__msg__Twist twist_msg;
    rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);
	watchdog_update();
	// Lift command subscriber
    rcl_subscription_t lift_subscriber;
    std_msgs__msg__Float32 lift_msg;
    rclc_subscription_init_default(
        &lift_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "lift_raw");
    rclc_executor_add_subscription(&executor, &lift_subscriber, &lift_msg, &raw_lift_callback, ON_NEW_DATA);
	watchdog_update();
    // -- general inits --
    init_all_motors();
    pid_setup();
    uart_log(LEVEL_DEBUG, "Finished init, starting exec");
    
    multicore_launch_core1(core1task);
   	rclc_executor_spin(&executor);
	uart_log(LEVEL_ERROR, "Executor exited! Emergency Stop.");
	gpio_put(LED_PIN, 0);
	// wait to be killed by watchdog
    while(1) {
    	kill_all_actuators();
    	uart_log(LEVEL_ERROR,"KILL ME");
    }
}

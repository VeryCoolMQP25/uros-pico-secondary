#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"
#include "controls.h"

// globals
const char *namespace = "";
// system states
DriveMode drive_mode = dm_raw;

// onboard green LED
const uint LED_PIN = 25;


void publish_all_cb(rcl_timer_t *timer, int64_t last_call_time){
	//uart_log(LEVEL_DEBUG, "publisher CB run");
	return;
	//TODO
}

// checks if we have comms with serial agent
void check_connectivity(rcl_timer_t *timer, int64_t last_call_time){
	//uart_log(LEVEL_DEBUG, "connectivity CB run");
	bool ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	if (!ok){
		drive_mode = dm_halt;
	}
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
		switch(drive_mode){
			case dm_halt:
				kill_all_actuators();
				break;
			case dm_raw:
				drivetrain_power_from_ros();
				break;
			default:
				uart_log(LEVEL_WARN, "Invalid drive state!");
				drive_mode = dm_halt;
		}
		sleep_ms(10);
	}
	uart_log(LEVEL_ERROR, "Exiting core1 task!");
	kill_all_actuators();	
}

int main()
{
	// init USB serial comms
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

	// init uart0 debugging iface
    uart_setup();
    uart_log(LEVEL_DEBUG, "Started UART comms");
    uart_log(LEVEL_INFO, "Waiting for agent...");

    rcl_ret_t ret = rmw_uros_ping_agent(50, 120);

    if (ret != RCL_RET_OK)
    {
        uart_log(LEVEL_ERROR, "Cannot contact USB Serial Agent! Bailing out!");
        return ret;
    }

	// init uros
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();	
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", namespace, &support);
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    
    // create timed events
    create_timer_callback(&executor, &support, 500, publish_all_cb);
    create_timer_callback(&executor, &support, 200, check_connectivity);
    
	std_msgs__msg__Int32MultiArray dt_pwr_msg;
	std_msgs__msg__Int32MultiArray__init(&dt_pwr_msg); // Initialize the message
	// allocate space for the message payload
	// int32_t dt_powers[2];
	dt_pwr_msg.data.data = malloc(sizeof(int32_t)*2);
	dt_pwr_msg.data.size = 0;
	dt_pwr_msg.data.capacity = 2;

    // create message subscribers
    rcl_subscription_t dt_pwr_sub;
    rclc_subscription_init_default(&dt_pwr_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/drivetrain_powers");    
    ret =rclc_executor_add_subscription(&executor, &dt_pwr_sub, &dt_pwr_msg, &dt_power_callback, ON_NEW_DATA);
	char debugbuff[50];
	snprintf(debugbuff,50,"Add subscription returned code %d", ret);
	uart_log(LEVEL_DEBUG,debugbuff);
    init_all_motors();
    uart_log(LEVEL_DEBUG, "Finished init, starting exec");
    
    multicore_launch_core1(core1task);
   	rclc_executor_spin(&executor);
	uart_log(LEVEL_ERROR, "Executor exited! Emergency Stop.");
	kill_all_actuators();
	gpio_put(LED_PIN, 0);
    return 0;
}

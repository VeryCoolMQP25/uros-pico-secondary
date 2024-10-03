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
CommState comm_state = cs_disconnected;
DriveMode drive_mode = dm_raw;

int comm_fail_counter = 0;
// onboard green LED
const uint LED_PIN = 25;


void publish_all_cb(rcl_timer_t *timer, int64_t last_call_time)
{
	//TODO
    return;
}

// checks if we have comms with serial agent
void check_connectivity(rcl_timer_t *timer, int64_t last_call_time){
	bool ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	if (!ok){
		comm_state = cs_disconnected;
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
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    // create timed events
    create_timer_callback(&executor, &support, 500, publish_all_cb);
    create_timer_callback(&executor, &support, 200, check_connectivity);
    
	std_msgs__msg__Int32MultiArray dt_pwr_msg;
	std_msgs__msg__Int32MultiArray__init(&dt_pwr_msg); // Initialize the messag
    // create message subscribers
    rcl_subscription_t dt_pwr_sub;
    rclc_subscription_init_default(
        &dt_pwr_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "drivetrain_powers");    
    rclc_executor_add_subscription(&executor, &dt_pwr_sub, &dt_pwr_msg, &dt_power_callback, ON_NEW_DATA);
    init_all_motors();
    uart_log(LEVEL_DEBUG, "Finished init, starting exec");
    multicore_launch_core1(core1task);
	while (true){
		switch(comm_state){
			case cs_down: {
				kill_all_actuators();
				if (rmw_uros_ping_agent(10, 5) == RCL_RET_OK){
					uart_log(LEVEL_INFO, "trying to re-connect...");
					comm_state = cs_disconnected;
					comm_fail_counter = 0;
				}
				break;
			}
			case cs_connected: {
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
				break;
			}
			case cs_disconnected: {
				if(rmw_uros_ping_agent(50, 1) == RCL_RET_OK){
					uart_log(LEVEL_INFO, "Connected to ROS agent");
					comm_state = cs_connected;
					comm_fail_counter = 0;
				}
				else if(++comm_fail_counter > COMM_FAIL_THRESH){
					comm_state = cs_down;
					uart_log(LEVEL_WARN, "No connection to ROS agent!");
				}
				break;
			}
			default:
				uart_log(LEVEL_ERROR, "Illegal state!");
				return 1;
		}
	}
	uart_log(LEVEL_ERROR, "Executor exited!");
	
    return 0;
}

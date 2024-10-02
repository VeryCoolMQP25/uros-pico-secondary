#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"

// onboard green LED
const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    uart_log(LEVEL_INFO, "Sent ROS message");
    msg.data+=5;
}

// checks if we have comms with serial agent
bool check_connectivity(){
	bool ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	return ok;
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

	// init uros
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

	// wait for host agent to come online (60 seconds)
    rcl_ret_t ret = rmw_uros_ping_agent(50, 120);

    if (ret != RCL_RET_OK)
    {
    	char tosend[50];
    	snprintf(tosend, 50, "Cannot contact USB Serial Agent! Got code %i", ret);
        uart_log(LEVEL_ERROR, tosend);
        return ret;
    }
	uart_log(LEVEL_INFO, "Connected to ROS agent");
	
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    create_timer_callback(&executor, &support, 100, timer_callback);

    msg.data = 0;
    init_all_motors();
    uart_log(LEVEL_DEBUG, "Finished init, starting exec");

    rclc_executor_spin(&executor);
    
	uart_log(LEVEL_ERROR, "Executor exited!");
	
    return 0;
}

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"
#include "pins.h"
#include "message_types.h"
#include "sensors.h"

#define RCL_CONTEXT_COUNT 4 // servo angle,  check conn timer, button_press_checker_timer

// globals
const char *namespace = "";


void die()
{
	while (1)
	{
		stop_servo();
	  uart_log(LEVEL_ERROR, "KILL ME");
	}
}

// checks if we have comms with serial agent
void check_connectivity(rcl_timer_t *timer, int64_t last_call_time)
{
	// uart_log(LEVEL_DEBUG, "connectivity CB run");
	bool ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	if (!ok)
	{
		uart_log(LEVEL_ERROR, "Disconnected from uROS!");
		die();
	}
	watchdog_update();
}

// rcl_publisher_t height_publisher;
// sensor_msgs__msg__Range height_message;
// void publish_range(rcl_timer_t *timer, int64_t last_call_time)
// {
// 	//fill in up-to-date values for odom
// 	update_lift_height(&height_message);
// 	// Publish messages
// 	if (rcl_publish(&height_publisher, &height_message, NULL))
// 	{
// 		uart_log(LEVEL_WARN, "Height reading publish failed!");
// 	}

// }

rcl_publisher_t limit_switch_publisher;
std_msgs__msg__Bool limit_switch_message;

void publish_limit_switch(rcl_timer_t *timer, int64_t last_call_time)
{
	// button is active low
	bool state = !gpio_get(BUTTON_LIMIT_PIN);
	// skip if state is unchanged
	if (state == limit_switch_message.data)
	{
		return;
	}
	limit_switch_message.data = state;
	// Publish messages
	if (rcl_publish(&limit_switch_publisher, &limit_switch_message, NULL))
	{
		uart_log(LEVEL_WARN, "Limit switch publish failed!");
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

int main()
{
	// init uart0 debugging iface
	uart_setup();
	if (watchdog_caused_reboot())
	{
		uart_log(LEVEL_WARN, "Rebooted by watchdog!");
		// in case of unclean boot, make sure actuators are off
	}
	else
	{
		uart_log(LEVEL_DEBUG, "Boot was clean.");
	}
	uart_log(LEVEL_INFO, "Starting watchdog...");
	watchdog_enable(300, 1);
	// init USB serial comms
	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read);

	// setup on-board status LED
	gpio_init(LED_PIN);
	gpio_init(BUTTON_LIMIT_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_set_dir(BUTTON_LIMIT_PIN, GPIO_IN);
	gpio_pull_up(BUTTON_LIMIT_PIN);

	uart_log(LEVEL_INFO, "Waiting for agent...");

	// try 50 times to ping, 50ms timeout each ping
	for (int i = 0; i < 50; i++)
	{
		watchdog_update();
		if (rmw_uros_ping_agent(1, 80) == RCL_RET_OK)
		{
			uart_log(LEVEL_INFO, "Connected to host.");
			watchdog_update();
			break;
		}
		char outbuff[25];
		snprintf(outbuff, 25, "Ping %d/50 failed.", i + 1);
		uart_log(LEVEL_DEBUG, outbuff);
		if (i == 49)
		{
			uart_log(LEVEL_ERROR, "Cannot contact USB Serial Agent! Bailing!");
			// wait for watchdog to reset board
			while (1)
				;
		}
	}

	// --init uros--
	rcl_node_t node;
	rcl_allocator_t allocator;
	rclc_support_t support;
	rclc_executor_t executor;

	allocator = rcl_get_default_allocator();
	rclc_support_init(&support, 0, NULL, &allocator);
	rclc_node_init_default(&node, "lift_pico", namespace, &support);
	rclc_executor_init(&executor, &support.context, RCL_CONTEXT_COUNT, &allocator);
	init_servo(&button_pusher_horiz, SERVO_PWM);
	// --create timed events--
	create_timer_callback(&executor, &support, 200, check_connectivity);
	// create_timer_callback(&executor, &support, 20, publish_range);
	create_timer_callback(&executor, &support, 20, publish_limit_switch);
	watchdog_update();

	// Servo command subscriber
	// /servo/degrees
	// /servo/step_degrees
	rcl_subscription_t servo_subscriber_absolute;
	std_msgs__msg__Int16 servo_msg_absolute;
	rclc_subscription_init_default(
		&servo_subscriber_absolute,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
		"/servo/degrees");

	rclc_executor_add_subscription(&executor, &servo_subscriber_absolute, &servo_msg_absolute, &pusher_servo_callback_absolute, ON_NEW_DATA);

	rcl_subscription_t servo_subscriber_step;
	std_msgs__msg__Int16 servo_msg_step;
	rclc_subscription_init_default(
		&servo_subscriber_step,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
		"/servo/step_degrees");
	rclc_executor_add_subscription(&executor, &servo_subscriber_step, &servo_msg_step, &pusher_servo_callback_step, ON_NEW_DATA);
	// prepare_lift_height(&height_message);
	watchdog_update();

		// --create publishers--
	// rclc_publisher_init_default(
	// 	&height_publisher,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
	// 	"height");
	rclc_publisher_init_default(
		&limit_switch_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"limit_switch");
		limit_switch_message.data = false;
	// -- general inits --

	uart_log(LEVEL_DEBUG, "Finished init, starting exec");
	// multicore_launch_core1(height_monitor_c1);
	rclc_executor_spin(&executor);
	uart_log(LEVEL_ERROR, "Executor exited!");
	gpio_put(LED_PIN, 0);
	// wait to be killed by watchdog
	die();
}

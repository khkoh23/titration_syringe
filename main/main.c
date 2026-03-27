#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/float32.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "syringe_gpio.h"
#include "tanmone_uart.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define ROS_NAMESPACE "" //#define ROS_NAMESPACE CONFIG_MICRO_ROS_NAMESPACE

#define STEP_MOTOR_GPIO_STEP (GPIO_NUM_1)
#define STEP_MOTOR_GPIO_DIR (GPIO_NUM_2)
#define STEP_MOTOR_GPIO_EN (GPIO_NUM_4)
#define RS1_DE (GPIO_NUM_16)
#define RS1_TX (GPIO_NUM_17)
#define RS1_RX (GPIO_NUM_18)

static const char *SYRINGE_GPIO_TAG = "syringe_gpio";
static const char *TANMONE_UART_TAG = "tanmone_uart";
static const char *MICRO_ROS_TASK_TAG = "micro_ros";

enum micro_ros_agent_state {
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
} uros_state;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t ph_publisher, temperature_publisher, orp_publisher;
// rcl_timer_t ph_timer, temperature_timer, orp_timer;
rcl_timer_t tanmone_timer;
rcl_subscription_t syringecmd_subscriber, syringesetvol_subscriber, syringestepvol_subscriber;
std_msgs__msg__Float32 ph_msg, temperature_msg;
std_msgs__msg__Int16 orp_msg;
std_msgs__msg__Int8 syringecmd_msg;
std_msgs__msg__UInt16 syringesetvol_msg, syringestepvol_msg;
const int micro_ros_timeout_ms = 100; // Timeout for each micro ros ping attempt
const uint8_t micro_ros_attempts = 1; // Number of micro ros ping attempts
const uint64_t micro_ros_spin_timeout = RCL_MS_TO_NS(1); // Spin period for micro ros executor
uint16_t micro_ros_i; // Counter for micro ros time sync
const int micro_ros_sync_session_timeout_ms = 50; // Timeout for micro ros time sync

const float syringe_id = 12.4; // UOM is mm; Barrel inner diameter for 5 ml syringe
const uint8_t leadscrew_feedrate = 1; // UOM is mm/rev; syringe piston travel distance per stepper revoluation
const uint8_t step_per_rev = 200; // 200 step/rev for 1.8deg stepper motor
const uint16_t microstep = 10;
const float syringe_feedrate = 3.14159265 * (syringe_id/2)*(syringe_id/2) * leadscrew_feedrate / step_per_rev / microstep; // UOM is uL/step

bool new_syringe;
uint8_t syringe_task = 0; 
uint16_t syringe_set_volume = 20000, new_syringe_set_volume; // unit: 0.1uL, default: 2000 uL, range: 5 uL to 5000 uL
uint16_t syringe_step_volume = 200; // default: 20 uL
uint16_t syringe_holding_volume; // unit: 0.1uL (Maximum 0xFFFF =  6553.5 uL (6.5535 mL)

uint8_t tanmone_device_ph = 0x02;
uint8_t tanmone_device_orp = 0x04;
uint8_t tanmone_task = 0;
int16_t tanmone_temperature_buffer, tanmone_ORP_buffer;
uint16_t tanmone_pH_buffer;
float tanmone_pH, tanmone_temperature;
int16_t tanmone_ORP; // UOM is mV

static size_t uart_port = UART_NUM_0;


/* -------------------- syringe gpio -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/



void syringe_gpio_help_response() {

}

void syringe_gpio_task(void *arg) {  

	while (1) {
		if (new_syringe) {
			switch (syringe_task) {
				case 1: // readDeviceId
					break;
				case 2: // writeDeviceId
					break;
				case 3: // readSetVolume
					break;
				case 4: // writeSetVolume
					syringe_set_volume = new_syringe_set_volume;
					break;
				case 5: // readPipetteSpeed
					break;
				case 6: // writePipetteSpeed
					break;
				case 7: // cmdAspire
					if (syringe_gpio_cmdAspire(syringe_set_volume * 0.1)) {
						// Placeholder: the holding volume is tracked by higher level process
						syringe_holding_volume = syringe_holding_volume + syringe_set_volume; 
					}
					break;
				case 8: // cmdDispense
					if (syringe_gpio_cmdDispense(syringe_set_volume * 0.1)) {
						// Placeholder: the holding volume is tracked by higher level process
						syringe_holding_volume = syringe_holding_volume - syringe_set_volume;
					}
					break;
				case 9: // cmdDispenseStepVolume
					if (syringe_gpio_cmdDispenseStepVolume(syringe_step_volume * 0.1)) {
						// Placeholder: the holding volume is tracked by higher level process
						syringe_holding_volume = syringe_holding_volume - syringe_step_volume;
					}					
					break;
				case 10: // cmdZero
					break;
				case 80: // cmdExitOnlineMode
					break;
				case 81: // cmdEnterOnlineMode
					break;
				default:
					break;
			}
			new_syringe = false;
		}
		syringe_gpio_help_response();
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	vTaskDelete(NULL);
}


/* -------------------- tanmone uart -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

const int tanmone_uart_buffer_size = 128;
uart_config_t tanmone_uart_config = {
	.baud_rate = 9600,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	.source_clk = UART_SCLK_DEFAULT, 
};

void tanmone_uart_help_response() {

}

void tanmone_uart_task(void *arg) { 
	tanmone_task = 4;
	vTaskDelay(pdMS_TO_TICKS(250)); 
	while (1) {
		switch (tanmone_task) {
			case 1:
				if (tanmone_uart_readpH(tanmone_device_ph, &tanmone_pH_buffer)) {
					tanmone_pH = tanmone_pH_buffer / 100.0;
				}
				break;
			case 2:
				if (tanmone_uart_readTemperature(tanmone_device_ph, &tanmone_temperature_buffer)) {
					tanmone_temperature = tanmone_temperature_buffer / 10.0;
				}
				break;
			case 3:
				if (tanmone_uart_readBatch(tanmone_device_ph, &tanmone_pH_buffer, &tanmone_temperature_buffer)) {
					tanmone_pH = tanmone_pH_buffer / 100.0;
					tanmone_temperature = tanmone_temperature_buffer / 10.0;
				}
//				tanmone_task = 4;
				break;
			case 4:
				if (tanmone_uart_readORP(tanmone_device_orp, &tanmone_ORP_buffer)) {
					tanmone_ORP = tanmone_ORP_buffer; 
				}
//				tanmone_task = 3;
				break;
			default:
				break;
		}
		tanmone_uart_help_response();
		vTaskDelay(pdMS_TO_TICKS(500)); // Note: according to vendor, not less than 500 ms
	}
	vTaskDelete(NULL);
}


/* -------------------- micro ros -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/
/*
void ph_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		ph_msg.data = tanmone_pH;
		RCSOFTCHECK(rcl_publish(&ph_publisher, &ph_msg, NULL));
	}
}

void temperature_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		temperature_msg.data = tanmone_temperature;
		RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
	}
}

void orp_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		orp_msg.data = tanmone_ORP;
		RCSOFTCHECK(rcl_publish(&orp_publisher, &orp_msg, NULL));
	}
}
*/
void tanmone_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		ph_msg.data = tanmone_pH;
		temperature_msg.data = tanmone_temperature;
		orp_msg.data = tanmone_ORP;
		RCSOFTCHECK(rcl_publish(&ph_publisher, &ph_msg, NULL));
		RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
		RCSOFTCHECK(rcl_publish(&orp_publisher, &orp_msg, NULL));
	}
}

void syringecmd_callback(const void * msgin) { 
	syringe_task = syringecmd_msg.data;
	new_syringe = true;
}

void syringesetvol_callback(const void * msgin) { 
	syringe_task = 4; 
	new_syringe_set_volume = syringesetvol_msg.data;
	new_syringe = true;
}

void syringestepvol_callback(const void * msgin) { 
	syringe_step_volume = syringestepvol_msg.data;
}

bool create_entities(void) {
	allocator = rcl_get_default_allocator();
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); // create init_options
	RCCHECK(rclc_node_init_default(&node, "titration_mcu", ROS_NAMESPACE, &support)); // create node
	RCCHECK(rclc_publisher_init_default(&ph_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "ph"));
	RCCHECK(rclc_publisher_init_default(&temperature_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "temperature"));
	RCCHECK(rclc_publisher_init_default(&orp_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "orp"));
	RCCHECK(rclc_subscription_init_default(&syringecmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "syringe_cmd"));
	RCCHECK(rclc_subscription_init_default(&syringesetvol_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "syringe_set_vol"));
	RCCHECK(rclc_subscription_init_default(&syringestepvol_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "syringe_step_vol"));
//	RCCHECK(rclc_timer_init_default2(&ph_timer, &support, RCL_MS_TO_NS(250), ph_timer_callback, true)); 
//	RCCHECK(rclc_timer_init_default2(&temperature_timer, &support, RCL_MS_TO_NS(250), temperature_timer_callback, true)); 
//	RCCHECK(rclc_timer_init_default2(&orp_timer, &support, RCL_MS_TO_NS(250), orp_timer_callback, true)); 
	RCCHECK(rclc_timer_init_default2(&tanmone_timer, &support, RCL_MS_TO_NS(250), tanmone_timer_callback, true)); 
	executor = rclc_executor_get_zero_initialized_executor(); 
	RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); // create executor
//	RCCHECK(rclc_executor_add_timer(&executor, &ph_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &temperature_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &orp_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &tanmone_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &syringecmd_subscriber, &syringecmd_msg, &syringecmd_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &syringesetvol_subscriber, &syringesetvol_msg, &syringesetvol_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &syringestepvol_subscriber, &syringestepvol_msg, &syringestepvol_callback, ON_NEW_DATA));
	return true;
}

void destroy_entities(void) {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	RCCHECK(rcl_publisher_fini(&ph_publisher, &node));
	RCCHECK(rcl_publisher_fini(&temperature_publisher, &node));
	RCCHECK(rcl_publisher_fini(&orp_publisher, &node));
//	RCCHECK(rcl_timer_fini(&ph_timer));
//	RCCHECK(rcl_timer_fini(&temperature_timer));
//	RCCHECK(rcl_timer_fini(&orp_timer));
	RCCHECK(rcl_timer_fini(&tanmone_timer));
	RCCHECK(rcl_subscription_fini(&syringecmd_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&syringesetvol_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&syringestepvol_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));
}

void micro_ros_task(void * arg) {
    while(1) {
        switch (uros_state) {
            case WAITING_AGENT: // Check for agent connection
                uros_state = (RMW_RET_OK == rmw_uros_ping_agent(micro_ros_timeout_ms, micro_ros_attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
                break;
            case AGENT_AVAILABLE: // Create micro-ROS entities
                uros_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (uros_state == WAITING_AGENT) { // Creation failed, release allocated resources
                    destroy_entities();
                };
                break;
            case AGENT_CONNECTED: // Check connection and spin on success
                uros_state = (RMW_RET_OK == rmw_uros_ping_agent(micro_ros_timeout_ms, micro_ros_attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (uros_state == AGENT_CONNECTED) {
                    RCSOFTCHECK(rclc_executor_spin_some(&executor, micro_ros_spin_timeout));
                }
                break;
            case AGENT_DISCONNECTED: // Connection is lost, destroy entities and go back to first step
                destroy_entities();
                uros_state = WAITING_AGENT;
                break;
            default:
                break;
        }
		if (micro_ros_i > 1000) { // micro ros time sync after 5 s
			micro_ros_i = 0;
			rmw_uros_sync_session(micro_ros_sync_session_timeout_ms);
		}
		else micro_ros_i ++;
		vTaskDelay(pdMS_TO_TICKS(5));
    }
	vTaskDelete(NULL);
}


/* -------------------- app main -------------------- -------------------- -------------------- -------------------- -------------------- --------------------
*/

void app_main(void) {
	uros_state = WAITING_AGENT;

	syringe_gpio_init(SYRINGE_GPIO_TAG, STEP_MOTOR_GPIO_STEP, STEP_MOTOR_GPIO_DIR, STEP_MOTOR_GPIO_EN, syringe_feedrate);
	vTaskDelay(pdMS_TO_TICKS(1000));

	ESP_LOGI(TANMONE_UART_TAG,"Install tanmone uart driver");
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, tanmone_uart_buffer_size * 2, 0, 0, NULL, 0)); // no tx buffer, no event queue
	ESP_LOGI(TANMONE_UART_TAG,"Configure tanmone uart parameter");
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &tanmone_uart_config));
	ESP_LOGI(TANMONE_UART_TAG,"Assign signals of tanmone uart peripheral to gpio pins");
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, RS1_TX, RS1_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	ESP_LOGI(TANMONE_UART_TAG, "Discard all data in the tanmone uart rx buffer");
	ESP_ERROR_CHECK(uart_flush(UART_NUM_1));

	ESP_LOGI(SYRINGE_GPIO_TAG, "Create syringe gpio task");
	xTaskCreate(syringe_gpio_task, "syringe_gpio_task", 4096, NULL, 5, NULL);
	ESP_LOGI(TANMONE_UART_TAG, "Create tanmone uart task");
	xTaskCreate(tanmone_uart_task, "tanmone_uart_task", 4096, NULL, 5, NULL);

	#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(true, (void *) &uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write, esp32_serial_read);
	#else
	#error micro-ROS transports misconfigured
	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

	ESP_LOGI(MICRO_ROS_TASK_TAG, "Create micro ros task");
	vTaskDelay(pdMS_TO_TICKS(1000));
	xTaskCreate(micro_ros_task, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}

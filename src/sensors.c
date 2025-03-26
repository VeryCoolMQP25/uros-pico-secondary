#include "sensors.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "uart_logging.h"
#include "pico/sync.h"
#include "math.h"
#define RR_LEN  6
#define ADC_TO_M_CAL 0.0007721693f
#define HEIGHT_DEBUG
float cur_range;
critical_section_t range_crit;

void prepare_lift_height(sensor_msgs__msg__Range *height_message){
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    cur_range = 0.0;
    height_message->min_range = 0.02; // 2cm
    height_message->max_range = 3.0; // 3m (floating range pin)
    height_message->header.frame_id.data = "lift";
    height_message->header.frame_id.size = sizeof("lift");
    height_message->header.frame_id.capacity = sizeof("lift");
    critical_section_init(&range_crit);
}

float get_cur_reading(){
    uint16_t val = adc_read();
    #ifdef HEIGHT_DEBUG
        char heightmessagebuff[60];
        snprintf(heightmessagebuff, 60, "Raw height: %d", val);
        uart_log(LEVEL_DEBUG, heightmessagebuff);
	#endif
    return ((float)val)*ADC_TO_M_CAL;
}

//task executed on core 1
void height_monitor_c1(){
    uart_log(LEVEL_INFO, "Starting core1 task");
    float *reading_rr = malloc(sizeof(float)*RR_LEN);
    uint8_t rr_idx = 0;
    // pre-load round robin
    for(uint8_t i=0; i< RR_LEN; i++){
        reading_rr[i] = get_cur_reading();
    }
    while (1)
    {
        reading_rr[rr_idx++] = get_cur_reading();
        if (rr_idx >= RR_LEN) rr_idx = 0;
        float out= 0.0;
        for(uint8_t i=0; i< RR_LEN; i++){
            out += reading_rr[i];
        }
        out = out/RR_LEN;
        critical_section_enter_blocking(&range_crit);
        cur_range = out;
        critical_section_exit(&range_crit);
        sleep_ms(1);
    }
}

void update_lift_height(sensor_msgs__msg__Range *height_message){
    unsigned long curTime = time_us_64();
    height_message->header.stamp.sec = curTime / 1000000;
    height_message->header.stamp.nanosec = curTime % 1000000;
    critical_section_enter_blocking(&range_crit);
    height_message->range = cur_range;
    critical_section_exit(&range_crit);
}
#include "sensors.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "uart_logging.h"
#include "pico/sync.h"
#include "math.h"
#define RR_LEN  30
#define ADC_TO_M_CAL 22290
#define SMOOTHER_ALPHA 0.1
float cur_range;
critical_section_t range_crit;

void prepare_lift_height(sensor_msgs__msg__Range *height_message){
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    cur_range = 0.0;
    height_message->min_range = 0.02; // 2cm
    height_message->max_range = 3.0; // 3m (floating range pin)
    critical_section_init(&range_crit);
}

float get_cur_reading(){
    uint16_t val = adc_read();
    return (float)val / ADC_TO_M_CAL;
}

//task executed on core 1
void height_monitor_c1(){
    uart_log(LEVEL_INFO, "Starting core1 task");
    float ema = get_cur_reading(); // init exp moving average with reading
    uint16_t rr_idx = 0;
    while (1)
    {
        float new_rdg = get_cur_reading();
        float val = (SMOOTHER_ALPHA*new_rdg) + ((1-SMOOTHER_ALPHA)*ema);
        critical_section_enter_blocking(&range_crit);
        cur_range = val;
        critical_section_exit(&range_crit);
        ema=val;
    }
}

void update_lift_height(sensor_msgs__msg__Range *height_message){
    critical_section_enter_blocking(&range_crit);
    height_message->range = cur_range;
    critical_section_exit(&range_crit);
}
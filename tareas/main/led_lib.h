#ifndef LED_LIB_H
#define LED_LIB_H

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_MODE               LEDC_LOW_SPEED_MODE

typedef struct{
    ledc_channel_t canal;
    int gpio;
    uint32_t duty;
    ledc_timer_bit_t duty_resolution;
}LED;

typedef struct{
    LED led_r;
    LED led_g;
    LED led_b;
    ledc_timer_t  timer_num; 
    uint32_t freq_hz;    
}LED_RGB; 

void config_timer(ledc_timer_t  timer_num, ledc_timer_bit_t duty_resolution,uint32_t freq_hz);
void config_led_rgb( LED_RGB led_to_configure );
void cambiar_intensidad_unico_led( LED *led_to_change, uint8_t duty );
void cambiar_intensidad_led_rgb(LED_RGB *led_to_change, uint8_t duty_red, uint8_t duty_blue, uint8_t duty_green );





#endif
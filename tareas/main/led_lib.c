#include "led_lib.h"


/*
Primera funcion que se debe llamar para configurar el timer que usara el led
*/
void config_timer(ledc_timer_t  timer_num, ledc_timer_bit_t duty_resolution,uint32_t freq_hz){
    // Prepare and then apply the LEDC PWM timer configuration
   ledc_timer_config_t ledc_timer = {
       .speed_mode       = LEDC_MODE,
       .timer_num        = timer_num,
       .duty_resolution  = duty_resolution,
       .freq_hz          = freq_hz,  // Set output frequency at 4 kHz
       .clk_cfg          = LEDC_AUTO_CLK
   };
   ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

void config_led_rgb( LED_RGB led_to_configure ){
    // configure led red
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = led_to_configure.led_r.canal,
        .timer_sel      = led_to_configure.timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led_to_configure.led_r.gpio,
        .duty           = led_to_configure.led_r.duty, // Set duty to 0%
        .hpoint         = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    //configure led green
    ledc_channel.channel = led_to_configure.led_g.canal;
    ledc_channel.gpio_num = led_to_configure.led_g.gpio;
    ledc_channel.duty = led_to_configure.led_g.duty;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // configure led blue
    ledc_channel.channel = led_to_configure.led_b.canal;
    ledc_channel.gpio_num = led_to_configure.led_b.gpio;
    ledc_channel.duty = led_to_configure.led_b.duty;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

}
/*
duty 0 ~ 100
*/

void cambiar_intensidad_unico_led( LED *led_to_change, uint8_t duty ){
    if ( duty > 100 ){
        duty = 100;
    }
    led_to_change -> duty = (int)(( (int)(1 << led_to_change -> duty_resolution) *duty )/100);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, led_to_change -> canal, led_to_change -> duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_to_change -> canal));
}


void cambiar_intensidad_led_rgb( LED_RGB *led_to_change, uint8_t duty_red, uint8_t duty_green, uint8_t duty_blue ){
    cambiar_intensidad_unico_led( &led_to_change -> led_r ,duty_red );
    cambiar_intensidad_unico_led( &led_to_change -> led_g ,duty_green );
    cambiar_intensidad_unico_led( &led_to_change -> led_b ,duty_blue );
}


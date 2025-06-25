/*
Leer temperatura cada 100ms
Leer porcentaje del potenciómetro cada 30ms, usar semáforos y colas
Usar umbrales (thresholds) para cambiar el color del LED RGB, usar un intervalo de 50ms **
Crear tarea que lea el UART y permita cambiar el threshold de cualquiera de los colores

1. tarea que lea temperatura cada  100 ms
2 Una tarea que lea % de potenciometro cada  30 ms
)debe estar impplementado con semaforos y colas)
3 una tarea que controle el color del led basado en  thresholds para color verde rojo y azul  (leer cola de temperatura, leer cola de potenciometro, leer cola de cambio de tresholds)
4 Una tarea que lea el UART y que nos permita cambiar el threshold de cualquiera de los colores
*/


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#define ADC_BANDWIDTH       ADC_BITWIDTH_12
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define POT_CHANNEL         ADC_CHANNEL_5

void app_main(void)
{
    // Estructura del manejador del ADC
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,               // Selección del ADC
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);    // Uniendo configuración y manejador

    // Estrucuta de control de entrada del ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BANDWIDTH,
        .atten = ADC_ATTEN,
    };

    // Enlace del control de entrada con el manejador
    adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &config);
    int pot_raw;            // Variable para la lectura del adc

    while (1)
    {
        esp_err_t ret = adc_oneshot_read(adc_handle, POT_CHANNEL, &pot_raw);        // Lectura del ADC
        if (ret == ESP_OK) {
            printf("ADC raw value: %d\n", pot_raw);
        }
        else {
            printf("ADC error: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}    

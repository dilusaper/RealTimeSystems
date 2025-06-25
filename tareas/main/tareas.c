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
#include "esp_adc/adc_cali.h"


#define ADC_UNIT            ADC_UNIT_1
#define ADC_RESOLUTION      ADC_BITWIDTH_12
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define POT_CHANNEL         ADC_CHANNEL_5
#define MAX_VOLTAJE_MV      3204

adc_oneshot_unit_handle_t adc_handle = NULL;
adc_cali_handle_t cali_handle = NULL;
bool do_calibration = false;

// Estrucuta de control de entrada del ADC
adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_RESOLUTION,
    .atten = ADC_ATTEN,
};

void iniciar_adc (void) {
    // Estructura del manejador del ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,               // Selección del ADC
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);    // Uniendo configuración y manejador
}

// Función para conocer el esquema de calibración de la placa
void esquemas_calibracion(void){
    adc_cali_scheme_ver_t scheme_mask;
    esp_err_t ret = adc_cali_check_scheme(&scheme_mask);

    if (ret == ESP_OK) {
        printf("Esquemas disponibles: \n");
        if (scheme_mask & ADC_CALI_SCHEME_VER_LINE_FITTING) {
            printf(" - Line \n");   // Según mis pruebas, este no está disponible en mi placa
        }
        else if (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) {
            printf(" - Curve \n");
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = ADC_UNIT,
                .atten = ADC_ATTEN,
                .bitwidth = ADC_RESOLUTION,
                .chan = POT_CHANNEL,
            };

            if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK) {
                do_calibration = true;
            }
        }
    }
    else {
        printf("Error al verificar esquemas de calibración: %s", esp_err_to_name(ret));
    }
}

void lectura_potenciometro (void *arg) {
    
    iniciar_adc();

    // Enlace del control de entrada con el manejador
    adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &config);

    esquemas_calibracion();
    while (1)
    {
        int adc_raw;
        int mili_volts;  
        

        esp_err_t ret = adc_oneshot_read(adc_handle, POT_CHANNEL, &adc_raw);
        if (ret == ESP_OK) {
            if (do_calibration) {
                if (adc_cali_raw_to_voltage(cali_handle, adc_raw, &mili_volts) == ESP_OK) {
                    printf("Potenciómetro al %d%%\n", mili_volts * 100 / MAX_VOLTAJE_MV);
                };
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void app_main(void) {
    xTaskCreate(lectura_potenciometro, "Lectura_Pot", 2048, NULL, 5, NULL);
}   

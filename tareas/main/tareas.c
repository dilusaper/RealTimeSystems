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
#include <math.h>
#include "driver/gpio.h"

#define ADC_UNIT            ADC_UNIT_1
#define ADC_RESOLUTION      ADC_BITWIDTH_12
#define ADC_ATTEN           ADC_ATTEN_DB_12

#define POT_CHANNEL         ADC_CHANNEL_5
#define MAX_VOLTAJE_MV      3204

#define NTC_CHANNEL                     ADC_CHANNEL_6
#define NTC_VCC_GPIO                    GPIO_NUM_21
#define NTC_VCC                         3300.0
#define TEMPERATURA_REFERENCIA          25.0
#define FACTOR_CONVERSION               273.15
#define CELCIUS_TO_KELVIN               FACTOR_CONVERSION + TEMPERATURA_REFERENCIA
#define BETA_NTC                        3470
#define R_REF                           10000.0
#define R_MALLA                         10000.0

adc_oneshot_unit_handle_t adc_handle = NULL;
adc_cali_handle_t cali_handle_pot = NULL;
adc_cali_handle_t cali_handle_ntc = NULL;
bool pot_calib = false;
bool ntc_calib = false;


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

// Crear manejadores (handles) para los canales de calibración de cada dispositivo
adc_cali_handle_t crear_calibracion_para_canal(adc_channel_t channel_to_configure, bool *booleano_in) {
    adc_cali_handle_t handle;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_RESOLUTION,
        .chan = channel_to_configure,
    };

    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (err == ESP_OK) {
        printf("✅ Calibración creada para el canal %d\n", channel_to_configure);
        *booleano_in = true;
        return handle;
    } else {
        printf("❌ Error creando calibración para el canal %d: %s\n", channel_to_configure, esp_err_to_name(err));
        return NULL;
    }
}

void lectura_potenciometro (void *arg) {

    // Enlace del control de entrada con el manejador
    adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &config);

    while (1)
    {
        int pot_raw;
        int mili_volts;  
        

        esp_err_t ret = adc_oneshot_read(adc_handle, POT_CHANNEL, &pot_raw);
        if (ret == ESP_OK) {
            if (pot_calib) {
                if (adc_cali_raw_to_voltage(cali_handle_pot, pot_raw, &mili_volts) == ESP_OK) {
                    printf("Potenciómetro al %d%%\n\n", mili_volts * 100 / MAX_VOLTAJE_MV);
                };
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// Configuración de un GPIO como alimentación para mejorar el fucionamiento de la NTC
void configurar_gpio_ntc(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NTC_VCC_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(NTC_VCC_GPIO, 0);  // inicia apagado
}

void temperatura_NTC(void *arg){

    // Enlace del control de entrada con el manejador
    adc_oneshot_config_channel(adc_handle, NTC_CHANNEL, &config);

    gpio_set_level(NTC_VCC_GPIO, 1);    // Activar alimentación
    vTaskDelay(pdMS_TO_TICKS(10));      // Tiempo de espera para estabilización

    while (1)
    {
            int ntc_raw;
            int voltaje_ntc;  
            gpio_set_level(NTC_VCC_GPIO, 1);

            esp_err_t ret = adc_oneshot_read(adc_handle, NTC_CHANNEL, &ntc_raw);
            if (ret == ESP_OK) {
                if (ntc_calib) {
                    if (adc_cali_raw_to_voltage(cali_handle_ntc, ntc_raw, &voltaje_ntc) == ESP_OK) {

                        float R_ntc = (voltaje_ntc * R_MALLA) / (NTC_VCC - voltaje_ntc);
                        float temperatura = (1.0 / (1.0 / (CELCIUS_TO_KELVIN) + (1.0 / BETA_NTC) * log(R_ntc / R_REF))) - FACTOR_CONVERSION;
                        printf("🌡️ Temperatura: %.2f °C\n", temperatura);
                    };
                }
            }
            else {
            printf("❌ Error leyendo el ADC o sin calibración\n");
        };
        gpio_set_level(NTC_VCC_GPIO, 0);    // Apagado luego de la medición

        vTaskDelay(pdMS_TO_TICKS(90));;    // Tiempo hasta la siguiente medición
    }
    
}

void app_main(void) {
    
    iniciar_adc();
    configurar_gpio_ntc();

    cali_handle_pot = crear_calibracion_para_canal(POT_CHANNEL, &pot_calib);
    cali_handle_ntc = crear_calibracion_para_canal(NTC_CHANNEL, &ntc_calib);

    xTaskCreate(lectura_potenciometro, "Lectura_Pot", 2048, NULL, 1, NULL);
    xTaskCreate(temperatura_NTC, "Temperatura_medida", 2048, NULL, 3, NULL);
}
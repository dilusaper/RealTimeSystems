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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "led_lib.h"


// COnfiguración del LED RGB
#define LED_1_R_CHANNEL   LEDC_CHANNEL_0
#define LED_1_G_CHANNEL   LEDC_CHANNEL_1
#define LED_1_B_CHANNEL   LEDC_CHANNEL_2
#define LED_1_R_GPIO      GPIO_NUM_4
#define LED_1_G_GPIO      GPIO_NUM_5
#define LED_1_B_GPIO      GPIO_NUM_6
#define LED_FREQUENCY     10000
#define PWM_RESOLUTION    LEDC_TIMER_10_BIT

// Configuración del ADC
#define ADC_UNIT            ADC_UNIT_1
#define ADC_RESOLUTION      ADC_BITWIDTH_12
#define ADC_ATTEN           ADC_ATTEN_DB_12

// ADC para el potenciómetro
#define POT_CHANNEL         ADC_CHANNEL_0
#define MAX_VOLTAJE_MV      3204

// ADC para el termistor
#define NTC_CHANNEL                     ADC_CHANNEL_1
#define NTC_VCC_GPIO                    GPIO_NUM_21
#define NTC_VCC                         3300.0
#define TEMPERATURA_REFERENCIA          25.0
#define FACTOR_CONVERSION               273.15
#define CELCIUS_TO_KELVIN               FACTOR_CONVERSION + TEMPERATURA_REFERENCIA
#define BETA_NTC                        3470
#define R_REF                           10000.0
#define R_MALLA                         10000.0

// Definiciones para las colas
#define SIZE_QUEUE                  10

typedef struct      // Para mensajes del potenciómetro
{   int porcentaje;
} data_pot;

typedef struct      // Para mensajes de la NTC
{    float temperatura;
} data_temp;

typedef struct      // Para manejo de umbrales de la NTC
{   float inferior;
    float superior;
} data_umbrales;

static data_umbrales umbrales_global = {
    .inferior = 20.0,
    .superior = 30.0 };

// Creación de manejadores
adc_oneshot_unit_handle_t adc_handle = NULL;
adc_cali_handle_t cali_handle_pot = NULL;
adc_cali_handle_t cali_handle_ntc = NULL;
// Configuración para calibración de los ADC
bool calib_pot = false;
bool calib_ntc = false;
// Creación de colas
static QueueHandle_t cola_pot;
static QueueHandle_t cola_ntc;
// Creación del semáforo
static SemaphoreHandle_t semaforo_mutex;

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
};

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
};

void lectura_potenciometro (void *arg) {

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(30);  // 30 ms

    // Enlace del control de entrada con el manejador
    adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &config);

    data_pot msg_pot;       // Creación del mensaje a enviar

    while (1)
    {
        int pot_raw, mili_volts;
        
        if (adc_oneshot_read(adc_handle, POT_CHANNEL, &pot_raw) == ESP_OK && calib_pot &&
            adc_cali_raw_to_voltage(cali_handle_pot, pot_raw, &mili_volts) == ESP_OK) {

            msg_pot.porcentaje = mili_volts * 100 / MAX_VOLTAJE_MV;
            xQueueSend(cola_pot, &msg_pot, portMAX_DELAY);
        }
        vTaskDelayUntil(&last_wake, period);
    }
};

void tarea_consumidor_pot(void *arg) {
    data_pot dato_pot;
    while (1) {
        // Espera indefinida por un nuevo dato de la cola
        if (xQueueReceive(cola_pot, &dato_pot, portMAX_DELAY) == pdPASS) {
            printf("Potenciómetro al %d%%\n\n", dato_pot.porcentaje);
        }
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
};

void temperatura_NTC(void *arg){

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); // 100 ms

    // Enlace del control de entrada con el manejador
    adc_oneshot_config_channel(adc_handle, NTC_CHANNEL, &config);
    
    data_temp msg_temp;

    while (1)
    {
        int ntc_raw, voltaje_ntc;

        gpio_set_level(NTC_VCC_GPIO, 1);    // Activar alimentación
        vTaskDelay(pdMS_TO_TICKS(10));      // Tiempo de espera para estabilización

        if (adc_oneshot_read(adc_handle, NTC_CHANNEL, &ntc_raw) == ESP_OK && calib_ntc &&
            adc_cali_raw_to_voltage(cali_handle_ntc, ntc_raw, &voltaje_ntc) == ESP_OK) {

            float R_ntc = (voltaje_ntc * R_MALLA) / (NTC_VCC - voltaje_ntc);
            float temperatura = (1.0 / (1.0 / (CELCIUS_TO_KELVIN) + (1.0 / BETA_NTC) * log(R_ntc / R_REF))) - FACTOR_CONVERSION;

            msg_temp.temperatura = temperatura;
            xQueueSend(cola_ntc, &msg_temp, portMAX_DELAY);
            printf("Temperatura: %.2f\n", temperatura);
        }

        gpio_set_level(NTC_VCC_GPIO, 0);    // Apagado luego de la medición
        vTaskDelayUntil(&last_wake, period);
    }
    
};

void actualizar_umbrales(float inf, float sup) {
    if (xSemaphoreTake(semaforo_mutex, portMAX_DELAY)) {
        umbrales_global.inferior  = inf;
        umbrales_global.superior  = sup;
        xSemaphoreGive(semaforo_mutex);
    }
}

void color_control(void *arg) {

    LED_RGB led = {
        .led_r = { .canal = LED_1_R_CHANNEL, .gpio = LED_1_R_GPIO, .duty_resolution = PWM_RESOLUTION, .duty = 0 },
        .led_g = { .canal = LED_1_G_CHANNEL, .gpio = LED_1_G_GPIO, .duty_resolution = PWM_RESOLUTION, .duty = 0 },
        .led_b = { .canal = LED_1_B_CHANNEL, .gpio = LED_1_B_GPIO, .duty_resolution = PWM_RESOLUTION, .duty = 0 },
        .timer_num = LEDC_TIMER_0,
        .freq_hz   = LED_FREQUENCY
    };

    config_timer(led.timer_num, led.led_r.duty_resolution, led.freq_hz);
    config_led_rgb(led);

    float temp;
    while(1) 
    {
        if (xQueueReceive(cola_ntc, &temp, portMAX_DELAY) == pdPASS) {
            // copia bajo mutex
            data_umbrales um;
            xSemaphoreTake(semaforo_mutex, portMAX_DELAY);
            um = umbrales_global;
            xSemaphoreGive(semaforo_mutex);

            if (temp <= um.inferior) {
                cambiar_intensidad_led_rgb(&led, 0,0,100);      // Encender azul si es menor al límite inferior
            } else if (temp >= um.superior) {
                cambiar_intensidad_led_rgb(&led, 100,0,0);      // Encender rojo si es mayor al límite superior
            } else {
                cambiar_intensidad_led_rgb(&led, 0,100,0);      // Encender verde si está entre ambos
            }
        }
    }
}

void app_main(void) {

    semaforo_mutex = xSemaphoreCreateMutex();

    cola_pot = xQueueCreate(SIZE_QUEUE, sizeof(data_pot));
    cola_ntc = xQueueCreate(SIZE_QUEUE, sizeof(data_temp));
    
    iniciar_adc();
    configurar_gpio_ntc();

    cali_handle_pot = crear_calibracion_para_canal(POT_CHANNEL, &calib_pot);
    cali_handle_ntc = crear_calibracion_para_canal(NTC_CHANNEL, &calib_ntc);

    xTaskCreate(lectura_potenciometro, "Lectura_Pot", 2048, NULL, 3, NULL);
    xTaskCreate(temperatura_NTC, "Temperatura_medida", 2048, NULL, 1, NULL);
    xTaskCreate(color_control, "Color_RGB", 2048, NULL, 1, NULL);
    xTaskCreate(tarea_consumidor_pot, "Cons_Pot", 2048, NULL, 1, NULL);
}

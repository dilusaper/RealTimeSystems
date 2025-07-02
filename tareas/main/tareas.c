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

// ==== INCLUDES ==== //
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "led_lib.h"

// ==== CONFIGURACIONES GLOBALES ==== //

// ==== UART CONFIG ==== //
#define UART_NUM         UART_NUM_0     // UART por USB para monitoreo y comandos
#define BUF_SIZE         1024           // Tamaño del buffer de recepción UART
#define UART_BAUDRATE    115200         // Velocidad de transmisión UART

// LED RGB: canales PWM y pines GPIO
#define LED_1_R_CHANNEL   LEDC_CHANNEL_0
#define LED_1_G_CHANNEL   LEDC_CHANNEL_1
#define LED_1_B_CHANNEL   LEDC_CHANNEL_2
#define LED_1_R_GPIO      GPIO_NUM_4
#define LED_1_G_GPIO      GPIO_NUM_5
#define LED_1_B_GPIO      GPIO_NUM_6
#define LED_FREQUENCY     10000
#define PWM_RESOLUTION    LEDC_TIMER_10_BIT

// ADC: resolución y atenuación
#define ADC_UNIT          ADC_UNIT_1
#define ADC_RESOLUTION    ADC_BITWIDTH_12
#define ADC_ATTEN         ADC_ATTEN_DB_12

// Canal ADC para el potenciómetro
#define POT_CHANNEL       ADC_CHANNEL_0   // GPIO0 (ajustar si es necesario)
#define MAX_VOLTAJE_MV    3204

// Canal ADC para el termistor NTC
#define NTC_CHANNEL       ADC_CHANNEL_1   // GPIO1 (ajustar si es necesario)
#define NTC_VCC_GPIO      GPIO_NUM_21
#define NTC_VCC           3300.0
#define BETA_NTC          3470
#define R_REF             10000.0
#define R_MALLA           10000.0
#define KELVIN_OFFSET     273.15
#define T0_KELVIN         (25.0 + KELVIN_OFFSET)

// Colas y sincronización
#define SIZE_QUEUE        10

// ==== ESTRUCTURAS DE DATOS ==== //
typedef struct { int porcentaje; } data_pot;
typedef struct { float temperatura; } data_temp;
typedef struct { float inferior; float superior; } data_umbrales;

static data_umbrales umbrales_global = {
    .inferior = 20.0,
    .superior = 30.0 };

// ==== MANEJADORES GLOBALES ==== //
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle_pot = NULL;
static adc_cali_handle_t cali_handle_ntc = NULL;
static SemaphoreHandle_t semaforo_mutex;
static QueueHandle_t cola_pot;
static QueueHandle_t cola_ntc;

// ==== CONFIGURACIÓN DE ADC ==== //
static adc_oneshot_chan_cfg_t config_adc = {
    .bitwidth = ADC_RESOLUTION,
    .atten = ADC_ATTEN,
};

void iniciar_adc(void) {
    adc_oneshot_unit_init_cfg_t cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&cfg, &adc_handle);
}

adc_cali_handle_t crear_calibracion(adc_channel_t canal, bool *ok_flag) {
    adc_cali_handle_t handle;
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_RESOLUTION,
        .chan = canal
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg, &handle) == ESP_OK) {
        *ok_flag = true;
        return handle;
    } else {
        *ok_flag = false;
        return NULL;
    }
}


// ==== CONIFGURACIÓN DEL UART ==== //

void configurar_uart() {        // IA
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ==== FUNCIONES DEL PROGRAMA ==== //

// ==== LECTURA DE POTENCIÓMETRO ==== //
void lectura_potenciometro(void *arg) {
    adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &config_adc);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t periodo = pdMS_TO_TICKS(30);
    data_pot msg;

    while (1) {
        int raw, mv;
        if (adc_oneshot_read(adc_handle, POT_CHANNEL, &raw) == ESP_OK &&
            cali_handle_pot &&
            adc_cali_raw_to_voltage(cali_handle_pot, raw, &mv) == ESP_OK) {

            msg.porcentaje = mv * 100 / MAX_VOLTAJE_MV;
            xQueueSend(cola_pot, &msg, pdMS_TO_TICKS(10));
        }
        vTaskDelayUntil(&last_wake, periodo);
    }
}

void tarea_consumidor_pot(void *arg) {
    data_pot dato;
    while (1) {
        if (xQueueReceive(cola_pot, &dato, portMAX_DELAY)) {
            printf("Potenciometro al %d%%\n\n", dato.porcentaje);
        }
    }
}

// ==== LECTURA DE TEMPERATURA NTC ==== //
void configurar_gpio_ntc(void) {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << NTC_VCC_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
    gpio_set_level(NTC_VCC_GPIO, 0);
}

void temperatura_NTC(void *arg) {
    adc_oneshot_config_channel(adc_handle, NTC_CHANNEL, &config_adc);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t periodo = pdMS_TO_TICKS(100);
    data_temp msg;

    while (1) {
        int raw, mv;
        gpio_set_level(NTC_VCC_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(10));

        if (adc_oneshot_read(adc_handle, NTC_CHANNEL, &raw) == ESP_OK &&
            cali_handle_ntc &&
            adc_cali_raw_to_voltage(cali_handle_ntc, raw, &mv) == ESP_OK) {

            float R_ntc = (mv * R_MALLA) / (NTC_VCC - mv);
            float temp_k = 1.0 / (1.0 / T0_KELVIN + log(R_ntc / R_REF) / BETA_NTC);
            msg.temperatura = temp_k - KELVIN_OFFSET;

            xQueueSend(cola_ntc, &msg, pdMS_TO_TICKS(10));
            printf("🌡️ Temperatura: %.2f °C\n", msg.temperatura);
        }

        gpio_set_level(NTC_VCC_GPIO, 0);
        vTaskDelayUntil(&last_wake, periodo);
    }
}

// ==== CONTROL DE COLOR DEL LED RGB ==== //
void color_control(void *arg) {
    LED_RGB led = {
        .led_r = { .canal = LED_1_R_CHANNEL, .gpio = LED_1_R_GPIO, .duty_resolution = PWM_RESOLUTION },
        .led_g = { .canal = LED_1_G_CHANNEL, .gpio = LED_1_G_GPIO, .duty_resolution = PWM_RESOLUTION },
        .led_b = { .canal = LED_1_B_CHANNEL, .gpio = LED_1_B_GPIO, .duty_resolution = PWM_RESOLUTION },
        .timer_num = LEDC_TIMER_0,
        .freq_hz   = LED_FREQUENCY
    };

    config_timer(led.timer_num, PWM_RESOLUTION, LED_FREQUENCY);
    config_led_rgb(led);

    data_temp dato;
    while (1) {
        if (xQueueReceive(cola_ntc, &dato, portMAX_DELAY)) {
            data_umbrales um;
            xSemaphoreTake(semaforo_mutex, portMAX_DELAY);
            um = umbrales_global;
            xSemaphoreGive(semaforo_mutex);

            if (dato.temperatura <= um.inferior) {
                cambiar_intensidad_led_rgb(&led, 0, 0, 100);  // Azul
            } else if (dato.temperatura >= um.superior) {
                cambiar_intensidad_led_rgb(&led, 100, 0, 0);  // Rojo
            } else {
                cambiar_intensidad_led_rgb(&led, 0, 100, 0);  // Verde
            }
        }
    }
}

// ==== MODIFICAR UMBRALES POR UART ==== //
void actualizar_umbrales(float inf, float sup) {
    if (xSemaphoreTake(semaforo_mutex, portMAX_DELAY)) {
        umbrales_global.inferior = inf;
        umbrales_global.superior = sup;
        xSemaphoreGive(semaforo_mutex);
    }
}


void actualizar_umbrales_uart(void *arg) {      // IA
    uint8_t datos_rx[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_NUM, datos_rx, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            datos_rx[len] = '\0';  // Asegura que sea string válido

            float inf = umbrales_global.inferior;
            float sup = umbrales_global.superior;
            float nuevo_inf = inf, nuevo_sup = sup;

            if (strncmp((char *)datos_rx, "SET", 3) == 0) {
                // Caso SET <inferior> <superior>
                if (sscanf((char *)datos_rx, "SET %f %f", &nuevo_inf, &nuevo_sup) == 2) {
                    if (nuevo_inf < nuevo_sup) {
                        actualizar_umbrales(nuevo_inf, nuevo_sup);
                        printf("✅ Umbrales actualizados: inf=%.2f sup=%.2f\n", nuevo_inf, nuevo_sup);
                    } else {
                        printf("❌ Error: El inferior (%.2f) debe ser menor que el superior (%.2f)\n", nuevo_inf, nuevo_sup);
                    }
                }
                // Caso SET INF <valor>
                else if (sscanf((char *)datos_rx, "SET INF %f", &nuevo_inf) == 1) {
                    if (nuevo_inf < sup) {
                        actualizar_umbrales(nuevo_inf, sup);
                        printf("✅ Inferior actualizado a %.2f\n", nuevo_inf);
                    } else {
                        printf("❌ Error: El nuevo inferior (%.2f) no puede ser mayor o igual al superior (%.2f)\n", nuevo_inf, sup);
                    }
                }
                // Caso SET SUP <valor>
                else if (sscanf((char *)datos_rx, "SET SUP %f", &nuevo_sup) == 1) {
                    if (inf < nuevo_sup) {
                        actualizar_umbrales(inf, nuevo_sup);
                        printf("✅ Superior actualizado a %.2f\n", nuevo_sup);
                    } else {
                        printf("❌ Error: El nuevo superior (%.2f) debe ser mayor que el inferior (%.2f)\n", nuevo_sup, inf);
                    }
                } else {
                    printf("❌ Formato inválido. Usa: SET <inf> <sup>, SET INF <val>, o SET SUP <val>\n");
                }
            }
        }
    }
}


// ==== FUNCIÓN PRINCIPAL ==== //
void app_main(void) {
    
    semaforo_mutex = xSemaphoreCreateMutex();
    cola_pot = xQueueCreate(SIZE_QUEUE, sizeof(data_pot));
    cola_ntc = xQueueCreate(SIZE_QUEUE, sizeof(data_temp));

    iniciar_adc();
    configurar_gpio_ntc();
    configurar_uart();

    bool ok_pot = false, ok_ntc = false;
    cali_handle_pot = crear_calibracion(POT_CHANNEL, &ok_pot);
    cali_handle_ntc = crear_calibracion(NTC_CHANNEL, &ok_ntc);

    xTaskCreate(lectura_potenciometro,  "Lectura_Pot", 2048, NULL, 3, NULL);
    xTaskCreate(tarea_consumidor_pot,   "Consumo_Pot", 2048, NULL, 2, NULL);
    xTaskCreate(temperatura_NTC,        "NTC_temp", 2048, NULL, 2, NULL);
    xTaskCreate(color_control,          "Color_LED", 2048, NULL, 1, NULL);
    xTaskCreate(actualizar_umbrales_uart, "UART_Umbrales", 4096, NULL, 1, NULL);

}
/**
 * 
 * Sistema de control de péndulo dual con motores sobre FreeRTOS
 * Arquitectura de software basada en tareas para el control de un balancín con motores brushless. 
 * Implementa un lazo PID, recepción de Set Point vía Wi-Fi (UDP Sockets), telemetría en tiempo real y un Modo Seguro ante pérdida de enlace.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

// --- LIBRERÍAS WI-FI Y UDP ---
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

//DEFINICIONES DE HARDWARE Y PUERTOS                                   
#define PIN_MOTOR_DER       4    // Pin PWM para el ESC del motor derecho
#define PIN_MOTOR_IZQ       2    // Pin PWM para el ESC del motor izquierdo
#define PIN_LED_ALERTA      21   // Pin para indicador visual de Modo Seguro

// Configuración del ADC 
#define ADC_CANAL           ADC_CHANNEL_6  
#define ADC_UNIDAD          ADC_UNIT_1
#define ADC_RESOLUCION      ADC_BITWIDTH_12 // Resolución de 12 bits (0-4095)
#define ADC_ATENUACION      ADC_ATTEN_DB_12 // Atenuación para 3.3

// Configuración de los  PWM  para los ESCs 
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_14_BIT // Resolución de 14 bits 
#define LEDC_FREQUENCY      50                // Frecuencia para ESCs (50Hz = 20ms)
#define CANAL_MOTOR_DER     LEDC_CHANNEL_0
#define CANAL_MOTOR_IZQ     LEDC_CHANNEL_1


// LÍMITES DE SEGURIDAD Y CALIBRACIÓN DEL CONTROLADOR                     
// Tiempos de pulso en microsegundos
#define PULSO_MIN_ARMADO    1000  // Pulso mínimo para inicializar el ESC sin girar
#define PULSO_PISO_GIRO     1200  // Mínimo empuje para superar fricción inercial
#define PULSO_MAX_SEGURO    1700  // Límite de seguridad 
#define PULSO_MAX_ABSOLUTO  2000  // Máximo pulso físico soportado por el ESC

// Calibración del sensor ADC en bits respecto a la posición física 
#define ADC_IZQ_BITS        676.0f    // Valor ADC a -45 grados
#define ADC_CENTRO_BITS     627.0f    // Valor ADC en reposo (0 grados horizontal)
#define ADC_DER_BITS        576.0f    // Valor ADC a 45 grados
#define ANGULO_IZQ          -45.0f  
#define ANGULO_DER          45.0f    

// Restricciones del algoritmo PID
#define BORDE_INTEGRAL      100   // Límite anti-windup para la ganancia integral
#define MAX_PASO_RAMPA      20    // Delta máximo permitido por ciclo para suavizado

// Macro de conversión: Microsegundos a Ciclo de Trabajo (Duty) de 14 bits
#define US_TO_DUTY(us)      ((us * 16383) / 20000)


// CREDENCIALES DE RED (COMUNICACIÓN MAESTRO-ESCLAVO)     
#define WIFI_SSID           "Trust Fund Baby"     
#define WIFI_PASS           "220403Bz"   
#define UDP_PORT            3333

// ESTRUCTURAS DE DATOS PARA COLAS RTOS          

typedef struct {
    float target_angle;
    float kp_der;
    float kp_izq;
    float ki;
    float kd;
} ControlSettings_t;

typedef struct {
    float current_angle;
    float error;
    uint32_t pwm_izq;
    uint32_t pwm_der;
    int raw_adc;            
    float target_angle;    
} Telemetry_t;

// VARIABLES GLOBALES 
QueueHandle_t xSensorQueue = NULL;    // Cola: Sensor - Controlador (ADC filtrado)
QueueHandle_t xSettingsQueue = NULL;  // Cola: UDP - Controlador (Set Point y PID)
QueueHandle_t xTelemetryQueue = NULL; // Cola: Controlador - Monitor 

adc_oneshot_unit_handle_t adc_handle; // Handle del ADC en ESP-IDF

// Variables globales para el manejo del Watchdog (Modo Seguro) 
volatile TickType_t ultimo_mensaje_udp = 0;
volatile bool modo_seguro_activo = true; // Inicia en falla hasta recibir conexión

// PROTOTIPOS DE FUNCIONES Y TAREAS                                           
float mapear_rango(float x, float in_min, float in_max, float out_min, float out_max);
float acotar_integral(float integral_val);
void inicializar_hardware(void);
void fijar_velocidad_motor(ledc_channel_t canal, uint32_t pulso_us);
void inicializar_wifi(void);

void vTaskSensor(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskUDP_Servidor(void *pvParameters);
void vTaskWatchdog(void *pvParameters);
void vTaskMonitor(void *pvParameters);

// RUTINAS DE INICIALIZACIÓN Y EVENTOS     
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf(" DIRECCION IP DEL ESP32: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        ultimo_mensaje_udp = xTaskGetTickCount(); // Reseteo de seguridad
    }
}

void inicializar_wifi(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


void app_main(void) {
    inicializar_hardware();
    inicializar_wifi();

    // Secuencia de armado de los ESC (mantener pulso mínimo por 3 seg)
    fijar_velocidad_motor(CANAL_MOTOR_DER, PULSO_MIN_ARMADO);
    fijar_velocidad_motor(CANAL_MOTOR_IZQ, PULSO_MIN_ARMADO);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Inicialización de colas de comunicación  
    xSensorQueue    = xQueueCreate(1, sizeof(int));
    xSettingsQueue  = xQueueCreate(1, sizeof(ControlSettings_t));
    xTelemetryQueue = xQueueCreate(1, sizeof(Telemetry_t));

    /* 
     * Planificación de Tareas Concurrentes
     * Prioridades configuradas según requerimientos del proyecto (5 = más alta).
     */
    xTaskCreatePinnedToCore(vTaskSensor,       "SENSOR",   2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vTaskControl,      "CONTROL",  4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vTaskWatchdog,     "WATCHDOG", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vTaskUDP_Servidor, "UDP_SRV",  4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(vTaskMonitor,      "MONITOR",  2048, NULL, 2, NULL, 0);
}

// IMPLEMENTACIÓN DE TAREAS FREE-RTOS       
void vTaskUDP_Servidor(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    for (;;) {
        struct sockaddr_in source_addr; 
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        
        if (len > 0) {
            rx_buffer[len] = 0; 
            float angulo_recibido = 0.0f;
        
            if (sscanf(rx_buffer, "ANGLE:%f", &angulo_recibido) == 1) {
                
                ultimo_mensaje_udp = xTaskGetTickCount(); // Refresca el Watchdog

                ControlSettings_t nueva_config = {
                    .target_angle = angulo_recibido,
                    .kp_der = 1.8f, .kp_izq = 3.6f, 
                    .ki = 0.85f,    .kd = 0.75f     
                };
                // Sobrescribe el set point de manera segura
                xQueueOverwrite(xSettingsQueue, &nueva_config);
            }
        }
    }
}


void vTaskSensor(void *pvParameters) {
    int valor_adc = 0;
    int adc_filtrado = (int)ADC_CENTRO_BITS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);

    for (;;) {
        adc_oneshot_read(adc_handle, ADC_CANAL, &valor_adc);
        
        // Filtro IIR de suavizado  (Alpha = 0.1)
        adc_filtrado = (adc_filtrado * 9 + valor_adc * 1) / 10;

        // Saturación física para evitar fuera de rangos
        if (adc_filtrado > (int)ADC_IZQ_BITS) adc_filtrado = (int)ADC_IZQ_BITS;
        if (adc_filtrado < (int)ADC_DER_BITS) adc_filtrado = (int)ADC_DER_BITS;

        xQueueOverwrite(xSensorQueue, &adc_filtrado);
        
        // Temporización estricta RTOS
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskControl(void *pvParameters) {
    ControlSettings_t config = { .target_angle = 0.0f, .kp_der = 1.8f, .kp_izq = 3.6f, .ki = 0.85f, .kd = 0.75f };
    Telemetry_t telemetria;

    int raw_adc_val = (int)ADC_CENTRO_BITS;
    float angulo_medido = 0.0f, error = 0.0f, error_previo = 0.0f, error_integral = 0.0f;
    const float dt = 0.02f; // Delta de tiempo de integración (20ms)

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);

    for (;;) {
        // Lee la configuración (si no hay nueva, retiene la anterior)
        xQueueReceive(xSettingsQueue, &config, 0);

        // Lee el último ADC y convierte a grados usando interpolación lineal
        if (xQueueReceive(xSensorQueue, &raw_adc_val, 0) == pdTRUE) {
            if ((float)raw_adc_val > ADC_CENTRO_BITS) {
                angulo_medido = mapear_rango((float)raw_adc_val, ADC_CENTRO_BITS, ADC_IZQ_BITS, 0.0f, ANGULO_IZQ);
            } else {
                angulo_medido = mapear_rango((float)raw_adc_val, ADC_CENTRO_BITS, ADC_DER_BITS, 0.0f, ANGULO_DER);
            }
        }

        // GESTIÓN DE SEGURIDAD 
        if (modo_seguro_activo) {
            config.target_angle = 0.0f; // Forzar horizontalidad como emergencia
        }

        // CÁLCULO PID
        error = config.target_angle - angulo_medido;
        
        // Acumulación y restricción Anti-Windup 
        error_integral += error * dt;
        error_integral = acotar_integral(error_integral);
        float termino_integral = config.ki * error_integral;

        // Delta (término D)
        float derivada = (error - error_previo) / dt;
        error_previo = error;

        // Saturación derivativa 
        if (derivada > 150.0f) derivada = 150.0f;
        if (derivada < -150.0f) derivada = -150.0f;
        float termino_derivativo = config.kd * derivada;

        // Ley de control aplicada asimétricamente para equilibrar motores distintos
        float correccion_izq = (config.kp_izq * error) + termino_integral + termino_derivativo;
        float correccion_der = (config.kp_der * error) + termino_integral + termino_derivativo;

        float pulso_izq_objetivo = 1370.0f - correccion_izq;
        float pulso_der_objetivo = 1220.0f + correccion_der;

        // ACCIÓN MITIGATIVA DE MODO SEGURO 
        if (modo_seguro_activo) {
            pulso_izq_objetivo = PULSO_MIN_ARMADO + ((pulso_izq_objetivo - PULSO_MIN_ARMADO) * 0.5f);
            pulso_der_objetivo = PULSO_MIN_ARMADO + ((pulso_der_objetivo - PULSO_MIN_ARMADO) * 0.5f);
        }

        // Restricciones de ventana operativa segura para los ESC
        if (pulso_izq_objetivo < PULSO_PISO_GIRO) pulso_izq_objetivo = PULSO_PISO_GIRO;
        if (pulso_der_objetivo < PULSO_PISO_GIRO) pulso_der_objetivo = PULSO_PISO_GIRO;
        if (pulso_izq_objetivo > PULSO_MAX_SEGURO) pulso_izq_objetivo = PULSO_MAX_SEGURO;
        if (pulso_der_objetivo > PULSO_MAX_SEGURO) pulso_der_objetivo = PULSO_MAX_SEGURO;

        // Salida física al hardware
        fijar_velocidad_motor(CANAL_MOTOR_IZQ, (uint32_t)pulso_izq_objetivo);
        fijar_velocidad_motor(CANAL_MOTOR_DER, (uint32_t)pulso_der_objetivo);

        // Empaquetar y despachar telemetría a la cola 
        telemetria.current_angle = angulo_medido;
        telemetria.target_angle = config.target_angle;
        telemetria.error = error;
        telemetria.pwm_izq = (uint32_t)pulso_izq_objetivo;
        telemetria.pwm_der = (uint32_t)pulso_der_objetivo;
        xQueueOverwrite(xTelemetryQueue, &telemetria);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskWatchdog(void *pvParameters) {
    for (;;) {
        if ((xTaskGetTickCount() - ultimo_mensaje_udp) > pdMS_TO_TICKS(500)) {
            modo_seguro_activo = true;
            gpio_set_level(PIN_LED_ALERTA, 1); 
            vTaskDelay(pdMS_TO_TICKS(250));    // Frecuencia 2 Hz
            gpio_set_level(PIN_LED_ALERTA, 0); 
            vTaskDelay(pdMS_TO_TICKS(250));
        } else {
            modo_seguro_activo = false;
            gpio_set_level(PIN_LED_ALERTA, 0); 
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void vTaskMonitor(void *pvParameters) {
    Telemetry_t telemetria;
    for (;;) {
        if(xQueuePeek(xTelemetryQueue, &telemetria, 0) == pdTRUE) {
            printf("[MONITOR] ESTADO: %s | Ref: %4.1f | Real: %4.1f | Stack Libre: %d bytes\n", 
                   modo_seguro_activo ? "FALLA (MODO SEGURO)" : "OK",
                   telemetria.target_angle, telemetria.current_angle,
                   (int)uxTaskGetStackHighWaterMark(NULL));
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Reporte cada segundo
    }
}
// FUNCIONES DE APOYO MATEMÁTICO Y HARDWARE                      
float mapear_rango(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float acotar_integral(float integral_val) {
    if (integral_val > BORDE_INTEGRAL)  return BORDE_INTEGRAL;
    if (integral_val < -BORDE_INTEGRAL) return -BORDE_INTEGRAL;
    return integral_val;
}

void inicializar_hardware(void) {
    // Configuración GPIO para alerta visual
    gpio_reset_pin(PIN_LED_ALERTA);
    gpio_set_direction(PIN_LED_ALERTA, GPIO_MODE_OUTPUT);

    // Configuración del motor LEDC (PWM)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER, 
        .duty_resolution = LEDC_DUTY_RES, .freq_hz = LEDC_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch_motor_der = {
        .speed_mode = LEDC_MODE, .channel = CANAL_MOTOR_DER, .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE, .gpio_num = PIN_MOTOR_DER, 
        .duty = US_TO_DUTY(PULSO_MIN_ARMADO), .hpoint = 0
    };
    ledc_channel_config(&ch_motor_der);

    ledc_channel_config_t ch_motor_izq_cfg = {
        .speed_mode = LEDC_MODE, .channel = CANAL_MOTOR_IZQ, .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE, .gpio_num = PIN_MOTOR_IZQ, 
        .duty = US_TO_DUTY(PULSO_MIN_ARMADO), .hpoint = 0
    };
    ledc_channel_config(&ch_motor_izq_cfg);

    // Inicialización del Convertidor Analógico-Digital (ADC)
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIDAD };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_RESOLUCION, .atten = ADC_ATENUACION };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CANAL, &config));
}

void fijar_velocidad_motor(ledc_channel_t canal, uint32_t pulso_us) {
    if (pulso_us < PULSO_MIN_ARMADO)   pulso_us = PULSO_MIN_ARMADO;
    if (pulso_us > PULSO_MAX_ABSOLUTO) pulso_us = PULSO_MAX_ABSOLUTO;
    
    uint32_t duty = US_TO_DUTY(pulso_us);
    ledc_set_duty(LEDC_MODE, canal, duty);
    ledc_update_duty(LEDC_MODE, canal);
}
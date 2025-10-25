/**
 * @file hello_world_main.c
 * @brief Orquestrador Principal - GODAR Carrinho Autônomo
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"

// Módulos do projeto
#include "mecatronic.h"
#include "wifi_server.h"
#include "streaming.h"

/**
 * UART Config
 */
#define UART_NUM                     UART_NUM_0
#define UART_BUF_SIZE                1024
#define SERIAL_READ_TIMEOUT_MS       20
#define COMMAND_IDLE_TIMEOUT_MS      500

/**
 * ============================================================
 * TIPOS DE COMANDO (MIDDLEWARE)
 * ============================================================
 */
#define COMMAND_QUEUE_LENGTH    10

typedef enum {
    CMD_NONE,
    CMD_MOTOR,
    CMD_SYNC,
    CMD_SWEEP_START,
    CMD_SWEEP_STOP,
    CMD_ANGLE_MANUAL,
    CMD_CALIBRATION,
    CMD_CAM_BUZZER,
    CMD_CAM_FLASH,
    CMD_CAM_POWER
} control_command_t;

typedef struct {
    control_command_t type;
    union {
        struct {
            motor_axis_t axis;
            motor_direction_t direction;
            int speed;
        } motor;
        struct {
            int value; // speed para SWEEP, angle para MANUAL
        } value;
        motor_axis_t calib_axis;
        struct {
            bool state;
        } cam_control;
    } payload;
} control_message_t;

/**
 * ============================================================
 * VARIÁVEIS GLOBAIS
 * ============================================================
 */
static const char *TAG = "[GODAR]";
static QueueHandle_t control_queue;

// Estados da ESP32-CAM (acessados pelo wifi_server.c)
bool cam_buzzer_state = false;
bool cam_flash_state = false;
bool cam_power_state = false;

/**
 * ============================================================
 * FORWARD DECLARATIONS
 * ============================================================
 */
void control_hub_task(void *pvParameters);
void serial_task(void *pvParameters);
void process_command(char *cmd_str);

/**
 * ============================================================
 * CONTROL HUB TASK (MIDDLEWARE)
 * ============================================================
 */
void control_hub_task(void *pvParameters)
{
    control_message_t msg;
    const int delay_control = 10;
    
    ESP_LOGI(TAG, "Control Hub Task iniciada");
    
    // Estado inicial do servo
    mecatronic_servo_set_angle(ANGLE_CENTER);
    
    while (1) {
        // 1. Processa comandos da Queue
        if (xQueueReceive(control_queue, &msg, 0) == pdPASS) {
            
            switch (msg.type) {
                case CMD_MOTOR:
                    mecatronic_motor_control(msg.payload.motor.axis, 
                                           msg.payload.motor.direction, 
                                           msg.payload.motor.speed);
                    break;
                    
                case CMD_SYNC:
                    mecatronic_motor_sync(msg.payload.motor.direction, 
                                        msg.payload.motor.speed);
                    break;
                    
                case CMD_SWEEP_START:
                    mecatronic_sweep_stop(); // Para o sweep antigo
                    mecatronic_sweep_start(msg.payload.value.value);
                    break;
                    
                case CMD_SWEEP_STOP:
                    mecatronic_sweep_stop();
                    mecatronic_servo_set_target(ANGLE_CENTER);
                    break;
                    
                case CMD_ANGLE_MANUAL:
                    mecatronic_sweep_stop(); // Para sweep se houver
                    mecatronic_servo_set_target(msg.payload.value.value);
                    break;
                    
                case CMD_CALIBRATION:
                    mecatronic_motor_calibration(msg.payload.calib_axis);
                    break;
                
                case CMD_CAM_BUZZER:
                    streaming_set_buzzer(msg.payload.cam_control.state);
                    cam_buzzer_state = msg.payload.cam_control.state;
                    break;
                
                case CMD_CAM_FLASH:
                    streaming_set_flash(msg.payload.cam_control.state);
                    cam_flash_state = msg.payload.cam_control.state;
                    break;
                
                case CMD_CAM_POWER:
                    streaming_set_camera(msg.payload.cam_control.state);
                    cam_power_state = msg.payload.cam_control.state;
                    break;

                default:
                    break;
            }
        }
        
        // 2. Atualiza rampa do servo (se não estiver em sweep)
        if (!mecatronic_sweep_is_active()) {
            mecatronic_servo_update_ramp();
        }
        
        vTaskDelay(pdMS_TO_TICKS(delay_control));
    }
}

/**
 * ============================================================
 * SERIAL TASK (PRODUTOR)
 * ============================================================
 */

void process_command(char *cmd_str)
{
    char temp_buffer[128];
    strncpy(temp_buffer, cmd_str, sizeof(temp_buffer) - 1);
    temp_buffer[sizeof(temp_buffer) - 1] = '\0';
    
    char *token = strtok(temp_buffer, " ");
    control_message_t msg = { .type = CMD_NONE };
    
    // Comando SWEEP
    if (token && strcasecmp(token, "SWEEP") == 0) {
        char *sub_command = strtok(NULL, " ");
        char *speed_str = strtok(NULL, " ");
        
        if (sub_command && strcasecmp(sub_command, "START") == 0) {
            msg.type = CMD_SWEEP_START;
            msg.payload.value.value = speed_str ? (int)strtol(speed_str, NULL, 10) : 5;
        } else if (sub_command && strcasecmp(sub_command, "STOP") == 0) {
            msg.type = CMD_SWEEP_STOP;
        } else {
            ESP_LOGW(TAG, "Comando SWEEP inválido");
            return;
        }
    }
    // Comando MOTOR
    else if (token && strcasecmp(token, "MOTOR") == 0) {
        char *axis_str = strtok(NULL, " ");
        char *dir_str = strtok(NULL, " ");
        char *speed_str = strtok(NULL, " ");
        
        // Comando CALIB
        if (axis_str && dir_str && strcasecmp(dir_str, "CALIB") == 0) {
            msg.type = CMD_CALIBRATION;
            if (strcasecmp(axis_str, "FRONT") == 0) msg.payload.calib_axis = MOTOR_FRONT;
            else if (strcasecmp(axis_str, "REAR") == 0) msg.payload.calib_axis = MOTOR_REAR;
            else return;
        }
        // Comando Normal
        else if (axis_str && dir_str) {
            motor_direction_t direction;
            int speed = 5;
            
            if (strcasecmp(axis_str, "FRONT") == 0) msg.payload.motor.axis = MOTOR_FRONT;
            else if (strcasecmp(axis_str, "REAR") == 0) msg.payload.motor.axis = MOTOR_REAR;
            else { ESP_LOGW(TAG, "Eixo inválido: %s", axis_str); return; }
            
            if (strcasecmp(dir_str, "FORWARD") == 0) direction = MOTOR_FORWARD;
            else if (strcasecmp(dir_str, "BACKWARD") == 0) direction = MOTOR_BACKWARD;
            else if (strcasecmp(dir_str, "STOP") == 0) direction = MOTOR_STOP;
            else { ESP_LOGW(TAG, "Direção inválida: %s", dir_str); return; }
            
            if (speed_str && direction != MOTOR_STOP) speed = (int)strtol(speed_str, NULL, 10);
            
            msg.type = CMD_MOTOR;
            msg.payload.motor.direction = direction;
            msg.payload.motor.speed = speed;
        } else {
            ESP_LOGW(TAG, "Comando MOTOR incompleto.");
            return;
        }
    }
    // Comando SYNC
    else if (token && strcasecmp(token, "SYNC") == 0) {
        char *dir_str = strtok(NULL, " ");
        char *speed_str = strtok(NULL, " ");
        
        if (dir_str) {
            motor_direction_t direction;
            int speed = 5;
            
            if (strcasecmp(dir_str, "FORWARD") == 0) direction = MOTOR_FORWARD;
            else if (strcasecmp(dir_str, "BACKWARD") == 0) direction = MOTOR_BACKWARD;
            else if (strcasecmp(dir_str, "STOP") == 0) direction = MOTOR_STOP;
            else { ESP_LOGW(TAG, "Direção SYNC inválida."); return; }
            
            if (speed_str && direction != MOTOR_STOP) speed = (int)strtol(speed_str, NULL, 10);
            
            msg.type = CMD_SYNC;
            msg.payload.motor.direction = direction;
            msg.payload.motor.speed = speed;
        } else {
            ESP_LOGW(TAG, "Comando SYNC incompleto.");
            return;
        }
    }
    // Comandos ESP32-CAM
    else if (token && strcasecmp(token, "CAM") == 0) {
        char *sub_cmd = strtok(NULL, " ");
        char *state_str = strtok(NULL, " ");
        
        if (sub_cmd && state_str) {
            bool state = (strcasecmp(state_str, "ON") == 0);
            
            if (strcasecmp(sub_cmd, "BUZZER") == 0) {
                msg.type = CMD_CAM_BUZZER;
                msg.payload.cam_control.state = state;
            } else if (strcasecmp(sub_cmd, "FLASH") == 0) {
                msg.type = CMD_CAM_FLASH;
                msg.payload.cam_control.state = state;
            } else if (strcasecmp(sub_cmd, "POWER") == 0) {
                msg.type = CMD_CAM_POWER;
                msg.payload.cam_control.state = state;
            } else {
                ESP_LOGW(TAG, "Comando CAM inválido");
                return;
            }
        } else {
            ESP_LOGW(TAG, "Comando CAM incompleto");
            return;
        }
    }
    // Ângulo manual
    else {
        char *endptr;
        long angle_l = strtol(cmd_str, &endptr, 10);
        int angle = (int)angle_l;
        
        if (*endptr == '\0' || *endptr == ' ' || *endptr == '\t') {
            if (angle >= SWEEP_LIMIT_MIN && angle <= SWEEP_LIMIT_MAX) {
                msg.type = CMD_ANGLE_MANUAL;
                msg.payload.value.value = angle;
            } else {
                ESP_LOGW(TAG, "Ângulo fora do range (%d a %d): %d", 
                         SWEEP_LIMIT_MIN, SWEEP_LIMIT_MAX, angle);
                return;
            }
        } else {
            ESP_LOGW(TAG, "Comando inválido: '%s'", cmd_str);
            return;
        }
    }

    // Envia comando para o Hub
    if (msg.type != CMD_NONE) {
        if (xQueueSend(control_queue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
            ESP_LOGE(TAG, "Queue de controle cheia. Comando descartado.");
        }
    }
}

void serial_task(void *pvParameters)
{
    char data_buffer[128];
    int current_pos = 0;
    TickType_t last_char_time = 0;
    
    while (1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t *)&data_buffer[current_pos], 1, 
                                  pdMS_TO_TICKS(SERIAL_READ_TIMEOUT_MS));
        
        if (len > 0) {
            last_char_time = xTaskGetTickCount();
            uart_write_bytes(UART_NUM, (const char *)&data_buffer[current_pos], 1);
            
            if (data_buffer[current_pos] == '\n' || data_buffer[current_pos] == '\r') {
                uart_write_bytes(UART_NUM, "\r\n", 2);
                
                data_buffer[current_pos] = '\0';
                char *ptr = data_buffer;
                while (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r') ptr++;
                
                if (strlen(ptr) > 0) {
                    process_command(ptr);
                }
                
                current_pos = 0;
                memset(data_buffer, 0, sizeof(data_buffer));
                last_char_time = 0;
            } else {
                current_pos++;
                if (current_pos >= sizeof(data_buffer) - 1) {
                    ESP_LOGE(TAG, "Buffer Serial Overflow!");
                    current_pos = 0;
                    memset(data_buffer, 0, sizeof(data_buffer));
                    last_char_time = 0;
                }
            }
        } else {
            if (current_pos > 0 && last_char_time > 0) {
                TickType_t now = xTaskGetTickCount();
                TickType_t elapsed_ms = (now - last_char_time) * portTICK_PERIOD_MS;
                
                if (elapsed_ms >= COMMAND_IDLE_TIMEOUT_MS) {
                    uart_write_bytes(UART_NUM, "\r\n", 2);
                    data_buffer[current_pos] = '\0';
                    
                    char *ptr = data_buffer;
                    while (*ptr == ' ' || *ptr == '\t') ptr++;
                    
                    if (strlen(ptr) > 0) {
                        ESP_LOGI(TAG, "Comando processado por timeout: '%s'", ptr);
                        process_command(ptr);
                    }
                    
                    current_pos = 0;
                    memset(data_buffer, 0, sizeof(data_buffer));
                    last_char_time = 0;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * ============================================================
 * APP_MAIN
 * ============================================================
 */
void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  GODAR - Carrinho Autônomo ESP32-C3");
    ESP_LOGI(TAG, "  Arquitetura Modular v2.0");
    ESP_LOGI(TAG, "============================================");
    
    // UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    // Inicialização de Hardware (Mecatrônica)
    mecatronic_motor_init();
    mecatronic_motor_stop_all();
    mecatronic_servo_init();
    
    // Criação da Queue do Middleware
    control_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(control_message_t));
    if (control_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar Queue de Controle!");
        return;
    }

    // Inicialização WiFi
    wifi_init_softap();
    start_webserver();
    
    // Inicialização Streaming (ESP32-CAM)
    streaming_init();
    streaming_start_poll_task();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Comandos disponíveis:");
    ESP_LOGI(TAG, "  Servo: -90 a 90");
    ESP_LOGI(TAG, "  SWEEP START <1-10> | SWEEP STOP");
    ESP_LOGI(TAG, "  MOTOR <front|rear> <forward|backward|stop> [1-10]");
    ESP_LOGI(TAG, "  MOTOR <front|rear> CALIB");
    ESP_LOGI(TAG, "  SYNC <forward|backward|stop> [1-10]");
    ESP_LOGI(TAG, "  CAM BUZZER <on|off>");
    ESP_LOGI(TAG, "  CAM FLASH <on|off>");
    ESP_LOGI(TAG, "  CAM POWER <on|off>");
    ESP_LOGI(TAG, "");

    // Tasks
    xTaskCreate(serial_task, "serial_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_hub_task, "control_hub_task", 4096, NULL, 6, NULL);
    
    // Loop principal
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        streaming_sensors_t sensors;
        if (streaming_get_sensors(&sensors)) {
            ESP_LOGI(TAG, "Status: Servo=%d°, CAM Front=%dcm, Rear=%dcm", 
                     mecatronic_servo_get_angle(), sensors.front_cm, sensors.rear_cm);
        } else {
            ESP_LOGI(TAG, "Status: Servo=%d° (ESP32-CAM desconectado)", 
                     mecatronic_servo_get_angle());
        }
    }
}
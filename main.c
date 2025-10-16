/**
 * AUTHOR: VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * DATE: 2025-10-16
*/

/**
 * CONTROLE DE CARRINHO AUTÔNOMO - ESP32-C3 DevKitM-1
 * Versão refatorada com MIDDLEWARE (Control Hub Task) para resolver concorrência.
 */

/**
 * CONFIGURAÇÃO DE HARDWARE - Escolha UMA das opções abaixo:
 */
// #define JUMPER_SPEED_MOTOR_ON   // Jumpers ENA/ENB instalados na L298N (velocidade fixa, sem PWM)
#define JUMPER_SPEED_MOTOR_OFF     // Jumpers ENA/ENB removidos (velocidade controlada por PWM)

/**
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Adicionado para o Middleware
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

/**
 * DEFINES - Pinos ESP32-C3 DevKitM-1
 */
#define GPIO_ULTRASONIC_TRIGGER      5
#define GPIO_ULTRASONIC_ECHO         6
#define GPIO_SERVO_DIRECTION         9
#define GPIO_MOTOR_FRONT_PWM         10
#define GPIO_MOTOR_FRONT_IN1         7
#define GPIO_MOTOR_FRONT_IN2         8
#define GPIO_MOTOR_REAR_PWM          4
#define GPIO_MOTOR_REAR_IN3          18
#define GPIO_MOTOR_REAR_IN4          19

/**
 * Ângulos do Servo
 */
#define ANGLE_MIN                    -90
#define ANGLE_MAX                    90
#define ANGLE_CENTER                 0

/**
 * PWM Config (Servo)
 */
#define SERVO_LEDC_TIMER             LEDC_TIMER_0
#define SERVO_LEDC_MODE              LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL           LEDC_CHANNEL_0
#define SERVO_LEDC_DUTY_RES          LEDC_TIMER_14_BIT
#define SERVO_LEDC_FREQUENCY         50
#define MAX_DUTY_VALUE               16383
#define DUTY_PULSE_MIN               410
#define DUTY_PULSE_MAX               1228

/**
 * SWEEP Config
 */
#define SWEEP_LIMIT_MIN              -90
#define SWEEP_LIMIT_MAX              90
#define SWEEP_STEP_DELAY_MS_MIN      5
#define SWEEP_STEP_DELAY_MS_MAX      50
#define SWEEP_STEP_DEGREES           1

/**
 * PWM Config (Motores) - Usado apenas se JUMPER_SPEED_MOTOR_OFF
 */
#define MOTOR_LEDC_TIMER             LEDC_TIMER_1
#define MOTOR_LEDC_MODE              LEDC_LOW_SPEED_MODE
#define MOTOR_FRONT_LEDC_CHANNEL     LEDC_CHANNEL_1
#define MOTOR_REAR_LEDC_CHANNEL      LEDC_CHANNEL_2
#define MOTOR_LEDC_DUTY_RES          LEDC_TIMER_8_BIT  // 0-255
#define MOTOR_LEDC_FREQUENCY         1000              // 1kHz

/**
 * Motor Speed Range (1-10 convertido para PWM 0-255)
 * CALIBRAÇÃO INDIVIDUAL por eixo
 */
#define MOTOR_SPEED_MIN              1
#define MOTOR_SPEED_MAX              10

// PWM FRONT (1 motor sozinho - menos carga)
#define MOTOR_PWM_MIN_FRONT          60    // Ajuste conforme necessário
#define MOTOR_PWM_MAX_FRONT          255

// PWM REAR (2 motores em paralelo - mais carga)
#define MOTOR_PWM_MIN_REAR           100   // Precisa de mais força inicial
#define MOTOR_PWM_MAX_REAR           255

/**
 * UART Config
 */
#define UART_NUM                     UART_NUM_0
#define UART_BUF_SIZE                1024
#define SERIAL_READ_TIMEOUT_MS       20
#define COMMAND_IDLE_TIMEOUT_MS      500   // Processa comando após 500ms de inatividade

/**
 * ENUMS
 */
typedef enum {
    MOTOR_FRONT,
    MOTOR_REAR
} motor_axis_t;

typedef enum {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} motor_direction_t;

// ====================================================================
//                          ESTRUTURA MIDDLEWARE (COMANDO)
// ====================================================================

#define COMMAND_QUEUE_LENGTH    10 // Aumentado para 10

typedef enum {
    CMD_NONE,
    CMD_MOTOR,
    CMD_SYNC,
    CMD_SWEEP_START,
    CMD_SWEEP_STOP,
    CMD_ANGLE_MANUAL,
    CMD_CALIBRATION
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
    } payload;
} control_message_t;

/**
 * GLOBAL VARIABLES
 * Variáveis de estado agora controladas/escritas SOMENTE pela control_hub_task
 */
static const char *TAG = "[GODAR]";
static int current_angle = ANGLE_CENTER;
static int angle_target = ANGLE_CENTER;
static TaskHandle_t sweep_task_handle = NULL;
static int sweep_delay_ms = 10;
static QueueHandle_t control_queue; // Handle da Queue de Mensagens

/**
 * FORWARD DECLARATIONS (Atualizadas)
 */
void servo_set_angle(int angle);
void sweep_test_task(void *pvParameters);
void control_hub_task(void *pvParameters); // Nova Task (Middleware)
// car_control_task removida
void servo_init(void);
void motor_gpio_init(void);
void motor_control(motor_axis_t axis, motor_direction_t direction, int speed);
void motor_stop_all(void);
void motor_sync(motor_direction_t direction, int speed);
void motor_calibration(motor_axis_t axis);
void set_sweep_speed(int speed);
void start_sweep_test(int speed);
void stop_sweep_test(void);
void update_target_angle(int new_angle);
void process_command(char *cmd_str);

/**
 * ============================================================
 * SERVO FUNCTIONS (Mantidas, agora chamadas pelo HUB)
 * ============================================================
 */
// ... (angle_to_duty permanece inalterada)
uint32_t angle_to_duty(int angle)
{
    if (angle < ANGLE_MIN) angle = ANGLE_MIN;
    if (angle > ANGLE_MAX) angle = ANGLE_MAX;
    uint32_t duty_range = DUTY_PULSE_MAX - DUTY_PULSE_MIN;
    int angle_range = ANGLE_MAX - ANGLE_MIN;
    int normalized_angle = angle - ANGLE_MIN;
    uint32_t duty_offset = (normalized_angle * duty_range) / angle_range;
    uint32_t duty = DUTY_PULSE_MAX - duty_offset;
    return duty;
}

void servo_set_angle(int angle)
{
    // Apenas a control_hub_task e a sweep_test_task (quando rodando)
    // chamam esta função, eliminando a necessidade de Critical Section AQUI.
    if (angle < ANGLE_MIN || angle > ANGLE_MAX) {
        ESP_LOGW(TAG, "Ângulo fora do range: %d", angle);
        return;
    }
    uint32_t duty = angle_to_duty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL));
    current_angle = angle;
}

void servo_init(void)
{
    // ... (servo_init permanece inalterada)
    gpio_reset_pin(GPIO_SERVO_DIRECTION);
    gpio_set_direction(GPIO_SERVO_DIRECTION, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_SERVO_DIRECTION, GPIO_PULLDOWN_ONLY);
    gpio_set_level(GPIO_SERVO_DIRECTION, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = SERVO_LEDC_MODE,
        .duty_resolution  = SERVO_LEDC_DUTY_RES,
        .timer_num        = SERVO_LEDC_TIMER,
        .freq_hz          = SERVO_LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = GPIO_SERVO_DIRECTION,
        .speed_mode     = SERVO_LEDC_MODE,
        .channel        = SERVO_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = SERVO_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "Servo inicializado no GPIO %d", GPIO_SERVO_DIRECTION);
    servo_set_angle(ANGLE_CENTER);
}

/**
 * ============================================================
 * MOTOR FUNCTIONS (Mantidas, agora chamadas pelo HUB)
 * ============================================================
 */

void motor_gpio_init(void)
{
    // ... (motor_gpio_init permanece inalterada)
    gpio_reset_pin(GPIO_MOTOR_FRONT_IN1);
    gpio_reset_pin(GPIO_MOTOR_FRONT_IN2);
    gpio_reset_pin(GPIO_MOTOR_REAR_IN3);
    gpio_reset_pin(GPIO_MOTOR_REAR_IN4);
    gpio_reset_pin(GPIO_MOTOR_FRONT_PWM);
    gpio_reset_pin(GPIO_MOTOR_REAR_PWM);

    // Configura como OUTPUT
    gpio_set_direction(GPIO_MOTOR_FRONT_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_FRONT_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_REAR_IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_REAR_IN4, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_FRONT_PWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_REAR_PWM, GPIO_MODE_OUTPUT);

    // FORÇA todos os pinos para LOW (ESTADO SEGURO - DESLIGADO)
    gpio_set_level(GPIO_MOTOR_FRONT_IN1, 0);
    gpio_set_level(GPIO_MOTOR_FRONT_IN2, 0);
    gpio_set_level(GPIO_MOTOR_REAR_IN3, 0);
    gpio_set_level(GPIO_MOTOR_REAR_IN4, 0);
    gpio_set_level(GPIO_MOTOR_FRONT_PWM, 0);
    gpio_set_level(GPIO_MOTOR_REAR_PWM, 0);

    // Adiciona pull-down para garantir LOW no boot
    gpio_set_pull_mode(GPIO_MOTOR_FRONT_IN1, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_MOTOR_FRONT_IN2, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_MOTOR_REAR_IN3, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_MOTOR_REAR_IN4, GPIO_PULLDOWN_ONLY);

#ifdef JUMPER_SPEED_MOTOR_OFF
    // Se jumpers estão REMOVIDOS, configura PWM para controle de velocidade
    ledc_timer_config_t motor_timer = {
        .speed_mode       = MOTOR_LEDC_MODE,
        .duty_resolution  = MOTOR_LEDC_DUTY_RES,
        .timer_num        = MOTOR_LEDC_TIMER,
        .freq_hz          = MOTOR_LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    ledc_channel_config_t motor_front_channel = {
        .gpio_num       = GPIO_MOTOR_FRONT_PWM,
        .speed_mode     = MOTOR_LEDC_MODE,
        .channel        = MOTOR_FRONT_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_front_channel));

    ledc_channel_config_t motor_rear_channel = {
        .gpio_num       = GPIO_MOTOR_REAR_PWM,
        .speed_mode     = MOTOR_LEDC_MODE,
        .channel        = MOTOR_REAR_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_rear_channel));

    ESP_LOGI(TAG, "Motores inicializados com PWM (Jumpers REMOVIDOS)");
#else
    // Se jumpers estão INSTALADOS, mantém ENA/ENB em LOW (DESLIGADO)
    gpio_set_level(GPIO_MOTOR_FRONT_PWM, 0);
    gpio_set_level(GPIO_MOTOR_REAR_PWM, 0);
    ESP_LOGI(TAG, "Motores inicializados DESLIGADOS (Jumpers INSTALADOS)");
#endif

    ESP_LOGI(TAG, "Todos os motores inicializados em estado DESLIGADO");
}

void motor_control(motor_axis_t axis, motor_direction_t direction, int speed)
{
    // ... (motor_control permanece inalterada, é chamada somente pelo HUB)
    if (speed < MOTOR_SPEED_MIN) speed = MOTOR_SPEED_MIN;
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;

    int pwm_min = (axis == MOTOR_FRONT) ? MOTOR_PWM_MIN_FRONT : MOTOR_PWM_MIN_REAR;
    int pwm_max = (axis == MOTOR_FRONT) ? MOTOR_PWM_MAX_FRONT : MOTOR_PWM_MAX_REAR;

    int pwm_range = pwm_max - pwm_min;
    int pwm_value = pwm_min + ((speed - 1) * pwm_range) / (MOTOR_SPEED_MAX - 1);

    const char *axis_name = (axis == MOTOR_FRONT) ? "FRONT" : "REAR";
    const char *dir_name = (direction == MOTOR_FORWARD) ? "FORWARD" : 
                          (direction == MOTOR_BACKWARD) ? "BACKWARD" : "STOP";

    if (axis == MOTOR_FRONT) {
        if (direction == MOTOR_FORWARD) {
            gpio_set_level(GPIO_MOTOR_FRONT_IN1, 1);
            gpio_set_level(GPIO_MOTOR_FRONT_IN2, 0);
        } else if (direction == MOTOR_BACKWARD) {
            gpio_set_level(GPIO_MOTOR_FRONT_IN1, 0);
            gpio_set_level(GPIO_MOTOR_FRONT_IN2, 1);
        } else { // STOP
            gpio_set_level(GPIO_MOTOR_FRONT_IN1, 0);
            gpio_set_level(GPIO_MOTOR_FRONT_IN2, 0);
            pwm_value = 0;
        }

#ifdef JUMPER_SPEED_MOTOR_OFF
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_FRONT_LEDC_CHANNEL, pwm_value);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_FRONT_LEDC_CHANNEL);
        ESP_LOGI(TAG, "Motor %s: %s - Speed=%d (PWM=%d)", axis_name, dir_name, speed, pwm_value);
#else
        gpio_set_level(GPIO_MOTOR_FRONT_PWM, (direction != MOTOR_STOP) ? 1 : 0);
        ESP_LOGI(TAG, "Motor %s: %s - Velocidade FIXA (jumper)", axis_name, dir_name);
#endif

    } else { // MOTOR_REAR
        if (direction == MOTOR_FORWARD) {
            gpio_set_level(GPIO_MOTOR_REAR_IN3, 1);
            gpio_set_level(GPIO_MOTOR_REAR_IN4, 0);
        } else if (direction == MOTOR_BACKWARD) {
            gpio_set_level(GPIO_MOTOR_REAR_IN3, 0);
            gpio_set_level(GPIO_MOTOR_REAR_IN4, 1);
        } else { // STOP
            gpio_set_level(GPIO_MOTOR_REAR_IN3, 0);
            gpio_set_level(GPIO_MOTOR_REAR_IN4, 0);
            pwm_value = 0;
        }

#ifdef JUMPER_SPEED_MOTOR_OFF
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_REAR_LEDC_CHANNEL, pwm_value);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_REAR_LEDC_CHANNEL);
        ESP_LOGI(TAG, "Motor %s: %s - Speed=%d (PWM=%d)", axis_name, dir_name, speed, pwm_value);
#else
        gpio_set_level(GPIO_MOTOR_REAR_PWM, (direction != MOTOR_STOP) ? 1 : 0);
        ESP_LOGI(TAG, "Motor %s: %s - Velocidade FIXA (jumper)", axis_name, dir_name);
#endif
    }
}

void motor_stop_all(void)
{
    // ... (motor_stop_all permanece inalterada)
    motor_control(MOTOR_FRONT, MOTOR_STOP, 0);
    motor_control(MOTOR_REAR, MOTOR_STOP, 0);
    ESP_LOGI(TAG, "TODOS os motores PARADOS");
}

void motor_sync(motor_direction_t direction, int speed)
{
    // ... (motor_sync permanece inalterada)
    if (speed < MOTOR_SPEED_MIN) speed = MOTOR_SPEED_MIN;
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
    
    const char *dir_name = (direction == MOTOR_FORWARD) ? "FORWARD" : 
                          (direction == MOTOR_BACKWARD) ? "BACKWARD" : "STOP";
    
    ESP_LOGI(TAG, "=== SYNC MOTORS ===");
    ESP_LOGI(TAG, "Direção: %s, Speed Abstrato: %d", dir_name, speed);
    
    motor_control(MOTOR_FRONT, direction, speed);
    motor_control(MOTOR_REAR, direction, speed);
    
    ESP_LOGI(TAG, "=== SYNC COMPLETO ===");
}

void motor_calibration(motor_axis_t axis)
{
    // ... (motor_calibration permanece inalterada)
    const char *axis_name = (axis == MOTOR_FRONT) ? "FRONT" : "REAR";
    int pwm_min = (axis == MOTOR_FRONT) ? MOTOR_PWM_MIN_FRONT : MOTOR_PWM_MIN_REAR;
    int pwm_max = (axis == MOTOR_FRONT) ? MOTOR_PWM_MAX_FRONT : MOTOR_PWM_MAX_REAR;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  CALIBRAÇÃO MOTOR %s", axis_name);
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "PWM_MIN: %d", pwm_min);
    ESP_LOGI(TAG, "PWM_MAX: %d", pwm_max);
    ESP_LOGI(TAG, "--------------------------------------");
    ESP_LOGI(TAG, "Speed | PWM  | Percentual");
    ESP_LOGI(TAG, "--------------------------------------");
    
    int pwm_range = pwm_max - pwm_min;
    for (int speed = 1; speed <= 10; speed++) {
        int pwm = pwm_min + ((speed - 1) * pwm_range) / 9;
        int percent = (pwm * 100) / 255;
        ESP_LOGI(TAG, "  %2d  | %3d  |   %3d%%", speed, pwm, percent);
    }
    
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "");
}

/**
 * ============================================================
 * SWEEP & SERVO CONTROL (Refatorado para o HUB)
 * ============================================================
 */

void update_target_angle(int new_angle)
{
    // Não precisa de Critical Section, pois SÓ o HUB (ou sweep, se ativo) altera
    if (new_angle < SWEEP_LIMIT_MIN) new_angle = SWEEP_LIMIT_MIN;
    if (new_angle > SWEEP_LIMIT_MAX) new_angle = SWEEP_LIMIT_MAX;
    angle_target = new_angle;
}

void set_sweep_speed(int speed)
{
    // Não precisa de Critical Section, pois SÓ o HUB a altera
    if (speed < 1) speed = 1;
    if (speed > 10) speed = 10;
    int delay_range = SWEEP_STEP_DELAY_MS_MAX - SWEEP_STEP_DELAY_MS_MIN;
    float normalized_speed = (float)(speed - 1) / 9.0;
    float curve_factor = pow(1.0 - normalized_speed, 2.0);
    sweep_delay_ms = SWEEP_STEP_DELAY_MS_MIN + (int)(delay_range * curve_factor);
    ESP_LOGI(TAG, "Velocidade SWEEP: %d (Delay: %dms)", speed, sweep_delay_ms);
}

// Funções start/stop movidas para dentro do HUB ou chamadas por ele
// Para evitar concorrência no 'sweep_task_handle', estas funções SÃO CHAMADAS APENAS PELO HUB.

void stop_sweep_test(void)
{
    // Chamado EXCLUSIVAMENTE pelo HUB
    if (sweep_task_handle != NULL) {
        vTaskDelete(sweep_task_handle);
        sweep_task_handle = NULL;
        update_target_angle(ANGLE_CENTER); // Seta alvo para o HUB estabilizar o servo
        ESP_LOGI(TAG, "Sweep Test PARADO - Retornando ao centro");
    }
}

void start_sweep_test(int speed)
{
    // Chamado EXCLUSIVAMENTE pelo HUB
    set_sweep_speed(speed);
    if (sweep_task_handle == NULL) {
        xTaskCreate(sweep_test_task, "sweep_test_task", 4096, NULL, 5, &sweep_task_handle);
        ESP_LOGI(TAG, "Sweep Test INICIADO");
    } else {
        ESP_LOGW(TAG, "Sweep Test já rodando. Reconfigurando velocidade.");
    }
}

void sweep_test_task(void *pvParameters)
{
    int direction = 1;
    while (1) {
        int next_angle = current_angle + (SWEEP_STEP_DEGREES * direction);
        
        if (next_angle > SWEEP_LIMIT_MAX) {
            direction = -1;
            next_angle = SWEEP_LIMIT_MAX;
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (next_angle < SWEEP_LIMIT_MIN) {
            direction = 1;
            next_angle = SWEEP_LIMIT_MIN;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        servo_set_angle(next_angle); // Atualiza current_angle
        vTaskDelay(pdMS_TO_TICKS(sweep_delay_ms)); // Usa o delay definido pelo HUB
    }
}

// ====================================================================
//                       MIDDLEWARE: CONTROL HUB TASK
// ====================================================================

/**
 * *************************************************************************
 * INÍCIO DO MIDDLEWARE: control_hub_task
 * * Função: Este é o ponto central de controle do carrinho. Ele atua como um
 * consumidor dos comandos da Queue, garantindo que todas as mudanças de
 * ESTADO (sweep, angle_target) e chamadas de motor/servo sejam feitas
 * SEQUENCIALMENTE dentro do contexto desta task.
 * * Isto ELIMINA todas as race conditions que ocorriam quando a serial_task
 * (produtor) interferia diretamente na lógica de controle (consumidor).
 * * A LÓGICA DE RAMPA (antiga car_control_task) foi integrada aqui.
 * *************************************************************************
 */
void control_hub_task(void *pvParameters)
{
    control_message_t msg;
    const int step = SWEEP_STEP_DEGREES;
    const int delay_control = 10; // Frequência da rampa de controle

    // Garante estado inicial do servo
    servo_set_angle(ANGLE_CENTER); 
    
    while (1) {
        // 1. Processa um comando da Queue (prioridade sobre a rampa)
        if (xQueueReceive(control_queue, &msg, 0) == pdPASS) { 
            
            switch (msg.type) {
                case CMD_MOTOR:
                    motor_control(msg.payload.motor.axis, msg.payload.motor.direction, msg.payload.motor.speed);
                    break;
                    
                case CMD_SYNC:
                    motor_sync(msg.payload.motor.direction, msg.payload.motor.speed);
                    break;
                    
                case CMD_SWEEP_START:
                    stop_sweep_test(); // Para o sweep antigo, se houver
                    // start_sweep_test manipula o handle de task
                    start_sweep_test(msg.payload.value.value); 
                    break;
                    
                case CMD_SWEEP_STOP:
                    stop_sweep_test();
                    angle_target = ANGLE_CENTER; // Força retorno ao centro
                    break;
                    
                case CMD_ANGLE_MANUAL:
                    stop_sweep_test(); // Para o sweep se um comando de ângulo manual for enviado
                    update_target_angle(msg.payload.value.value);
                    break;
                    
                case CMD_CALIBRATION:
                    motor_calibration(msg.payload.calib_axis);
                    break;

                default:
                    // CMD_NONE
                    break;
            }
        }
        
        // 2. Lógica de Rampa (Substitui car_control_task)
        // Só executa se o sweep NÃO estiver ativo.
        if (sweep_task_handle == NULL) { 
            if (current_angle != angle_target) {
                int direction = (angle_target > current_angle) ? step : -step;
                int next_angle = current_angle + direction;
                
                // Limita para evitar overshoot
                if ((direction > 0 && next_angle > angle_target) ||
                    (direction < 0 && next_angle < angle_target))
                {
                    next_angle = angle_target;
                }
                
                servo_set_angle(next_angle);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(delay_control)); // Frequência do loop
    }
}
/**
 * *************************************************************************
 * FIM DO MIDDLEWARE: control_hub_task
 * *************************************************************************
 */

/**
 * ============================================================
 * SERIAL/UART TASK (PRODUTOR)
 * ============================================================
 */

/**
 * Processa um comando recebido via UART
 * Esta função agora apenas MONTA a mensagem e ENVIA para a Queue.
 * A EXECUÇÃO do comando é feita pela control_hub_task.
 */
void process_command(char *cmd_str)
{
    char temp_buffer[128];
    strncpy(temp_buffer, cmd_str, sizeof(temp_buffer) - 1);
    temp_buffer[sizeof(temp_buffer) - 1] = '\0';
    
    char *token = strtok(temp_buffer, " ");
    control_message_t msg = { .type = CMD_NONE }; // Inicia como NONE
    
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
            
            // Parse axis
            if (strcasecmp(axis_str, "FRONT") == 0) msg.payload.motor.axis = MOTOR_FRONT;
            else if (strcasecmp(axis_str, "REAR") == 0) msg.payload.motor.axis = MOTOR_REAR;
            else { ESP_LOGW(TAG, "Eixo inválido: %s", axis_str); return; }
            
            // Parse direction
            if (strcasecmp(dir_str, "FORWARD") == 0) direction = MOTOR_FORWARD;
            else if (strcasecmp(dir_str, "BACKWARD") == 0) direction = MOTOR_BACKWARD;
            else if (strcasecmp(dir_str, "STOP") == 0) direction = MOTOR_STOP;
            else { ESP_LOGW(TAG, "Direção inválida: %s", dir_str); return; }
            
            // Parse speed (optional)
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
                ESP_LOGW(TAG, "Ângulo fora do range (%d a %d): %d", SWEEP_LIMIT_MIN, SWEEP_LIMIT_MAX, angle);
                return;
            }
        } else {
            ESP_LOGW(TAG, "Comando inválido: '%s'", cmd_str);
            return;
        }
    }

    // ENVIA o comando para o Hub
    if (msg.type != CMD_NONE) {
        if (xQueueSend(control_queue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
            ESP_LOGE(TAG, "Queue de controle cheia. Comando descartado.");
        }
    }
}

void serial_task(void *pvParameters)
{
    // ... (serial_task permanece inalterada, apenas chama o refatorado process_command)
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
 * MAIN
 * ============================================================
 */

void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  GODAR - Carrinho Autônomo ESP32-C3");
    ESP_LOGI(TAG, "  ESTADO INICIAL: TUDO DESLIGADO");
    ESP_LOGI(TAG, "============================================");
    
#ifdef JUMPER_SPEED_MOTOR_OFF
    ESP_LOGI(TAG, "Modo: PWM por Software (Jumpers REMOVIDOS)");
#else
    ESP_LOGI(TAG, "Modo: Velocidade Fixa (Jumpers INSTALADOS)");
#endif

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

    // Inicialização de Hardware
    motor_gpio_init();
    motor_stop_all();
    servo_init();

    // CRIAÇÃO DA QUEUE DO MIDDLEWARE
    control_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(control_message_t));
    if (control_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar Queue de Controle!");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Comandos disponíveis:");
    ESP_LOGI(TAG, "  Servo Ângulo: -90 a 90");
    ESP_LOGI(TAG, "  SWEEP START <1-10>");
    ESP_LOGI(TAG, "  SWEEP STOP");
    ESP_LOGI(TAG, "  ---");
    ESP_LOGI(TAG, "  MOTOR <front|rear> <forward|backward|stop> [1-10]");
    ESP_LOGI(TAG, "  MOTOR <front|rear> CALIB");
    ESP_LOGI(TAG, "  SYNC <forward|backward|stop> [1-10]");
    ESP_LOGI(TAG, "  ---");
    ESP_LOGI(TAG, "Exemplos e comandos antigos continuam válidos. A lógica de concorrência foi eliminada.");
    ESP_LOGI(TAG, "");

    // Tasks
    // Prioridade do HUB (6) > Prioridade da Serial (5) e Sweep (5)
    xTaskCreate(serial_task, "serial_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_hub_task, "control_hub_task", 4096, NULL, 6, NULL); // Task de controle centralizada

    // car_control_task removida, sua lógica está no control_hub_task

    // Loop principal
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Status: Servo Alvo=%d°, Atual=%d°", angle_target, current_angle);
    }
}

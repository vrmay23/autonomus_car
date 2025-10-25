/*
 * =====================================================================================
 *
 *       Filename:  mecatronic.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/19/2025 11:58:23 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
/**
 * @file mecatronic.c
 * @brief Implementação do controle mecatrônico
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#include "mecatronic.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

/**
 * CONFIGURAÇÃO DE HARDWARE
 */
// #define JUMPER_SPEED_MOTOR_ON   // Jumpers ENA/ENB instalados
#define JUMPER_SPEED_MOTOR_OFF     // Jumpers ENA/ENB removidos

/**
 * PINOS ESP32-C3 DevKitM-1
 */
#define GPIO_SERVO_DIRECTION         9
#define GPIO_MOTOR_FRONT_PWM         10
#define GPIO_MOTOR_FRONT_IN1         7
#define GPIO_MOTOR_FRONT_IN2         8
#define GPIO_MOTOR_REAR_PWM          4
#define GPIO_MOTOR_REAR_IN3          18
#define GPIO_MOTOR_REAR_IN4          19

/**
 * PWM Config (Servo)
 */
#define SERVO_LEDC_TIMER             LEDC_TIMER_0
#define SERVO_LEDC_MODE              LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL           LEDC_CHANNEL_0
#define SERVO_LEDC_DUTY_RES          LEDC_TIMER_14_BIT
#define SERVO_LEDC_FREQUENCY         50
#define DUTY_PULSE_MIN               410
#define DUTY_PULSE_MAX               1228

/**
 * PWM Config (Motores)
 */
#define MOTOR_LEDC_TIMER             LEDC_TIMER_1
#define MOTOR_LEDC_MODE              LEDC_LOW_SPEED_MODE
#define MOTOR_FRONT_LEDC_CHANNEL     LEDC_CHANNEL_1
#define MOTOR_REAR_LEDC_CHANNEL      LEDC_CHANNEL_2
#define MOTOR_LEDC_DUTY_RES          LEDC_TIMER_8_BIT
#define MOTOR_LEDC_FREQUENCY         1000

/**
 * Motor PWM Calibration
 */
#define MOTOR_PWM_MIN_FRONT          60
#define MOTOR_PWM_MAX_FRONT          255
#define MOTOR_PWM_MIN_REAR           100
#define MOTOR_PWM_MAX_REAR           230 //original is 255. mas tem que ir ajustando para calibrar

/**
 * SWEEP Config
 */
#define SWEEP_STEP_DEGREES           5
#define SWEEP_STEP_DELAY_MS_MIN      5
#define SWEEP_STEP_DELAY_MS_MAX      50

/**
 * VARIÁVEIS INTERNAS
 */
static const char *TAG = "[MECATRONIC]";
static int current_angle = ANGLE_CENTER;
static int angle_target = ANGLE_CENTER;
static TaskHandle_t sweep_task_handle = NULL;
static int sweep_delay_ms = 10;

/**
 * ============================================================
 * FUNÇÕES PRIVADAS - SERVO
 * ============================================================
 */

static uint32_t angle_to_duty(int angle)
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

/**
 * ============================================================
 * FUNÇÕES PRIVADAS - SWEEP
 * ============================================================
 */

static void set_sweep_speed(int speed)
{
    if (speed < 1) speed = 1;
    if (speed > 10) speed = 10;
    
    int delay_range = SWEEP_STEP_DELAY_MS_MAX - SWEEP_STEP_DELAY_MS_MIN;
    float normalized_speed = (float)(speed - 1) / 9.0;
    float curve_factor = pow(1.0 - normalized_speed, 2.0);
    sweep_delay_ms = SWEEP_STEP_DELAY_MS_MIN + (int)(delay_range * curve_factor);
    
    ESP_LOGI(TAG, "Velocidade SWEEP: %d (Delay: %dms)", speed, sweep_delay_ms);
}

static void sweep_test_task(void *pvParameters)
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
        
        mecatronic_servo_set_angle(next_angle);
        vTaskDelay(pdMS_TO_TICKS(sweep_delay_ms));
    }
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - INICIALIZAÇÃO
 * ============================================================
 */

void mecatronic_servo_init(void)
{
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
    mecatronic_servo_set_angle(ANGLE_CENTER);
}

void mecatronic_motor_init(void)
{
    // Configura pinos como OUTPUT
    gpio_reset_pin(GPIO_MOTOR_FRONT_IN1);
    gpio_reset_pin(GPIO_MOTOR_FRONT_IN2);
    gpio_reset_pin(GPIO_MOTOR_REAR_IN3);
    gpio_reset_pin(GPIO_MOTOR_REAR_IN4);
    gpio_reset_pin(GPIO_MOTOR_FRONT_PWM);
    gpio_reset_pin(GPIO_MOTOR_REAR_PWM);

    gpio_set_direction(GPIO_MOTOR_FRONT_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_FRONT_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_REAR_IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_REAR_IN4, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_FRONT_PWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_REAR_PWM, GPIO_MODE_OUTPUT);

    // Estado inicial: DESLIGADO
    gpio_set_level(GPIO_MOTOR_FRONT_IN1, 0);
    gpio_set_level(GPIO_MOTOR_FRONT_IN2, 0);
    gpio_set_level(GPIO_MOTOR_REAR_IN3, 0);
    gpio_set_level(GPIO_MOTOR_REAR_IN4, 0);
    gpio_set_level(GPIO_MOTOR_FRONT_PWM, 0);
    gpio_set_level(GPIO_MOTOR_REAR_PWM, 0);

    // Pull-down para segurança
    gpio_set_pull_mode(GPIO_MOTOR_FRONT_IN1, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_MOTOR_FRONT_IN2, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_MOTOR_REAR_IN3, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(GPIO_MOTOR_REAR_IN4, GPIO_PULLDOWN_ONLY);

#ifdef JUMPER_SPEED_MOTOR_OFF
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
    gpio_set_level(GPIO_MOTOR_FRONT_PWM, 0);
    gpio_set_level(GPIO_MOTOR_REAR_PWM, 0);
    ESP_LOGI(TAG, "Motores inicializados DESLIGADOS (Jumpers INSTALADOS)");
#endif
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - CONTROLE DE SERVO
 * ============================================================
 */

void mecatronic_servo_set_angle(int angle)
{
    if (angle < ANGLE_MIN || angle > ANGLE_MAX) {
        ESP_LOGW(TAG, "Ângulo fora do range: %d", angle);
        return;
    }
    
    uint32_t duty = angle_to_duty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL));
    
    current_angle = angle;
}

int mecatronic_servo_get_angle(void)
{
    return current_angle;
}

void mecatronic_servo_set_target(int angle)
{
    if (angle < SWEEP_LIMIT_MIN) angle = SWEEP_LIMIT_MIN;
    if (angle > SWEEP_LIMIT_MAX) angle = SWEEP_LIMIT_MAX;
    angle_target = angle;
}

int mecatronic_servo_get_target(void)
{
    return angle_target;
}

bool mecatronic_servo_update_ramp(void)
{
    if (current_angle == angle_target) {
        return true; // Já atingiu o alvo
    }
    
    int direction = (angle_target > current_angle) ? SWEEP_STEP_DEGREES : -SWEEP_STEP_DEGREES;
    int next_angle = current_angle + direction;
    
    // Limita para evitar overshoot
    if ((direction > 0 && next_angle > angle_target) ||
        (direction < 0 && next_angle < angle_target))
    {
        next_angle = angle_target;
    }
    
    mecatronic_servo_set_angle(next_angle);
    return false; // Ainda está movendo
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - CONTROLE DE MOTORES
 * ============================================================
 */

void mecatronic_motor_control(motor_axis_t axis, motor_direction_t direction, int speed)
{
    // Validação de velocidade
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
        } else {
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
        ESP_LOGI(TAG, "Motor %s: %s - Velocidade FIXA", axis_name, dir_name);
#endif

    } else { // MOTOR_REAR
        // Lógica invertida para compensar fiação
        if (direction == MOTOR_FORWARD) {
            gpio_set_level(GPIO_MOTOR_REAR_IN3, 0);
            gpio_set_level(GPIO_MOTOR_REAR_IN4, 1);
        } else if (direction == MOTOR_BACKWARD) {
            gpio_set_level(GPIO_MOTOR_REAR_IN3, 1);
            gpio_set_level(GPIO_MOTOR_REAR_IN4, 0);
        } else {
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
        ESP_LOGI(TAG, "Motor %s: %s - Velocidade FIXA", axis_name, dir_name);
#endif
    }
}

void mecatronic_motor_stop_all(void)
{
    mecatronic_motor_control(MOTOR_FRONT, MOTOR_STOP, 0);
    mecatronic_motor_control(MOTOR_REAR, MOTOR_STOP, 0);
    ESP_LOGI(TAG, "TODOS os motores PARADOS");
}

void mecatronic_motor_sync(motor_direction_t direction, int speed)
{
    if (speed < MOTOR_SPEED_MIN) speed = MOTOR_SPEED_MIN;
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
    
    const char *dir_name = (direction == MOTOR_FORWARD) ? "FORWARD" : 
                          (direction == MOTOR_BACKWARD) ? "BACKWARD" : "STOP";
    
    ESP_LOGI(TAG, "=== SYNC MOTORS ===");
    ESP_LOGI(TAG, "Direção: %s, Speed: %d", dir_name, speed);
    
    mecatronic_motor_control(MOTOR_FRONT, direction, speed);
    mecatronic_motor_control(MOTOR_REAR, direction, speed);
    
    ESP_LOGI(TAG, "=== SYNC COMPLETO ===");
}

void mecatronic_motor_calibration(motor_axis_t axis)
{
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
 * FUNÇÕES PÚBLICAS - SWEEP TEST
 * ============================================================
 */

void mecatronic_sweep_start(int speed)
{
    set_sweep_speed(speed);
    
    if (sweep_task_handle == NULL) {
        xTaskCreate(sweep_test_task, "sweep_test_task", 4096, NULL, 5, &sweep_task_handle);
        ESP_LOGI(TAG, "Sweep Test INICIADO");
    } else {
        ESP_LOGW(TAG, "Sweep Test já rodando. Reconfigurando velocidade.");
    }
}

void mecatronic_sweep_stop(void)
{
    if (sweep_task_handle != NULL) {
        vTaskDelete(sweep_task_handle);
        sweep_task_handle = NULL;
        mecatronic_servo_set_target(ANGLE_CENTER);
        ESP_LOGI(TAG, "Sweep Test PARADO - Retornando ao centro");
    }
}

bool mecatronic_sweep_is_active(void)
{
    return (sweep_task_handle != NULL);
}

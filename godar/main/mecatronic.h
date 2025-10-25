/*
 * =====================================================================================
 *
 *       Filename:  mecatronic.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/19/2025 11:58:03 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

/**
 * @file mecatronic.h
 * @brief Controle Mecatrônico - Motores, Servo e Sensores
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#ifndef MECATRONIC_H
#define MECATRONIC_H

#include <stdbool.h>

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

/**
 * Ângulos do Servo
 */
#define ANGLE_MIN                    -90
#define ANGLE_MAX                    90
#define ANGLE_CENTER                 0

/**
 * SWEEP Limits
 */
#define SWEEP_LIMIT_MIN              -90
#define SWEEP_LIMIT_MAX              90

/**
 * Motor Speed Range
 */
#define MOTOR_SPEED_MIN              1
#define MOTOR_SPEED_MAX              10

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - INICIALIZAÇÃO
 * ============================================================
 */

/**
 * @brief Inicializa o servo motor
 */
void mecatronic_servo_init(void);

/**
 * @brief Inicializa os motores DC
 */
void mecatronic_motor_init(void);

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - CONTROLE DE SERVO
 * ============================================================
 */

/**
 * @brief Define o ângulo do servo
 * @param angle Ângulo desejado (-90 a 90)
 */
void mecatronic_servo_set_angle(int angle);

/**
 * @brief Obtém o ângulo atual do servo
 * @return Ângulo atual
 */
int mecatronic_servo_get_angle(void);

/**
 * @brief Define o ângulo alvo (para rampa suave)
 * @param angle Ângulo desejado (-90 a 90)
 */
void mecatronic_servo_set_target(int angle);

/**
 * @brief Obtém o ângulo alvo
 * @return Ângulo alvo
 */
int mecatronic_servo_get_target(void);

/**
 * @brief Atualiza servo com rampa suave (chamado pelo control_hub)
 * @return true se atingiu o alvo, false se ainda está movendo
 */
bool mecatronic_servo_update_ramp(void);

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - CONTROLE DE MOTORES
 * ============================================================
 */

/**
 * @brief Controla um motor individual
 * @param axis Eixo (MOTOR_FRONT ou MOTOR_REAR)
 * @param direction Direção (FORWARD, BACKWARD, STOP)
 * @param speed Velocidade (1-10)
 */
void mecatronic_motor_control(motor_axis_t axis, motor_direction_t direction, int speed);

/**
 * @brief Para todos os motores
 */
void mecatronic_motor_stop_all(void);

/**
 * @brief Controla ambos os motores sincronizados
 * @param direction Direção (FORWARD, BACKWARD, STOP)
 * @param speed Velocidade (1-10)
 */
void mecatronic_motor_sync(motor_direction_t direction, int speed);

/**
 * @brief Exibe tabela de calibração PWM de um motor
 * @param axis Eixo (MOTOR_FRONT ou MOTOR_REAR)
 */
void mecatronic_motor_calibration(motor_axis_t axis);

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - SWEEP TEST
 * ============================================================
 */

/**
 * @brief Inicia teste de sweep automático
 * @param speed Velocidade do sweep (1-10)
 */
void mecatronic_sweep_start(int speed);

/**
 * @brief Para o teste de sweep
 */
void mecatronic_sweep_stop(void);

/**
 * @brief Verifica se o sweep está ativo
 * @return true se sweep está rodando
 */
bool mecatronic_sweep_is_active(void);

#endif // MECATRONIC_H

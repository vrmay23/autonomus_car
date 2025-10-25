/**
 * @file sensors.h
 * @brief Sensores Ultrassônicos e Buzzer - ESP32-CAM
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include "esp_http_server.h"

/**
 * Pinos dos sensores
 */
#define GPIO_ULTRASONIC_FRONT_TRIGGER    12
#define GPIO_ULTRASONIC_FRONT_ECHO       13
#define GPIO_ULTRASONIC_REAR_TRIGGER     14
#define GPIO_ULTRASONIC_REAR_ECHO        15
#define GPIO_BUZZER                      2

#define ULTRASONIC_MAX_DISTANCE_CM       400
#define ULTRASONIC_TIMEOUT_US            30000

/**
 * @brief Inicializa sensores e buzzer
 */
void sensors_init(void);

/**
 * @brief Lê distância do sensor ultrassônico
 */
int sensors_read_ultrasonic(int trig_pin, int echo_pin);

/**
 * @brief Lê sensor frontal
 */
int sensors_get_front_distance(void);

/**
 * @brief Lê sensor traseiro
 */
int sensors_get_rear_distance(void);

/**
 * @brief Liga/desliga buzzer
 */
void sensors_set_buzzer(bool state);

/**
 * @brief Obtém estado do buzzer
 */
bool sensors_get_buzzer_state(void);

/**
 * @brief Inicia HTTP server (porta 80) com API de sensores
 */
httpd_handle_t sensors_start_http_server(void);

#endif // SENSORS_H
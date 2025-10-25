/**
 * @file sensors.c
 * @brief Implementação dos sensores e HTTP server
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#include "sensors.h"
#include "camera.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char *TAG = "[SENSORS]";

/**
 * Estado interno
 */
static bool buzzer_state = false;

/**
 * Tipo de função genérica para setters que retornam bool/void. 
 * Usado para resolver o warning de cast (Wcast-function-type).
 */
typedef void (*setter_func_t)(bool);

/**
 * ============================================================
 * FUNÇÕES PRIVADAS - ULTRASSÔNICO
 * ============================================================
 */

int sensors_read_ultrasonic(int trig_pin, int echo_pin)
{
    // Trigger pulse
    gpio_set_level(trig_pin, 0);
    ets_delay_us(2);
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);
    
    // Aguarda echo subir (timeout de 30ms)
    int timeout = ULTRASONIC_TIMEOUT_US;
    while (gpio_get_level(echo_pin) == 0 && timeout--) {
        ets_delay_us(1);
    }
    
    if (timeout <= 0) {
        return -1; // Timeout
    }
    
    // Mede a largura do pulso (tempo de voo)
    int start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1 && timeout--) {
        ets_delay_us(1);
    }
    int end_time = esp_timer_get_time();
    
    if (timeout <= 0) {
        return -1; // Timeout
    }
    
    // Distância em cm = (tempo_voo * velocidade_som) / 2
    int duration_us = end_time - start_time;
    int distance_cm = duration_us / 58; 

    if (distance_cm > ULTRASONIC_MAX_DISTANCE_CM) {
        return -1; // Fora do alcance
    }

    return distance_cm;
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - SENSORES
 * ============================================================
 */

void sensors_init(void)
{
    // Configura pinos Ultrassônicos Frontal - CORREÇÃO DE GPIO
    gpio_reset_pin(GPIO_ULTRASONIC_FRONT_TRIGGER);
    gpio_set_direction(GPIO_ULTRASONIC_FRONT_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_ULTRASONIC_FRONT_ECHO);
    gpio_set_direction(GPIO_ULTRASONIC_FRONT_ECHO, GPIO_MODE_INPUT);

    // Configura pinos Ultrassônicos Traseiro - CORREÇÃO DE GPIO
    gpio_reset_pin(GPIO_ULTRASONIC_REAR_TRIGGER);
    gpio_set_direction(GPIO_ULTRASONIC_REAR_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_ULTRASONIC_REAR_ECHO);
    gpio_set_direction(GPIO_ULTRASONIC_REAR_ECHO, GPIO_MODE_INPUT);

    // Configura pino Buzzer - CORREÇÃO DE GPIO
    gpio_reset_pin(GPIO_BUZZER);
    gpio_set_direction(GPIO_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_BUZZER, 0); // Desliga buzzer por padrão
    
    ESP_LOGI(TAG, "Sensores inicializados");
    ESP_LOGI(TAG, "  Front: TRIG=%d, ECHO=%d", GPIO_ULTRASONIC_FRONT_TRIGGER, GPIO_ULTRASONIC_FRONT_ECHO);
    ESP_LOGI(TAG, "  Rear:  TRIG=%d, ECHO=%d", GPIO_ULTRASONIC_REAR_TRIGGER, GPIO_ULTRASONIC_REAR_ECHO);
    ESP_LOGI(TAG, "  Buzzer: GPIO=%d", GPIO_BUZZER);
}

int sensors_get_front_distance(void)
{
    return sensors_read_ultrasonic(GPIO_ULTRASONIC_FRONT_TRIGGER, GPIO_ULTRASONIC_FRONT_ECHO);
}

int sensors_get_rear_distance(void)
{
    return sensors_read_ultrasonic(GPIO_ULTRASONIC_REAR_TRIGGER, GPIO_ULTRASONIC_REAR_ECHO);
}

void sensors_set_buzzer(bool state)
{
    gpio_set_level(GPIO_BUZZER, state ? 1 : 0);
    buzzer_state = state;
}

bool sensors_get_buzzer_state(void)
{
    return buzzer_state;
}

/**
 * ============================================================
 * HTTP HANDLERS
 * ============================================================
 */

static esp_err_t sensors_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    int front = sensors_get_front_distance();
    int rear = sensors_get_rear_distance();
    
    cJSON_AddNumberToObject(root, "front", front);
    cJSON_AddNumberToObject(root, "rear", rear);
    cJSON_AddBoolToObject(root, "buzzer", sensors_get_buzzer_state());
    cJSON_AddBoolToObject(root, "flash", camera_is_flash_on());
    cJSON_AddBoolToObject(root, "camera", camera_is_powered());
    
    const char *json_response = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    
    cJSON_Delete(root);
    free((void*)json_response);
    return ESP_OK;
}

// Handler genérico para comandos POST (liga/desliga)
static esp_err_t handle_post_command(httpd_req_t *req, const char *key, setter_func_t set_func, bool (*get_func)(void))
{
    if (req->method != HTTP_POST) {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Apenas POST permitido");
        return ESP_FAIL;
    }
    
    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) { 
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *root = cJSON_Parse(content);
    cJSON *state_json = cJSON_GetObjectItem(root, key);
    bool state = false;
    
    if (cJSON_IsBool(state_json)) {
        state = cJSON_IsTrue(state_json);
        set_func(state);
        
        cJSON_Delete(root);
        
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, key, get_func());
        const char *json_response = cJSON_PrintUnformatted(response);
        
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_response, strlen(json_response));
        
        cJSON_Delete(response);
        free((void*)json_response);
        return ESP_OK;
        
    } else {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Corpo do request inválido. Use {\"key\": true/false}");
        return ESP_FAIL;
    }
}

// Wrappers para o handle genérico
static esp_err_t buzzer_handler(httpd_req_t *req)
{
    // sensors_set_buzzer retorna void, o cast é implícito e OK
    return handle_post_command(req, "state", (setter_func_t)sensors_set_buzzer, sensors_get_buzzer_state);
}

static esp_err_t flash_handler(httpd_req_t *req)
{
    // Correção do warning de cast: usando setter_func_t
    return handle_post_command(req, "state", (setter_func_t)camera_set_flash, camera_is_flash_on);
}

static esp_err_t camera_power_handler(httpd_req_t *req)
{
    // Correção do warning de cast: usando setter_func_t
    return handle_post_command(req, "state", (setter_func_t)camera_set_power, camera_is_powered);
}


httpd_handle_t sensors_start_http_server(void)
{
    // CRITICAL FIX: Aumenta limites para evitar httpd_accept_conn: error in accept (23)
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 8;        // Aumentado
    config.server_port = 80;            
    config.max_req_hdr_len = 1024;      // Aumentado
    config.max_resp_headers = 16;       // Aumentado

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        // GET /sensors
        httpd_uri_t sensors_uri = {
            .uri       = "/sensors",
            .method    = HTTP_GET,
            .handler   = sensors_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &sensors_uri);
        
        // POST /buzzer
        httpd_uri_t buzzer_uri = {
            .uri       = "/buzzer",
            .method    = HTTP_POST,
            .handler   = buzzer_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &buzzer_uri);
        
        // POST /flash
        httpd_uri_t flash_uri = {
            .uri       = "/flash",
            .method    = HTTP_POST,
            .handler   = flash_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &flash_uri);
        
        // POST /camera
        httpd_uri_t camera_uri = {
            .uri       = "/camera",
            .method    = HTTP_POST,
            .handler   = camera_power_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &camera_uri);
        
        ESP_LOGI(TAG, "HTTP Server iniciado na porta 80");
        ESP_LOGI(TAG, "  GET  /sensors  → Lê sensores");
        ESP_LOGI(TAG, "  POST /buzzer   → Controla buzzer");
        ESP_LOGI(TAG, "  POST /flash    → Controla flash");
        ESP_LOGI(TAG, "  POST /camera   → Liga/desliga câmera");
        return server;
    }
    
    ESP_LOGE(TAG, "Falha ao iniciar HTTP Server");
    return NULL;
}
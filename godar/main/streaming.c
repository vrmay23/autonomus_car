/**
 * @file streaming.c
 * @brief Implementação do cliente HTTP para ESP32-CAM
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 * 
 * NOTA: Compatível com ESP-IDF v6
 */

#include "streaming.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"

// cJSON está incluído por padrão no ESP-IDF v6
#include "cJSON.h"

static const char *TAG = "[STREAMING]";

/**
 * VARIÁVEIS INTERNAS
 */
static streaming_sensors_t cached_sensors = {
    .front_cm = -1,
    .rear_cm = -1,
    .valid = false
};

static bool esp32cam_connected = false;
static uint32_t last_update_time = 0;
static TaskHandle_t poll_task_handle = NULL;

/**
 * Buffer para resposta HTTP
 */
#define HTTP_RESPONSE_BUFFER_SIZE  512
static char http_response_buffer[HTTP_RESPONSE_BUFFER_SIZE];
static int http_response_len = 0;

/**
 * ============================================================
 * FUNÇÕES PRIVADAS - HTTP CLIENT
 * ============================================================
 */

/**
 * @brief Callback para receber dados HTTP
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (http_response_len + evt->data_len < HTTP_RESPONSE_BUFFER_SIZE) {
                memcpy(http_response_buffer + http_response_len, evt->data, evt->data_len);
                http_response_len += evt->data_len;
                http_response_buffer[http_response_len] = '\0';
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

/**
 * @brief Faz requisição HTTP GET
 */
static bool http_get_request(const char *url, char *response, int max_len)
{
    http_response_len = 0;
    memset(http_response_buffer, 0, HTTP_RESPONSE_BUFFER_SIZE);
    
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .timeout_ms = 2000,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Falha ao criar cliente HTTP");
        return false;
    }
    
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status == 200) {
            if (response && max_len > 0) {
                strncpy(response, http_response_buffer, max_len - 1);
                response[max_len - 1] = '\0';
            }
            esp_http_client_cleanup(client);
            return true;
        } else {
            ESP_LOGW(TAG, "HTTP GET falhou com status %d", status);
        }
    } else {
        ESP_LOGW(TAG, "HTTP GET erro: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    return false;
}

/**
 * @brief Faz requisição HTTP POST
 */
static bool http_post_request(const char *url, const char *post_data)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 2000,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Falha ao criar cliente HTTP");
        return false;
    }
    
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    
    esp_err_t err = esp_http_client_perform(client);
    
    bool success = false;
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        success = (status == 200);
        if (!success) {
            ESP_LOGW(TAG, "HTTP POST falhou com status %d", status);
        }
    } else {
        ESP_LOGW(TAG, "HTTP POST erro: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    return success;
}

/**
 * ============================================================
 * FUNÇÕES PRIVADAS - PARSING
 * ============================================================
 */

/**
 * @brief Faz parsing do JSON de sensores
 * Formato esperado: {"front": 123, "rear": 456}
 */
static bool parse_sensors_json(const char *json_str, streaming_sensors_t *sensors)
{
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        ESP_LOGE(TAG, "Falha ao fazer parse do JSON");
        return false;
    }
    
    cJSON *front = cJSON_GetObjectItem(root, "front");
    cJSON *rear = cJSON_GetObjectItem(root, "rear");
    
    if (cJSON_IsNumber(front) && cJSON_IsNumber(rear)) {
        sensors->front_cm = front->valueint;
        sensors->rear_cm = rear->valueint;
        sensors->valid = true;
        
        cJSON_Delete(root);
        return true;
    }
    
    ESP_LOGE(TAG, "JSON não contém campos 'front' e 'rear'");
    cJSON_Delete(root);
    return false;
}

/**
 * ============================================================
 * FUNÇÕES PRIVADAS - POLLING TASK
 * ============================================================
 */

static void streaming_poll_task(void *pvParameters)
{
    char url[128];
    char response[256];
    
    ESP_LOGI(TAG, "Task de polling iniciada");
    
    // Aguarda um tempo inicial para o ESP32-CAM conectar
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    while (1) {
        snprintf(url, sizeof(url), "http://%s:%d/sensors", ESP32CAM_IP, ESP32CAM_PORT);
        
        if (http_get_request(url, response, sizeof(response))) {
            streaming_sensors_t new_data;
            
            if (parse_sensors_json(response, &new_data)) {
                // Atualiza cache
                cached_sensors = new_data;
                last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                esp32cam_connected = true;
                
                ESP_LOGD(TAG, "Sensores atualizados: Front=%dcm, Rear=%dcm", 
                         new_data.front_cm, new_data.rear_cm);
            } else {
                ESP_LOGW(TAG, "Falha ao parsear resposta dos sensores");
                esp32cam_connected = false;
            }
        } else {
            ESP_LOGD(TAG, "ESP32-CAM não respondeu (pode estar desconectado)");
            esp32cam_connected = false;
            cached_sensors.valid = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(STREAMING_POLL_INTERVAL_MS));
    }
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - INICIALIZAÇÃO
 * ============================================================
 */

void streaming_init(void)
{
    ESP_LOGI(TAG, "Módulo de streaming inicializado");
    ESP_LOGI(TAG, "ESP32-CAM esperado em: %s:%d", ESP32CAM_IP, ESP32CAM_PORT);
}

void streaming_start_poll_task(void)
{
    if (poll_task_handle == NULL) {
        xTaskCreate(streaming_poll_task, "streaming_poll_task", 4096, NULL, 4, &poll_task_handle);
        ESP_LOGI(TAG, "Task de polling iniciada (intervalo: %dms)", STREAMING_POLL_INTERVAL_MS);
    } else {
        ESP_LOGW(TAG, "Task de polling já está rodando");
    }
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - LEITURA DE SENSORES
 * ============================================================
 */

bool streaming_get_sensors(streaming_sensors_t *sensors)
{
    if (sensors == NULL) {
        return false;
    }
    
    *sensors = cached_sensors;
    return cached_sensors.valid;
}

bool streaming_fetch_sensors(streaming_sensors_t *sensors)
{
    if (sensors == NULL) {
        return false;
    }
    
    char url[128];
    char response[256];
    
    snprintf(url, sizeof(url), "http://%s:%d/sensors", ESP32CAM_IP, ESP32CAM_PORT);
    
    if (http_get_request(url, response, sizeof(response))) {
        if (parse_sensors_json(response, sensors)) {
            // Atualiza cache também
            cached_sensors = *sensors;
            last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            esp32cam_connected = true;
            return true;
        }
    }
    
    esp32cam_connected = false;
    return false;
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - CONTROLE REMOTO
 * ============================================================
 */

bool streaming_set_buzzer(bool state)
{
    char url[128];
    char post_data[64];
    
    snprintf(url, sizeof(url), "http://%s:%d/buzzer", ESP32CAM_IP, ESP32CAM_PORT);
    snprintf(post_data, sizeof(post_data), "{\"state\":\"%s\"}", state ? "on" : "off");
    
    bool success = http_post_request(url, post_data);
    
    if (success) {
        ESP_LOGI(TAG, "Buzzer %s", state ? "LIGADO" : "DESLIGADO");
    } else {
        ESP_LOGW(TAG, "Falha ao enviar comando do buzzer");
    }
    
    return success;
}

bool streaming_set_flash(bool state)
{
    char url[128];
    char post_data[64];
    
    snprintf(url, sizeof(url), "http://%s:%d/flash", ESP32CAM_IP, ESP32CAM_PORT);
    snprintf(post_data, sizeof(post_data), "{\"state\":\"%s\"}", state ? "on" : "off");
    
    bool success = http_post_request(url, post_data);
    
    if (success) {
        ESP_LOGI(TAG, "Flash %s", state ? "LIGADO" : "DESLIGADO");
    } else {
        ESP_LOGW(TAG, "Falha ao enviar comando do flash");
    }
    
    return success;
}

bool streaming_set_camera(bool state)
{
    char url[128];
    char post_data[64];
    
    snprintf(url, sizeof(url), "http://%s:%d/camera", ESP32CAM_IP, ESP32CAM_PORT);
    snprintf(post_data, sizeof(post_data), "{\"state\":\"%s\"}", state ? "on" : "off");
    
    bool success = http_post_request(url, post_data);
    
    if (success) {
        ESP_LOGI(TAG, "Câmera %s", state ? "LIGADA" : "DESLIGADA");
    } else {
        ESP_LOGW(TAG, "Falha ao enviar comando da câmera");
    }
    
    return success;
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - STATUS
 * ============================================================
 */

bool streaming_is_connected(void)
{
    return esp32cam_connected;
}

uint32_t streaming_get_last_update_time(void)
{
    return last_update_time;
}
/**
 * @file wifi_server.c
 * @brief Implementação do WiFi Access Point e HTTP Server
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#include "esp_mac.h"
#include "wifi_server.h"
#include "streaming.h"
#include <string.h>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "cJSON.h"

static const char *TAG = "[WIFI_SERVER]";

/**
 * Declaração externa da função process_command do módulo principal
 */
extern void process_command(char *cmd_str);

/**
 * Variáveis externas para status da CAM
 */
extern bool cam_buzzer_state;
extern bool cam_flash_state;
extern bool cam_power_state;

/**
 * ============================================================
 * FUNÇÕES AUXILIARES
 * ============================================================
 */

void url_decode(const char *src, char *dest)
{
    char a, b;
    while (*src) {
        if (*src == '%') {
            if ((a = src[1]) && (b = src[2])) {
                if (a >= '0' && a <= '9') a -= '0';
                else if (a >= 'a' && a <= 'f') a = a - 'a' + 10;
                else if (a >= 'A' && a <= 'F') a = a - 'A' + 10;
                
                if (b >= '0' && b <= '9') b -= '0';
                else if (b >= 'a' && b <= 'f') b = b - 'a' + 10;
                else if (b >= 'A' && b <= 'F') b = b - 'A' + 10;
                
                *dest++ = 16 * a + b;
                src += 3;
                continue;
            }
        } else if (*src == '+') {
            *dest++ = ' ';
            src++;
            continue;
        }
        
        *dest++ = *src++;
    }
    *dest = '\0';
}

/**
 * ============================================================
 * HTTP HANDLERS - CONTROLE DO CARRINHO
 * ============================================================
 */

static esp_err_t root_handler(httpd_req_t *req)
{
    const char *response = 
        "<!DOCTYPE html>"
        "<html><head><meta charset='UTF-8'><title>GODAR</title></head>"
        "<body style='font-family:Arial;text-align:center;padding:50px;background:#0a0a0a;color:#fff'>"
        "<h1>🚗 GODAR WiFi Control</h1>"
        "<p>Servidor HTTP ativo!</p>"
        "<p>Use o app para controlar o carrinho.</p>"
        "<hr>"
        "<h3>Endpoints Disponíveis:</h3>"
        "<ul style='text-align:left;max-width:400px;margin:auto'>"
        "<li>GET /cmd?c=COMANDO - Controle motores/servo</li>"
        "<li>GET /cam/sensors - Dados dos sensores</li>"
        "<li>GET /cam/status - Status câmera/buzzer/flash</li>"
        "<li>POST /cam/buzzer - Controla buzzer</li>"
        "<li>POST /cam/flash - Controla flash</li>"
        "<li>POST /cam/power - Liga/desliga câmera</li>"
        "</ul>"
        "<hr><small>GODAR v2.0 - Modular Architecture</small>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char query_buf[256];
    char param_buf[128];
    char decoded_buf[128];
    
    size_t query_len = httpd_req_get_url_query_len(req);
    
    if (query_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parametro 'c' ausente");
        return ESP_FAIL;
    }
    
    if (query_len >= sizeof(query_buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Query string muito longa");
        return ESP_FAIL;
    }
    
    if (httpd_req_get_url_query_str(req, query_buf, sizeof(query_buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erro ao ler query");
        return ESP_FAIL;
    }
    
    if (httpd_query_key_value(query_buf, "c", param_buf, sizeof(param_buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parametro 'c' nao encontrado");
        return ESP_FAIL;
    }
    
    url_decode(param_buf, decoded_buf);
    
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "📱 COMANDO HTTP RECEBIDO");
    ESP_LOGI(TAG, "   Decoded: [%s]", decoded_buf);
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    process_command(decoded_buf);
    
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

/**
 * ============================================================
 * HTTP HANDLERS - ESP32-CAM API
 * ============================================================
 */

/**
 * GET /cam/sensors
 * Retorna JSON com dados dos sensores
 */
static esp_err_t cam_sensors_handler(httpd_req_t *req)
{
    streaming_sensors_t sensors;
    char json_response[256];
    
    if (streaming_get_sensors(&sensors) && sensors.valid) {
        snprintf(json_response, sizeof(json_response),
                 "{\"front\":%d,\"rear\":%d,\"connected\":true}",
                 sensors.front_cm, sensors.rear_cm);
    } else {
        snprintf(json_response, sizeof(json_response),
                 "{\"front\":-1,\"rear\":-1,\"connected\":false}");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

/**
 * GET /cam/status
 * Retorna status de buzzer, flash e câmera
 */
static esp_err_t cam_status_handler(httpd_req_t *req)
{
    char json_response[256];
    bool connected = streaming_is_connected();
    uint32_t last_update = streaming_get_last_update_time();
    
    snprintf(json_response, sizeof(json_response),
             "{\"connected\":%s,\"buzzer\":%s,\"flash\":%s,\"camera\":%s,\"last_update\":%lu}",
             connected ? "true" : "false",
             cam_buzzer_state ? "true" : "false",
             cam_flash_state ? "true" : "false",
             cam_power_state ? "true" : "false",
             last_update);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

/**
 * POST /cam/buzzer
 * Body: "on" ou "off"
 */
static esp_err_t cam_buzzer_handler(httpd_req_t *req)
{
    char content[16];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body vazio");
        return ESP_FAIL;
    }
    
    content[ret] = '\0';
    
    bool state = (strcmp(content, "on") == 0);
    
    if (streaming_set_buzzer(state)) {
        cam_buzzer_state = state;
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ESP32-CAM nao respondeu");
    }
    
    return ESP_OK;
}

/**
 * POST /cam/flash
 * Body: "on" ou "off"
 */
static esp_err_t cam_flash_handler(httpd_req_t *req)
{
    char content[16];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body vazio");
        return ESP_FAIL;
    }
    
    content[ret] = '\0';
    
    bool state = (strcmp(content, "on") == 0);
    
    if (streaming_set_flash(state)) {
        cam_flash_state = state;
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ESP32-CAM nao respondeu");
    }
    
    return ESP_OK;
}

/**
 * POST /cam/power
 * Body: "on" ou "off"
 */
static esp_err_t cam_power_handler(httpd_req_t *req)
{
    char content[16];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body vazio");
        return ESP_FAIL;
    }
    
    content[ret] = '\0';
    
    bool state = (strcmp(content, "on") == 0);
    
    if (streaming_set_camera(state)) {
        cam_power_state = state;
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ESP32-CAM nao respondeu");
    }
    
    return ESP_OK;
}

/**
 * ============================================================
 * WIFI EVENT HANDLER
 * ============================================================
 */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ESP_LOGI(TAG, "   Cliente WiFi CONECTADO");
        ESP_LOGI(TAG, "   MAC: " MACSTR, MAC2STR(event->mac));
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ESP_LOGI(TAG, "   Cliente WiFi DESCONECTADO");
        ESP_LOGI(TAG, "   MAC: " MACSTR, MAC2STR(event->mac));
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }
}

/**
 * ============================================================
 * INICIALIZAÇÃO WIFI ACCESS POINT
 * ============================================================
 */

void wifi_init_softap(void)
{
    ESP_LOGI(TAG, "Inicializando WiFi Access Point...");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL
    ));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║     WiFi Access Point INICIADO!       ║");
    ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
    ESP_LOGI(TAG, "║  SSID:     %-27s║", WIFI_SSID);
    ESP_LOGI(TAG, "║  Password: %-27s║", WIFI_PASS);
    ESP_LOGI(TAG, "║  Channel:  %-27d║", WIFI_CHANNEL);
    ESP_LOGI(TAG, "║  IP:       192.168.4.1                ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
}

/**
 * ============================================================
 * INICIALIZAÇÃO HTTP SERVER
 * ============================================================
 */

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;
    
    ESP_LOGI(TAG, "Iniciando servidor HTTP na porta %d...", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        
        // Handler para GET /
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        // Handler para GET /cmd
        httpd_uri_t cmd_uri = {
            .uri       = "/cmd",
            .method    = HTTP_GET,
            .handler   = cmd_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cmd_uri);
        
        // Handler para GET /cam/sensors
        httpd_uri_t cam_sensors_uri = {
            .uri       = "/cam/sensors",
            .method    = HTTP_GET,
            .handler   = cam_sensors_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cam_sensors_uri);
        
        // Handler para GET /cam/status
        httpd_uri_t cam_status_uri = {
            .uri       = "/cam/status",
            .method    = HTTP_GET,
            .handler   = cam_status_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cam_status_uri);
        
        // Handler para POST /cam/buzzer
        httpd_uri_t cam_buzzer_uri = {
            .uri       = "/cam/buzzer",
            .method    = HTTP_POST,
            .handler   = cam_buzzer_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cam_buzzer_uri);
        
        // Handler para POST /cam/flash
        httpd_uri_t cam_flash_uri = {
            .uri       = "/cam/flash",
            .method    = HTTP_POST,
            .handler   = cam_flash_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cam_flash_uri);
        
        // Handler para POST /cam/power
        httpd_uri_t cam_power_uri = {
            .uri       = "/cam/power",
            .method    = HTTP_POST,
            .handler   = cam_power_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cam_power_uri);
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
        ESP_LOGI(TAG, "║   HTTP Server INICIADO COM SUCESSO!   ║");
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  Endpoints Registrados:               ║");
        ESP_LOGI(TAG, "║    GET  /             → Home          ║");
        ESP_LOGI(TAG, "║    GET  /cmd?c=       → Comandos      ║");
        ESP_LOGI(TAG, "║    GET  /cam/sensors  → Sensores      ║");
        ESP_LOGI(TAG, "║    GET  /cam/status   → Status CAM    ║");
        ESP_LOGI(TAG, "║    POST /cam/buzzer   → Buzzer        ║");
        ESP_LOGI(TAG, "║    POST /cam/flash    → Flash         ║");
        ESP_LOGI(TAG, "║    POST /cam/power    → Câmera        ║");
        ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        
    } else {
        ESP_LOGE(TAG, "ERRO ao iniciar servidor HTTP!");
        return NULL;
    }
    
    return server;
}

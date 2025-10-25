/**
 * @file camera.c
 * @brief Implementação com AUTO-DETECÇÃO de pinos I2C (IDF v6.0)
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#include "camera.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "[CAMERA]";

/**
 * Pinos padrão da câmera OV2640
 */
#define CAM_PIN_PWDN     32
#define CAM_PIN_RESET    -1
#define CAM_PIN_XCLK      0
#define CAM_PIN_D7       35
#define CAM_PIN_D6       34
#define CAM_PIN_D5       39
#define CAM_PIN_D4       36
#define CAM_PIN_D3       21
#define CAM_PIN_D2       19
#define CAM_PIN_D1       18
#define CAM_PIN_D0        5
#define CAM_PIN_VSYNC    25
#define CAM_PIN_HREF     23
#define CAM_PIN_PCLK     22
#define GPIO_FLASH        4

static bool camera_powered = false;
static bool flash_state = false;
static httpd_handle_t stream_httpd = NULL;

/**
 * Combinações de pinos I2C para testar
 */
typedef struct {
    int sda;
    int scl;
    const char *name;
} i2c_pins_t;

static const i2c_pins_t i2c_combinations[] = {
    {26, 27, "AI-Thinker/WROVER (Padrão)"},
    {21, 22, "TTGO T-Camera"},
    {25, 23, "M5Stack"},
    {14, 15, "Clone Alternativo 1"},
    {12, 13, "Clone Alternativo 2"},
};

/**
 * ============================================================
 * HANDLER DO STREAMING MJPEG
 * ============================================================
 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    const char *boundary = "123456789000000000000987654321";
    char content_type[128];
    snprintf(content_type, sizeof(content_type), 
             "multipart/x-mixed-replace;boundary=%s", boundary);
    
    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Falha ao capturar frame");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (fb->format == PIXFORMAT_JPEG) {
            char part_buf[128];
            size_t part_len = snprintf(part_buf, sizeof(part_buf),
                "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                boundary, fb->len);
            
            if (httpd_resp_send_chunk(req, part_buf, part_len) != ESP_OK) {
                esp_camera_fb_return(fb);
                break;
            }
            
            if (httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
                esp_camera_fb_return(fb);
                break;
            }
            
            if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
                esp_camera_fb_return(fb);
                break;
            }
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    return ESP_OK;
}

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS
 * ============================================================
 */

bool camera_init(void)
{
    // Inicializa LED Flash
    gpio_set_direction(GPIO_FLASH, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_FLASH, 0);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "AUTO-DETECÇÃO DE CÂMERA INICIADA");
    ESP_LOGI(TAG, "========================================");

    // Tenta todas as combinações de pinos I2C
    for (int i = 0; i < sizeof(i2c_combinations) / sizeof(i2c_pins_t); i++) {
        ESP_LOGI(TAG, "Testando: %s (SDA=%d, SCL=%d)", 
                 i2c_combinations[i].name,
                 i2c_combinations[i].sda,
                 i2c_combinations[i].scl);
        
        // Configuração da câmera para esta combinação
        camera_config_t camera_config = {
            .pin_pwdn     = CAM_PIN_PWDN,
            .pin_reset    = CAM_PIN_RESET,
            .pin_xclk     = CAM_PIN_XCLK,
            .pin_sccb_sda = i2c_combinations[i].sda,  // IDF v6.0 usa pin_sccb_sda
            .pin_sccb_scl = i2c_combinations[i].scl,  // IDF v6.0 usa pin_sccb_scl
            .pin_d7       = CAM_PIN_D7,
            .pin_d6       = CAM_PIN_D6,
            .pin_d5       = CAM_PIN_D5,
            .pin_d4       = CAM_PIN_D4,
            .pin_d3       = CAM_PIN_D3,
            .pin_d2       = CAM_PIN_D2,
            .pin_d1       = CAM_PIN_D1,
            .pin_d0       = CAM_PIN_D0,
            .pin_vsync    = CAM_PIN_VSYNC,
            .pin_href     = CAM_PIN_HREF,
            .pin_pclk     = CAM_PIN_PCLK,
            .xclk_freq_hz = 20000000,
            .ledc_timer   = LEDC_TIMER_0,
            .ledc_channel = LEDC_CHANNEL_0,
            .pixel_format = PIXFORMAT_JPEG,
            .frame_size   = CAMERA_FRAME_SIZE,
            .jpeg_quality = CAMERA_JPEG_QUALITY,
            .fb_count     = 1,
              
            //   DRAM
            .fb_location  = CAMERA_FB_IN_DRAM,
            .grab_mode    = CAMERA_GRAB_LATEST,
        

            /* PSRAM 
            .fb_location  = CAMERA_FB_IN_PSRAM, 
            .grab_mode    = CAMERA_GRAB_LATEST,
            */
        };
        
        esp_err_t err = esp_camera_init(&camera_config);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "✓ CÂMERA DETECTADA!");
            ESP_LOGI(TAG, "Modelo: %s", i2c_combinations[i].name);
            ESP_LOGI(TAG, "Pinos I2C: SDA=%d, SCL=%d", 
                     i2c_combinations[i].sda, 
                     i2c_combinations[i].scl);
            ESP_LOGI(TAG, "Resolução: VGA (640x480)");
            ESP_LOGI(TAG, "Qualidade JPEG: %d", CAMERA_JPEG_QUALITY);
            ESP_LOGI(TAG, "========================================");
            
            camera_powered = true;
            return true;
        } else {
            ESP_LOGW(TAG, "  ✗ Falhou (erro 0x%x)", err);
            
            // Deinicializa antes de tentar próxima combinação
            esp_camera_deinit();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    
    ESP_LOGE(TAG, "========================================");
    ESP_LOGE(TAG, "✗ CÂMERA NÃO DETECTADA!");
    ESP_LOGE(TAG, "Possíveis causas:");
    ESP_LOGE(TAG, "  1. Módulo de câmera não conectado");
    ESP_LOGE(TAG, "  2. Flat cable mal encaixado");
    ESP_LOGE(TAG, "  3. Câmera defeituosa");
    ESP_LOGE(TAG, "  4. Pinos I2C diferentes (informe o modelo)");
    ESP_LOGE(TAG, "========================================");
    
    return false;
}

httpd_handle_t camera_start_stream_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    config.ctrl_port = 32769;
    
    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        ESP_LOGI(TAG, "Streaming server iniciado na porta 81");
        return stream_httpd;
    }
    
    ESP_LOGE(TAG, "Falha ao iniciar streaming server");
    return NULL;
}

bool camera_set_power(bool state)
{
    if (state == camera_powered) {
        return true;
    }
    
    if (state) {
        return camera_init();
    } else {
        esp_camera_deinit();
        camera_powered = false;
        camera_set_flash(false);
        return true;
    }
}

bool camera_is_powered(void)
{
    return camera_powered;
}

bool camera_set_flash(bool state)
{
    gpio_set_level(GPIO_FLASH, state ? 1 : 0);
    flash_state = state;
    ESP_LOGI(TAG, "Flash %s", state ? "ON" : "OFF");
    return true;
}

bool camera_is_flash_on(void)
{
    return flash_state;
}
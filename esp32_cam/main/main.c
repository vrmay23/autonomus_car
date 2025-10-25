/**
 * @file main.c
 * @brief Orquestrador Principal - ESP32-CAM GODAR
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 * 
 * CONFIGURAÇÃO:
 * - Em wifi_client.h, descomente SELFTEST_MODE para teste standalone (AP)
 * - Comente SELFTEST_MODE para produção (STA - conecta no carrinho)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "wifi_client.h"
#include "camera.h"
#include "sensors.h"

static const char *TAG = "[ESP32CAM]";

/**
 * Task de monitoramento (opcional)
 */
void sensor_monitor_task(void *pvParameters)
{
    while (1) {
        int front = sensors_get_front_distance();
        int rear = sensors_get_rear_distance();
        ESP_LOGI(TAG, "Sensores: Front=%dcm, Rear=%dcm", front, rear);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  ESP32-CAM GODAR - Sensores + Câmera");
    
#ifdef SELFTEST_MODE
    ESP_LOGW(TAG, "  MODO: SELFTEST (Standalone / AP)");
#else
    ESP_LOGI(TAG, "  MODO: PRODUÇÃO (Conecta no Carrinho / STA)");
#endif
    
    ESP_LOGI(TAG, "============================================");
    
    // 1. Inicializa sensores
    sensors_init();
    
    // 2. Inicializa câmera
    if (!camera_init()) {
        ESP_LOGE(TAG, "✗ Falha ao inicializar câmera");
        return;
    }
    ESP_LOGI(TAG, "✓ Câmera inicializada");
    
    // 3. Conecta WiFi
    if (!wifi_init()) {
        ESP_LOGE(TAG, "✗ Falha ao inicializar WiFi");
#ifndef SELFTEST_MODE
        ESP_LOGE(TAG, "Certifique-se que o AP GODAR_WIFI está ligado");
#endif
        return;
    }
    
    char ip[16];
    if (wifi_get_ip(ip, sizeof(ip))) {
        ESP_LOGI(TAG, "✓ WiFi conectado/AP iniciado: %s", ip);
        
#ifdef SELFTEST_MODE
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "╔═══════════════════════════════════════╗");
        ESP_LOGW(TAG, "║ MODO: SELFTEST (AP)                   ║");
        ESP_LOGW(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGW(TAG, "║ Conecte-se: godar-cam/godar-cam123    ║");
        ESP_LOGW(TAG, "║ Dashboard: http://192.168.4.2/        ║");
        ESP_LOGW(TAG, "║ Stream:    http://192.168.4.2:81/stream║");
        ESP_LOGW(TAG, "╚═══════════════════════════════════════╝");
        ESP_LOGW(TAG, "");
#else
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
        ESP_LOGI(TAG, "║ MODO: PRODUÇÃO (STA)                  ║");
        ESP_LOGI(TAG, "╠═══════════════════════════════════════╣");
        ESP_LOGI(TAG, "║ IP Câmera (STA): %-20s ║", ip);
        ESP_LOGI(TAG, "║ Stream URL:      http://%s:81/stream  ║", ip);
        ESP_LOGI(TAG, "║ Controle Total: http://192.168.4.1/   ║");
        ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
#endif
    }
    
    // 4. Inicia HTTP Server (porta 80 - API)
    if (!sensors_start_http_server()) {
        ESP_LOGE(TAG, "✗ Falha ao iniciar HTTP Server");
        return;
    }
    ESP_LOGI(TAG, "✓ HTTP Server iniciado (porta 80)");
    
    // 5. Inicia Streaming Server (porta 81 - vídeo)
    if (!camera_start_stream_server()) {
        ESP_LOGE(TAG, "✗ Falha ao iniciar Streaming Server");
        return;
    }
    ESP_LOGI(TAG, "✓ Streaming Server iniciado (porta 81)");
    
    // 6. Inicia task de monitoramento (opcional)
    xTaskCreate(sensor_monitor_task, "sensor_monitor", 2048, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      SISTEMA PRONTO E OPERACIONAL     ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    // Loop principal
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        if (!wifi_is_connected()) {
            ESP_LOGW(TAG, "WiFi desconectado! Sistema pode não responder.");
        }
    }
}
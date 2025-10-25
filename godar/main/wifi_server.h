/**
 * @file wifi_server.h
 * @brief WiFi Access Point e HTTP Server para controle remoto do GODAR
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include "esp_http_server.h"
#include "esp_wifi.h"

/**
 * Configurações do WiFi Access Point
 */
#define WIFI_SSID           "GODAR_WIFI"
#define WIFI_PASS           "godar123"
#define WIFI_CHANNEL        1
#define MAX_STA_CONN        4

/**
 * @brief Inicializa o WiFi em modo Access Point (AP)
 * 
 * Cria uma rede WiFi com SSID "GODAR_WIFI" e senha "godar123"
 * IP fixo: 192.168.4.1
 */
void wifi_init_softap(void);

/**
 * @brief Inicia o servidor HTTP na porta 80
 * 
 * Endpoints disponíveis:
 *   GET /             - Página de teste (opcional)
 *   GET /cmd?c=       - Recebe comandos do app (motores/servo)
 *   GET /cam/sensors  - Retorna JSON com dados dos sensores CAM
 *   GET /cam/status   - Status da câmera/buzzer/flash
 *   POST /cam/buzzer  - Liga/desliga buzzer (body: on/off)
 *   POST /cam/flash   - Liga/desliga flash (body: on/off)
 *   POST /cam/power   - Liga/desliga câmera (body: on/off)
 * 
 * @return Handle do servidor HTTP ou NULL em caso de erro
 */
httpd_handle_t start_webserver(void);

/**
 * @brief Decodifica string URL-encoded
 * 
 * Converte caracteres especiais:
 *   %20 -> espaço
 *   %2B -> +
 *   +   -> espaço
 * 
 * @param src String codificada (entrada)
 * @param dest String decodificada (saída)
 */
void url_decode(const char *src, char *dest);

#endif // WIFI_SERVER_H

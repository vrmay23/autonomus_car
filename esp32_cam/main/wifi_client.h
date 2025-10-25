/**
 * @file wifi_client.h
 * @brief WiFi Client/AP para ESP32-CAM
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#ifndef WIFI_CLIENT_H
#define WIFI_CLIENT_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * ============================================================
 * MODO DE OPERAÇÃO
 * ============================================================
 * Descomente SELFTEST_MODE para teste standalone (AP mode)
 * Comente para produção (STA mode - conecta no carrinho)
 */
#define SELFTEST_MODE  // <-- DESCOMENTE ESTA LINHA PARA TESTE!

#ifdef SELFTEST_MODE
    // MODO AP: ESP32-CAM cria sua própria rede WiFi
    #define WIFI_SSID           "godar-cam"
    #define WIFI_PASS           "godar-cam123"
    #define WIFI_CHANNEL        1
    #define WIFI_MAX_CONN       4
    #define WIFI_AP_IP          "192.168.4.2"
    #define WIFI_AP_GATEWAY     "192.168.4.1"
    #define WIFI_AP_NETMASK     "255.255.255.0"
#else
    // MODO STA: ESP32-CAM conecta na rede do carrinho
    #define WIFI_SSID           "GODAR_WIFI"
    #define WIFI_PASS           "godar123"
    #define WIFI_MAX_RETRY      10
#endif

/**
 * @brief Inicializa WiFi (AP ou STA baseado em SELFTEST_MODE)
 * @return true se sucesso
 */
bool wifi_init(void);

/**
 * @brief Verifica se está conectado (STA) ou AP ativo
 * @return true se operacional
 */
bool wifi_is_connected(void);

/**
 * @brief Obtém o IP local
 * @param ip_str Buffer para armazenar IP
 * @param max_len Tamanho do buffer
 * @return true se IP válido
 */
bool wifi_get_ip(char *ip_str, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CLIENT_H
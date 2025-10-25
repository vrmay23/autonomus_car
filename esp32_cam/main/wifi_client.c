/**
 * @file wifi_client.c
 * @brief Implementação WiFi Client/AP para ESP32-CAM
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#include "wifi_client.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/ip_addr.h"  // <-- ADICIONADO (para ipaddr_addr)

static const char *TAG = "[WIFI]";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Variáveis condicionalmente compiladas
#ifndef SELFTEST_MODE
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#endif

static bool s_wifi_ready = false;
static char s_ip_addr[16] = {0};

/**
 * ============================================================
 * EVENT HANDLERS
 * ============================================================
 */

#ifdef SELFTEST_MODE
// Modo AP: não precisa de event handler complexo
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    (void)arg;        // Suprime warning
    (void)event_data; // Suprime warning
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Cliente conectado ao AP");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Cliente desconectado do AP");
    }
}
#else
// Modo STA: handler com retry logic
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Tentando reconectar... (%d/%d)", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Falha ao conectar após %d tentativas", WIFI_MAX_RETRY);
        }
        s_wifi_ready = false;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(s_ip_addr, sizeof(s_ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "IP obtido: %s", s_ip_addr);
        s_retry_num = 0;
        s_wifi_ready = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
#endif

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS
 * ============================================================
 */

bool wifi_init(void)
{
    // Inicializa NVS (necessário para WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializa TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifdef SELFTEST_MODE
    // ======================================================
    // MODO AP (Access Point)
    // ======================================================
    ESP_LOGI(TAG, "Inicializando modo AP (SELFTEST)");
    
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    
    // Configura IP estático
    esp_netif_dhcps_stop(netif);
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
    ip_info.ip.addr = ipaddr_addr(WIFI_AP_IP);
    ip_info.gw.addr = ipaddr_addr(WIFI_AP_GATEWAY);
    ip_info.netmask.addr = ipaddr_addr(WIFI_AP_NETMASK);
    esp_netif_set_ip_info(netif, &ip_info);
    esp_netif_dhcps_start(netif);
    
    strncpy(s_ip_addr, WIFI_AP_IP, sizeof(s_ip_addr));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = WIFI_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_ready = true;
    
    ESP_LOGI(TAG, "AP iniciado:");
    ESP_LOGI(TAG, "  SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "  Password: %s", WIFI_PASS);
    ESP_LOGI(TAG, "  IP: %s", WIFI_AP_IP);
    ESP_LOGI(TAG, "  Gateway: %s", WIFI_AP_GATEWAY);

#else
    // ======================================================
    // MODO STA (Station)
    // ======================================================
    ESP_LOGI(TAG, "Inicializando modo STA (Produção)");
    
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler,
                                                         NULL,
                                                         &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Conectando ao AP: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Conectado ao AP: %s", WIFI_SSID);
        ESP_LOGI(TAG, "IP: %s", s_ip_addr);
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Falha ao conectar ao AP: %s", WIFI_SSID);
        return false;
    }
#endif

    return s_wifi_ready;
}

bool wifi_is_connected(void)
{
    return s_wifi_ready;
}

bool wifi_get_ip(char *ip_str, size_t max_len)
{
    if (!s_wifi_ready || strlen(s_ip_addr) == 0) {
        return false;
    }
    
    strncpy(ip_str, s_ip_addr, max_len - 1);
    ip_str[max_len - 1] = '\0';
    return true;
}
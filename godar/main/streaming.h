/**
 * @file streaming.h
 * @brief Cliente HTTP para comunicação com ESP32-CAM
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#ifndef STREAMING_H
#define STREAMING_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Configurações do ESP32-CAM
 */
#define ESP32CAM_IP              "192.168.4.2"
#define ESP32CAM_PORT            80
#define ESP32CAM_STREAM_PORT     81

/**
 * Intervalo de polling (em ms)
 */
#define STREAMING_POLL_INTERVAL_MS   500

/**
 * Estrutura de dados dos sensores
 */
typedef struct {
    int front_cm;      // Distância do sensor frontal (cm), -1 se erro
    int rear_cm;       // Distância do sensor traseiro (cm), -1 se erro
    bool valid;        // true se os dados são válidos
} streaming_sensors_t;

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - INICIALIZAÇÃO
 * ============================================================
 */

/**
 * @brief Inicializa o módulo de streaming
 * @note Deve ser chamado após wifi_init_softap()
 */
void streaming_init(void);

/**
 * @brief Inicia a task de polling automático dos sensores
 * @note Atualiza os dados dos sensores a cada STREAMING_POLL_INTERVAL_MS
 */
void streaming_start_poll_task(void);

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - LEITURA DE SENSORES
 * ============================================================
 */

/**
 * @brief Obtém os dados mais recentes dos sensores
 * @param sensors Ponteiro para estrutura que receberá os dados
 * @return true se os dados são válidos, false caso contrário
 */
bool streaming_get_sensors(streaming_sensors_t *sensors);

/**
 * @brief Força uma leitura imediata dos sensores (HTTP GET)
 * @param sensors Ponteiro para estrutura que receberá os dados
 * @return true se a requisição foi bem-sucedida
 */
bool streaming_fetch_sensors(streaming_sensors_t *sensors);

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - CONTROLE REMOTO
 * ============================================================
 */

/**
 * @brief Liga ou desliga o buzzer
 * @param state true para ligar, false para desligar
 * @return true se o comando foi enviado com sucesso
 */
bool streaming_set_buzzer(bool state);

/**
 * @brief Liga ou desliga o flash da câmera
 * @param state true para ligar, false para desligar
 * @return true se o comando foi enviado com sucesso
 */
bool streaming_set_flash(bool state);

/**
 * @brief Liga ou desliga a câmera
 * @param state true para ligar, false para desligar
 * @return true se o comando foi enviado com sucesso
 */
bool streaming_set_camera(bool state);

/**
 * ============================================================
 * FUNÇÕES PÚBLICAS - STATUS
 * ============================================================
 */

/**
 * @brief Verifica se o ESP32-CAM está conectado e respondendo
 * @return true se está conectado
 */
bool streaming_is_connected(void);

/**
 * @brief Obtém o timestamp da última atualização dos sensores
 * @return Timestamp em ms (desde boot)
 */
uint32_t streaming_get_last_update_time(void);

#endif // STREAMING_H

/**
 * @file camera.h
 * @brief Controle da Câmera OV2640 - ESP32-CAM
 * @author VINICIUS RODRIGO MAY & CLARICE MOURA SANTOS
 * @date 2025-10-19
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include "esp_camera.h"
#include "esp_http_server.h"

/**
 * Configurações da câmera
 */
// #define CAMERA_FRAME_SIZE    FRAMESIZE_VGA      // 640x480 (Muito pesado para Single-Core)
// #define CAMERA_FRAME_SIZE    FRAMESIZE_QVGA        // 320x240 (Use este)
// #define CAMERA_FRAME_SIZE    FRAMESIZE_XGA         // 1024x768 (Recomendado para Stream)
// #define CAMERA_FRAME_SIZE    FRAMESIZE_SXGA     // 1280x1024 (Quase HD, se a PSRAM aguentar)
// #define CAMERA_FRAME_SIZE    FRAMESIZE_UXGA     // 1600x1200 (Full, pode ser lento no ESP32)
#define CAMERA_FRAME_SIZE    FRAMESIZE_SVGA     // 800x600 (Maior chance de funcionar que XGA)
#define CAMERA_JPEG_QUALITY  30                 // Manter baixa para evitar estouro de buffer

/**
 * @brief Inicializa a câmera OV2640
 * @return true se sucesso
 */
bool camera_init(void);

/**
 * @brief Inicia servidor de streaming MJPEG na porta 81
 * @return Handle do servidor ou NULL
 */
httpd_handle_t camera_start_stream_server(void);

/**
 * @brief Liga/desliga câmera
 */
bool camera_set_power(bool state);

/**
 * @brief Verifica se câmera está ligada
 */
bool camera_is_powered(void);

/**
 * @brief Liga/desliga flash
 */
bool camera_set_flash(bool state);

/**
 * @brief Verifica estado do flash
 */
bool camera_is_flash_on(void);

#endif // CAMERA_H
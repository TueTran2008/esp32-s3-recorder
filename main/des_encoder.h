
#ifndef _DES_ENCRYPT_H_
#define _DES_ENCODER_H_

#include "audio_element.h"
#include "esp_err.h"
#include "mbedtls/des.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief      DES Encoder configurations
 */
typedef struct {
    int out_rb_size;   /*!< Size of output ringbuffer */
    int task_stack;    /*!< Task stack size */
    int task_core;     /*!< Task running in core (0 or 1) */
    int task_prio;     /*!< Task priority (based on freeRTOS priority) */
    bool stack_in_ext; /*!< Try to allocate stack in external memory */
    uint32_t header_size;
    uint32_t block_size;
    uint8_t *key;
} des_encrypt_cfg_t;

typedef struct {
    uint32_t header_size;
    uint32_t block_size;
    uint8_t key[8];
    uint8_t *buffer;
    mbedtls_des_context ctx;
} des_encrypt_t;

#define DES_ENCODER_TASK_STACK (3 * 1024)
#define DES_ENCODER_TASK_CORE (0)
#define DES_ENCODER_TASK_PRIO (5)
#define DES_ENCODER_RINGBUFFER_SIZE (8 * 1024)
#define HEADER_SIZE 44 // Standard WAV header size
#define BLOCK_SIZE 8   // DES block size
#define DEFAULT_DES_ENCODER_CONFIG()                                                                                                                                                                   \
    {                                                                                                                                                                                                  \
        .out_rb_size = DES_ENCODER_RINGBUFFER_SIZE,                                                                                                                                                    \
        .task_stack = DES_ENCODER_TASK_STACK,                                                                                                                                                          \
        .task_core = DES_ENCODER_TASK_CORE,                                                                                                                                                            \
        .task_prio = DES_ENCODER_TASK_PRIO,                                                                                                                                                            \
        .stack_in_ext = true,                                                                                                                                                                          \
    }

/**
 * @brief      Create a handle to an Audio Element to encode incoming data using DES format
 *
 * @param      config  The configuration
 *
 * @return     The audio element handle
 */
audio_element_handle_t des_encoder_init(des_encrypt_cfg_t *config);

#ifdef __cplusplus
}
#endif

#endif


// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include "des_encoder.h"
#include "audio_element.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "esp_log.h"
#include "mbedtls/des.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "DES_ENCODER";

typedef struct des_encoder {
    bool parsed_header;
} des_encoder_t;

static void encrypt_wav_buffer(audio_element_handle_t self, const uint8_t *input, size_t input_len, uint8_t *output, size_t *output_len) {
    des_encrypt_t *des = (des_encrypt_t *)audio_element_getdata(self);
    if (input_len < des->header_size) {
        ESP_LOGE(TAG, "Input data too small for WAV file.");
        return;
    }
    // Copy WAV header directly
    memcpy(output, input, des->header_size);

    mbedtls_des_context ctx;

    const uint8_t *data_start = input + des->header_size;
    uint8_t *output_start = output + des->header_size;
    size_t data_len = input_len - des->header_size;

    // Process WAV data in 8-byte blocks
    size_t block_count = data_len / des->block_size;
    size_t remaining = data_len % des->block_size;

    for (size_t i = 0; i < block_count; i++) {
        mbedtls_des_crypt_ecb(&ctx, data_start + (i * des->block_size), output_start + (i * des->block_size));
    }

    // Handle padding for the last block
    if (remaining > 0) {
        uint8_t padded_block[des->block_size];
        memset(padded_block, 0, sizeof(padded_block));
        memcpy(padded_block, data_start + (block_count * des->block_size), remaining);
        memset(padded_block + remaining, des->block_size - remaining, des->block_size - remaining);

        mbedtls_des_crypt_ecb(&ctx, padded_block, output_start + (block_count * des->block_size));
        *output_len = des->header_size + (block_count + 1) * des->block_size;
    } else {
        *output_len = des->header_size + block_count * des->block_size;
    }

    ESP_LOGI(TAG, "Encryption complete.");
}

static esp_err_t _des_encoder_destroy(audio_element_handle_t self) {
    des_encrypt_t *des = (des_encrypt_t *)audio_element_getdata(self);
    mbedtls_des_free(&des->ctx);
    audio_free(des);
    return ESP_OK;
}
static esp_err_t _des_encoder_open(audio_element_handle_t self) {
    ESP_LOGE(TAG, "_des_encoder_open");
    return ESP_OK;
}

static esp_err_t _des_encoder_close(audio_element_handle_t self) {
    ESP_LOGE(TAG, "_des_encoder_close");
    if (AEL_STATE_PAUSED != audio_element_get_state(self)) {
        audio_element_set_byte_pos(self, 0);
        audio_element_set_total_bytes(self, 0);
    }
    return ESP_OK;
}

static int _des_encoder_process(audio_element_handle_t self, char *in_buffer, int in_len) {
    int r_size = audio_element_input(self, in_buffer, in_len);
    des_encrypt_t *des = (des_encrypt_t *)audio_element_getdata(self);
    // uint8_t buffer[r_size * 2];
    size_t buffer_len = 0;
    /* memset(des->buffer, 0, 2 * DES_BUFFER_SIZE); */
    encrypt_wav_buffer(self, (uint8_t *)in_buffer, r_size, (uint8_t *)des->buffer, &buffer_len);
    int out_len = r_size;
    if (r_size > 0) {
        out_len = audio_element_output(self, (char *)des->buffer, buffer_len);
        if (out_len > 0) {
            audio_element_update_byte_pos(self, out_len);
        }
    }

    return out_len;
}

audio_element_handle_t des_encoder_init(des_encrypt_cfg_t *config) {
    des_encrypt_t *des = audio_calloc(1, sizeof(des_encrypt_t));
    AUDIO_MEM_CHECK(TAG, des, { return NULL; });
    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.destroy = _des_encoder_destroy;
    cfg.process = _des_encoder_process;
    cfg.open = _des_encoder_open;
    cfg.close = _des_encoder_close;
    cfg.task_stack = DES_ENCODER_TASK_STACK;
    if (config) {
        if (config->task_stack) {
            cfg.task_stack = config->task_stack;
        }
        cfg.stack_in_ext = config->stack_in_ext;
        cfg.task_prio = config->task_prio;
        cfg.task_core = config->task_core;
        cfg.out_rb_size = config->out_rb_size;
    }

    cfg.tag = "des";
    des->block_size = config->block_size;
    des->header_size = config->header_size;

    bool _success = true;
    des->buffer = (uint8_t *)audio_calloc(1, 2 * DES_BUFFER_SIZE);
    if (des->buffer == NULL) {
        _success = false;
    }
    //_success = ((des->buffer = (uint8_t*)audio_calloc(1, 2 * DES_BUFFER_SIZE) != NULL);

    AUDIO_NULL_CHECK(TAG, _success, {
        ESP_LOGE(TAG, "Error occured");
        // _des_encoder_destroy(el);
        audio_free(des);
        return NULL;
    });
    memcpy(des->key, config->key, sizeof(des->key));
    mbedtls_des_init(&des->ctx);
    mbedtls_des_setkey_enc(&des->ctx, des->key);
    audio_element_handle_t el = audio_element_init(&cfg);
    AUDIO_MEM_CHECK(TAG, el, {
        audio_free(des);
        return NULL;
    });
    audio_element_setdata(el, des);
    ESP_LOGD(TAG, "des_encoder_init");
    return el;
}

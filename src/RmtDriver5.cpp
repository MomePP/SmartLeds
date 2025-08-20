#include "RmtDriver5.h"

#if SMARTLEDS_NEW_RMT_DRIVER
#include <cstddef>
#include <cstring>
#include <esp_heap_caps.h>
#include <esp_cache.h>
#include <hal/cache_hal.h>
#include <hal/cache_ll.h>

#include "SmartLeds.h"

// Helper macro for getting non-cached address
#if CONFIG_IDF_TARGET_ESP32S3 || (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
#define RMT_GET_NON_CACHE_ADDR(addr) ((void*)(((intptr_t)(addr)) | 0x40000000))
#else
#define RMT_GET_NON_CACHE_ADDR(addr) (addr)
#endif

// Helper function to perform cache synchronization for ESP-IDF v5.5+
static esp_err_t rmt_cache_sync_buffer(void* buffer, size_t size) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
    uint32_t data_cache_line_size = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
    if (data_cache_line_size && buffer) {
        return esp_cache_msync(buffer, size, 
                              ESP_CACHE_MSYNC_FLAG_DIR_C2M | 
                              ESP_CACHE_MSYNC_FLAG_UNALIGNED | 
                              ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    }
#endif
    return ESP_OK;
}

namespace led_timing {

static constexpr const uint32_t RMT_RESOLUTION_HZ = 20 * 1000 * 1000; // 20 MHz
static constexpr const uint32_t RMT_NS_PER_TICK = 1000000000LLU / RMT_RESOLUTION_HZ;

static RmtEncoderWrapper* IRAM_ATTR encSelf(rmt_encoder_t* encoder) {
    return (RmtEncoderWrapper*)(((intptr_t)encoder) - offsetof(RmtEncoderWrapper, base));
}

static size_t IRAM_ATTR encEncode(rmt_encoder_t* encoder, rmt_channel_handle_t tx_channel, const void* primary_data,
    size_t data_size, rmt_encode_state_t* ret_state) {
    auto* self = encSelf(encoder);

    // Delay after last pixel
    if ((self->last_state & RMT_ENCODING_COMPLETE) && self->frame_idx == data_size) {
        *ret_state = (rmt_encode_state_t)0;
        return self->copy_encoder->encode(
            self->copy_encoder, tx_channel, (const void*)&self->reset_code, sizeof(self->reset_code), ret_state);
    }

    if (self->last_state & RMT_ENCODING_COMPLETE) {
        // Use a fallback static buffer if dynamic allocation failed
        static uint8_t fallback_buffer[SOC_RMT_MEM_WORDS_PER_CHANNEL / 8] __attribute__((aligned(32)));
        
        uint8_t* buffer_ptr;
        size_t buffer_size;
        
        if (self->buffer) {
            // Use dynamically allocated DMA-capable buffer
            buffer_ptr = self->buffer_nc ? self->buffer_nc : self->buffer;
            buffer_size = self->buffer_size;
        } else {
            // Fallback to static buffer for older ESP-IDF versions
            buffer_ptr = fallback_buffer;
            buffer_size = sizeof(fallback_buffer);
        }
        
        // Clear buffer to prevent stale data corruption in ESP-IDF v5.5+
        memset(buffer_ptr, 0, buffer_size);
        
        size_t buffer_pos = 0;
        
        // Fill buffer with pixel data, respecting all bounds
        while (buffer_pos < buffer_size && self->frame_idx < data_size) {
            // Safe pixel access with explicit bounds checking
            const Rgb* pixel = ((const Rgb*)primary_data) + self->frame_idx;
            
            // Get the color component for current position
            uint8_t color_byte = pixel->getGrb(self->component_idx);
            buffer_ptr[buffer_pos] = color_byte;
            buffer_pos++;
            
            // Move to next component
            self->component_idx++;
            if (self->component_idx >= 3) {
                self->component_idx = 0;
                self->frame_idx++;
            }
        }
        
        self->buffer_len = buffer_pos;
        
        // Synchronize cache for ESP-IDF v5.5+ DMA operations
        if (self->buffer) {
            rmt_cache_sync_buffer(self->buffer, self->buffer_size);
        }
    }

    self->last_state = (rmt_encode_state_t)0;
    
    // Use the appropriate buffer for encoding
    const void* encode_buffer;
    if (self->buffer) {
        encode_buffer = (const void*)self->buffer;
    } else {
        static uint8_t fallback_buffer[SOC_RMT_MEM_WORDS_PER_CHANNEL / 8] __attribute__((aligned(32)));
        encode_buffer = (const void*)fallback_buffer;
    }
    
    auto encoded_symbols = self->bytes_encoder->encode(
        self->bytes_encoder, tx_channel, encode_buffer, self->buffer_len, &self->last_state);
    if (self->last_state & RMT_ENCODING_MEM_FULL) {
        *ret_state = RMT_ENCODING_MEM_FULL;
    } else {
        *ret_state = (rmt_encode_state_t)0;
    }

    return encoded_symbols;
}

static esp_err_t encReset(rmt_encoder_t* encoder) {
    auto* self = encSelf(encoder);
    rmt_encoder_reset(self->bytes_encoder);
    rmt_encoder_reset(self->copy_encoder);
    self->last_state = RMT_ENCODING_COMPLETE;
    self->frame_idx = 0;
    self->component_idx = 0;
    return ESP_OK;
}

static esp_err_t encDelete(rmt_encoder_t* encoder) {
    auto* self = encSelf(encoder);
    rmt_del_encoder(self->bytes_encoder);
    rmt_del_encoder(self->copy_encoder);
    return ESP_OK;
}

RmtDriver::RmtDriver(const LedType& timing, int count, int pin, int channel_num, SemaphoreHandle_t finishedFlag)
    : _timing(timing)
    , _count(count)
    , _pin(pin)
    , _finishedFlag(finishedFlag)
    , _channel(nullptr)
    , _encoder {} {}

esp_err_t RmtDriver::init() {
    // Initialize encoder buffer for ESP-IDF v5.5+ compatibility
    _encoder.buffer_size = SOC_RMT_MEM_WORDS_PER_CHANNEL / 8;
    
    // Try to allocate DMA-capable buffer with proper alignment for ESP-IDF v5.5+
    _encoder.buffer = (uint8_t*)heap_caps_aligned_calloc(32, 1, _encoder.buffer_size, 
                                                         MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (_encoder.buffer) {
        // Get non-cached address for DMA operations
        _encoder.buffer_nc = (uint8_t*)RMT_GET_NON_CACHE_ADDR(_encoder.buffer);
        
        // Perform initial cache synchronization
        esp_err_t cache_err = rmt_cache_sync_buffer(_encoder.buffer, _encoder.buffer_size);
        if (cache_err != ESP_OK) {
            heap_caps_free(_encoder.buffer);
            _encoder.buffer = nullptr;
            _encoder.buffer_nc = nullptr;
        }
    }
    
    // If dynamic allocation failed, use a simpler approach for older versions
    if (!_encoder.buffer) {
        _encoder.buffer = nullptr;
        _encoder.buffer_nc = nullptr;
    }

    _encoder.base.encode = encEncode;
    _encoder.base.reset = encReset;
    _encoder.base.del = encDelete;

    _encoder.reset_code.duration0 = _timing.TRS / RMT_NS_PER_TICK;

    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = {
            .duration0 = uint16_t(_timing.T0H  / RMT_NS_PER_TICK),
            .level0 = 1,
            .duration1 = uint16_t(_timing.T0L  / RMT_NS_PER_TICK),
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = uint16_t(_timing.T1H / RMT_NS_PER_TICK),
            .level0 = 1,
            .duration1 = uint16_t(_timing.T1L  / RMT_NS_PER_TICK),
            .level1 = 0,
        },
        .flags = {
            .msb_first = 1,
        },
    };

    auto err = rmt_new_bytes_encoder(&bytes_cfg, &_encoder.bytes_encoder);
    if (err != ESP_OK) {
        // Clean up allocated buffer on error
        if (_encoder.buffer) {
            heap_caps_free(_encoder.buffer);
            _encoder.buffer = nullptr;
            _encoder.buffer_nc = nullptr;
        }
        return err;
    }

    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &_encoder.copy_encoder);
    if (err != ESP_OK) {
        // Clean up on error
        rmt_del_encoder(_encoder.bytes_encoder);
        if (_encoder.buffer) {
            heap_caps_free(_encoder.buffer);
            _encoder.buffer = nullptr;
            _encoder.buffer_nc = nullptr;
        }
        return err;
    }

    // The config must be in registerIsr, because rmt_new_tx_channel
    // registers the ISR
    return ESP_OK;
}

esp_err_t RmtDriver::registerIsr(bool isFirstRegisteredChannel) {
    rmt_tx_channel_config_t conf = {
        .gpio_num = (gpio_num_t)_pin,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .trans_queue_depth = 1,
        .flags = {},
    };

    auto err = rmt_new_tx_channel(&conf, &_channel);
    if (err != ESP_OK) {
        return err;
    }

    rmt_tx_event_callbacks_t callbacks_cfg = {};
    callbacks_cfg.on_trans_done = txDoneCallback;

    err = rmt_tx_register_event_callbacks(_channel, &callbacks_cfg, this);
    if (err != ESP_OK) {
        return err;
    }

    return rmt_enable(_channel);
}

esp_err_t RmtDriver::unregisterIsr() {
    auto err = rmt_del_encoder(&_encoder.base);
    if (err != ESP_OK) {
        return err;
    }

    // Clean up dynamically allocated buffer
    if (_encoder.buffer) {
        heap_caps_free(_encoder.buffer);
        _encoder.buffer = nullptr;
        _encoder.buffer_nc = nullptr;
    }

    err = rmt_disable(_channel);
    if (err != ESP_OK) {
        return err;
    }

    return rmt_del_channel(_channel);
}

bool IRAM_ATTR RmtDriver::txDoneCallback(
    rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t* edata, void* user_ctx) {
    auto* self = (RmtDriver*)user_ctx;
    auto taskWoken = pdTRUE;
    xSemaphoreGiveFromISR(self->_finishedFlag, &taskWoken);
    return taskWoken == pdTRUE;
}

esp_err_t RmtDriver::transmit(const Rgb* buffer) {
    rmt_encoder_reset(&_encoder.base);
    rmt_transmit_config_t cfg = {};
    return rmt_transmit(_channel, &_encoder.base, buffer, _count, &cfg);
}
};
#endif // !SMARTLEDS_NEW_RMT_DRIVER

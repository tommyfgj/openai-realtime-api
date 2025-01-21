#include <driver/i2s.h>
#include <opus.h>
#include <esp_log.h>
#include "esp_audio_dec_default.h"
#include "esp_audio_dec.h"
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_system.h"
#include <stdlib.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"



#include "main.h"
#include "media.h"
#include "webrtc.h"

#define OPUS_OUT_BUFFER_SIZE 1276  // 1276 bytes is recommended by opus_encode
#define SAMPLE_RATE 8000
#define BUFFER_SAMPLES 320

#define MCLK_PIN 0
#define DAC_BCLK_PIN 15
#define DAC_LRCLK_PIN 16
#define DAC_DATA_PIN 7
#define ADC_BCLK_PIN 4
#define ADC_LRCLK_PIN 5
#define ADC_DATA_PIN 6

#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0


typedef union {
    esp_opus_dec_cfg_t  opus_cfg;
    esp_adpcm_dec_cfg_t adpcm_cfg;
    esp_alac_dec_cfg_t  alac_cfg;
    esp_aac_dec_cfg_t   aac_cfg;
    esp_g711_dec_cfg_t  g711_cfg;
} dec_all_cfg_t;

typedef struct {
    uint8_t *data;
    int      read_size;
    int      size;
} read_ctx_t;

typedef struct {
    bool                      use_common_api;
    esp_audio_dec_handle_t    decoder;
    esp_audio_dec_out_frame_t out_frame;
    bool                      decode_err;
} write_ctx_t;

static read_ctx_t  read_ctx;
static write_ctx_t write_ctx;

esp_audio_dec_handle_t  g_decoder = NULL ;

static uint64_t get_timestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void oai_init_audio_capture() {
  i2s_config_t i2s_config_out = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_STAND_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 3,
      .dma_buf_len = BUFFER_SAMPLES,
      .use_apll = false,
        .tx_desc_auto_clear = true,            // 自动清除描述符
        .fixed_mclk = 0
  };
  if (i2s_driver_install(I2S_NUM_1, &i2s_config_out, 0, NULL) != ESP_OK) {
    printf("Failed to configure I2S driver for audio output");
    return;
  }

  i2s_pin_config_t pin_config_out = {
      // .mck_io_num = MCLK_PIN,
      .bck_io_num = DAC_BCLK_PIN,
      .ws_io_num = DAC_LRCLK_PIN,
      .data_out_num = DAC_DATA_PIN,
      .data_in_num = I2S_PIN_NO_CHANGE,
  };
  if (i2s_set_pin(I2S_NUM_1, &pin_config_out) != ESP_OK) {
    printf("Failed to set I2S pins for audio output");
    return;
  }
  i2s_zero_dma_buffer(I2S_NUM_1);

  i2s_config_t i2s_config_in = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = BUFFER_SAMPLES,
      .use_apll = 1,
  };
  if (i2s_driver_install(I2S_NUM_0, &i2s_config_in, 0, NULL) != ESP_OK) {
    printf("Failed to configure I2S driver for audio input");
    return;
  }

  i2s_pin_config_t pin_config_in = {
      .mck_io_num = MCLK_PIN,
      .bck_io_num = ADC_BCLK_PIN,
      .ws_io_num = ADC_LRCLK_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = ADC_DATA_PIN,
  };
  if (i2s_set_pin(I2S_NUM_0, &pin_config_in) != ESP_OK) {
    printf("Failed to set I2S pins for audio input");
    return;
  }

}

opus_int16 *output_buffer = NULL;
OpusDecoder *opus_decoder = NULL;

// A-Law to PCM16 conversion table
static const int16_t aLawToPcmTable[256] = { -5504, -5248, -6016, -5760, -4480, -4224,
  -4992, -4736, -7552, -7296, -8064, -7808, -6528, -6272, -7040, -6784,
  -2752, -2624, -3008, -2880, -2240, -2112, -2496, -2368, -3776, -3648,
  -4032, -3904, -3264, -3136, -3520, -3392, -22016, -20992, -24064,
  -23040, -17920, -16896, -19968, -18944, -30208, -29184, -32256,
  -31232, -26112, -25088, -28160, -27136, -11008, -10496, -12032,
  -11520, -8960, -8448, -9984, -9472, -15104, -14592, -16128, -15616,
  -13056, -12544, -14080, -13568, -344, -328, -376, -360, -280, -264,
  -312, -296, -472, -456, -504, -488, -408, -392, -440, -424, -88, -72,
  -120, -104, -24, -8, -56, -40, -216, -200, -248, -232, -152, -136,
  -184, -168, -1376, -1312, -1504, -1440, -1120, -1056, -1248, -1184,
  -1888, -1824, -2016, -1952, -1632, -1568, -1760, -1696, -688, -656,
  -752, -720, -560, -528, -624, -592, -944, -912, -1008, -976, -816,
  -784, -880, -848, 5504, 5248, 6016, 5760, 4480, 4224, 4992, 4736,
  7552, 7296, 8064, 7808, 6528, 6272, 7040, 6784, 2752, 2624, 3008,
  2880, 2240, 2112, 2496, 2368, 3776, 3648, 4032, 3904, 3264, 3136,
  3520, 3392, 22016, 20992, 24064, 23040, 17920, 16896, 19968, 18944,
  30208, 29184, 32256, 31232, 26112, 25088, 28160, 27136, 11008, 10496,
  12032, 11520, 8960, 8448, 9984, 9472, 15104, 14592, 16128, 15616,
  13056, 12544, 14080, 13568, 344, 328, 376, 360, 280, 264, 312, 296,
  472, 456, 504, 488, 408, 392, 440, 424, 88, 72, 120, 104, 24, 8, 56,
  40, 216, 200, 248, 232, 152, 136, 184, 168, 1376, 1312, 1504, 1440,
  1120, 1056, 1248, 1184, 1888, 1824, 2016, 1952, 1632, 1568, 1760,
  1696, 688, 656, 752, 720, 560, 528, 624, 592, 944, 912, 1008, 976,
  816, 784, 880, 848};

int16_t alawToPCM(uint8_t alawbyte) {
    alawbyte ^= 0x55;
    int sign = (alawbyte & 0x80) != 0;
    int exponent = (alawbyte & 0x70) >> 4;
    int16_t data = (alawbyte & 0x0F);

    data <<= 4;
    data += 8;

    if (exponent != 0) {
        data += 0x100;
    }
    if (exponent > 1) {
        data <<= (exponent - 1);
    }

    if (sign) {
        return data;
    }
    return -data;
}

int16_t convertALawToPCM16(uint8_t aLawByte) {
    // return aLawToPcmTable[aLawByte];
    return alawToPCM(aLawByte)*1.5;
}

// Convert a buffer of PCMA data to PCM16
void convertALawBufferToPCM16(const uint8_t *aLawData, int16_t *pcmData, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        pcmData[i] = convertALawToPCM16(aLawData[i]);
    }
}

void oai_init_audio_decoder() {
    printf("enter oai_init_audio_decoder\n");
    // xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);
  // int ret = 0;
  // esp_g711_dec_cfg_t g711_cfg = {
  //   .channel = 1,
  // };
  //  esp_audio_dec_cfg_t dec_cfg = {
  //       .type = ESP_AUDIO_TYPE_G711A,
  //       .cfg = &g711_cfg,
  //       .cfg_sz = sizeof(esp_g711_dec_cfg_t),
  //   };
  // esp_g711a_dec_register();
  // ret = esp_audio_dec_open(&dec_cfg, &g_decoder);
  // if (ret != ESP_AUDIO_ERR_OK) {
  //   ESP_LOGE(LOG_TAG, "Fail to open decoder ret %d", ret);
  //   esp_restart();
  //   return;
  // }
  // ESP_LOGI(LOG_TAG, "esp_audio_dec_open SUCCESS");

  // int decoder_error = 0;
  // opus_decoder = opus_decoder_create(SAMPLE_RATE, 2, &decoder_error);
  // if (decoder_error != OPUS_OK) {
  //   printf("Failed to create OPUS decoder");
  //   return;
  // }

  // output_buffer = (opus_int16 *)malloc(BUFFER_SAMPLES * sizeof(opus_int16));
}

// #define PCM_BUFFER_SIZE 7680 // 8KB

// static int16_t pcmBuffer[PCM_BUFFER_SIZE / sizeof(int16_t)]; // 缓存用于存储PCM数据
// static size_t pcmBufferIndex = 0; // 当前缓存写入位置

#define RINGBUFFER_SIZE (160 * 1024) // 160KB的RingBuffer
#define BUFFER_THRESHOLD (1 * 320) // 8KB的阈值

static RingbufHandle_t xRingbuffer = NULL;

void init_ringbuffer(void) {
    xRingbuffer = xRingbufferCreate(RINGBUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (xRingbuffer == NULL) {
        ESP_LOGE(LOG_TAG, "Failed to create ring buffer");
    }
}

void oai_audio_decode(uint8_t *data, size_t size) {
    char buffer[21];
    snprintf(buffer, sizeof(buffer), "%llu", (unsigned long long)get_timestamp());
    ESP_LOGD(LOG_TAG, "oai_audio_decode: %s, size: %d", buffer, size);

    int16_t pcmData[size];
    convertALawBufferToPCM16(data, pcmData, size);

    if (xRingbufferSend(xRingbuffer, pcmData, size * sizeof(int16_t), portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(LOG_TAG, "Failed to write to ring buffer");
    }
}

void i2s_task(void *arg) {
    size_t item_size;
    int16_t *item;

    while (1) {
        size_t available_size = xRingbufferGetMaxItemSize(xRingbuffer);

        if (available_size >= BUFFER_THRESHOLD) {
            item = (int16_t *)xRingbufferReceive(xRingbuffer, &item_size, portMAX_DELAY);

            if (item != NULL) {
                size_t bytes_written = 0;
                i2s_write(I2S_NUM_1, item, item_size, &bytes_written, portMAX_DELAY);
                vRingbufferReturnItem(xRingbuffer, (void *)item);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // 每100ms检查一次
    }
}

void start_i2s_task(void) {
    const BaseType_t core_id = 1; // 核心0为第一个核心，核心1为第二个核心
    xTaskCreatePinnedToCore(i2s_task, "i2s_task", 2048, NULL, 5, NULL, core_id);
}
// void oai_audio_decode(uint8_t *data, size_t size) {
//     char buffer[21]; // 最大20位数字加上终止符
//     snprintf(buffer, sizeof(buffer), "%llu", (unsigned long long)get_timestamp());
//     ESP_LOGD(LOG_TAG, "oai_audio_decode: %s, size: %d", buffer, size);
    
//     int16_t pcmData[size];
//     convertALawBufferToPCM16(data, pcmData, size);

//     // 将解码后的PCM数据存储到缓存中
//     if (pcmBufferIndex + size > PCM_BUFFER_SIZE / sizeof(int16_t)) {
//         // 如果缓存中的数据超过8KB，执行I2S操作
//         size_t bytes_written = 0;
//         i2s_write(I2S_NUM_1, pcmBuffer, pcmBufferIndex * sizeof(int16_t),
//                   &bytes_written, portMAX_DELAY);

//         // 清空缓存
//         pcmBufferIndex = 0;
//     }
    
//     // 将当前PCM数据拷贝到缓存中
//     memcpy(&pcmBuffer[pcmBufferIndex], pcmData, size * sizeof(int16_t));
//     pcmBufferIndex += size;
// }

// void oai_audio_decode(uint8_t *data, size_t size) {
//   char buffer[21]; // 最大20位数字加上终止符
//   snprintf(buffer, sizeof(buffer), "%llu", (unsigned long long)get_timestamp());
//   ESP_LOGD(LOG_TAG, "oai_audio_decode: %s, size: %d", buffer, size);

//   int16_t pcmData[size];
//   convertALawBufferToPCM16(data, pcmData, size);
//   size_t bytes_written = 0;
//   i2s_write(I2S_NUM_1, pcmData, size,
//             &bytes_written, portMAX_DELAY);
// }

OpusEncoder *opus_encoder = NULL;
opus_int16 *encoder_input_buffer = NULL;
uint8_t *encoder_output_buffer = NULL;

void oai_init_audio_encoder() {
  int encoder_error;
  opus_encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP,
                                     &encoder_error);
  if (encoder_error != OPUS_OK) {
    printf("Failed to create OPUS encoder");
    return;
  }

  if (opus_encoder_init(opus_encoder, SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP) !=
      OPUS_OK) {
    printf("Failed to initialize OPUS encoder");
    return;
  }

  opus_encoder_ctl(opus_encoder, OPUS_SET_BITRATE(OPUS_ENCODER_BITRATE));
  opus_encoder_ctl(opus_encoder, OPUS_SET_COMPLEXITY(OPUS_ENCODER_COMPLEXITY));
  opus_encoder_ctl(opus_encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
  encoder_input_buffer = (opus_int16 *)malloc(BUFFER_SAMPLES);
  encoder_output_buffer = (uint8_t *)malloc(OPUS_OUT_BUFFER_SIZE);
}

#define ALAW_MAX 0xFFF
#define ALAW_BIAS 0x84

static uint8_t linear_to_alaw(int16_t pcm_val) {
    int16_t mask;
    int16_t seg;
    uint8_t aval;

    // Convert from 16-bit signed to 13-bit signed
    if (pcm_val >= 0) {
        mask = 0xD5; // Sign (7th) bit = 1
    } else {
        mask = 0x55; // Sign bit = 0
        pcm_val = -pcm_val - 1;
    }

    // Bias the input
    pcm_val += ALAW_BIAS;

    if (pcm_val > ALAW_MAX) {
        pcm_val = ALAW_MAX;
    }

    // Find the segment
    if (pcm_val >= 256) {
        seg = 7;
        for (int i = 0x4000; i > 0xFF; i >>= 1) {
            if (pcm_val & i) {
                seg++;
            }
        }
        aval = (seg << 4) | ((pcm_val >> (seg + 3)) & 0x0F);
    } else {
        aval = pcm_val >> 4;
    }

    return aval ^ mask;
}



void pcm_to_alaw(const int16_t* pcm_samples, uint8_t* alaw_samples, size_t num_samples) {
    for (size_t i = 0; i < num_samples; ++i) {
        alaw_samples[i] = linear_to_alaw(pcm_samples[i]);
    }
}


void oai_send_audio(PeerConnection *peer_connection) {
//   size_t bytes_read = 0;

//   int16_t buf[BUFFER_SAMPLES];
//   i2s_read(I2S_NUM_0, buf, BUFFER_SAMPLES*sizeof(int16_t), &bytes_read,
//            portMAX_DELAY);
//   // ESP_LOGI(LOG_TAG, "READ BYTE: %d", bytes_read);
//   uint8_t pcmaBuf[bytes_read/sizeof(int16_t)];
//   lin2alaw(buf, pcmaBuf, 1, 1, bytes_read/sizeof(int16_t));
//   // auto encoded_size =
//   //     opus_encode(opus_encoder, encoder_input_buffer, BUFFER_SAMPLES / 2,
//   //                 encoder_output_buffer, OPUS_OUT_BUFFER_SIZE);

//   peer_connection_send_audio(peer_connection, pcmaBuf,
//                              bytes_read/sizeof(int16_t));
}


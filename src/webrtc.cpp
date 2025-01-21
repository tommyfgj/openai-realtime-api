#ifndef LINUX_BUILD
#include <driver/i2s.h>
#include <opus.h>
#endif

#include <esp_event.h>
#include <esp_log.h>
#include <string.h>

#include "main.h"
#include "media.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#define TICK_INTERVAL 15
#define GREETING                                                    \
  "{\"type\": \"response.create\", \"response\": {\"modalities\": " \
  "[\"audio\", \"text\"], \"instructions\": \"Say 'How can I help?.'\"}}"


static TaskHandle_t xPcTaskHandle = NULL;
PeerConnection *peer_connection = NULL;

#ifndef LINUX_BUILD
StaticTask_t task_buffer;
void oai_send_audio_task(void *user_data) {
  oai_init_audio_encoder();

  while (1) {
    oai_send_audio(peer_connection);
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL));
  }
}
#endif

static void oai_ondatachannel_onmessage_task(char *msg, size_t len,
                                             void *userdata, uint16_t sid) {
  ESP_LOGI(LOG_TAG, "DataChannel Message: %s", msg);
}

static void oai_ondatachannel_onopen_task(void *userdata) {
  if (peer_connection_create_datachannel(peer_connection, DATA_CHANNEL_RELIABLE,
                                         0, 0, (char *)"oai-events",
                                         (char *)"") != -1) {
    ESP_LOGI(LOG_TAG, "DataChannel created");
    peer_connection_datachannel_send(peer_connection, (char *)GREETING,
                                     strlen(GREETING));
  } else {
    ESP_LOGE(LOG_TAG, "Failed to create DataChannel");
  }
}

static void oai_onconnectionstatechange_task(PeerConnectionState state,
                                             void *user_data) {
  ESP_LOGI(LOG_TAG, "PeerConnectionState: %s",
           peer_connection_state_to_string(state));

  if (state == PEER_CONNECTION_DISCONNECTED ||
      state == PEER_CONNECTION_CLOSED) {
#ifndef LINUX_BUILD
    esp_restart();
#endif
  } else if (state == PEER_CONNECTION_CONNECTED) {
#ifndef LINUX_BUILD
    StackType_t *stack_memory = (StackType_t *)heap_caps_malloc(
        20000 * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
    xTaskCreateStaticPinnedToCore(oai_send_audio_task, "audio_publisher", 20000,
                                  NULL, 7, stack_memory, &task_buffer, 0);
#endif
  }
}

static void oai_on_icecandidate_task(char *description, void *user_data) {
  char local_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0};
  oai_http_request(description, local_buffer);
  peer_connection_set_remote_description(peer_connection, local_buffer);
  // peer_signaling_http_post("s.sdad22624319.cn", "/whip", 8877, "", description);
}

#define QUANT_MASK      (0xf)           /* Quantization field mask. */
#define NSEGS           (8)             /* Number of A-law segments. */
#define SEG_SHIFT       (4)             /* Left shift for segment number. */

void lin2alaw(int16_t *linp, uint8_t *alawp, int linc, int ainc, long npts)
{
    int linear, seg; 
    uint8_t aval, mask;
    static int16_t seg_aend[NSEGS] 
        = {0x1f, 0x3f, 0x7f, 0xff, 0x1ff, 0x3ff, 0x7ff, 0xfff};
    long i;

    for (i = 0; i < npts; ++i) {
        linear = (*linp) >> 3;

        if (linear >= 0) {
            mask = 0xd5;                /* sign (7th) bit = 1 */
        } else {
            mask = 0x55;                /* sign bit = 0 */
            linear = -linear - 1;
        }

        /* Convert the scaled magnitude to segment number. */
        for (seg = 0; seg < NSEGS && linear > seg_aend[seg]; seg++);
  
        /* Combine the sign, segment, and quantization bits. */
        if (seg >= NSEGS) {             /* out of range, return maximum value. */
            aval = (uint8_t)(0x7F ^ mask);
        } else {
            aval = (uint8_t)seg << SEG_SHIFT;
            if (seg < 2)
                aval |= (linear >> 1) & QUANT_MASK;
            else
                aval |= (linear >> seg) & QUANT_MASK;
            aval = (aval ^ mask);
        }

        /* alaw */
        *alawp = aval;

        alawp += ainc;
        linp += linc;
    }
}

// UART 参数
#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024

void uart_task(void *pvParameters) {
    ESP_LOGI(LOG_TAG, "enter uart_task\n");
    
    // 创建一个缓冲区用于接收数据
    uint8_t data[UART_BUF_SIZE];

    // 配置 UART 参数
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // 初始化 UART 驱动
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    
    while (1) {
        // 从 UART 接收数据
        int length = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (length > 0) {
            // 确保字符串以'\0'结尾
            if (length < UART_BUF_SIZE) {
                data[length] = '\0'; // 将数据视作字符串
            } else {
                data[UART_BUF_SIZE - 1] = '\0';
            }
            
            printf("Received: %s\n", data);
            if (strcmp((const char*)data, "start") == 0) {
                peer_connection_datachannel_send(peer_connection, "start", 5);
                int totalSample = 8000 * 5 / 320;
                int16_t *buf = (int16_t *)malloc(320 * sizeof(int16_t));
                if (buf == NULL) {
                    ESP_LOGE(LOG_TAG, "Failed to allocate memory for I2S buffer");
                    continue;
                }

                for (int i = 0; i < totalSample; i++) {
                    size_t bytes_read = 0;
                    esp_err_t err = i2s_read(I2S_NUM_0, buf, 320 * sizeof(int16_t), &bytes_read, portMAX_DELAY);
                    if (err != ESP_OK) {
                        ESP_LOGE(LOG_TAG, "I2S read failed: %s", esp_err_to_name(err));
                        break;
                    }
                    ESP_LOGD(LOG_TAG, "READ BYTE: %d", bytes_read);
                    uint8_t pcmaBuf[bytes_read / sizeof(int16_t)];
                    lin2alaw(buf, pcmaBuf, 1, 1, bytes_read / sizeof(int16_t));
                    peer_connection_send_audio(peer_connection, pcmaBuf, bytes_read / sizeof(int16_t));
                }

                free(buf);
                peer_connection_datachannel_send(peer_connection, "end", 3);
                printf("end\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL));
    }
}


void oai_webrtc() {
  PeerConfiguration peer_connection_config = {
      .ice_servers = {},
      .audio_codec = CODEC_PCMA,
      .video_codec = CODEC_NONE,
      .datachannel = DATA_CHANNEL_STRING,
      .onaudiotrack = [](uint8_t *data, size_t size, void *userdata) -> void {
#ifndef LINUX_BUILD
        oai_audio_decode(data, size);
#endif
      },
      .onvideotrack = NULL,
      .on_request_keyframe = NULL,
      .user_data = NULL,
  };

  peer_connection = peer_connection_create(&peer_connection_config);
  if (peer_connection == NULL) {
    ESP_LOGE(LOG_TAG, "Failed to create peer connection");
#ifndef LINUX_BUILD
    esp_restart();
#endif
  }

  peer_connection_oniceconnectionstatechange(peer_connection,
                                             oai_onconnectionstatechange_task);
  peer_connection_onicecandidate(peer_connection, oai_on_icecandidate_task);
  peer_connection_ondatachannel(peer_connection,
                                oai_ondatachannel_onmessage_task,
                                oai_ondatachannel_onopen_task, NULL);
  // peer_signaling_connect("mqtts://s.sdad22624319.cn/public/spotted-happy-panda", "dGVzdDp0ZXN0", peer_connection);
  peer_connection_create_offer(peer_connection);

  // xTaskCreatePinnedToCore(peer_connection_task, "peer_connection", 8192, NULL, 5, &xPcTaskHandle, 1);
  xTaskCreatePinnedToCore(uart_task, "uart_task", 8192, NULL, 5, &xPcTaskHandle, 1);
  // uart_task();
  while (1) {
    peer_connection_loop(peer_connection);
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL));
  }
}

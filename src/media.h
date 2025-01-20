#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include <stddef.h>
#include <stdint.h>

// 初始化RingBuffer
void init_ringbuffer(void);

// 启动I2S任务
void start_i2s_task(void);

// 音频解码函数
void oai_audio_decode(uint8_t *data, size_t size);

#endif // AUDIO_PROCESSING_H
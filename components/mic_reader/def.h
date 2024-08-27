#ifndef _DEF_H_
#define _DEF_H_

#include "stddef.h"
#include "stdint.h"

#include "driver/gpio.h"

#if CONFIG_TARGET_GRC_DEVBOARD
#define MIC_WS_PIN   GPIO_NUM_38
#define MIC_DATA_PIN GPIO_NUM_45
#elif CONFIG_TARGET_AI_MODULE
#define MIC_WS_PIN   GPIO_NUM_48
#define MIC_DATA_PIN GPIO_NUM_47
#else
#error "unknown target"
#endif

typedef int16_t audio_t;
#define FRAME_LEN_MS 10
#define ELEM_BYTES   sizeof(audio_t)
#define FRAME_LEN    (CONFIG_SAMPLE_RATE / 1000 * FRAME_LEN_MS)
#define FRAME_SZ     (FRAME_LEN * ELEM_BYTES)

// inpm41: 24bits data, mult=3
#define HW_ELEM_BYTES 2
#define HW_FRAME_MULT 2
#define HW_FRAME_LEN  FRAME_LEN *HW_FRAME_MULT
#define HW_FRAME_SZ   HW_FRAME_LEN *HW_ELEM_BYTES

#define I2S_RX_PORT_NUMBER I2S_NUM_0
#define I2S_RX_BIT_WIDTH   I2S_DATA_BIT_WIDTH_16BIT
#define I2S_RX_SLOT_MODE   I2S_SLOT_MODE_MONO

#define I2S_RX_DMA_BUF_LEN HW_FRAME_LEN
#define I2S_RX_DMA_BUF_SZ  HW_FRAME_SZ
#define I2S_RX_DMA_BUF_NUM 2

typedef enum MicResult_e {
  MIC_OK,
  MIC_SILENT,
  MIC_NOISY,
  MIC_INIT_ERROR,
} MicResult_t;

#endif // _DEF_H_

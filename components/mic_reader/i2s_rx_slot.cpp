#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "string.h"

#include <driver/i2s_pdm.h>
#include <driver/i2s_types.h>

#include "def.h"
#include "i2s_rx_slot.h"

static const char *TAG = "i2s_rx_slot";

EventGroupHandle_t xMicEventGroup = NULL;
SemaphoreHandle_t xMicSema = NULL;
RingbufHandle_t xMicRingBuffer = NULL;

static i2s_chan_handle_t s_rx_handle = NULL;
static TaskHandle_t xRxTaskHandle = NULL;
static int g_skip_frames = 0;

static size_t s_rx_queue_ovf_count = 0;
static IRAM_ATTR bool i2s_rx_queue_overflow_callback(i2s_chan_handle_t handle,
                                                     i2s_event_data_t *event,
                                                     void *data) {
  s_rx_queue_ovf_count++;
  return false;
}

static IRAM_ATTR bool s_rx_on_recv_callback(i2s_chan_handle_t handle,
                                            i2s_event_data_t *event,
                                            void *user_ctx) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Set task notification for RX task to continue
  configASSERT(xRxTaskHandle != NULL);
  vTaskNotifyGiveFromISR(xRxTaskHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  return false;
}

static uint8_t rx_buffer[I2S_RX_DMA_BUF_SZ] = {0};

void i2s_receive_task(void *pvParameters) {
  for (;;) {
    size_t data_received = 0;

    int64_t t1 = esp_timer_get_time();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    const auto ret = i2s_channel_read(s_rx_handle, rx_buffer, I2S_RX_DMA_BUF_SZ,
                                      &data_received, 0);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "err=%d, read %d/%d bytes", ret, data_received,
               I2S_RX_DMA_BUF_SZ);
      continue;
    }

    ESP_LOGV(TAG, "data_received=%d, s_rx_queue_ovf_count=%d, elapsed=%lld",
             data_received, s_rx_queue_ovf_count, esp_timer_get_time() - t1);

    if (g_skip_frames > 0) {
      g_skip_frames--;
      ESP_LOGV(TAG, "skip frame=%d", g_skip_frames);
      continue;
    }
    const auto res = xRingbufferSend(xMicRingBuffer, rx_buffer, HW_FRAME_SZ, 0);
    ESP_LOGV(TAG, " xRingbufferSend res=%d", res);
  }
}

void i2s_rx_slot_init(const mic_conf_t &conf) {
  i2s_pdm_slot_mask_t i2s_slot_mask = I2S_PDM_SLOT_RIGHT;
  switch (conf.slot_type) {
  case stLeft:
    i2s_slot_mask = I2S_PDM_SLOT_LEFT;
    break;
  case stRight:
    i2s_slot_mask = I2S_PDM_SLOT_RIGHT;
    break;
  case stBoth:
    i2s_slot_mask = I2S_PDM_SLOT_BOTH;
    break;
  default:
    break;
  }

  i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = I2S_RX_DMA_BUF_NUM;
  chan_cfg.dma_frame_num = I2S_RX_DMA_BUF_LEN;
  chan_cfg.auto_clear = true;

  i2s_pdm_rx_clk_config_t clk_cfg = {
    .sample_rate_hz = conf.sample_rate,
    .clk_src = I2S_CLK_SRC_DEFAULT,
    .mclk_multiple = I2S_MCLK_MULTIPLE_128,
    .dn_sample_mode = I2S_PDM_DSR_16S,
  };

  i2s_pdm_rx_config_t rx_pdm_rx_cfg = {
    .clk_cfg = clk_cfg,
    .slot_cfg =
      I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_RX_BIT_WIDTH, I2S_RX_SLOT_MODE),
    .gpio_cfg =
      {
        .clk = MIC_WS_PIN,
        .din = MIC_DATA_PIN,
        .invert_flags =
          {
            .clk_inv = false,
          },
      },
  };

  rx_pdm_rx_cfg.slot_cfg.slot_mask = i2s_slot_mask;

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx_handle));
  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(s_rx_handle, &rx_pdm_rx_cfg));

  i2s_event_callbacks_t cbs = {
    .on_recv = s_rx_on_recv_callback,
    .on_recv_q_ovf = i2s_rx_queue_overflow_callback,
    .on_sent = NULL,
    .on_send_q_ovf = NULL,
  };
  ESP_ERROR_CHECK(i2s_channel_register_event_callback(s_rx_handle, &cbs, NULL));
}

void i2s_rx_slot_start() {
  xSemaphoreTake(xMicSema, portMAX_DELAY);
  const auto xNotifs = ulTaskNotifyValueClear(xRxTaskHandle, 0xffffffff);
  ESP_LOGD(TAG, "%s: xNotifs=%ld", __FUNCTION__, xNotifs);
  g_skip_frames = 30;
  const size_t delay_ticks = g_skip_frames * FRAME_LEN_MS;
  ESP_ERROR_CHECK(i2s_channel_enable(s_rx_handle));
  vTaskDelay(pdMS_TO_TICKS(delay_ticks));
  xEventGroupSetBits(xMicEventGroup, MIC_ON_MSK);
}

void i2s_rx_slot_stop() {
  ESP_LOGD(TAG, "%s", __FUNCTION__);
  ESP_ERROR_CHECK(i2s_channel_disable(s_rx_handle));
  xSemaphoreGive(xMicSema);
  xEventGroupClearBits(xMicEventGroup, MIC_ON_MSK);
}

void i2s_rx_slot_release() {
  if (s_rx_handle) {
    ESP_ERROR_CHECK(i2s_del_channel(s_rx_handle));
    s_rx_handle = NULL;
  }
}

int i2s_receiver_init() {
  if ((xMicRingBuffer != NULL) || (xMicSema != NULL) ||
      (xRxTaskHandle != NULL) || (xMicEventGroup != NULL)) {
    return -1;
  }

  xMicEventGroup = xEventGroupCreate();
  if (xMicEventGroup == NULL) {
    ESP_LOGE(TAG, "Error creating xMicEventGroup");
    return -1;
  }

  xMicRingBuffer = xRingbufferCreate(HW_FRAME_SZ, RINGBUF_TYPE_BYTEBUF);

  if (xMicRingBuffer == NULL) {
    ESP_LOGE(TAG, "Error creating mic ring buffer");
    return -1;
  }

  xMicSema = xSemaphoreCreateBinary();
  if (xMicSema) {
    xSemaphoreGive(xMicSema);
  } else {
    ESP_LOGE(TAG, "Error creating xMicSema");
    return -1;
  }

  auto xReturned =
    xTaskCreate(i2s_receive_task, "i2s_receive_task",
                configMINIMAL_STACK_SIZE + 1024, NULL, 3, &xRxTaskHandle);
  if (xReturned != pdPASS) {
    ESP_LOGE(TAG, "Error creating i2s_receive_task");
    return -1;
  }
  return 0;
}

int i2s_receiver_release() {
  if (xMicEventGroup) {
    vEventGroupDelete(xMicEventGroup);
    xMicEventGroup = NULL;
  }
  if (xMicRingBuffer) {
    vRingbufferDelete(xMicRingBuffer);
    xMicRingBuffer = NULL;
  }
  if (xMicSema) {
    vSemaphoreDelete(xMicSema);
    xMicSema = NULL;
  }
  if (xRxTaskHandle) {
    vTaskDelete(xRxTaskHandle);
    xRxTaskHandle = NULL;
  }
  return 0;
}

int i2s_rx_slot_read(void *buffer, size_t bytes, size_t timeout_ticks) {
  if (!buffer || !buffer || !xMicRingBuffer) {
    return -1;
  }
  size_t item_size = 0;
  uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(xMicRingBuffer, &item_size,
                                                    timeout_ticks, bytes);
  if (item != NULL && item_size == bytes) {
    ESP_LOGV(TAG, "recv bytes=%d", item_size);

    memcpy(buffer, item, bytes);

    vRingbufferReturnItem(xMicRingBuffer, (void *)item);
    return 0;
  } else {
    ESP_LOGE(TAG, "failed to receive item");
    return -1;
  }
};

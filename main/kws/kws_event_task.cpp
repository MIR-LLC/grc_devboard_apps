#include "Event.hpp"

#include "kws_event_task.h"
#include "kws_task.h"

#include "esp_log.h"

static const char *TAG = "KWS";

static TaskHandle_t xTaskHandle = NULL;
static nn_model_handle_t s_model_handle = NULL;

extern const unsigned char *kws_model_ptr;
extern const char *kws_labels[];
extern unsigned int kws_labels_num;

static void kws_event_task(void *pv) {
  nn_model_handle_t model_handle = static_cast<nn_model_handle_t>(pv);
  char result[32];
  for (;;) {
    size_t req_words = 0;
    xQueuePeek(xKWSRequestQueue, &req_words, portMAX_DELAY);
    if (xSemaphoreTake(xKWSSema, pdMS_TO_TICKS(50))) {
      int category;
      if (xQueueReceive(xKWSResultQueue, &category, pdMS_TO_TICKS(50)) ==
          pdPASS) {
        nn_model_get_label(model_handle, category, result, sizeof(result));
        if (strcmp(result, "robot") == 0) {
          sendEvent(eEvent::CMD_WAKEUP);
        } else if (strcmp(result, "stop") == 0) {
          sendEvent(eEvent::CMD_STOP);
        } else {
          sendEvent(eEvent::WORD_UNKNOWN);
        }
      }
      xSemaphoreGive(xKWSSema);
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

int initKWS() {
  if (nn_model_init(&s_model_handle,
                    nn_model_config_t{
                      .model_ptr = kws_model_ptr,
                      .labels = kws_labels,
                      .labels_num = kws_labels_num,
                      .is_quantized = false,
                      .inference_threshold = KWS_INFERENCE_THRESHOLD,
                    }) < 0) {
    ESP_LOGE(TAG, "KWS model init error");
    return -1;
  }
  kws_task_init(kws_task_conf_t{
    .model_handle = s_model_handle,
    .mic_gain = 30,
    .ns_level = 1,
  });

  auto xReturned = xTaskCreate(kws_event_task, "kws_event_task",
                               configMINIMAL_STACK_SIZE + 1024, s_model_handle,
                               tskIDLE_PRIORITY, &xTaskHandle);
  if (xReturned != pdPASS) {
    ESP_LOGE(TAG, "Error creating KWS event task");
    return -1;
  }

  return 0;
}

void releaseKWS() {
  if (xTaskHandle) {
    vTaskDelete(xTaskHandle);
    xTaskHandle = NULL;
  }
  kws_task_release();
  if (s_model_handle) {
    nn_model_release(s_model_handle);
    s_model_handle = NULL;
  }
}

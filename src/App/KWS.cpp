#include "KWS.hpp"
#include "Event.hpp"
#include "Status.hpp"

#include "kws_preprocessor.h"
#include "mic_reader.h"

#include "esp_log.h"

static const char *TAG = "KWS";

extern const unsigned char *kws_model_ptr;
extern const char *kws_labels[];
extern unsigned int kws_labels_num;

static void kws_event_task(void *pvParameters) {
  char result[32];
  for (;;) {
    size_t req_words = 0;
    xQueuePeek(xKWSRequestQueue, &req_words, portMAX_DELAY);
    if (xSemaphoreTake(xKWSSema, pdMS_TO_TICKS(50))) {
      int category;
      if (xQueueReceive(xKWSResultQueue, &category, pdMS_TO_TICKS(50)) ==
          pdPASS) {
        kws_get_category(category, result, sizeof(result));
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

bool initKWS() {
  int errors = !kws_model_init_op_resolver();
  errors += !kws_model_init(kws_model_ptr, kws_labels, kws_labels_num);
  if (errors) {
    ESP_LOGE(TAG, "KWS model init error");
    return false;
  }
  kws_preprocessor_init();
  const auto result = mic_reader_init();
  switch (result) {
  case MIC_SILENT:
    ESP_LOGE(TAG, "Mic is silent");
    return false;
    break;
  case MIC_INIT_ERROR:
    ESP_LOGE(TAG, "Mic init error");
    return false;
    break;
  case MIC_NOISY:
    ESP_LOGE(TAG, "Mic is noisy");
    return false;
    break;
  default:
    break;
  }

  auto xReturned = xTaskCreate(kws_event_task, "kws_event_task",
                               configMINIMAL_STACK_SIZE + 1024 * 4, NULL,
                               tskIDLE_PRIORITY, NULL);
  if (xReturned != pdPASS) {
    ESP_LOGE(TAG, "Error creating KWS event task");
    return false;
  }

  xEventGroupSetBits(xStatusEventGroup, STATUS_MIC_INIT_MSK);
  return true;
}
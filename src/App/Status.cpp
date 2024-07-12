#include "Status.hpp"
#include "ILed.hpp"

#include "esp_log.h"

static constexpr char TAG[] = "Status";

EventGroupHandle_t xStatusEventGroup;

static void status_monitor_task(void *pvParameters) {
  auto *p_led = static_cast<ILed *>(pvParameters);
  const TickType_t xTicks = 50 / portTICK_PERIOD_MS;
  TickType_t xEnterTime[GET_BIT_POS(STATUS_ALL_BITS_MSK) + 1] = {0};

  auto blinker = [p_led, xTicks, &xEnterTime](
                   EventBits_t mask, TickType_t timeout, ILed::Colour colour,
                   unsigned hold_time, unsigned led_num = -1,
                   ILed::Brightness b = ILed::Brightness::_50) mutable {
    auto &time = xEnterTime[GET_BIT_POS(mask)];
    time += xTicks;
    if (time > timeout) {
      p_led->set(colour, led_num, b);
      vTaskDelay(pdMS_TO_TICKS(hold_time));
      time = 0;
    } else {
      p_led->set(ILed::Black, led_num);
    }
  };

  for (;;) {
    auto xBits = xEventGroupWaitBits(xStatusEventGroup, STATUS_EVENT_BITS_MSK,
                                     pdTRUE, pdFALSE, xTicks);
    if (xBits & STATUS_EVENT_BITS_MSK) {
      if (xBits & STATUS_EVENT_GOOD_MSK) {
        p_led->set(ILed::Green);
        vTaskDelay(pdMS_TO_TICKS(2000));
      } else if (xBits & STATUS_EVENT_BAD_MSK) {
        p_led->set(ILed::Red);
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      p_led->set(ILed::Black);
    } else {
      xBits = xEventGroupGetBits(xStatusEventGroup);

      if (xBits & STATUS_UNLOCKED_MSK) {
        p_led->set(ILed::Green);
      } else if (xBits & STATUS_STATE_BAD_MSK) {
        blinker(STATUS_STATE_BAD_MSK, 800, ILed::Red, 200, -1);
      } else if (xBits & STATUS_SYSTEM_SUSPENDED_MSK) {
        blinker(STATUS_SYSTEM_SUSPENDED_MSK, 5000, ILed::Cyan, 200, -1,
                ILed::Brightness::_25);
      } else if (xBits & STATUS_MIC_ON_MSK) {
        p_led->set(ILed::Blue);
      } else {
        p_led->set(ILed::Black);
      }
    }
  }
}

void initStatusMonitor(ILed *p_led) {
  xStatusEventGroup = xEventGroupCreate();
  if (xStatusEventGroup == NULL) {
    ESP_LOGE(TAG, "Error creating xStatusEventGroup");
  }

  TaskHandle_t xHandle = NULL;
  auto xReturned =
    xTaskCreate(status_monitor_task, "status_monitor_task",
                configMINIMAL_STACK_SIZE + 512, p_led, 2, &xHandle);
  if (xReturned != pdPASS) {
    ESP_LOGE(TAG, "Error creating task");
    vTaskDelete(xHandle);
  }
}
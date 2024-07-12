#include "Event.hpp"
#include "Button.hpp"

#include "esp_log.h"

enum eButtonState { DOWN, UP };

#define BUTTON_HOLD_DURATION_MS 5000

TimerHandle_t xTimer;
QueueHandle_t xEventQueue;

static constexpr char TAG[] = "Event";

size_t getTimeMS() { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

void sendEvent(eEvent ev) {
  if (xQueueSend(xEventQueue, &ev, 0) == pdPASS) {
    ESP_LOGD(TAG, "send ev=%s", event_to_str(ev));
  } else {
    ESP_LOGD(TAG, "skip ev=%s", event_to_str(ev));
  }
}

static void button_event_task(void *pvParameters) {
  Button int_button(INT_BUTTON_PIN);
  eButtonState bstate = eButtonState::UP;
  unsigned start_time = 0;
  bool pending_hold = false;

  auto get_bstate = [&int_button]() {
    const bool s_b1 = int_button.isPressed();
    ESP_LOGD(TAG, "buttons state: %d", s_b1);
    return !s_b1;
  };

  for (;;) {
    if (bstate == eButtonState::UP) {
      if (get_bstate()) {
        bstate = eButtonState::DOWN;
        start_time = getTimeMS();
      }
    } else if (bstate == eButtonState::DOWN) {
      if (!pending_hold) {
        const auto cur_time = getTimeMS();
        if (cur_time > start_time + BUTTON_HOLD_DURATION_MS) {
          pending_hold = true;
          // sendEvent(eEvent::BUTTON_HOLD);
        } else if (!get_bstate()) {
          bstate = eButtonState::UP;
          sendEvent(eEvent::BUTTON_CLICK);
        }
      } else if (!get_bstate()) {
        pending_hold = false;
        bstate = eButtonState::UP;
      }
    }
  }
}

static void vTimerCallback(TimerHandle_t xTimer) {
  ESP_LOGD(TAG, "send timeout event");
  sendEvent(eEvent::TIMEOUT);
}

void initEventsGenerator() {
  xEventQueue = xQueueCreate(5, sizeof(eEvent));
  if (xEventQueue == NULL) {
    ESP_LOGE(TAG, "Error creating event queue");
  }
  xTimer =
    xTimerCreate("Timer", pdMS_TO_TICKS(1000), pdFALSE, NULL, vTimerCallback);
  if (xTimer == NULL) {
    ESP_LOGE(TAG, "Error creating xTimer");
  } else {
    xTimerStop(xTimer, 0);
  }

  TaskHandle_t xHandle = NULL;
  auto xReturned = xTaskCreate(button_event_task, "button_event_task",
                               configMINIMAL_STACK_SIZE + 1024, NULL,
                               tskIDLE_PRIORITY, &xHandle);
  if (xReturned != pdPASS) {
    ESP_LOGE(TAG, "Error creating button event task");
    vTaskDelete(xHandle);
  }
}

const char *event_to_str(eEvent ev) {
  switch (ev) {
  case eEvent::NO_EVENT:
    return "NO_EVENT";
    break;
  case eEvent::BUTTON_CLICK:
    return "BUTTON_CLICK";
    break;
  case eEvent::CMD_WAKEUP:
    return "CMD_WAKEUP";
    break;
  case eEvent::CMD_STOP:
    return "WORD_STOP";
    break;
  case eEvent::WORD_UNKNOWN:
    return "WORD_UNKNOWN";
    break;
  case eEvent::TIMEOUT:
    return "TIMEOUT";
    break;
  default:
    return "";
  }
}
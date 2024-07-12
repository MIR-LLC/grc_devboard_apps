#include "App.hpp"
#include "Button.hpp"
#include "DisplaySTDOUT.hpp"
#include "KWS.hpp"
#include "Lcd.hpp"
#include "Led.hpp"
#include "State.hpp"

#include "git_version.h"
#include "kws_preprocessor.h"

#include "esp_log.h"

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

#define TITLE      "VoiceRelay"
#define HEADER_STR TITLE " " TOSTRING(MAJOR_VERSION) "." TOSTRING(MINOR_VERSION)

static constexpr char TAG[] = "App";

App::App() : current_state_(nullptr) {
  gpio_reset_pin(LOCK_PIN);
  gpio_set_direction(LOCK_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LOCK_PIN, 0);

  transition_queue_ = xQueueCreate(1, sizeof(State *));
  if (transition_queue_ == NULL) {
    ESP_LOGE(TAG, "Error creating transition queue");
  }
  p_led = std::make_unique<Led>();
  initStatusMonitor(p_led.get());

  initEventsGenerator();

#if TARGET_GRC_DEVBOARD
  p_display = std::make_unique<Lcd>(Lcd::Rotation::UPSIDE_DOWN);
#else
  p_display = std::make_unique<DisplaySTDOUT>();
#endif

  p_display->print_header("%s", HEADER_STR);
  p_display->send();

  const bool kws_init_res = initKWS();

  if (!kws_init_res) {
    xEventGroupSetBits(xStatusEventGroup, STATUS_STATE_BAD_MSK);
  }

  xEventGroupWaitBits(xStatusEventGroup, STATUS_INIT_BITS_MSK, pdFALSE, pdTRUE,
                      portMAX_DELAY);
  transition(new Suspended(new Main));
}

void App::transition(State *target_state) {
  xQueueSend(transition_queue_, &target_state, portMAX_DELAY);
}

void App::do_transition(State *target_state) {
  if (current_state_ != nullptr) {
    current_state_->exitAction(this);
    delete current_state_;
    current_state_ = nullptr;
  }
  if (target_state != nullptr) {
    target_state->enterAction(this);
  }
  current_state_ = target_state;
  xQueueReset(xEventQueue);
}

void App::run() {
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  for (;;) {
    State *target_state = nullptr;
    if (xQueueReceive(transition_queue_, &target_state, 0) == pdPASS) {
      do_transition(target_state);
    }
    if (current_state_ == nullptr) {
      return;
    }
    eEvent ev;
    if (xQueueReceive(xEventQueue, &ev, xTicksToWait) == pdPASS) {
      current_state_->handleEvent(this, ev);
    } else {
      current_state_->update(this);
    }
  }
}

State *Suspended::clone() { return new Suspended(*this); }
void Suspended::enterAction(App *app) {
  ESP_LOGI(TAG, "Entering suspended state");
  xEventGroupSetBits(xStatusEventGroup, STATUS_SYSTEM_SUSPENDED_MSK);
  kws_req_word(1);
  app->p_display->print_header("%s", HEADER_STR);
  app->p_display->print_string("OFF");
  app->p_display->print_status("robot --> ON");
  app->p_display->send();
}
void Suspended::exitAction(App *app) {
  xEventGroupClearBits(xStatusEventGroup, STATUS_SYSTEM_SUSPENDED_MSK);
  ESP_LOGI(TAG, "Exiting suspended state");
  app->p_display->clear();
  app->p_display->send();
}
void Suspended::handleEvent(App *app, eEvent ev) {
  switch (ev) {
  case eEvent::BUTTON_CLICK:
    kws_req_cancel();
    app->transition(back_state_);
    break;
  case eEvent::CMD_WAKEUP:
    app->transition(back_state_);
    break;
  default:
    if (uxQueueMessagesWaiting(xKWSRequestQueue) == 0) {
      kws_req_word(1);
    }
    break;
  }
}

State *Main::clone() { return new Main(*this); }
void Main::enterAction(App *app) {
  ESP_LOGI(TAG, "Entering main state");
  kws_req_word(1);
  gpio_set_level(LOCK_PIN, 1);
  xEventGroupSetBits(xStatusEventGroup, STATUS_UNLOCKED_MSK);
  xTimerChangePeriod(xTimer, SUSPEND_AFTER_TICKS, 0);
  timer_val_s_ = float(SUSPEND_AFTER_TICKS * portTICK_PERIOD_MS) / 1000;
  ESP_LOGD(TAG, "timer_val_s_=%d", timer_val_s_);
}
void Main::exitAction(App *app) {
  ESP_LOGI(TAG, "Exiting main state");
  xEventGroupClearBits(xStatusEventGroup, STATUS_UNLOCKED_MSK);
  gpio_set_level(LOCK_PIN, 0);
  app->p_display->clear();
  app->p_display->send();
}
void Main::handleEvent(App *app, eEvent ev) {
  switch (ev) {
  case eEvent::CMD_STOP:
    app->transition(new Suspended(clone()));
    break;
  case eEvent::TIMEOUT:
    app->transition(new Suspended(clone()));
    break;
  default:
    if (uxQueueMessagesWaiting(xKWSRequestQueue) == 0) {
      kws_req_word(1);
    }
    break;
  }
}
void Main::update(App *app) {
  const TickType_t xRemainingTime =
    xTimerGetExpiryTime(xTimer) - xTaskGetTickCount();
  const size_t rem_time_s = float(xRemainingTime * portTICK_PERIOD_MS) / 1000;
  if (timer_val_s_ != rem_time_s) {
    timer_val_s_ = rem_time_s;
    ESP_LOGD(TAG, "timer_val_s_=%d", timer_val_s_);
    app->p_display->print_header("%s", HEADER_STR);
    app->p_display->print_string("ON: %d", timer_val_s_ + 1);
    app->p_display->print_status("stop --> OFF");
    app->p_display->send();
  }
}

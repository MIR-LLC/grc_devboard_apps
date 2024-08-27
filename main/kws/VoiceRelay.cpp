#include "App.hpp"
#include "git_version.h"
#include "kws_event_task.h"
#include "kws_task.h"

#define TITLE      "VoiceRelay"
#define HEADER_STR TITLE " " TOSTRING(MAJOR_VERSION) "." TOSTRING(MINOR_VERSION)

static constexpr char TAG[] = TITLE;

namespace VoiceRelay {
struct Suspended : State {
  Suspended(State *back_state) : back_state_(back_state) {}
  State *clone() { return new Suspended(*this); }
  ~Suspended() {
    if (back_state_) {
      delete back_state_;
    }
  }

  void enterAction(App *app) {
    ESP_LOGI(TAG, "Entering suspended state");
    xEventGroupSetBits(xStatusEventGroup, STATUS_SYSTEM_SUSPENDED_MSK);
    kws_req_word(1);
    app->p_display->print_header("%s", HEADER_STR);
    app->p_display->print_string("OFF");
    app->p_display->print_status("robot --> ON");
    app->p_display->send();
  }
  void exitAction(App *app) {
    xEventGroupClearBits(xStatusEventGroup, STATUS_SYSTEM_SUSPENDED_MSK);
    ESP_LOGI(TAG, "Exiting suspended state");
    app->p_display->clear();
    app->p_display->send();
  }
  void handleEvent(App *app, eEvent ev) {
    switch (ev) {
    case eEvent::BUTTON_CLICK:
      app->transition(back_state_);
      back_state_ = nullptr;
      kws_req_cancel();
      break;
    case eEvent::CMD_WAKEUP:
      app->transition(back_state_);
      back_state_ = nullptr;
      break;
    default:
      if (uxQueueMessagesWaiting(xKWSRequestQueue) == 0) {
        kws_req_word(1);
      }
      break;
    }
  }

protected:
  State *back_state_;
};

struct Main : State {
  Main() : timer_val_s_(0) {}

  State *clone() { return new Main(*this); }
  void enterAction(App *app) {
    ESP_LOGI(TAG, "Entering main state");
    kws_req_word(1);
    gpio_set_level(LOCK_PIN, 1);
    gpio_set_level(LOCK_PIN_INV, 0);
    xEventGroupSetBits(xStatusEventGroup, STATUS_UNLOCKED_MSK);
    xTimerChangePeriod(xTimer, SUSPEND_AFTER_TICKS, 0);
    timer_val_s_ = float(SUSPEND_AFTER_TICKS * portTICK_PERIOD_MS) / 1000;
    ESP_LOGD(TAG, "timer_val_s_=%d", timer_val_s_);
  }
  void exitAction(App *app) {
    ESP_LOGI(TAG, "Exiting main state");
    xEventGroupClearBits(xStatusEventGroup, STATUS_UNLOCKED_MSK);
    gpio_set_level(LOCK_PIN, 0);
    gpio_set_level(LOCK_PIN_INV, 1);
    app->p_display->clear();
    app->p_display->send();
  }
  void handleEvent(App *app, eEvent ev) {
    switch (ev) {
    case eEvent::CMD_STOP:
      app->transition(new Suspended(clone()));
      break;
    case eEvent::TIMEOUT:
      kws_req_cancel();
      app->transition(new Suspended(clone()));
      break;
    default:
      if (uxQueueMessagesWaiting(xKWSRequestQueue) == 0) {
        kws_req_word(1);
      }
      break;
    }
  }
  void update(App *app) {
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

private:
  size_t timer_val_s_;
};
} // namespace VoiceRelay

void releaseScenario(App *app) {
  ESP_LOGI(TAG, "Exiting VoiceRelay scenairo");
  releaseKWS();
}

void initScenario(App *app) {
  if (initKWS() < 0) {
    ESP_LOGE(TAG, "Unable to init KWS");
    app->transition(nullptr);
  } else {
    ESP_LOGI(TAG, "Entering VoiceRelay scenairo");
    app->transition(new VoiceRelay::Suspended(new VoiceRelay::Main));
  }
}

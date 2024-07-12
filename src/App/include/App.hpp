#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"

#include <array>
#include <memory>

#include "Event.hpp"
#include "IDisplay.hpp"
#include "State.hpp"
#include "Status.hpp"

#include "driver/gpio.h"
#if TARGET_GRC_DEVBOARD
#define LOCK_PIN GPIO_NUM_10
#elif TARGET_AI_MODULE
#define LOCK_PIN GPIO_NUM_39
#else
#error "unknown target"
#endif

#if SUSPEND_TIMEOUT_S > 0
#define SUSPEND_AFTER_TICKS pdMS_TO_TICKS(1000 * SUSPEND_TIMEOUT_S)
#else
#define SUSPEND_AFTER_TICKS portMAX_DELAY
#endif

/*!
 * \brief Simple fsm.
 */
class App {
public:
  /*!
   * \brief Constructor.
   */
  App();
  /*!
   * \brief Start processing events.
   */
  void run();
  /*!
   * \brief State transition.
   * \param target_state Pointer to next state.
   */
  void transition(State *target_state);

  /*! \brief Pointer to led. */
  std::unique_ptr<ILed> p_led;
  /*! \brief Pointer to display. */
  std::unique_ptr<IDisplay> p_display;

private:
  /*!
   * \brief State transition impl.
   * \param target_state Pointer to next state.
   */
  void do_transition(State *target_state);

private:
  /*! \brief Queue of transitions. */
  QueueHandle_t transition_queue_;
  /*! \brief Current state of the application. */
  State *current_state_;
};

/*!
 * \brief Enter this state after no user actions.
 */
struct Suspended : State {
  Suspended(State *back_state) : back_state_(back_state) {}
  State *clone() override final;

  void enterAction(App *app) override final;
  void exitAction(App *app) override final;
  void handleEvent(App *app, eEvent ev) override final;

protected:
  State *back_state_;
};

/*!
 * \brief Main state.
 */
struct Main : State {
  Main() : timer_val_s_(0) {}
  State *clone() override final;

  void enterAction(App *app) override final;
  void exitAction(App *app) override final;
  void handleEvent(App *app, eEvent ev) override final;
  void update(App *app) override final;

private:
  size_t timer_val_s_;
};

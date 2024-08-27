#pragma once
#include "IButton.hpp"

#include "driver/gpio.h"

#if CONFIG_TARGET_GRC_DEVBOARD
#define INT_BUTTON_PIN GPIO_NUM_0
#define SW3_BUTTON_PIN GPIO_NUM_1
#define SW4_BUTTON_PIN GPIO_NUM_2
#elif CONFIG_TARGET_AI_MODULE
#define INT_BUTTON_PIN GPIO_NUM_0
#else
#error "unknown target"
#endif

class Button : public IButton {
public:
  Button(int pin);
  bool isPressed() const override final;
};

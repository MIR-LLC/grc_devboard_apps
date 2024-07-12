#pragma once
#include "IButton.hpp"

#include "driver/gpio.h"

#define INT_BUTTON_PIN GPIO_NUM_0

class Button : public IButton {
public:
  Button(int pin);
  bool isPressed() const override final;
};

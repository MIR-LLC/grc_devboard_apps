#include "I2C.hpp"
#include "U8g2lib.h"

#include "Lcd.hpp"

#include <algorithm>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#ifndef _countof
#define _countof(arr) (sizeof(arr) / sizeof(arr[0]))
#endif

static constexpr char TAG[] = "Lcd";

#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64
#define FONT_WIDTH     7
#define FONT_HEIGHT    10
#define FONT           Lcd::Font::COURR10

#define CHARS_X int(DISPLAY_WIDTH / FONT_WIDTH)
#define LINES_Y int(DISPLAY_HEIGHT / FONT_HEIGHT)

#define HEADER_LINE_COEF 1.0
#define STRING_LINE_COEF 1.2
static char HEADER_LINE[DISPLAY_WIDTH / FONT_WIDTH + 1] = {0};

extern "C" uint8_t u8x8_byte_hw_i2c_cb(U8X8_UNUSED u8x8_t *u8x8,
                                       U8X8_UNUSED uint8_t msg,
                                       U8X8_UNUSED uint8_t arg_int,
                                       U8X8_UNUSED void *arg_ptr) {
  static uint8_t s_buf[100];
  static unsigned s_cnt = 0;

  switch (msg) {
  case U8X8_MSG_BYTE_INIT:
    if (u8x8->bus_clock == 0)
      u8x8->bus_clock = u8x8->display_info->i2c_bus_clock_100kHz * 100000UL;
    break;

  case U8X8_MSG_BYTE_SEND: {
    unsigned rest =
      std::min((unsigned)arg_int, sizeof(s_buf) / sizeof(s_buf[0]) - s_cnt);
    uint8_t *ptr = (uint8_t *)arg_ptr;
    while (rest--)
      s_buf[s_cnt++] = *ptr++;
  } break;

  case U8X8_MSG_BYTE_START_TRANSFER:
    break;
  case U8X8_MSG_BYTE_END_TRANSFER: {
    bool ret = g_i2c.write(u8x8_GetI2CAddress(u8x8) >> 1, s_buf, s_cnt);
    (void)ret;
    s_cnt = 0;
  } break;
  default:
    return 0;
  }
  return 1;
}

extern "C" uint8_t u8x8_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg,
                                          uint8_t arg_int,
                                          U8X8_UNUSED void *arg_ptr) {
  return 1;
}

class U8G2_SSD1306_128X64_CUSTOM : public U8G2 {
public:
  U8G2_SSD1306_128X64_CUSTOM(const u8g2_cb_t *rotation,
                             uint8_t reset = U8X8_PIN_NONE,
                             uint8_t clock = U8X8_PIN_NONE,
                             uint8_t data = U8X8_PIN_NONE)
    : U8G2() {
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, rotation, u8x8_byte_hw_i2c_cb,
                                           u8x8_gpio_and_delay_cb);
    u8x8_SetPin_HW_I2C(getU8x8(), reset, clock, data);
  }
};

static U8G2_SSD1306_128X64_CUSTOM u8g2(U8G2_R0, U8X8_PIN_NONE);

Lcd::Lcd(Rotation rot) : IDisplay(), y_offset_(FONT_HEIGHT) {
  if (!g_i2c.open()) {
    ESP_LOGE(TAG, "Failed to open I2C");
  }
  u8g2.begin();
  setRotation(rot);
  setFont(FONT);
  memset(HEADER_LINE, '-', _countof(HEADER_LINE) - 1);
}

void Lcd::setRotation(Rotation rot) const {
  switch (rot) {
  default:
  case Rotation::PORTRAIT:
    u8g2.setDisplayRotation(U8G2_R0);
    break;
  case Rotation::UPSIDE_DOWN:
    u8g2.setDisplayRotation(U8G2_R2);
    break;
  }
}

void Lcd::setFont(Font f) const {
  switch (f) {
  default:
  case Font::COURR08:
    u8g2.setFont(u8g_font_courR08);
    break;
  case Font::COURR10:
    u8g2.setFont(u8g_font_courR10);
    break;
  case Font::COURB10:
    u8g2.setFont(u8g_font_courB10);
    break;
  case Font::COURR14:
    u8g2.setFont(u8g_font_courR14);
    break;
  case Font::F8X13B:
    u8g2.setFont(u8g_font_8x13B);
    break;
  case Font::F9X15:
    u8g2.setFont(u8g_font_9x15);
    break;
  case Font::F9X18:
    u8g2.setFont(u8g_font_9x18);
    break;
  }
}

void Lcd::draw_string(const char *fmt, va_list argp) {
  const unsigned buf_size = 32;
  char buf[buf_size];
  vsnprintf(buf, buf_size, fmt, argp);
  u8g2.drawStr(0, y_offset_, buf);
}

void Lcd::print_header(const char *fmt, ...) {
  va_list argp;
  va_start(argp, fmt);
  draw_string(fmt, argp);
  va_end(argp);

  y_offset_ += FONT_HEIGHT * HEADER_LINE_COEF;
  u8g2.drawStr(0, y_offset_, HEADER_LINE);
  y_offset_ += FONT_HEIGHT * HEADER_LINE_COEF;
}

void Lcd::print_string(const char *fmt, ...) {
  setFont(Lcd::Font::COURB10);
  va_list argp;
  va_start(argp, fmt);
  draw_string(fmt, argp);
  va_end(argp);
  y_offset_ += FONT_HEIGHT * STRING_LINE_COEF;
  setFont(Lcd::Font::COURR10);
}

void Lcd::print_status(const char *fmt, ...) {
  setFont(Lcd::Font::COURR08);
  const size_t saved_offset = y_offset_;
  y_offset_ = DISPLAY_HEIGHT - 8;
  va_list argp;
  va_start(argp, fmt);
  draw_string(fmt, argp);
  va_end(argp);
  setFont(Lcd::Font::COURR10);
  y_offset_ = saved_offset;
}

void Lcd::send() {
  u8g2.sendBuffer();
  u8g2.clearBuffer();
  y_offset_ = FONT_HEIGHT * STRING_LINE_COEF;
}

void Lcd::clear() {
  u8g2.clearBuffer();
  y_offset_ = FONT_HEIGHT * STRING_LINE_COEF;
}

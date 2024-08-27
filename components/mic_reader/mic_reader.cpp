#include "freertos/FreeRTOS.h"

#include <algorithm>
#include <cmath>

#include "esp_log.h"

#include "i2s_rx_slot.h"
#include "mic_proc.h"
#include "mic_reader.h"

static const char *TAG = "mic_reader";

#define FILTER_INIT_FRAME_NUM 30
#define MIC_TEST_FRAME_NUM    10

static constexpr size_t DEF_MEAN = 1 << (8 * HW_ELEM_BYTES - 6);
static constexpr size_t DEF_STD_DEV =
  1 << (8 * HW_ELEM_BYTES - 3); // 1/4 of dynamic range

static dc_blocker<int32_t> s_filter;

int mic_reader_read_frame(audio_t *dst) {
  if (i2s_rx_slot_read(dst, FRAME_SZ, portMAX_DELAY) < 0) {
    return -1;
  }
  for (size_t j = 0; j < FRAME_LEN; j++) {
    dst[j] = s_filter.proc_val(dst[j]);
  }
  return 0;
};

size_t compute_max_abs(audio_t *data, size_t len) {
  size_t max_abs = 0;
  for (size_t j = 0; j < len; j++) {
    const size_t abs_val = abs(data[j]);
    if (abs_val > max_abs) {
      max_abs = abs_val;
    }
  }
  return max_abs;
}

static float compute_mean(const audio_t *data, size_t samples) {
  int32_t sum = 0;
  for (size_t i = 0; i < samples; i++) {
    sum += data[i];
  }
  return float(sum) / samples;
}

static float compute_std_dev(const audio_t *data, size_t samples,
                             float mean_val) {
  float sum_sq = 0;
  for (size_t i = 0; i < samples; i++) {
    sum_sq += std::pow(data[i] - mean_val, 2);
  }
  return std::sqrt(sum_sq / float(samples));
}

static MicResult_t test_microphone() {
  float mean = 0.0;
  float std_dev = 0.0;
  audio_t buffer[FRAME_LEN] = {0};

  for (size_t i = 0; i < MIC_TEST_FRAME_NUM; i++) {
    memset(buffer, 0, FRAME_SZ);
    mic_reader_read_frame(buffer);
    const float frame_mean = compute_mean(buffer, FRAME_LEN);
    const float frame_std_dev = compute_std_dev(buffer, FRAME_LEN, frame_mean);
    ESP_LOGV(TAG, "frame_mean=%f, frame_std_dev=%f", mean, std_dev);
    mean += frame_mean / MIC_TEST_FRAME_NUM;
    std_dev += frame_std_dev / MIC_TEST_FRAME_NUM;
  }

  ESP_LOGD(TAG, "mean=%f, std_dev=%f", mean, std_dev);
  if (mean == 0.f && std_dev == 0.f) {
    return MIC_SILENT;
  } else if (abs(mean) > DEF_MEAN || std_dev > DEF_STD_DEV) {
    return MIC_NOISY;
  } else {
    return MIC_OK;
  }
}

MicResult_t mic_reader_init() {
  if (i2s_receiver_init() < 0) {
    ESP_LOGE(TAG, "Unable to init mic reciever");
    return MIC_INIT_ERROR;
  }
  mic_conf_t mic_conf[2] = {
    {
      .sample_rate = CONFIG_SAMPLE_RATE,
      .slot_type = stRight,
    },
    {
      .sample_rate = CONFIG_SAMPLE_RATE,
      .slot_type = stLeft,
    },
  };

  audio_t buffer[FRAME_LEN] = {0};
  ESP_LOGD(TAG, "DEF_MEAN=%d, DEF_STD_DEV=%d", DEF_MEAN, DEF_STD_DEV);
  MicResult_t test_result;
  for (size_t i = 0; i < sizeof(mic_conf) / sizeof(mic_conf[0]); ++i) {
    ESP_LOGD(TAG, "Try conf[%d]", i);
    i2s_rx_slot_init(mic_conf[i]);
    i2s_rx_slot_start();

    // read first bad samples, init filter
    for (size_t i = 0; i < FILTER_INIT_FRAME_NUM; i++) {
      mic_reader_read_frame(buffer);
    }

    test_result = test_microphone();
    if (test_result == MIC_OK) {
      i2s_rx_slot_stop();
      break;
    } else {
      i2s_rx_slot_stop();
      i2s_rx_slot_release();
    }
  }
  if (test_result != MIC_OK) {
    return test_result;
  }
  return MIC_OK;
}

void mic_reader_release() {
  i2s_receiver_release();
  i2s_rx_slot_release();
}
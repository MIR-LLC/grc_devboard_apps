#include <algorithm>
#include <cmath>

#include "esp_agc.h"
#include "esp_log.h"
#include "esp_ns.h"
#include "esp_vad.h"

#include "i2s_rx_slot.h"
#include "kws_preprocessor.h"
#include "mic_proc.h"
#include "mic_reader.h"

static const char *TAG = "mic_reader";

#define FILTER_INIT_FRAME_NUM 30
#define MIC_TEST_FRAME_NUM    10

static constexpr size_t DEF_MEAN = 1 << (8 * HW_ELEM_BYTES - 6);
static constexpr size_t DEF_STD_DEV =
  1 << (8 * HW_ELEM_BYTES - 3); // 1/4 of dynamic range

static dc_blocker<int32_t> s_filter;

static ns_handle_t s_ns_handle = NULL;
static vad_handle_t s_vad_handle = NULL;
static void *s_agc_handle = NULL;

static size_t max_abs_arr[DET_VOICED_FRAMES_WINDOW] = {0};
static uint8_t is_speech_arr[DET_VOICED_FRAMES_WINDOW] = {0};
static uint8_t current_frames[DET_VOICED_FRAMES_WINDOW * FRAME_SZ] = {0};

#define PROC_FRAME_SZ FRAME_LEN *HW_ELEM_BYTES
static audio_t buffer[FRAME_LEN];

static int32_t convert_hw_bytes(uint8_t *buf, size_t bytes_width) {
  int32_t val = (buf[bytes_width - 1] & 0x80) ? -1 : 0;
  for (int b = bytes_width - 1; b >= 0; b--)
    val = (val << 8) | (buf[b] & 0xFF);
  return val;
}

static int read_frame(audio_t *dst) {
  size_t item_size = 0;
  uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(
    xMicRingBuffer, &item_size, portMAX_DELAY, PROC_FRAME_SZ);
  if (item != NULL && item_size == PROC_FRAME_SZ) {
    ESP_LOGV(TAG, "recv bytes=%d", item_size);

    for (size_t j = 0; j < FRAME_LEN; j++) {
      int32_t val = convert_hw_bytes(&item[j * HW_ELEM_BYTES], HW_ELEM_BYTES);
      dst[j] = s_filter.proc_val(val);
      // dst[j] = audio_t(val);
    }
    esp_agc_process(s_agc_handle, dst, dst, FRAME_LEN, KWS_SAMPLE_RATE);

    vRingbufferReturnItem(xMicRingBuffer, (void *)item);
    return 0;
  } else {
    ESP_LOGE(TAG, "failed to receive item");
    return -1;
  }
};

static size_t compute_max_abs(audio_t *data, size_t len) {
  size_t max_abs = 0;
  for (size_t j = 0; j < len; j++) {
    const size_t abs_val = abs(data[j]);
    if (abs_val > max_abs) {
      max_abs = abs_val;
    }
  }
  return max_abs;
}

static void mic_proc_task(void *pv) {
  size_t cur_frame = 0;
  size_t num_voiced = 0;
  uint8_t trig = 0;
  KWSWordDesc_t word;

  for (;;) {
    audio_t *proc_data =
      (audio_t
         *)&current_frames[(cur_frame % DET_VOICED_FRAMES_WINDOW) * FRAME_SZ];
    if (read_frame(buffer) < 0) {
      continue;
    }

    ns_process(s_ns_handle, buffer, proc_data);

    for (size_t i = 0; i < FRAME_LEN; i++) {
      proc_data[i] += white_noise_array
        [(cur_frame % size_t(float(KWS_SAMPLE_RATE) / FRAME_LEN)) * FRAME_LEN +
         i];
    }

    const size_t max_abs = compute_max_abs(proc_data, FRAME_LEN);

    max_abs_arr[cur_frame % DET_VOICED_FRAMES_WINDOW] = max_abs;

    const uint8_t is_speech =
      vad_process(s_vad_handle, proc_data, KWS_SAMPLE_RATE, FRAME_LEN_MS);
    num_voiced += is_speech;
    is_speech_arr[cur_frame % DET_VOICED_FRAMES_WINDOW] = is_speech;

    if (!trig) {
      if (num_voiced >= DET_VOICED_FRAMES_THRESHOLD) {
        word.frame_num = 0;
        word.max_abs = 0;
        trig = 1;
        ESP_LOGD(TAG, "__start[%d]=%d, max_abs=%d",
                 cur_frame - DET_VOICED_FRAMES_WINDOW, cur_frame, max_abs);
        for (size_t k = 1; k <= DET_VOICED_FRAMES_WINDOW; k++) {
          const size_t frame_num = (cur_frame + k) % DET_VOICED_FRAMES_WINDOW;
          audio_t *frame_ptr = (audio_t *)&current_frames[frame_num * FRAME_SZ];
          const auto xBytesSent =
            xStreamBufferSend(xKWSFramesBuffer, frame_ptr, FRAME_SZ, 0);
          if (xBytesSent < FRAME_SZ) {
            ESP_LOGW(TAG, "xKWSFramesBuffer: xBytesSent=%d (%d)", xBytesSent,
                     FRAME_SZ);
          }
          word.frame_num++;
          word.max_abs = std::max(word.max_abs, max_abs_arr[frame_num]);
        }
      }
    } else {
      if (num_voiced <= DET_UNVOICED_FRAMES_THRESHOLD) {
        trig = 0;
        ESP_LOGD(TAG, "__end[%d]=%d, max_abs=%d",
                 cur_frame - DET_VOICED_FRAMES_WINDOW, cur_frame, max_abs);
        xQueueSend(xKWSWordQueue, &word, 0);
      } else {
        word.frame_num++;
        word.max_abs = std::max(word.max_abs, max_abs);
        const auto xBytesSent =
          xStreamBufferSend(xKWSFramesBuffer, proc_data, FRAME_SZ, 0);
        if (xBytesSent < FRAME_SZ) {
          ESP_LOGW(TAG, "xKWSFramesBuffer: xBytesSent=%d (%d)", xBytesSent,
                   FRAME_SZ);
        }
      }
    }

    num_voiced -= is_speech_arr[(cur_frame + 1) % DET_VOICED_FRAMES_WINDOW];

    cur_frame++;
  }
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

  for (size_t i = 0; i < MIC_TEST_FRAME_NUM; i++) {
    audio_t *proc_data = (audio_t *)&current_frames[0];
    memset(proc_data, 0, FRAME_SZ);
    read_frame(proc_data);
    const float frame_mean = compute_mean(proc_data, FRAME_LEN);
    const float frame_std_dev =
      compute_std_dev(proc_data, FRAME_LEN, frame_mean);
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
  s_ns_handle = ns_pro_create(FRAME_LEN_MS, 2, KWS_SAMPLE_RATE);
  if (!s_ns_handle) {
    ESP_LOGE(TAG, "Unable to create esp_ns");
    return MIC_INIT_ERROR;
  }
  s_vad_handle = vad_create(VAD_MODE_3);
  if (!s_vad_handle) {
    ESP_LOGE(TAG, "Unable to create esp_vad");
    return MIC_INIT_ERROR;
  }
  s_agc_handle = esp_agc_open(3, KWS_SAMPLE_RATE);
  if (!s_agc_handle) {
    ESP_LOGE(TAG, "Unable to create agc");
    return MIC_INIT_ERROR;
  }
  set_agc_config(s_agc_handle, MIC_GAIN, 1, 0);

  i2s_receiver_init();
  mic_conf_t mic_conf[2] = {
    {
      .sample_rate = KWS_SAMPLE_RATE,
      .slot_type = stRight,
    },
    {
      .sample_rate = KWS_SAMPLE_RATE,
      .slot_type = stLeft,
    },
  };

  ESP_LOGV(TAG, "DEF_MEAN=%d, DEF_STD_DEV=%d", DEF_MEAN, DEF_STD_DEV);
  MicResult_t test_result;
  for (size_t i = 0; i < sizeof(mic_conf) / sizeof(mic_conf[0]); ++i) {
    ESP_LOGD(TAG, "Try conf[%d]", i);
    i2s_rx_slot_init(mic_conf[i]);
    i2s_rx_slot_start();

    if (i == 0) {
      // read first bad samples, init filter
      for (size_t i = 0; i < FILTER_INIT_FRAME_NUM; i++) {
        audio_t *proc_data = (audio_t *)&current_frames[0];
        read_frame(buffer);
        ns_process(s_ns_handle, buffer, proc_data);
      }
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

  auto xReturned =
    xTaskCreate(mic_proc_task, "mic_proc_task",
                configMINIMAL_STACK_SIZE + 1024 * 16, NULL, 2, NULL);
  if (xReturned != pdPASS) {
    ESP_LOGE(TAG, "Error creating mic_proc_task");
    return MIC_INIT_ERROR;
  }
  return MIC_OK;
}

void mic_reader_release() { i2s_rx_slot_release(); }
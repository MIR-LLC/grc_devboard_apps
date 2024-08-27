#ifndef _I2S_RX_SLOT_H_
#define _I2S_RX_SLOT_H_

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define MIC_ON_MSK BIT0

/*! \brief Global microphone semaphore. */
extern SemaphoreHandle_t xMicSema;
/*! \brief Global microphone raw data frames ring buffer. */
extern RingbufHandle_t xMicRingBuffer;
/*! \brief Global microphone event bits. */
extern EventGroupHandle_t xMicEventGroup;

typedef enum {
  stLeft = 0,
  stRight,
  stBoth,
} eSlotType;

typedef struct mic_conf_t {
  size_t sample_rate = 16000;
  eSlotType slot_type = stRight;
} mic_conf_t;

/*!
 * \brief Initialize microphone rx slot.
 * \param conf Config.
 */
void i2s_rx_slot_init(const mic_conf_t &conf);
/*!
 * \brief Enable microphone rx slot.
 */
void i2s_rx_slot_start();
/*!
 * \brief Stop microphone rx slot.
 */
void i2s_rx_slot_stop();
/*!
 * \brief Release microphone rx slot.
 */
void i2s_rx_slot_release();
/*!
 * \brief Read from rx slot.
 */
int i2s_rx_slot_read(void *buffer, size_t bytes, size_t timeout_ticks);

/*!
 * \brief Initialize microphone frames receiver.
 */
int i2s_receiver_init();
/*!
 * \brief Release microphone frames receiver.
 */
int i2s_receiver_release();

#endif // _I2S_RX_SLOT_H_

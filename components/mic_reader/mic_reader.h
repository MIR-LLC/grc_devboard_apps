#ifndef _MIC_READER_H_
#define _MIC_READER_H_

#include "def.h"

/*!
 * \brief Initialize microphone data reader.
 * \return Result.
 */
MicResult_t mic_reader_init();
/*!
 * \brief Release microphone data reader.
 */
void mic_reader_release();
/*!
 * \brief Read 1 mic data frame.
 * \return Result.
 */
int mic_reader_read_frame(audio_t *dst);
/*!
 * \brief Compute max abs value.
 * \return Result.
 */
size_t compute_max_abs(audio_t *data, size_t len);

#endif // _MIC_READER_H_

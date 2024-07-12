#ifndef _MIC_READER_H_
#define _MIC_READER_H_

#include "def.h"

extern const int16_t white_noise_array[];

/*!
 * \brief Initialize microphone data reader and processor.
 * \return Result.
 */
MicResult_t mic_reader_init();
/*!
 * \brief Release microphone data reader and processor.
 */
void mic_reader_release();

#endif // _MIC_READER_H_

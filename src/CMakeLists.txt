set(3RDPARTY_DIR "${PROJECT_DIR}/3rdparty")

set(KWS_DIR "${PROJECT_DIR}/src/KWS/")
set(KWS_SRC "${KWS_DIR}/kws.cpp" "${KWS_DIR}/model.cpp"
            "${KWS_DIR}/kws_preprocessor.cpp" "${KWS_DIR}/mfcc/mfcc.cpp")
set(KWS_INC "${KWS_DIR}/" "${KWS_DIR}/mfcc")

set(NMSIS_DIR "${3RDPARTY_DIR}/NMSIS/NMSIS/")
set(RISCV_MATH_SRC
    "${NMSIS_DIR}/DSP/Source/FastMathFunctions/riscv_cos_f32.c"
    "${NMSIS_DIR}/DSP/Source/CommonTables/riscv_common_tables.c"
    "${NMSIS_DIR}/DSP/Source/CommonTables/riscv_const_structs.c"
    "${NMSIS_DIR}/DSP/Source/TransformFunctions/riscv_rfft_fast_f32.c"
    "${NMSIS_DIR}/DSP/Source/TransformFunctions/riscv_cfft_f32.c"
    "${NMSIS_DIR}/DSP/Source/TransformFunctions/riscv_cfft_radix8_f32.c"
    "${NMSIS_DIR}/DSP/Source/TransformFunctions/riscv_bitreversal2.c"
    "${NMSIS_DIR}/DSP/Source/TransformFunctions/riscv_rfft_fast_init_f32.c"
    "${NMSIS_DIR}/DSP/Source/TransformFunctions/riscv_cfft_init_f32.c")
set(RISCV_MATH_INC "${NMSIS_DIR}/Core/Include/" "${NMSIS_DIR}/DSP/Include/")

set(MIC_SRC
    "${PROJECT_DIR}/src/MicReader/i2s_rx_slot.cpp"
    "${PROJECT_DIR}/src/MicReader/mic_reader.cpp"
    "${PROJECT_DIR}/src/MicReader/noise.cpp")
set(MIC_INC "${PROJECT_DIR}/src/MicReader/")

set(U8G2_DIR "${3RDPARTY_DIR}/u8g2")
set(U8G2_SRC
    "${U8G2_DIR}/csrc/u8x8_setup.c"
    "${U8G2_DIR}/csrc/u8x8_8x8.c"
    "${U8G2_DIR}/csrc/u8x8_display.c"
    "${U8G2_DIR}/csrc/u8x8_cad.c"
    "${U8G2_DIR}/csrc/u8x8_byte.c"
    "${U8G2_DIR}/csrc/u8x8_gpio.c"
    "${U8G2_DIR}/csrc/u8x8_d_ssd1306_128x64_noname.c"
    "${U8G2_DIR}/csrc/u8g2_setup.c"
    "${U8G2_DIR}/csrc/u8g2_cleardisplay.c"
    "${U8G2_DIR}/csrc/u8g2_buffer.c"
    "${U8G2_DIR}/csrc/u8g2_fonts.c"
    "${U8G2_DIR}/csrc/u8g2_font.c"
    "${U8G2_DIR}/csrc/u8g2_hvline.c"
    "${U8G2_DIR}/csrc/u8g2_ll_hvline.c"
    "${U8G2_DIR}/csrc/u8g2_intersection.c"
    "${U8G2_DIR}/csrc/u8g2_d_setup.c"
    "${U8G2_DIR}/csrc/u8g2_d_memory.c"
    "${U8G2_DIR}/cppsrc/U8x8lib.cpp")
set(U8G2_INC "${U8G2_DIR}/cppsrc" "${U8G2_DIR}/csrc")

idf_component_register(
  SRCS
  "main.cpp"
  "./App/App.cpp"
  "./App/KWS.cpp"
  "./App/Event.cpp"
  "./App/Status.cpp"
  "./Hardware/Led.cpp"
  "./Hardware/Lcd.cpp"
  "./Hardware/I2C.cpp"
  "./Hardware/Button.cpp"
  ${RISCV_MATH_SRC}
  ${KWS_SRC}
  ${MIC_SRC}
  ${U8G2_SRC}
  INCLUDE_DIRS
  "./"
  "./App/include"
  "./Hardware"
  ${RISCV_MATH_INC}
  ${KWS_INC}
  ${MIC_INC}
  ${U8G2_INC}
  REQUIRES
  "led_strip"
  "nvs_flash"
  "esp_timer"
  "driver")

target_compile_options(
  ${COMPONENT_LIB}
  PRIVATE -Wno-error=unused-const-variable -Wno-error=delete-non-virtual-dtor
          -Wno-error=implicit-function-declaration -fpermissive)
add_compile_definitions(U8X8_USE_PINS)

# Set defines
add_compile_definitions(MIC_GAIN=30)
add_compile_definitions(KWS_SAMPLE_RATE=16000)
add_compile_definitions(KWS_GET_MFCC=1)
add_compile_definitions(KWS_INFERENCE_THRESHOLD=0.95)

add_compile_definitions(SUSPEND_TIMEOUT_S=10)

if(BUILD_TARGET)
  add_compile_definitions("TARGET_${BUILD_TARGET}")
else()
  message(FATAL_ERROR "No build target passed (AI_MODULE or GRC_DEVBOARD)")
endif()

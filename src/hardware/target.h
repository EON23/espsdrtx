/*
 * The chip being built for: its public limits (ESPSDRTX_MAX_SAMPLES,
 * ESPSDRTX_MAX_WORD), from hardware/<chip>/target.h. espsdrtx.h includes
 * this; hardware/README.md says how to add a chip.
 */
#pragma once

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/target.h"
#elif CONFIG_IDF_TARGET_ESP32C6
#include "esp32c6/target.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/target.h"
#else
#error "espsdrtx: no port for this chip yet (see src/hardware/README.md)"
#endif

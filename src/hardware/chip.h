/*
 * Which port directory hardware/<chip>/ the chip being built for uses,
 * named as ESP-IDF names the target (IDF_TARGET). A new port adds its
 * target here, next to its directory; target.h and port.h in this directory
 * then include the chip's files, and a chip not listed fails the build with
 * a message.
 *
 * Private to the library; espsdrtx.h reaches it through target.h.
 */
#pragma once

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32C3
#define ESPSDRTX_CHIP_TARGET_H "esp32c3/target.h"
#define ESPSDRTX_CHIP_PORT_H   "esp32c3/port.h"
#elif CONFIG_IDF_TARGET_ESP32C6
#define ESPSDRTX_CHIP_TARGET_H "esp32c6/target.h"
#define ESPSDRTX_CHIP_PORT_H   "esp32c6/port.h"
#endif

/*
 * The chip being built for: its public limits (ESPSDRTX_MAX_SAMPLES,
 * ESPSDRTX_MAX_WORD), from hardware/<chip>/target.h, picked by chip.h.
 * espsdrtx.h includes this; hardware/README.md says how to add a chip.
 */
#pragma once

#include "chip.h"

#ifndef ESPSDRTX_CHIP_TARGET_H
#error "espsdrtx: no port for this chip yet (see src/hardware/README.md)"
#else
#if __has_include(ESPSDRTX_CHIP_TARGET_H)
#include ESPSDRTX_CHIP_TARGET_H
#else
#error "espsdrtx: no port for this chip yet (see src/hardware/README.md)"
#endif
#endif

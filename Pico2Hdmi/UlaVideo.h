#pragma once

#include <stddef.h>
#include <stdint.h>

namespace zx {

// The Spectrum renderer works at 360x288. The HSTX callback duplicates each
// logical pixel and line to fill the 720x576 HDMI active area.
constexpr size_t kVideoWidth = 360;
constexpr size_t kVideoHeight = 288;
constexpr size_t kOutputWidth = kVideoWidth * 2;
constexpr size_t kOutputHeight = kVideoHeight * 2;
constexpr size_t kSpectrumWidth = 256;
constexpr size_t kSpectrumHeight = 192;
constexpr size_t kBorderLeft = (kVideoWidth - kSpectrumWidth) / 2;
constexpr size_t kBorderTop = (kVideoHeight - kSpectrumHeight) / 2;

// Render one logical scanline as packed RGB565 output. Each uint32_t contains
// two identical horizontal pixels; HSTX calls this for both physical lines.
// frameNumber drives the Spectrum FLASH cadence.
void renderUlaScanline(uint32_t *pixels, size_t y, uint32_t frameNumber);

}  // namespace zx

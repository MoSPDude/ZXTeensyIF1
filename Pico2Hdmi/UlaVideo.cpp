#include "UlaVideo.h"

#include <pico.h>

#include "EmulationState.h"

// Spectrum scanline layout adapted from fruit-bat/pico-zxspectrum's
// ZxSpectrumPrepareDviScanline.cpp (MIT, copyright fruit-bat contributors).

namespace zx {
namespace {

// RGB565 approximations of the Spectrum's normal (0xcd) and BRIGHT (0xff)
// component levels. Spectrum colour numbers are encoded as G-R-B.
constexpr uint16_t kNormalColours[8] = {
    0x0000,  // black
    0x0019,  // blue
    0xc800,  // red
    0xc819,  // magenta
    0x0660,  // green
    0x0679,  // cyan
    0xce60,  // yellow
    0xce79,  // white
};

constexpr uint16_t kBrightColours[8] = {
    0x0000,  // black remains black
    0x001f,  // blue
    0xf800,  // red
    0xf81f,  // magenta
    0x07e0,  // green
    0x07ff,  // cyan
    0xffe0,  // yellow
    0xffff,  // white
};

constexpr uint32_t duplicatePixel(uint16_t colour) {
  return colour | (static_cast<uint32_t>(colour) << 16);
}

constexpr size_t bitmapOffset(size_t y, size_t characterColumn) {
  // The ULA stores scanlines in thirds, then character rows, then pixel rows.
  return ((y & 0xc0) << 5) | ((y & 0x07) << 8) |
         ((y & 0x38) << 2) | characterColumn;
}

}  // namespace

void __not_in_flash_func(renderUlaScanline)(uint32_t *pixels, size_t y,
                                            uint32_t frameNumber) {
  const uint32_t border = duplicatePixel(
      kNormalColours[machineState.ula.borderColour & 0x07]);
  for (size_t x = 0; x < kVideoWidth; ++x) {
    pixels[x] = border;
  }

  if (y < kBorderTop || y >= kBorderTop + kSpectrumHeight) {
    return;
  }

  const size_t screenY = y - kBorderTop;
  const size_t screenNumber = (machineState.memoryControl & 0x08) != 0 ? 1 : 0;
  const uint8_t *display = machineState.displayFile[screenNumber];
  const size_t attributeRow = kBitmapSize + (screenY >> 3) * 32;
  uint32_t *output = pixels + kBorderLeft;

  for (size_t column = 0; column < 32; ++column) {
    uint8_t bitmap = display[bitmapOffset(screenY, column)];
    const uint8_t attribute = display[attributeRow + column];
    const uint16_t *palette =
        (attribute & 0x40) != 0 ? kBrightColours : kNormalColours;
    uint16_t ink = palette[attribute & 0x07];
    uint16_t paper = palette[(attribute >> 3) & 0x07];

    // A Spectrum toggles FLASH every 16 50 Hz frames.  Swapping the colours
    // instead of modifying the bitmap also handles identical INK/PAPER values.
    if ((attribute & 0x80) != 0 && (frameNumber & 0x10) != 0) {
      const uint16_t temporary = ink;
      ink = paper;
      paper = temporary;
    }

    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
      *output++ = duplicatePixel((bitmap & mask) != 0 ? ink : paper);
    }
  }
}

}  // namespace zx

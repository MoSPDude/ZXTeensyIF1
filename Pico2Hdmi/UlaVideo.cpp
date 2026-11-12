#include "UlaVideo.h"

#include <pico.h>

#include "EmulationState.h"
#include "RasterEvents.h"

// Spectrum scanline layout adapted from fruit-bat/pico-zxspectrum's
// ZxSpectrumPrepareDviScanline.cpp (MIT, copyright fruit-bat contributors).

namespace zx {
namespace {

// RGB565 approximations of the Spectrum's normal (0xcd) and BRIGHT (0xff)
// component levels. Spectrum colour numbers are encoded as G-R-B.
// Keep lookup data in SRAM too: a RAM renderer must not fetch its palette
// through the flash cache while USB/audio processing runs on the other core.
uint16_t kNormalColours[8] = {
    0x0000,  // black
    0x0019,  // blue
    0xc800,  // red
    0xc819,  // magenta
    0x0660,  // green
    0x0679,  // cyan
    0xce60,  // yellow
    0xce79,  // white
};

uint16_t kBrightColours[8] = {
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

constexpr uint32_t packPixels(uint16_t left, uint16_t right) {
  return left | (static_cast<uint32_t>(right) << 16);
}

constexpr size_t bitmapOffset(size_t y, size_t characterColumn) {
  // The ULA stores scanlines in thirds, then character rows, then pixel rows.
  return ((y & 0xc0) << 5) | ((y & 0x07) << 8) |
         ((y & 0x38) << 2) | characterColumn;
}

inline __attribute__((always_inline)) void attributeColours(
    uint8_t attribute, bool ulaPlusEnabled, uint32_t frameNumber, uint16_t &ink,
    uint16_t &paper) {
  if (ulaPlusEnabled) {
    const size_t group = static_cast<size_t>(attribute >> 6) << 4;
    ink = machineState.ulaPlus.paletteRgb565[group + (attribute & 0x07)];
    paper = machineState.ulaPlus.paletteRgb565[
        group + 8 + ((attribute >> 3) & 0x07)];
    return;
  }

  const uint16_t *palette =
      (attribute & 0x40) != 0 ? kBrightColours : kNormalColours;
  ink = palette[attribute & 0x07];
  paper = palette[(attribute >> 3) & 0x07];

  // A Spectrum toggles FLASH every 16 50 Hz frames. Swapping the colours
  // instead of modifying the bitmap also handles identical values.
  if ((attribute & 0x80) != 0 && (frameNumber & 0x10) != 0) {
    const uint16_t temporary = ink;
    ink = paper;
    paper = temporary;
  }
}

inline __attribute__((always_inline)) void highResolutionColours(
    bool ulaPlusEnabled, uint8_t colour, uint16_t &ink, uint16_t &paper) {
  const uint8_t inkIndex = colour & 0x07;
  const uint8_t paperIndex = 7 - inkIndex;
  if (ulaPlusEnabled) {
    // ULAplus assigns Timex hi-res to the paper half of CLUT 1.
    ink = machineState.ulaPlus.paletteRgb565[24 + inkIndex];
    paper = machineState.ulaPlus.paletteRgb565[24 + paperIndex];
  } else {
    ink = kBrightColours[inkIndex];
    paper = kBrightColours[paperIndex];
  }
}

}  // namespace

void __not_in_flash_func(renderUlaScanline)(uint32_t *pixels, size_t y,
                                            uint32_t frameNumber) {
  const bool ulaPlusEnabled = (machineState.ulaPlus.mode & 0x01) != 0;
  const uint8_t timexMode = machineState.timex.active
                                ? machineState.timex.mode
                                : 0;
  uint16_t highResolutionInk = 0;
  uint16_t highResolutionPaper = 0;
  uint16_t borderRgb;
  if (timexMode == 0x06) {
    highResolutionColours(ulaPlusEnabled, machineState.timex.colour,
                          highResolutionInk, highResolutionPaper);
    borderRgb = highResolutionPaper;
  } else {
    const uint8_t borderColour = machineState.ula.borderColour & 0x07;
    borderRgb = ulaPlusEnabled
                    ? machineState.ulaPlus.paletteRgb565[8 + borderColour]
                    : kNormalColours[borderColour];
  }
  const uint32_t border = duplicatePixel(borderRgb);
  for (size_t x = 0; x < kVideoWidth; ++x) {
    pixels[x] = border;
  }

  if (y < kBorderTop || y >= kBorderTop + kSpectrumHeight) {
    return;
  }

  const size_t screenY = y - kBorderTop;
  const size_t screenNumber = machineState.timex.active
                                  ? machineState.timex.screen
                                  : ((machineState.memoryControl & 0x08) != 0
                                         ? 1
                                         : 0);
  const uint8_t *display = machineState.displayFile[screenNumber];
  const RasterDisplayLine *line = rasterDisplayLine(screenY);
  uint32_t *output = pixels + kBorderLeft;

  if (timexMode == 0x06) {
    // The SCLD alternates an eight-pixel column from the primary file with
    // one from the second file, producing 512 physical pixels in the same
    // active width used by doubled 256-pixel modes.
    for (size_t column = 0; column < 32; ++column) {
      const size_t offset = bitmapOffset(screenY, column);
      const uint8_t bitmaps[2] = {
          line ? line->bitmap[0][column] : display[offset],
          line ? line->bitmap[1][column]
               : display[kTimexSecondDisplayOffset + offset],
      };
      for (uint8_t bitmap : bitmaps) {
        for (uint8_t mask = 0x80; mask != 0; mask >>= 2) {
          const uint16_t left = (bitmap & mask) != 0
                                    ? highResolutionInk
                                    : highResolutionPaper;
          const uint16_t right = (bitmap & (mask >> 1)) != 0
                                     ? highResolutionInk
                                     : highResolutionPaper;
          *output++ = packPixels(left, right);
        }
      }
    }
    return;
  }

  const size_t displayBase =
      timexMode == 0x01 ? kTimexSecondDisplayOffset : 0;
  const size_t displayFile = timexMode == 0x01 ? 1 : 0;
  const size_t attributeRow =
      displayBase + kBitmapSize + (screenY >> 3) * 32;

  for (size_t column = 0; column < 32; ++column) {
    const size_t pixelOffset = bitmapOffset(screenY, column);
    uint8_t bitmap = line ? line->bitmap[displayFile][column]
                          : display[displayBase + pixelOffset];
    const uint8_t attribute =
        timexMode == 0x02
            ? (line ? line->bitmap[1][column]
                    : display[kTimexSecondDisplayOffset + pixelOffset])
            : (line ? line->attributes[displayFile][column]
                    : display[attributeRow + column]);
    uint16_t ink;
    uint16_t paper;
    attributeColours(attribute, ulaPlusEnabled, frameNumber, ink, paper);

    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
      *output++ = duplicatePixel((bitmap & mask) != 0 ? ink : paper);
    }
  }
}

}  // namespace zx

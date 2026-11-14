#pragma once

#include <stddef.h>
#include <stdint.h>

#include "UlaVideo.h"

namespace zx {

constexpr uint8_t kRasterPhaseBits = 7;
constexpr uint16_t kRasterPhaseMask = (1u << kRasterPhaseBits) - 1;
constexpr size_t kRasterSourceLineCount = 312;
constexpr size_t kRasterFirstRenderedLine = 16;
constexpr size_t kRasterOutputFieldCount = 2;

struct RasterEventCounters {
  uint32_t frameMarkers;
  uint32_t queuedEvents;
  uint32_t coalescedEvents;
  uint32_t playedFrames;
  uint32_t eventOverflows;
  uint32_t droppedFrames;
  uint32_t outputLockMisses;
};

extern volatile RasterEventCounters rasterEventCounters;

// Bytes fetched for one source scanline, from the screen selected at that
// time. Keep both Timex files so the renderer can decode every video mode.
struct RasterDisplayLine {
  uint8_t bitmap[2][kSpectrumWidth / 8];
  uint8_t attributes[2][kSpectrumWidth / 8];
};

// Set up the cross-core frame buffers before USB or HSTX begins using them.
void initializeRasterEvents();

// Discard pending frames after a machine reset. The currently rendered buffer
// remains immutable until core 1 releases it at the next HDMI frame boundary.
void resetRasterEvents();

// Core-0 protocol input. A marker closes the preceding source frame and starts
// a new one. Timestamp metadata applies to following screen and control writes.
void beginRasterSourceFrame(uint8_t sequence);
void setRasterTimestamp(uint16_t timestamp);

// Resolve and queue screen-memory or video-control writes. Returns true once
// source-frame timing is active, including when a write had to be dropped, so
// callers never apply a late frame-buffered write immediately.
bool queueRasterDisplayWrite(uint16_t address, uint8_t data);
bool queueRasterUlaPlusWrite(uint8_t selectedRegister, uint8_t data);
bool queueRasterTimexWrite(uint8_t data);
bool queueRasterBorderWrite(uint8_t data);
bool queueRasterMemoryControlWrite(uint8_t data);

// Core-1 HSTX input. Retain one completed source frame for each output field,
// then apply that field's events before its physical output line is rendered.
// Untimed startup renders live RAM; once source timing starts, an unavailable
// field returns false so the callback can blank only that field.
void beginRasterOutputFrame();
bool applyRasterEventsForOutputLine(size_t outputLine, size_t field = 0);
void finishRasterOutputLine(size_t outputLine, size_t field = 0);

// Return the immutable source scanline selected for one HDMI output field.
// Untimed startup/reset returns null to select live RAM, even while an older
// playing buffer is waiting to be released at the next output-frame boundary.
const RasterDisplayLine *rasterDisplayLine(size_t y, size_t field = 0);

}  // namespace zx

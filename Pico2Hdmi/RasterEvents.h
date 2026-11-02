#pragma once

#include <stddef.h>
#include <stdint.h>

namespace zx {

constexpr uint8_t kRasterPhaseBits = 7;
constexpr uint16_t kRasterPhaseMask = (1u << kRasterPhaseBits) - 1;
constexpr size_t kRasterSourceLineCount = 312;
constexpr size_t kRasterFirstRenderedLine = 16;

struct RasterEventCounters {
  uint32_t frameMarkers;
  uint32_t queuedEvents;
  uint32_t playedFrames;
  uint32_t eventOverflows;
  uint32_t droppedFrames;
};

extern volatile RasterEventCounters rasterEventCounters;

// Set up the cross-core frame buffers before USB or HSTX begins using them.
void initializeRasterEvents();

// Discard pending frames after a machine reset. The currently rendered buffer
// remains immutable until core 1 releases it at the next HDMI frame boundary.
void resetRasterEvents();

// Core-0 protocol input. A marker closes the preceding source frame and starts
// a new one. Timestamp metadata applies to following video-control writes.
void beginRasterSourceFrame(uint8_t sequence);
void setRasterTimestamp(uint16_t timestamp);

// Resolve and queue video-control writes. Returns true once source-frame
// timing is active, including when an event had to be dropped, so callers
// never apply a late timed event immediately.
bool queueRasterUlaPlusWrite(uint8_t selectedRegister, uint8_t data);
bool queueRasterTimexWrite(uint8_t data);

// Core-1 HSTX input. Select one completed source frame at HDMI frame start,
// then apply its events before each logical output line is rendered.
void beginRasterOutputFrame();
void applyRasterEventsForOutputLine(size_t outputLine);
void finishRasterOutputLine(size_t outputLine);

}  // namespace zx

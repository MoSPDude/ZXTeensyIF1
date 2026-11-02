#include "RasterEvents.h"

#include <pico.h>
#include <pico/sync.h>

#include "EmulationState.h"
#include "UlaVideo.h"

namespace zx {
namespace {

constexpr size_t kRasterBufferCount = 3;
constexpr size_t kRasterEventsPerFrame = 4096;
constexpr int8_t kNoBuffer = -1;

enum class BufferState : uint8_t {
  kFree,
  kCollecting,
  kReady,
  kPlaying,
};

enum class EventType : uint8_t {
  kUlaPlus,
  kTimex,
};

struct RasterEvent {
  uint16_t timestamp;
  EventType type;
  uint8_t selectedRegister;
  uint8_t data;
};

struct RasterFrameBuffer {
  RasterEvent events[kRasterEventsPerFrame];
  uint8_t palette[kUlaPlusPaletteSize];
  uint8_t modeRegister;
  uint8_t mode;
  TimexState timex;
  size_t count;
  uint32_t ordinal;
  uint8_t sequence;
  BufferState state;
};

RasterFrameBuffer frameBuffers[kRasterBufferCount];
critical_section_t frameBufferLock;
int8_t collectingBuffer = 0;
int8_t playingBuffer = kNoBuffer;
size_t playingEvent = 0;
uint16_t currentTimestamp = 0;
uint32_t nextFrameOrdinal = 0;
volatile bool sourceTimingActive = false;
volatile uint32_t resetGeneration = 0;
uint32_t playingResetGeneration = 0;
uint8_t sourcePalette[kUlaPlusPaletteSize] = {};
uint8_t sourceModeRegister = 0x40;
uint8_t sourceMode = 0;
TimexState sourceTimex = {};

int8_t findFreeBuffer() {
  for (size_t i = 0; i < kRasterBufferCount; ++i) {
    if (frameBuffers[i].state == BufferState::kFree) {
      return static_cast<int8_t>(i);
    }
  }
  return kNoBuffer;
}

int8_t findOldestReadyBuffer() {
  int8_t oldest = kNoBuffer;
  for (size_t i = 0; i < kRasterBufferCount; ++i) {
    if (frameBuffers[i].state != BufferState::kReady) {
      continue;
    }
    if (oldest == kNoBuffer ||
        frameBuffers[i].ordinal < frameBuffers[oldest].ordinal) {
      oldest = static_cast<int8_t>(i);
    }
  }
  return oldest;
}

size_t eventSourceLine(const RasterEvent &event) {
  size_t line = event.timestamp >> kRasterPhaseBits;
  // The first implementation schedules at line boundaries. Preserve the
  // sub-line field on the wire and round a future non-zero phase forward.
  if ((event.timestamp & kRasterPhaseMask) != 0) {
    ++line;
  }
  return line;
}

void applyEvent(const RasterEvent &event) {
  if (event.type == EventType::kUlaPlus) {
    applyUlaPlusRegister(event.selectedRegister, event.data);
  } else {
    applyTimexControl(event.data);
  }
}

void captureSourceState(RasterFrameBuffer &frame) {
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    frame.palette[i] = sourcePalette[i];
  }
  frame.modeRegister = sourceModeRegister;
  frame.mode = sourceMode;
  frame.timex = sourceTimex;
}

void applySourceState(const RasterFrameBuffer &frame) {
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    applyUlaPlusRegister(static_cast<uint8_t>(i), frame.palette[i]);
  }
  applyUlaPlusRegister(frame.modeRegister, frame.mode);
  machineState.timex = frame.timex;
}

void updateSourceState(uint8_t selectedRegister, uint8_t data) {
  if (selectedRegister < kUlaPlusPaletteSize) {
    sourcePalette[selectedRegister] = data;
  } else if ((selectedRegister & 0xc0) == 0x40) {
    sourceModeRegister = selectedRegister;
    sourceMode = data;
    sourceTimex = decodeTimexControl(selectedRegister, true);
  }
}

void updateSourceTimexState(uint8_t data) {
  sourceTimex = decodeTimexControl(data, false);
}

bool queueRasterEvent(EventType type, uint8_t selectedRegister, uint8_t data) {
  if (type == EventType::kUlaPlus) {
    updateSourceState(selectedRegister, data);
  } else {
    updateSourceTimexState(data);
  }
  if (!sourceTimingActive) {
    return false;
  }

  critical_section_enter_blocking(&frameBufferLock);
  if (collectingBuffer == kNoBuffer) {
    rasterEventCounters.eventOverflows =
        rasterEventCounters.eventOverflows + 1;
    critical_section_exit(&frameBufferLock);
    return true;
  }

  RasterFrameBuffer &collecting = frameBuffers[collectingBuffer];
  if (collecting.count == kRasterEventsPerFrame) {
    rasterEventCounters.eventOverflows =
        rasterEventCounters.eventOverflows + 1;
    critical_section_exit(&frameBufferLock);
    return true;
  }

  RasterEvent &event = collecting.events[collecting.count++];
  event.timestamp = currentTimestamp;
  event.type = type;
  event.selectedRegister = selectedRegister;
  event.data = data;
  rasterEventCounters.queuedEvents = rasterEventCounters.queuedEvents + 1;
  critical_section_exit(&frameBufferLock);
  return true;
}

void stopPlayingAfterReset() {
  if (playingResetGeneration != resetGeneration) {
    playingResetGeneration = resetGeneration;
    if (playingBuffer != kNoBuffer) {
      playingEvent = frameBuffers[playingBuffer].count;
    }
  }
}

}  // namespace

volatile RasterEventCounters rasterEventCounters = {};

void initializeRasterEvents() {
  critical_section_init(&frameBufferLock);
  for (size_t i = 0; i < kRasterBufferCount; ++i) {
    frameBuffers[i].count = 0;
    frameBuffers[i].ordinal = 0;
    frameBuffers[i].sequence = 0;
    frameBuffers[i].state = BufferState::kFree;
  }
  collectingBuffer = 0;
  frameBuffers[collectingBuffer].state = BufferState::kCollecting;
  playingBuffer = kNoBuffer;
  playingEvent = 0;
  currentTimestamp = 0;
  nextFrameOrdinal = 0;
  sourceTimingActive = false;
  resetGeneration = 0;
  playingResetGeneration = 0;
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    sourcePalette[i] = 0;
  }
  sourceModeRegister = 0x40;
  sourceMode = 0;
  sourceTimex = {};
}

void resetRasterEvents() {
  critical_section_enter_blocking(&frameBufferLock);
  ++resetGeneration;
  sourceTimingActive = false;
  currentTimestamp = 0;
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    sourcePalette[i] = 0;
  }
  sourceModeRegister = 0x40;
  sourceMode = 0;
  sourceTimex = {};

  for (size_t i = 0; i < kRasterBufferCount; ++i) {
    if (static_cast<int8_t>(i) != playingBuffer) {
      frameBuffers[i].count = 0;
      frameBuffers[i].state = BufferState::kFree;
    }
  }

  collectingBuffer = findFreeBuffer();
  if (collectingBuffer != kNoBuffer) {
    frameBuffers[collectingBuffer].count = 0;
    frameBuffers[collectingBuffer].state = BufferState::kCollecting;
  }
  critical_section_exit(&frameBufferLock);
}

void beginRasterSourceFrame(uint8_t sequence) {
  critical_section_enter_blocking(&frameBufferLock);
  rasterEventCounters.frameMarkers = rasterEventCounters.frameMarkers + 1;
  currentTimestamp = 0;

  if (!sourceTimingActive) {
    sourceTimingActive = true;
    if (collectingBuffer == kNoBuffer) {
      collectingBuffer = findFreeBuffer();
    }
    if (collectingBuffer != kNoBuffer) {
      RasterFrameBuffer &collecting = frameBuffers[collectingBuffer];
      collecting.count = 0;
      collecting.sequence = sequence;
      captureSourceState(collecting);
      collecting.state = BufferState::kCollecting;
    }
    critical_section_exit(&frameBufferLock);
    return;
  }

  if (collectingBuffer != kNoBuffer) {
    RasterFrameBuffer &completed = frameBuffers[collectingBuffer];
    const uint8_t sequenceDelta =
        static_cast<uint8_t>(sequence - completed.sequence);
    if (sequenceDelta == 1) {
      completed.ordinal = nextFrameOrdinal++;
      completed.state = BufferState::kReady;
    } else {
      // Without every marker, the collected writes cannot be assigned to one
      // source frame. A later frame snapshot restores their persistent state.
      completed.count = 0;
      completed.state = BufferState::kFree;
      rasterEventCounters.droppedFrames =
          rasterEventCounters.droppedFrames + 1;
    }
    collectingBuffer = kNoBuffer;
  } else {
    rasterEventCounters.droppedFrames =
        rasterEventCounters.droppedFrames + 1;
  }

  collectingBuffer = findFreeBuffer();
  if (collectingBuffer != kNoBuffer) {
    RasterFrameBuffer &collecting = frameBuffers[collectingBuffer];
    collecting.count = 0;
    collecting.sequence = sequence;
    captureSourceState(collecting);
    collecting.state = BufferState::kCollecting;
  }
  critical_section_exit(&frameBufferLock);
}

void setRasterTimestamp(uint16_t timestamp) {
  currentTimestamp = timestamp;
}

bool queueRasterUlaPlusWrite(uint8_t selectedRegister, uint8_t data) {
  return queueRasterEvent(EventType::kUlaPlus, selectedRegister, data);
}

bool queueRasterTimexWrite(uint8_t data) {
  return queueRasterEvent(EventType::kTimex, 0, data);
}

void beginRasterOutputFrame() {
  critical_section_enter_blocking(&frameBufferLock);
  if (playingBuffer != kNoBuffer) {
    frameBuffers[playingBuffer].count = 0;
    frameBuffers[playingBuffer].state = BufferState::kFree;
    playingBuffer = kNoBuffer;
  }

  const int8_t ready = findOldestReadyBuffer();
  if (ready != kNoBuffer) {
    playingBuffer = ready;
    frameBuffers[playingBuffer].state = BufferState::kPlaying;
    playingEvent = 0;
    playingResetGeneration = resetGeneration;
    applySourceState(frameBuffers[playingBuffer]);
    rasterEventCounters.playedFrames = rasterEventCounters.playedFrames + 1;
  }
  critical_section_exit(&frameBufferLock);
}

void applyRasterEventsForOutputLine(size_t outputLine) {
  stopPlayingAfterReset();
  if (playingBuffer == kNoBuffer) {
    return;
  }

  RasterFrameBuffer &playing = frameBuffers[playingBuffer];
  const size_t sourceLine = kRasterFirstRenderedLine + outputLine;
  while (playingEvent < playing.count &&
         eventSourceLine(playing.events[playingEvent]) <= sourceLine) {
    applyEvent(playing.events[playingEvent++]);
  }
}

void finishRasterOutputLine(size_t outputLine) {
  stopPlayingAfterReset();
  if (playingBuffer == kNoBuffer || outputLine + 1 != kVideoHeight) {
    return;
  }

  // Apply writes from the final cropped/blank source lines after the last
  // visible line so their persistent state is correct for the next frame.
  RasterFrameBuffer &playing = frameBuffers[playingBuffer];
  while (playingEvent < playing.count) {
    applyEvent(playing.events[playingEvent++]);
  }
}

}  // namespace zx

#include "RasterEvents.h"

#include <pico.h>
#include <pico/sync.h>
#include <string.h>

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
  kBorder,
  kMemoryControl,
};

struct RasterEvent {
  uint16_t timestamp;
  EventType type;
  uint8_t selectedRegister;
  uint8_t data;
};

struct RasterVideoState {
  uint8_t palette[kUlaPlusPaletteSize];
  uint8_t modeRegister;
  uint8_t mode;
  TimexState timex;
  uint8_t borderColour;
  uint8_t memoryControl;
};

struct RasterFrameBuffer {
  RasterEvent events[kRasterEventsPerFrame];
  uint8_t displayFile[kScreenCount][kDisplayMemorySize];
  RasterVideoState visibleStartState;
  RasterVideoState endState;
  size_t count;
  size_t firstVisibleEvent;
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
uint8_t sourceBorderColour = 0;
uint8_t sourceMemoryControl = 0;

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
  switch (event.type) {
    case EventType::kUlaPlus:
      applyUlaPlusRegister(event.selectedRegister, event.data);
      break;

    case EventType::kTimex:
      applyTimexControl(event.data);
      break;

    case EventType::kBorder:
      applyBorderColour(event.data);
      break;

    case EventType::kMemoryControl:
      applyMemoryControl(event.data);
      break;
  }
}

void captureSourceState(RasterVideoState &state) {
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    state.palette[i] = sourcePalette[i];
  }
  state.modeRegister = sourceModeRegister;
  state.mode = sourceMode;
  state.timex = sourceTimex;
  state.borderColour = sourceBorderColour;
  state.memoryControl = sourceMemoryControl;
}

void captureSourceDisplay(RasterFrameBuffer &frame) {
  memcpy(frame.displayFile, machineState.displayFile,
         sizeof(frame.displayFile));
}

void applySourceState(const RasterVideoState &state) {
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    applyUlaPlusRegister(static_cast<uint8_t>(i), state.palette[i]);
  }
  applyUlaPlusRegister(state.modeRegister, state.mode);
  machineState.timex = state.timex;
  applyBorderColour(state.borderColour);
  applyMemoryControl(state.memoryControl);
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

void updateSourceBorderState(uint8_t data) {
  sourceBorderColour = data & 0x07;
}

void updateSourceMemoryControlState(uint8_t data) {
  sourceMemoryControl = data;
}

bool sameEventTarget(const RasterEvent &left, EventType rightType,
                     uint8_t rightSelectedRegister) {
  if (left.type != rightType) {
    return false;
  }
  if (rightType != EventType::kUlaPlus) {
    return true;
  }

  constexpr uint8_t kUlaPlusModeGroup = 0x40;
  constexpr uint8_t kUlaPlusGroupMask = 0xc0;
  const bool leftIsMode =
      (left.selectedRegister & kUlaPlusGroupMask) == kUlaPlusModeGroup;
  const bool rightIsMode =
      (rightSelectedRegister & kUlaPlusGroupMask) == kUlaPlusModeGroup;
  return (leftIsMode && rightIsMode) ||
         (left.selectedRegister == rightSelectedRegister);
}

bool queueRasterEvent(EventType type, uint8_t selectedRegister, uint8_t data) {
  switch (type) {
    case EventType::kUlaPlus:
      updateSourceState(selectedRegister, data);
      break;

    case EventType::kTimex:
      updateSourceTimexState(data);
      break;

    case EventType::kBorder:
      updateSourceBorderState(data);
      break;

    case EventType::kMemoryControl:
      updateSourceMemoryControlState(data);
      break;
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
  for (size_t i = collecting.count; i != 0; --i) {
    const size_t existingIndex = i - 1;
    const RasterEvent &existing = collecting.events[existingIndex];
    if (existing.timestamp != currentTimestamp) {
      break;
    }
    if (!sameEventTarget(existing, type, selectedRegister)) {
      continue;
    }

    // Move the replacement to the end so its ordering relative to other
    // state targets at this timestamp remains the same as the source writes.
    for (size_t move = existingIndex; move + 1 < collecting.count; ++move) {
      collecting.events[move] = collecting.events[move + 1];
    }
    --collecting.count;
    rasterEventCounters.coalescedEvents =
        rasterEventCounters.coalescedEvents + 1;
    break;
  }

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
  if (eventSourceLine(event) <= kRasterFirstRenderedLine) {
    // Top-cropped writes only establish the state for the first visible line.
    // Capture that state on core 0 so core 1 never has to replay the cropped
    // prefix in its HSTX DMA interrupt.
    captureSourceState(collecting.visibleStartState);
    collecting.firstVisibleEvent = collecting.count;
  }
  critical_section_exit(&frameBufferLock);
  return true;
}

void stopPlayingAfterReset() {
  if (playingResetGeneration != resetGeneration) {
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
  captureSourceDisplay(frameBuffers[collectingBuffer]);
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
  sourceBorderColour = 0;
  sourceMemoryControl = 0;
}

void resetRasterEvents() {
  int8_t displayCaptureBuffer = kNoBuffer;
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
  sourceBorderColour = 0;
  sourceMemoryControl = 0;

  for (size_t i = 0; i < kRasterBufferCount; ++i) {
    if (static_cast<int8_t>(i) != playingBuffer) {
      frameBuffers[i].count = 0;
      frameBuffers[i].state = BufferState::kFree;
    }
  }

  collectingBuffer = findFreeBuffer();
  if (collectingBuffer != kNoBuffer) {
    frameBuffers[collectingBuffer].count = 0;
    frameBuffers[collectingBuffer].firstVisibleEvent = 0;
    frameBuffers[collectingBuffer].state = BufferState::kCollecting;
    displayCaptureBuffer = collectingBuffer;
  }
  critical_section_exit(&frameBufferLock);
  if (displayCaptureBuffer != kNoBuffer) {
    captureSourceDisplay(frameBuffers[displayCaptureBuffer]);
  }
}

void beginRasterSourceFrame(uint8_t sequence) {
  int8_t displayCaptureBuffer = kNoBuffer;
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
      collecting.firstVisibleEvent = 0;
      collecting.sequence = sequence;
      captureSourceState(collecting.visibleStartState);
      collecting.state = BufferState::kCollecting;
      displayCaptureBuffer = collectingBuffer;
    }
    critical_section_exit(&frameBufferLock);
    if (displayCaptureBuffer != kNoBuffer) {
      captureSourceDisplay(frameBuffers[displayCaptureBuffer]);
    }
    return;
  }

  if (collectingBuffer != kNoBuffer) {
    RasterFrameBuffer &completed = frameBuffers[collectingBuffer];
    const uint8_t sequenceDelta =
        static_cast<uint8_t>(sequence - completed.sequence);
    if (sequenceDelta == 1) {
      captureSourceState(completed.endState);
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
    collecting.firstVisibleEvent = 0;
    collecting.sequence = sequence;
    captureSourceState(collecting.visibleStartState);
    collecting.state = BufferState::kCollecting;
    displayCaptureBuffer = collectingBuffer;
  }
  critical_section_exit(&frameBufferLock);
  if (displayCaptureBuffer != kNoBuffer) {
    captureSourceDisplay(frameBuffers[displayCaptureBuffer]);
  }
}

void setRasterTimestamp(uint16_t timestamp) {
  currentTimestamp = timestamp;
}

bool queueRasterDisplayWrite(uint16_t address, uint8_t data) {
  size_t screen;
  size_t offset;
  if (!resolveDisplayFileAddress(address, screen, offset) ||
      !sourceTimingActive) {
    return false;
  }

  // Keep a complete source-side image so a later buffer can recover after a
  // marker gap or temporary lack of a free frame buffer.
  machineState.displayFile[screen][offset] = data;

  critical_section_enter_blocking(&frameBufferLock);
  if (collectingBuffer == kNoBuffer) {
    rasterEventCounters.eventOverflows =
        rasterEventCounters.eventOverflows + 1;
    critical_section_exit(&frameBufferLock);
    return true;
  }

  frameBuffers[collectingBuffer].displayFile[screen][offset] = data;
  critical_section_exit(&frameBufferLock);
  return true;
}

bool queueRasterUlaPlusWrite(uint8_t selectedRegister, uint8_t data) {
  return queueRasterEvent(EventType::kUlaPlus, selectedRegister, data);
}

bool queueRasterTimexWrite(uint8_t data) {
  return queueRasterEvent(EventType::kTimex, 0, data);
}

bool queueRasterBorderWrite(uint8_t data) {
  return queueRasterEvent(EventType::kBorder, 0, data);
}

bool queueRasterMemoryControlWrite(uint8_t data) {
  return queueRasterEvent(EventType::kMemoryControl, 0, data);
}

void beginRasterOutputFrame() {
  critical_section_enter_blocking(&frameBufferLock);
  if (playingResetGeneration != resetGeneration &&
      playingBuffer != kNoBuffer) {
    frameBuffers[playingBuffer].count = 0;
    frameBuffers[playingBuffer].state = BufferState::kFree;
    playingBuffer = kNoBuffer;
  }
  playingResetGeneration = resetGeneration;

  const int8_t ready = findOldestReadyBuffer();
  if (ready != kNoBuffer) {
    if (playingBuffer != kNoBuffer) {
      frameBuffers[playingBuffer].count = 0;
      frameBuffers[playingBuffer].state = BufferState::kFree;
    }
    playingBuffer = ready;
    frameBuffers[playingBuffer].state = BufferState::kPlaying;
    playingEvent = frameBuffers[playingBuffer].firstVisibleEvent;
    playingResetGeneration = resetGeneration;
    applySourceState(frameBuffers[playingBuffer].visibleStartState);
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

  // Writes from the final cropped/blank source lines are not visible. Restore
  // their already captured final state in bounded time rather than replaying
  // an arbitrary number of events in the HSTX DMA interrupt.
  RasterFrameBuffer &playing = frameBuffers[playingBuffer];
  if (playingEvent < playing.count) {
    applySourceState(playing.endState);
    playingEvent = playing.count;
  }
}

const uint8_t *rasterDisplayFile(size_t screen) {
  if (playingBuffer != kNoBuffer) {
    return frameBuffers[playingBuffer].displayFile[screen];
  }
  return machineState.displayFile[screen];
}

}  // namespace zx

#include "RasterEvents.h"

#include <pico.h>
#include <pico/sync.h>
#include <string.h>

#include "EmulationState.h"
#include "UlaVideo.h"

namespace zx {
namespace {

constexpr size_t kRasterBufferCount = 4;
constexpr size_t kRasterEventsPerFrame = 4096;
constexpr size_t kOutputLockAttempts = 64;
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
  uint16_t paletteRgb565[kUlaPlusPaletteSize];
  uint8_t modeRegister;
  uint8_t mode;
  TimexState timex;
  uint8_t borderColour;
  uint8_t memoryControl;
};

struct RasterFrameBuffer {
  RasterEvent events[kRasterEventsPerFrame];
  RasterDisplayLine displayLines[kSpectrumHeight];
  RasterVideoState visibleStartState;
  RasterVideoState endState;
  size_t count;
  size_t firstVisibleEvent;
  uint32_t ordinal;
  uint8_t sequence;
  BufferState state;
};

struct RasterPlayback {
  RasterVideoState videoState;
  int8_t buffer;
  size_t event;
};

static_assert((sizeof(RasterVideoState) & 3) == 0,
              "Raster video state must be word aligned");

RasterFrameBuffer frameBuffers[kRasterBufferCount];
critical_section_t frameBufferLock;
int8_t collectingBuffer = 0;
RasterPlayback playingFields[kRasterOutputFieldCount];
uint16_t currentTimestamp = 0;
size_t nextDisplayLine = 0;
uint32_t nextFrameOrdinal = 0;
volatile bool sourceTimingActive = false;
volatile uint32_t resetGeneration = 0;
uint32_t playingResetGeneration = 0;
bool fieldSequenceValid = false;
uint8_t fieldSequenceOrigin = 0;
uint8_t sourcePalette[kUlaPlusPaletteSize] = {};
uint8_t sourceModeRegister = 0x40;
uint8_t sourceMode = 0;
TimexState sourceTimex = {};
uint8_t sourceBorderColour = 0;
uint8_t sourceMemoryControl = 0;

bool __not_in_flash_func(tryEnterFrameBufferLock)() {
  const uint32_t savedInterruptState = save_and_disable_interrupts();
  for (size_t attempt = 0; attempt < kOutputLockAttempts; ++attempt) {
    if (spin_try_lock_unsafe(frameBufferLock.spin_lock)) {
      frameBufferLock.save = savedInterruptState;
      return true;
    }
  }
  restore_interrupts_from_disabled(savedInterruptState);
  return false;
}

int8_t findFreeBuffer() {
  for (size_t i = 0; i < kRasterBufferCount; ++i) {
    if (frameBuffers[i].state == BufferState::kFree) {
      return static_cast<int8_t>(i);
    }
  }
  return kNoBuffer;
}

int8_t __not_in_flash_func(findOldestReadyBuffer)() {
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

bool isPlayingBuffer(size_t buffer) {
  for (size_t field = 0; field < kRasterOutputFieldCount; ++field) {
    if (playingFields[field].buffer == static_cast<int8_t>(buffer)) {
      return true;
    }
  }
  return false;
}

size_t __not_in_flash_func(timestampSourceLine)(uint16_t timestamp) {
  size_t line = timestamp >> kRasterPhaseBits;
  // The first implementation schedules at line boundaries. Preserve the
  // sub-line field on the wire and round a future non-zero phase forward.
  if ((timestamp & kRasterPhaseMask) != 0) {
    ++line;
  }
  return line;
}

size_t __not_in_flash_func(eventSourceLine)(const RasterEvent &event) {
  return timestampSourceLine(event.timestamp);
}

void __not_in_flash_func(applyEvent)(const RasterEvent &event) {
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
    // Decode on core 0, never rebuild 64 colours in the video DMA interrupt.
    state.paletteRgb565[i] = ulaPlusColourRgb565(sourcePalette[i]);
  }
  state.modeRegister = sourceModeRegister;
  state.mode = sourceMode;
  state.timex = sourceTimex;
  state.borderColour = sourceBorderColour;
  state.memoryControl = sourceMemoryControl;
}

void __not_in_flash_func(copyRasterVideoState)(RasterVideoState &destination,
                                                const RasterVideoState &source) {
  volatile uint32_t *output =
      reinterpret_cast<volatile uint32_t *>(&destination);
  const volatile uint32_t *input =
      reinterpret_cast<const volatile uint32_t *>(&source);
  for (size_t i = 0; i < sizeof(RasterVideoState) / sizeof(uint32_t); ++i) {
    output[i] = input[i];
  }
}

void __not_in_flash_func(capturePlaybackState)(RasterVideoState &state) {
  volatile uint8_t *palette = state.palette;
  const volatile uint8_t *machinePalette = machineState.ulaPlus.palette;
  volatile uint16_t *paletteRgb565 = state.paletteRgb565;
  const volatile uint16_t *machinePaletteRgb565 =
      machineState.ulaPlus.paletteRgb565;
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    palette[i] = machinePalette[i];
    paletteRgb565[i] = machinePaletteRgb565[i];
  }
  state.modeRegister = machineState.ulaPlus.modeRegister;
  state.mode = machineState.ulaPlus.mode;
  state.timex = machineState.timex;
  state.borderColour = machineState.ula.borderColour;
  state.memoryControl = machineState.memoryControl;
}

void captureSourceDisplayBefore(size_t sourceLine) {
  if (collectingBuffer == kNoBuffer) {
    return;
  }

  constexpr size_t kFirstDisplayLine = kRasterFirstRenderedLine + kBorderTop;
  const size_t screen = sourceTimex.active
                            ? sourceTimex.screen
                            : ((sourceMemoryControl & 0x08) != 0 ? 1 : 0);
  const uint8_t *display = machineState.displayFile[screen];
  while (nextDisplayLine < kSpectrumHeight &&
         kFirstDisplayLine + nextDisplayLine < sourceLine) {
    const size_t y = nextDisplayLine++;
    const size_t bitmap = ((y & 0xc0) << 5) | ((y & 0x07) << 8) |
                          ((y & 0x38) << 2);
    const size_t attributes = kBitmapSize + (y >> 3) * (kSpectrumWidth / 8);
    RasterDisplayLine &line = frameBuffers[collectingBuffer].displayLines[y];
    for (size_t file = 0; file < 2; ++file) {
      const size_t base = file * kTimexSecondDisplayOffset;
      memcpy(line.bitmap[file], display + base + bitmap,
             sizeof(line.bitmap[file]));
      memcpy(line.attributes[file], display + base + attributes,
             sizeof(line.attributes[file]));
    }
  }
}

void __not_in_flash_func(applySourceState)(const RasterVideoState &state) {
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    machineState.ulaPlus.palette[i] = state.palette[i];
    machineState.ulaPlus.paletteRgb565[i] = state.paletteRgb565[i];
  }
  machineState.ulaPlus.modeRegister = state.modeRegister;
  machineState.ulaPlus.mode = state.mode;
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

void __not_in_flash_func(stopPlayingAfterReset)(size_t field) {
  if (playingResetGeneration != resetGeneration &&
      field < kRasterOutputFieldCount) {
    RasterPlayback &playing = playingFields[field];
    if (playing.buffer != kNoBuffer) {
      playing.event = frameBuffers[playing.buffer].count;
    }
  }
}

void __not_in_flash_func(restartPlayingFields)() {
  if (playingResetGeneration != resetGeneration) {
    return;
  }
  for (size_t field = 0; field < kRasterOutputFieldCount; ++field) {
    RasterPlayback &playing = playingFields[field];
    if (playing.buffer == kNoBuffer) {
      continue;
    }
    const RasterFrameBuffer &frame = frameBuffers[playing.buffer];
    playing.event = frame.firstVisibleEvent;
    copyRasterVideoState(playing.videoState, frame.visibleStartState);
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
  for (size_t field = 0; field < kRasterOutputFieldCount; ++field) {
    playingFields[field].buffer = kNoBuffer;
    playingFields[field].event = 0;
  }
  currentTimestamp = 0;
  nextDisplayLine = 0;
  nextFrameOrdinal = 0;
  sourceTimingActive = false;
  resetGeneration = 0;
  playingResetGeneration = 0;
  fieldSequenceValid = false;
  fieldSequenceOrigin = 0;
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
  critical_section_enter_blocking(&frameBufferLock);
  ++resetGeneration;
  sourceTimingActive = false;
  currentTimestamp = 0;
  nextDisplayLine = 0;
  for (size_t i = 0; i < kUlaPlusPaletteSize; ++i) {
    sourcePalette[i] = 0;
  }
  sourceModeRegister = 0x40;
  sourceMode = 0;
  sourceTimex = {};
  sourceBorderColour = 0;
  sourceMemoryControl = 0;
  fieldSequenceValid = false;
  fieldSequenceOrigin = 0;

  for (size_t i = 0; i < kRasterBufferCount; ++i) {
    if (!isPlayingBuffer(i)) {
      frameBuffers[i].count = 0;
      frameBuffers[i].state = BufferState::kFree;
    }
  }

  collectingBuffer = findFreeBuffer();
  if (collectingBuffer != kNoBuffer) {
    frameBuffers[collectingBuffer].count = 0;
    frameBuffers[collectingBuffer].firstVisibleEvent = 0;
    frameBuffers[collectingBuffer].state = BufferState::kCollecting;
  }
  critical_section_exit(&frameBufferLock);
}

void beginRasterSourceFrame(uint8_t sequence) {
  // Complete the remaining source lines before publishing the frame. This
  // work stays on core 0, outside the video interrupt and cross-core lock.
  if (sourceTimingActive) {
    captureSourceDisplayBefore(kRasterSourceLineCount);
  }
  nextDisplayLine = 0;
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
    }
    critical_section_exit(&frameBufferLock);
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
  }
  critical_section_exit(&frameBufferLock);
}

void setRasterTimestamp(uint16_t timestamp) {
  if (sourceTimingActive) {
    // All writes in the preceding timestamp bucket have now arrived. Fetch
    // its scanline before applying writes belonging to the new bucket.
    captureSourceDisplayBefore(timestampSourceLine(timestamp));
  }
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

  if (collectingBuffer == kNoBuffer) {
    rasterEventCounters.eventOverflows =
        rasterEventCounters.eventOverflows + 1;
  }
  // setRasterTimestamp() captures each completed line from this source RAM.
  // Later erases cannot alter that line, and memory writes need no event slots.
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

void __not_in_flash_func(beginRasterOutputFrame)() {
  // This runs in the core-1 HSTX scanline interrupt. Waiting for core 0 here
  // can miss a DMA deadline and make the HSTX driver restart the HDMI stream.
  // The source and output frame boundaries are both 50 Hz and can repeatedly
  // coincide, so allow a short fixed retry window for the normal handoff. If
  // core 0 is doing longer USB work, keep the current immutable frame instead
  // of waiting without a deadline.
  if (!tryEnterFrameBufferLock()) {
    rasterEventCounters.outputLockMisses =
        rasterEventCounters.outputLockMisses + 1;
    restartPlayingFields();
    return;
  }
  if (playingResetGeneration != resetGeneration) {
    for (size_t field = 0; field < kRasterOutputFieldCount; ++field) {
      RasterPlayback &playing = playingFields[field];
      if (playing.buffer != kNoBuffer) {
        frameBuffers[playing.buffer].count = 0;
        frameBuffers[playing.buffer].state = BufferState::kFree;
        playing.buffer = kNoBuffer;
        playing.event = 0;
      }
    }
  }
  playingResetGeneration = resetGeneration;

  for (;;) {
    const int8_t ready = findOldestReadyBuffer();
    if (ready == kNoBuffer) {
      break;
    }

    RasterFrameBuffer &frame = frameBuffers[ready];
    if (!fieldSequenceValid) {
      fieldSequenceOrigin = frame.sequence;
      fieldSequenceValid = true;
    }
    const size_t field =
        static_cast<uint8_t>(frame.sequence - fieldSequenceOrigin) & 1;
    RasterPlayback &playing = playingFields[field];
    if (playing.buffer != kNoBuffer) {
      frameBuffers[playing.buffer].count = 0;
      frameBuffers[playing.buffer].state = BufferState::kFree;
    }
    playing.buffer = ready;
    playing.event = frame.firstVisibleEvent;
    copyRasterVideoState(playing.videoState, frame.visibleStartState);
    frame.state = BufferState::kPlaying;
    rasterEventCounters.playedFrames = rasterEventCounters.playedFrames + 1;
  }
  critical_section_exit(&frameBufferLock);
  // Held fields must replay their controls from the beginning too.
  restartPlayingFields();
}

bool __not_in_flash_func(applyRasterEventsForOutputLine)(size_t outputLine,
                                                         size_t field) {
  if (field >= kRasterOutputFieldCount) {
    return false;
  }
  stopPlayingAfterReset(field);
  // RAM testing and ESXDOS startup can run before the first IRQ/frame marker.
  // The callback must still render their untimed writes from live screen RAM.
  if (!sourceTimingActive) {
    return true;
  }
  RasterPlayback &playing = playingFields[field];
  if (playing.buffer == kNoBuffer ||
      playingResetGeneration != resetGeneration) {
    return false;
  }

  RasterFrameBuffer &frame = frameBuffers[playing.buffer];
  applySourceState(playing.videoState);

  const size_t sourceLine = kRasterFirstRenderedLine + outputLine;
  bool stateChanged = false;
  while (playing.event < frame.count &&
         eventSourceLine(frame.events[playing.event]) <= sourceLine) {
    applyEvent(frame.events[playing.event++]);
    stateChanged = true;
  }
  if (stateChanged) {
    capturePlaybackState(playing.videoState);
  }
  return true;
}

void __not_in_flash_func(finishRasterOutputLine)(size_t outputLine,
                                                 size_t field) {
  if (field >= kRasterOutputFieldCount) {
    return;
  }
  stopPlayingAfterReset(field);
  RasterPlayback &playing = playingFields[field];
  if (playing.buffer == kNoBuffer ||
      playingResetGeneration != resetGeneration ||
      outputLine + 1 != kVideoHeight) {
    return;
  }

  // Writes from the final cropped/blank source lines are not visible. Restore
  // their already captured final state in bounded time rather than replaying
  // an arbitrary number of events in the HSTX DMA interrupt.
  RasterFrameBuffer &frame = frameBuffers[playing.buffer];
  if (playing.event < frame.count) {
    copyRasterVideoState(playing.videoState, frame.endState);
    applySourceState(playing.videoState);
    playing.event = frame.count;
  }
}

const RasterDisplayLine *__not_in_flash_func(rasterDisplayLine)(size_t y,
                                                                size_t field) {
  if (!sourceTimingActive || playingResetGeneration != resetGeneration) {
    return nullptr;
  }
  if (field < kRasterOutputFieldCount &&
      playingFields[field].buffer != kNoBuffer) {
    return &frameBuffers[playingFields[field].buffer].displayLines[y];
  }
  return nullptr;
}

}  // namespace zx

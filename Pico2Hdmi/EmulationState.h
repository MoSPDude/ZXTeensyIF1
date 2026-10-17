#pragma once

#include <stddef.h>
#include <stdint.h>

namespace zx {

constexpr size_t kBitmapSize = 6144;
constexpr size_t kAttributeSize = 768;
constexpr size_t kDisplayFileSize = kBitmapSize + kAttributeSize;
constexpr size_t kScreenCount = 2;
constexpr size_t kUlaPlusPaletteSize = 64;
constexpr size_t kAyRegisterCount = 16;
constexpr size_t kBeeperTransitionQueueSize = 256;
static_assert((kBeeperTransitionQueueSize &
               (kBeeperTransitionQueueSize - 1)) == 0,
              "Beeper transition queue size must be a power of two");

struct UlaState {
  uint8_t portValue;
  uint8_t borderColour;
  bool micHigh;
  bool earHigh;
  // USB is delivered in bursts, so several ULA writes can be dispatched
  // before another PCM frame is rendered. Keep audio-level changes in order
  // instead of allowing all but the final EAR/MIC state to disappear.
  bool audioMicHigh;
  bool audioEarHigh;
  uint8_t beeperTransitions[kBeeperTransitionQueueSize];
  uint16_t beeperTransitionRead;
  uint16_t beeperTransitionWrite;
  uint16_t beeperTransitionCount;
  uint32_t beeperTransitionOverflows;
  int32_t earFilterFast;
  int32_t earFilterSlow;
  int32_t micFilterFast;
  int32_t micFilterSlow;
  uint32_t beeperFilterPhase;
};

struct UlaPlusState {
  uint8_t selectedRegister;
  uint8_t mode;
  uint8_t palette[kUlaPlusPaletteSize];
};

// AY register state and the synthesis counters advanced by the 44.1 kHz HDMI
// sample producer. They live together so a Spectrum reset is deterministic.
struct AyState {
  uint8_t selectedRegister;
  uint8_t registers[kAyRegisterCount];
  uint32_t tonePhase[3];
  uint32_t noisePhase;
  uint64_t envelopePhase;
  uint32_t noiseLfsr;
  uint8_t toneOutput;
  bool noiseOutput;
  uint8_t envelopeLevel;
  int8_t envelopeDirection;
  bool envelopeHolding;
};

struct MachineState {
  uint8_t displayFile[kScreenCount][kDisplayFileSize];
  uint8_t memoryControl;
  UlaState ula;
  UlaPlusState ulaPlus;
  AyState ay;
};

extern MachineState machineState;

// Restore the Spectrum-facing state to the power-on state described by the
// wire protocol. Protocol framing and diagnostic counters are intentionally
// not part of the emulated machine, so they survive a reset command.
void resetMachineState();

// Apply one decoded write. Returns false only for an address (or reset value)
// that is not part of the documented protocol.
bool applyCommand(uint16_t address, uint8_t data);

}  // namespace zx

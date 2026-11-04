#pragma once

#include <stddef.h>
#include <stdint.h>

namespace zx {

constexpr size_t kBitmapSize = 6144;
constexpr size_t kAttributeSize = 768;
constexpr size_t kDisplayFileSize = kBitmapSize + kAttributeSize;
constexpr size_t kTimexSecondDisplayOffset = 0x2000;
constexpr size_t kDisplayMemorySize =
    kTimexSecondDisplayOffset + kDisplayFileSize;
constexpr size_t kScreenCount = 2;
constexpr size_t kUlaPlusPaletteSize = 64;
constexpr size_t kAyRegisterCount = 16;
constexpr size_t kBeeperSampleQueueSize = 2048;
static_assert((kBeeperSampleQueueSize & (kBeeperSampleQueueSize - 1)) == 0,
              "Beeper sample queue size must be a power of two");

struct UlaState {
  uint8_t portValue;
  uint8_t borderColour;
  bool micHigh;
  bool earHigh;
  // USB is delivered in bursts. Queue changed EAR/MIC levels to preserve their
  // order. The HDMI producer applies at most one per PCM frame and otherwise
  // holds the existing level.
  bool audioMicHigh;
  bool audioEarHigh;
  uint8_t beeperSamples[kBeeperSampleQueueSize];
  uint16_t beeperSampleRead;
  uint16_t beeperSampleWrite;
  uint16_t beeperSampleCount;
  uint32_t beeperSampleOverflows;
  int32_t beeperPreviousInput;
  int32_t beeperHighPass;
  int32_t beeperLowPass;
};

struct UlaPlusState {
  uint8_t selectedRegister;
  uint8_t modeRegister;
  uint8_t mode;
  uint8_t palette[kUlaPlusPaletteSize];
  uint16_t paletteRgb565[kUlaPlusPaletteSize];
};

struct TimexState {
  uint8_t mode;
  uint8_t colour;
  uint8_t screen;
  bool active;
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
  uint8_t displayFile[kScreenCount][kDisplayMemorySize];
  uint8_t memoryControl;
  UlaState ula;
  UlaPlusState ulaPlus;
  TimexState timex;
  AyState ay;
};

extern MachineState machineState;

// Resolve one encoded display-file address into the mirrored screen storage.
// Returns false for protocol addresses that do not represent screen memory.
bool resolveDisplayFileAddress(uint16_t address, size_t &screen,
                               size_t &offset);

// Restore the Spectrum-facing state to the power-on state described by the
// wire protocol. Protocol framing and diagnostic counters are intentionally
// not part of the emulated machine, so they survive a reset command.
void resetMachineState();

// Queue one 44.1 kHz EAR/MIC sample decoded from a command extension byte.
void queueBeeperSample(uint8_t levels);

// Apply one decoded write. Returns false only for an address (or reset value)
// that is not part of the documented protocol.
bool applyCommand(uint16_t address, uint8_t data);

// Apply a resolved ULAplus data-port write. Raster scheduling resolves the
// selected register while parsing, then calls this on the video core at the
// requested source scanline.
void applyUlaPlusRegister(uint8_t selectedRegister, uint8_t data);

// Apply video state resolved by the raster scheduler. Only the low three ULA
// port bits are mirrored for video; EAR and MIC use their separate sampled
// audio path.
void applyBorderColour(uint8_t data);
void applyMemoryControl(uint8_t data);

// Resolve the Timex control port directly or through the optional ULAplus
// mode-register mirror. The latter extends the mode encoding with bank 7.
TimexState decodeTimexControl(uint8_t control, bool ulaPlusModeRegister);
void applyTimexControl(uint8_t control);

}  // namespace zx

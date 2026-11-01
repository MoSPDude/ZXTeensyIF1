#include "EmulationState.h"

#include <string.h>

namespace zx {
namespace {

constexpr uint16_t kScreen0First = 0x0000;
constexpr uint16_t kScreen0Last = 0x1aff;
constexpr uint16_t kMemoryControlAddress = 0x7ffd;
constexpr uint16_t kScreen1First = 0x8000;
constexpr uint16_t kScreen1Last = 0x9aff;
constexpr uint16_t kUlaPlusSelectAddress = 0xbf3b;
constexpr uint16_t kAyDataAddress = 0xbffd;
constexpr uint16_t kUlaPlusDataAddress = 0xff3b;
constexpr uint16_t kAySelectAddress = 0xfffd;
constexpr uint16_t kUlaPortAddress = 0xfffe;
constexpr uint16_t kResetAddress = 0xffff;
constexpr uint8_t kResetData = 0xff;

constexpr uint8_t kUlaPlusModeRegister = 0x40;
constexpr uint8_t kAyEnvelopeShapeRegister = 13;
constexpr uint8_t kMicMask = 0x08;
constexpr uint8_t kEarMask = 0x10;

static_assert(kScreen0Last - kScreen0First + 1 == kDisplayFileSize);
static_assert(kScreen1Last - kScreen1First + 1 == kDisplayFileSize);

void writeUlaPlusData(uint8_t data) {
  const uint8_t selected = machineState.ulaPlus.selectedRegister;
  if (selected < kUlaPlusPaletteSize) {
    machineState.ulaPlus.palette[selected] = data;
  } else if (selected == kUlaPlusModeRegister) {
    machineState.ulaPlus.mode = data;
  }
}

void writeUlaPort(uint8_t data) {
  UlaState &ula = machineState.ula;

  ula.portValue = data;
  ula.borderColour = data & 0x07;
  ula.micHigh = (data & kMicMask) != 0;
  ula.earHigh = (data & kEarMask) != 0;
}

void restartAyEnvelope(uint8_t shape) {
  machineState.ay.envelopePhase = 0;
  machineState.ay.envelopeLevel = (shape & 0x04) != 0 ? 0 : 15;
  machineState.ay.envelopeDirection = (shape & 0x04) != 0 ? 1 : -1;
  machineState.ay.envelopeHolding = false;
}

}  // namespace

MachineState machineState;

void resetMachineState() {
  memset(machineState.displayFile, 0, sizeof(machineState.displayFile));
  machineState.memoryControl = 0;

  machineState.ula.portValue = 0;
  machineState.ula.borderColour = 0;
  machineState.ula.micHigh = false;
  machineState.ula.earHigh = false;
  machineState.ula.audioMicHigh = false;
  machineState.ula.audioEarHigh = false;
  machineState.ula.beeperSampleRead = 0;
  machineState.ula.beeperSampleWrite = 0;
  machineState.ula.beeperSampleCount = 0;
  machineState.ula.beeperSampleOverflows = 0;
  machineState.ula.beeperPreviousInput = 0;
  machineState.ula.beeperHighPass = 0;
  machineState.ula.beeperLowPass = 0;

  machineState.ulaPlus.selectedRegister = 0;
  machineState.ulaPlus.mode = 0;
  memset(machineState.ulaPlus.palette, 0,
         sizeof(machineState.ulaPlus.palette));

  machineState.ay.selectedRegister = 0;
  memset(machineState.ay.registers, 0, sizeof(machineState.ay.registers));
  memset(machineState.ay.tonePhase, 0, sizeof(machineState.ay.tonePhase));
  machineState.ay.noisePhase = 0;
  machineState.ay.envelopePhase = 0;
  machineState.ay.noiseLfsr = 0x1ffff;
  machineState.ay.toneOutput = 0;
  machineState.ay.noiseOutput = true;
  machineState.ay.envelopeLevel = 15;
  machineState.ay.envelopeDirection = -1;
  machineState.ay.envelopeHolding = false;
}

void queueBeeperSample(uint8_t levels) {
  UlaState &ula = machineState.ula;
  if (ula.beeperSampleCount == kBeeperSampleQueueSize) {
    ula.beeperSampleRead =
        (ula.beeperSampleRead + 1) & (kBeeperSampleQueueSize - 1);
    --ula.beeperSampleCount;
    ++ula.beeperSampleOverflows;
  }

  ula.beeperSamples[ula.beeperSampleWrite] = levels;
  ula.beeperSampleWrite =
      (ula.beeperSampleWrite + 1) & (kBeeperSampleQueueSize - 1);
  ++ula.beeperSampleCount;
}

bool applyCommand(uint16_t address, uint8_t data) {
  if (address >= kScreen0First && address <= kScreen0Last) {
    machineState.displayFile[0][address - kScreen0First] = data;
    return true;
  }

  if (address >= kScreen1First && address <= kScreen1Last) {
    machineState.displayFile[1][address - kScreen1First] = data;
    return true;
  }

  switch (address) {
    case kMemoryControlAddress:
      machineState.memoryControl = data;
      return true;

    case kUlaPlusSelectAddress:
      machineState.ulaPlus.selectedRegister = data;
      return true;

    case kAyDataAddress:
      if (machineState.ay.selectedRegister == kAyEnvelopeShapeRegister) {
        // Writing R13 restarts the envelope even when the shape value did not
        // change.
        machineState.ay.registers[kAyEnvelopeShapeRegister] = data;
        restartAyEnvelope(data);
      } else {
        machineState.ay.registers[machineState.ay.selectedRegister] = data;
      }
      return true;

    case kUlaPlusDataAddress:
      writeUlaPlusData(data);
      return true;

    case kAySelectAddress:
      // The AY-3-8912 exposes sixteen registers. Real hardware ignores the
      // upper address bits, so retain only the low nibble.
      machineState.ay.selectedRegister = data & 0x0f;
      return true;

    case kUlaPortAddress:
      writeUlaPort(data);
      return true;

    case kResetAddress:
      if (data != kResetData) {
        return false;
      }
      resetMachineState();
      return true;

    default:
      return false;
  }
}

}  // namespace zx

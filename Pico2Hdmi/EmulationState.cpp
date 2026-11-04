#include "EmulationState.h"

#include <string.h>

namespace zx {
namespace {

constexpr uint16_t kScreen0First = 0x0000;
constexpr uint16_t kScreen0Last = 0x1aff;
constexpr uint16_t kScreen0SecondFirst = 0x2000;
constexpr uint16_t kScreen0SecondLast = 0x3aff;
constexpr uint16_t kMemoryControlAddress = 0x7ffd;
constexpr uint16_t kScreen1First = 0x8000;
constexpr uint16_t kScreen1Last = 0x9aff;
constexpr uint16_t kScreen1SecondFirst = 0xa000;
constexpr uint16_t kScreen1SecondLast = 0xbaff;
constexpr uint16_t kUlaPlusSelectAddress = 0xbf3b;
constexpr uint16_t kAyDataAddress = 0xbffd;
constexpr uint16_t kUlaPlusDataAddress = 0xff3b;
constexpr uint16_t kAySelectAddress = 0xfffd;
constexpr uint16_t kUlaPortAddress = 0xfffe;
constexpr uint16_t kTimexControlAddress = 0xfffb;
constexpr uint16_t kResetAddress = 0xffff;
constexpr uint8_t kResetData = 0xff;

constexpr uint8_t kUlaPlusModeGroup = 0x40;
constexpr uint8_t kUlaPlusGroupMask = 0xc0;
constexpr uint8_t kAyEnvelopeShapeRegister = 13;
constexpr uint8_t kMicMask = 0x08;
constexpr uint8_t kEarMask = 0x10;

static_assert(kScreen0Last - kScreen0First + 1 == kDisplayFileSize);
static_assert(kScreen1Last - kScreen1First + 1 == kDisplayFileSize);
static_assert(kScreen0SecondLast - kScreen0SecondFirst + 1 ==
              kDisplayFileSize);
static_assert(kScreen1SecondLast - kScreen1SecondFirst + 1 ==
              kDisplayFileSize);

constexpr uint8_t expandBlue(uint8_t blue) {
  return static_cast<uint8_t>((blue << 1) |
                              (((blue >> 1) | blue) & 0x01));
}

constexpr uint16_t expandUlaPlusColour(uint8_t colour) {
  const uint8_t green = colour >> 5;
  const uint8_t red = (colour >> 2) & 0x07;
  const uint8_t blue = expandBlue(colour & 0x03);
  const uint16_t red5 = static_cast<uint16_t>((red << 2) | (red >> 1));
  const uint16_t green6 = static_cast<uint16_t>((green << 3) | green);
  const uint16_t blue5 = static_cast<uint16_t>((blue << 2) | (blue >> 1));
  return static_cast<uint16_t>((red5 << 11) | (green6 << 5) | blue5);
}

static_assert(expandUlaPlusColour(0x00) == 0x0000);
static_assert(expandUlaPlusColour(0x03) == 0x001f);
static_assert(expandUlaPlusColour(0x1c) == 0xf800);
static_assert(expandUlaPlusColour(0xe0) == 0x07e0);
static_assert(expandUlaPlusColour(0xff) == 0xffff);

void writeUlaPort(uint8_t data) {
  UlaState &ula = machineState.ula;

  ula.portValue = data;
  applyBorderColour(data);
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

bool resolveDisplayFileAddress(uint16_t address, size_t &screen,
                               size_t &offset) {
  if (address >= kScreen0First && address <= kScreen0Last) {
    screen = 0;
    offset = address - kScreen0First;
    return true;
  }

  if (address >= kScreen1First && address <= kScreen1Last) {
    screen = 1;
    offset = address - kScreen1First;
    return true;
  }

  if (address >= kScreen0SecondFirst && address <= kScreen0SecondLast) {
    screen = 0;
    offset = address - kScreen0First;
    return true;
  }

  if (address >= kScreen1SecondFirst && address <= kScreen1SecondLast) {
    screen = 1;
    offset = address - kScreen1First;
    return true;
  }

  return false;
}

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
  machineState.ulaPlus.modeRegister = kUlaPlusModeGroup;
  machineState.ulaPlus.mode = 0;
  memset(machineState.ulaPlus.palette, 0,
         sizeof(machineState.ulaPlus.palette));
  memset(machineState.ulaPlus.paletteRgb565, 0,
         sizeof(machineState.ulaPlus.paletteRgb565));

  machineState.timex.mode = 0;
  machineState.timex.colour = 0;
  machineState.timex.screen = 0;
  machineState.timex.active = false;

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
  size_t screen;
  size_t offset;
  if (resolveDisplayFileAddress(address, screen, offset)) {
    machineState.displayFile[screen][offset] = data;
    return true;
  }

  switch (address) {
    case kMemoryControlAddress:
      applyMemoryControl(data);
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
      applyUlaPlusRegister(machineState.ulaPlus.selectedRegister, data);
      return true;

    case kAySelectAddress:
      // The AY-3-8912 exposes sixteen registers. Real hardware ignores the
      // upper address bits, so retain only the low nibble.
      machineState.ay.selectedRegister = data & 0x0f;
      return true;

    case kUlaPortAddress:
      writeUlaPort(data);
      return true;

    case kTimexControlAddress:
      applyTimexControl(data);
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

void applyBorderColour(uint8_t data) {
  machineState.ula.borderColour = data & 0x07;
}

void applyMemoryControl(uint8_t data) {
  machineState.memoryControl = data;
}

void applyUlaPlusRegister(uint8_t selectedRegister, uint8_t data) {
  if (selectedRegister < kUlaPlusPaletteSize) {
    machineState.ulaPlus.palette[selectedRegister] = data;
    machineState.ulaPlus.paletteRgb565[selectedRegister] =
        expandUlaPlusColour(data);
  } else if ((selectedRegister & kUlaPlusGroupMask) == kUlaPlusModeGroup) {
    machineState.ulaPlus.modeRegister = selectedRegister;
    machineState.ulaPlus.mode = data;
    machineState.timex = decodeTimexControl(selectedRegister, true);
  }
}

TimexState decodeTimexControl(uint8_t control, bool ulaPlusModeRegister) {
  TimexState state = {};
  state.active = true;
  state.colour = (control >> 3) & 0x07;

  const uint8_t encodedMode = control & 0x07;
  if (ulaPlusModeRegister) {
    switch (encodedMode) {
      case 0x03:
        state.mode = 0x02;
        state.screen = 1;
        break;

      case 0x04:
        state.mode = 0x00;
        state.screen = 1;
        break;

      case 0x05:
        state.mode = 0x01;
        state.screen = 1;
        break;

      case 0x07:
        state.mode = 0x06;
        state.screen = 1;
        break;

      default:
        state.mode = encodedMode;
        state.screen = 0;
        break;
    }
  } else {
    // The original Timex SCLD defines only modes 0, 1, 2 and 6. Treat its
    // documented unpredictable encodings as normal mode rather than showing
    // stale or uninitialized data.
    state.mode = (encodedMode == 0x01 || encodedMode == 0x02 ||
                  encodedMode == 0x06)
                     ? encodedMode
                     : 0x00;
    state.screen = 0;
  }
  return state;
}

void applyTimexControl(uint8_t control) {
  machineState.timex = decodeTimexControl(control, false);
}

}  // namespace zx

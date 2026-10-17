#include "AudioSynth.h"

#include <pico.h>

#include "EmulationState.h"

namespace zx {
namespace {

// The 128K Spectrum clocks its AY-3-8912 at half the 3.5468 MHz CPU clock.
constexpr uint32_t kAyClock = 1773400;
constexpr uint32_t kSpectrumClock = kAyClock * 2;
constexpr uint32_t kToneDivider = 8;
constexpr uint32_t kNoiseDivider = 16;
// One envelope level advances every 16 AY clocks. A complete 16-level ramp
// therefore has the often-quoted AY envelope divisor of 256.
constexpr uint32_t kEnvelopeDivider = 16;

// Logarithmic AY DAC levels adapted from fruit-bat/pico-zxspectrum's
// ZxSpectrumAy.h (MIT). Values are normalized to an 8-bit peak.
// Kept in SRAM with the hot synthesis path, as in pico-zxspectrum.
uint8_t kAyVolume[16] = {
    0, 3, 5, 7, 10, 15, 21, 34,
    41, 65, 91, 114, 144, 181, 216, 255,
};

constexpr int32_t kAyOuterWeight = 16;
constexpr int32_t kAyCentreWeight = 11;
constexpr int32_t kAyCrossfeedWeight = 4;

// fruit-bat/pico-zxspectrum advances its beeper filter every four Z80
// T-states. Preserve that cadence while the current EAR/MIC state is sampled
// at 44.1 kHz; USB transport timing is intentionally outside this prototype.
constexpr uint32_t kBeeperFilterDivider = 4;
constexpr uint32_t kBeeperFilterRate =
    kSpectrumClock / kBeeperFilterDivider;
constexpr int32_t kBeeperLimit = 112;
// Scale the filter's -112..112 result to approximately the previous EAR/MIC
// peak levels while making both signals bipolar around signed PCM zero.
constexpr int32_t kEarScale = 55;
constexpr int32_t kMicScale = 14;
constexpr uint8_t kMicMask = 0x08;
constexpr uint8_t kEarMask = 0x10;

uint16_t tonePeriod(const uint8_t *registers, size_t channel) {
  uint16_t period =
      static_cast<uint16_t>(registers[channel * 2]) |
      (static_cast<uint16_t>(registers[channel * 2 + 1] & 0x0f) << 8);
  return period == 0 ? 1 : period;
}

uint8_t noisePeriod(const uint8_t *registers) {
  const uint8_t period = registers[6] & 0x1f;
  return period == 0 ? 1 : period;
}

uint16_t envelopePeriod(const uint8_t *registers) {
  const uint16_t period = static_cast<uint16_t>(registers[11]) |
                          (static_cast<uint16_t>(registers[12]) << 8);
  return period == 0 ? 1 : period;
}

void advanceTone(AyState &ay, size_t channel) {
  const uint32_t limit =
      kAudioSampleRate * kToneDivider * tonePeriod(ay.registers, channel);
  uint32_t phase = ay.tonePhase[channel] + kAyClock;
  while (phase >= limit) {
    phase -= limit;
    ay.toneOutput ^= static_cast<uint8_t>(1u << channel);
  }
  ay.tonePhase[channel] = phase;
}

void advanceNoise(AyState &ay) {
  const uint32_t limit =
      kAudioSampleRate * kNoiseDivider * noisePeriod(ay.registers);
  uint32_t phase = ay.noisePhase + kAyClock;
  while (phase >= limit) {
    phase -= limit;
    const uint32_t feedback = (ay.noiseLfsr ^ (ay.noiseLfsr >> 3)) & 1;
    ay.noiseLfsr = (ay.noiseLfsr >> 1) | (feedback << 16);
    ay.noiseOutput = (ay.noiseLfsr & 1) != 0;
  }
  ay.noisePhase = phase;
}

void advanceEnvelopeStep(AyState &ay) {
  if (ay.envelopeHolding) {
    return;
  }

  const int next = static_cast<int>(ay.envelopeLevel) +
                   ay.envelopeDirection;
  if (next >= 0 && next <= 15) {
    ay.envelopeLevel = static_cast<uint8_t>(next);
    return;
  }

  const uint8_t shape = ay.registers[13] & 0x0f;
  if ((shape & 0x08) == 0) {
    // CONTINUE clear always decays to zero after the first ramp, including
    // nominal attack shapes 4-7.
    ay.envelopeLevel = 0;
    ay.envelopeHolding = true;
    return;
  }

  if ((shape & 0x01) != 0) {
    if ((shape & 0x02) != 0) {
      ay.envelopeLevel = ay.envelopeDirection > 0 ? 0 : 15;
    }
    ay.envelopeHolding = true;
    return;
  }

  if ((shape & 0x02) != 0) {
    ay.envelopeDirection = -ay.envelopeDirection;
  }
  ay.envelopeLevel = ay.envelopeDirection > 0 ? 0 : 15;
}

void advanceEnvelope(AyState &ay) {
  const uint64_t limit = static_cast<uint64_t>(kAudioSampleRate) *
                         kEnvelopeDivider * envelopePeriod(ay.registers);
  ay.envelopePhase += kAyClock;
  while (ay.envelopePhase >= limit) {
    ay.envelopePhase -= limit;
    advanceEnvelopeStep(ay);
  }
}

int32_t __not_in_flash_func(channelSignal)(const AyState &ay,
                                           size_t channel) {
  const uint8_t volumeRegister = ay.registers[8 + channel];
  const uint8_t level = (volumeRegister & 0x10) != 0
                            ? ay.envelopeLevel
                            : volumeRegister & 0x0f;
  const int32_t amplitude = kAyVolume[level];
  if (amplitude == 0) {
    return 0;
  }

  const uint8_t mixer = ay.registers[7];
  const bool toneEnabled = (mixer & (1u << channel)) == 0;
  const bool noiseEnabled = (mixer & (1u << (channel + 3))) == 0;
  if (!toneEnabled && !noiseEnabled) {
    // A disabled channel produces only DC on the physical chip. Suppressing
    // that DC keeps the signed HDMI stream centred and preserves headroom.
    return 0;
  }

  const bool toneHigh =
      !toneEnabled || (ay.toneOutput & (1u << channel)) != 0;
  const bool noiseHigh = !noiseEnabled || ay.noiseOutput;
  const bool gateHigh = toneHigh && noiseHigh;

  // Tone or noise alone has a 50% duty cycle. With both enabled, the AND gate
  // has a 25% duty cycle, so +3/-1 removes its DC component without a filter.
  if (toneEnabled && noiseEnabled) {
    return gateHigh ? amplitude * 3 : -amplitude;
  }
  return gateHigh ? amplitude : -amplitude;
}

int16_t clampSample(int32_t sample) {
  if (sample > 32767) {
    return 32767;
  }
  if (sample < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(sample);
}

inline void advanceBeeperPole(int32_t &fast, int32_t &slow, bool high) {
  const int32_t drive = high ? 1 : -1;
  fast -= fast >> 6;
  slow -= slow >> 10;
  fast += drive * 95;
  slow += drive * 1024;
}

inline int32_t beeperPoleOutput(int32_t fast, int32_t slow) {
  const int32_t output = (fast >> 4) - (slow >> 12);
  if (output < -kBeeperLimit) {
    return -kBeeperLimit;
  }
  if (output > kBeeperLimit) {
    return kBeeperLimit;
  }
  return output;
}

int32_t __not_in_flash_func(renderBeeper)(UlaState &ula) {
  // Retain edge order without allowing a USB burst to collapse several ULA
  // changes into one state. One transition per PCM frame bounds the work and
  // avoids stretching the audio service enough to drain the HSTX queue.
  if (ula.beeperTransitionCount != 0) {
    const uint8_t levels = ula.beeperTransitions[ula.beeperTransitionRead];
    ula.beeperTransitionRead =
        (ula.beeperTransitionRead + 1) &
        (kBeeperTransitionQueueSize - 1);
    --ula.beeperTransitionCount;
    ula.audioMicHigh = (levels & kMicMask) != 0;
    ula.audioEarHigh = (levels & kEarMask) != 0;
  }

  ula.beeperFilterPhase += kBeeperFilterRate;
  while (ula.beeperFilterPhase >= kAudioSampleRate) {
    ula.beeperFilterPhase -= kAudioSampleRate;
    advanceBeeperPole(ula.earFilterFast, ula.earFilterSlow,
                      ula.audioEarHigh);
    advanceBeeperPole(ula.micFilterFast, ula.micFilterSlow,
                      ula.audioMicHigh);
  }

  return beeperPoleOutput(ula.earFilterFast, ula.earFilterSlow) *
             kEarScale +
         beeperPoleOutput(ula.micFilterFast, ula.micFilterSlow) *
             kMicScale;
}

}  // namespace

StereoSample __not_in_flash_func(renderAudioSample)() {
  AyState &ay = machineState.ay;

  for (size_t channel = 0; channel < 3; ++channel) {
    advanceTone(ay, channel);
  }
  advanceNoise(ay);
  advanceEnvelope(ay);

  const int32_t channelA = channelSignal(ay, 0);
  const int32_t channelB = channelSignal(ay, 1);
  const int32_t channelC = channelSignal(ay, 2);

  // ABC stereo: A is left, B is centred and C is right, with modest
  // crossfeed so no channel disappears when only one speaker is available.
  int32_t left = channelA * kAyOuterWeight +
                 channelB * kAyCentreWeight +
                 channelC * kAyCrossfeedWeight;
  int32_t right = channelA * kAyCrossfeedWeight +
                  channelB * kAyCentreWeight +
                  channelC * kAyOuterWeight;

  // The two-pole response softens the ideal ULA edges and removes the
  // unipolar offset from the HDMI mix. EAR remains roughly four times louder
  // than MIC, matching the previous level relationship.
  const int32_t beeper = renderBeeper(machineState.ula);
  left += beeper;
  right += beeper;

  return {clampSample(left), clampSample(right)};
}

}  // namespace zx

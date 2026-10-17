#pragma once

#include <stdint.h>

namespace zx {

constexpr uint32_t kAudioSampleRate = 44100;

struct StereoSample {
  int16_t left;
  int16_t right;
};

// Produce one 44.1 kHz signed PCM frame and advance the AY tone, noise and
// envelope generators. The caller owns serialization with reset/register
// writes; in the firmware both run cooperatively on core 0.
StereoSample renderAudioSample();

}  // namespace zx

#include <Arduino.h>

#include <Adafruit_TinyUSB.h>
#define DVHSTX_NO_DEFAULT_PINOUT_WARNING
#include <Adafruit_dvhstx.h>

#include "AudioSynth.h"
#include "CommandProtocol.h"
#include "EmulationState.h"
#include "RasterEvents.h"
#include "UlaVideo.h"

#ifndef USE_TINYUSB
#error "Select Adafruit TinyUSB as the Arduino USB stack"
#endif

#if !defined(ARDUINO_RASPBERRY_PI_PICO_2)
#error "Select Raspberry Pi Pico 2 as the Arduino board"
#endif

namespace {

constexpr size_t kAudioServiceBatch = 64;  // Must be a multiple of four.
constexpr size_t kAudioQueueTargetFrames = 256;  // About 5.8 ms at 44.1 kHz.
constexpr size_t kUsbPacketSize = 64;
constexpr size_t kUsbPacketsPerService = 4;
constexpr size_t kUsbStagingBufferSize = 4096;  // Must be a power of two.
static_assert((kUsbStagingBufferSize & (kUsbStagingBufferSize - 1)) == 0,
              "USB staging buffer size must be a power of two");
static_assert((kUsbStagingBufferSize % kUsbPacketSize) == 0,
              "USB staging buffer must hold complete packets");
static_assert((kAudioServiceBatch & 3) == 0,
              "HSTX audio is queued in four-frame packets");
static_assert((kAudioQueueTargetFrames & 3) == 0,
              "HSTX audio target must contain whole four-frame packets");
constexpr uint32_t kCommandLedPulseMs = 30;

DVHSTXPinout pinConfig = ADAFRUIT_HSTXDVIBELL_CFG;
DVHSTXScanline display(pinConfig, DVHSTX_RESOLUTION_360x288);

int16_t audioBuffer[kAudioServiceBatch * 2];
size_t audioQueueCapacityFrames = 0;
uint32_t scanlineCache[zx::kVideoWidth];
uint32_t displayedCommandCount = 0;
uint32_t commandLedOffAt = 0;
bool commandLedOn = false;
uint8_t usbStagingBuffer[kUsbStagingBufferSize];
size_t usbStagingHead = 0;
size_t usbStagingTail = 0;
size_t usbStagingCount = 0;
volatile bool machineStateReady = false;
volatile int8_t displayStartupStatus = 0;

// The HSTX scanline callback supplies a complete 720-pixel RGB565 line. The
// The ULA renderer writes two packed 16-bit output pixels in each 32-bit word;
// they are duplicates in 256-pixel modes and distinct in Timex hi-res. Render
// each logical row once, then copy it into both physical output lines.
static void __not_in_flash_func(drawVideoScanline)(uint32_t vScanline,
                                                    uint32_t activeLine,
                                                    uint32_t *destination) {
  (void)vScanline;
  if ((activeLine & 1) == 0) {
    const size_t logicalLine = activeLine >> 1;
    if (logicalLine == 0) {
      zx::beginRasterOutputFrame();
    }
    zx::applyRasterEventsForOutputLine(logicalLine);
    zx::renderUlaScanline(scanlineCache, logicalLine,
                          display.getFrameCount());
    zx::finishRasterOutputLine(logicalLine);
  }
  for (size_t x = 0; x < zx::kVideoWidth; ++x) {
    destination[x] = scanlineCache[x];
  }
}

size_t serviceAudio() {
  const size_t available = display.audioAvailableForWrite();
  if (audioQueueCapacityFrames == 0 || available > audioQueueCapacityFrames) {
    return 0;
  }

  const size_t queued = audioQueueCapacityFrames - available;
  if (queued >= kAudioQueueTargetFrames) {
    return 0;
  }

  size_t count = kAudioQueueTargetFrames - queued;
  if (count > available) {
    count = available;
  }
  if (count > kAudioServiceBatch) {
    count = kAudioServiceBatch;
  }
  count &= ~static_cast<size_t>(3);
  if (count == 0) {
    return 0;
  }

  for (size_t i = 0; i < count; ++i) {
    const zx::StereoSample sample = zx::renderAudioSample();
    audioBuffer[i * 2] = sample.left;
    audioBuffer[i * 2 + 1] = sample.right;
  }

  // Only this function produces audio. Space can increase while samples are
  // rendered, so audioWrite() accepts every frame advertised above.
  return display.audioWrite(audioBuffer, count);
}

void primeAudio() {
  while (serviceAudio() != 0) {
  }
}

void drainUsbInput() {
  const int available = Serial.available();
  if (available <= 0) {
    return;
  }

  size_t remaining = static_cast<size_t>(available);
  size_t free = kUsbStagingBufferSize - usbStagingCount;

  // Leave excess data in TinyUSB so its endpoint applies USB backpressure;
  // never discard a byte merely because the application staging ring is full.
  if (remaining > free) {
    remaining = free;
  }
  if (remaining == 0) {
    return;
  }

  while (remaining != 0) {
    size_t contiguous = kUsbStagingBufferSize - usbStagingHead;
    if (contiguous > remaining) {
      contiguous = remaining;
    }
    const size_t received =
        Serial.read(usbStagingBuffer + usbStagingHead, contiguous);
    if (received == 0) {
      break;
    }

    usbStagingHead =
        (usbStagingHead + received) & (kUsbStagingBufferSize - 1);
    usbStagingCount += received;
    remaining -= received;
    if (received < contiguous) {
      break;
    }
  }
}

void serviceUsbCommands() {
  // The experimental Teensy sender pads every transmission to one complete
  // full-speed CDC packet. Retain a partial packet until all 64 bytes arrive.
  // Bound each pass so a burst cannot starve HDMI audio generation or contend
  // with the core-1 scanline callback for an unbounded interval.
  size_t packets = 0;
  while (usbStagingCount >= kUsbPacketSize &&
         packets < kUsbPacketsPerService) {
    // The staging buffer and tail are packet-aligned, so a packet never wraps.
    zx::commandParser.pushPacket(usbStagingBuffer + usbStagingTail,
                                 kUsbPacketSize);
    usbStagingTail =
        (usbStagingTail + kUsbPacketSize) & (kUsbStagingBufferSize - 1);
    usbStagingCount -= kUsbPacketSize;
    ++packets;
  }
}

void updateCommandLed() {
  const uint32_t completedCommands = zx::protocolCounters.completedCommands;
  if (completedCommands != displayedCommandCount) {
    displayedCommandCount = completedCommands;
    digitalWrite(LED_BUILTIN, HIGH);
    commandLedOn = true;
    commandLedOffAt = millis() + kCommandLedPulseMs;
  }

  // Signed subtraction keeps the deadline comparison correct across the
  // millis() counter wrapping roughly every 49 days.
  if (commandLedOn &&
      static_cast<int32_t>(millis() - commandLedOffAt) >= 0) {
    digitalWrite(LED_BUILTIN, LOW);
    commandLedOn = false;
  }
}

}  // namespace

// Give core 1 an independent 8 KiB stack. HSTX initialization and the video
// DMA interrupt stay on that core, leaving TinyUSB and command handling on
// core 0.
bool core1_separate_stack = true;

void setup() {
  zx::resetMachineState();
  zx::initializeRasterEvents();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  __dmb();
  machineStateReady = true;

  while (displayStartupStatus == 0) {
    tight_loop_contents();
  }
  __dmb();
  if (displayStartupStatus < 0) {
    panic("Unable to start 720x576p50 HSTX video and audio");
  }
}

void setup1() {
  while (!machineStateReady) {
    tight_loop_contents();
  }
  __dmb();

  display.enableAudio(zx::kAudioSampleRate);
  if (!display.begin(drawVideoScanline) ||
      display.width() != zx::kOutputWidth ||
      display.height() != zx::kOutputHeight) {
    displayStartupStatus = -1;
    return;
  }
  audioQueueCapacityFrames = display.audioAvailableForWrite();
  primeAudio();

  __dmb();
  displayStartupStatus = 1;
}

void loop1() {
  // HSTX scanlines are produced by the core-1 DMA interrupt.
  tight_loop_contents();
}

void loop() {
  // Core 1 handles HSTX scanline interrupts. Core 0 keeps HDMI audio queued
  // while draining and applying every USB command already staged in RAM.
  serviceAudio();
  drainUsbInput();
  serviceAudio();
  serviceUsbCommands();
  serviceAudio();
  updateCommandLed();
}

# Pico2Hdmi

Arduino firmware for a Raspberry Pi Pico 2 and Adafruit PiCowBell HSTX DVI
Output. It receives ZX Spectrum ULA, ULAplus, Timex-video, and AY writes from the
ZXTeensyIF1 over USB CDC and produces 720x576p50 HDMI video with 44.1 kHz
stereo audio.

## USB command protocol

USB CDC is a byte stream. The experimental Teensy sender always transmits a
complete 64-byte packet. After its commands and incremental runs, one
`FF FF 00 00` header terminates the useful packet contents. The Pico discards
that header and every remaining filler byte; when fewer than four bytes remain,
it discards those bytes directly. Audio-bearing no-ops have a nonzero extension
byte and do not terminate the packet. The Pico waits for all 64 bytes before
parsing a packet. Every message begins with:

| Byte | Meaning |
| --- | --- |
| 0 | Encoded address bits 7-0 |
| 1 | Encoded address bits 15-8 |
| 2 | Write data |
| 3 | Status/extension flags |

Decode a command as:

```text
address = byte0 | (byte1 << 8)
data    = byte2
flags   = byte3
```

The extension bits are:

| Bit | Meaning |
| ---: | --- |
| 0 | Start an incremental-address run |
| 1 | Legacy EAR/MIC level sample in bits 3-4; new senders use PCM blocks |
| 2 | Reserved; currently ignored |
| 3 | MIC level for the beeper sample; matches ULA-port bit 3 |
| 4 | EAR level for the beeper sample; matches ULA-port bit 4 |
| 5-7 | Reserved; currently ignored |

With bit 0 clear, the message is complete. For example, `10 00 AA 00` writes
`0xAA` to encoded address `0x0010`.

With bit 0 set, bytes following the message write to successive addresses. The
sequence `B6 E0` ends the run. A data byte of `B6` is escaped as `B6 B6`.
For example, `10 00 AA 01 FF B6 B6 CC B6 E0` writes `AA FF B6 CC` to addresses
`0x0010` through `0x0013`. Runs always terminate within their 64-byte packet.

The Teensy sends unsigned 8-bit beeper PCM in blocks. Header `FA FF nn 00`
is followed by `nn` raw sample bytes (1-60), with no escaping. Every byte value,
including `B6` and `FF`, is valid PCM. The block must fit in its 64-byte packet;
normal senders use 60 samples, filling the packet exactly. Short blocks can be
followed by ordinary commands or the packet terminator. Zero/oversized lengths,
nonzero flags, and blocks extending beyond their packet are framing errors.

Every sample represents one 44.1 kHz period, including unchanged values. Zero
means EAR and MIC both low; 255 means both high. The Teensy applies a 55:14
EAR/MIC weighting while averaging measured high time between foreground
snapshots. Its 256-byte sample ring holds about 5.8 ms; full 60-sample packets
carry audio at 47,040 bytes/s, including their headers.

The Pico reuses its 2048-byte beeper ring, starts after 120 samples (about
2.7 ms) have arrived, and consumes one sample per HDMI PCM frame. If it runs
dry, it holds the previous level and refills before restarting. Debugger-visible
`beeperSampleUnderflows` and `beeperSampleOverflows` count these events.

Legacy extension samples remain supported: audio-only headers are
`FF FF 00 02`, `FF FF 00 0A`, `FF FF 00 12`, or `FF FF 00 1A`.
They are converted to the same weighted PCM range and applied without prefill.
New PCM-block senders require the updated Pico firmware.

| Encoded address | Size | Meaning |
| --- | ---: | --- |
| `0x0000`-`0x1AFF` | `0x1B00` | Screen 0 bitmap and attributes |
| `0x2000`-`0x3AFF` | `0x1B00` | Screen 0 second Timex display file |
| `0x7FFD` | 1 | Complete 128K memory-control value; bit 3 selects the screen |
| `0x8000`-`0x9AFF` | `0x1B00` | Screen 1/shadow bitmap and attributes |
| `0xA000`-`0xBAFF` | `0x1B00` | Screen 1/shadow second Timex display file |
| `0xBF3B` | 1 | ULAplus register select |
| `0xBFFD` | 1 | AY data for the selected register |
| `0xFC00`-`0xFCFF` | 1 | Raster timestamp metadata; address low byte is timestamp bits 7-0 and data is bits 15-8 |
| `0xFFFA` | 1-60 | Beeper PCM block; data is the following raw sample count, flags must be zero |
| `0xFFFB` | 1 | Timex video-control port `FF` |
| `0xFFFC` | 1 | Source-frame marker; data is an 8-bit wrapping frame sequence |
| `0xFF3B` | 1 | ULAplus data |
| `0xFFFD` | 1 | AY register select |
| `0xFFFE` | 1 | ULA border colour in bits 2-0; EAR/MIC use separate PCM blocks |
| `0xFFFF`, data `0x00` | 1 | Audio-only no-op |
| `0xFFFF`, data `0xFF` | 1 | Reset notification |

The reset message is `FF FF FF 00`. It clears all display files and restores
the ULA, ULAplus, Timex, AY, and beeper state without resetting protocol
diagnostics or discarding bytes after that message.

The 16-bit raster timestamp uses bits 15-7 for a source scanline and bits 6-0
for a reserved position within that line. A timestamp is stateful and applies
to following screen-memory, ULAplus data, Timex control, ULA border, or 128K
memory-control writes until another timestamp or frame marker arrives. All
writes in one source scanline share a timestamp. A new timestamp ends any
incremental screen-write run. For example, a timestamp value of
`0x3200` means source line 100, phase zero, and is encoded as the message
`00 FC 32 00`.

ULAplus implements the official 64-entry `GGGRRRBB` palette. While mode bit 0
is set, attribute bits 7-6 choose one of four 16-colour CLUTs, FLASH animation
is disabled, and border colours use paper entries 8-15 in the first CLUT.
Reading `FF3B` is handled synchronously by a palette/mode mirror on the Teensy;
the register port `BF3B` remains write-only. The selected ULAplus mode-group
sub-register mirrors Timex mode and colour selection, including the bank-7
extensions from the version 1.1 specification.

Timex port `FF` modes `0`, `1`, `2`, and `6` select the primary display file,
second display file, 256x192 hi-colour, and 512x192 hi-res respectively.
Hi-colour takes each 8x1 attribute byte from the corresponding offset in the
second display file. Hi-res alternates eight-pixel columns from the two bitmap
files and uses bits 5-3 for complementary ink/paper colours; with ULAplus on,
those colours use palette entries 24-31. On configurations with the VTX modem
enabled, the modem retains ownership of port `FF`; the ULAplus mode-group mirror
remains available for Timex-mode selection.

## Hardware

Adafruit DVI HSTX supplies the PiCowBell mapping as
`ADAFRUIT_HSTXDVIBELL_CFG`:

| HDMI signal | RP2350 GPIO |
| --- | --- |
| TMDS clock | 14 (+), 15 (-) |
| TMDS data 0 | 12 (+), 13 (-) |
| TMDS data 1 | 18 (+), 19 (-) |
| TMDS data 2 | 16 (+), 17 (-) |

The HSTX library owns DMA channels 0-2 for video and channel 3 when HDMI audio
is enabled. HSTX is initialized on Arduino core 1, so its DMA interrupt and
scanline renderer cannot delay TinyUSB command reception on core 0.

In the intended system, the Spectrum powers the ZXTeensyIF1 and the Teensy's
USB host connection powers the Pico 2/Cowbell assembly. Check the power
arrangement before attaching a separate development USB supply.

## Video and audio path

`DVHSTX_RESOLUTION_360x288` selects the CEA 720x576p50 output timing. The HSTX
DMA interrupt requests complete 720-pixel RGB565 lines from `drawVideoScanline`.
The ULA renderer works at 360x288. Its 256-pixel modes write each logical pixel
twice into one packed 32-bit word; Timex hi-res instead packs two distinct
pixels per word for a native 512-pixel active area. Consecutive Spectrum frames
are woven into alternate physical HDMI lines: the first completed source frame
occupies lines 0, 2, 4 and so on, and the next occupies lines 1, 3, 5 and so on.
The HDMI timing remains progressive, but two 50 Hz source frames form one
complete interlaced image.

The Timex hi-res renderer expands each bitmap byte with four packed RGB565
pixel-pair lookups and fills only the side borders on active lines. This keeps
the per-line work down when weaving requires a new 512-pixel render on every
physical HDMI line. The output timing remains 720x576p50 in all video modes.

The scanline path has no RGB framebuffer. Core 0 reconstructs each source
scanline from timestamped screen-memory writes and saves its bitmap and
attribute bytes. Completed lines are immutable: restoring a sprite's background
after its line has been fetched cannot erase it from that frame. The collected
lines are selected together with that frame's raster events at an HDMI frame
boundary, without copying a whole display file in the HSTX DMA interrupt.

Screen-memory, ULAplus palette/mode, Timex control, ULA border, and 128K
memory-control writes use source-frame raster timestamps. The Teensy records
timing when capturing each bus write, detects the Z80 maskable-
interrupt acknowledge cycle (`M1` and `IORQ` active), divides the measured
frame into 312 source lines, and sends a frame marker plus a timestamp whenever
the source scanline changes. Consecutive screen addresses still form incremental
runs within each timestamp bucket.

The Pico buffers up to 4096 video-control events plus 192 reconstructed display
lines in each of four source-frame buffers. Two buffers retain the even and odd
output fields while the remaining buffers collect and hand off incoming source
frames. Each display line holds both Timex files' bitmap and attribute bytes
from the screen selected at that source line. Screen writes update the source
RAM image directly and consume no event slots, so full-screen redraws cannot
exhaust the control-event buffer. The source RAM image persists between frames,
including while a frame is dropped.
Repeated writes to the same video-state target at one timestamp are coalesced
to their final value. State changes in the cropped lines before and after the
visible area are captured as bounded state snapshots instead of being replayed
as an unbounded event burst inside the HSTX DMA interrupt.

Source lines 16-303 map to the 288 logical HDMI lines; writes in the cropped lines
are applied before or after the visible area. The seven low timestamp bits reserve
sub-line precision, but this version deliberately rounds events to scanline
boundaries. All writes in a line are applied before capturing that line when
the next timestamp arrives; the next frame marker completes any remaining
lines. Writes racing the ULA within one scanline are therefore approximate.
Each buffered frame also carries its starting palette, Timex mode, border
colour, and 128K memory-control value. If a marker or whole frame is lost, the
Pico discards ambiguous raster timing and restores persistent video state from
the next complete frame instead of replaying a multi-frame event list as one
frame.

Before the first source-frame marker, both HDMI fields render the live mirrored
screen RAM. This displays Spectrum RAM testing and ESXDOS initialisation while
the Spectrum has interrupts disabled. A reset returns to this untimed display
without waiting for a new interrupt acknowledge. Once source timing starts,
completed frames are woven into their normal even/odd fields.

The acknowledge cycle is a software-only frame reference rather than the
ULA's physical vertical-sync edge. It is absent while the Spectrum has
maskable interrupts disabled, and other hardware sharing `INT` can produce an
acknowledge. The sender rejects acknowledgements that are too close together
or do not fit the measured frame phase. Debugger-visible counters record
accepted and rejected acknowledgements, event overflow, and dropped frames.
After a long gap it accepts a fresh reference, allowing recovery from a wrap
of the Teensy's 32-bit cycle counter.

The Teensy advances a predicted raster clock while interrupt acknowledgements
are absent. At the end of each Z80 read cycle, a lightweight ISR tick closes
any elapsed source frame with a synthetic marker. This also runs during normal
RAM reads and HALT fetch cycles, without adding another producer to the CDC
ring buffer. A sequence jump represents multiple elapsed frames. When real
acknowledgements resume, their sequence is reconciled with the predicted
markers so an already represented frame is not marked twice.

After a frame containing timed video writes is closed, the Teensy also closes
two following quiet frames. These settling frames republish the persistent
final screen RAM into both output fields after a display line was captured
partway through an update, while still preserving the preceding frame's
raster-timed sprite contents.

Adafruit DVI HSTX inserts HDMI data islands and accepts interleaved signed
16-bit stereo frames through `audioWrite()`. The foreground loop keeps a
256-frame queue target, about 5.8 ms at 44.1 kHz, fed in bounded batches. USB
command parsing is limited to four packets per foreground pass so sustained
video or beeper traffic cannot starve audio generation. Audio synthesis stays
on core 0 with USB command application, while HSTX consumes the encoded audio
queue from its core-1 interrupt.

At each HDMI frame boundary, the core-1 scanline interrupt uses a short, fixed
retry window to assign completed raster buffers to alternating output fields.
This permits the 50 Hz source-frame and output-frame handoffs to coincide
normally. Each field independently replays its palette, mode, border and screen
selection events. If core 0 is still updating the buffer state when that window
expires, core 1 keeps both current immutable fields and retries at the next
boundary. It never waits without a deadline, because delaying the scanline DMA
callback can force the HSTX driver to restart the HDMI stream.

The entire sketch scanline callback path, including raster playback, state
application, Timex decoding, and normal/BRIGHT colour tables, resides in SRAM.
This prevents USB/audio processing on core 0 from evicting code or lookup data
needed by the core-1 video interrupt from the shared flash cache. Core 0 also
prepares RGB565 palette snapshots: the video interrupt copies prepared colours
at frame boundaries instead of making 64 palette-conversion calls. These
snapshots add 768 bytes across the three raster buffers.

Debugger-visible `hdmiTimingCounters` records the longest scanline callback in
microseconds, callbacks lasting at least the 32 us physical line period, and
the HSTX audio-queue underrun count. Callback time excludes DMA handler work
outside the sketch and time waiting to enter the interrupt, so a value below
32 us does not by itself prove that every DMA deadline was met.

The ULA beeper uses resistor-weighted unipolar EAR and MIC levels followed by a
fixed-point 160 Hz high-pass and 10 kHz low-pass at the HDMI sample rate. This
approximates the AC-coupled grey +2 SOUND output: a held ULA level decays to
silence, while edges and audible square-wave content pass without the previous
per-source hard clipping.

On each EAR/MIC transition, the Teensy GPIO ISR adds the elapsed CPU cycles to
two unsigned high-time counters and publishes the new level and timestamp.
This path has no division, multiplication, modulo, loops, or sample-buffer work.
Foreground code takes coherent snapshots without masking GPIO interrupts,
includes the unfinished held level, normalizes to 8-bit PCM, and queues samples
independently of USB availability. Unsigned differences handle cycle/counter wrap.

This pulse-area averaging reduces point-sampling aliasing but is not a steep
anti-alias filter. Snapshots have foreground scheduling jitter. If a snapshot
spans multiple sample periods, its average is repeated for those periods;
individual missed boundaries cannot be recovered without edge history.
`hdmiCdcAudioLateSamples` counts those extra periods, while
`hdmiCdcAudioOverflows` counts samples discarded when the 256-byte transmit ring
fills. Oldest samples are discarded to bound latency after a long USB stall.

## Building

Prerequisites:

- Arduino CLI or Arduino IDE 2.x.
- Earle Philhower's Arduino-Pico core 6.3.0.
- Adafruit DVI HSTX 2.1.0 and its Adafruit GFX dependency.
- Board: **Raspberry Pi Pico 2**, ARM architecture.
- CPU Speed menu: **150 MHz**. Adafruit DVI HSTX requires this build setting and
  configures the runtime clock itself.
- USB stack: **Adafruit TinyUSB**.
- Flash: **4MB (no FS)**.

Install the display library through Arduino CLI if needed:

```powershell
arduino-cli lib install "Adafruit DVI HSTX@2.1.0"
```

From the ZXTeensyIF1 repository root:

```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --clean `
  --fqbn "rp2040:rp2040:rpipico2:arch=arm,freq=150,usbstack=tinyusb,flash=4194304_0" `
  --build-path "build/Pico2Hdmi" `
  "Pico2Hdmi"
```

The flashable artifact is `build/Pico2Hdmi/Pico2Hdmi.ino.uf2`.

Adafruit DVI HSTX 2.1.0 emits a default-pinout warning while compiling its own
source even though this sketch explicitly uses `ADAFRUIT_HSTXDVIBELL_CFG`. The
warning does not affect the selected Cowbell mapping or the build.

## Upstream

- [Adafruit DVI HSTX](https://github.com/adafruit/Adafruit-DVI-HSTX)
- [Adafruit PiCowBell HSTX DVI Output guide](https://learn.adafruit.com/adafruit-picowbell-hstx-dvi-output)
- [fruit-bat/pico-zxspectrum](https://github.com/fruit-bat/pico-zxspectrum)

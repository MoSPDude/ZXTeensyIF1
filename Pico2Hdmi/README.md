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
| 1 | Bits 3-4 contain a changed beeper level sampled on the 44.1 kHz clock |
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

The Teensy samples EAR and MIC at 44.1 kHz but transmits a level only when it
differs from the last transmitted level. When a bus command is waiting, that
command and the changed level share the header. Otherwise `FF FF 00` is used as
an audio-only no-op command. The Pico ignores that address/data pair while
queuing the changed level encoded in the extension byte. It applies at most one
queued change per PCM frame and holds the existing level when the queue is
empty. Audio-only headers therefore use `FF FF 00 02`, `FF FF 00 0A`,
`FF FF 00 12`, or `FF FF 00 1A` for the four EAR/MIC levels.
An audio-bearing bus command may also set bit 0 and begin an
incremental-address run.

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
| `0xFFFB` | 1 | Timex video-control port `FF` |
| `0xFFFC` | 1 | Source-frame marker; data is an 8-bit wrapping frame sequence |
| `0xFF3B` | 1 | ULAplus data |
| `0xFFFD` | 1 | AY register select |
| `0xFFFE` | 1 | ULA border colour in bits 2-0; EAR/MIC use command extension bits |
| `0xFFFF`, data `0x00` | 1 | Audio-only no-op |
| `0xFFFF`, data `0xFF` | 1 | Reset notification |

The reset message is `FF FF FF 00`. It clears all display files and restores
the ULA, ULAplus, Timex, AY, and beeper state without resetting protocol
diagnostics or discarding bytes after that message.

The 16-bit raster timestamp uses bits 15-7 for a source scanline and bits 6-0
for a reserved position within that line. A timestamp is stateful and applies
to following ULAplus data, Timex control, ULA border, or 128K memory-control
writes until another timestamp arrives. For example, a timestamp value of
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
pixels per word for a native 512-pixel active area. Each logical line is
rendered once into a small cache and copied to the two physical lines requested
by HSTX.

The scanline path has no framebuffer. It reads the current emulated display
state while the foreground loop applies USB commands, so a state change may
become visible part-way through a frame. This is the same deliberately relaxed
tearing policy used by the first implementation.

ULAplus palette/mode, Timex control, ULA border, and 128K memory-control writes
use source-frame raster timestamps. The Teensy detects the Z80 maskable-
interrupt acknowledge cycle (`M1` and `IORQ` active), divides the measured
frame into 312 source lines, and sends a frame marker plus a timestamp before
each timed video-control write.

The Pico buffers up to 4096 video events in each of three source-frame buffers
and replays a completed frame one HDMI frame later.
Repeated writes to the same video-state target at one timestamp are coalesced
to their final value. State changes in the cropped lines before and after the
visible area are captured as bounded state snapshots instead of being replayed
as an unbounded event burst inside the HSTX DMA interrupt.

Source lines 16-303 map to the 288 logical HDMI lines; writes in the cropped lines
are applied before or after the visible area. The seven low timestamp bits reserve
sub-line precision, but this version deliberately rounds events to scanline boundaries.
Each buffered frame also carries its starting palette, Timex mode, border
colour, and 128K memory-control value. If a marker or whole frame is lost, the
Pico discards ambiguous raster timing and restores persistent video state from
the next complete frame instead of replaying a multi-frame event list as one
frame.

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

Adafruit DVI HSTX inserts HDMI data islands and accepts interleaved signed
16-bit stereo frames through `audioWrite()`. The foreground loop keeps its queue
fed in bounded batches at 44.1 kHz. Audio synthesis stays on core 0 with USB
command application, while HSTX consumes the encoded audio queue from its
core-1 interrupt.

The ULA beeper uses resistor-weighted unipolar EAR and MIC levels followed by a
fixed-point 160 Hz high-pass and 10 kHz low-pass at the HDMI sample rate. This
approximates the AC-coupled grey +2 SOUND output: a held ULA level decays to
silence, while edges and audible square-wave content pass without the previous
per-source hard clipping.

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
arduino-cli compile `
  --fqbn "rp2040:rp2040:rpipico2:arch=arm,freq=150,usbstack=tinyusb,flash=4194304_0" `
  --output-dir "Pico2Hdmi/build" `
  "Pico2Hdmi"
```

The flashable artifact is `Pico2Hdmi/build/Pico2Hdmi.ino.uf2`.

Adafruit DVI HSTX 2.1.0 emits a default-pinout warning while compiling its own
source even though this sketch explicitly uses `ADAFRUIT_HSTXDVIBELL_CFG`. The
warning does not affect the selected Cowbell mapping or the build.

## Upstream

- [Adafruit DVI HSTX](https://github.com/adafruit/Adafruit-DVI-HSTX)
- [Adafruit PiCowBell HSTX DVI Output guide](https://learn.adafruit.com/adafruit-picowbell-hstx-dvi-output)
- [fruit-bat/pico-zxspectrum](https://github.com/fruit-bat/pico-zxspectrum)

# GSX screen driver for ZXTeensyIF1 CP/M Plus

`patch_gsx.py` adapts the PCW `DDSCREEN.PRL` from your own disk for the CP/M
Plus port in `CPM/`. It creates a regular 180K CPC DSK containing a 256 by
192 monochrome Spectrum screen driver, an assignment file and a standalone
demonstration. It does not change the firmware or CP/M boot image.

## Create the driver disk

You need Python 3.7 or later, [sjasmplus](https://github.com/z00m128/sjasmplus)
and a local disk containing the supported `PCW8256 GIOS #8` driver. No external
disk converter or Python packages are needed to build it.

The default source is:

```text
ignoredfiles/Pascal 80 (Side B).dsk
```

From the repository root:

```powershell
python CPMGSX/patch_gsx.py
```

The output is `build/CPMGSX/GSX.DSK`. The input is preserved, and an existing
output is never overwritten. Choose a new filename when rebuilding:

```powershell
python CPMGSX/patch_gsx.py "path/to/source.dsk" --output build/CPMGSX/my-gsx.dsk
```

To keep a previous screen-callback build for comparison, create the direct
bank-7 revision under a separate name:

```powershell
python CPMGSX/patch_gsx.py --output build/CPMGSX/GSX-BANK7.DSK
```

Only the original driver is checked by SHA256, including its PRL header,
code and relocation bitmap. Disk filenames, unrelated files, filesystem
fragmentation and CP/M record padding do not need to match. Unsupported or
already patched drivers are rejected rather than patched at guessed offsets.
The source may be a normal/extended CPC DSK or a Teledisk 2.x image, including
advanced compression, with a supported Amstrad filesystem.

The CPC6128 `cpmplue3.dsk` contains `GSX.SYS` and `GENGRAF.COM`, which can
be supplied using `--runtime-disk` when building a disk for GSX applications.

| Option | Use |
|---|---|
| `--inspect` | Validate the supported driver without writing anything or requiring the assembler. |
| `--output PATH` | Choose a new output DSK path. |
| `--sjasmplus PATH` | Use an assembler outside `PATH`. |
| `--runtime-disk PATH` | Also copy your `GSX.SYS` and `GENGRAF.COM` from another local DSK/TD0. Both must be present. |

Intermediate files are assembled afresh under the ignored `build/CPMGSX`
directory and removed after the build. Generated disks, drivers and demos
contain components from your original software and must stay local; no
original PCW code, fonts or OS binaries are supplied in this directory.

## Try it on the Spectrum

1. Boot your existing ZXTeensyIF1 CP/M Plus disk as A:.
2. Mount `GSX.DSK` as B:, with the second floppy drive enabled before booting
   as described in [`CPM/README.md`](../CPM/README.md#drive-b-and-changing-disks).
3. Run the standalone demonstration:

   ```text
   B:GSXTEST
   ```

4. Expect a rectangular frame, a filled rectangle and `ZXTeensyIF1 GSX` text.
   Press a key to close the demonstration and return to CP/M.

This is a **data disk**, not a replacement for `CPMBOOT.DSK`. The demo embeds
the locally patched driver and calls its GIOS entry directly, so it needs no
GSX runtime. It also works when copied to another available CP/M drive.

For other GSX applications, keep `ASSIGN.SYS` and `DDSCREEN.PRL` in the current
drive/user area. The supplied assignment is:

```text
01 @:DDSCREEN
```

GSX applications also need GDOS: either linked into the application already,
or attached using your own `GSX.SYS` and `GENGRAF.COM`. These runtime files are
absent from the supplied Pascal disk. The optional `--runtime-disk` switch
copies them from another local image; it does not modify them.

## Adaptation and limitations

The patch keeps the original GSX dispatch, clipping, line styles, markers,
8x8 text, polygon/bar fills, patterns/hatches and writing modes. New assembly
replaces PCW roller-RAM addressing, the optimised horizontal fill, screen
clearing and PCW-specific initialisation/input. The PRL size and relocation
bitmap are rebuilt; the original bitmap is retained outside replaced bytes,
and new relocation bits are derived from two assembler origins.

Drawing accesses Spectrum RAM bank 7 directly at `C000h` (attributes at
`D800h`). It does not call `SCR RUN ROUTINE`, copy the screen through RAM 5,
or use the D6 application-RAM backup page. The port uses 32 columns and
disables the status line for graphics.

Mapping bank 7 hides the loaded GIOS/BDOS in RAM 0. To keep drawing code and
its stack visible, the driver reserves the otherwise unused **AllRAM D7**
page, distinct from Spectrum RAM bank 7. At open workstation it copies and
relocates the raster code there once and caches the native 8x8 font from D3
at `3000h`. During each drawing call only registers and a small graphics-state
block are transferred. Raster routines run below `4000h` with a private
stack at `2FF0h`, write the visible bank-7 bitmap directly, then restore RAM 0
and the caller's low bank, stack and interrupt state. Interrupts are disabled
while the normal BIOS is hidden; 1FFD motor/other bits are preserved, and E3
is written last when restoring the AllRAM map.

The font is cached again on each open workstation. D7 remains reserved while
the driver is open; applications must not use that AllRAM page as scratch.
Normal close restores the previous full-screen 32/51/80-column terminal mode
and status-line setting, clears the display and enables the text cursor.
Custom text viewports and their cursor positions are reset on close.

The driver reports monochrome, square pixels and coordinates `0..255,0..191`.
GSX coordinates have their origin at the bottom left; the retained driver
converts these to the Spectrum's top-left bitmap. The font and graphics use
white on black. Colour indices are still the original two-colour palette;
additional Spectrum attribute colours are not exposed as GSX pens.

There is one keyboard locator. The usual translated cursor controls move it;
other keys select/terminate a request. PCW mouse/tablet RSX discovery is
disabled. No new features are added to GSX operations unsupported by the
original driver. GIOS code must be loaded at a page-aligned address at or
above `8000h`, entirely below the BDOS. The direct renderer requires this
project's CP/M memory map, with common memory starting at `4000h`.

The driver and demo have passed Z80 integration checks, including loading the
demo from B: under the project's CP/M, drawing and returning to its prompt.

### Run the checks

```powershell
python -m unittest discover -s CPMGSX/tests -v
```

Disk tests cover compression/CRC failures, standard/extended containers,
fragmentation, multiple extents, 720K allocation pointers, user areas and
overwrite protection. Driver integration tests additionally require your local
PCW/+3 originals, sjasmplus and the project's SkoolKit 8.10 test dependency in
`build/CPM/deps`. They exercise the actual patched PRL with the CP/M terminal,
four load addresses, pixel/line/fill/text operations, writing modes, clipping,
keyboard locator and terminal restoration. Additional checks reject any screen
callback or drawing write to RAM 5, verify that D6 is untouched, and exercise
low/shared/upper caller stacks, motor state and both interrupt-enable states.
The final boot test uses the original CP/M BDOS/CCP and emulated FDC to load
the demo.

## Background references

- [John Elliott's Amstrad GSX driver investigation](https://www.seasip.info/Cpm/gsxint.html),
  including his prototype Spectrum +3 adaptation.
- [GSX API](https://www.seasip.info/Cpm/gsx.html).
- [Amstrad XBIOS and screen callback](https://www.seasip.info/Cpm/xbios.html).

# CP/M Plus for ZXTeensyIF1

Use this patcher to create a CP/M Plus boot disk for a 128K Spectrum with
ZXTeensyIF1, +3e ROMs, the +3 FDC and DivMMC AllRAM without its ROM. The boot
DSK provides drive A:; a second DSK can provide B:, and an optional IMG
provides storage on C:, D: and E:.

## Create the boot DSK

You need Python 3.7 or later, [sjasmplus](https://github.com/z00m128/sjasmplus)
on `PATH`, and your own original +3 CP/M Plus disk image. The patcher requires
no additional Python packages.

The original disk is **non-distributable**. The patcher preserves your input
and creates a separate disk using components extracted from it. Generated DSK
and EMS files still contain non-distributable software and must stay local.
No original disk, operating-system, font or ROM binary is supplied here.

Place the original disk at the default input path:

```text
ignoredfiles/CP-M Plus v1.0 (1988)(Locomotive)(+3)(Side A).dsk
```

Then run this from the repository root:

```powershell
python CPM/patch_ems.py
```

The output is `build/CPM/CPMBOOT.DSK`. Intermediate files are kept in the
repository's ignored `build/CPM` directory. To use different paths:

```powershell
python CPM/patch_ems.py "path/to/original.dsk" --output "build/CPM/my-boot.dsk"
```

Any valid disk in the supported +3 format can be used when its `S10CPM3.EMS`
and boot sector match the supported revisions. These two components are
checked by hash; the disk filename, whole-disk hash and other files are not
required to match. Edited profiles, added files and removed utilities are
accepted. Unrelated files and their contents are preserved.

The patcher deletes the incompatible `RAMDISC.FID` and reclaims unused EMS
allocation blocks before installing `DIVMMC.FID`. There must be enough allocation
blocks and directory entries
for the rebuilt EMS and driver; otherwise it reports an error and creates
no output disk. An existing user-0 `DIVMMC.FID` is replaced by the built driver.

The patcher refuses to overwrite the input or any existing output image.
Choose a new output filename when rebuilding.

Other options:

| Option | Use |
|---|---|
| `--inspect` | Validate the EMS and boot sector without creating an output image. |
| `--sjasmplus "path/to/sjasmplus.exe"` | Select an assembler outside `PATH`. |
| `--keep-profile` | Keep `PROFILE.SUB` active, including a customised startup script. The default renames it to disable execution, preserving its contents. |

## Create an optional data DSK for B:

Create an empty 720K data disk, with no operating-system files:

```powershell
python CPM/make_floppy.py build/CPM/CPMDATA.DSK
```

The default is the double-sided, 80-track PCW format. Use `--format plus3`,
`--format cpc-system` or `--format cpc-data` for a 40-track, single-sided disk.
The command refuses to overwrite an existing image. These blank data images
contain no original CP/M software and are not bootable.

Existing standard or extended CPC DSK containers can also be mounted. Supported
media have 512-byte sectors and a recognised +3/CPC format or PCW boot-sector
specification. A: retains the internal single-sided 40-track drive geometry;
B: supports single/double-sided media with up to 80 tracks per side.

## Create optional IMG storage

Skip this step if you already have a compatible storage image. To create a
new, empty image:

```powershell
python CPM/make_img.py build/CPM/CPMSTORE.IMG
```

This creates a 64MiB IMG containing three empty 16MiB storage partitions. It
refuses to overwrite an existing file.

| CP/M drive | Partition name |
|---|---|
| C: | `CP/M` |
| D: | `SOFTWARE` |
| E: | `DEVEL` |

An existing IMG must have these named +3DOS partitions and the supported disk
geometry described under [Storage format and FIDs](#storage-format-and-fids).
If the IMG is missing or incompatible, CP/M still boots from A:; unavailable
partitions are not registered.

## Boot on the Spectrum

1. Install the repository firmware **20261118 or later**, which adds the external
   B: geometry and live IMG remounting. Copy the generated boot DSK to the
   Teensy SD card as `/CPMBOOT.DSK`.
2. Copy any data DSK as `/CPMDATA.DSK`, and optional IMG storage as `/CPMSTORE.IMG`.
3. Select a configuration based on [CPM-Plus.CFG](../ZXTEENSY/CONFIGS/CPM-Plus.CFG). It enables the
   FDC and DivMMC AllRAM, disables the DivMMC ROM, and selects `dvmen3eE.rom`.
   It also enables `divMmcInGameUnmount` for the SDA:/SDB: unmount controls.
   Supply the +3e ROM separately.
4. Start +3e from a fresh power-on, then choose its disk **Loader**.
5. Expect the `CP/M Plus - ZXTeensyIF1` banner, the 61K TPA and detected
   physical floppy-drive count, then any FID startup messages and `A>`.
   `DivMMC SDHC disk FID` confirms IMG storage was installed; without suitable
   storage, `DivMMC image unavailable` is displayed and A: remains usable.

The configuration's image paths are:

```text
dskFdaPath = /CPMBOOT.DSK
divMmcSdaPath = /CPMSTORE.IMG
```

Use the firmware with AllRAM paging/reset support and the
+3-compatible `7FFD` port-decoding modification described in the main
[README](../README.md). Only 128K of DivMMC RAM is required; the 512K extension
is unnecessary.

Coloured blocks during loading are normal. The disk loader temporarily uses
screen memory before the BIOS switches to the console display. Startup uses
51 columns, 23 text rows and a status line, with white text on blue paper and
a blue border.

With `divMmcAllRamClearOnReset = 1` in the supplied configuration, an ordinary
Spectrum reset returns to the +3e ROMs. This reset option requires firmware
`20261117` or later. A button-held menu reset or power-cycle also returns to
+3e.

## Using CP/M

The boot disk is A:. For example, list its files and start Mallard BASIC:

```text
DIR
BASIC
```

At BASIC's prompt, `PRINT 2+2` evaluates an expression and `SYSTEM` returns
to CP/M. With IMG storage mounted, copy BASIC to C: and run it there:

```text
DIR C:
PIP C:=A:BASIC.COM
DIR C:
C:BASIC
```

D: and E: work in the same way. The boot disk has little free space, so use
B: or the IMG partitions for programs and files.

Running a `.SUB` file also needs free space for SUBMIT's temporary file.
If SUBMIT reports `Disk Write Error`, free space on the temporary drive or,
when C: is available, run `SETDEF [TEMPORARY=C:]` before submitting the file.
Automatic `PROFILE.SUB` execution needs working space on A: at startup.

### Drive B: and changing disks

For a separate B: drive, enable it in the configuration **before booting**:

```text
dskEnableDriveB = 1
dskFdbPath = /CPMDATA.DSK
```

Then use normal commands such as `DIR B:` and `PIP B:=A:BASIC.COM`. B: may
start empty and have a DSK inserted later. With B: disabled, CP/M supplies a
second logical drive through A: and prompts when its disk must be exchanged.

To change a DSK or IMG, finish the running program and return to the CP/M
prompt. Wait for the floppy motor to stop (about five seconds), open the
Teensy's in-game menu, eject/mount the replacement and resume CP/M. Press
**Ctrl-C before using the changed disk** so CP/M logs it in again. The
Spectrum combination for Ctrl-C is CAPS SHIFT + SYMBOL SHIFT + C. Leave the
boot DSK available if a program still needs to load files from A:.

IMG storage uses **SDA:** in the Teensy menu; C:/D:/E: are partitions inside
it. A new login rereads the partition table and updates partition locations.
Only the C:/D:/E: callbacks installed at boot remain available: adding a
partition that was absent at boot requires a fresh boot. Swapping during an
active write or before Ctrl-C can leave CP/M with the previous disk's state.

### Disk errors and recovery

Floppy operations retry up to fifteen times, including head realignment,
before displaying a question. Native +3 messages include the failing track
and sector where applicable, and identify the drive on two-drive systems. Choose:

- **R** to retry after inserting/remounting a disk or removing write protection.
- **I** to ignore an I/O failure. A failed read supplies an empty `E5` sector;
  a failed write is discarded. Use this only when losing that data is acceptable.
- **C** to cancel and return the failure to CP/M or the calling program.

Login failures accept Retry/Cancel because CP/M needs a valid disk format.
IMG I/O failures use the same recovery interface; Retry reinitialises the
card and refreshes its partition table. The motor stops while a recovery or
single-drive swap prompt waits, allowing the menu to remount an image.
Questions use the original rotating status line in all display modes, and
the console when the status line is disabled. Native bleeps announce status
questions and invalid responses. Pending typeahead is cleared before a response
is accepted. BDOS error-return mode (`FF`) returns failures without asking a
recovery question. The single-drive indicator follows the physical A:/B:
mapping and is suppressed when two physical floppy drives are fitted.

### Display and keyboard

The console uses the original +3 terminal and both fonts. It supports 32, 51
and 80 columns, eight national character sets, colours, a status line,
viewports and the original editing escape sequences. The supplied `LANGUAGE`
and `PALETTE` utilities use their normal +3 interfaces. For 80-column mode:

```text
SET24X80 ON
SET24X80 OFF
```

The 80-column display shows part of the logical screen at a time. It follows
the cursor automatically; CAPS SHIFT + SYMBOL SHIFT + ENTER changes the
visible half manually. The cursor appears after a short delay following
output and then stays steady.

The keyboard has the original programmable maps, expansion strings, lock
states and repeat behaviour. `SETKEYS RESET.KEY` loads the supplied keyboard
definitions. CAPS SHIFT uppercases letters; CAPS SHIFT + SYMBOL SHIFT + a
letter produces its control code. CAPS SHIFT + SPACE produces Escape.
The separate cursor/Delete keys use the corresponding CAPS+digit combinations.
These are the boot-time defaults; SETKEYS can change them:

| CAPS SHIFT + | Default function |
|---|---|
| `0` | Delete left. |
| `1` | Control-W. |
| `2` | Toggle Caps Lock; no character. |
| `3` | Control-S. |
| `4` | Control-I. |
| `5/6/7/8` | Left/down/up/right. |
| `9` | Delete under cursor, Control-G. |

## Compatibility and limitations

The image uses the original CP/M Plus BDOS and CCP, retains the 61K transient
program area, and supports warm boot, A:/B: floppy reads/writes, multiple disk
formats, removable DSK/IMG images, recovery prompts, IMG storage, the native
terminal, programmable keyboard and a 50Hz software clock. Boot, the console
and Mallard BASIC have been confirmed on hardware. The new disk features are
covered by CPU integration tests and still need confirmation on the Spectrum.

The following restrictions apply:

- Serial/parallel hardware I/O is unsupported. Disk changes require Ctrl-C;
  arbitrary media swaps during active file operations are not automatic.
- Floppy support is limited to 512-byte sectors, at most 364 allocation blocks,
  256 directory entries and 64 directory-checksum bytes. Supported formats
  include standard +3, CPC system/data and suitable PCW specifications.
  Unsupported or oversized formats fail login with a recoverable prompt.
- Formatting through XBIOS operates inside an existing DSK container. It does
  not create or grow the host file. Raw 765 calls stage at most 512 data bytes.
- LIST currently uses the console. The character device table advertises
  CRT only, and `CD INFO` reports serial absent.
- FID support covers disk devices. Character FID hooks/find and arbitrary
  application service exports from private FIDs are unsupported. The supplied
  original `RAMDISC.FID` is deleted from the output disk to reclaim its space.
- The rearranged memory map does not reproduce direct native `SELBNK 0/2`
  screen access. Applications should use the terminal or `SCR RUN ROUTINE`.
- The software clock has no RTC synchronisation. Heavy rendering can slow it
  while interrupts are disabled, and 80-column screen swaps take longer than
  native hardware paging.

## Technical details

### Disk image patching

The DSK container can be standard or extended with the standard 40-track,
single-sided +3 format. Extended containers may also declare unused,
unformatted trailing tracks. Sector order and file fragmentation are handled
using the DSK headers and CP/M allocation lists. Malformed and cross-linked
disks are rejected.

The patcher rebuilds and compresses `S10CPM3.EMS`, releases unused trailing
allocation blocks, patches the boot sector and adds the assembled `DIVMMC.FID`.
It renames `PROFILE.SUB` to `PROFILE.OLD` to disable the automatic startup script;
`--keep-profile` retains that script's automatic execution, including user
edits. It deletes `RAMDISC.FID` before allocating the rebuilt EMS and driver,
freeing its blocks and directory entries. That RAM disk driver consumes the
private heap and assumes the original +3 memory environment. If a profile backup
name already exists, the next unused `.001`, `.002`, etc. suffix is used without
replacing old backups. The profile's contents, all other supplied files, and
disk/track headers are preserved, including existing `RAMDISC.OLD` backups.

The boot-sector initialiser enables DivMMC AllRAM and disables automapping
before the original disk loader writes the EMS into low memory. CP/M then
uses ordinary 128K paging plus DivMMC AllRAM; native +3 all-RAM mapping of the
Spectrum's internal RAM is not required.

### Memory and banking

Here D0..D7 are **16K AllRAM pages**, not the usual 8K DivMMC banks.
Only 128K of DivMMC RAM is required; the 512K extension is unnecessary.

| Area | CP/M bank 0 | CP/M bank 1 | CP/M bank 2 |
|---|---|---|---|
| `0000-00FF` | Vectors and XBIOS jumpblock, D0 | CP/M page zero, D1 | Driver vectors, D4 |
| `0100-0BFF` | Disk buffers, floppy/FID DPHs, BCBs, FID header mirrors and XBIOS stubs, D0 | TPA, D1 | Hardware BIOS, D4 |
| `0C00-39FF` | Original banked BDOS, relocated from `8C00-B9FF`, D0 | TPA, D1 | Keyboard, native format builders, FDC buffers, XBIOS and recovery code, D4 |
| `3A00-3FFF` | Three FID allocation vectors (510 bytes each), D0 | TPA, D1 | Private disk-call stack, D4 |
| `4000-7FFF` | Shared TPA, Spectrum RAM 5 | Same | Same |
| `8000-BFFF` | Shared TPA, Spectrum RAM 2 | Same | Same |
| `C000-F3FF` | Shared TPA, Spectrum RAM 0 | Same | Same |
| `F400-F9FF` | Original resident BDOS | Same | Same |
| `FA00-FFFF` | Common BIOS, DPBs, structures, stacks and MOVE/IRQ buffer | Same | Same |

The remaining logical banks are:

| Bank | AllRAM page | Use |
|---|---|---|
| 3 | D5 | Cached original CCP for warm boot. |
| 4 | D2 | Disk-FID host, relocated FIDs, supervisor allocations, `3C00` staging buffer and private stack. |
| 5 | D3 | Original terminal, messages, display state, private stack and fonts at `3000`/`3800`. |
| 6 | D6 | Temporary 16K TPA backup during `SCR RUN ROUTINE`. |
| 7 | D7 | Free, 16K. |

The FID host starts allocating code at the next page after its own code/data;
allocations and sixteen bank-0 header mirrors are bounded. A boot-only program
at `6000` scans user 0 on A: through BDOS before the CCP starts. Its space and the
PRL staging area then become available to programs. FID loading is not repeated
on warm boot. The transient area remains `0100-F3FF`: 61K less CP/M page zero.

Spectrum RAM 7 is always the visible screen. RAM 6 holds the other half of the
native 80-column display. Rendering pages the required surface at `C000` and
runs entirely from D3 with a private stack; RAM 0 is restored before returning
to common code. Changing the visible half swaps the two 8K surfaces. The 765
transfers through a private 512-byte buffer, followed by a banked DMA copy.
Writes are staged before the FDC command is sent.

The common gate keeps a stack frame for each active call, so nested USERF,
keyboard, terminal, FID and timer calls preserve bank, registers and IFF state.
IM 1 interrupts use `FF80-FFFF`, which MOVE only uses with interrupts disabled.
MOVE disables interrupts before saving its shared stack/copy context, so a timer
interrupt cannot replace its saved caller stack during setup.
This avoids using the CCP loader's small stack for interrupt register saves.
The compressed EMS uses literals, byte runs and overlapping LZ backreferences;
its decoder also handles the staged source crossing `FFFF` to `4000`.

Bank selection writes `7FFD` and `1FFD` before `E3`. The final `E3` write
refreshes both 8K AllRAM halves after the page-selection bits have changed.
Current firmware also refreshes its pointers in the paging handlers; the
explicit `E3` keeps the image compatible with older pointer-refresh behaviour,
which the CPU test harness models.

`bdos_relocations.txt` contains high-byte offsets only. These are the internal
relocations from the standard 2E00-byte banked BDOS SPR relocation bitmap,
validated against the supplied EMS. External references to resident BDOS, the
SCB and BIOS stay at their existing addresses. The two resident-to-banked BDOS
references and the SCB common-memory base are updated separately. The patcher
does not disassemble and guess which bytes might be addresses.

### Terminal, keyboard and XBIOS

The terminal retains the original +3 parser and plotting routines, including
line/character insertion and deletion, viewport erasure/scrolling, reverse
video, underline, wrapping, page mode, national character sets and
RGB/bright/flash attributes. The SCB width and page length follow the viewport.
The cursor appears five keyboard scans (0.1s) after output. Auto-flip in
80-column mode can be disabled with `ESC 5`.

The bank-0 XBIOS jumpblock and USERF inline-word calling convention support:

| Address | Call | Behaviour |
|---|---|---|
| `0080-00B3` | All eighteen `DD` entries | Initialise/setup, sector read/write/check, format, login, select format, drive status, read ID, DPB/XDPB construction, motor controls, raw 765 commands and seek. |
| `00BF-00D1` | All seven `TE` calls | Native viewport/cursor query, reset, status-line query/control, ink, border and hardware-flash speed no-op. |
| `00D4` | `KM SET EXPAND` | 31 programmable tokens sharing 120 expansion-character bytes; invalid/full definitions fail atomically. |
| `00D7` | `KM SET KEY` | Set any combination of the four forty-key translation maps. |
| `00DA/00DD` | `KM KT GET/PUT` | Native twenty-token queue, physical key numbers, shifts and lock states. PUT prepends. |
| `00E0` | `KM SET SPEED` | Repeat delay/period in 50Hz scans; defaults 30/2, zero means 256. |
| `00E3` | `CD VERSION` | A=3; BC=000A and HL=000A identify this port's BIOS 0.10. |
| `00E6` | `CD INFO` | Detected one/two floppy drives, eight Spectrum memory blocks, no serial backend, common buffer-table pointer. |
| `00E9` | `SCR RUN ROUTINE` | Common callback with main screen at `4000`, fonts at `3000`/`3800`, nested BIOS/XBIOS calls and returned registers. Shared TPA is restored afterwards. |
| `00EC` | `FIND FID` | Find installed FID header metadata by its eight-byte name. |

USERF can also call an application-supplied routine in bank 0. Calls preserve
index/alternate registers except documented outputs, the caller's bank and IFF,
and accept low or shared stacks. `SCR RUN ROUTINE` temporarily backs up all
Spectrum RAM 5 in D6, provides a common callback stack outside the screen, then
restores the TPA with a stackless common trampoline. Font changes persist.
The rearranged map does not reproduce direct native `SELBNK 0/2` screen access;
callbacks must preserve the selected bank and use the supplied screen window.

Keyboard scanning follows the original matrix numbering and row order,
including modifier tokens. Translations, repeat flags, Num/Shift/Caps Lock,
expansion strings, ignored keys and NUL input follow the +3 interface. `CONST`
leaves a valid token available to `KM KT GET`.

The timer advances the SCB's BCD hours/minutes/seconds and binary day counter
every fifty interrupt services. BIOS TIME resets the fractional second on a
clock write. `SVC_CATCHUP` performs one timer/keyboard service with interrupts
disabled. Rendering and screen swaps disable interrupts while common RAM is
hidden, so the original scroll-vblank wait is omitted.
Once cold-start setup is complete, floppy motor spin-up waits permit interrupts
so the keyboard and clock continue running during the configured delay.
Empty `KM KT GET` calls return from common RAM with the caller's interrupt state
unchanged. This lets programs such as DISCKIT poll for a key while keyboard scans,
the delayed cursor and the motor-off timer continue at 50Hz.

### Floppy formats and recovery

The patcher extracts the original +3 format specifications and pure DPB/XDPB
construction routines locally. Login recognises CPC system IDs `41..49`, CPC
data IDs `C1..C9`, and PCW specifications beginning at sector 1. Blank uniform
specification sectors fall back to the standard +3 format. PCW specifications
can describe single sides, alternating sides or side 0 followed by side 1 in
reverse track order. The native extended-DPB freeze flag is honoured by BIOS
selection; explicit DD LOGIN always probes the media again.

Each floppy has its own 27-byte common XDPB, bank-0 DPH, directory checksum
vector and two-bit allocation vector. Directory/data BCBs are shared. Fresh
login invalidates this drive's cached sectors, including when its geometry
matches the preceding disk. The driver tracks the physical cylinder per unit.
An unfitted B: selects the single-drive swap path; an empty fitted B: remains
a separate drive. The updated firmware models B: as an 80-cylinder,
double-sided 3.5-inch drive while retaining A:'s internal +3 geometry.

High-level BIOS I/O stages one 512-byte sector in driver RAM. Native format
builders, error text and DD routines also live in the driver bank, with a
private stack for nested keyboard/terminal calls during prompts. DD calls
return their documented success/error status directly; interactive recovery
belongs to BIOS/FID I/O. Read-ID/raw command results are copied to a common
result area after any banked DMA copy. Low-level raw calls are limited to
512-byte transfers and require the caller to prepare the drive/motor.

DD FORMAT and READ ID use the same fifteen-attempt recovery policy as sector
I/O. DD DRIVE STATUS starts the motor before sampling the drive. DD L ON MOTOR
waits for the configured spin-up delay and cancels a pending shutdown; only
DD L T OFF MOTOR starts the off timeout. BIOS disk calls start that timeout
when they finish. DD SETUP preserves the native SPECIFY parameter encoding,
and PCW login returns the native allocation-vector and hash-table sizes.

### Storage format and FIDs

The IMG uses a PLUSIDEDOS partition table with three 16MiB type-3 +3DOS
partitions, following the named-partition/DPB convention of Paul Osmialowski's
public-domain [ZXMMC FID](https://sourceforge.net/p/zxmmcfid/wiki/Home/).

| Drive | Partition | Start cylinder | Start LBA |
|---|---|---|---|
| C: | `CP/M` | 1 | 256 |
| D: | `SOFTWARE` | 129 | 33024 |
| E: | `DEVEL` | 257 | 65792 |

Geometry is two heads and 128 sectors per track, with 512-byte sectors. Each
partition uses the ZXMMC FID's 17-byte DPB: 8K allocation blocks, DSM 2039,
512 directory entries, two directory blocks, no reserved tracks and no CSV.
Usable file capacity is slightly below 16MiB. The formatter writes the table
and empty directories; other sectors start as zeroes. The system partition's
Editor/BASIC colour bytes are both `38` hex.

The FID searches for the partition table at LBA 0, then 128. It uses DivMMC's
`E7`/`EB` ports and SDv2/SDHC block addressing, matching the firmware's IMG
backend. It does not use the +3e ROM disk API after CP/M starts. The FID keeps
a 512-byte supervisor allocation for partition scans throughout its lifetime;
LOGON reinitialises SD and rescans the current image instead of retaining old
partition locations after a remount.

At cold boot the loader collects user-0 `*.FID` filenames from A:, reads each
PRL, and asks the private host to validate its size, `SPECTRUM/FID` header,
checksum, page bounds and relocation bitmap. The host relocates ordinary
addresses and the +3 supervisor references, zeroes BSS, links retained FIDs,
and calls FID_EMS with environment version 0.0 and country 3 (U.K.), matching
the supplied original EMS. Rejected startup allocations are rolled back.
Installed drive callbacks survive warm boot.

Disk support implements `D_HOOK`, `D_CHANGED`, `ALLOCATE`, `MAX_ALLOCATE` and
`DEALLOCATE`, plus the banking-shadow/SCB addresses and timer/keyboard `CATCHUP`.
Allocations support sixteen supervisor-allocation records, arbitrary release
order, partial releases, hole reuse, and largest-contiguous-block allocation.
Unused parts of a loaded FID can also be returned; retained image fragments
use a separate table with 32 image-fragment records. The three extra drive
slots are C: through E:, with at most 510 ALV bytes per drive, no CSV, and optional BDOS
hashing disabled. LOGON validates the supported hard-disk DPB limits. READ and
WRITE pass logical track/sector, write type, DPB and a visible 512-byte buffer
to the FID. A banked copy moves sectors to/from the original DMA bank/address;
index and alternate registers survive BIOS calls. Callbacks execute with a
private stack and mapped FID page. FLUSH invokes every installed disk driver.

Disk-FID failures honour the native carry/zero convention: noninteractive
failures return directly, while interactive failures can supply their own
message through FID_D_MESS. Cancel preserves the driver's CP/M error code,
including read-only and media-changed results. Retry repeats the failed
callback; DIVMMC.FID refreshes SD and the partition table itself when retrying
a failed transfer. SVC_D_CHANGED marks both the DPH and the BDOS notification
flag, which BDOS consumes through its normal disk-change processing.

`DIVMMC.FID` initializes SDv2/SDHC with CMD0/8/55/ACMD41/58 and uses CMD17/24
single-sector I/O. Command, token, busy and partition-table searches are bounded;
unsupported geometry or DPBs are rejected. Legacy byte-addressed MMC/SDSC is
unsupported. The original ZXMMC binary uses different SPI ports and cannot be
used unchanged. FIND FID returns header metadata in bank 0; it does not make
code following a private FID header callable.

Each floppy BIOS READ/WRITE performs one 512-byte physical transfer; MULTIO
is a hint. FLUSH and warm boot stop the motor so ZXTeensyIF1 flushes the DSK
file. IMG writes wait for the SD data-response/busy sequence, and FID FLUSH
deselects the device. Persistence is supplied by the firmware's IMG backend.

### Validation

The native comparisons in `tests/test_native_behaviour.py` cover message text,
status-line rotation, bleep output, physical drive indicators, FID country and
memory-release behaviour, and keyboard/clock progress during motor spin-up.
DISCKIT runs on both one- and two-drive configurations also check the initial
eject prompt, motor timeout, delayed cursor and response to a physical key.

Run the automated suite from the repository root:

```powershell
python -m unittest discover -s CPM/tests -v
```

Disk/parser tests use synthetic data;
local integration tests require the original disk and sjasmplus. CPU tests
also require the development-only SkoolKit dependency:

```powershell
python -m pip install --target build/CPM/deps skoolkit==8.10
python CPM/verify_boot.py build/CPM/CPMBOOT.DSK --dependencies build/CPM/deps
```

For an IMG-backed run, with all emulated writes staying in RAM:

```powershell
python CPM/verify_boot.py build/CPM/CPMBOOT.DSK --dependencies build/CPM/deps --storage build/CPM/CPMSTORE.IMG
```

A second floppy can be supplied with `--drive-b build/CPM/CPMDATA.DSK`.

The harness executes the actual patched boot sector, disk loader, rebuilt EMS,
BDOS, CCP and programs using a Z80 CPU, fixed Spectrum RAM 5/2 at `4000/8000`,
ordinary `7FFD` paging, DivMMC AllRAM, keyboard matrix input, IM 1 interrupts and
a functional 765 sector model. It starts at the documented +3e boot handoff
(`FE10`, with the boot sector in Spectrum RAM 3), not at an already loaded CCP.
It does not emulate the +3e menu or its initial ROM disk read.

The integration tests fill all shared TPA (`4000-F3FF`) with a pattern, call
BDOS console output, and verify that the pattern survives. They run the original
DUMP, SHOW, SETKEYS, TIME, SET24X80, LANGUAGE, PIP and Mallard BASIC programs;
exercise physical cursor/Delete editing; write/read/delete on A: across reboot;
and copy BASIC.COM A: to C:, D: and E: with byte comparisons and a subsequent
BASIC run from the IMG. Emulated disk writes remain in RAM or private test files.

The unmodified +3 disk is also booted in a native +3 banking model as an oracle.
Tests compare all 2,560 key/shift/lock/repeat translations, raw matrix tokens
including modifiers, terminal bitmaps/attributes in 32/51/80-column modes, all
eight languages, colours, status-line behaviour, viewport edits, page mode and
cursor/auto-flip behaviour. Separate checks verify glyph pixels at every bit
alignment, keyboard queue/expansion contracts, SCB clock rollover, nested screen
callbacks, incoming callback A/flags, register/IFF preservation, and interrupts
with small stacks in every executable AllRAM bank, plus interrupts during banked
MOVE setup. The reference code is always read from the user's disk.

Disk tests also cover delayed/rejected SD transactions, DMA across `3FFF/4000`,
all eight AllRAM pages, allocation failure/non-LIFO frees, malformed FID PRLs,
missing storage, partition-table fallback and FIND FID metadata. Patcher tests
check determinism, original-file hashes, permitted image changes, malformed
containers and refusal to overwrite an input or output image. Floppy tests
cover A:/B: copies and persistence, CPC and PCW geometries, both double-sided
track orders, blank-image creation, native XDPB comparisons, DD formatting and
sector calls, single-drive swaps, same/different-format changes, write protection,
missing disks, Retry/Ignore/Cancel, unsupported formats, and changed IMG partition
locations. Recovery messages are compared with native terminal rendering. The single-drive
`DIR`, `B:` path checks the swap message glyphs and dismissal with Return.

The harness is a functional CPU/sector model. It does not model Teensy bus
or SD/FDC timing cycle by cycle; automated coverage complements the hardware
confirmation.

### References

[CP/M Plus on the +3](https://k1.spdns.de/Vintage/Sinclair/86/ZX%20Spectrum%2B3/ZX%20Spectrum%2B3%20CP:M%20Manual/appendix6.html),
[terminal escape sequences](https://k1.spdns.de/Vintage/Sinclair/86/ZX%20Spectrum%2B3/ZX%20Spectrum%2B3%20CP:M%20Manual/appendix3.html),
[Digital Research's CP/M 3 System Guide](https://www.cpm.z80.de/manuals/cpm3-sys.pdf),
[DivMMC AllRAM specification](https://github.com/mprato/DivMMC/blob/master/docs/Divmmc_allram_manual.txt),
[ZXMMC FID](https://sourceforge.net/p/zxmmcfid/wiki/Home/).

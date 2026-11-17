"""Build a local ZXTeensyIF1 CP/M Plus boot DSK from the user's original disk.

Only new BIOS/loader source and relocation offsets are distributed here. The
original disk, its EMS, and every generated image remain non-distributable.
"""

import argparse
from bisect import bisect_left
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys

from disk import Disk, DiskError


ROOT = Path(__file__).resolve().parent.parent
SOURCE = Path(__file__).resolve().parent
DEFAULT_INPUT = ROOT / "ignoredfiles/CP-M Plus v1.0 (1988)(Locomotive)(+3)(Side A).dsk"
DEFAULT_OUTPUT = ROOT / "build/CPM/CPMBOOT.DSK"
EMS_NAME = "S10CPM3.EMS"
EMS_SHA256 = "47dc895c1b6582be8226639d70c2458a87a5b6915d8ab457ce90210537c9a9c0"
BNK_SHA256 = "1fc8c041bcae7f0808b2759e3cf8e17c5d8bfcb0c7e459b5a3e1ff53763b82af"
RES_SHA256 = "642434851107c41fe2a460b2cfb04105b7a3b313424e3c48b6be951b3c5e3fc8"
BOOT_SHA256 = "9d241c2cb6beb5cba3ea9bfad6fee1f885da535cb4a08ccf92f4175d8e78a7dc"
RELOCATION_SHA256 = "12e48205c0c90ff0ff607f2346926932e905a3516f63576f56f7b38313950f44"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def unpack_pages(ems, end, count):
    # The original EMS loader copies 128-byte chunks in descending source order.
    return b"".join(ems[end - 256 - i * 128:end - 128 - i * 128] for i in range(count))


def extract_components(ems):
    if len(ems) != 32640 or digest(ems) != EMS_SHA256:
        raise ValueError("Unsupported or already patched S10CPM3.EMS (SHA256 %s)" % digest(ems))
    banked = unpack_pages(ems, 0x8000, 0x5c)
    resident = unpack_pages(ems, 0x4e00, 12)
    if digest(banked) != BNK_SHA256 or digest(resident) != RES_SHA256:
        raise ValueError("Original EMS component validation failed")
    font = bytearray(ems[0x20b0:0x30b0])
    font[0] = ems[0x3000]          # original LDDR ends at 3001, not 3000
    return banked, resident, ems[0x3880:0x4500], bytes(font)


def relocate_bdos(banked, resident):
    banked = bytearray(banked)
    resident = bytearray(resident)
    tokens = []
    for line in (SOURCE / "bdos_relocations.txt").read_text().splitlines():
        tokens.extend(line.split("#", 1)[0].split())
    offsets = [int(token, 16) for token in tokens]
    if (len(offsets) != 1913 or offsets != sorted(set(offsets)) or
            digest(b"".join(n.to_bytes(2, "little") for n in offsets)) != RELOCATION_SHA256):
        raise ValueError("Invalid BDOS relocation metadata")
    for offset in offsets:
        if not 0 < offset < len(banked) or not 0x8c <= banked[offset] < 0xba:
            raise ValueError("Unexpected BDOS relocation at %04X" % offset)
        banked[offset] -= 0x80     # 8C00..B9FF -> 0C00..39FF
    for offset in (0x2db, 0x5fd):
        if resident[offset] != 0x8c:
            raise ValueError("Unexpected resident-to-banked BDOS reference")
        resident[offset] = 0x0c
    resident[0x5fa] = 0x40         # common memory base page
    return bytes(banked), bytes(resident)


def assemble(assembler, name, work, defines=()):
    output = work / (name + ".bin")
    command = [assembler, "--nologo", "--inc=" + str(SOURCE), "--inc=" + str(work),
               "--raw=" + str(output), "--sym=" + str(work / (name + ".sym")),
               str(SOURCE / (name + ".asm"))]
    command[1:1] = ["-D" + value for value in defines]
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                universal_newlines=True, timeout=60)
    except subprocess.TimeoutExpired:
        raise ValueError("Assembly timed out for " + name)
    if result.returncode or not output.exists():
        raise ValueError("Assembly failed for %s:\n%s" % (name, result.stdout))
    return output.read_bytes()


def pack(data):
    """Literals, byte runs and 12-bit-distance LZ references.

    1..126: literal count. 127: run (count minus one, value).
    128..255: length 3..10 in bits 4..6, distance high nibble in bits 0..3;
    the next byte completes distance minus one. Length 10 has an extension
    byte, allowing a match of up to 265 bytes. Zero terminates the stream.
    """
    positions = {}
    for position in range(len(data) - 2):
        positions.setdefault(data[position:position + 3], []).append(position)

    def match(position):
        limit = min(265, len(data) - position)
        run = 1
        while run < min(limit, 256) and data[position + run] == data[position]:
            run += 1
        best, distance = 0, 0
        if run < min(limit, 256):
            candidates = positions.get(data[position:position + 3], [])
            index = bisect_left(candidates, position)
            for candidate in reversed(candidates[max(0, index - 64):index]):
                if position - candidate > 4096:
                    break
                length = 3
                while length < limit and data[candidate + length] == data[position + length]:
                    length += 1
                if length > best:
                    best, distance = length, position - candidate
                if length == limit:
                    break
        if run >= 10 and run >= best - 1:
            return run, bytes((127, run - 1, data[position]))
        if best >= 3:
            length, encoded = min(10, best), distance - 1
            token = bytes((128 + ((length - 3) << 4) + (encoded >> 8), encoded & 255))
            if length == 10:
                token += bytes((best - 10,))
            return best, token
        return 0, b''

    output = bytearray()
    position = 0
    while position < len(data):
        count, token = match(position)
        if count:
            output.extend(token)
            position += count
        else:
            start = position
            position += 1
            while position < len(data) and position - start < 126 and not match(position)[0]:
                position += 1
            output.append(position - start)
            output.extend(data[start:position])
    output.append(0)
    return bytes(output)


def make_fid(assembler, work):
    first = assemble(assembler, "divmmc_fid", work)
    relocation_work = work / "fid-relocation"
    relocation_work.mkdir(exist_ok=True)
    second = assemble(assembler, "divmmc_fid", relocation_work, ("FID_ORIGIN=512",))
    if len(first) != len(second) or not 32 <= len(first) < 0x3000:
        raise ValueError("Unexpected FID size")
    bitmap = bytearray((len(first) + 7) // 8)
    for offset, (a, b) in enumerate(zip(first, second)):
        if a != b:
            if b != (a + 1) & 255:
                raise ValueError("Non-page-relocatable FID instruction")
            bitmap[offset // 8] |= 128 >> (offset % 8)
    # Supervisor references have the same FFxx value at both origins.
    # Identify them from assembler labels rather than mistaking data for code.
    labels = {}
    for line in (relocation_work / "divmmc_fid.sym").read_text().splitlines():
        if ": EQU " in line:
            key, value = line.split(": EQU ")
            labels[key] = int(value.strip().replace("0x", ""), 16)
    for key, address in labels.items():
        if ".svc_" in key:
            offset = address - 512 + 2
            if first[offset - 2] != 0xcd or first[offset] != 0xff or first[offset - 1] not in (5, 7, 9):
                raise ValueError("Invalid supervisor relocation label")
            bitmap[offset // 8] |= 128 >> (offset % 8)
    if not bitmap[0] & 32:
        raise ValueError("FID entry point was not relocated")
    header = bytearray(256)
    header[1:3] = len(first).to_bytes(2, "little")
    result = header + first + bitmap
    checksum = sum(result) & 65535
    result[0x110:0x112] = checksum.to_bytes(2, "little")
    (work / "DIVMMC.FID").write_bytes(result)
    return bytes(result)


def terminal_resources(ems, work):
    # The original loader leaves two contiguous terminal regions at these
    # runtime addresses. Keep every original byte local to the build folder.
    prefix = bytearray(ems[0x1e2 + 0xe0:0x5b0 + 0xe0])
    # Only the indirect-call helpers, messages and default border byte are
    # used. Drop the following native serial/disk code from this private page.
    prefix[0x363 - 0x1e2:] = bytes(len(prefix) - (0x363 - 0x1e2))
    core = bytearray(ems[0x900 + 0x258:0x1790 + 0x258])

    # The original keyboard tables in this region belong to our separate
    # keyboard bank; the terminal resumes at its cold-start entry, 1010.
    core[0xf55 - 0x900:0x1010 - 0x900] = bytes(0x1010 - 0xf55)

    def patch(address, before, after):
        offset = address - 0x900
        if core[offset:offset + len(before)] != before:
            raise ValueError("Unexpected terminal instruction at %04X" % address)
        core[offset:offset + len(before)] = after

    # Retain the native parser, plotting, scrolling and 80-column logic.
    # Its working surface is paged at C000 instead of the TPA's 4000 window.
    for address, before, after in ((0x930, 0x4000, 0xc000),
                                    (0x933, 0x4001, 0xc001),
                                    (0x93d, 0x5800, 0xd800),
                                    (0x940, 0x5801, 0xd801),
                                    (0x924, 0xfff8, 0x27f0),
                                    (0xc7e, 0xfff8, 0x27f0),
                                    (0xc8f, 0xfff8, 0x27f0)):
        patch(address, before.to_bytes(2, 'little'), after.to_bytes(2, 'little'))
    # Native scrolling waits for a flag set by its IRQ. Our common ISR is
    # hidden while C000 is the screen window, so rendering runs with DI.
    # Keep the native scroll operation, omitting its vblank synchronisation.
    patch(0xd23, bytes((0xf5,)), bytes((0xc9,)))
    patch(0xecb, bytes((0x40,)), bytes((0xc0,)))
    patch(0xed7, bytes((0x58,)), bytes((0xd8,)))
    patch(0xf3d, bytes((0xc5, 0x4f, 0x3a)), bytes((0xc3, 0, 0x24)))
    for address in (0x929, 0xc87):
        patch(address, bytes((0xed, 0x79)), bytes(2))
    # This build has one CRT console. Native status routing's BA06 hook is
    # inside the original BDOS/BIOS layout and cannot be called here.
    patch(0x15fc, bytes((0xc3, 6, 0xba)), bytes((0xc9, 0, 0)))
    # Native message formatting and BEL routing remain inside the CRT bank.
    patch(0x1519, bytes((9, 0xba)), bytes((0x25, 0x10)))
    patch(0x1642, bytes((9, 0xba)), bytes((0x25, 0x10)))
    (work / 'terminal_prefix.bin').write_bytes(prefix)
    (work / 'terminal_core.bin').write_bytes(core)


def make_ems(ems, assembler, work):
    banked, resident, ccp, font = extract_components(ems)
    banked, resident = relocate_bdos(banked, resident)
    # Data from the user's validated disk remains local, just like the fonts.
    # Four forty-key maps, a five-byte repeat bitmap, 31 expansion records,
    # from the original +3 BIOS; language substitutions stay in the terminal.
    (work / "keyboard_defaults.bin").write_bytes(ems[0x11b8:0x125d])
    (work / "expansion_defaults.bin").write_bytes(ems[0x88f:0x8b9])
    # Reuse the original pure DPB/XDPB calculations and standard specifications.
    # Their runtime addresses stay unchanged within the hardware driver bank.
    for name, start, end in (("format_specs", 0x18a4, 0x18cc),
                             ("format_dpb", 0x1926, 0x1a39),
                             ("format_math", 0x1b37, 0x1b52)):
        (work / (name + ".bin")).write_bytes(ems[start + 0x258:end + 0x258])
    terminal_resources(ems, work)
    (work / "font.bin").write_bytes(font)
    terminal = assemble(assembler, "terminal", work)
    entry = assemble(assembler, "entry", work)
    bios = assemble(assembler, "resident", work)
    drivers = assemble(assembler, "drivers", work)
    if len(entry) != 256 or len(bios) != 1536 or len(drivers) >= 0x3c00:
        raise ValueError("Unexpected loader/BIOS component size")
    host = assemble(assembler, "fid_host", work)
    boot_fids = assemble(assembler, "boot_fids", work)
    kernel_data = assemble(assembler, "kernel_data", work)
    result = bytearray(32640)
    result[:256] = entry
    cursor = 0x0300
    segments = []

    def add(data, bank, destination):
        nonlocal cursor
        packed = pack(data)
        if cursor + len(packed) > len(result):
            raise ValueError("Rebuilt EMS exceeds the original allocation")
        source_address = cursor + 0xc000 if cursor < 0x4000 else cursor
        segments.append((bank, source_address, destination))
        result[cursor:cursor + len(packed)] = packed
        cursor += len(packed)

    add(kernel_data, 0, 0)
    add(banked, 0, 0x0c00)
    add(resident + bios, 0, 0x8000)
    add(drivers, 2, 0)
    add(terminal, 5, 0)
    add(ccp, 3, 0)
    add(host, 4, 0)
    add(boot_fids, 0, 0x6000)
    macro = ["    MACRO SEGMENTS"]
    for bank, source_address, destination in segments:
        macro.extend(("    DB %d" % bank,
                      "    DW $%04X,$%04X" % (source_address, destination)))
    macro.extend(("    DB $FF", "    ENDM", ""))
    (work / "segments.inc").write_text("\n".join(macro))
    loader = assemble(assembler, "loader", work)
    if len(loader) > 0x200:
        raise ValueError("Staging loader is too large")
    result[256:256 + len(loader)] = loader
    return bytes(result[:(cursor + 1023) & ~1023]), {"host_bytes": len(host), "relocations": 1913, "payload_bytes": cursor,
                           "driver_bytes": len(drivers), "bios_bytes": len(bios)}


def validate_boot(boot):
    if len(boot) != 512 or sum(boot) % 256 != 3:
        raise ValueError("The original disk does not have a valid +3 boot sector")
    if digest(boot) != BOOT_SHA256:
        raise ValueError("Unsupported original boot sector (SHA256 %s)" % digest(boot))
    if boot[0x10:0x18] != bytes.fromhex("F3 01 FD 1F 3E 1F ED 79"):
        raise ValueError("Unsupported disk bootstrap")
    if boot[0x136:0x13d] != bytes.fromhex("01 FD 1F 3E 17 ED 79"):
        raise ValueError("Unsupported bootstrap handoff")


def patch_boot(boot, assembler, work):
    validate_boot(boot)
    result = bytearray(boot)
    result[0x10:0x18] = bytes.fromhex("C3 0A FF 00 00 00 00 00")
    initialiser = assemble(assembler, "boot_init", work)
    result[0x10a:0x10a + len(initialiser)] = initialiser
    result[0x13a] = 0x10           # keep the loaded DivMMC page when entering EMS
    # Replace only the error animation that boot_init now occupies.
    result[0xef:0x100] = bytes.fromhex("F3 3E 02 D3 FE 76 18 FD").ljust(17, b"\0")
    result[15] = 0
    result[15] = (3 - sum(result)) & 255
    return bytes(result)


def disable_file(disk, name):
    """Retain a startup file under an unused name without replacing a backup."""
    if not disk.entries(name):
        return None
    stem = name.partition(".")[0]
    backup = stem + ".OLD"
    number = 0
    while disk.entries(backup):
        number += 1
        backup = "%s.%03d" % (stem, number)
    disk.rename_file(name, backup)
    return backup


def patch_disk(data, assembler, work, keep_profile=False):
    disk = Disk(data)
    boot = disk.read(0, 512)
    validate_boot(boot)
    original = disk.read_file(EMS_NAME)
    patched, report = make_ems(original, assembler, work)
    disk.put_file(EMS_NAME, patched)
    fid = make_fid(assembler, work)
    disk.put_file("DIVMMC.FID", fid)
    report["fid_bytes"] = len(fid)
    # The supplied RAMDISC assumes the original +3 memory environment and
    # consumes the entire FID heap. Retain its bytes, but don't auto-install it.
    report["ramdisc_backup"] = disable_file(disk, "RAMDISC.FID")
    disk.write(0, patch_boot(boot, assembler, work))
    # Disable automatic startup without changing the profile's contents.
    # --keep-profile leaves a user's script active.
    report["profile_backup"] = None if keep_profile else disable_file(disk, "PROFILE.SUB")
    report["profile_renamed"] = report["profile_backup"] is not None
    verified = Disk(bytes(disk.data))
    if (verified.read_file(EMS_NAME) != patched or
            not verified.read_file("DIVMMC.FID").startswith(fid) or
            sum(verified.read(0, 512)) % 256 != 3):
        raise ValueError("Patched disk read-back verification failed")
    report.update(input_ems_sha256=digest(original), output_ems_sha256=digest(patched),
                  output_dsk_sha256=digest(disk.data))
    return bytes(disk.data), report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sjasmplus", default="sjasmplus")
    parser.add_argument("--inspect", action="store_true", help="validate the EMS and boot sector without patching")
    parser.add_argument("--keep-profile", action="store_true",
                        help="retain automatic PROFILE.SUB execution, including custom scripts")
    args = parser.parse_args(argv)
    try:
        if args.input.resolve() == args.output.resolve():
            raise ValueError("Output must be different from the original disk")
        original_disk = args.input.read_bytes()
        disk = Disk(original_disk)
        ems = disk.read_file(EMS_NAME)
        extract_components(ems)
        validate_boot(disk.read(0, 512))
        print("Validated %s: %d bytes, SHA256 %s" % (EMS_NAME, len(ems), digest(ems)))
        print("Validated boot sector: SHA256 " + BOOT_SHA256)
        if args.inspect:
            return 0
        assembler = shutil.which(args.sjasmplus)
        if not assembler:
            raise ValueError("sjasmplus was not found; supply --sjasmplus PATH")
        if args.output.exists():
            raise ValueError("Output already exists; choose a new --output path")
        work = ROOT / "build/CPM/patch-build"
        work.mkdir(parents=True, exist_ok=True)
        patched, report = patch_disk(original_disk, assembler, work, args.keep_profile)
        if args.input.read_bytes() != original_disk:
            raise ValueError("Input changed during patching; refusing to publish the output")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation prevents an existing image being overwritten.
        with args.output.open("xb") as target:
            target.write(patched)
        print("Created %s" % args.output.resolve())
        print("EMS SHA256: " + report["output_ems_sha256"])
        print("DSK SHA256: " + report["output_dsk_sha256"])
        if report["profile_renamed"]:
            print("Retained PROFILE.SUB as %s (automatic startup script is disabled)." % report["profile_backup"])
        if report["ramdisc_backup"]:
            print("Retained RAMDISC.FID as %s (original RAM disk driver is disabled)." % report["ramdisc_backup"])
        print("Generated DSK/EMS files contain the original non-distributable software. Keep them local.")
        return 0
    except (OSError, ValueError, DiskError) as error:
        parser.exit(1, "error: %s\n" % error)


if __name__ == "__main__":
    sys.exit(main())

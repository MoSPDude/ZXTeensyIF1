import contextlib
import io
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

CPM = Path(__file__).resolve().parents[1]
ROOT = CPM.parent
sys.path.insert(0, str(CPM))
from disk import Disk, DiskError
import patch_ems


def synthetic_disk(extended=False, interleaved=False):
    data = bytearray(256)
    magic = b"EXTENDED CPC DSK File\r\nDisk-Info\r\n" if extended else b"MV - CPCEMU Disk-File\r\nDisk-Info\r\n"
    data[:len(magic)] = magic
    data[48:50] = bytes((40, 1))
    if extended:
        data[52:92] = bytes((0x13,)) * 40
    else:
        struct.pack_into("<H", data, 50, 0x1300)
    order = [1, 6, 2, 7, 3, 8, 4, 9, 5] if interleaved else range(1, 10)
    for track in range(40):
        header = bytearray(256)
        header[:12] = b"Track-Info\r\n"
        header[16:18] = bytes((track, 0))
        header[20:24] = bytes((2, 9, 0x4e, 0xe5))
        for index, sector in enumerate(order):
            struct.pack_into("<6BH", header, 24 + 8 * index,
                             track, 0, sector, 2, 0, 0, 512 if extended else 0)
        data.extend(header)
        for sector in order:
            data.extend(bytes(((track * 9 + sector) & 255,)) * 512)
    # Seed the logical boot specification and empty directory without Disk.
    data[512:520] = bytes((0, 0, 40, 9, 2, 1, 3, 2))
    for sector in (1, 2, 3, 4):
        physical = list(order).index(sector)
        offset = 256 + 0x1300 + 256 + physical * 512
        data[offset:offset + 512] = b"\xE5" * 512
    return bytes(data)


class ConsoleHarness:
    def __init__(self, work, original, dependencies):
        from verify_boot import Machine, simulator_class, symbols
        self.machine = Machine(Disk(original))
        drivers = (work / "drivers.bin").read_bytes()
        self.font = patch_ems.extract_components(Disk(original).read_file(patch_ems.EMS_NAME))[3][2048:]
        host = (work / "fid_host.bin").read_bytes()
        self.machine.divram[0][:(work / "kernel_data.bin").stat().st_size] = (work / "kernel_data.bin").read_bytes()
        self.machine.divram[2][:len(host)] = host
        self.machine.divram[4][:len(drivers)] = drivers
        self.machine.divram[3][:] = (work / "terminal.bin").read_bytes()
        self.machine.ram[0][0x3a00:] = (work / "resident.bin").read_bytes()
        self.labels = symbols(work / "drivers.sym")
        self.simulator = simulator_class(dependencies)(
            self.machine.memory, registers={"BC": 0x1234, "DE": 0x5678,
                                            "HL": 0x9abc, "IX": 0xdef0, "IY": 0x1357},
            config={"fast_ldir": True, "fast_djnz": True})
        self.simulator.set_tracer(self.machine)
        for port, value in ((0x0f3b, 0xa0), (0x7ffd, 8), (0x00e3, 8)):
            self.machine.write_port(self.simulator.registers, port, value)
        self.machine.memory[0xff00] = 2
        self.driver_call(self.labels['terminal_initialise'])
        self.userf(0xc8, values={0: 0})  # pixel tests use the full 24-row viewport
        self.output('\x1bf')

    def driver_call(self, address, limit=1000000):
        self.select_bank(2)
        registers = self.simulator.registers
        registers[12], registers[24] = 0xfee0, address
        self.machine.memory[0xfee0:0xfee2] = [0, 0xe0]
        for _ in range(limit):
            self.simulator.run()
            if registers[24] == 0xe000:
                return registers.copy()
        raise AssertionError('Driver call did not return')

    def tick(self, count=5):
        for _ in range(count):
            self.driver_call(0x11b)

    def select_bank(self, bank):
        page = (0, 1, 4, 5, 2, 3, 6, 7)[bank]
        self.machine.write_port(self.simulator.registers, 0x7ffd, 8 | ((page & 1) << 4))
        self.machine.write_port(self.simulator.registers, 0x1ffd, (page & 2) << 1)
        self.machine.write_port(self.simulator.registers, 0x00e3, (page & 4) << 1)
        self.machine.memory[0xff00] = bank

    def position(self, row, column):
        self.machine.divram[3][0x218b:0x218d] = [column, row]
        self.machine.memory[0xff1d:0xff1f] = [column, row]

    def cursor(self):
        column, row = self.machine.divram[3][0x218b:0x218d]
        return row, column

    def output(self, text):
        self.select_bank(2)
        for character in text:
            registers = self.simulator.registers
            registers[3] = ord(character)
            preserved = registers[2:12] + registers[16:24]
            registers[12] = 0xfee0
            registers[24] = 0x0109
            self.machine.memory[0xfee0:0xfee2] = [0, 0xe0]
            for _ in range(1000000):  # erasing a screen can draw 51 * 24 blank cells
                self.simulator.run()
                if registers[24] == 0xe000:
                    break
            else:
                raise AssertionError("Console call did not return")
            if (registers[12] != 0xfee2 or registers[2:12] + registers[16:24] != preserved or
                    self.machine.port7 != 8 or self.machine.lower_page != 4):
                raise AssertionError("Console corrupted registers, stack or bank mapping")

    def userf(self, target, bank=2, stack=0x3f00, inline=0x8120, iff=0, values=None):
        self.select_bank(bank)
        registers = self.simulator.registers
        registers[:12] = [0x5a, 0xa5, 0x12, 0x34, 0x56, 0x78,
                          0x9a, 0xbc, 0xde, 0xf0, 0x13, 0x57]
        registers[16:24] = [0x25, 0x36, 0x47, 0x58, 0x69, 0x7a, 0x8b, 0x9c]
        if values:
            for index, value in values.items():
                registers[index] = value
        registers[12], registers[24] = stack, 0xfa5a
        self.simulator.iff = iff
        self.machine.memory[inline:inline + 2] = [target & 255, target >> 8]
        self.machine.memory[stack:stack + 2] = [inline & 255, inline >> 8]
        before = registers.copy()
        for _ in range(1000000):
            self.simulator.run()
            if registers[24] == inline + 2:
                break
        else:
            raise AssertionError("USERF did not return after its inline parameter")
        if (registers[12] != stack + 2 or self.machine.memory[0xff00] != bank or
                self.machine.lower_page != (0, 1, 4, 5, 2, 3, 6, 7)[bank] or
                self.machine.port7 != 8 | ((bank & 1) << 4) or self.simulator.iff != iff):
            raise AssertionError("USERF corrupted stack, bank mapping or interrupt-enable state")
        return before, registers.copy()

    def paint(self, screen, row, column, character):
        # Independent pixel reference: five font bits per cell, in Spectrum's
        # physical bitmap order. Neighbour pixels and attributes must survive.
        for line in range(8):
            y = row * 8 + line
            bits = self.font[ord(character) * 8 + line]
            for pixel in range(5):
                x = column * 5 + pixel
                offset = ((y & 7) << 8) | ((y & 56) << 2) | ((y & 192) << 5) | (x // 8)
                mask = 128 >> (x % 8)
                screen[offset] = (screen[offset] & (255 ^ mask)) | (mask if bits & (128 >> pixel) else 0)
                screen[0x1800 + row * 32 + x // 8] = 15


class DiskTests(unittest.TestCase):
    def test_teensy_lower_8k_requires_e3_after_7ffd(self):
        from verify_boot import Machine
        machine = Machine(Disk(synthetic_disk()))
        machine.divram[0][:] = [0x11] * 16384
        machine.divram[1][:] = [0x22] * 16384
        registers = [0] * 26
        machine.write_port(registers, 0x0f3b, 0xa0)
        machine.write_port(registers, 0x00e3, 0)
        machine.write_port(registers, 0x7ffd, 0x18)
        self.assertEqual(machine.memory[0x1000], 0x11)
        self.assertEqual(machine.memory[0x3000], 0x22)
        machine.write_port(registers, 0x00e3, 0)
        self.assertEqual(machine.memory[0x1000], 0x22)
        self.assertEqual(machine.memory[0x3000], 0x22)
        machine.write_port(registers, 0x7ffd, 8)
        machine.write_port(registers, 0x00e3, 0)
        self.assertEqual(machine.memory[0x1000], 0x11)
        self.assertEqual(machine.memory[0x3000], 0x11)

    def test_standard_and_extended_interleaving(self):
        for extended in (False, True):
            for interleaved in (False, True):
                with self.subTest(extended=extended, interleaved=interleaved):
                    disk = Disk(synthetic_disk(extended, interleaved))
                    self.assertEqual(disk.read(512, 512), bytes((2,)) * 512)
                    disk.write(511, b"ABC")
                    self.assertEqual(disk.read(511, 3), b"ABC")

    def test_fragmented_file_and_extent_order(self):
        disk = Disk(synthetic_disk(interleaved=True))
        payload = bytes(range(256)) * 80
        blocks = list(range(2, 22))[::-1]
        for index, block in enumerate(blocks):
            disk.write(4608 + 1024 * block, payload[index * 1024:(index + 1) * 1024])
        for extent, directory_slot in ((0, 4), (1, 1)):
            entry = bytearray(32)
            entry[1:12] = b"FRAGMENTEMS"
            entry[12] = extent
            entry[15] = 128 if extent == 0 else 32
            selected = blocks[extent * 16:(extent + 1) * 16]
            entry[16:16 + len(selected)] = bytes(selected)
            disk.write(4608 + directory_slot * 32, entry)
        disk = Disk(disk.data)
        self.assertEqual(disk.read_file("FRAGMENT.EMS"), payload)
        replacement = payload.translate(bytes.maketrans(b"\0", b"\xFF"))
        disk.replace_file("FRAGMENT.EMS", replacement)
        self.assertEqual(Disk(disk.data).read_file("FRAGMENT.EMS"), replacement)

    def test_invalid_geometry_truncation_and_bounds(self):
        data = synthetic_disk()
        for value in (data[:200], data[:-1], data[:48] + b"\x50" + data[49:]):
            with self.assertRaises(DiskError):
                Disk(value)
        disk = Disk(data)
        with self.assertRaises(DiskError):
            disk.read(-1, 1)
        with self.assertRaises(DiskError):
            disk.write(40 * 9 * 512, b"x")

    def test_extended_unformatted_trailing_tracks(self):
        original = synthetic_disk(extended=True, interleaved=True)
        for tracks in (41, 42, 204):
            with self.subTest(tracks=tracks):
                data = bytearray(original)
                data[48] = tracks
                disk = Disk(data)
                self.assertEqual(disk.data, data)
                self.assertEqual(disk.read(0, 40 * 9 * 512),
                                 Disk(original).read(0, 40 * 9 * 512))
                with self.assertRaises(DiskError):
                    disk.read(40 * 9 * 512, 1)

    def test_extra_tracks_must_be_unformatted_in_extended_container(self):
        for extended, tracks, extra_size in ((False, 42, 0), (True, 39, 0),
                                              (True, 42, 19), (True, 205, 0)):
            with self.subTest(extended=extended, tracks=tracks, extra_size=extra_size):
                data = bytearray(synthetic_disk(extended=extended))
                data[48] = tracks
                data[93] = extra_size
                with self.assertRaises(DiskError):
                    Disk(data)
        data = bytearray(synthetic_disk(extended=True))
        data[48] = 42
        data[91] = 0  # A missing required track is still an error.
        with self.assertRaises(DiskError):
            Disk(data)

    def test_cross_linked_allocation_rejected(self):
        disk = Disk(synthetic_disk())
        for index, name in enumerate((b"ONE     EMS", b"TWO     COM")):
            entry = bytearray(32)
            entry[1:12] = name
            entry[15] = 8
            entry[16] = 2
            disk.write(4608 + index * 32, entry)
        with self.assertRaisesRegex(DiskError, "cross-linked"):
            Disk(disk.data)

    def test_insufficient_blocks_preserves_existing_files(self):
        disk = Disk(synthetic_disk())
        contents = b"X" * (173 * 1024)
        disk.put_file("FULL.DAT", contents)
        before = bytes(disk.data)
        with self.assertRaisesRegex(DiskError, "Insufficient free disk space"):
            disk.put_file("DIVMMC.FID", b"driver")
        self.assertEqual(disk.data, before)
        self.assertEqual(Disk(disk.data).read_file("FULL.DAT"), contents)

    def test_delete_file_releases_all_extents_blocks_and_directory_entries(self):
        disk = Disk(synthetic_disk())
        payload = b"R" * (17 * 1024)
        disk.put_file("RAMDISC.FID", payload)
        disk.put_file("KEEP.DAT", b"Kept".ljust(1024, b"\0"))
        entries = disk.entries("RAMDISC.FID")
        self.assertEqual(len(entries), 2)
        self.assertTrue(disk.delete_file("RAMDISC.FID"))
        self.assertFalse(disk.delete_file("RAMDISC.FID"))
        disk = Disk(disk.data)
        self.assertFalse(disk.entries("RAMDISC.FID"))
        disk.put_file("NEW.DAT", payload)
        self.assertEqual([(index, entry[12:]) for index, entry in disk.entries("NEW.DAT")],
                         [(index, entry[12:]) for index, entry in entries])
        self.assertEqual(disk.read_file("KEEP.DAT"), b"Kept".ljust(1024, b"\0"))


@unittest.skipUnless(patch_ems.DEFAULT_INPUT.exists() and shutil.which("sjasmplus"),
                     "Requires the user's local disk and sjasmplus")
class LocalDiskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = patch_ems.DEFAULT_INPUT.read_bytes()
        cls.work = ROOT / "build/CPM/tests"
        cls.work.mkdir(parents=True, exist_ok=True)
        cls.patched, cls.report = patch_ems.patch_disk(cls.original, shutil.which("sjasmplus"), cls.work)
        cls.image = cls.work / "patched.dsk"
        cls.image.write_bytes(cls.patched)

    def test_deterministic_patch_and_original_preserved(self):
        second, _ = patch_ems.patch_disk(self.original, shutil.which("sjasmplus"), self.work)
        self.assertEqual(second, self.patched)
        self.assertEqual(patch_ems.DEFAULT_INPUT.read_bytes(), self.original)
        self.assertEqual(len(self.patched), len(self.original))
        self.assertEqual(sum(Disk(self.patched).read(0, 512)) % 256, 3)

    def test_alternative_disk_with_unformatted_trailing_tracks(self):
        path = ROOT / "ignoredfiles/Spectrum CP-M Plus + Mallard BASIC (1988)(Locomotive)(+3)(Side A).dsk"
        if not path.exists():
            self.skipTest("Requires the user's alternative local disk")
        original = path.read_bytes()
        patched, _ = patch_ems.patch_disk(original, shutil.which("sjasmplus"), self.work)
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(len(patched), len(original))
        self.assertEqual(patched[:256], original[:256])
        self.assertEqual(Disk(patched).read(0, 40 * 9 * 512),
                         Disk(self.patched).read(0, 40 * 9 * 512))

    def test_customised_profile_and_full_disk_preserve_user_files(self):
        disk = Disk(self.original)
        profile = b"DIR BASIC.COM\r\n\x1a".ljust(128, b"\x1a")
        extra = b"User data\r\n".ljust(1024, b"\x1a")
        disk.put_file("PROFILE.SUB", profile)
        disk.put_file("USER.DAT", extra)
        occupied = {b for _, entry in disk.directory() for b in entry[16:] if b}
        self.assertEqual(len(occupied), 173)  # FID must use space reclaimed from EMS.
        original = bytes(disk.data)
        for keep_profile in (False, True):
            with self.subTest(keep_profile=keep_profile):
                patched, report = patch_ems.patch_disk(original, shutil.which("sjasmplus"),
                                                       self.work, keep_profile=keep_profile)
                result = Disk(patched)
                self.assertEqual(disk.data, original)
                self.assertEqual(result.read_file("USER.DAT"), extra)
                self.assertEqual(result.read_file("PROFILE.SUB" if keep_profile else "PROFILE.OLD"), profile)
                self.assertEqual(report["profile_renamed"], not keep_profile)
                self.assertTrue(report["ramdisc_deleted"])
                self.assertFalse(result.entries("RAMDISC.FID"))
                occupied = {b for _, entry in result.directory() for b in entry[16:] if b}
                self.assertLess(len(occupied), 173)  # leave working space for SUBMIT
                self.assertEqual(result.read_file("DIVMMC.FID")[:report["fid_bytes"]],
                                 (self.work / "DIVMMC.FID").read_bytes())
                for name in ("BASIC.COM", "RESET.KEY", "RPED.BAS"):
                    self.assertEqual(result.read_file(name), disk.read_file(name))

    def test_existing_backups_are_preserved(self):
        disk = Disk(self.original)
        for index, _ in disk.entries("BASIC.COM"):
            disk.write(4608 + index * 32, b"\xe5" * 32)
        backups = ("PROFILE.OLD", "PROFILE.001", "RAMDISC.OLD", "RAMDISC.001")
        for name in backups:
            disk.put_file(name, name.encode("ascii").ljust(128, b"\x1a"))
        original = bytes(disk.data)
        patched, report = patch_ems.patch_disk(original, shutil.which("sjasmplus"), self.work)
        result = Disk(patched)
        self.assertEqual(disk.data, original)
        self.assertEqual(report["profile_backup"], "PROFILE.002")
        self.assertTrue(report["ramdisc_deleted"])
        self.assertFalse(result.entries("RAMDISC.002"))
        for name in backups:
            self.assertEqual(result.read_file(name), disk.read_file(name))
        self.assertFalse(result.entries("PROFILE.SUB"))
        self.assertFalse(result.entries("RAMDISC.FID"))
        self.assertEqual(result.read_file("PROFILE.002"), disk.read_file("PROFILE.SUB"))

    def test_missing_optional_files_and_removed_utilities_are_accepted(self):
        disk = Disk(self.original)
        for name in ("PROFILE.SUB", "RAMDISC.FID", "BASIC.COM"):
            for index, _ in disk.entries(name):
                disk.write(4608 + index * 32, b"\xe5" * 32)
        patched, report = patch_ems.patch_disk(bytes(disk.data), shutil.which("sjasmplus"), self.work)
        result = Disk(patched)
        self.assertIsNone(report["profile_backup"])
        self.assertFalse(report["ramdisc_deleted"])
        self.assertFalse(report["profile_renamed"])
        for name in ("PROFILE.SUB", "RAMDISC.FID", "BASIC.COM"):
            self.assertFalse(result.entries(name))
        self.assertTrue(result.entries("DIVMMC.FID"))

    def test_existing_divmmc_driver_is_replaced(self):
        disk = Disk(self.original)
        disk.put_file("DIVMMC.FID", b"Previous driver".ljust(128, b"\0"))
        patched, report = patch_ems.patch_disk(bytes(disk.data), shutil.which("sjasmplus"), self.work)
        self.assertEqual(Disk(patched).read_file("DIVMMC.FID")[:report["fid_bytes"]],
                         (self.work / "DIVMMC.FID").read_bytes())

    def test_no_directory_space_creates_no_output(self):
        disk = Disk(self.original)
        disk.delete_file("RAMDISC.FID")
        for index in range(64):
            if disk.read(4608 + index * 32, 1) == b"\xe5":
                entry = bytearray(32)
                entry[1:12] = ("EMPTY%03dTXT" % index).encode("ascii")
                disk.write(4608 + index * 32, entry)
        original = bytes(disk.data)
        Disk(original)
        with tempfile.TemporaryDirectory(dir=str(self.work)) as directory:
            source, output = Path(directory) / "input.dsk", Path(directory) / "output.dsk"
            source.write_bytes(original)
            result = subprocess.run([sys.executable, str(CPM / "patch_ems.py"), str(source),
                                     "--output", str(output)],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b"Insufficient free disk space or directory entries", result.stdout)
            self.assertFalse(output.exists())
            self.assertEqual(source.read_bytes(), original)

    def test_ramdisc_directory_slot_is_reused_on_full_directory(self):
        disk = Disk(self.original)
        for index in range(64):
            if disk.read(4608 + index * 32, 1) == b"\xe5":
                entry = bytearray(32)
                entry[1:12] = ("EMPTY%03dTXT" % index).encode("ascii")
                disk.write(4608 + index * 32, entry)
        original = bytes(disk.data)
        self.assertEqual(len(list(disk.directory())), 64)
        patched, report = patch_ems.patch_disk(original, shutil.which("sjasmplus"), self.work)
        result = Disk(patched)
        self.assertTrue(report["ramdisc_deleted"])
        self.assertFalse(result.entries("RAMDISC.FID"))
        self.assertEqual([index for index, _ in result.entries("DIVMMC.FID")],
                         [index for index, _ in disk.entries("RAMDISC.FID")])
        self.assertEqual(disk.data, original)

    def test_inspect_validates_boot_sector_as_well_as_ems(self):
        disk = Disk(self.original)
        disk.put_file("PROFILE.SUB", b"Custom startup script".ljust(128, b"\x1a"))
        with tempfile.TemporaryDirectory(dir=str(self.work)) as directory:
            source = Path(directory) / "custom.dsk"
            source.write_bytes(disk.data)
            command = [sys.executable, str(CPM / "patch_ems.py"), str(source), "--inspect"]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn(b"Validated boot sector", result.stdout)
            boot = bytearray(disk.read(0, 512))
            boot[100] ^= 1
            boot[15] = (boot[15] + 3 - sum(boot)) & 255
            disk.write(0, boot)
            source.write_bytes(disk.data)
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b"Unsupported original boot sector", result.stdout)

    def test_only_owned_file_allocations_boot_and_directory_change(self):
        original, patched = Disk(self.original), Disk(self.patched)
        logical_allowed = set(range(512)) | set(range(4608, 6656))
        for disk, name in ((original, patch_ems.EMS_NAME),
                           (patched, patch_ems.EMS_NAME), (patched, "DIVMMC.FID")):
            for _, entry in disk.entries(name):
                for block in (b for b in entry[16:] if b):
                    logical_allowed.update(range(4608 + block * 1024, 4608 + (block + 1) * 1024))
        allowed = {original.sectors[n // 512] + n % 512 for n in logical_allowed}
        changed = {n for n, (a, b) in enumerate(zip(self.original, self.patched)) if a != b}
        self.assertTrue(changed)
        self.assertFalse(changed - allowed)
        names = {(bytes(c & 127 for c in e[1:9]).decode().rstrip() + "." +
                  bytes(c & 127 for c in e[9:12]).decode().rstrip()) for _, e in original.directory()}
        for name in names - {patch_ems.EMS_NAME, "RAMDISC.FID"}:
            target = {"PROFILE.SUB": "PROFILE.OLD"}.get(name, name)
            self.assertEqual(original.read_file(name), patched.read_file(target), name)
        self.assertFalse(patched.entries("RAMDISC.FID"))
        self.assertFalse(patched.entries("RAMDISC.OLD"))
        self.assertLess(len(patched.read_file(patch_ems.EMS_NAME)), 32640)
        Disk(patched.data)  # validate the new allocation lists for cross-links

    def test_unknown_and_already_patched_input_rejected(self):
        original = Disk(self.original).read_file(patch_ems.EMS_NAME)
        with self.assertRaisesRegex(ValueError, "Unsupported"):
            patch_ems.extract_components(original[:-1])
        with self.assertRaisesRegex(ValueError, "Unsupported"):
            patch_ems.extract_components(Disk(self.patched).read_file(patch_ems.EMS_NAME))
        boot = bytearray(Disk(self.original).read(0, 512))
        boot[100] ^= 1
        boot[15] = (boot[15] + 3 - sum(boot)) & 255
        with self.assertRaisesRegex(ValueError, "Unsupported"):
            patch_ems.patch_boot(bytes(boot), shutil.which("sjasmplus"), self.work)

    def test_cli_refuses_overwriting_any_image(self):
        output = self.work / "existing.dsk"
        output.write_bytes(b"existing image")
        for target in (output, patch_ems.DEFAULT_INPUT):
            before = target.read_bytes()
            result = subprocess.run([sys.executable, str(CPM / "patch_ems.py"),
                                     str(patch_ems.DEFAULT_INPUT), "-o", str(target)],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(target.read_bytes(), before)

    def console(self):
        dependencies = ROOT / "build/CPM/deps"
        if not (dependencies / "skoolkit").exists():
            self.skipTest("Install SkoolKit 8.10 in build/CPM/deps for CPU integration testing")
        return ConsoleHarness(self.work, self.original, dependencies)

    def test_cpu_narrow_console_pixels_and_edges(self):
        console = self.console()
        background = bytearray((index * 37 + 11) & 255 for index in range(16384))
        # Every printable glyph at every bit alignment, then the remaining
        # columns including the right edge, across all three screen thirds.
        cases = [(character, column) for character in range(32, 127) for column in range(8)]
        cases.extend((ord('M'), column) for column in range(8, 51))
        rows = (0, 7, 8, 15, 16, 22)
        for index, (character, column) in enumerate(cases):
            row = rows[index % len(rows)]
            with self.subTest(character=character, row=row, column=column):
                console.machine.ram[7][:] = background
                console.position(row, column)
                expected = background.copy()
                console.paint(expected, row, column, chr(character))
                console.output(chr(character))
                self.assertEqual(bytes(console.machine.ram[7]), expected)
                self.assertEqual(console.cursor(), (row, column + 1) if column < 50 else (row + 1, 0))
                # A space replaces all five pixels, including any old wide
                # glyph, without erasing either adjacent character.
                console.position(row, column)
                console.paint(expected, row, column, ' ')
                console.output(' ')
                self.assertEqual(bytes(console.machine.ram[7]), expected)

    def test_cpu_narrow_console_controls_and_scroll(self):
        console = self.console()
        console.output('\x1bE\x1bH')
        self.assertEqual(console.cursor(), (0, 0))
        self.assertEqual(console.machine.ram[7][:0x1800], [0] * 0x1800)
        self.assertEqual(console.machine.ram[7][0x1800:0x1b00], [15] * 768)
        console.output('MW\b \tA\rB\nC')
        expected = bytearray(16384)
        expected[0x1800:0x1b00] = bytes((15,)) * 768
        for row, column, character in ((0, 0, 'B'), (0, 2, 'A'), (1, 1, 'C')):
            console.paint(expected, row, column, character)
        self.assertEqual(bytes(console.machine.ram[7]), expected)
        self.assertEqual(console.cursor(), (1, 2))
        console.position(2, 49)
        console.output('\t')
        self.assertEqual(console.cursor(), (2, 49))  # native CRT ignores TAB
        console.output('\b')
        self.assertEqual(console.cursor(), (2, 48))
        # Fill a complete screen: output in the 51st cell of the last row
        # must wrap and scroll one character row without touching attributes.
        console.output('\x1bE\x1bH')
        expected[:0x1800] = bytes(0x1800)
        for row in range(24):
            character = chr(ord('A') + row)
            console.output(character * 51)
            if row:
                for column in range(51):
                    console.paint(expected, row - 1, column, character)
        self.assertEqual(console.cursor(), (23, 0))
        self.assertEqual(bytes(console.machine.ram[7]), expected)
        console.output('\x1bE\x1bH')
        self.assertEqual(console.cursor(), (0, 0))
        self.assertEqual(console.machine.ram[7][:0x1800], [0] * 0x1800)

    def test_cpu_terminal_escape_sequences(self):
        console = self.console()
        console.output('\x1bE\x1bH')
        console.position(7, 10)
        before = bytes(console.machine.ram[7])
        console.output('\x1bA\x1bD\x1bB\x1bC')
        self.assertEqual(console.cursor(), (7, 10))
        self.assertEqual(bytes(console.machine.ram[7]), before)
        for row, column in ((0, 0), (0, 50), (23, 0), (23, 50)):
            console.position(row, column)
            console.output('\x1b' + ('A' if row == 0 else 'B'))
            console.output('\x1b' + ('D' if column == 0 else 'C'))
            self.assertEqual(console.cursor(), (row, column))
            self.assertEqual(bytes(console.machine.ram[7]), before)
        console.output('\x1bY' + chr(32 + 7) + chr(32 + 10) + '\x1bj')
        self.assertEqual(console.cursor(), (7, 10))
        console.output('\x1bY' + chr(255) + chr(255))
        self.assertEqual(console.cursor(), (23, 50))
        console.output('\x1bk')
        self.assertEqual(console.cursor(), (7, 10))
        # Fill all bitmap bits, including neighbours outside each five-pixel
        # cell, and ensure erasure starts exactly at the selected character.
        console.machine.ram[7][:0x1800] = [255] * 0x1800
        expected = bytearray(console.machine.ram[7])
        for column in range(10, 51):
            console.paint(expected, 7, column, ' ')
        console.output('\x1bK')
        self.assertEqual(console.cursor(), (7, 10))
        self.assertEqual(bytes(console.machine.ram[7]), expected)
        for row in range(8, 24):
            for column in range(51):
                console.paint(expected, row, column, ' ')
        console.output('\x1bJ')
        self.assertEqual(console.cursor(), (7, 10))
        self.assertEqual(bytes(console.machine.ram[7]), expected)
        for row in range(8):
            for column in range(51):
                console.paint(expected, row, column, ' ')
        console.output('\x1bE')
        self.assertEqual(console.cursor(), (7, 10))
        self.assertEqual(bytes(console.machine.ram[7]), expected)  # unused x=255 survives
        console.output('\x1bH')
        self.assertEqual(console.cursor(), (0, 0))

    def test_cpu_userf_cursor_query_and_inline_parameter(self):
        console = self.console()
        console.position(7, 10)
        console.output('X')
        machine, simulator = console.machine, console.simulator
        for bank, page in ((0, 0), (1, 1), (2, 4)):
            for stack in (0x3f00, 0xf3e0):
                with self.subTest(bank=bank, stack=stack):
                    machine.write_port(simulator.registers, 0x7ffd, 8 | ((page & 1) << 4))
                    machine.write_port(simulator.registers, 0x00e3, (page & 4) << 1)
                    machine.memory[0xff00] = bank
                    machine.memory[0x8120:0x8122] = [0xbf, 0]  # inline TE ASK address
                    machine.memory[stack:stack + 2] = [0x20, 0x81]
                    registers = simulator.registers
                    registers[0:2] = [0x5a, 0xa5]
                    registers[12], registers[24] = stack, 0xfa5a
                    preserved = registers[0:2] + registers[8:12] + registers[16:24]
                    for _ in range(2000):
                        simulator.run()
                        if registers[24] == 0x8122:
                            break
                    else:
                        self.fail("USERF did not skip its two-byte inline parameter")
                    self.assertEqual(registers[12], stack + 2)
                    self.assertEqual(registers[0:2] + registers[8:12] + registers[16:24], preserved)
                    self.assertEqual(registers[2:8], [0, 0, 23, 50, 7, 11])
                    self.assertEqual(machine.memory[0xff00], bank)
                    self.assertEqual(machine.lower_page, page)

    def test_cpu_mallard_basic_command_entry_and_editing(self):
        console = self.console()
        from verify_boot import verify
        commands = "basic\rprint 2+2\rprint 2+3\x01\x072\rsystem\r"
        with contextlib.redirect_stdout(io.StringIO()):
            machine, transcript, _ = verify(self.image, ROOT / "build/CPM/deps", 8000000, commands)
        self.assertIn("print 2+2\r\n 4 \r\nOk", transcript)
        self.assertIn("print 2+3\x1bD\x1bK2\r\n 4 \r\nOk", transcript)
        self.assertNotIn("\x1bA", transcript)  # the old TE ASK stub caused spurious cursor-up commands
        self.assertNotIn("Syntax error", transcript)
        # Check the actual bitmap as well as BASIC's numeric result: neither
        # escape letters nor the overwritten '3' may remain in the input line.
        expected = bytearray(16384)
        for row in (13, 16):
            for column, character in enumerate("print 2+2"):
                console.paint(expected, row, column, character)
            for line in range(row * 8, row * 8 + 8):
                offset = ((line & 7) << 8) | ((line & 56) << 2) | ((line & 192) << 5)
                self.assertEqual(bytes(machine.ram[7][offset:offset + 32]), expected[offset:offset + 32])

    def test_cpu_keyboard_matrix_caps_digits_and_key_release(self):
        console = self.console()
        machine, simulator = console.machine, console.simulator

        def call(address):
            registers = simulator.registers
            registers[12], registers[24] = 0xfee0, address
            machine.memory[0xfee0:0xfee2] = [0, 0xe0]
            # Keep a physical key held for the whole call. The test controls
            # release explicitly instead of using the harness's timed release.
            machine.keyboard_start = registers[25]
            for _ in range(10000):
                simulator.run()
                if registers[24] == 0xe000:
                    return registers[0]
            self.fail("Keyboard call did not return")

        # Independent physical switch positions, in digit order 0..9.
        switches = ((4, 0), (3, 0), (3, 1), (3, 2), (3, 3),
                    (3, 4), (4, 4), (4, 3), (4, 2), (4, 1))
        shifted = (127, 23, None, 19, 9, 1, 30, 31, 6, 7)
        cases = [(str(digit), {switch}, ord(str(digit))) for digit, switch in enumerate(switches)]
        cases += [("CAPS+%d" % digit, {(0, 0), switch}, shifted[digit])
                  for digit, switch in enumerate(switches)]
        cases += [("CAPS+A", {(0, 0), (1, 0)}, ord('A')),
                  ("SYMBOL+M", {(7, 1), (7, 2)}, ord('.')),
                  ("CAPS+SYMBOL+C", {(0, 0), (7, 1), (0, 3)}, 3),
                  ("CAPS+SPACE", {(0, 0), (7, 0)}, 27)]
        for name, pressed, expected in cases:
            with self.subTest(key=name):
                machine.keyboard = pressed
                call(0x011b)  # real +3 scanning happens on the 50 Hz interrupt
                status = 0 if expected is None else 255
                self.assertEqual(call(0x0103), status)
                self.assertEqual(call(0x0103), status)  # peeking must retain the pending key
                if expected is not None:
                    self.assertEqual(call(0x0106), expected)
                    self.assertEqual(call(0x0103), 0)  # no repeat while still held
                machine.keyboard = None
                call(0x011b)
                self.assertEqual(call(0x0103), 0)
        # A queued arrow survives release before the input routine consumes it.
        machine.keyboard = {(0, 0), (3, 4)}
        call(0x011b)
        self.assertEqual(call(0x0103), 255)
        machine.keyboard = None
        self.assertEqual(call(0x0103), 255)
        self.assertEqual(call(0x0106), 1)
        self.assertEqual(call(0x0103), 0)

    def test_cpu_mallard_basic_physical_cursor_and_delete_keys(self):
        self.console()  # check CPU-test dependencies before running the boot
        from verify_boot import verify
        commands = list("basic\rprint 2+3") + ["DELETE"] + list("2\r")
        commands += list("print 2+3") + ["LEFT", "DELETE_RIGHT"] + list("2\r")
        commands += list("print 2+3") + ["LEFT", "LEFT", "RIGHT", "DELETE_RIGHT"] + list("2\r")
        # Move between rows of a wrapped input line, then remove its last
        # character. Every event uses actual CAPS+digit matrix switches.
        commands += list("print " + ' ' * 48 + "2+3") + ["UP", "DOWN", "DELETE"] + list("2\r")
        # DEL must also retain the CP/M command editor's rubout behaviour.
        commands += list("system\rdix") + ["DELETE"] + list("r\r")
        with contextlib.redirect_stdout(io.StringIO()):
            # Native spin-up waits add instructions to the final DIR load.
            machine, transcript, _ = verify(self.image, ROOT / "build/CPM/deps", 12000000, commands)
        self.assertEqual(transcript.count("\r\n 4 \r\n"), 4)
        self.assertNotIn("Syntax error", transcript)
        self.assertIn("\x1bD", transcript)
        self.assertIn("\x1bC", transcript)
        self.assertIn("\x1bA", transcript)
        self.assertIn("\x1bB", transcript)
        self.assertIn("A: S10CPM3", transcript)
        self.assertNotIn("CP/M Error", transcript)

    def test_cpu_xbios_calls_preserve_registers_banks_and_interrupts(self):
        # Contracts from the +3 manual. Registers outside each call's explicit
        # outputs/clobbers, including index and alternate registers, survive.
        calls = ((0xbf, {2, 3, 4, 5, 6, 7}),
                 (0xc5, {0, 1}), (0xd1, set()),
                 (0xe3, {0, 1, 2, 3, 6, 7}),
                 (0xec, set(range(8))), (0xce, set(range(8))),
                 (0xc2, set(range(8))))
        for bank in range(3):
            for stack in (0x3f00, 0xf3e0):
                for iff in (0, 1):
                    console = self.console()
                    console.position(7, 10)
                    console.output('X')
                    for target, clobbered in calls:
                        with self.subTest(target=hex(target), bank=bank, stack=stack, iff=iff):
                            before, after = console.userf(target, bank, stack, iff=iff)
                            for index in list(range(12)) + list(range(16, 24)):
                                if index not in clobbered:
                                    self.assertEqual(after[index], before[index], "register %d" % index)
                            if target == 0xbf:
                                self.assertEqual(after[2:8], [0, 0, 23, 50, 7, 11])
                            elif target == 0xc5:
                                self.assertEqual(after[0], 0)
                                self.assertEqual(after[1] & 0x41, 0x40)  # Z=1, carry=0
                            elif target == 0xe3:
                                self.assertEqual(after[0], 3)
                                self.assertEqual(after[2:4], [0, 10])
                                self.assertEqual(after[6:8], [0, 10])
                            elif target == 0xec:
                                self.assertFalse(after[1] & 1)
                            elif target == 0xc2:
                                self.assertEqual(console.machine.memory[0xff1d:0xff1f], [0, 0])

    def test_cpu_xbios_border_all_colour_encodings(self):
        console = self.console()
        screen = bytes(console.machine.ram[7])
        for colour in range(256):
            with self.subTest(colour=colour):
                start = len(console.machine.ports)
                console.userf(0xce, values={2: colour})
                writes = [(port, value) for _, port, value in console.machine.ports[start:]
                          if port & 255 == 0xfe]
                expected = bool(colour & 3) | (bool(colour & 12) << 1) | (bool(colour & 48) << 2)
                self.assertEqual(len(writes), 1)
                self.assertEqual(writes[0][1], expected | 8)
                self.assertEqual(bytes(console.machine.ram[7]), screen)

    def test_cpu_xbios_reset_restores_terminal_without_touching_tpa(self):
        console = self.console()
        console.output('old screen\x1bj\x1bY!')  # leave an incomplete cursor-position sequence
        console.machine.ram[7][:] = [0xaa] * 16384  # simulate writes outside the CRT device
        console.machine.memory[0x4000:0xf400] = [0xa5] * 0xb400
        console.machine.memory[0xf9b6] = 0
        console.machine.memory[0xf9b8] = 0
        console.userf(0xc2, bank=1, stack=0x3000, inline=0x3fff)
        self.assertEqual(console.machine.memory[0x4001:0xf400], [0xa5] * 0xb3ff)
        # The inline argument crosses 3FFF/4000; its high byte is the only
        # intentional caller write to shared TPA.
        self.assertEqual(console.machine.memory[0x4000], 0)
        self.assertEqual(console.machine.memory[0xf9b6], 50)
        self.assertEqual(console.machine.memory[0xf9b8], 23)
        engine = console.machine.divram[3]
        self.assertEqual(engine[0x2178:0x217a], [0x2a, 0x10])  # native parser idle
        self.assertEqual(engine[0x218b:0x218f], [0, 0, 0, 0])
        self.assertEqual(engine[0x2180:0x2182], [255, 0])  # enabled; delayed cursor
        expected = bytearray(16384)
        expected[0x1800:0x1b00] = bytes((15,)) * 768
        expected[0x1b00:] = bytes((0xaa,)) * (16384 - 0x1b00)
        self.assertEqual(bytes(console.machine.ram[7]), expected)
        self.assertEqual([value & 7 for _, port, value in console.machine.ports if port & 255 == 0xfe][-1], 1)
        console.output('\x1bfX')  # parser reset accepts ESC f; X is drawn without an overlay
        expected[:0x1800] = bytes(0x1800)
        console.paint(expected, 0, 0, 'X')
        self.assertEqual(bytes(console.machine.ram[7]), expected)
        self.assertEqual(console.cursor(), (0, 1))

    def test_cpu_cursor_overlay_restores_pixels_at_all_alignments_and_edges(self):
        console = self.console()
        background = bytearray((index * 37 + 11) & 255 for index in range(16384))
        for column in range(51):
            row = (0, 7, 8, 15, 16, 23)[column % 6]
            with self.subTest(row=row, column=column):
                console.machine.ram[7][:] = background
                console.position(row, column)
                console.output('\x1be')
                console.tick()
                expected = background.copy()
                for y in range(row * 8, row * 8 + 8):
                    for x in range(column * 5, column * 5 + 5):
                        offset = ((y & 7) << 8) | ((y & 56) << 2) | ((y & 192) << 5) | (x // 8)
                        expected[offset] ^= 128 >> (x % 8)
                self.assertEqual(bytes(console.machine.ram[7]), expected)
                console.output('\x1be')  # output delays the steady cursor again
                console.tick()
                self.assertEqual(bytes(console.machine.ram[7]), expected)
                console.output('\x1bf')
                self.assertEqual(bytes(console.machine.ram[7]), background)
                console.output('\x1bf')
                self.assertEqual(bytes(console.machine.ram[7]), background)
        # An overlay must not become part of a glyph when output, cursor
        # movement, clearing or bottom-row scrolling changes the display.
        console.userf(0xc2)
        console.output('\x1beAB\x1bD')
        console.output('\x1bf')
        expected = bytearray(16384)
        expected[0x1800:0x1b00] = bytes((15,)) * 768
        expected[0x1b00:] = background[0x1b00:]
        console.paint(expected, 0, 0, 'A')
        console.paint(expected, 0, 1, 'B')
        self.assertEqual(bytes(console.machine.ram[7]), expected)
        console.output('\x1bY' + chr(32 + 23) + chr(32 + 50) + '\x1beX\x1bf')
        expected[:0x1800] = bytes(0x1800)
        console.paint(expected, 22, 50, 'X')
        self.assertEqual(bytes(console.machine.ram[7]), expected)

    def test_cpu_boot_programs_tpa_write_and_reboot(self):
        dependencies = ROOT / "build/CPM/deps"
        if not (dependencies / "skoolkit").exists():
            self.skipTest("Install SkoolKit 8.10 in build/CPM/deps for CPU integration testing")
        from verify_boot import verify
        binary = self.work / "ramtest.bin"
        subprocess.run([shutil.which("sjasmplus"), "--nologo", "--raw=" + str(binary),
                        str(CPM / "tests/ramtest.asm")], check=True, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT)
        disk = Disk(self.patched)
        disk.rename_file("RPED.SUB", "RAMTEST.COM")
        disk.replace_file("RAMTEST.COM", binary.read_bytes())
        image = self.work / "ramtest.dsk"
        image.write_bytes(disk.data)
        before = image.read_bytes()
        commands = "ramtest\rdump profile.old\rshow\rpip probe.txt=profile.old\rtype probe.txt\r"
        with contextlib.redirect_stdout(io.StringIO()):
            machine, transcript, _ = verify(image, dependencies, 15000000, commands)
        self.assertIn("RAM TEST PASS", transcript)
        self.assertNotIn("RAM TEST FAIL", transcript)
        self.assertIn("DUMP - Version 3.0", transcript)
        self.assertIn("A: RW", transcript)
        self.assertEqual(machine.memory[0xf9b6], 50)  # SCB width agrees with the renderer
        self.assertNotIn("ERROR:", transcript)
        self.assertNotIn("CP/M Error", transcript)
        self.assertGreater(len(machine.writes), 0)
        expected = Disk(self.original).read_file("PROFILE.SUB").split(b"\x1A")[0]
        self.assertEqual(machine.disk.read_file("PROBE.TXT").split(b"\x1A")[0], expected)
        self.assertEqual(image.read_bytes(), before)  # emulated writes stay in RAM
        reboot = self.work / "after-write.dsk"
        reboot.write_bytes(machine.disk.data)
        with contextlib.redirect_stdout(io.StringIO()):
            machine, transcript, _ = verify(reboot, dependencies, 10000000,
                                             "type probe.txt\rera probe.txt\rdir probe.txt\r")
        self.assertIn("SETDEF *,C:", transcript)
        self.assertIn("No File", transcript)
        self.assertNotIn("CP/M Error", transcript)
        with self.assertRaises(DiskError):
            machine.disk.read_file("PROBE.TXT")


if __name__ == "__main__":
    unittest.main()

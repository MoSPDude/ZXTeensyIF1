"""Two-drive, native format, media-swap and error-recovery CPU checks."""
import contextlib
import io
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import unittest

CPM = Path(__file__).resolve().parents[1]
ROOT = CPM.parent
sys.path.insert(0, str(CPM))
import patch_ems
from disk import Disk
from floppy_model import Floppy
from verify_boot import verify, symbols, simulator_class


class Media(Floppy):
    """Independent CP/M file layout inside a synthetic standard/extended DSK."""
    def __init__(self, kind=2, sidedness=None, tracks=None, text=b'PROBE\r\n', extended=False):
        specs = ((0, 0, 40, 9, 2, 1, 3, 2), (1, 0, 40, 9, 2, 2, 3, 2),
                 (2, 0, 40, 9, 2, 0, 3, 2), (3, 129, 80, 9, 2, 1, 4, 4))
        self.spec = bytearray(specs[kind] + (42, 82) + (0,) * 6)
        if sidedness is not None:
            self.spec[1] = sidedness
        if tracks is not None:
            self.spec[2] = tracks
        if self.spec[2] > 40:
            self.spec[6] = max(4, self.spec[6])
        self.sides = 1 if self.spec[1] & 127 == 0 else 2
        self.first = (1, 65, 193, 1)[kind]
        cylinders = self.spec[2]
        data = bytearray(256)
        magic = b'EXTENDED CPC DSK File\r\nDisk-Info\r\n' if extended else b'MV - CPCEMU Disk-File\r\nDisk-Info\r\n'
        data[:len(magic)] = magic
        data[48:50] = bytes((cylinders, self.sides))
        if extended:
            data[52:52 + cylinders * self.sides] = bytes((19,)) * (cylinders * self.sides)
        else:
            struct.pack_into('<H', data, 50, 4864)
        for cylinder in range(cylinders):
            for head in range(self.sides):
                header = bytearray(256)
                header[:12] = b'Track-Info\r\n'
                header[16:18] = bytes((cylinder, head))
                header[20:24] = bytes((2, 9, 82, 229))
                # Interleaved IDs exercise address-based access, not file order.
                order = (0, 5, 1, 6, 2, 7, 3, 8, 4)
                for i, sector in enumerate(order):
                    struct.pack_into('<6BH', header, 24 + i * 8, cylinder, head,
                                     self.first + sector, 2, 0, 0, 512 if extended else 0)
                data.extend(header)
                data.extend(bytes((229,)) * 4608)
        super().__init__(data)
        if kind in (0, 3):
            self.write_sector(0, 0, 1, bytes(self.spec).ljust(512, b'\xe5'))
        self.block_size = 128 << self.spec[6]
        self.directory_offset = self.spec[5] * 4608
        self.directory_size = self.spec[7] * self.block_size
        self.dsm = ((cylinders * self.sides - self.spec[5]) * 4608) // self.block_size - 1
        self.logical_write(self.directory_offset, bytes((229,)) * self.directory_size)
        if text is not None:
            entry = bytearray(32)
            entry[1:12] = b'PROBE   TXT'
            entry[15] = (len(text) + 127) // 128
            first_block = self.spec[7]
            if self.dsm < 256:
                entry[16] = first_block
            else:
                struct.pack_into('<H', entry, 16, first_block)
            self.logical_write(self.directory_offset, entry)
            self.logical_write(self.directory_offset + first_block * self.block_size,
                               text.ljust(entry[15] * 128, b'\x1a'))

    def address(self, offset):
        track, within = divmod(offset, 4608)
        if self.sides == 1:
            cylinder, head = track, 0
        elif self.spec[1] & 127 == 1:
            cylinder, head = divmod(track, 2)
        elif track < self.spec[2]:
            cylinder, head = track, 0
        else:
            cylinder, head = self.spec[2] * 2 - track - 1, 1
        sector, byte = divmod(within, 512)
        return cylinder, head, self.first + sector, byte

    def logical_write(self, offset, data):
        position = 0
        while position < len(data):
            cylinder, head, sector, byte = self.address(offset + position)
            record = self.sectors[cylinder, head, sector]
            count = min(512 - byte, len(data) - position)
            self.data[record[0] + byte:record[0] + byte + count] = data[position:position + count]
            position += count

    def logical_read(self, offset, length):
        result = bytearray()
        while len(result) < length:
            cylinder, head, sector, byte = self.address(offset + len(result))
            record = self.sectors[cylinder, head, sector]
            count = min(512 - byte, length - len(result))
            result.extend(self.data[record[0] + byte:record[0] + byte + count])
        return bytes(result)

    def read_file(self, name):
        stem, _, suffix = name.upper().partition('.')
        wanted = (stem.ljust(8) + suffix.ljust(3)).encode('ascii')
        directory = self.logical_read(self.directory_offset, self.directory_size)
        chunks = []
        extent_mask = (1 << (self.spec[6] - (3 if self.dsm < 256 else 4))) - 1
        for offset in range(0, len(directory), 32):
            entry = directory[offset:offset + 32]
            if entry[0] or bytes(c & 127 for c in entry[1:12]) != wanted:
                continue
            size = (entry[12] & extent_mask) * 16384 + entry[15] * 128
            payload = bytearray()
            blocks = entry[16:] if self.dsm < 256 else struct.unpack('<8H', entry[16:])
            for block in blocks:
                count = min(self.block_size, size - len(payload))
                if not count:
                    break
                if not self.spec[7] <= block <= self.dsm:
                    raise AssertionError('Invalid allocation')
                payload.extend(self.logical_read(self.directory_offset + block * self.block_size, count))
            chunks.append((((entry[12] & 31) | entry[14] * 32) & ~extent_mask, bytes(payload)))
        if not chunks:
            raise AssertionError('File missing: ' + name)
        return b''.join(data for _, data in sorted(chunks))


class FloppyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dependencies = ROOT / 'build/CPM/deps'
        if not patch_ems.DEFAULT_INPUT.exists() or not shutil.which('sjasmplus') or not (cls.dependencies / 'skoolkit').exists():
            raise unittest.SkipTest('Local original disk, sjasmplus and SkoolKit required')
        cls.temporary = tempfile.TemporaryDirectory(prefix='floppy-tests-', dir=str(ROOT / 'build/CPM'))
        cls.work = Path(cls.temporary.name)
        cls.original = patch_ems.DEFAULT_INPUT.read_bytes()
        patched, _ = patch_ems.patch_disk(cls.original, shutil.which('sjasmplus'), cls.work)
        cls.image = cls.work / 'boot.dsk'
        cls.image.write_bytes(patched)
        cls.labels = symbols(cls.work / 'drivers.sym')

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def boot(self, commands='', disk=None, observe=None, fitted=False, limit=14000000):
        with contextlib.redirect_stdout(io.StringIO()):
            return verify(self.image, self.dependencies, limit, commands, observe,
                          drive_b=disk, drive_b_fitted=fitted)

    def prompt(self, machine, cpu):
        waiting = (machine.memory[0xff00] == 2 and machine.memory[cpu.registers[24]] == 0x76 and
                   machine.memory[0xff09] and machine.memory[self.labels['floppy_busy']])
        if waiting:
            self.assertFalse(machine.motor_on, 'Recovery must allow firmware remounting')
        return waiting

    def test_startup_reports_physical_drive_count(self):
        for fitted, summary in ((False, '61K TPA, 1 floppy drive\r\n'),
                                (True, '61K TPA, 2 floppy drives\r\n')):
            with self.subTest(fitted=fitted):
                machine, text, _ = self.boot(fitted=fitted)
                self.assertEqual(text.count(summary), 1)
                self.assertLess(text.index(summary), text.index('A>'))
                self.assertEqual(machine.memory[0xff20], 2 if fitted else 1)

    def test_read_write_two_drives_and_native_formats(self):
        cases = ((1, None, None), (2, None, None), (0, 128, 80), (3, 129, None), (3, 130, None))
        expected = Disk(self.original).read_file('BASIC.COM')
        for kind, sides, tracks in cases:
            with self.subTest(kind=kind, sides=sides, tracks=tracks):
                media = Media(kind, sides, tracks, extended=kind == 3)
                machine, text, _ = self.boot('dir b:\rtype b:probe.txt\rpip b:=a:basic.com\r', media)
                self.assertIn('B: PROBE', text)
                self.assertIn('PROBE\r\n', text)
                self.assertNotIn('Error', text)
                self.assertEqual(media.read_file('BASIC.COM'), expected)
                self.assertEqual(machine.memory[0xff20], 2)
                reboot, text, _ = self.boot('dir b:\rtype b:probe.txt\r', media)
                self.assertIn('B: PROBE', text)
                self.assertNotIn('Error', text)

    def test_retry_missing_disk_and_cancel_read_errors(self):
        mounted = []
        def insert(machine, cpu):
            if self.prompt(machine, cpu) and not mounted:
                mounted.append(True)
                machine.mount(1, Media())
        machine, text, _ = self.boot('dir b:\rr', observe=insert, fitted=True)
        self.assertTrue(mounted)
        self.assertIn('B: PROBE', text)
        self.assertNotIn('CP/M Error', text)
        failures = []
        def crc(machine, cpu):
            if not failures:
                machine.fdc_failures.append({'unit': 1, 'opcode': 6, 'count': 15})
                failures.append(True)
        machine, text, _ = self.boot('dir b:\rc', Media(), crc)
        self.assertIn('CP/M Error On B:', text)
        self.assertEqual(machine.fdc_failures[0]['count'], 0)

    def test_ignore_read_error_and_retry_write_protection(self):
        def crc(machine, cpu):
            if not machine.fdc_failures:
                machine.fdc_failures.append({'unit': 1, 'opcode': 6, 'count': 15})
        machine, text, _ = self.boot('dir b:\ri', Media(), crc)
        self.assertEqual(machine.fdc_failures[0]['count'], 0)
        self.assertNotIn('CP/M Error', text)
        state = [False, False]
        media = Media()
        def protect(machine, cpu):
            if not state[0]:
                machine.write_protected[1] = True
                state[0] = True
            if (machine.memory[0xff00] == 2 and cpu.registers[24] == self.labels['floppy_bios_io'] and
                    machine.memory[self.labels['write_operation']]):
                # PIP selects error-return mode itself. This case deliberately
                # exercises interactive BIOS Retry after write protection.
                machine.memory[0xf9e7] = 0
            if self.prompt(machine, cpu) and not state[1]:
                self.assertEqual(machine.memory[self.labels['last_error']], 1)
                machine.write_protected[1] = False
                state[1] = True
        machine, text, _ = self.boot('pip b:=a:profile.old\rr', media, protect)
        self.assertTrue(state[1])
        self.assertNotIn('Error', text)
        self.assertEqual(media.read_file('PROFILE.OLD'), Disk(self.original).read_file('PROFILE.SUB'))

    def test_create_empty_data_floppies_and_refuse_overwrite(self):
        from make_floppy import create_image, FORMATS
        for name in FORMATS:
            with self.subTest(format=name):
                path = self.work / (name + '.dsk')
                create_image(path, name)
                data = path.read_bytes()
                with self.assertRaises(FileExistsError):
                    create_image(path, name)
                self.assertEqual(path.read_bytes(), data)
                media = Floppy(data)
                self.assertEqual(media.cylinders, 80 if name == 'pcw720' else 40)
                if name in ('plus3', 'pcw720'):
                    self.assertEqual(sum(media.sector(0, 0, 1)[0]) & 255, 0)
                machine, text, _ = self.boot('dir\rdir b:\rpip b:=a:basic.com\rb:basic\rprint 2+2\rsystem\r',
                                             media, limit=20000000)
                self.assertIn('No File', text)
                self.assertIn('\r\n 4 \r\nOk', text)
                self.assertNotIn('Error', text)
                self.assertTrue(machine.writes)

    def test_native_xdpb_builders_match_original(self):
        from test_patcher import ConsoleHarness
        from reference_plus3 import OriginalBios
        reference = OriginalBios(self.dependencies)
        console = ConsoleHarness(self.work, self.original, self.dependencies)
        for kind in range(4):
            reference.machine.memory[0xc800:0xc81b] = [0] * 27
            console.machine.memory[0xc800:0xc81b] = [0] * 27
            values = {0: kind, 8: 0xc8, 9: 0}
            reference.call(0x1926, values)
            _, result = console.userf(0x95, values=values)
            self.assertTrue(result[1] & 1)
            self.assertEqual(console.machine.memory[0xc800:0xc81b], reference.machine.memory[0xc800:0xc81b])
        for side in (0, 128, 129, 130):
            reference.machine.memory[0xc800:0xc81b] = [0] * 27
            console.machine.memory[0xc800:0xc81b] = [0] * 27
            spec = bytes((3, side, 80, 9, 2, 1, 4, 4, 42, 82))
            reference.machine.memory[0xc900:0xc90a] = spec
            console.machine.memory[0xc900:0xc90a] = spec
            values = {6: 0xc9, 7: 0, 8: 0xc8, 9: 0}
            reference.call(0x1939, values)
            _, result = console.userf(0xa1, values=values)
            self.assertTrue(result[1] & 1)
            self.assertEqual(console.machine.memory[0xc800:0xc81b], reference.machine.memory[0xc800:0xc81b])

    def test_disk_xbios_format_read_write_check_and_read_id(self):
        media = Media(3, extended=True)
        machine, _, _ = self.boot(disk=media)
        cpu = simulator_class(self.dependencies)(machine.memory, config={'fast_ldir': True, 'fast_djnz': True})
        cpu.set_tracer(machine)
        def call(target, values):
            for port, value in ((0x7ffd, 0x18), (0x1ffd, machine.port1 & 0xfb), (0xe3, 0)):
                machine.write_port(cpu.registers, port, value)
            machine.memory[0xff00] = 1
            r = cpu.registers
            for k, v in values.items():
                r[k] = v
            r[12], r[24] = 0x3f00, 0xfa5a
            machine.memory[0x3f00:0x3f02] = [0x20, 0x81]
            machine.memory[0x8120:0x8122] = [target, 0]
            for _ in range(1000000):
                cpu.run()
                if r[24] == 0x8122:
                    self.assertEqual(machine.memory[0xff00], 1)
                    self.assertEqual(r[12], 0x3f02)
                    return r.copy()
            self.fail('Disk XBIOS did not return: %02x PC=%04x' % (target, r[24]))
        call(0x95, {0: 3, 8: 0xc8, 9: 0})
        headers = bytes(v for sector in range(1, 10) for v in (79, 1, sector, 2))
        machine.memory[0x7000:0x7000 + len(headers)] = headers
        params = {2: 1, 3: 1, 4: 159, 5: 0xf6, 6: 0x70, 7: 0, 8: 0xc8, 9: 0}
        self.assertTrue(call(0x8f, params)[1] & 1)
        for sector in range(1, 10):
            self.assertEqual(media.sector(79, 1, sector)[0], bytes((0xf6,)) * 512)
        pattern = bytes((i * 37 + 11) & 255 for i in range(512))
        machine.memory[0x7000:0x7200] = pattern
        params[5] = 8
        self.assertTrue(call(0x89, params)[1] & 1)
        self.assertEqual(media.sector(79, 1, 9)[0], pattern)
        equal = call(0x8c, params)
        self.assertEqual(equal[1] & 0x41, 0x41)
        machine.memory[0x7000] ^= 1
        different = call(0x8c, params)
        self.assertEqual(different[1] & 0x41, 1)
        machine.memory[0x7000:0x7200] = [0] * 512
        self.assertTrue(call(0x86, params)[1] & 1)
        self.assertEqual(bytes(machine.memory[0x7000:0x7200]), pattern)
        result = call(0x9b, params)
        self.assertTrue(result[1] & 1)
        pointer = result[6] * 256 + result[7]
        self.assertEqual(machine.memory[pointer], 7)
        self.assertEqual(machine.memory[pointer + 4:pointer + 8], [79, 1, 1, 2])
        # The raw 765 entry also stages a full 512-byte sector and exports
        # its result after the DMA copy has finished using the common buffer.
        machine.memory[0xc900:0xc90f] = [1, 0, 0x70, 0, 2, 9, 0x46, 5, 79, 1, 9, 2, 9, 42, 255]
        machine.memory[0x7000:0x7200] = [0] * 512
        result = call(0xad, {6: 0xc9, 7: 0})
        self.assertEqual(bytes(machine.memory[0x7000:0x7200]), pattern)
        self.assertEqual(machine.memory[result[6] * 256 + result[7]], 7)
        media.read_only = True
        result = call(0x89, params)
        self.assertFalse(result[1] & 1)
        self.assertEqual(result[0], 1)

    def test_img_hot_swap_reloads_partition_locations_and_retries(self):
        from make_img import create_image
        from sd_model import SdImage
        path = self.work / 'swap.img'
        create_image(path)
        original = bytearray(path.read_bytes())
        def image(name, cylinder):
            data = bytearray(original)
            for index, start in enumerate((cylinder, cylinder + 128, cylinder + 256), 1):
                struct.pack_into('<HBHB', data, index * 64 + 17, start, 0, start + 127, 1)
                base = start * 256 * 512
                data[base:base + 16384] = bytes((229,)) * 16384
            base = cylinder * 256 * 512
            entry = bytearray(32)
            entry[1:12] = name.encode('ascii').ljust(8) + b'TXT'
            entry[15] = 1
            struct.pack_into('<H', entry, 16, 2)
            data[base:base + 32] = entry
            data[base + 16384:base + 16512] = (name.encode('ascii') + b'\r\n').ljust(128, b'\x1a')
            return SdImage(data)
        first, second = image('FIRST', 1), image('SECOND', 5)
        output, changed, restored = [], [], []
        def swap(machine, cpu):
            if machine.memory[0xff00] == 2:
                address = machine.memory[0x10a] + 256 * machine.memory[0x10b]
                if cpu.registers[24] == address:
                    output.append(chr(cpu.registers[3]))
            if not changed and 'FIRST' in ''.join(output) and ''.join(output[-2:]) == 'A>':
                machine.storage = None
                changed.append(True)
            if self.prompt(machine, cpu) and changed and not restored:
                machine.storage = second
                restored.append(True)
        with contextlib.redirect_stdout(io.StringIO()):
            machine, text, _ = verify(self.image, self.dependencies, 16000000,
                'dir c:\r\x03dir c:\rrtype c:second.txt\r', swap, first)
        self.assertTrue(changed and restored)
        self.assertIn('C: FIRST', text)
        self.assertIn('C: SECOND', text)
        self.assertIn('SECOND\r\n', text)
        self.assertNotIn('CP/M Error', text)
        self.assertIn(5 * 256, second.reads)

    def test_hot_swap_after_control_c_reloads_format(self):
        for replacement in (Media(text=b'SECOND\r\n'), Media(3, 130, text=b'SECOND\r\n', extended=True)):
            first = Media(text=b'FIRST\r\n')
            output, changed = [], []
            def swap(machine, cpu):
                if machine.memory[0xff00] == 2:
                    address = machine.memory[0x10a] + 256 * machine.memory[0x10b]
                    if cpu.registers[24] == address:
                        output.append(chr(cpu.registers[3]))
                if not changed and 'FIRST' in ''.join(output) and ''.join(output[-2:]) == 'A>':
                    machine.mount(1, replacement)
                    changed.append(True)
            machine, text, _ = self.boot('type b:probe.txt\r\x03type b:probe.txt\r', first, swap)
            self.assertTrue(changed)
            self.assertIn('FIRST', text)
            self.assertIn('SECOND', text)
            self.assertEqual(machine.memory[0xfe1b + 17], replacement.spec[1])
            self.assertNotIn('Error', text)

    def test_bad_format_and_unsuitable_internal_drive_are_recoverable(self):
        media = Media(3)
        spec = bytearray(media.sector(0, 0, 1)[0])
        spec[4] = 1  # unsupported 256-byte sectors
        media.write_sector(0, 0, 1, spec)
        errors = []
        def observe(machine, cpu):
            if self.prompt(machine, cpu) and not errors:
                errors.append(machine.memory[self.labels['last_error']])
        _, text, _ = self.boot('dir b:\rc', media, observe)
        self.assertEqual(errors, [6])
        self.assertIn('CP/M Error On B:', text)
        changed = []
        def swap(machine, cpu):
            if self.prompt(machine, cpu) and not changed:
                machine.mount(0, Media(3))
                changed.append(True)
            elif changed and machine.memory[self.labels['mapped_drive']] == 1 and self.prompt(machine, cpu):
                self.assertEqual(machine.memory[self.labels['last_error']], 9)
        _, text, _ = self.boot('dir b:\r\rc\r', observe=swap)
        self.assertTrue(changed)
        self.assertIn('CP/M Error On B:', text)

    def test_single_drive_selection_renders_swap_text_and_return_dismisses_it(self):
        class ReachedDrivePrompt(Exception):
            pass
        glyphs, output, shown = [], [], []
        drawing_message = [False]
        expected = '     Please put the disc for B: into the drive then press any key'
        def bitmap(machine):
            # Last text row: eight Spectrum bitmap scanlines, no attributes.
            return bytes(machine.ram[7][base + x]
                         for base in (0x10e0 + line * 256 for line in range(8))
                         for x in range(32))
        def observe(machine, cpu):
            bank, pc = machine.memory[0xff00], cpu.registers[24]
            if bank == 2 and pc == machine.memory[0x10a] + 256 * machine.memory[0x10b]:
                output.append(chr(cpu.registers[3]))
            if machine.lower_page == 3 and pc == 0x2800:
                drawing_message[0] = machine.memory[0xff73] == 0xf9
            if machine.lower_page == 3 and drawing_message[0] and pc == 0x1764:
                # Screen paging hides common RAM while native plotting runs.
                # The native rotating-message loop has fetched its glyph in C.
                glyphs.append(chr(cpu.registers[3]))
            if self.prompt(machine, cpu):
                self.assertTrue(''.join(glyphs).startswith(expected[:51]))
                text = bytes(machine.divram[3][0x1deb:0x1e3b]).split(b'\xff')[0]
                self.assertEqual(text.decode(), expected)
                if not shown:
                    shown.append(bitmap(machine))
            if (shown and bank == 2 and machine.memory[pc] == 0x76 and
                    not machine.memory[self.labels['floppy_busy']] and ''.join(output[-2:]) == 'B>'):
                self.assertNotEqual(bitmap(machine), shown[0])
                self.assertIn('A: S10CPM3', ''.join(output))
                self.assertEqual(machine.memory[0xff20], 1)
                raise ReachedDrivePrompt()
        with self.assertRaises(ReachedDrivePrompt):
            self.boot('dir\rb:\r\r', observe=observe)

    def test_single_drive_b_swap_prompt(self):
        changed, restored = [], []
        def swap(machine, cpu):
            if changed and machine.memory[0xff00] == 5 and machine.memory[0xff73] == 0xfa:
                restored.append(True)
            if self.prompt(machine, cpu) and not changed:
                changed.append(True)
                machine.mount(0, Media())
        machine, text, _ = self.boot('dir b:\r\r', observe=swap)
        self.assertTrue(changed)
        self.assertIn('B: PROBE', text)
        self.assertNotIn('Error', text)
        self.assertEqual(machine.memory[0xff20], 1)
        self.assertTrue(restored, 'Normal status must return after the swap')


if __name__ == '__main__':
    unittest.main()

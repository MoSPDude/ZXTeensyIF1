"""Execute the patched original GIOS with the ZXTeensyIF1 memory/BIOS model."""
import contextlib
import io
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1]
ROOT = SOURCE.parent
sys.path.insert(0, str(SOURCE))
sys.path.insert(0, str(ROOT / 'CPM'))
sys.path.insert(0, str(ROOT / 'CPM/tests'))
import patch_gsx
import patch_ems
from disk_image import CpmFiles, ImageError, read_image
from test_patcher import ConsoleHarness
from verify_boot import Machine


class CheckedMemory(list):
    guard = False
    mapping_update = False
    stack_window = (0, 0)

    def __setitem__(self, index, value):
        if self.guard and not self.mapping_update:
            if isinstance(index, slice):
                start, stop, step = index.indices(len(self))
                addresses = range(max(start, 0x4000), min(stop, 0x8000), step)
            else:
                addresses = (index,) if 0x4000 <= index < 0x8000 else ()
            for address in addresses:
                if not self.stack_window[0] <= address < self.stack_window[1]:
                    raise AssertionError('Drawing wrote shared TPA at %04X' % address)
        super().__setitem__(index, value)


class CheckedMachine(Machine):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.memory = CheckedMemory(self.memory)

    def update_mapping(self, refresh_lower=True):
        self.memory.mapping_update = True
        try:
            super().update_mapping(refresh_lower)
        finally:
            self.memory.mapping_update = False


class Harness:
    def __init__(self, work, original, driver, base=0xc000):
        with patch('verify_boot.Machine', CheckedMachine):
            self.console = ConsoleHarness(work, original, ROOT / 'build/CPM/deps')
        self.machine = self.console.machine
        self.cpu = self.console.simulator
        self.console.select_bank(1)
        self.machine.memory[1:3] = [3, 0xfa]
        self.machine.memory[0xf9fa] = 0x40
        self.machine.memory[base:base + len(patch_gsx.read_prl(driver)[1])] = patch_gsx.relocate_prl(driver, base)
        self.base = base
        self.keys = []
        self.wait_screen = None
        self.forbid_scr = True
        self.raster_seen = False
        self.tpa = bytes((n * 37 + 11) & 255 for n in range(16384))
        self.machine.memory[0x4000:0x8000] = self.tpa
        self.machine.memory[0x9800:0x980a] = list(struct.pack('<5H', 0x9900, 0x9940, 0x9980, 0x9a00, 0x9b00))

    def bdos(self):
        r = self.cpu.registers
        function, character = r[3], r[5]
        if function == 2:
            saved = r.copy()
            iff = self.cpu.iff
            self.console.output(chr(character))
            self.console.select_bank(1)
            r[:] = saved
            self.cpu.iff = iff
        elif function == 12:
            r[0], r[6], r[7] = 0x31, 0, 0x31
        elif function == 49:
            r[6], r[7] = 0, 0    # output-route SCB; only CRT is modelled here
        elif function in (1, 6):
            if function == 1:
                self.wait_screen = self.screen()
            value = self.keys.pop(0) if self.keys else (13 if function == 1 else 0)
            r[0], r[6], r[7] = value, 0, value
        else:
            raise AssertionError('Unexpected BDOS function %d' % function)
        sp = r[12]
        r[24] = self.machine.memory[sp] + 256 * self.machine.memory[sp + 1]
        r[12] += 2

    def execute(self, address, stack=0x9ff0, limit=8000000):
        r = self.cpu.registers
        self.machine.memory.stack_window = (stack - 512, stack + 2)
        r[12], r[24] = stack, address
        self.machine.memory[stack:stack + 2] = [0x00, 0x97]
        for _ in range(limit):
            if r[24] == 0x9700:
                self.machine.update_mapping()
                actual = bytes(self.machine.memory[0x4000:0x8000])
                for n, value in enumerate(actual):
                    if (value != self.tpa[n] and
                            not stack - 512 <= n + 0x4000 < stack + 2):
                        raise AssertionError('Drawing changed the shared TPA')
                self.tpa = actual
                return
            if self.forbid_scr and r[24] == 0xfa5a:
                sp = r[12]
                inline = self.machine.memory[sp] + 256 * self.machine.memory[sp + 1]
                if self.machine.memory[inline:inline + 2] == [0xe9, 0]:
                    raise AssertionError('GIOS called SCR RUN ROUTINE')
            if (self.machine.port7 & 7) == 7 and self.machine.lower_page == 7:
                self.raster_seen = True
                if self.cpu.iff:
                    raise AssertionError('Interrupts enabled while RAM 0/BDOS is hidden')
            if r[24] == 5:
                self.bdos()
            else:
                self.cpu.run()
        raise AssertionError('GIOS did not return (PC=%04X, SP=%04X)' % (r[24], r[12]))

    def call(self, function, ints=(), points=(), sub=0, stack=0x9ff0):
        self.machine.memory[0x9900:0x9910] = list(struct.pack('<8H', function, len(points), 0, len(ints), 0, sub, 1, 0))
        if ints:
            self.machine.memory[0x9940:0x9940 + 2 * len(ints)] = struct.pack('<%dH' % len(ints), *ints)
        if points:
            values = [v & 65535 for pair in points for v in pair]
            self.machine.memory[0x9980:0x9980 + 2 * len(values)] = struct.pack('<%dH' % len(values), *values)
        r = self.cpu.registers
        r[4], r[5] = 0x98, 0
        self.execute(self.base, stack=stack)
        return struct.unpack('<45H', bytes(self.machine.memory[0x9a00:0x9a5a]))

    def open(self):
        return self.call(1, (1, 1, 1, 1, 1, 1, 1, 1, 1, 1))

    def screen(self):
        self.machine.update_mapping()
        return bytes(self.machine.ram[7][:6912])

    def pixel(self, x, y):
        # GSX's origin is bottom-left; Spectrum's bitmap starts at top-left.
        y = 191 - y
        offset = ((y & 192) << 5) | ((y & 7) << 8) | ((y & 56) << 2) | (x >> 3)
        return bool(self.screen()[offset] & (128 >> (x & 7)))


class DriverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = patch_gsx.DEFAULT_INPUT
        if not (source.exists() and patch_ems.DEFAULT_INPUT.exists() and shutil.which('sjasmplus') and
                (ROOT / 'build/CPM/deps/skoolkit').exists()):
            raise unittest.SkipTest('Local PCW/+3 disks, sjasmplus and SkoolKit 8.10 required')
        work_root = ROOT / 'build/CPMGSX'
        work_root.mkdir(parents=True, exist_ok=True)
        cls.temporary = tempfile.TemporaryDirectory(prefix='driver-tests-', dir=str(work_root))
        cls.work = Path(cls.temporary.name)
        cls.original = patch_ems.DEFAULT_INPUT.read_bytes()
        with contextlib.redirect_stdout(io.StringIO()):
            boot, _ = patch_ems.patch_disk(cls.original, shutil.which('sjasmplus'), cls.work)
        cls.boot_image = cls.work / 'boot.dsk'
        cls.boot_image.write_bytes(boot)
        cls.driver = patch_gsx.patch_driver(patch_gsx.source_driver(source.read_bytes()), shutil.which('sjasmplus'), cls.work)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def harness(self, base=0xc000):
        return Harness(self.work, self.original, self.driver, base)

    def test_open_clear_close_at_several_relocations(self):
        highest = (0xf400 - len(patch_gsx.read_prl(self.driver)[1])) & 0xff00
        for base in (0x8000, 0xa000, 0xc000, highest):
            with self.subTest(base=base):
                h = self.harness(base)
                before = bytes(h.machine.memory[0x4000:0x8000])
                out = h.open()
                self.assertEqual(out[:2], (255, 191))
                self.assertEqual(out[3:5], (1000, 1000))
                self.assertEqual(bytes(h.machine.memory[0x4000:0x8000]), before)
                h.call(3)
                self.assertEqual(h.screen(), b'\0' * 6144 + b'\7' * 768)
                h.call(2)
                self.assertEqual(h.machine.divram[3][0x1f25], 51)
                self.assertEqual(h.machine.memory[0xff00], 1)

    def test_plot_all_spectrum_row_groups_and_corners(self):
        h = self.harness()
        h.open()
        h.call(3)
        for x, y in ((0, 0), (255, 0), (0, 191), (255, 191), (7, 7), (8, 8),
                     (127, 63), (128, 64), (127, 127), (128, 128)):
            h.call(7, points=((x, y),))
            self.assertTrue(h.pixel(x, y), (x, y))
        self.assertEqual(sum(bin(b).count('1') for b in h.screen()[:6144]), 10)

    def test_direct_drawing_never_borrows_ram5_or_d6(self):
        h = self.harness()
        h.machine.memory.guard = True
        backup = bytes((n * 19 + 7) & 255 for n in range(16384))
        h.machine.divram[6][:] = backup
        h.open()
        h.call(3)
        h.call(6, points=((3, 4), (245, 180)))
        h.call(8, ints=(65, 66, 67), points=((40, 80),))
        h.call(11, points=((90, 30), (150, 50)), sub=1)
        self.assertTrue(h.raster_seen)
        self.assertTrue(h.pixel(100, 40))
        self.assertEqual(bytes(h.machine.divram[6]), backup)
        self.assertEqual(bytes(h.machine.divram[7][0x3000:0x3800]),
                         bytes(h.machine.divram[3][0x3000:0x3800]))

    def test_direct_gate_preserves_stacks_mapping_motor_and_iff(self):
        h = self.harness()
        h.open()
        h.machine.memory.guard = True
        for stack in (0x3000, 0x7100, 0x9ff0, 0xeff0, 0xf3e0):
            for enabled in (False, True):
                for motor in (0, 8):
                    with self.subTest(stack=stack, iff=enabled, motor=motor):
                        h.machine.memory[0xff25] = motor
                        h.machine.write_port(h.cpu.registers, 0x1ffd, motor)
                        h.cpu.iff = enabled
                        alternate = h.cpu.registers[16:24].copy()
                        h.call(6, points=((5, 15), (230, 160)), stack=stack)
                        self.assertEqual(h.cpu.registers[12], stack + 2)
                        self.assertEqual(h.cpu.iff, enabled)
                        self.assertEqual(h.cpu.registers[16:24], alternate)
                        self.assertEqual((h.machine.lower_page, h.machine.port7,
                                          h.machine.port1, h.machine.e3), (1, 0x18, motor, 0))

    def test_polyline_fill_and_xor(self):
        h = self.harness()
        h.open()
        h.call(3)
        h.call(6, points=((0, 0), (255, 0)))
        self.assertTrue(all(h.pixel(x, 0) for x in range(256)))
        self.assertFalse(h.pixel(128, 1))
        h.call(3)
        h.call(11, points=((48, 48), (207, 112)), sub=1)
        for x, y, expected in ((48, 48, True), (207, 112, True), (128, 80, True),
                               (47, 80, False), (208, 80, False), (128, 47, False)):
            self.assertEqual(h.pixel(x, y), expected, (x, y))
        h.call(3)
        h.call(32, ints=(3,))
        h.call(6, points=((12, 20), (225, 170)))
        self.assertTrue(any(h.screen()[:6144]))
        h.call(6, points=((12, 20), (225, 170)))
        self.assertFalse(any(h.screen()[:6144]))
        h.call(32, ints=(1,))
        h.call(11, points=((-50, -30), (300, 220)), sub=1)
        self.assertEqual(h.screen()[:6144], b'\xff' * 6144)

    def test_text_uses_the_spectrum_font(self):
        h = self.harness()
        h.open()
        h.call(3)
        h.call(8, ints=(65,), points=((40, 80),))
        screen = h.screen()
        font = h.machine.divram[3][0x3000 + 65 * 8:0x3000 + 66 * 8]
        for row, bits in enumerate(font):
            y = 191 - (80 + 6) + row
            offset = ((y & 192) << 5) | ((y & 7) << 8) | ((y & 56) << 2) | 5
            self.assertEqual(screen[offset], bits, row)

    def test_polygon_and_pattern_fill(self):
        h = self.harness()
        h.open()
        h.call(3)
        h.call(9, points=((40, 40), (128, 150), (215, 40)))
        self.assertTrue(h.pixel(128, 80))
        self.assertFalse(h.pixel(35, 80))
        solid = sum(bin(b).count('1') for b in h.screen()[:6144])
        h.call(3)
        h.call(23, ints=(2,))
        h.call(24, ints=(1,))
        h.call(9, points=((40, 40), (128, 150), (215, 40)))
        patterned = sum(bin(b).count('1') for b in h.screen()[:6144])
        self.assertGreater(patterned, 0)
        self.assertLess(patterned, solid)

    def test_keyboard_sample_locator(self):
        h = self.harness()
        h.open()
        h.call(33, ints=(1, 2))
        h.keys = [12]
        h.call(28, ints=(1,), points=((50, 50),))
        self.assertEqual(struct.unpack('<2H', bytes(h.machine.memory[0x9b00:0x9b04])), (51, 50))
        h.keys = [32]
        out = h.call(28, ints=(1,), points=((50, 50),))
        self.assertEqual(out[0], 32)

    def test_terminal_modes_and_status_are_restored(self):
        for mode, width in enumerate((32, 51, 80)):
            for status in (0, 255):
                with self.subTest(width=width, status=status):
                    h = self.harness()
                    h.console.output('\x1b3' + chr(mode))
                    h.console.userf(0xc8, values={0: status})
                    h.console.select_bank(1)
                    h.open()
                    self.assertEqual(h.machine.divram[3][0x1f25], 32)
                    h.call(2)
                    self.assertEqual(h.machine.divram[3][0x1f25], width)
                    _, after = h.console.userf(0xc5, bank=1)
                    self.assertEqual(after[0], status)

    def test_standalone_demo_and_generated_disk(self):
        (self.work / 'demo-driver.bin').write_bytes(patch_gsx.relocate_prl(self.driver, 0xc000))
        demo, _ = patch_gsx.assemble(shutil.which('sjasmplus'), 'demo', self.work)
        h = self.harness()
        h.machine.memory[0x100:0x100 + len(demo)] = demo
        h.execute(0x100)
        self.assertIsNotNone(h.wait_screen)
        self.assertTrue(any(h.wait_screen[:6144]))
        self.assertEqual(h.machine.divram[3][0x1f25], 51)
        image = patch_gsx.make_disk(self.driver, demo, self.work)
        files = CpmFiles(read_image(image)).files
        self.assertEqual(set(n for u, n in files), {'DDSCREEN.PRL', 'ASSIGN.SYS', 'GSXTEST.COM', 'README.TXT'})
        self.assertTrue(files[0, 'DDSCREEN.PRL'].startswith(self.driver))

    def test_modified_and_wrong_sources_rejected(self):
        with self.assertRaisesRegex(ImageError, 'absent'):
            patch_gsx.source_driver((ROOT / 'ignoredfiles/pcw9512.td0').read_bytes())
        original = bytearray(patch_gsx.source_driver(patch_gsx.DEFAULT_INPUT.read_bytes()))
        original[400] ^= 1
        with self.assertRaisesRegex(ImageError, 'Unsupported'):
            patch_gsx.patch_driver(original, shutil.which('sjasmplus'), self.work)

    def test_demo_loaded_by_real_cpm_from_drive_b(self):
        from floppy_model import Floppy
        from verify_boot import verify
        (self.work / 'demo-driver.bin').write_bytes(patch_gsx.relocate_prl(self.driver, 0xc000))
        demo, _ = patch_gsx.assemble(shutil.which('sjasmplus'), 'demo', self.work)
        image = patch_gsx.make_disk(self.driver, demo, self.work)
        with contextlib.redirect_stdout(io.StringIO()):
            machine, transcript, _ = verify(self.boot_image, ROOT / 'build/CPM/deps',
                                            20000000, 'b:gsxtest\rx', drive_b=Floppy(image))
        self.assertIn('B:GSXTEST', transcript.upper())
        self.assertTrue(transcript.endswith('A>'), transcript[-200:])
        self.assertNotIn('Error', transcript)
        self.assertEqual(machine.divram[3][0x1f25], 51)


if __name__ == '__main__':
    unittest.main()

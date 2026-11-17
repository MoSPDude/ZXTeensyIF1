"""Compare the adapted terminal with code from the user's original +3 disk."""
import contextlib
import io
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

CPM = Path(__file__).resolve().parents[1]
ROOT = CPM.parent
sys.path.insert(0, str(CPM))
import patch_ems
from test_patcher import ConsoleHarness
from reference_plus3 import OriginalBios


class TerminalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dependencies = ROOT / 'build/CPM/deps'
        if not patch_ems.DEFAULT_INPUT.exists() or not shutil.which('sjasmplus') or not (cls.dependencies / 'skoolkit').exists():
            raise unittest.SkipTest('Local original disk, sjasmplus and SkoolKit 8.10 required')
        cls.temporary = tempfile.TemporaryDirectory(prefix='terminal-tests-', dir=str(ROOT / 'build/CPM'))
        cls.work = Path(cls.temporary.name)
        cls.original = patch_ems.DEFAULT_INPUT.read_bytes()
        patched, _ = patch_ems.patch_disk(cls.original, shutil.which('sjasmplus'), cls.work)
        cls.image = cls.work / 'boot.dsk'
        cls.image.write_bytes(patched)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.console = ConsoleHarness(self.work, self.original, self.dependencies)
        self.reference = OriginalBios(self.dependencies)
        self.reference.call(0x101d)
        # Standalone CRT routing: the native status test's common hook returns
        # with its carry unchanged. No BDOS/device dispatcher is running here.
        self.reference.machine.memory[0xba06] = 0xc9
        self.reference.machine.memory[0x75] = 0  # one physical floppy drive
        self.reference.call(0x161a, {0: 0})
        self.send('\x1bf')

    def send(self, text):
        for character in text:
            self.reference.call(0x1025, {3: ord(character)})
        self.console.output(text)

    def compare(self):
        native = self.reference.machine
        native.update_mapping()
        actual = self.console.machine
        front = actual.divram[3][0x27f1]
        for page, source in ((7 if front == 0 else 6, 5), (6 if front == 0 else 7, 7)):
            # The 6K bitmap and 768 hardware attributes. The original stores
            # unrelated boot/CCP workspace after these, outside screen RAM.
            a, b = actual.ram[page][:6912], native.ram[source][:6912]
            differences = [(i, x, y) for i, (x, y) in enumerate(zip(a, b)) if x != y]
            self.assertFalse(differences, 'screen differences: %r' % differences[:12])
        self.assertEqual(actual.divram[3][0x2182:0x2191], native.memory[0x2182:0x2191])
        self.assertEqual(actual.memory[0xf9b6], native.memory[0x2184])
        self.assertEqual(actual.memory[0xf9b8], native.memory[0x2185])

    def test_native_modes_viewports_editing_page_and_language(self):
        self.send('\x1bE\x1bH')
        for mode in range(3):
            self.send('\x1b3' + chr(mode) + '\x1bH')
            width = (32, 51, 80)[mode]
            sequences = [
                ''.join(chr(32 + n % 224) for n in range(width * 3)),
                '\x1bY!*\x1bj\x1bA\x1bB\x1bC\x1bD\x1bk',
                '\x1bLinsert\x1bM\x1bN\x1bK',
                '\x1bY"%\x1bo\x1bl\x1bd\x1bJ',
                '\x1bH\x1bIreverse index\x1bE',
                '\x1bX"#$1\x1bH' + 'viewport text ' * 20,
                '\x1b4\x01\x1bY\xff\xffZ\n\x1bI\b',
                '\x1b4\x00\x1bw\x1bY\xff\xffXY\x1bvZ',
                '\x1bpREV\x1bq\x1brUNDER\x1bu\x1b\r',
            ]
            for text in sequences:
                self.send(text)
                self.compare()
            self.send('\x1by')
        for language in range(8):
            self.send('\x1b2' + chr(language) + '\x1bH' + ''.join(map(chr, range(32, 256))))
            self.compare()

    def test_native_colours_status_and_delayed_cursor_flip(self):
        for ink in (0, 1):
            for colour in range(64):
                values = {0: ink, 2: colour, 3: colour ^ (1 if colour & 1 else 0)}
                self.reference.call(0x1455, values)
                self.console.userf(0xcb, values=values)
                self.send('Colour')
                self.compare()
        self.reference.call(0x101d)
        self.console.userf(0xc2)
        for status in (255, 0, 255):
            self.reference.call(0x161a, {0: status})
            self.reference.call(0x1674)
            self.console.userf(0xc8, values={0: status})
            self.compare()
            a = self.reference.call(0x15ff)
            _, b = self.console.userf(0xc5)
            self.assertEqual((b[0], b[1] & 0x41), (a[0], a[1] & 0x41))
        self.send('\x1bx\x1bY!' + chr(32 + 70) + 'R\x1be')
        for _ in range(5):
            self.reference.call(0x119a)
            self.console.tick(1)
        self.compare()
        self.assertEqual(self.console.machine.divram[3][0x27f1], 1)
        self.reference.call(0xc73)
        # Physical CAPS + SYMBOL + ENTER consumes ENTER and flips once.
        self.console.machine.keyboard = {(0, 0), (7, 1), (6, 0)}
        self.console.machine.keyboard_start = self.console.simulator.registers[25] + 1000000
        self.console.tick(1)
        self.console.machine.keyboard = None
        self.compare()
        self.assertEqual(self.console.machine.divram[3][0x27f1], 0)
        self.assertEqual(self.console.driver_call(0x103)[0], 0)

    def test_recovery_messages_survive_timer_and_fit_console_modes(self):
        message = b'Write protected'
        for mode, status in ((0, 255), (1, 255), (2, 255), (1, 0)):
            with self.subTest(mode=mode, status=status):
                self.reference.call(0x101d)
                self.console.userf(0xc2)
                self.send('\x1b3' + chr(mode) + '\x1bf')
                self.reference.call(0x161a, {0: status})
                self.console.userf(0xc8, values={0: status})
                self.reference.machine.memory[0x1de7] = 1
                self.console.machine.memory[0xff75] = 1
                self.reference.machine.memory[0x1800:0x1800 + len(message) + 1] = message + b'\xff'
                self.reference.call(0x1625, {2: 20, 4: 0x18, 5: 0}, limit=1000000)
                self.console.select_bank(2)
                self.console.machine.divram[3][0x1800:0x1800 + len(message) + 1] = message + b'\0'
                self.console.machine.memory[0xff73] = 0xfb
                self.console.driver_call(0xfa75)
                self.compare()
                for _ in range(5):
                    self.reference.call(0x119a)
                    self.reference.call(0x172b)
                    self.console.tick(1)
                self.compare()
                self.reference.call(0x1665)
                self.console.machine.memory[0xff73] = 0xfa
                self.console.driver_call(0xfa75)
                self.compare()

    def test_screen_callback_preserves_incoming_af_including_nested_calls(self):
        # Capture AF before any flag-changing instruction. The outer callback
        # can invoke SCR RUN ROUTINE again while the screen window is active.
        capture = bytes.fromhex('f5d1ed530080')  # PUSH AF; POP DE; LD (8000),DE
        nested = bytes.fromhex('f5d1ed530280c9')
        for a, flags in ((0x5a, 0xa5), (0xc3, 0x5a)):
            values = {0: a, 1: flags, 2: 0xc8, 3: 0}
            self.reference.machine.memory[0xc800:0xc807] = capture + b'\xc9'
            native = self.reference.call(0xe9, values)
            expected = self.reference.machine.memory[0x8000:0x8002]
            self.assertEqual(expected, [flags, a])
            self.assertEqual(native[:2], [a, flags])
            for recurse in (False, True):
                code = capture
                if recurse:
                    code += bytes.fromhex('0100c9cd5afae900')
                code += b'\xc9'
                for stack in (0x3f00, 0x7100, 0xf3e0):
                    for iff in (0, 1):
                        with self.subTest(a=a, flags=flags, recurse=recurse, stack=stack, iff=iff):
                            c = self.console
                            c.machine.memory[0xc800:0xc800 + len(code)] = code
                            c.machine.memory[0xc900:0xc900 + len(nested)] = nested
                            c.machine.memory[0x8000:0x8004] = [0] * 4
                            _, after = c.userf(0xe9, bank=1, stack=stack, iff=iff, values=values)
                            self.assertEqual(c.machine.memory[0x8000:0x8002], expected)
                            if recurse:
                                self.assertEqual(c.machine.memory[0x8002:0x8004], expected)
                            self.assertEqual(after[:2], native[:2])

    def test_screen_callback_nested_bios_fonts_and_tpa(self):
        # New test code writes the native screen window, reads a glyph, and
        # makes nested BIOS CONOUT and USERF/TE ASK calls before returning.
        code = bytes.fromhex('3ef832083a3e5a3200403a083a3201400e51cd0cfacd5afabf00'
                             '113412217856dd21bc9afd21f0de37c9')
        for stack in (0x3f00, 0x7100, 0xf3e0):
            c = self.console
            c.machine.memory[0x4000:0xf400] = [0xa5] * 0xb400
            c.machine.memory[0xc800:0xc800 + len(code)] = code
            c.machine.memory[0x8120:0x8122] = [0xe9, 0]
            before = bytes(c.machine.memory[0x4000:0xf400])
            c.position(3, 10)
            _, after = c.userf(0xe9, bank=1, stack=stack, values={2: 0xc8, 3: 0})
            self.assertEqual(after[4:12], [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0])
            self.assertTrue(after[1] & 1)
            self.assertEqual(c.machine.ram[7][0:2], [0x5a, 0xf8])
            self.assertEqual(c.machine.divram[3][0x3a08], 0xf8)
            for i, value in enumerate(before, 0x4000):
                if stack - 128 <= i < stack + 2:
                    continue  # normal caller/gate stack scratch
                self.assertEqual(c.machine.memory[i], value, hex(i))
            _, info = c.userf(0xe6)
            self.assertEqual(info[0:4:2], [0, 8])
            self.assertEqual(info[3], 0)
            table = info[6] * 256 + info[7]
            self.assertEqual(c.machine.memory[table:table + 6], [0, 0, 1, 0, 4, 255])

    def test_original_display_utilities_and_basic(self):
        from verify_boot import verify
        commands = 'set24x80 on\rbasic\rprint 2+2\rsystem\rset24x80 off\rlanguage 0\r'
        widths = set()
        def observe(machine, cpu):
            if machine.port7 & 7 == 0:
                widths.add(machine.memory[0xf9b6])
        with contextlib.redirect_stdout(io.StringIO()):
            machine, text, _ = verify(self.image, self.dependencies, 12000000, commands, observe)
        self.assertIn(79, widths)
        self.assertIn(' 4 ', text)
        self.assertNotIn('Syntax error', text)
        self.assertNotIn('will not run', text)
        self.assertNotIn('Bad ', text)
        self.assertEqual(machine.memory[0xf9b6], 50)


if __name__ == '__main__':
    unittest.main()

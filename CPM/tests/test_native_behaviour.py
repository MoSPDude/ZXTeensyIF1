"""Native EMS comparisons for messages, motor waits and FID memory release."""
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

CPM = Path(__file__).resolve().parents[1]
ROOT = CPM.parent
sys.path.insert(0, str(CPM))
import patch_ems
from reference_plus3 import OriginalBios
from test_patcher import ConsoleHarness
from verify_boot import key_matrix, symbols, verify


def make_prl(body, relocations, bss=0):
    code = bytearray(32)
    code[:3] = bytes((0xc3, 0x20, 1))
    code[3:14] = b'SPECTRUMFID'
    code.extend(body)
    bitmap = bytearray((len(code) + 7) // 8)
    for offset in (2,) + tuple(relocations):
        bitmap[offset // 8] |= 128 >> (offset % 8)
    header = bytearray(256)
    header[1:3] = len(code).to_bytes(2, 'little')
    header[4:6] = bss.to_bytes(2, 'little')
    result = header + code + bitmap
    result[0x110:0x112] = (sum(result) & 65535).to_bytes(2, 'little')
    return result


class NativeBehaviourTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dependencies = ROOT / 'build/CPM/deps'
        if not patch_ems.DEFAULT_INPUT.exists() or not shutil.which('sjasmplus') or not (cls.dependencies / 'skoolkit').exists():
            raise unittest.SkipTest('Local original disk, sjasmplus and SkoolKit required')
        cls.temporary = tempfile.TemporaryDirectory(prefix='native-behaviour-', dir=str(ROOT / 'build/CPM'))
        cls.work = Path(cls.temporary.name)
        cls.original = patch_ems.DEFAULT_INPUT.read_bytes()
        patched, _ = patch_ems.patch_disk(cls.original, shutil.which('sjasmplus'), cls.work)
        cls.image = cls.work / 'boot.dsk'
        cls.image.write_bytes(patched)
        cls.host = symbols(cls.work / 'fid_host.sym')

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.console = ConsoleHarness(self.work, self.original, self.dependencies)

    def reference(self):
        reference = OriginalBios(self.dependencies)
        reference.machine.memory[0xba06] = 0xc9
        reference.call(0x101d)
        return reference

    def screen_call(self, function, values=None):
        c = self.console
        c.select_bank(2)
        if values:
            for key, value in values.items():
                c.simulator.registers[key] = value
        c.machine.memory[0xff73] = function
        return c.driver_call(0xfa75)

    def compare_screen(self, reference):
        reference.machine.update_mapping()
        actual = self.console.machine
        front = actual.divram[3][0x27f1]
        for page, source in ((7 if front == 0 else 6, 5), (6 if front == 0 else 7, 7)):
            self.assertEqual(actual.ram[page][:6912], reference.machine.ram[source][:6912])

    def call_host(self, name, values=None):
        c = self.console
        c.select_bank(4)
        r = c.simulator.registers
        if values:
            for key, value in values.items():
                r[key] = value
        r[12], r[24] = 0xf380, self.host[name]
        c.machine.memory[0xf380:0xf382] = [0, 0xe0]
        for _ in range(1000000):
            c.simulator.run()
            if r[24] == 0xe000:
                return r.copy()
        self.fail('FID host call did not return: ' + name)

    def test_status_tracks_physical_floppy_mapping_and_count(self):
        reference = self.reference()
        for count in (1, 2):
            for mapped in (0, 1):
                for selected in (0, 1, 2, 4):
                    with self.subTest(count=count, mapped=mapped, selected=selected):
                        reference.machine.memory[0x75] = 0 if count == 1 else 255
                        reference.machine.memory[0x1de7] = mapped
                        reference.call(0x161a, {0: 255})
                        reference.call(0x1674)
                        c = self.console
                        c.machine.memory[0xff20] = count
                        c.machine.memory[0xff75] = mapped
                        c.machine.memory[0xff24] = selected
                        c.userf(0xc8, values={0: 255})
                        self.compare_screen(reference)
                        self.assertEqual(c.machine.divram[3][0x75], 0 if count == 1 else 255)
                        self.assertEqual(c.machine.divram[3][0x1de7], mapped)

    def test_native_error_messages_bleep_and_rotate_in_all_modes(self):
        reference = self.reference()
        c = self.console
        for mode in range(3):
            for count in (1, 2):
                # Include every native disk error and the single-drive swap.
                for number in tuple(range(10)) + (16,):
                    with self.subTest(mode=mode, count=count, message=number):
                        reference.call(0x101d)
                        c.userf(0xc2)
                        for character in '\x1b3' + chr(mode) + '\x1bf':
                            reference.call(0x1025, {3: ord(character)})
                        c.output('\x1b3' + chr(mode) + '\x1bf')
                        reference.machine.memory[0x75] = 0 if count == 1 else 255
                        reference.machine.memory[0x1de7] = 1
                        c.machine.memory[0xff20] = count
                        c.machine.memory[0xff75] = 1
                        reference.call(0x161a, {0: 255})
                        c.userf(0xc8, values={0: 255})
                        values = {2: number, 3: 1, 4: 17, 5: 8}
                        native_start, actual_start = len(reference.machine.ports), len(c.machine.ports)
                        reference.call(0x1625, values, limit=1000000)
                        self.screen_call(0xf9, values)
                        self.compare_screen(reference)
                        native_ay = [(p, v) for _, p, v in reference.machine.ports[native_start:] if p in (0xfffd, 0xbffd)]
                        actual_ay = [(p, v) for _, p, v in c.machine.ports[actual_start:] if p in (0xfffd, 0xbffd)]
                        self.assertEqual(actual_ay, native_ay)
                        self.assertTrue(actual_ay)
                        for _ in range(12):
                            reference.call(0x119a)
                            reference.call(0x172b)
                            c.tick(1)
                            self.compare_screen(reference)
                        reference.call(0x1665)
                        self.screen_call(0xfa)
                        self.compare_screen(reference)

    def test_long_fid_questions_follow_native_status_and_console_routing(self):
        reference = self.reference()
        c = self.console
        message = b'Custom FID failure on the current storage partition'
        for mode, status in ((0, 255), (1, 255), (2, 255), (1, 0)):
            with self.subTest(mode=mode, status=status):
                reference.call(0x101d)
                c.userf(0xc2)
                for character in '\x1b3' + chr(mode) + '\x1bf':
                    reference.call(0x1025, {3: ord(character)})
                c.output('\x1b3' + chr(mode) + '\x1bf')
                reference.machine.memory[0x75] = 0
                reference.machine.memory[0x1de7] = 0
                reference.call(0x161a, {0: status})
                c.userf(0xc8, values={0: status})
                reference.machine.memory[0x1800:0x1800 + len(message) + 1] = message + b'\xff'
                c.machine.divram[3][0x1800:0x1800 + len(message) + 1] = message + b'\0'
                reference.call(0x1625, {2: 20, 4: 0x18, 5: 0}, limit=1000000)
                self.screen_call(0xfb)
                self.compare_screen(reference)
                for _ in range(15):
                    reference.call(0x119a)
                    reference.call(0x172b)
                    c.tick(1)
                    self.compare_screen(reference)
                reference.call(0x1665)
                self.screen_call(0xfa)
                self.compare_screen(reference)

    def test_partial_private_release_reuses_and_coalesces_ranges(self):
        for offset, size in ((0, 256), (256, 256), (128, 128)):
            with self.subTest(offset=offset, size=size):
                self.call_host('initialise')
                r = self.call_host('allocate', {4: 2, 5: 0})
                self.assertTrue(r[1] & 1)
                original = r[6] * 256 + r[7]
                pointer = original + offset
                self.call_host('deallocate', {4: size >> 8, 5: size & 255,
                                             6: pointer >> 8, 7: pointer & 255})
                r = self.call_host('allocate', {4: size >> 8, 5: size & 255})
                self.assertTrue(r[1] & 1)
                self.assertEqual(r[6] * 256 + r[7], pointer)
                self.call_host('deallocate', {4: size >> 8, 5: size & 255,
                                             6: pointer >> 8, 7: pointer & 255})
                for start, length in ((original, offset), (pointer + size, 512 - offset - size)):
                    if length:
                        self.call_host('deallocate', {4: length >> 8, 5: length & 255,
                                                     6: start >> 8, 7: start & 255})
                r = self.call_host('allocate', {4: 2, 5: 0})
                self.assertTrue(r[1] & 1)
                self.assertEqual(r[6] * 256 + r[7], original)

    def test_invalid_recovery_response_has_native_bleep_feedback(self):
        reference = self.reference()
        c = self.console
        reference.machine.memory[0x75] = 255
        reference.call(0x161a, {0: 255})
        c.machine.memory[0xff20] = 2
        c.userf(0xc8, values={0: 255})
        m = reference.machine.memory
        m[0xba03:0xba06] = [0xc3, 0, 0x2d]
        m[0x2d00:0x2d09] = bytes.fromhex('2a202d7e2322202dc9')
        m[0x2d20:0x2d22] = [0x30, 0x2d]
        m[0x2d30:0x2d32] = b'xC'
        start = len(reference.machine.ports)
        reference.call(0x414, {2: 0, 3: 1, 4: 0, 5: 0}, limit=1000000)
        expected = [(p, v) for _, p, v in reference.machine.ports[start:] if p in (0xfffd, 0xbffd)]
        c.select_bank(2)
        m = c.machine.memory
        address = c.labels['console_input']
        m[address:address + 3] = [0xc3, 0, 0x2b]
        m[0x2b00:0x2b09] = bytes.fromhex('2a202b7e2322202bc9')
        m[0x2b20:0x2b22] = [0x30, 0x2b]
        m[0x2b30:0x2b32] = b'xC'
        m[c.labels['last_error']] = 0
        m[c.labels['logical_drive']] = 1
        m[c.labels['allow_ignore']] = 1
        start = len(c.machine.ports)
        r = c.driver_call(c.labels['floppy_error_prompt'])
        actual = [(p, v) for _, p, v in c.machine.ports[start:] if p in (0xfffd, 0xbffd)]
        self.assertEqual(r[0], ord('C'))
        self.assertEqual(actual, expected)
        self.assertTrue(actual)

    def install(self, prl, reset=True):
        if reset:
            self.call_host('initialise')
        c = self.console
        c.machine.memory[0x8000:0x8000 + len(prl)] = prl
        c.machine.memory[0xff30:0xff3b] = b'PROBE   FID'
        result = self.call_host('install', {2: len(prl) >> 8, 3: len(prl) & 255})
        self.assertEqual(result[0], 0)

    def test_fid_receives_native_country_and_environment(self):
        body = bytes.fromhex('79c6303200c9ed5301c9213001af37c9') + b'Country FID\r\n\xff'
        self.install(make_prl(body, (44,)))
        self.assertEqual(self.console.machine.memory[0xc900:0xc903], [ord('3'), 0, 0])

    def test_fid_can_return_unused_loaded_bss_tail(self):
        available = []
        for release in (False, True):
            body = bytearray(9) + bytes.fromhex('212f01af37c9') + b'Tail FID\r\n\xff'
            relocations = [43]
            if release:
                tail = 0x100 + 32 + len(body) + 512
                body[:9] = b'\x21' + tail.to_bytes(2, 'little') + bytes.fromhex('110002cd09ff')
                relocations += [34, 40]
            self.install(make_prl(body, relocations, bss=1024))
            r = self.call_host('max_allocate')
            self.assertTrue(r[1] & 1)
            available.append(r[4] * 256 + r[5])
        self.assertEqual(available[1] - available[0], 512)

    def test_released_image_middle_can_hold_another_fid(self):
        body = bytearray(9) + bytes.fromhex('212f01af37c9') + b'Tail FID\r\n\xff'
        middle = 0x100 + 32 + len(body) + 128
        body[:9] = b'\x21' + middle.to_bytes(2, 'little') + bytes.fromhex('110002cd09ff')
        self.install(make_prl(body, (34, 40, 43), bss=1024))
        loading = self.console.machine.memory[self.host['loading']:self.host['loading'] + 2]
        loading = loading[0] + 256 * loading[1]
        released = loading + 32 + len(body) + 128
        # Consume the large free area above this image. The next image must
        # then fit in the released middle without overwriting retained bytes.
        self.assertTrue(self.call_host('max_allocate')[1] & 1)
        self.console.select_bank(4)
        before = bytes(self.console.machine.memory[loading:released])
        self.install(make_prl(bytes.fromhex('212601af37c9') + b'Next FID\r\n\xff', (34,)), reset=False)
        self.console.select_bank(4)
        new = self.console.machine.memory[self.host['loading']:self.host['loading'] + 2]
        new = new[0] + 256 * new[1]
        self.assertGreaterEqual(new, released)
        self.assertLess(new + 64, released + 512)
        self.assertEqual(new & 255, 0)
        self.assertEqual(bytes(self.console.machine.memory[loading:released]), before)

    def test_motor_wait_services_clock_and_a_short_key_press(self):
        c = self.console
        c.select_bank(2)
        c.machine.memory[0x38:0x3b] = [0xc3, 0x6c, 0xfa]
        c.machine.memory[0xff76] = 1
        c.driver_call(c.labels['floppy_motor_stop'])
        c.select_bank(1)
        c.machine.memory[0xf380:0xf382] = [0, 0xc9]
        c.machine.memory[0xc900:0xc902] = [0xa4, 0]
        r = c.simulator.registers
        r[12], r[24] = 0xf380, 0xfa5a
        c.simulator.iff = 1
        start = r[25]
        frame = start // 70908
        accepted = 0
        for _ in range(1000000):
            elapsed = r[25] - start
            c.machine.keyboard = {(1, 0)} if 700000 < elapsed < 1400000 else None
            c.machine.keyboard_start = r[25] + 10000000
            pc = r[24]
            c.simulator.run()
            current = r[25] // 70908
            if current != frame:
                if c.simulator.iff:
                    c.simulator.accept_interrupt(r, c.machine.memory, pc)
                    accepted += 1
                frame = current
            if r[24] == 0xc902:
                break
        else:
            self.fail('Motor wait did not return')
        self.assertGreaterEqual(accepted, 45)
        self.assertEqual(c.machine.memory[0xff00], 1)
        self.assertEqual(c.simulator.iff, 1)
        fraction = c.machine.divram[4][c.labels['clock_fraction']]
        self.assertEqual((c.machine.memory[0xf9f8] & 15) * 50 + fraction, accepted)
        r = c.driver_call(c.labels['km_get'])
        self.assertTrue(r[1] & 1)
        self.assertEqual(r[3], 5)  # the brief A key press survived the spin-up

    def test_disckit_empty_key_poll_services_timer_and_accepts_key(self):
        for fitted in (False, True):
            with self.subTest(second_drive_fitted=fitted):
                self.check_disckit_prompt(fitted)

    def check_disckit_prompt(self, second_drive_fitted):
        class Finished(Exception):
            pass

        output = []
        waiting = None
        resumed = False
        prompt_length = 0
        fraction = self.console.labels['clock_fraction']

        def ticks(machine):
            second = machine.memory[0xf9f8]
            return ((second >> 4) * 10 + (second & 15)) * 50 + machine.divram[4][fraction]

        def observe(machine, cpu):
            nonlocal waiting, resumed, prompt_length
            r = cpu.registers
            if machine.memory[0xff00] == 2 and r[24] == self.console.labels['console_output']:
                output.append(chr(r[3]))
            if resumed and r[24] == 0xfa5a and len(output) > prompt_length + 20:
                raise Finished()
            if waiting is None:
                if ''.join(output).endswith('Press any key to continue\x1be') and r[24] == 0xfa5a:
                    waiting = r[25], ticks(machine)
                return
            elapsed = r[25] - waiting[0]
            if not resumed and elapsed >= 300 * 70908:
                self.assertFalse(machine.motor_on, 'DISCKIT polling starved the motor timeout')
                self.assertGreaterEqual((ticks(machine) - waiting[1]) % 3000, 275)
                self.assertNotEqual(machine.divram[3][0x2181], 0, 'The delayed cursor must appear')
                machine.disks[0] = None
                machine.keyboard = key_matrix('\r')
                machine.keyboard_start = r[25]
                resumed = True
                prompt_length = len(output)
            if elapsed > 600 * 70908:
                self.fail('DISCKIT did not accept a physical key after its eject prompt: PC=%04X output=%r' %
                          (r[24], ''.join(output)[prompt_length:]))

        with self.assertRaises(Finished):
            verify(self.image, self.dependencies, 16000000, 'disckit\r', observe=observe,
                   drive_b_fitted=second_drive_fitted)
        self.assertTrue(resumed)


if __name__ == '__main__':
    unittest.main()

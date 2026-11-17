"""Timer and keyboard checks, including the original +3 BIOS as a reference."""
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


class KeyboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not patch_ems.DEFAULT_INPUT.exists() or not shutil.which('sjasmplus'):
            raise unittest.SkipTest('Local original disk and sjasmplus required')
        cls.dependencies = ROOT / 'build/CPM/deps'
        if not (cls.dependencies / 'skoolkit').exists():
            raise unittest.SkipTest('Local SkoolKit 8.10 required')
        cls.temporary = tempfile.TemporaryDirectory(prefix='keyboard-tests-', dir=str(ROOT / 'build/CPM'))
        cls.work = Path(cls.temporary.name)
        cls.original = patch_ems.DEFAULT_INPUT.read_bytes()
        patched, _ = patch_ems.patch_disk(cls.original, shutil.which('sjasmplus'), cls.work)
        cls.image = cls.work / 'boot.dsk'
        cls.image.write_bytes(patched)
        from reference_plus3 import OriginalBios
        cls.reference = OriginalBios(cls.dependencies)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.console = ConsoleHarness(self.work, self.original, self.dependencies)

    def call(self, name, values=None, limit=100000):
        console = self.console
        console.select_bank(2)
        registers = console.simulator.registers
        if values:
            for index, value in values.items():
                registers[index] = value
        registers[12], registers[24] = 0xfee0, console.labels[name]
        console.machine.memory[0xfee0:0xfee2] = [0, 0xe0]
        for _ in range(limit):
            console.simulator.run()
            if registers[24] == 0xe000:
                return registers.copy()
        self.fail(name + ' did not return')

    def tick(self, keys=None, count=1):
        self.console.machine.keyboard = keys
        self.console.machine.keyboard_start = self.console.simulator.registers[25] + 10000000
        for _ in range(count):
            self.call('timer_service')

    def test_translation_matches_original_bios(self):
        # Every real key, shift combination, lock combination and repeat flag.
        for state in range(0, 256, 4):
            for key in range(40):
                with self.subTest(state=state, key=key):
                    original = self.reference.call(0x6f5, {2: state, 3: key})
                    actual = self.call('keyboard_translate', {2: state, 3: key})
                    self.assertEqual(actual[1] & 1, original[1] & 1)
                    if actual[1] & 1:
                        self.assertEqual(actual[0], original[0])

    def test_raw_matrix_tokens_match_original_including_modifiers(self):
        reference = self.reference
        def drain_native():
            result = []
            for _ in range(25):
                r = reference.call(0x790)
                if not r[1] & 1:
                    return result
                result.append(tuple(r[2:4]))
            self.fail('Native keyboard queue did not empty')
        def drain_actual():
            result = []
            for _ in range(25):
                r = self.call('km_get')
                if not r[1] & 1:
                    return result
                result.append(tuple(r[2:4]))
            self.fail('Keyboard queue did not empty')
        drain_native()
        for shifts in (set(), {(0, 0)}, {(7, 1)}, {(0, 0), (7, 1)}):
            for key in range(40):
                reference.machine.keyboard = None
                reference.call(0x837)
                self.tick()
                drain_native()
                drain_actual()
                # Reset lock state without generating a printable character.
                reference.call(0x7af, {2: 0, 3: 0})
                reference.call(0x790)
                self.call('km_put', {2: 0, 3: 0})
                self.call('km_get')
                pressed = shifts | {divmod(key, 5)}
                reference.machine.keyboard = pressed
                reference.machine.keyboard_start = reference.cpu.registers[25] + 10000000
                reference.call(0x837)
                self.tick(pressed)
                self.assertEqual(drain_actual(), drain_native(), (shifts, key))
        # A modifier still produces raw repeat tokens. Its repeat bitmap is
        # applied by character translation, not by KM KT GET.
        reference.machine.keyboard = None
        reference.call(0x837)
        self.tick()
        reference.machine.keyboard = {(0, 0)}
        reference.machine.keyboard_start = reference.cpu.registers[25] + 10000000
        reference.call(0x837)
        self.tick({(0, 0)})
        drain_native()
        drain_actual()
        for _ in range(30):
            reference.call(0x837)
            self.tick({(0, 0)})
            self.assertEqual(drain_actual(), drain_native())

    def test_clock_bcd_rollovers_and_native_cursor_delay(self):
        memory = self.console.machine.memory
        memory[0xf9f4:0xf9f9] = [0x34, 0x12, 0x23, 0x59, 0x59]
        self.tick(count=49)
        self.assertEqual(memory[0xf9f4:0xf9f9], [0x34, 0x12, 0x23, 0x59, 0x59])
        self.tick()
        self.assertEqual(memory[0xf9f4:0xf9f9], [0x35, 0x12, 0, 0, 0])
        self.console.output('\x1be')
        before = bytes(self.console.machine.ram[7])
        self.tick(count=4)
        self.assertEqual(bytes(self.console.machine.ram[7]), before)
        self.tick()
        drawn = bytes(self.console.machine.ram[7])
        self.assertNotEqual(drawn, before)
        self.tick(count=50)
        self.assertEqual(bytes(self.console.machine.ram[7]), drawn)
        memory[0xf9f8] = 0x09
        self.tick(count=50)
        self.assertEqual(memory[0xf9f8], 0x10)

    def test_empty_raw_key_poll_preserves_bank_stack_iff_and_lock_state(self):
        c = self.console
        for locks, shifts in ((0, 0), (0x58, 0xa0)):
            c.machine.memory[c.labels['key_locks']] = locks
            c.machine.memory[c.labels['physical_shifts']] = shifts
            for bank in (0, 1, 2, 4, 5, 6, 7):
                for stack in (0x3f00, 0xf3e0):
                    for iff in (0, 1):
                        with self.subTest(locks=locks, shifts=shifts, bank=bank, stack=stack, iff=iff):
                            c.machine.memory[c.labels['pending_valid']] = 1
                            before, after = c.userf(0xda, bank=bank, stack=stack, iff=iff)
                            self.assertEqual(after[2], locks | shifts)
                            self.assertFalse(after[1] & 1)
                            self.assertEqual(after[3:12] + after[16:24], before[3:12] + before[16:24])
                            self.assertEqual(c.machine.memory[c.labels['pending_valid']], 0)

    def test_tokens_repeat_locks_nul_and_expansions(self):
        self.tick({(1, 2)})  # d, key 7
        result = self.call('km_get')
        self.assertTrue(result[1] & 1)
        self.assertEqual(result[2:4], [0, 7])
        self.tick({(1, 2)}, 29)
        self.assertFalse(self.call('km_get')[1] & 1)
        self.tick({(1, 2)})
        result = self.call('km_get')
        self.assertEqual(result[2:4], [4, 7])
        self.tick({(1, 2)}, 2)
        self.assertTrue(self.call('km_get')[1] & 1)
        self.tick()
        self.tick({(0, 0), (3, 1)})  # CAPS LOCK (CAPS + 2)
        self.assertEqual(self.call('km_get')[3], 0)  # native CAPS token
        self.assertTrue(self.call('km_get')[2] & 8)
        self.tick()
        self.tick({(1, 0)})
        self.call('keyboard_peek')
        self.assertEqual(self.console.machine.memory[self.console.labels['pending_key']], ord('A'))
        self.call('console_input')
        # PUT sets locks and prepends; GET preserves DE/HL/index registers.
        before, after = self.console.userf(0xdd, values={2: 0x50, 3: 17})
        self.assertEqual(after[:12], before[:12])
        _, after = self.console.userf(0xda)
        self.assertEqual(after[2:4], [0x50, 17])
        # NUL is a real queued character, not an empty-buffer marker.
        self.call('km_put', {2: 2, 3: 16})
        self.call('keyboard_peek')
        self.assertEqual(self.console.machine.memory[self.console.labels['pending_valid']], 1)
        self.assertEqual(self.call('console_input')[0], 0)
        memory = self.console.machine.memory
        memory[0x7000:0x7005] = b'A\x00BCD'
        _, after = self.console.userf(0xd4, values={2: 0x8c, 3: 5, 6: 0x70, 7: 0})
        self.assertTrue(after[1] & 1)
        self.console.userf(0xd7, values={2: 0x8c, 3: 7, 4: 1})
        self.call('km_put', {2: 0, 3: 7})
        output = [self.call('console_input')[0] for _ in range(5)]
        self.assertEqual(bytes(output), b'A\x00BCD')

    def test_original_setkeys_and_time_programs(self):
        from verify_boot import verify
        commands = 'setkeys reset.key\rtime\r'
        with contextlib.redirect_stdout(io.StringIO()):
            # Include the native motor spin-up delays on the two program loads.
            _, text, _ = verify(self.image, self.dependencies, 7000000, commands)
        self.assertNotIn('will not run', text)
        self.assertNotIn('Bad ', text)
        self.assertNotIn('invalid', text.lower())
        self.assertIn('A>time', text)

    def test_status_keeps_token_and_userf_calls_bank0_routines(self):
        self.tick({(1, 2)})
        self.assertEqual(self.call('console_status')[0], 255)
        self.assertEqual(self.call('console_status')[0], 255)
        _, result = self.console.userf(0xda)
        self.assertEqual(result[2:4], [0, 7])
        self.assertFalse(self.call('km_get')[1] & 1)
        self.assertEqual(self.call('console_status')[0], 0)
        self.console.machine.divram[0][0x870:0x873] = [0x23, 0x13, 0xc9]
        before, result = self.console.userf(0x870, bank=1,
                                          values={4: 0x56, 5: 0x78, 6: 0x12, 7: 0x34})
        self.assertEqual(result[0:4], before[0:4])
        self.assertEqual(result[4:8], [0x56, 0x79, 0x12, 0x35])
        self.assertEqual(result[8:12] + result[16:24], before[8:12] + before[16:24])

    def test_move_interrupt_cannot_replace_saved_stack(self):
        from verify_boot import symbols
        entry = symbols(self.work / 'resident.sym')['move_memory.extended']
        c = self.console
        machine, cpu = c.machine, c.simulator
        pattern = [(i * 37 + 19) & 255 for i in range(512)]
        accepted = []
        for phase in range(6):
            with self.subTest(phase=phase):
                c.select_bank(1)
                machine.memory[0x38:0x3b] = [0xc3, 0x6c, 0xfa]
                machine.divram[4][0x2600:0x2800] = pattern
                machine.memory[0x7000:0x7200] = [0] * 512
                machine.memory[0xff16:0xff19] = [2, 1, 1]  # source/destination/extended
                machine.memory[0x9000:0x9002] = [0, 0xe0]
                r = cpu.registers
                r[2:8] = [2, 0, 0x26, 0, 0x70, 0]
                r[12], r[24] = 0x9000, entry
                cpu.iff = 1
                steps, attempted = 0, False
                for _ in range(50000):
                    if not attempted and r[12] == 0x9000 and entry <= r[24] < entry + 24:
                        if steps == phase:
                            attempted = True
                            if cpu.iff:
                                accepted.append(phase)
                                cpu.accept_interrupt(r, machine.memory, r[24])
                        steps += 1
                    cpu.run()
                    if r[24] == 0xe000:
                        break
                else:
                    self.fail('MOVE lost its return stack after a timer interrupt')
                self.assertTrue(attempted)
                self.assertEqual(r[12], 0x9002)
                self.assertEqual(machine.memory[0x7000:0x7200], pattern)
                self.assertEqual(machine.memory[0xff00], 1)
                self.assertEqual(cpu.iff, 1)
        self.assertTrue(accepted, 'Test must actually service an interrupt')

    def test_irq_preserves_registers_banks_low_stacks_and_tpa(self):
        console = self.console
        machine, cpu = console.machine, console.simulator
        for bank in (0, 1, 2, 4, 5, 6, 7):
            console.select_bank(bank)
            machine.memory[0x38:0x3b] = [0xc3, 0x6c, 0xfa]
            machine.memory[0x4000:0xf400] = [0xa5] * 0xb400
            registers = cpu.registers
            registers[:12] = [0x5a, 0xa5, 0x12, 0x34, 0x56, 0x78,
                              0x9a, 0xbc, 0xde, 0xf0, 0x13, 0x57]
            registers[12], registers[24] = 0x3f00, 0xe000
            before = registers[:12] + registers[16:24]
            stack_guard = machine.memory[0x3e80:0x3efe]
            cpu.iff = 1
            cpu.accept_interrupt(registers, machine.memory, 0xe000)
            for _ in range(20000):
                cpu.run()
                if registers[24] == 0xe000:
                    break
            else:
                self.fail('Timer interrupt did not return')
            self.assertEqual(registers[:12] + registers[16:24], before)
            self.assertEqual(registers[12], 0x3f00)
            self.assertEqual(machine.memory[0x3e80:0x3efe], stack_guard)
            self.assertEqual(machine.memory[0xff00], bank)
            self.assertEqual(cpu.iff, 1)
            self.assertEqual(machine.memory[0x4000:0xf400], [0xa5] * 0xb400)

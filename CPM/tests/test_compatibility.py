"""Regression checks for the implemented +3 EMS BIOS and FID contracts."""
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
from make_img import DPB
from test_patcher import ConsoleHarness
from test_floppy import Media
from reference_plus3 import OriginalBios
from verify_boot import simulator_class, symbols, verify
from disk import Disk


class CompatibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dependencies = ROOT / 'build/CPM/deps'
        if not patch_ems.DEFAULT_INPUT.exists() or not shutil.which('sjasmplus') or not (cls.dependencies / 'skoolkit').exists():
            raise unittest.SkipTest('Local original disk, sjasmplus and SkoolKit required')
        cls.temporary = tempfile.TemporaryDirectory(prefix='compatibility-tests-', dir=str(ROOT / 'build/CPM'))
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
        # Native spin-up encoding is checked separately; keep other probes short.
        self.console.machine.divram[4][self.console.labels['dd_setup']] = 1

    def call(self, address, bank=1, values=None, observe=None, stop=0xe000):
        c = self.console
        c.select_bank(bank)
        r = c.simulator.registers
        if values:
            for key, value in values.items():
                r[key] = value
        r[12], r[24] = 0xf380, address
        c.machine.memory[0xf380:0xf382] = [stop & 255, stop >> 8]
        for _ in range(2000000):
            if observe:
                observe(c.machine, c.simulator)
            c.simulator.run()
            if r[24] == stop:
                return r.copy()
        self.fail('Call did not return: PC=%04x bank=%d' % (r[24], c.machine.memory[0xff00]))

    def fid(self, code, response='C', message=b'Custom FID failure'):
        c = self.console
        c.select_bank(4)
        c.machine.memory[self.host['callbacks']:self.host['callbacks'] + 2] = [0, 0x20]
        for offset in (0, 3, 6, 9):
            c.machine.memory[0x2000 + offset:0x2003 + offset] = [0xc3, 0, 0x21]
        c.machine.memory[0x200c:0x200f] = [0xc3, 0, 0x22]
        c.machine.memory[0x2100:0x2100 + len(code)] = code
        c.machine.memory[0x2200:0x2205] = bytes.fromhex('21002337c9')
        c.machine.memory[0x2300:0x2300 + len(message) + 1] = message + b'\xff'
        c.machine.memory[0xff40:0xff51] = DPB
        c.machine.memory[0xff24] = 2
        c.machine.memory[0xff01:0xff04] = [1, 0, 0x90]
        prompts = []

        def observe(machine, cpu):
            if machine.memory[0xff00] == 2 and cpu.registers[24] in (0x127, 0x12a):
                entry = cpu.registers[24]
                prompts.append((entry, bytes(machine.memory[0x2800:0x2833]).split(b'\0')[0]))
                # Supply a deterministic answer, retaining the real common gate.
                machine.memory[entry:entry + 3] = [0x3e, ord(response), 0xc9]
        return prompts, observe

    def test_warm_boot_preserves_rsx_entry_and_stack_boundary(self):
        c = self.console
        c.machine.memory[0xf9fe:0xfa00] = [6, 0xe6]
        c.machine.memory[0xff2a] = 0  # FIDs have already been installed
        result = self.call(0xfa03, stop=0x100)
        self.assertEqual(c.machine.memory[5:8], [0xc3, 6, 0xe6])
        self.assertEqual(result[12], 0xe600)

    def test_submit_executes_startup_and_manual_scripts_across_warm_boots(self):
        script = b'TYPE PROBE.SUB\r\nSETDEF [ORDER=(SUB,COM)]\r\nTYPE PROBE.SUB\r\n\x1a'
        for startup in (True, False):
            with self.subTest(startup=startup):
                disk = Disk(self.image.read_bytes())
                # Leave space for both nested SUBMIT temporary files.
                for index, _ in disk.entries('BASIC.COM'):
                    disk.write(4608 + index * 32, b'\xe5')
                disk.put_file('PROBE.SUB', script)
                if startup:
                    disk.put_file('PROFILE.SUB', b'SUBMIT PROBE\r\n\x1a')
                image = self.work / ('submit-startup.dsk' if startup else 'submit-manual.dsk')
                image.write_bytes(disk.data)
                commands = 'a:setdef.com\r'
                if not startup:
                    commands = 'submit probe\r' + commands
                with contextlib.redirect_stdout(io.StringIO()):
                    _, text, _ = verify(image, self.dependencies, 20000000, commands)
                self.assertNotIn('Disk Write Error', text)
                self.assertEqual(text.count('A>TYPE PROBE.SUB'), 2)
                self.assertIn('Search Order         - SUB, COM', text)
                self.assertTrue(text.endswith('A>'))

    @staticmethod
    def error(code, zero=True):
        # LD HL,AF; PUSH HL; POP AF; LD B,7; RET. No later flags are changed.
        return bytes((0x21, 0x40 if zero else 0, code, 0xe5, 0xf1, 6, 7, 0xc9))

    def test_bdos_error_return_mode_does_not_prompt_for_missing_floppy(self):
        with contextlib.redirect_stdout(io.StringIO()):
            machine, _, _ = verify(self.image, self.dependencies, 16000000, '', drive_b_fitted=True)
        cpu = simulator_class(self.dependencies)(machine.memory, config={'fast_ldir': True, 'fast_djnz': True})
        cpu.set_tracer(machine)
        # Select TPA, BDOS 45/FF, then Search First on the absent fitted B:.
        code = bytes.fromhex('3e01cd66fa0e2d1effcd05001100910e11cd0500c30092')
        machine.memory[0x9000:0x9000 + len(code)] = code
        machine.memory[0x9100:0x9124] = bytes((2,)) + b'???????????' + bytes(24)
        cpu.registers[12], cpu.registers[24] = 0xf380, 0x9000
        for _ in range(2000000):
            self.assertFalse(machine.memory[0xff00] == 2 and cpu.registers[24] == self.console.labels['floppy_show_message'],
                             'BDOS error-return mode must not ask for keyboard input')
            cpu.run()
            if cpu.registers[24] == 0x9200:
                self.assertEqual(machine.memory[0xf9e7], 255)
                self.assertEqual(cpu.registers[0], 255)
                return
        self.fail('BDOS did not return the disk error')

    def test_d_changed_matches_native_scb_notification_and_preserves_registers(self):
        reference = OriginalBios(self.dependencies)
        reference.machine.memory[0xfc11:0xfc13] = [0, 0xc8]
        reference.machine.memory[0xf9f0] = 0
        reference.call(0xbc71, {2: 2})
        self.console.machine.memory[0xf9f0] = 0
        r = self.console.simulator.registers
        r[:12] = [0x5a, 0xa5, 2, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x13, 0x57]
        before = r[:12] + r[16:24]
        result = self.call(self.host['changed'], bank=4)
        self.assertEqual(result[:12] + result[16:24], before)
        self.assertEqual(self.console.machine.divram[0][0x80b], 255)
        self.assertEqual(self.console.machine.memory[0xf9f0], reference.machine.memory[0xf9f0])

    def test_pip_handles_write_protection_in_its_own_error_return_mode(self):
        modes = []
        def protect(machine, cpu):
            machine.write_protected[1] = True
            if machine.memory[0xff00] == 2 and cpu.registers[24] == self.console.labels['floppy_bios_io']:
                if machine.memory[self.console.labels['write_operation']]:
                    modes.append(machine.memory[0xf9e7])
            self.assertFalse(machine.memory[0xff00] == 2 and cpu.registers[24] == self.console.labels['floppy_show_message'])
        with contextlib.redirect_stdout(io.StringIO()):
            _, text, _ = verify(self.image, self.dependencies, 16000000, 'pip b:=a:profile.old\r',
                                protect, drive_b=Media())
        self.assertIn(255, modes)
        self.assertIn('MAKE FILE NONRECOVERABLE', text)
        self.assertTrue(text.endswith('A>'))

    def test_d_changed_notification_is_consumed_by_bdos_flush(self):
        from make_img import create_image
        from sd_model import SdImage
        path = self.work / 'changed.img'
        create_image(path)
        data = path.read_bytes()
        with contextlib.redirect_stdout(io.StringIO()):
            machine, _, _ = verify(self.image, self.dependencies, 16000000, 'dir c:\r', storage=SdImage(data))
        cpu = simulator_class(self.dependencies)(machine.memory, config={'fast_ldir': True, 'fast_djnz': True})
        cpu.set_tracer(machine)
        changed = self.host['changed']
        # Native BDOS processes F9F0 through its flush path. Exercise that
        # consumer, then confirm the logged-in drive can still be searched.
        code = bytes.fromhex('3e01cd66fa060221') + bytes((changed & 255, changed >> 8))
        code += bytes.fromhex('cd6ffa0e301e00cd05001100910e11cd0500c30092')
        machine.memory[0x9000:0x9000 + len(code)] = code
        machine.memory[0x9100:0x9124] = bytes((3,)) + b'???????????' + bytes(24)
        cpu.registers[12], cpu.registers[24] = 0xf380, 0x9000
        notified = False
        for _ in range(2000000):
            if machine.memory[0xf9f0] == 255:
                notified = True
            cpu.run()
            if cpu.registers[24] == 0x9200:
                self.assertTrue(notified)
                self.assertEqual(machine.memory[0xf9f0], 0)
                self.assertEqual(cpu.registers[0], 255)
                return
        self.fail('Disk-change notification did not finish the subsequent search')

    def test_fid_noninteractive_errors_preserve_codes_for_read_write_and_flush(self):
        for target in (0xfa27, 0xfa2a, 0xfa48):
            for code in (1, 2, 255):
                with self.subTest(target=target, code=code):
                    prompts, observe = self.fid(self.error(code))
                    result = self.call(target, observe=observe)
                    self.assertEqual(prompts, [])
                    self.assertEqual(result[0], code)

    def test_fid_logon_no_retry_and_bdos_error_return_mode(self):
        for zero, mode in ((True, 0), (False, 255)):
            with self.subTest(zero=zero, mode=mode):
                prompts, observe = self.fid(self.error(1, zero))
                self.console.machine.memory[0xf9e7] = mode
                result = self.call(0xfa1b, values={2: 0, 3: 2, 5: 0}, observe=observe)
                self.assertEqual(prompts, [])
                self.assertEqual(result[6:8], [0, 0])
                for target in (0xfa27, 0xfa2a, 0xfa48):
                    self.call(target, observe=observe)
                    self.assertEqual(prompts, [])

    def test_fid_custom_message_cancel_and_ignore(self):
        for response in ('C', 'I'):
            with self.subTest(response=response):
                prompts, observe = self.fid(self.error(2, False), response)
                result = self.call(0xfa2a, observe=observe)
                self.assertEqual(prompts, [(0x12a, b'Custom FID failure')])
                self.assertEqual(result[0], 2 if response == 'C' else 0)

    def test_custom_fid_question_uses_native_rotating_status_line(self):
        c = self.console
        message = b'Custom FID failure on the current storage partition'
        c.select_bank(2)
        c.machine.memory[0x2800:0x2800 + len(message) + 1] = message + b'\0'
        c.userf(0xc8, values={0: 255})
        c.select_bank(2)
        address = c.labels['console_input']
        c.machine.memory[address:address + 3] = [0x3e, ord('C'), 0xc9]
        output = []
        def observe(machine, cpu):
            if machine.lower_page == 3 and cpu.registers[24] == 0x1025:
                output.append(chr(cpu.registers[3]))
        result = self.call(0xfa63, values={3: 1, 6: 1, 7: 0x2a}, observe=observe)
        self.assertEqual(result[0], ord('C'))
        self.assertEqual(output, [])
        native_text = bytes(c.machine.divram[3][0x1deb:0x1e3b]).split(b'\xff')[0]
        self.assertTrue(native_text.startswith(b'     ' + message))
        self.assertIn(b' - Retry, Ignore', native_text)

    def test_fid_retry_repeats_io_without_extra_logon_or_losing_write_buffer(self):
        # Fail once, then succeed. Logon is a different counter to detect an
        # unsolicited logon between the two write attempts.
        code = bytes.fromhex('210024347efe0230043e01b7c9af37c9')
        prompts, observe = self.fid(code, 'R')
        self.console.machine.memory[0x2000:0x2003] = [0xc3, 0, 0x25]
        self.console.machine.memory[0x2500:0x2506] = bytes.fromhex('21012434afc9')
        self.console.machine.memory[0x9000:0x9200] = pattern = [(i * 37 + 19) & 255 for i in range(512)]
        result = self.call(0xfa2a, observe=observe)
        self.assertEqual(result[0], 0)
        self.console.select_bank(4)
        self.assertEqual(self.console.machine.memory[0x2400:0x2402], [2, 0])
        self.assertEqual(self.console.machine.memory[0x3c00:0x3e00], pattern)
        self.assertEqual(len(prompts), 1)

    def test_userf_protects_alternate_registers_for_arbitrary_and_nested_routines(self):
        clobber = bytes.fromhex('d901341211785621bc9ad908af08')
        output = bytes.fromhex('3e8101341211785621bc9a37c9')
        for nested in (False, True):
            code = clobber + (bytes.fromhex('cd5afa9008') if nested else b'') + output
            self.console.machine.divram[0][0x870:0x870 + len(code)] = code
            self.console.machine.divram[0][0x890:0x890 + len(clobber) + 1] = clobber + b'\xc9'
            for bank in (0, 1, 2):
                for stack in (0x3f00, 0x7100, 0xf3e0):
                    for iff in (0, 1):
                        with self.subTest(nested=nested, bank=bank, stack=stack, iff=iff):
                            before, result = self.console.userf(0x870, bank=bank, stack=stack, iff=iff)
                            self.assertEqual(result[16:24], before[16:24])
                            self.assertEqual(result[0], 0x81)
                            self.assertEqual(result[2:8], [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc])
                            self.assertTrue(result[1] & 1)

    def test_xbios_stubs_survive_fid_dph_and_all_header_slots(self):
        c = self.console
        c.machine.divram[0][0x800:0x860] = [0xa5] * 0x60
        c.machine.divram[0][0x900:0xb00] = [0x5a] * 0x200
        c.userf(0xbf)  # TE ASK uses the first stub region
        _, result = c.userf(0xe3)  # CD VERSION uses the other region
        self.assertEqual(result[0], 3)
        self.assertEqual(result[2:4], [0, 10])
        _, result = c.userf(0xda)
        self.assertFalse(result[1] & 1)
        self.assertEqual(c.machine.divram[0][0x800:0x860], [0xa5] * 0x60)
        self.assertEqual(c.machine.divram[0][0x900:0xb00], [0x5a] * 0x200)

    def test_dd_format_retries_transient_errors_and_bounds_exhaustion(self):
        c = self.console
        c.machine.disks[1], c.machine.fitted[1] = Media(3), True
        c.userf(0x95, values={0: 3, 8: 0xc8, 9: 0})
        headers = bytes(v for sector in range(1, 10) for v in (0, 0, sector, 2))
        c.machine.memory[0xc900:0xc900 + len(headers)] = headers
        for failures, attempts, success in ((1, 2, True), (20, 15, False)):
            with self.subTest(failures=failures):
                c.machine.fdc_operations.clear()
                c.machine.fdc_failures[:] = [{'opcode': 13, 'unit': 1, 'count': failures}]
                _, result = c.userf(0x8f, values={2: 1, 3: 1, 4: 0, 5: 0xe5, 6: 0xc9, 7: 0, 8: 0xc8, 9: 0})
                self.assertEqual(bool(result[1] & 1), success)
                self.assertEqual(len([op for op in c.machine.fdc_operations if op[0] == 13]), attempts)
                self.assertEqual(c.machine.disks[1].sector(0, 0, 1)[0], b'\xe5' * 512)

    def test_motor_status_and_timeout_contracts_match_native(self):
        reference = OriginalBios(self.dependencies)
        reference.machine.memory[0x21bc] = 1
        self.console.driver_call(self.console.labels['floppy_motor_stop'])
        reference.call(0xaa)
        _, actual = self.console.userf(0x98, values={3: 0})
        native = reference.call(0x98, {3: 0})
        self.assertEqual(actual[0] & 0x60, native[0] & 0x60)
        self.assertEqual(actual[0] & 0x60, 0x20)
        for target, timeout in ((0xa4, 0), (0xa7, 250), (0xa4, 0)):
            before, after = self.console.userf(target)
            self.assertEqual(after[:12] + after[16:24], before[:12] + before[16:24])
            reference.call(target)
            data = self.console.machine.divram[4]
            address = self.console.labels['motor_timeout']
            self.assertEqual(data[address] + 256 * data[address + 1], timeout)
            self.assertEqual(reference.machine.memory[0x21c1:0x21c3], [timeout & 255, timeout >> 8])
        for _ in range(250):
            address = self.console.labels['floppy_motor_tick']
            self.call(0xfa63, values={6: address >> 8, 7: address & 255})
        self.assertTrue(self.console.machine.motor_on, 'ON MOTOR must cancel a pending shutdown')
        self.console.userf(0xa7)
        for _ in range(250):
            self.call(0xfa63, values={6: address >> 8, 7: address & 255})
        self.assertFalse(self.console.machine.motor_on)

    def test_pcw_login_allocation_sizes_match_native_builder(self):
        reference = OriginalBios(self.dependencies)
        for kind, sides, tracks in ((0, 0, 40), (0, 128, 80), (3, 129, 80), (3, 130, 80)):
            with self.subTest(kind=kind, sides=sides, tracks=tracks):
                media = Media(kind, sides, tracks)
                self.console.machine.disks[1], self.console.machine.fitted[1] = media, True
                _, actual = self.console.userf(0x92, values={3: 1, 8: 0xc8, 9: 0})
                reference.machine.memory[0xc900:0xc90a] = media.spec
                native = reference.call(0x1939, {6: 0xc9, 7: 0, 8: 0xc8, 9: 0})
                self.assertTrue(actual[1] & 1)
                self.assertEqual(actual[4:8], native[4:8])
                self.assertEqual(self.console.machine.memory[0xc800:0xc81a], reference.machine.memory[0xc800:0xc81a])

    def test_dd_setup_specify_encoding_matches_native(self):
        reference = OriginalBios(self.dependencies)
        for step in (0, 1, 2, 3, 12, 16, 32, 255):
            for mode in (0, 3):
                with self.subTest(step=step, mode=mode):
                    params = bytes((10, 50, 175, 30, step, 15, mode))
                    self.console.machine.memory[0xc900:0xc907] = params
                    reference.machine.memory[0xc900:0xc907] = params
                    start = len(self.console.machine.ports)
                    self.console.userf(0x83, values={6: 0xc9, 7: 0})
                    actual = [v for _, p, v in self.console.machine.ports[start:] if p & 0xf002 == 0x3000]
                    start = len(reference.machine.ports)
                    reference.call(0x83, {6: 0xc9, 7: 0})
                    native = [v for _, p, v in reference.machine.ports[start:] if p & 0xf002 == 0x3000]
                    self.assertEqual(actual[-3:], native[-3:])

    def test_expansion_token_aliases_match_native(self):
        reference = OriginalBios(self.dependencies)
        for token in (0, 30, 0x80, 0x9e, 31, 0x9f, 255):
            with self.subTest(token=token):
                self.console.machine.memory[0xc900:0xc901] = b'X'
                reference.machine.memory[0xc900:0xc901] = b'X'
                values = {2: token, 3: 1, 6: 0xc9, 7: 0}
                _, actual = self.console.userf(0xd4, values=values)
                native = reference.call(0xd4, values)
                self.assertEqual(actual[1] & 1, native[1] & 1)

    def test_find_fid_masks_stored_and_query_attribute_bits(self):
        reference = OriginalBios(self.dependencies)
        for stored in (0, 128):
            for query in (0, 128):
                with self.subTest(stored=stored, query=query):
                    self.console.select_bank(4)
                    self.console.machine.memory[0xff28:0xff2a] = [0, 0x20]
                    self.console.machine.memory[0x2003:0x200b] = bytes((ord('T') | stored,)) + b'EST    '
                    self.console.machine.memory[0x2014:0x2016] = [0, 9]
                    self.console.machine.memory[0x201e:0x2020] = [0, 0]
                    reference.machine.memory[0x21ec:0x21ee] = [0, 0xc8]
                    reference.machine.memory[0xc803:0xc80b] = bytes((ord('T') | stored,)) + b'EST    '
                    reference.machine.memory[0xc81e:0xc820] = [0, 0]
                    name = bytes((ord('T') | query,)) + b'EST    '
                    self.console.machine.memory[0xc900:0xc908] = name
                    reference.machine.memory[0xc900:0xc908] = name
                    _, actual = self.console.userf(0xec, values={4: 0xc9, 5: 0})
                    native = reference.call(0xec, {4: 0xc9, 5: 0})
                    self.assertEqual(actual[1] & 1, native[1] & 1)
                    self.assertTrue(actual[1] & 1)
                    self.assertEqual(actual[6:8], [9, 0])


if __name__ == '__main__':
    unittest.main()

"""CPU integration coverage for PRL loading, SDHC and the private FID bank."""
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
from disk import Disk, DiskError
from make_img import create_image, PARTITIONS
from sd_model import SdImage
import patch_ems


def read_img_file(data, cylinder, name):
    start = cylinder * 256 * 512
    stem, _, suffix = name.upper().partition('.')
    wanted = (stem.ljust(8) + suffix.ljust(3)).encode('ascii')
    chunks = []
    for offset in range(start, start + 16384, 32):
        entry = data[offset:offset + 32]
        if entry[0] != 0 or bytes(n & 127 for n in entry[1:12]) != wanted:
            continue
        size = (entry[12] & 3) * 16384 + entry[15] * 128
        payload = bytearray()
        for block in struct.unpack('<8H', entry[16:32]):
            if not size:
                break
            if not 2 <= block <= 2039:
                raise AssertionError('Invalid IMG allocation')
            count = min(8192, size)
            payload.extend(data[start + block * 8192:start + block * 8192 + count])
            size -= count
        if size:
            raise AssertionError('Incomplete IMG allocation')
        chunks.append(((entry[12] & 28) + 32 * entry[14], bytes(payload)))
    if not chunks:
        raise AssertionError('IMG file missing: ' + name)
    return b''.join(payload for _, payload in sorted(chunks))


class AllocationTests(unittest.TestCase):
    def test_resize_and_add_preserve_files_and_reject_full_disk(self):
        from test_patcher import synthetic_disk
        disk = Disk(synthetic_disk())
        disk.put_file('BIG.EMS', bytes(range(256)) * 100)
        disk.put_file('KEEP.COM', b'kept' * 40)
        kept = disk.read_file('KEEP.COM')
        disk.put_file('BIG.EMS', b'new' * 100)
        disk.put_file('NEW.FID', b'fid' * 2000)
        self.assertEqual(Disk(disk.data).read_file('KEEP.COM'), kept)
        self.assertEqual(disk.read_file('NEW.FID')[:6000], b'fid' * 2000)
        before = bytes(disk.data)
        with self.assertRaises(DiskError):
            disk.put_file('FULL.IMG', bytes(180000))
        self.assertEqual(bytes(disk.data), before)


class FidTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not patch_ems.DEFAULT_INPUT.exists() or not shutil.which('sjasmplus'):
            raise unittest.SkipTest('Local original disk and sjasmplus required')
        if not (ROOT / 'build/CPM/deps/skoolkit').exists():
            raise unittest.SkipTest('Local SkoolKit 8.10 required')
        base = ROOT / 'build/CPM'
        base.mkdir(parents=True, exist_ok=True)
        cls.temporary = tempfile.TemporaryDirectory(prefix='fid-tests-', dir=str(base))
        cls.work = Path(cls.temporary.name)
        cls.original = patch_ems.DEFAULT_INPUT.read_bytes()
        cls.patched, cls.report = patch_ems.patch_disk(cls.original, shutil.which('sjasmplus'), cls.work)
        cls.image = cls.work / 'boot.dsk'
        cls.image.write_bytes(cls.patched)
        cls.storage_path = cls.work / 'storage.img'
        create_image(cls.storage_path)
        cls.storage_bytes = cls.storage_path.read_bytes()

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def boot(self, commands='', storage=None, image=None, observe=None, limit=12000000):
        from verify_boot import verify
        with contextlib.redirect_stdout(io.StringIO()):
            return verify(image or self.image, ROOT / 'build/CPM/deps', limit, commands, observe, storage)

    def test_fid_prl_checksum_relocations_and_image_refuses_overwrite(self):
        fid = Disk(self.patched).read_file('DIVMMC.FID')
        size = int.from_bytes(fid[1:3], 'little')
        length = 256 + size + (size + 7) // 8
        expected = int.from_bytes(fid[0x110:0x112], 'little')
        self.assertEqual((sum(fid[:length]) - sum(fid[0x110:0x112])) & 65535, expected)
        self.assertTrue(fid[256 + size] & 32)  # JP start high byte
        before = self.storage_path.read_bytes()
        self.assertEqual(before[:16], b'PLUSIDEDOS      ')
        self.assertEqual(before[40:42], b'\x38\x38')  # +3e Editor/BASIC colours
        self.assertEqual(struct.unpack_from('<I', before, 23)[0], 255)
        for offset in (64, 128, 192):
            self.assertEqual(struct.unpack_from('<I', before, offset + 23)[0], 32767)
        with self.assertRaises(FileExistsError):
            create_image(self.storage_path)
        self.assertEqual(self.storage_path.read_bytes(), before)

    def test_signon_is_printed_once_before_ccp(self):
        machine, text, _ = self.boot('basic\rsystem\r', SdImage(self.storage_bytes))
        signon = 'DivMMC SDHC disk FID\r\n'
        self.assertEqual(text.count('CP/M Plus - ZXTeensyIF1'), 1)
        self.assertEqual(text.count('61K TPA, 1 floppy drive\r\n'), 1)
        self.assertEqual(text.count(signon), 1)
        self.assertLess(text.index('61K TPA'), text.index(signon))
        self.assertLess(text.index(signon), text.index('A>'))
        self.assertIn('BASIC', text)
        self.assertGreaterEqual(text.count('A>'), 2)
        self.assertNotIn('\xff', text)
        self.assertNotEqual(machine.memory[0xfe3e:0xfe44], [0] * 6)

    def test_copy_three_drives_reboot_basic_and_delete(self):
        storage = SdImage(self.storage_bytes, response_delay=7)
        commands = 'pip c:=a:basic.com\rpip d:=c:basic.com\rpip e:=d:basic.com\rpip c:=a:profile.old\r'
        # The extra startup rows can make the final prompt scroll the screen.
        machine, text, _ = self.boot(commands, storage, limit=14000000)
        self.assertNotIn('error', text.lower())
        original = Disk(self.original)
        for _, cylinder in PARTITIONS:
            self.assertEqual(read_img_file(storage.data, cylinder, 'BASIC.COM'), original.read_file('BASIC.COM'))
        self.assertEqual(read_img_file(storage.data, 1, 'PROFILE.OLD'), original.read_file('PROFILE.SUB'))
        self.assertTrue(storage.writes)
        snapshot = bytes(storage.data)
        again = SdImage(snapshot, response_delay=11)
        commands = 'dir c:\rdir d:\rdir e:\re:basic\rprint 2+2\rsystem\rera d:basic.com\rdir d:\r'
        machine, text, _ = self.boot(commands, again)
        self.assertNotIn('CP/M Error', text)
        self.assertIn('\r\n 4 \r\nOk', text)
        self.assertIn('No File', text)
        self.assertEqual(read_img_file(again.data, 257, 'BASIC.COM'), original.read_file('BASIC.COM'))
        with self.assertRaisesRegex(AssertionError, 'missing'):
            read_img_file(again.data, 129, 'BASIC.COM')
        self.assertEqual(self.storage_path.read_bytes(), self.storage_bytes)
        self.assertEqual(patch_ems.DEFAULT_INPUT.read_bytes(), self.original)

    def test_missing_image_and_bad_fid_fail_without_breaking_a(self):
        for damage in ('missing_image', 'checksum', 'machine', 'bounds', 'svc', 'entry'):
            with self.subTest(damage=damage):
                disk = Disk(self.patched)
                fid = bytearray(disk.read_file('DIVMMC.FID'))
                if damage == 'checksum':
                    fid[0x110] ^= 1
                elif damage == 'machine':
                    fid[0x103] = ord('X')
                elif damage in ('bounds', 'svc', 'entry'):
                    if damage == 'bounds':
                        fid[0x112] = 0x40  # outside this host's low 16K window
                    elif damage == 'entry':
                        fid[0x101:0x103] = bytes(2)
                    else:
                        # A supervisor word with an out-of-range index and a
                        # valid checksum must fail before executing the FID.
                        size = int.from_bytes(fid[1:3], 'little')
                        for n in range(size):
                            if fid[256 + n] == 255 and n and fid[255 + n] == 7 and fid[256 + size + n // 8] & (128 >> (n % 8)):
                                fid[255 + n] = 11
                                break
                        else:
                            self.fail('Allocate SVC relocation not found')
                    fid[0x110:0x112] = bytes(2)
                    length = 256 + int.from_bytes(fid[1:3], 'little')
                    length += (length - 256 + 7) // 8
                    fid[0x110:0x112] = (sum(fid[:length]) & 65535).to_bytes(2, 'little')
                disk.replace_file('DIVMMC.FID', fid)
                image = self.work / ('bad-' + damage + '.dsk')
                image.write_bytes(disk.data)
                storage = None if damage == 'missing_image' else SdImage(self.storage_bytes)
                machine, text, _ = self.boot('dir\r', storage, image)
                self.assertIn('A: S10CPM3', text)
                self.assertNotIn('CP/M Error', text)
                self.assertEqual(machine.memory[0xfe3e:0xfe44], [0] * 6)
                if damage == 'missing_image':
                    self.assertEqual(text.count('DivMMC image unavailable\r\n'), 1)
                    self.assertLess(text.index('DivMMC image unavailable'), text.index('A>'))
                else:
                    self.assertNotIn('DivMMC image unavailable', text)
                self.assertNotIn('DivMMC SDHC disk FID', text)
                if storage:
                    self.assertFalse(storage.commands)

    def test_failed_entry_rolls_back_allocations_and_table_fallback(self):
        from verify_boot import symbols
        # A new test FID allocates 512 bytes and rejects startup. Both its
        # code reservation and its supervisor allocation must be reclaimed.
        code = bytearray(32)
        code[:3] = bytes((0xc3, 0x20, 1))
        code[3:14] = b'SPECTRUMFID'
        message = b'Test FID declined startup after allocating its private scratch buffer\r\n'
        code.extend(bytes((0x11, 0, 2, 0xcd, 7, 255, 0x21, 43, 1, 0xb7, 0xc9)))
        code.extend(message + b'\xff')
        bitmap = bytearray((len(code) + 7) // 8)
        for offset in (2, 37, 40):
            bitmap[offset // 8] |= 128 >> (offset % 8)
        prl = bytearray(256)
        prl[1:3] = len(code).to_bytes(2, 'little')
        prl += code + bitmap
        prl[0x110:0x112] = (sum(prl) & 65535).to_bytes(2, 'little')
        disk = Disk(self.patched)
        disk.put_file('FAIL.FID', prl)
        image = self.work / 'failed-start.dsk'
        image.write_bytes(disk.data)
        storage_bytes = bytearray(self.storage_bytes)
        storage_bytes[128 * 512:128 * 512 + 4096] = storage_bytes[:4096]
        storage_bytes[:4096] = bytes(4096)
        storage_bytes[128 * 512 + 10:128 * 512 + 16] = bytes(6)  # accept old zero padding too
        storage = SdImage(storage_bytes)
        machine, text, _ = self.boot('dir c:\r', storage, image)
        self.assertNotIn('error', text.lower())
        self.assertEqual(text.count(message.decode('ascii')), 1)
        self.assertEqual(text.count('DivMMC SDHC disk FID\r\n'), 1)
        self.assertLess(text.index(message.decode('ascii')), text.index('A>'))
        self.assertNotIn('\xff', text)
        self.assertIn('No File', text)
        self.assertEqual(storage.reads[:2], [0, 128])
        labels = symbols(self.work / 'fid_host.sym')
        private = machine.divram[2]
        records = private[labels['allocations']:labels['allocations'] + 64]
        active = [records[n:n + 4] for n in range(0, 64, 4) if records[n:n + 2] != [0, 0]]
        self.assertEqual(active, [[0, 0x3a, 0, 2]])  # retained IMG table buffer
        self.assertEqual(private[labels['heap_high']:labels['heap_high'] + 2], [0, 0x3a])
        # A rejected FID is absent from the bank-0 mirror chain.
        self.assertEqual(private[labels['next_alias']:labels['next_alias'] + 2], [32, 9])

    def test_dma_crossing_bank_boundary_errors_and_find_header(self):
        from verify_boot import simulator_class
        storage = SdImage(self.storage_bytes)
        machine, _, _ = self.boot(storage=storage)
        simulator = simulator_class(ROOT / 'build/CPM/deps')(
            machine.memory, registers={'IX': 0x1234, 'IY': 0x5678},
            config={'fast_ldir': True, 'fast_djnz': True})
        simulator.set_tracer(machine)

        def call(address, stop=0xe000):
            registers = simulator.registers
            registers[12], registers[24] = 0xf380, address
            machine.memory[0xf380:0xf382] = [0, 0xe0]
            preserved = registers[8:12] + registers[16:24]
            last_frame = registers[25] // 70908
            waiting_since = None
            replied = False
            for _ in range(500000):
                pc = registers[24]
                if machine.memory[0xff00] == 2 and machine.memory[pc] == 0x76 and not replied:
                    if waiting_since is None:
                        machine.keyboard = None
                        waiting_since = registers[25]
                    elif registers[25] - waiting_since >= 220000:
                        from verify_boot import key_matrix
                        machine.keyboard = key_matrix('c')  # exercise explicit Cancel
                        machine.keyboard_start = registers[25]
                        replied = True
                simulator.run()
                frame = registers[25] // 70908
                if frame != last_frame:
                    if simulator.iff:
                        simulator.accept_interrupt(registers, machine.memory, pc)
                    last_frame = frame
                if registers[24] == stop:
                    self.assertEqual(registers[8:12] + registers[16:24], preserved)
                    self.assertEqual(registers[12], 0xf382)
                    return registers
            self.fail('BIOS disk/FIND call did not return')

        simulator.registers[0] = 1
        call(0xfa66)
        simulator.registers[2:6] = [0, 2, 0, 0]  # SELDSK C:, first login
        registers = call(0xfa1b)
        self.assertEqual(registers[6:8], [8, 0])
        pattern = [(n * 37 + 19) & 255 for n in range(512)]
        machine.memory[0x3f00:0x4100] = pattern
        machine.memory[0xff01:0xff08] = [1, 0, 0x3f, 16, 0, 127, 0]
        simulator.registers[3] = 0
        self.assertEqual(call(0xfa2a)[0], 0)
        lba = 256 + 16 * 128 + 127
        self.assertEqual(storage.data[lba * 512:(lba + 1) * 512], bytes(pattern))
        machine.memory[0x3f00:0x4100] = [0] * 512
        self.assertEqual(call(0xfa27)[0], 0)
        self.assertEqual(machine.memory[0x3f00:0x4100], pattern)
        self.assertEqual(machine.memory[0xff00], 1)
        self.assertEqual(machine.lower_page, 1)
        # End-of-partition LBAs, including the carry across FFFF on D:.
        for drive, start in ((2, 256), (3, 33024), (4, 65792)):
            simulator.registers[2:6] = [0, drive, 0, 0]
            self.assertNotEqual(call(0xfa1b)[6:8], [0, 0])
            machine.memory[0xff04:0xff08] = [255, 0, 127, 0]
            machine.memory[0x3f00:0x4100] = pattern
            self.assertEqual(call(0xfa2a)[0], 0)
            last = start + 32767
            self.assertEqual(storage.data[last * 512:(last + 1) * 512], bytes(pattern))
        simulator.registers[2:6] = [0, 2, 0, 1]
        call(0xfa1b)
        machine.memory[0xff04:0xff08] = [16, 0, 127, 0]
        storage.read_only = True
        before = bytes(storage.data[lba * 512:(lba + 1) * 512])
        machine.memory[0x3f00] ^= 1
        self.assertEqual(call(0xfa2a)[0], 1)
        self.assertEqual(storage.data[lba * 512:(lba + 1) * 512], before)
        self.assertEqual(call(0xfa27)[0], 0)  # error leaves the SPI interface usable
        machine.memory[0xff05] = 1  # track >= 256: reject without wrapping LBA
        reads = len(storage.reads)
        self.assertEqual(call(0xfa27)[0], 1)
        self.assertEqual(len(storage.reads), reads)
        simulator.registers[0] = 0
        call(0xfa66)
        machine.memory[0x7000:0x7008] = b'DIVMMC  '
        machine.memory[0xe000:0xe002] = [0xec, 0]
        simulator.registers[4:6] = [0x70, 0]
        registers = call(0xfa5a, 0xe002)
        address = registers[6] * 256 + registers[7]
        self.assertTrue(registers[1] & 1)  # FIND FID success is carry set
        self.assertEqual(address, 0x900)
        self.assertEqual(bytes(machine.memory[address + 3:address + 14]), b'DIVMMC  FID')
        self.assertEqual(machine.memory[0xff00], 0)

    def test_allram_selector_and_allocator_failure(self):
        from verify_boot import Machine, simulator_class, symbols
        machine = Machine(Disk(self.patched))
        host = (self.work / 'fid_host.bin').read_bytes()
        machine.divram[2][:len(host)] = host
        machine.ram[0][0x3a00:] = (self.work / 'resident.bin').read_bytes()
        simulator = simulator_class(ROOT / 'build/CPM/deps')(
            machine.memory, config={'fast_ldir': True, 'fast_djnz': True})
        simulator.set_tracer(machine)
        for port, value in ((0x0f3b, 0xa0), (0x7ffd, 8), (0xe3, 0)):
            machine.write_port(simulator.registers, port, value)
        machine.memory[0xff25] = 0x18

        def call(address):
            registers = simulator.registers
            registers[12], registers[24] = 0xf380, address
            machine.memory[0xf380:0xf382] = [0, 0xe0]
            for _ in range(10000):
                simulator.run()
                if registers[24] == 0xe000:
                    return registers
            self.fail('Private bank call did not return')

        for bank, page in enumerate((0, 1, 4, 5, 2, 3, 6, 7)):
            simulator.registers[0] = bank
            call(0xfa66)
            self.assertEqual(machine.lower_page, page)
            self.assertEqual(machine.allram_page(), page)
            self.assertEqual(machine.port1 & 0x18, 0x18)
        simulator.registers[0] = 4
        call(0xfa66)
        call(0x100)
        labels = symbols(self.work / 'fid_host.sym')
        simulator.registers[4:6] = [0x40, 0]
        registers = call(labels['allocate'])
        self.assertFalse(registers[1] & 1)  # out-of-memory is NC, never a pointer
        simulator.registers[4:6] = [2, 0]
        registers = call(labels['allocate'])
        self.assertTrue(registers[1] & 1)
        pointer = registers[6] * 256 + registers[7]
        self.assertEqual(pointer, 0x3a00)
        simulator.registers[4:6] = [2, 0]
        call(labels['deallocate'])
        self.assertEqual(machine.memory[labels['heap_high']:labels['heap_high'] + 2], [0, 0x3c])

        # Free the older allocation first, then reuse its hole. MAX_ALLOCATE
        # must find the largest remaining contiguous gap and report its size.
        def allocate(size):
            simulator.registers[4:6] = [size >> 8, size & 255]
            result = call(labels['allocate'])
            self.assertTrue(result[1] & 1)
            return result[6] * 256 + result[7]

        def release(pointer, size):
            simulator.registers[4:8] = [size >> 8, size & 255, pointer >> 8, pointer & 255]
            call(labels['deallocate'])

        first, second = allocate(512), allocate(512)
        release(first, 512)
        third = allocate(256)
        self.assertEqual(third, 0x3b00)
        release(second, 512)
        result = call(labels['max_allocate'])
        self.assertTrue(result[1] & 1)
        heap_low = machine.memory[labels['heap_low']] + 256 * machine.memory[labels['heap_low'] + 1]
        self.assertEqual(result[6] * 256 + result[7], heap_low)
        self.assertEqual(result[4] * 256 + result[5], third - heap_low)
        release(heap_low, third - heap_low)
        release(third, 256)
        self.assertEqual(machine.memory[labels['heap_high']:labels['heap_high'] + 2], [0, 0x3c])

        for iff in (0, 1):
            simulator.iff = iff
            simulator.registers[:12] = [0x5a, 0xa5, 2, 0x34, 0x56, 0x78,
                                        0x9a, 0xbc, 0xde, 0xf0, 0x13, 0x57]
            before = simulator.registers[:12] + simulator.registers[16:24]
            call(labels['changed'])
            self.assertEqual(simulator.registers[:12] + simulator.registers[16:24], before)
            self.assertEqual(simulator.iff, iff)
            self.assertEqual(machine.divram[0][0x80b], 255)

        pointers = [allocate(8) for _ in range(16)]
        simulator.registers[4:6] = [0, 8]
        self.assertFalse(call(labels['allocate'])[1] & 1)  # bounded record table
        for pointer in pointers[::2] + pointers[1::2]:
            release(pointer, 8)
        self.assertEqual(machine.memory[labels['heap_high']:labels['heap_high'] + 2], [0, 0x3c])

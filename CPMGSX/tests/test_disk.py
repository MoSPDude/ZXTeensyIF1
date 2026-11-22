"""Disk preparation checks; no original OS or GSX driver bytes are bundled."""
import contextlib
import hashlib
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1]
ROOT = SOURCE.parent
sys.path.insert(0, str(SOURCE))
from disk_image import CpmFiles, ImageError, read_dsk, read_image, standard_dsk
from inspect_disk import convert, inspect
from teledisk import crc16, expand_sector, read_sectors


def volume(double=False):
    spec = (3, 129, 80, 9, 2, 1, 4, 4) if double else (0, 0, 40, 9, 2, 1, 3, 2)
    sectors = {(c, h, r): b'\xe5' * 512 for c in range(spec[2])
               for h in range(2 if double else 1) for r in range(1, 10)}
    sectors[0, 0, 1] = bytes(spec).ljust(512, b'\xe5')
    return sectors


def add_file(sectors, payload, blocks, user=0, first_slot=0, name=b'DDSCREENPRL'):
    """Independent allocator for test files in alternating-head volumes."""
    double = len(sectors) > 360
    block_size, pointer_size, reserved = (2048, 2, 4) if double else (1024, 1, 2)
    raw = bytearray(b''.join(sectors[k] for k in sorted(sectors)))
    capacity = block_size * 16 // pointer_size
    for n, block in enumerate(blocks):
        offset = 4608 + block * block_size
        raw[offset:offset + block_size] = payload[n * block_size:(n + 1) * block_size].ljust(block_size, b'\x1a')
    for extent in range((len(payload) + capacity - 1) // capacity):
        entry = bytearray(32)
        entry[0], entry[1:12] = user, name
        entry[12], entry[14] = extent & 31, extent >> 5
        entry[15] = (min(capacity, len(payload) - extent * capacity) + 127) // 128
        count = 16 // pointer_size
        for n, block in enumerate(blocks[extent * count:(extent + 1) * count]):
            entry[16 + n * pointer_size:16 + (n + 1) * pointer_size] = block.to_bytes(pointer_size, 'little')
        pos = 4608 + (first_slot + extent) * 32
        raw[pos:pos + 32] = entry
    return {key: bytes(raw[n * 512:(n + 1) * 512]) for n, key in enumerate(sorted(sectors))}


def td0(payload=b'A' * 512, encoding=None, comment=False):
    header = bytearray(b'TD\0\0\x15\0\x03\0\0\x01')
    if comment:
        header[7] = 128
    result = header + struct.pack('<H', crc16(header))
    if comment:
        text = b'Synthetic test\0'
        record = struct.pack('<H6B', len(text), 126, 10, 22, 0, 0, 0) + text
        result += struct.pack('<H', crc16(record)) + record
    track = b'\x01\0\0'
    result += track + bytes((crc16(track) & 255,))
    result += bytes((0, 0, 1, 2, 0, crc16(payload) & 255))
    encoded = b'\0' + payload if encoding is None else encoding
    return bytes(result + struct.pack('<H', len(encoded)) + encoded + b'\xff')


class TelediskTests(unittest.TestCase):
    def test_raw_and_comment(self):
        for comment in (False, True):
            self.assertEqual(read_sectors(td0(comment=comment)), {(0, 0, 1): b'A' * 512})

    def test_repeated_word(self):
        self.assertEqual(read_sectors(td0(encoding=b'\x01\0\x01AA'))[0, 0, 1], b'A' * 512)

    def test_fragmented_rle_and_power_of_two_patterns(self):
        encoded = b'\x02\x00\x02AB\x01\x03CD\x03\x02abcdefgh'
        self.assertEqual(expand_sector(encoded, 24), b'ABCDCDCDabcdefghabcdefgh')

    def test_bad_sector_length(self):
        for encoded in (b'', b'\x03', b'\x01\0', b'\x02\0\x03A', b'\x02\x01\xffAB'):
            with self.subTest(encoded=encoded), self.assertRaises(ImageError):
                expand_sector(encoded, 128)

    def test_crc_rejection(self):
        for offset in (4, 15, 21, 25):
            data = bytearray(td0())
            data[offset] ^= 1
            with self.subTest(offset=offset), self.assertRaises(ImageError):
                read_sectors(data)

    def test_truncation(self):
        data = td0()
        for length in (0, 11, 14, 22, len(data) - 1):
            with self.subTest(length=length), self.assertRaises(ImageError):
                read_sectors(data[:length])


class DiskTests(unittest.TestCase):
    def test_standard_dsk_sector_round_trip(self):
        for double in (False, True):
            sectors = volume(double)
            self.assertEqual(read_dsk(standard_dsk(sectors)), sectors)

    def test_extended_interleaved_dsk(self):
        sectors = volume()
        data = bytearray(standard_dsk(sectors))
        data[:34] = b'EXTENDED CPC DSK File\r\nDisk-Info\r\n'
        data[50:52] = b'\0\0'
        data[52:92] = bytes((19,)) * 40
        for track in range(40):
            pos = 256 + track * 4864
            for n, r in enumerate((1, 6, 2, 7, 3, 8, 4, 9, 5)):
                struct.pack_into('<6BH', data, pos + 24 + n * 8, track, 0, r, 2, 0, 0, 512)
                data[pos + 256 + n * 512:pos + 768 + n * 512] = sectors[track, 0, r]
        self.assertEqual(read_dsk(data), sectors)

    def test_fragmentation_and_multiple_extents(self):
        payload = bytes(range(256)) * 67
        blocks = list(reversed(range(2, 19)))
        filesystem = CpmFiles(add_file(volume(), payload, blocks))
        self.assertEqual(filesystem.files[0, 'DDSCREEN.PRL'], payload)

    def test_720k_sixteen_bit_pointers_and_user_area(self):
        payload = b'Driver test\r\n' * 200
        sectors = add_file(volume(True), payload, [300, 299], user=7)
        filesystem = CpmFiles(sectors)
        self.assertEqual(filesystem.files[7, 'DDSCREEN.PRL'][:len(payload)], payload)
        with contextlib.redirect_stdout(io.StringIO()) as report:
            inspect(standard_dsk(sectors))
        self.assertIn('DDSCREEN.PRL user 7', report.getvalue())

    def test_missing_driver_report(self):
        with contextlib.redirect_stdout(io.StringIO()) as report:
            inspect(standard_dsk(volume()))
        self.assertIn('DDSCREEN.PRL MISSING', report.getvalue())
        self.assertIn('Cannot patch', report.getvalue())

    def test_cross_linked_blocks_rejected(self):
        sectors = add_file(volume(), b'A' * 2048, [2, 2])
        with self.assertRaisesRegex(ImageError, 'cross-linked'):
            CpmFiles(sectors)

    def test_missing_sector_and_bad_dsk_rejected(self):
        sectors = volume()
        del sectors[3, 0, 5]
        with self.assertRaises(ImageError):
            standard_dsk(sectors)
        with self.assertRaises(ImageError):
            CpmFiles(sectors)
        data = standard_dsk(volume())
        for broken in (data[:-1], data + b'extra', b'x' + data[1:]):
            with self.subTest(size=len(broken)), self.assertRaises(ImageError):
                read_dsk(broken)

    def test_source_and_existing_output_protected(self):
        work = ROOT / 'build/CPMGSX'
        work.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=str(work)) as temporary:
            source = Path(temporary) / 'source.dsk'
            output = Path(temporary) / 'converted.dsk'
            source.write_bytes(b'original')
            output.write_bytes(b'existing')
            with self.assertRaises(ImageError):
                convert(volume(), source, source)
            with self.assertRaises(FileExistsError):
                convert(volume(), source, output)
            self.assertEqual(source.read_bytes(), b'original')
            self.assertEqual(output.read_bytes(), b'existing')


class LocalImageTests(unittest.TestCase):
    def test_supplied_advanced_td0_and_independent_reader(self):
        source = ROOT / 'ignoredfiles/pcw9512.td0'
        if not source.exists():
            self.skipTest('Local original disk is unavailable')
        data = source.read_bytes()
        if hashlib.sha256(data).hexdigest() != 'ef66405aa0c3239b37840e6200f91fcbcf67adb22935167f8ea19a1057c8aa06':
            self.skipTest('The local disk has been replaced with a different revision')
        sectors = read_image(data)
        raw = b''.join(sectors[k] for k in sorted(sectors))
        self.assertEqual(hashlib.sha256(raw).hexdigest(),
                         '39730dc7d880edf2b6f1954b78ea65fcfb8777216ee9236c8778dc0da9ede436')
        self.assertEqual(len(sectors), 360)
        files = CpmFiles(sectors).files
        self.assertEqual(len(files), 25)
        self.assertNotIn((0, 'DDSCREEN.PRL'), files)
        sys.path.insert(0, str(ROOT / 'CPM'))
        from disk import Disk
        independent = Disk(standard_dsk(sectors))
        self.assertEqual(independent.read(0, len(raw)), raw)
        for (user, name), payload in files.items():
            self.assertEqual(user, 0)
            self.assertEqual(independent.read_file(name), payload)


if __name__ == '__main__':
    unittest.main()

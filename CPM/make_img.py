"""Create a new, empty 64MiB IDEDOS IMG for the DivMMC CP/M disk FID."""
import argparse
from pathlib import Path
import struct

DPB = struct.pack('<HBBBHHBBHHBB', 512, 6, 63, 3, 2039, 511, 0xc0, 0, 0x8000, 0, 2, 3)
PARTITIONS = (('CP/M', 1), ('SOFTWARE', 129), ('DEVEL', 257))
IMAGE_SIZE = 64 * 1024 * 1024


def create_image(path):
    table = bytearray(4096)
    table[:16] = b'PLUSIDEDOS'.ljust(16, b' ')
    table[16] = 1
    table[22] = 1
    struct.pack_into('<I', table, 23, 255)  # highest logical sector, not count
    struct.pack_into('<HBBHH', table, 32, 512, 2, 128, 256, 64)
    # +3e restores Editor/BASIC colours from system-partition info bytes 8/9.
    # Zeroes mean black on black; use the normal black-on-white attributes.
    table[40:42] = b'\x38\x38'
    table[50] = 255  # no IDEDOS boot filename
    for index, (name, cylinder) in enumerate(PARTITIONS, 1):
        offset = index * 64
        table[offset:offset + 16] = name.ljust(16).encode('ascii')
        table[offset + 16] = 3
        struct.pack_into('<HBHB', table, offset + 17, cylinder, 0, cylinder + 127, 1)
        struct.pack_into('<I', table, offset + 23, 32767)
        table[offset + 32:offset + 49] = DPB
    # Describe the remaining cylinders as free space for IDEDOS tools.
    table[256:272] = b' ' * 16
    table[272] = 255
    struct.pack_into('<HBHBI', table, 273, 385, 0, 511, 1, 127 * 256 - 1)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as image:
        image.truncate(IMAGE_SIZE)
        image.write(table)
        for _, cylinder in PARTITIONS:
            image.seek(cylinder * 256 * 512)
            image.write(b'\xe5' * 16384)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    try:
        create_image(args.output)
        print('Created %s (C: CP/M, D: SOFTWARE, E: DEVEL; 16MiB each)' % args.output.resolve())
    except OSError as error:
        parser.exit(1, 'error: %s\n' % error)


if __name__ == '__main__':
    main()

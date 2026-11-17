#!/usr/bin/env python3
"""Create an empty, formatted CP/M data DSK without operating-system bytes."""
import argparse
from pathlib import Path
import struct


FORMATS = {
    'plus3': (0, 0, 40, 9, 2, 1, 3, 2, 42, 82),
    'cpc-system': (1, 0, 40, 9, 2, 2, 3, 2, 42, 82),
    'cpc-data': (2, 0, 40, 9, 2, 0, 3, 2, 42, 82),
    'pcw720': (3, 129, 80, 9, 2, 1, 4, 4, 42, 82),
}


def create_image(path, format_name='pcw720'):
    spec = FORMATS[format_name]
    heads = 1 if spec[1] & 127 == 0 else 2
    first_id = (1, 65, 193, 1)[spec[0]]
    track_bytes = 256 + 512 * spec[3]
    header = bytearray(256)
    header[:34] = b'MV - CPCEMU Disk-File\r\nDisk-Info\r\n'
    header[34:48] = b'ZXTeensyIF1'.ljust(14)
    header[48:50] = bytes((spec[2], heads))
    struct.pack_into('<H', header, 50, track_bytes)
    data = bytearray(header)
    for cylinder in range(spec[2]):
        for head in range(heads):
            track = bytearray(256)
            track[:12] = b'Track-Info\r\n'
            track[16:18] = bytes((cylinder, head))
            track[20:24] = bytes((2, spec[3], spec[9], 229))
            for sector in range(spec[3]):
                struct.pack_into('<6BH', track, 24 + sector * 8,
                                 cylinder, head, first_id + sector, 2, 0, 0, 0)
            data.extend(track)
            data.extend(bytes((229,)) * (512 * spec[3]))
    if spec[0] in (0, 3):
        boot = bytearray((229,)) * 512
        boot[:16] = bytes(spec) + bytes(6)
        boot[15] = (-sum(boot)) & 255  # explicitly non-bootable (+3 needs sum 3)
        data[512:1024] = boot
    with Path(path).open('xb') as output:
        output.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--format', choices=sorted(FORMATS), default='pcw720')
    args = parser.parse_args()
    try:
        create_image(args.output, args.format)
    except (OSError, ValueError) as error:
        parser.exit(1, 'error: %s\n' % error)
    print('Created %s (%s); no boot system is included.' % (args.output, args.format))


if __name__ == '__main__':
    main()

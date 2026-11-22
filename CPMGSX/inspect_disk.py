#!/usr/bin/env python3
"""Inspect a PCW GSX source disk, optionally converting TD0 to regular DSK.

Conversion preserves the PCW files and boot sector; it does not port a driver
or create a Spectrum boot disk. Driver patching requires DDSCREEN.PRL.
"""
import argparse
import hashlib
from pathlib import Path

from disk_image import CpmFiles, ImageError, read_image, read_dsk, standard_dsk


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = ROOT / 'ignoredfiles/Pascal 80 (Side B).dsk'
GSX_NAMES = ('DDSCREEN.PRL', 'GSX.SYS', 'GENGRAF.COM', 'ASSIGN.SYS')


def inspect(data):
    sectors = read_image(data)
    filesystem = CpmFiles(sectors)
    print('%d cylinders, %d side(s), %d sectors, %d-byte CP/M blocks' %
          (filesystem.cylinders, filesystem.heads, len(sectors), filesystem.block_size))
    for (user, name), payload in sorted(filesystem.files.items()):
        print('User %2d  %-12s %6d bytes' % (user, name, len(payload)))
    print('GSX components:')
    for wanted in GSX_NAMES:
        matches = [(u, p) for (u, n), p in filesystem.files.items() if n == wanted]
        if not matches:
            print('  %-12s MISSING' % wanted)
        for user, payload in matches:
            print('  %-12s user %d, SHA256 %s' % (wanted, user, hashlib.sha256(payload).hexdigest()))
    if not any(n == 'DDSCREEN.PRL' for _, n in filesystem.files):
        print('Cannot patch a GSX screen driver: DDSCREEN.PRL is absent from this disk.')
    return sectors


def convert(sectors, source, output):
    if source.resolve() == output.resolve():
        raise ImageError('Refusing to overwrite the input image')
    data = standard_dsk(sectors)
    if read_dsk(data) != sectors:
        raise ImageError('Converted DSK failed sector-by-sector verification')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        stream.write(data)
    print('Created %s; all sector bytes verified against the source.' % output)
    print('This is a container conversion, with the original PCW boot code and files.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--convert', type=Path, metavar='OUTPUT.DSK',
                        help='convert all sectors to standard CPC DSK; never overwrite an existing file')
    args = parser.parse_args(argv)
    try:
        print('Source: %s' % args.input)
        sectors = inspect(args.input.read_bytes())
        if args.convert:
            convert(sectors, args.input, args.convert)
    except (OSError, ValueError) as error:
        parser.exit(1, 'error: %s\n' % error)


if __name__ == '__main__':
    main()

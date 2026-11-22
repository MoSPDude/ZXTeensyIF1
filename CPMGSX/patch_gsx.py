#!/usr/bin/env python3
"""Build a Spectrum GSX driver DSK from a locally supplied PCW GIOS #8."""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from disk_image import CpmFiles, ImageError, read_image


SOURCE = Path(__file__).resolve().parent
ROOT = SOURCE.parent
DEFAULT_INPUT = ROOT / 'ignoredfiles/Pascal 80 (Side B).dsk'
DEFAULT_OUTPUT = ROOT / 'build/CPMGSX/GSX.DSK'
DRIVER_SHA256 = 'f9b371b81c5a20b492995f162b17b6d2230d96336917e9dbfd38480921fbc177'


def read_prl(data):
    if len(data) < 256:
        raise ImageError('Truncated PRL header')
    length = int.from_bytes(data[1:3], 'little')
    end = 256 + length + (length + 7) // 8
    if not length or len(data) < end:
        raise ImageError('Truncated PRL code or relocation bitmap')
    return bytes(data[:256]), bytes(data[256:256 + length]), bytes(data[256 + length:end])


def source_driver(data):
    files = CpmFiles(read_image(data)).files
    matches = [p for (_, name), p in files.items() if name == 'DDSCREEN.PRL']
    if not matches:
        raise ImageError('DDSCREEN.PRL is absent from this disk; supply a PCW GSX/DR Draw/Graph disk')
    valid = []
    for payload in matches:
        header, code, bitmap = read_prl(payload)
        digest = hashlib.sha256(header + code + bitmap).hexdigest()
        if digest == DRIVER_SHA256:
            valid.append(payload)
    if not valid:
        raise ImageError('Unsupported DDSCREEN.PRL revision; expected the supplied PCW8256 GIOS #8')
    return valid[0]


def symbols(path):
    result = {}
    for line in path.read_text().splitlines():
        if ': EQU ' in line:
            name, value = line.split(': EQU ')
            result[name] = int(value.strip(), 16)
    return result


def assemble(assembler, name, work, delta=None):
    output = work / (name + ('' if delta is None else str(delta)) + '.bin')
    command = [assembler, '--nologo', '--inc=' + str(work), '--raw=' + str(output),
               '--sym=' + str(output.with_suffix('.sym'))]
    if delta is not None:
        command.append('-DPATCH_DELTA=%d' % delta)
    command.append(str(SOURCE / (name + '.asm')))
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                universal_newlines=True, timeout=60)
    except subprocess.TimeoutExpired:
        raise ImageError('Assembly timed out: ' + name)
    if result.returncode or not output.exists():
        raise ImageError('Assembly failed for %s:\n%s' % (name, result.stdout))
    return output.read_bytes(), symbols(output.with_suffix('.sym'))


def patch_driver(original, assembler, work):
    header, code, bitmap = read_prl(original)
    if hashlib.sha256(header + code + bitmap).hexdigest() != DRIVER_SHA256:
        raise ImageError('Unsupported or already patched DDSCREEN.PRL')
    backend, labels = assemble(assembler, 'spectrum', work, 0)
    relocated, _ = assemble(assembler, 'spectrum', work, 256)
    if len(backend) != len(relocated):
        raise ImageError('Backend assembly sizes differ')
    relocations = {n for n in range(len(code)) if bitmap[n // 8] & (128 >> (n % 8))}
    for n, (a, b) in enumerate(zip(backend, relocated)):
        if a != b:
            if b != (a + 1) & 255:
                raise ImageError('Backend is not page relocatable')
            relocations.add(len(code) + n)
    code = bytearray(code + backend)

    def replace(address, size, payload, relative=()):
        offset = address - 256
        if offset < 0 or len(payload) > size or offset + size > len(code):
            raise ImageError('Invalid driver patch range')
        code[offset:offset + size] = payload.ljust(size, b'\0')
        relocations.difference_update(range(offset, offset + size))
        relocations.update(offset + n for n in relative)

    def branch(address, size, name, opcode=0xc3):
        replace(address, size, bytes((opcode,)) + labels[name].to_bytes(2, 'little'), (2,))

    replace(0x0102, 15, b'ZXTEENSY GSX B7')
    branch(0x0d20, 3, 'initialise')
    replace(0x0227, 8, b'')       # PCW-only CD VERSION == 1 check
    branch(0x022f, 10, 'begin_graphics', 0xcd)
    branch(0x0318, 11, 'close_graphics')
    branch(0x0274, 3, 'keyboard_initialise', 0xcd)
    branch(0x09de, 3, 'keyboard_poll')
    branch(0x0e45, 3, 'pixel_address')
    replace(0x0f8f, 2, labels['horizontal_span'].to_bytes(2, 'little'), (1,))
    replace(0x10f6, 2, labels['fill_screen'].to_bytes(2, 'little'), (1,))
    for address in (0x0dd5, 0x0e21, 0x0e38, 0x0eec, 0x0f91, 0x10b4, 0x10f8):
        if original[address:address + 5] != bytes.fromhex('cd880de900'):
            raise ImageError('Unexpected screen callback at %04X' % address)
        branch(address, 5, 'direct_gate', 0xcd)
    replace(0x10c4, 2, b'\0\x30')  # font cached in raster AllRAM D7 at open
    replace(0x02ac, 4, (1000).to_bytes(2, 'little') * 2)  # square pixels
    new_bitmap = bytearray((len(code) + 7) // 8)
    for n in relocations:
        new_bitmap[n // 8] |= 128 >> (n % 8)
    # The runtime copies the executable body into D7 once and undoes its page
    # relocation using this embedded bitmap. The bitmap itself is plain data.
    code.extend(new_bitmap)
    new_bitmap.extend(bytes((len(code) + 7) // 8 - len(new_bitmap)))
    header = bytearray(header)
    header[1:3] = len(code).to_bytes(2, 'little')
    result = bytes(header + code + new_bitmap)
    (work / 'DDSCREEN.PRL').write_bytes(result)
    return result


def relocate_prl(data, address):
    header, code, bitmap = read_prl(data)
    if address & 255 or address < 0x8000 or address + len(code) > 0xf400 or any(header[3:]):
        raise ImageError('Screen driver needs a page-aligned common address below the BDOS')
    result = bytearray(code)
    for n in range(len(result)):
        if bitmap[n // 8] & (128 >> (n % 8)):
            result[n] = (result[n] + (address >> 8) - 1) & 255
    return bytes(result)


def make_disk(driver, demo, work, runtime=None):
    sys.path.insert(0, str(ROOT / 'CPM'))
    from disk import Disk
    from make_floppy import create_image
    with tempfile.TemporaryDirectory(prefix='disk-', dir=str(work)) as temporary:
        blank = Path(temporary) / 'blank.dsk'
        create_image(blank, 'plus3')
        disk = Disk(blank.read_bytes())
    files = {'DDSCREEN.PRL': driver, 'ASSIGN.SYS': b'01 @:DDSCREEN\r\n\x1a',
             'GSXTEST.COM': demo,
             'README.TXT': (b'ZXTeensyIF1 GSX screen driver\r\n\r\n'
                            b'Boot your existing CP/M Plus disk, then mount this data disk as B:.\r\n'
                            b'Use B: followed by GSXTEST for a standalone driver demonstration.\r\n'
                            b'Press a key to return to the CP/M prompt.\r\n\r\n'
                            b'DDSCREEN.PRL draws 256x192 monochrome directly in bank 7.\r\n'
                            b'GSX applications need their own GDOS, or GSX.SYS/GENGRAF.COM.\r\n'
                            b'Keep ASSIGN.SYS and DDSCREEN.PRL on the current drive/user area.\r\n'
                            b'Keyboard locator: cursor controls move, other keys select.\r\n'
                            b'This disk is not bootable. Hardware testing is still required.\r\n\x1a')}
    if runtime is not None:
        candidates = CpmFiles(read_image(runtime)).files
        for name in ('GSX.SYS', 'GENGRAF.COM'):
            matches = [p for (_, n), p in candidates.items() if n == name]
            if not matches:
                raise ImageError('Runtime disk is missing ' + name)
            if any(p != matches[0] for p in matches):
                raise ImageError('Runtime disk has conflicting copies of ' + name)
            files[name] = matches[0]
    for name, payload in files.items():
        disk.put_file(name, payload)
    checked = CpmFiles(read_image(bytes(disk.data))).files
    for name, payload in files.items():
        if checked.get((0, name)) != payload.ljust((len(payload) + 127) // 128 * 128, b'\0'):
            raise ImageError('Output verification failed for ' + name)
    return bytes(disk.data)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--sjasmplus', default='sjasmplus')
    parser.add_argument('--runtime-disk', type=Path,
                        help='optional local disk containing GSX.SYS and GENGRAF.COM')
    parser.add_argument('--inspect', action='store_true', help='validate the driver without assembling or writing')
    args = parser.parse_args(argv)
    try:
        original = source_driver(args.input.read_bytes())
        print('Validated PCW8256 GIOS #8 from %s' % args.input)
        if args.inspect:
            return
        if args.output.resolve() == args.input.resolve() or (args.runtime_disk and args.output.resolve() == args.runtime_disk.resolve()):
            raise ImageError('Refusing to overwrite a source disk')
        if args.output.exists():
            raise ImageError('Output already exists; choose a new filename: ' + str(args.output))
        assembler = shutil.which(args.sjasmplus)
        if not assembler:
            raise ImageError('sjasmplus was not found; use --sjasmplus with its executable path')
        runtime = args.runtime_disk.read_bytes() if args.runtime_disk else None
        work_root = ROOT / 'build/CPMGSX'
        work_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='patch-', dir=str(work_root)) as temporary:
            work = Path(temporary)
            driver = patch_driver(original, assembler, work)
            (work / 'demo-driver.bin').write_bytes(relocate_prl(driver, 0xc000))
            demo, _ = assemble(assembler, 'demo', work)
            result = make_disk(driver, demo, work, runtime)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('xb') as output:
            output.write(result)
    except (OSError, ValueError) as error:
        parser.exit(1, 'error: %s\n' % error)
    print('Created %s: DDSCREEN.PRL, ASSIGN.SYS, GSXTEST.COM, README.TXT%s' %
          (args.output, ', GSX.SYS, GENGRAF.COM' if runtime else ''))
    print('256x192 monochrome, direct Spectrum bank 7; boot your existing CP/M disk first.')
    if not runtime:
        print('GSXTEST runs directly; other GSX applications must supply their GDOS/runtime.')


if __name__ == '__main__':
    main()

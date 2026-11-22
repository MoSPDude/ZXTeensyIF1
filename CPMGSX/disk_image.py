"""Read ordinary Amstrad CP/M files and write standard CPC DSK containers."""
import struct

from teledisk import ImageError, read_sectors


def read_dsk(data):
    extended = data.startswith(b'EXTENDED CPC DSK File\r\nDisk-Info\r\n')
    if len(data) < 256 or not (extended or data.startswith(b'MV - CPCEMU Disk-File\r\nDisk-Info\r\n')):
        raise ImageError('Not a standard or extended CPC DSK')
    cylinders, heads = data[48:50]
    if not cylinders or heads not in (1, 2) or (extended and cylinders * heads > 204):
        raise ImageError('Unsupported DSK geometry')
    fixed = struct.unpack_from('<H', data, 50)[0]
    offset = 256
    sectors = {}
    for index in range(cylinders * heads):
        length = data[52 + index] * 256 if extended else fixed
        if not length:
            continue
        if length < 256 or offset + length > len(data):
            raise ImageError('Truncated DSK track')
        track = data[offset:offset + 256]
        cylinder, head = divmod(index, heads)
        if track[:12] != b'Track-Info\r\n' or tuple(track[16:18]) != (cylinder, head):
            raise ImageError('Invalid DSK track header')
        count = track[21]
        if not 1 <= count <= 29:
            raise ImageError('Unsupported DSK sector count')
        position = offset + 256
        for n in range(count):
            c, h, r, shift, st1, st2, stored = struct.unpack_from('<6BH', track, 24 + n * 8)
            size = stored if extended else (128 << shift if shift <= 6 else 0)
            if (c, h, shift, size, st1, st2) != (cylinder, head, 2, 512, 0, 0):
                raise ImageError('Unsupported or damaged DSK sector')
            key = c, h, r
            if key in sectors or position + size > offset + length:
                raise ImageError('Duplicate or truncated DSK sector')
            sectors[key] = bytes(data[position:position + size])
            position += size
        offset += length
    if offset != len(data):
        raise ImageError('Unexpected bytes after DSK tracks')
    return sectors


def read_image(data):
    return read_sectors(data) if data[:2] in (b'TD', b'td') else read_dsk(data)


def standard_dsk(sectors):
    """Normalise sector order; retain every sector byte, including the boot."""
    if not sectors:
        raise ImageError('No sectors in source image')
    cylinders = max(c for c, _, _ in sectors) + 1
    heads = max(h for _, h, _ in sectors) + 1
    ids = sorted(r for c, h, r in sectors if (c, h) == (0, 0))
    if not 1 <= cylinders <= 100 or heads not in (1, 2) or not 1 <= len(ids) <= 29:
        raise ImageError('Unsupported standard DSK geometry')
    expected = {(c, h, r) for c in range(cylinders) for h in range(heads) for r in ids}
    if set(sectors) != expected or any(len(s) != 512 for s in sectors.values()):
        raise ImageError('Standard DSK conversion needs complete, uniform 512-byte sectors')
    header = bytearray(256)
    header[:34] = b'MV - CPCEMU Disk-File\r\nDisk-Info\r\n'
    header[34:48] = b'ZXTeensyIF1'.ljust(14)
    header[48:50] = bytes((cylinders, heads))
    struct.pack_into('<H', header, 50, 256 + len(ids) * 512)
    result = bytearray(header)
    for cylinder in range(cylinders):
        for head in range(heads):
            track = bytearray(256)
            track[:12] = b'Track-Info\r\n'
            track[16:18] = bytes((cylinder, head))
            track[20:24] = bytes((2, len(ids), 0x52, 0xe5))
            for n, sector in enumerate(ids):
                struct.pack_into('<6BH', track, 24 + n * 8, cylinder, head, sector, 2, 0, 0, 0)
            result.extend(track)
            for sector in ids:
                result.extend(sectors[cylinder, head, sector])
    return bytes(result)


class CpmFiles:
    """Read 180K/720K PCW/+3 and CPC system/data files in all user areas."""
    def __init__(self, sectors):
        ids = sorted(r for c, h, r in sectors if (c, h) == (0, 0))
        if ids == list(range(1, 10)):
            spec = sectors[0, 0, 1][:16]
            if spec[0] not in (0, 3):
                raise ImageError('Unrecognised Amstrad CP/M disk specification')
        elif ids in (list(range(65, 74)), list(range(193, 202))):
            spec = bytes((1, 0, 40, 9, 2, 2, 3, 2) if ids[0] == 65 else
                         (2, 0, 40, 9, 2, 0, 3, 2))
        else:
            raise ImageError('Unrecognised Amstrad CP/M sector IDs')
        mode = spec[1] & 127
        cylinders = spec[2]
        heads = 1 if mode == 0 else 2
        if mode > 2 or not 1 <= cylinders <= 80 or spec[3:5] != bytes((9, 2)):
            raise ImageError('Unsupported CP/M disk geometry')
        order = [(c, h) for c in range(cylinders) for h in range(heads)]
        if mode == 2:
            order = [(c, 0) for c in range(cylinders)] + [(c, 1) for c in reversed(range(cylinders))]
        keys = [(c, h, r) for c, h in order for r in ids]
        if set(keys) != set(sectors) or any(len(s) != 512 for s in sectors.values()):
            raise ImageError('Missing sectors or geometry differs from CP/M specification')
        if not 3 <= spec[6] <= 7 or not 1 <= spec[7] <= 16 or spec[5] >= cylinders * heads:
            raise ImageError('Invalid CP/M allocation geometry')
        self.cylinders, self.heads = cylinders, heads
        self.block_size = 128 << spec[6]
        self.directory_blocks = spec[7]
        self.offset = spec[5] * 9 * 512
        self.data = b''.join(sectors[k] for k in keys)
        self.blocks = (len(self.data) - self.offset) // self.block_size
        if self.blocks <= self.directory_blocks or self.blocks > 65536:
            raise ImageError('Invalid CP/M allocation block count')
        self.pointer_size = 1 if self.blocks <= 256 else 2
        self.extent_mask = (self.block_size // (1024 * self.pointer_size)) - 1
        self.files = {}
        occupied = set(range(self.directory_blocks))
        end = self.offset + self.directory_blocks * self.block_size
        for pos in range(self.offset, end, 32):
            entry = self.data[pos:pos + 32]
            if entry[0] >= 16:
                continue
            name = bytes(c & 127 for c in entry[1:12])
            if any(c < 32 or c > 126 for c in name) or not name[:8].strip():
                raise ImageError('Invalid CP/M directory filename')
            stem, suffix = name[:8].decode('ascii').rstrip(), name[8:].decode('ascii').rstrip()
            name = stem + ('.' + suffix if suffix else '')
            extent = (entry[12] & 31) + 32 * entry[14]
            records = (extent & self.extent_mask) * 128 + entry[15]
            if entry[15] > 128:
                raise ImageError('Invalid CP/M record count')
            pointers = [int.from_bytes(entry[n:n + self.pointer_size], 'little')
                        for n in range(16, 32, self.pointer_size)]
            needed = (records * 128 + self.block_size - 1) // self.block_size
            if any(not b for b in pointers[:needed]):
                raise ImageError('Missing CP/M file allocation')
            for block in pointers:
                if block:
                    if block >= self.blocks or block in occupied:
                        raise ImageError('Invalid or cross-linked CP/M file allocation')
                    occupied.add(block)
            payload = b''.join(self.data[self.offset + b * self.block_size:
                                         self.offset + (b + 1) * self.block_size]
                               for b in pointers[:needed])[:records * 128]
            key = entry[0], name
            self.files.setdefault(key, []).append((extent & ~self.extent_mask, payload))
        capacity = 16 // self.pointer_size * self.block_size
        for key, extents in self.files.items():
            extents.sort()
            for n, (extent, payload) in enumerate(extents):
                if extent != n * (self.extent_mask + 1) or (n < len(extents) - 1 and len(payload) != capacity):
                    raise ImageError('Missing, duplicate or incomplete CP/M extent: ' + key[1])
        self.files = {key: b''.join(p for _, p in extents) for key, extents in self.files.items()}

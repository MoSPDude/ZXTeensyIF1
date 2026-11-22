"""Read Teledisk 2.x sectors, including its adaptive-Huffman/LZSS stream.

Format references: Sydex Teledisk documentation and Will Kranz's TD0 notes
(https://github.com/jmechnich/wteledsk/blob/master/doc/wteledsk.htm).
No converter or third-party Python package is needed.
"""
import struct


class ImageError(ValueError):
    pass


def crc16(data):
    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0xa097 if crc & 0x8000 else 0)) & 0xffff
    return crc


class LzhReader:
    """Teledisk's 4096-byte window, 60-byte matches and 314-symbol tree."""
    def __init__(self, data):
        self.data = data
        self.bit = 0
        self.freq = [1] * 314 + [0] * 313 + [65535]
        self.child = [627 + n for n in range(314)] + [0] * 313
        self.parent = [0] * 941
        for n in range(314):
            self.parent[627 + n] = n
        for n in range(314, 627):
            first = (n - 314) * 2
            self.child[n] = first
            self.freq[n] = self.freq[first] + self.freq[first + 1]
            self.parent[first] = self.parent[first + 1] = n
        self.window = bytearray(b' ' * 4096)
        self.position = 4096 - 60
        self.pending = bytearray()
        # Canonical prefix codes for the high six bits of a match distance.
        self.positions = {}
        code = 0
        previous = 3
        for value, length in enumerate([3] + [4] * 3 + [5] * 8 +
                                       [6] * 12 + [7] * 24 + [8] * 16):
            code <<= length - previous
            self.positions[length, code] = value
            code += 1
            previous = length

    def bits(self, count):
        value = 0
        for _ in range(count):
            if self.bit >= len(self.data) * 8:
                raise ImageError('Truncated Teledisk compressed stream')
            value = (value << 1) | ((self.data[self.bit // 8] >> (7 - self.bit % 8)) & 1)
            self.bit += 1
        return value

    def rebuild(self):
        leaves = [((self.freq[n] + 1) >> 1, self.child[n])
                  for n in range(627) if self.child[n] >= 627]
        self.freq[:314] = [f for f, _ in leaves]
        self.child[:314] = [c for _, c in leaves]
        for n in range(314, 627):
            first = (n - 314) * 2
            frequency = self.freq[first] + self.freq[first + 1]
            target = n
            while target and self.freq[target - 1] > frequency:
                target -= 1
            self.freq[target + 1:n + 1] = self.freq[target:n]
            self.child[target + 1:n + 1] = self.child[target:n]
            self.freq[target], self.child[target] = frequency, first
        for n, child in enumerate(self.child):
            self.parent[child] = n
            if child < 627:
                self.parent[child + 1] = n

    def symbol(self):
        node = self.child[626]
        while node < 627:
            node = self.child[node + self.bits(1)]
        value = node - 627
        if self.freq[626] == 32768:
            self.rebuild()
        node = self.parent[value + 627]
        while True:
            self.freq[node] += 1
            other = node
            while self.freq[other + 1] < self.freq[node]:
                other += 1
            if other != node:
                self.freq[node], self.freq[other] = self.freq[other], self.freq[node]
                a, b = self.child[node], self.child[other]
                self.parent[a], self.parent[b] = other, node
                if a < 627:
                    self.parent[a + 1] = other
                if b < 627:
                    self.parent[b + 1] = node
                self.child[node], self.child[other] = b, a
                node = other
            node = self.parent[node]
            if not node:
                return value

    def read(self, count):
        while len(self.pending) < count:
            symbol = self.symbol()
            if symbol < 256:
                chunk = bytes((symbol,))
            else:
                code = 0
                for length in range(1, 9):
                    code = (code << 1) | self.bits(1)
                    if (length, code) in self.positions:
                        break
                else:
                    raise ImageError('Invalid Teledisk match distance')
                distance = (self.positions[length, code] << 6) | self.bits(6)
                start = (self.position - distance - 1) & 4095
                chunk = bytearray()
                for n in range(symbol - 253):
                    value = self.window[(start + n) & 4095]
                    chunk.append(value)
                    self.window[self.position] = value
                    self.position = (self.position + 1) & 4095
                self.pending.extend(chunk)
                continue
            for value in chunk:
                self.window[self.position] = value
                self.position = (self.position + 1) & 4095
            self.pending.extend(chunk)
        result = self.pending[:count]
        del self.pending[:count]
        return bytes(result)


class RawReader:
    def __init__(self, data):
        self.data, self.position = data, 0

    def read(self, count):
        result = self.data[self.position:self.position + count]
        self.position += count
        if len(result) != count:
            raise ImageError('Truncated Teledisk record')
        return result


def expand_sector(data, size):
    if not data:
        raise ImageError('Empty Teledisk sector record')
    method, data = data[0], data[1:]
    if method == 0:
        result = data
    elif method == 1:
        if len(data) != 4:
            raise ImageError('Invalid Teledisk repeated-word sector')
        result = data[2:] * struct.unpack_from('<H', data)[0]
    elif method == 2:
        result = bytearray()
        position = 0
        while position < len(data):
            if position + 2 > len(data):
                raise ImageError('Truncated Teledisk RLE sector')
            length, count = data[position:position + 2]
            position += 2
            width = (1 << length) if length else count
            if position + width > len(data):
                raise ImageError('Truncated Teledisk RLE pattern')
            result.extend(data[position:position + width] * (count if length else 1))
            position += width
            if len(result) > size:
                raise ImageError('Oversized Teledisk sector')
    else:
        raise ImageError('Unknown Teledisk sector compression')
    if len(result) != size:
        raise ImageError('Incorrect expanded Teledisk sector length')
    return bytes(result)


def read_sectors(data):
    """Return {(cylinder, head, sector): bytes}, checking every stored CRC."""
    if len(data) < 12 or data[:2] not in (b'TD', b'td'):
        raise ImageError('Not a Teledisk image')
    if crc16(data[:10]) != struct.unpack_from('<H', data, 10)[0]:
        raise ImageError('Teledisk header CRC mismatch')
    if data[2] or data[8] or data[9] not in (1, 2):
        raise ImageError('Unsupported split/allocated-only Teledisk image')
    if data[:2] == b'td' and data[4] < 20:
        raise ImageError('Old Teledisk advanced compression is unsupported')
    reader = LzhReader(data[12:]) if data[:2] == b'td' else RawReader(data[12:])
    if data[7] & 128:
        comment = reader.read(10)
        comment += reader.read(struct.unpack_from('<H', comment, 2)[0])
        if crc16(comment[2:]) != struct.unpack_from('<H', comment)[0]:
            raise ImageError('Teledisk comment CRC mismatch')
    sectors = {}
    tracks = set()
    while True:
        first = reader.read(1)[0]
        if first == 255:
            break
        track = bytes((first,)) + reader.read(3)
        count, cylinder, head, crc = track
        if crc16(track[:3]) & 255 != crc or (cylinder, head) in tracks:
            raise ImageError('Invalid or duplicate Teledisk track')
        if count > 64 or cylinder >= 100 or head >= data[9]:
            raise ImageError('Unsupported Teledisk track geometry: %r' % (tuple(track),))
        tracks.add((cylinder, head))
        for _ in range(count):
            c, h, r, n, flags, crc = reader.read(6)
            if (c, h) != (cylinder, head) or n > 6 or flags & ~1:
                raise ImageError('Unsupported/missing/damaged Teledisk sector')
            length = struct.unpack('<H', reader.read(2))[0]
            sector = expand_sector(reader.read(length), 128 << n)
            if crc16(sector) & 255 != crc:
                raise ImageError('Teledisk sector CRC mismatch')
            if (c, h, r) in sectors:
                raise ImageError('Duplicate Teledisk sector ID')
            sectors[c, h, r] = sector
    return sectors

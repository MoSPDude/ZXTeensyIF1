"""Functional CPC DSK/FDC media model used by CPU integration checks."""
import struct


class Floppy:
    def __init__(self, data, read_only=False):
        self.data = data if isinstance(data, bytearray) else bytearray(data)
        self.read_only = read_only
        self.parse()

    def parse(self):
        data = self.data
        if len(data) < 256 or not (data.startswith(b'MV - CPC') or data.startswith(b'EXTENDED')):
            raise ValueError('Invalid DSK container')
        self.extended = data.startswith(b'EXTENDED')
        self.cylinders, self.heads = data[48:50]
        self.sectors = {}
        self.tracks = {}
        offset = 256
        fixed = int.from_bytes(data[50:52], 'little')
        for index in range(self.cylinders * self.heads):
            size = data[52 + index] * 256 if self.extended else fixed
            if not size:
                continue
            if size < 256 or offset + size > len(data) or data[offset:offset + 12] != b'Track-Info\r\n':
                raise ValueError('Invalid DSK track')
            cylinder, head = divmod(index, self.heads)
            position = offset + 256
            ids = []
            for n in range(data[offset + 21]):
                c, h, r, shift, st1, st2, stored = struct.unpack_from('<6BH', data, offset + 24 + n * 8)
                length = stored if self.extended else 128 << shift
                if length < 128 or position + length > offset + size:
                    raise ValueError('Invalid DSK sector')
                record = (position, length, c, h, shift, st1, st2, offset + 24 + n * 8)
                self.sectors[cylinder, head, r] = record
                ids.append(r)
                position += length
            self.tracks[cylinder, head] = ids
            offset += size

    def sector(self, cylinder, head, sector):
        record = self.sectors.get((cylinder, head, sector))
        if record is None:
            return None
        position, size, c, h, shift, st1, st2, _ = record
        return bytes(self.data[position:position + size]), (c, h, sector, shift), st1, st2

    def write_sector(self, cylinder, head, sector, payload):
        record = self.sectors[cylinder, head, sector]
        position, size = record[:2]
        if self.read_only or len(payload) != size:
            raise ValueError('Invalid write')
        self.data[position:position + size] = payload

    def format_track(self, cylinder, head, ids, filler):
        old = list(self.tracks.get((cylinder, head), []))
        if self.read_only or len(ids) != len(old):
            raise ValueError('Format does not fit existing track')
        records = [self.sectors[cylinder, head, r] for r in old]
        for record, identifier in zip(records, ids):
            c, h, r, shift = identifier
            position, size = record[:2]
            if size != 128 << shift:
                raise ValueError('Format changes sector allocation')
            offset = record[-1]
            self.data[offset:offset + 6] = bytes((c, h, r, shift, 0, 0))
            self.data[position:position + size] = bytes((filler,)) * size
        self.parse()

    def linear_sector(self, cylinder, head, sector):
        per_track = len(self.tracks.get((cylinder, head), []))
        ids = sorted(self.tracks.get((cylinder, head), []))
        return (cylinder * self.heads + head) * per_track + ids.index(sector)


def as_floppy(disk):
    if disk is None or isinstance(disk, Floppy):
        return disk
    return Floppy(disk.data)

"""Read and update standard +3 CP/M files inside CPC DSK containers."""

import struct


class DiskError(ValueError):
    pass


class Disk:
    """A 40-track, single-sided +3 disk; retain its original container bytes."""

    def __init__(self, data):
        self.data = bytearray(data)
        if len(data) < 256:
            raise DiskError("Truncated DSK header")
        extended = data.startswith(b"EXTENDED CPC DSK File\r\nDisk-Info\r\n")
        if not extended and not data.startswith(b"MV - CPC"):
            raise DiskError("Not a standard or extended CPC DSK")
        tracks, sides = data[48:50]
        # Extended containers may declare unused, unformatted trailing tracks.
        # The filesystem still requires exactly 40 formatted tracks.
        if (sides != 1 or not 40 <= tracks <= 256 - 52 or
                (tracks > 40 and (not extended or any(data[92:52 + tracks])))):
            raise DiskError("Expected a 40-track, single-sided +3 disk")
        fixed_size = struct.unpack_from("<H", data, 50)[0]
        self.sectors = []
        offset = 256
        for track in range(40):
            size = data[52 + track] * 256 if extended else fixed_size
            if size < 256 or offset + size > len(data):
                raise DiskError("Missing or truncated track %d" % track)
            header = data[offset:offset + 256]
            if not header.startswith(b"Track-Info\r\n"):
                raise DiskError("Invalid track header at track %d" % track)
            if tuple(header[16:18]) != (track, 0) or header[21] != 9:
                raise DiskError("Unexpected track numbering or sector count")
            positions = {}
            position = offset + 256
            for n in range(9):
                c, h, r, length, st1, st2, actual = struct.unpack_from("<6BH", header, 24 + n * 8)
                stored = actual if extended else 128 << length
                if (c, h, length, stored, st1, st2) != (track, 0, 2, 512, 0, 0):
                    raise DiskError("Unsupported sector geometry or error flags")
                if r in positions or position + stored > offset + size:
                    raise DiskError("Duplicate sector or truncated track data")
                positions[r] = position
                position += stored
            if set(positions) != set(range(1, 10)):
                raise DiskError("Expected physical sectors 1 through 9")
            self.sectors.extend(positions[r] for r in range(1, 10))
            offset += size
        specification = self.read(0, 16)
        if specification[:8] != bytes((0, 0, 40, 9, 2, 1, 3, 2)):
            raise DiskError("Unsupported +3 disk specification")
        allocated = set()
        for _, entry in self.directory():
            if entry[15] > 128:
                raise DiskError("Invalid record count in the directory")
            count = (entry[15] + 7) // 8
            if any(not b for b in entry[16:16 + count]):
                raise DiskError("Missing allocation block in the directory")
            for block in (b for b in entry[16:] if b):
                if not 2 <= block <= 174 or block in allocated:
                    raise DiskError("Invalid or cross-linked allocation block")
                allocated.add(block)

    def read(self, offset, size):
        if offset < 0 or size < 0 or offset + size > len(self.sectors) * 512:
            raise DiskError("Disk read outside the image")
        result = bytearray()
        while size:
            sector, within = divmod(offset, 512)
            count = min(size, 512 - within)
            position = self.sectors[sector] + within
            result.extend(self.data[position:position + count])
            offset += count
            size -= count
        return bytes(result)

    def write(self, offset, data):
        if offset < 0 or offset + len(data) > len(self.sectors) * 512:
            raise DiskError("Disk write outside the image")
        position = 0
        while position < len(data):
            sector, within = divmod(offset, 512)
            count = min(len(data) - position, 512 - within)
            target = self.sectors[sector] + within
            self.data[target:target + count] = data[position:position + count]
            offset += count
            position += count

    def directory(self):
        for index in range(64):
            entry = self.read(4608 + index * 32, 32)
            if entry[0] < 16:
                yield index, entry

    def entries(self, name):
        stem, dot, suffix = name.upper().partition(".")
        if not dot or not 1 <= len(stem) <= 8 or not 1 <= len(suffix) <= 3:
            raise DiskError("Expected an 8.3 CP/M filename")
        wanted = (stem.ljust(8) + suffix.ljust(3)).encode("ascii")
        entries = [(n, e) for n, e in self.directory()
                   if e[0] == 0 and bytes(c & 127 for c in e[1:12]) == wanted]
        entries.sort(key=lambda item: (item[1][12] & 31) + 32 * item[1][14])
        for extent, (_, entry) in enumerate(entries):
            if (entry[12] & 31) + 32 * entry[14] != extent or entry[15] > 128:
                raise DiskError("Missing, duplicated or invalid file extent")
        return entries

    def read_file(self, name):
        entries = self.entries(name)
        if not entries:
            raise DiskError("File not found: " + name)
        result = bytearray()
        allocated = set()
        for _, entry in entries:
            length = entry[15] * 128
            count = (length + 1023) // 1024
            for block in entry[16:16 + count]:
                if not 2 <= block <= 174 or block in allocated:
                    raise DiskError("Invalid or reused allocation block")
                allocated.add(block)
                result.extend(self.read(4608 + block * 1024, min(1024, length)))
                length -= min(1024, length)
        return bytes(result)

    def replace_file(self, name, data):
        entries = self.entries(name)
        if not entries:
            raise DiskError("File not found: " + name)
        capacity = sum(sum(b != 0 for b in e[16:]) * 1024 for _, e in entries)
        if not data or len(data) > capacity:
            raise DiskError("Replacement exceeds the existing file allocation")
        remaining = bytes(data)
        for index, old in entries:
            entry = bytearray(old)
            blocks = [b for b in old[16:] if b]
            length = min(len(remaining), len(blocks) * 1024)
            entry[15] = (length + 127) // 128
            if not length:
                entry[0] = 0xe5
            for block in blocks:
                chunk = remaining[:1024]
                remaining = remaining[len(chunk):]
                self.write(4608 + block * 1024, chunk.ljust(1024, b"\0"))
            # Keep the allocation list for the final extent: the boot loader reads it.
            self.write(4608 + index * 32, entry)
        if remaining:
            raise DiskError("Replacement did not fit")

    def rename_file(self, old_name, new_name):
        if self.entries(new_name):
            raise DiskError("Destination already exists: " + new_name)
        stem, _, suffix = new_name.partition(".")
        encoded = (stem.ljust(8) + suffix.ljust(3)).encode("ascii")
        for index, old in self.entries(old_name):
            entry = bytearray(old)
            entry[1:12] = encoded
            self.write(4608 + index * 32, entry)

    def put_file(self, name, data):
        """Allocate a file, releasing surplus old blocks without touching others."""
        old = self.entries(name)
        own = {index for index, _ in old}
        occupied = set()
        for index, entry in self.directory():
            if index not in own:
                for block in entry[16:]:
                    if block:
                        if not 2 <= block <= 174 or block in occupied:
                            raise DiskError("Invalid or cross-linked disk allocation")
                        occupied.add(block)
        blocks = [b for _, entry in old for b in entry[16:] if b]
        blocks += [b for b in range(2, 175) if b not in occupied and b not in blocks]
        count = (len(data) + 1023) // 1024
        slots = sorted(own) + [n for n in range(64) if n not in own and
                                  self.read(4608 + n * 32, 1)[0] == 0xe5]
        extents = max(1, (len(data) + 16383) // 16384)
        if not data or count > len(blocks) or extents > len(slots):
            raise DiskError("Insufficient free disk space or directory entries")
        stem, _, suffix = name.upper().partition(".")
        encoded = (stem.ljust(8) + suffix.ljust(3)).encode("ascii")
        for index in own:
            self.write(4608 + index * 32, b"\xe5" * 32)
        for i, block in enumerate(blocks[:count]):
            self.write(4608 + block * 1024, data[i * 1024:(i + 1) * 1024].ljust(1024, b"\0"))
        for extent in range(extents):
            entry = bytearray(32)
            entry[1:12] = encoded
            entry[12], entry[14] = extent & 31, extent >> 5
            entry[15] = (min(16384, len(data) - extent * 16384) + 127) // 128
            used = blocks[extent * 16:min(count, (extent + 1) * 16)]
            entry[16:16 + len(used)] = bytes(used)
            self.write(4608 + slots[extent] * 32, entry)

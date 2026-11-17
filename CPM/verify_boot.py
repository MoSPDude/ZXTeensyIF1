"""Execute a locally patched DSK with a Z80, 128K RAM and DivMMC AllRAM model.

Development check, not a substitute for testing the Teensy bus/FDC on hardware.
Requires SkoolKit 8.10 (test dependency only). No ROM or OS bytes are bundled.
"""

import argparse
from collections import deque
import importlib.util
from pathlib import Path
import sys
import types

from disk import Disk
from floppy_model import as_floppy


def simulator_class(dependency_path=None):
    if dependency_path:
        sys.path.insert(0, str(dependency_path))
    import skoolkit
    if skoolkit.VERSION != "8.10":
        raise ValueError("This harness requires SkoolKit 8.10")
    spec = importlib.util.find_spec("skoolkit.simulator")
    source = Path(spec.origin).read_text()
    # SkoolKit's Spectrum CPU otherwise suppresses every write below 4000.
    # This model supplies external writable RAM there, so remove that fixed
    # ROM assumption from the CPU; actual bank mapping remains in Machine.
    if source.count("> 0x3FFF") != 32:
        raise ValueError("Unexpected SkoolKit RAM write guards")
    module = types.ModuleType("allram_simulator")
    exec(compile(source.replace("> 0x3FFF", ">= 0"), spec.origin, "exec"), module.__dict__)
    # HALT must wake on the same 128K frame boundary as our interrupt source.
    module.FRAME_DURATION = 70908
    return module.Simulator


class Machine:
    def __init__(self, disk, stale_low_page=True, storage=None, drive_b=None, drive_b_fitted=False):
        self.storage = storage
        self.disk = disk
        self.disks = [as_floppy(disk), as_floppy(drive_b)]
        self.fitted = [True, drive_b is not None or drive_b_fitted]
        self.drive_tracks = [0, 0]
        self.write_protected = [False, False]
        self.motor_on = False
        self.fdc_operations = []
        self.fdc_failures = []
        self.transfer_result = []
        self.write_command = None
        self.memory = [0] * 65536
        self.ram = [[0] * 16384 for _ in range(8)]
        self.divram = [[0] * 16384 for _ in range(8)]
        self.mapping = [("rom", 0, 0), ("rom", 0, 8192),
                        ("ram", 5, 0), ("ram", 5, 8192),
                        ("ram", 2, 0), ("ram", 2, 8192),
                        ("ram", 3, 0), ("ram", 3, 8192)]
        self.stale_low_page = stale_low_page
        self.lower_page = 0
        self.port7 = 3
        self.port1 = 0
        self.e3 = 0
        self.allram = 0
        self.command = []
        self.results = deque()
        self.transfer = deque()
        self.write_data = None
        self.write_target = None
        self.track = 0
        self.interrupt = False
        self.reads = []
        self.writes = []
        self.ports = []
        self.keyboard = None
        self.keyboard_start = 0

    def allram_page(self):
        return ((self.e3 >> 1) & 4) | ((self.port1 >> 1) & 2) | ((self.port7 >> 4) & 1)

    def update_mapping(self, refresh_lower=True):
        for slot, (kind, bank, offset) in enumerate(self.mapping):
            if kind != "rom":
                backing = self.ram if kind == "ram" else self.divram
                backing[bank][offset:offset + 8192] = self.memory[slot * 8192:(slot + 1) * 8192]
        page = self.allram_page()
        if refresh_lower:
            self.lower_page = page
        self.mapping = [("div", self.lower_page, 0), ("div", page, 8192)] if self.allram & 128 else [
            ("rom", 0, 0), ("rom", 0, 8192)]
        for bank in (5, 2, self.port7 & 7):
            self.mapping.extend((("ram", bank, 0), ("ram", bank, 8192)))
        for slot, (kind, bank, offset) in enumerate(self.mapping):
            if kind == "rom":
                self.memory[slot * 8192:(slot + 1) * 8192] = [0xff] * 8192
            else:
                backing = self.ram if kind == "ram" else self.divram
                self.memory[slot * 8192:(slot + 1) * 8192] = backing[bank][offset:offset + 8192]

    def write_port(self, registers, port, value):
        self.ports.append((registers[24], port, value))
        if port & 255 == 0xe7:
            if self.storage:
                self.storage.select(value)
            return
        if port & 255 == 0xeb:
            if self.storage:
                self.storage.write(value)
            return
        if port == 0x7ffd:
            if value & 0x20:
                raise ValueError("Firmware attempted to lock Spectrum paging")
            # Current firmware refreshes romPtr before changing bankRom01,
            # then updates divMmcRamPtr but not romPtr. Thus E3 must be written
            # after 7FFD to refresh the lower 8K to the new AllRAM page too.
            self.lower_page = self.allram_page()
            self.port7 = value
        elif port == 0x1ffd:
            self.port1 = value     # never changes the physical middle RAM slots
            self.motor_on = bool(value & 8)
        elif port == 0x0f3b:
            self.allram = value
        elif port & 255 == 0xe3:
            self.e3 = value
        elif port & 0xf002 == 0x3000:
            self.fdc_write(value)
            return
        else:
            return
        self.update_mapping(not (port == 0x7ffd and self.stale_low_page))

    def read_port(self, registers, port):
        if port & 255 == 0xeb:
            return self.storage.read() if self.storage else 255
        if port & 0xf002 == 0x2000:
            if self.transfer:
                return 0xf0
            if self.write_data is not None:
                return 0xb0
            return 0xd0 if self.results else 0x80
        if port & 0xf002 == 0x3000:
            if self.transfer:
                value = self.transfer.popleft()
                if not self.transfer:
                    self.finish_transfer()
                return value
            return self.results.popleft() if self.results else 0xff
        if port & 255 == 0xfe and self.keyboard is not None:
            # A complete press/release lasts longer than a matrix scan.
            if registers[25] - self.keyboard_start >= 100000:
                self.keyboard = None
                return 0xff
            value = 0xff
            for row, bit in self.keyboard:
                if not (port >> 8) & (1 << row):
                    value &= 0xff ^ (1 << bit)
            return value
        return 0xff

    def mount(self, unit, disk):
        """Replace/eject media without resetting the running CPU or controller."""
        self.disks[unit] = as_floppy(disk)
        if unit == 0 and disk is not None:
            self.disk = disk

    def finish_transfer(self):
        self.results.extend(self.transfer_result)

    def fdc_write(self, value):
        if self.write_data is not None:
            self.write_data.append(value)
            if len(self.write_data) == self.write_length:
                unit, cylinder, head, sector = self.write_target
                disk = self.disks[unit]
                if self.write_command == 13:
                    ids = [tuple(self.write_data[i:i + 4]) for i in range(0, len(self.write_data), 4)]
                    disk.format_track(cylinder, head, ids, self.format_filler)
                else:
                    disk.write_sector(cylinder, head, sector, self.write_data)
                    self.writes.append(disk.linear_sector(cylinder, head, sector))
                self.write_data = None
                self.finish_transfer()
            return
        self.command.append(value)
        opcode = self.command[0] & 31
        lengths = {3: 3, 4: 2, 5: 9, 6: 9, 7: 2, 8: 1, 10: 2, 13: 6, 15: 3}
        if opcode not in lengths:
            raise ValueError("Unsupported FDC command %02X" % self.command[0])
        if len(self.command) != lengths[opcode]:
            return
        command, self.command = self.command, []
        unit = command[1] & 1 if len(command) > 1 else 0
        head = (command[1] >> 2) & 1 if len(command) > 1 else 0
        disk = self.disks[unit]
        ready = self.fitted[unit] and disk is not None and self.motor_on
        protected = self.write_protected[unit] or disk is not None and disk.read_only
        if opcode == 4:
            status = unit | (head << 2)
            if self.fitted[unit]:
                status |= (32 if ready else 64)
                status |= 16 if self.drive_tracks[unit] == 0 else 0
                status |= 64 if protected else 0
                status |= 8 if disk is not None and disk.heads > 1 else 0
            self.results.append(status)
            return
        if opcode in (7, 15):
            self.track = 0 if opcode == 7 else command[2]
            self.drive_tracks[unit] = self.track
            self.seek_unit = unit
            self.interrupt = True
            return
        if opcode == 8:
            self.results.extend((0x20 | getattr(self, 'seek_unit', 0), self.track) if self.interrupt else (0x80,))
            self.interrupt = False
            return
        if opcode == 3:
            return
        cylinder = self.drive_tracks[unit]
        sector, shift = (command[4], command[5]) if opcode in (5, 6) else (1, 2)
        self.chrn = (cylinder, head, sector, shift)
        self.fdc_operations.append((opcode, unit, cylinder, head, sector))
        if not ready:
            self.results.extend((0x48 | unit, 0, 0, *self.chrn))
            return
        if opcode == 10:
            ids = disk.tracks.get((cylinder, head), [])
            if ids:
                sector = ids[0]
                data, chrn, st1, st2 = disk.sector(cylinder, head, sector)
                self.results.extend((unit | (head << 2), st1, st2, *chrn))
            else:
                self.results.extend((0x40 | unit, 1, 0, *self.chrn))
            return
        record = disk.sector(cylinder, head, sector) if opcode != 13 else None
        if opcode != 13 and record is None:
            self.results.extend((0x40 | unit, 4, 0, *self.chrn))
            return
        if opcode in (5, 13) and protected:
            self.results.extend((0x40 | unit, 2, 0, *self.chrn))
            return
        st0 = unit | (head << 2)
        st1, st2 = record[2:] if record else (0, 0)
        if record and (record[1][0], record[1][1], record[1][3]) != (command[2], command[3], shift):
            st0, st1, st2 = 0x40 | unit, 4, 0x10
        for failure in self.fdc_failures:
            if failure.get('count', 1) and all(failure.get(k, v) == v for k, v in (
                    ('opcode', opcode), ('unit', unit), ('cylinder', cylinder), ('head', head), ('sector', sector))):
                failure['count'] = failure.get('count', 1) - 1
                st0, st1, st2 = 0x40 | unit, failure.get('st1', 0x20), failure.get('st2', 0)
                break
        if st1 or st2:
            st0 |= 0x40
        self.transfer_result = (st0, st1, st2, *self.chrn)
        self.write_target = (unit, cylinder, head, sector)
        if opcode == 6:
            self.reads.append(disk.linear_sector(cylinder, head, sector))
            self.transfer.extend(record[0])
        elif opcode in (5, 13):
            self.write_command = opcode
            self.write_length = command[3] * 4 if opcode == 13 else len(record[0])
            self.format_filler = command[5] if opcode == 13 else 0
            self.write_data = bytearray()


def symbols(path):
    result = {}
    for line in path.read_text().splitlines():
        name, _, value = line.partition(": EQU ")
        if value:
            result[name] = int(value.strip(), 16)
    return result


def key_matrix(character):
    """Translate a character or named editing key to physical matrix switches."""
    keys = "\0zxcvasdfgqwert1234509876poiuy\rlkjh \0mnb"
    symbol_keys = "\0:`?/~|\\{}<>[]^!@#$%_)('&\";:][\r=+-^ \0.,*"
    caps_keys = {"LEFT": '5', "DOWN": '6', "UP": '7', "RIGHT": '8',
                 "DELETE": '0', "DELETE_RIGHT": '9', "INSERT": '4',
                 "\x7f": '0'}
    modifiers = set()
    if character in caps_keys:
        index = keys.index(caps_keys[character])
        modifiers.add((0, 0))
    elif len(character) != 1:
        raise ValueError("No simulated key for %r" % character)
    elif character in keys:
        index = keys.index(character)
    elif character in symbol_keys:
        index = symbol_keys.index(character)
        modifiers.add((7, 1))
    elif character.lower() in keys:
        index = keys.index(character.lower())
        modifiers.add((0, 0))
    elif 0 < ord(character) < 32:
        index = keys.index(chr(ord(character) + 96))
        modifiers.update(((0, 0), (7, 1)))
    else:
        raise ValueError("No simulated key for %r" % character)
    return modifiers | {divmod(index, 5)}


def verify(path, dependency_path=None, max_instructions=10000000, commands="dir\r", observe=None, storage=None, drive_b=None, drive_b_fitted=False):
    """Boot and type characters or a list containing characters and named keys."""
    Simulator = simulator_class(dependency_path)
    machine = Machine(Disk(path.read_bytes()), storage=storage, drive_b=drive_b, drive_b_fitted=drive_b_fitted)
    machine.memory[0xfe00:] = machine.disk.read(0, 512)
    simulator = Simulator(machine.memory, registers={"PC": 0xfe10, "SP": 0xfe00},
                          config={"fast_ldir": True, "fast_djnz": True})
    simulator.set_tracer(machine)
    transcript = []
    queue = deque(commands)
    started_commands = False
    last_key = -100000
    prompt_count = 0
    last_frame = 0
    finish_return = None
    timer_keyboard = False
    for operations in range(max_instructions):
        if observe is not None:
            observe(machine, simulator)
        pc = simulator.registers[24]
        bank = machine.memory[0xff00]
        if finish_return is not None and pc == finish_return and bank == 2:
            return machine, "".join(transcript), operations
        output_address = machine.memory[0x10a] + 256 * machine.memory[0x10b]
        input_address = machine.memory[0x107] + 256 * machine.memory[0x108]
        if bank == 2 and pc == output_address:
            character = simulator.registers[3]
            transcript.append(chr(character))
            if "".join(transcript[-2:]) == "A>":
                prompt_count += 1
                print("Reached A: prompt (%d), %d FDC reads" % (prompt_count, len(machine.reads)), flush=True)
                if not commands or started_commands and not queue:
                    # Finish rendering '>' and restore common RAM before
                    # returning, so RAM snapshots show the completed prompt.
                    sp = simulator.registers[12]
                    finish_return = machine.memory[sp] + 256 * machine.memory[(sp + 1) & 65535]
                started_commands = True
        if bank == 2 and all(machine.memory[a] == 0xc3 for a in (0x11b, 0x11e, 0x121)):
            timer_keyboard = True
        waiting = (bank == 2 and machine.memory[pc] == 0x76 and machine.memory[0xff09]) if timer_keyboard else (bank == 2 and pc == input_address)
        if started_commands and queue and waiting:
            if simulator.registers[25] - last_key >= 220000 and machine.keyboard is None:
                character = queue.popleft()
                machine.keyboard = key_matrix(character)
                machine.keyboard_start = simulator.registers[25]
                last_key = simulator.registers[25]
        simulator.run()
        frame = simulator.registers[25] // 70908
        if frame != last_frame:
            if simulator.iff:
                simulator.accept_interrupt(simulator.registers, machine.memory, pc)
            last_frame = frame
    text = "".join(transcript)
    raise ValueError("Boot did not finish: PC=%04X bank=%d reads=%d output=%r" %
                     (simulator.registers[24], machine.memory[0xff00], len(machine.reads), text[-300:]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--dependencies", type=Path)
    parser.add_argument("--drive-b", type=Path, help="model a second floppy DSK (changes stay in RAM)")
    parser.add_argument("--storage", type=Path, help="model a DivMMC SDHC IMG (changes stay in RAM)")
    parser.add_argument("--instructions", type=int, default=10000000)
    parser.add_argument("--commands", default="dir\r")
    args = parser.parse_args()
    try:
        from sd_model import SdImage
        storage = SdImage(args.storage.read_bytes()) if args.storage else None
        from floppy_model import Floppy
        drive_b = Floppy(args.drive_b.read_bytes()) if args.drive_b else None
        machine, transcript, operations = verify(args.image, args.dependencies, args.instructions, args.commands,
                                                  storage=storage, drive_b=drive_b)
        print(transcript)
        print("Executed %d Z80 instructions; %d sector reads, %d writes." %
              (operations, len(machine.reads), len(machine.writes)))
        return 0
    except (ValueError, OSError, ImportError) as error:
        parser.exit(1, "error: %s\n" % error)


if __name__ == "__main__":
    sys.exit(main())

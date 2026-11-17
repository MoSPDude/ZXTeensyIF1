"""Execute the user's original +3 BIOS as a test oracle; bundle no OS bytes."""
from pathlib import Path
from verify_boot import Machine, simulator_class
from disk import Disk
from patch_ems import DEFAULT_INPUT, extract_components


class Plus3Machine(Machine):
    def fdc_write(self, value):
        if self.command and self.command[0] & 31 == 10:
            self.command.append(value)
            self.results.extend((0, 0, 0, self.track, 0, 1, 2))
            self.command.clear()
            return
        if not self.command and value & 31 == 10:
            self.command = [value]
            return
        super().fdc_write(value)

    def update_mapping(self, refresh_lower=True):
        for slot, (kind, bank, offset) in enumerate(self.mapping):
            if kind != 'rom':
                self.ram[bank][offset:offset + 8192] = self.memory[slot * 8192:(slot + 1) * 8192]
        if self.port1 & 1:
            pages = ((0, 1, 2, 3), (4, 5, 6, 7), (4, 5, 6, 3), (4, 7, 6, 3))[(self.port1 >> 1) & 3]
            self.mapping = [('ram', bank, offset) for bank in pages for offset in (0, 8192)]
        else:
            self.mapping = [('rom', 0, 0), ('rom', 0, 8192)]
            self.mapping += [('ram', bank, offset) for bank in (5, 2, self.port7 & 7) for offset in (0, 8192)]
        for slot, (kind, bank, offset) in enumerate(self.mapping):
            self.memory[slot * 8192:(slot + 1) * 8192] = ([255] * 8192 if kind == 'rom' else
                                                       self.ram[bank][offset:offset + 8192])


class OriginalBios:
    def __init__(self, dependencies):
        disk = Disk(DEFAULT_INPUT.read_bytes())
        extract_components(disk.read_file('S10CPM3.EMS'))
        self.machine = Plus3Machine(disk)
        self.machine.memory[0xfe00:] = disk.read(0, 512)
        self.cpu = simulator_class(dependencies)(self.machine.memory,
            registers={'PC': 0xfe10, 'SP': 0xfe00}, config={'fast_ldir': True, 'fast_djnz': True})
        self.cpu.set_tracer(self.machine)
        last_frame = 0
        for operation in range(2000000):
            pc = self.cpu.registers[24]
            if operation > 1000000 and pc == 0x1c48:
                break
            self.cpu.run()
            frame = self.cpu.registers[25] // 70908
            if frame != last_frame:
                if self.cpu.iff:
                    self.cpu.accept_interrupt(self.cpu.registers, self.machine.memory, pc)
                last_frame = frame
        else:
            raise AssertionError('Original +3 BIOS did not finish initialising')
        self.cpu.iff = 0
        if self.machine.memory[0xf60:0xf65] != [159, 122, 120, 99, 118]:
            raise AssertionError('Unexpected original +3 BIOS layout')

    def call(self, address, values=None, limit=100000):
        registers = self.cpu.registers
        if values:
            for index, value in values.items():
                registers[index] = value
        registers[12], registers[24] = 0x2e00, address
        self.machine.memory[0x2e00:0x2e02] = [0, 0x2f]
        for _ in range(limit):
            # Isolated terminal calls still receive the native vblank flag.
            if registers[24] == 0x0d28:
                self.machine.memory[0x1f2f] = 255
            self.cpu.run()
            if registers[24] == 0x2f00:
                return registers.copy()
        raise AssertionError('Original +3 BIOS call did not return')

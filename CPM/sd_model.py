"""Small SDv2/SPI model matching the Teensy IMG command subset, for CPU tests."""
from collections import deque


class SdImage:
    def __init__(self, data, read_only=False, response_delay=3):
        self.data = bytearray(data)
        self.read_only = read_only
        self.delay = response_delay
        self.selected = False
        self.idle = True
        self.app_command = False
        self.command = []
        self.response = deque()
        self.write_lba = None
        self.write_buffer = None
        self.reads = []
        self.writes = []
        self.commands = []

    def select(self, value):
        self.selected = value & 3 == 2

    def read(self):
        return self.response.popleft() if self.selected and self.response else 255

    def respond(self, values):
        self.response.extend([255] * self.delay)
        self.response.extend(values)

    def write(self, value):
        if not self.selected:
            return
        if self.write_lba is not None:
            if self.write_buffer is None:
                if value == 254:
                    self.write_buffer = bytearray()
            else:
                self.write_buffer.append(value)
                if len(self.write_buffer) == 514:
                    if self.read_only:
                        self.respond((13,))
                    else:
                        offset = self.write_lba * 512
                        self.data[offset:offset + 512] = self.write_buffer[:512]
                        self.writes.append(self.write_lba)
                        self.respond((5, 0, 0, 255))
                    self.write_lba = self.write_buffer = None
            return
        if not self.command and value & 0xc0 != 0x40:
            return
        self.command.append(value)
        if len(self.command) != 6:
            return
        command = self.command[0] & 63
        argument = int.from_bytes(bytes(self.command[1:5]), 'big')
        self.command.clear()
        self.commands.append((command, argument))
        if self.app_command:
            self.app_command = False
            if command == 41:
                self.idle = False
                self.respond((0,))
            else:
                self.respond((4,))
        elif command == 0:
            self.idle = True
            self.respond((1,))
        elif command == 8:
            self.respond((int(self.idle), 0, 0, 1, 170))
        elif command == 55:
            self.app_command = True
            self.respond((int(self.idle),))
        elif command == 58:
            self.respond((int(self.idle), 192, 255, 128, 0))
        elif command in (17, 24):
            if argument >= len(self.data) // 512:
                self.respond((64,))
            elif command == 17:
                self.reads.append(argument)
                self.respond((0, 255, 254))
                self.response.extend(self.data[argument * 512:(argument + 1) * 512])
                self.response.extend((0, 0))
            else:
                self.respond((0, 255))
                self.write_lba = argument
        else:
            self.respond((4,))

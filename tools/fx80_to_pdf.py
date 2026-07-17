#!/usr/bin/env python3
"""Convert raw Epson FX-80/ESC/P printer bytes to a PDF.

The ZXTeensyIF1 firmware captures printer output as raw bytes in printer.txt.
This tool intentionally treats that file as binary input despite the suffix.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable


PAGE_SIZES = {
    "a4": (595.276, 841.89),
    "letter": (612.0, 792.0),
}

POINTS_PER_INCH = 72.0
DEFAULT_LEFT_EDGE = 18.0
DEFAULT_TOP_EDGE = 18.0
DEFAULT_BOTTOM_EDGE = 18.0
DEFAULT_RIGHT_COLUMNS = 80

ESC = 0x1B

BIT_IMAGE_MODE_DENSITY = {
    0: (60.0, 1),
    1: (120.0, 1),
    2: (120.0, 1),
    3: (240.0, 1),
    4: (80.0, 1),
    5: (72.0, 1),
    6: (90.0, 1),
    7: (144.0, 1),
    32: (60.0, 3),
    33: (120.0, 3),
    38: (90.0, 3),
    39: (180.0, 3),
    40: (360.0, 3),
}

FX_IMAGE_COMMAND_MODES = {
    ord("K"): 0,
    ord("L"): 1,
    ord("Y"): 2,
    ord("Z"): 3,
}

FONT_NAMES = {
    "regular": "F1",
    "italic": "F2",
    "bold": "F3",
    "bold_italic": "F4",
}

FONT_OBJECTS = {
    "F1": "Courier",
    "F2": "Courier-Oblique",
    "F3": "Courier-Bold",
    "F4": "Courier-BoldOblique",
}

INTERNATIONAL_CHARSETS = {
    0: {},
    1: {
        "#": "#",
        "@": "a",
        "[": "deg",
        "\\": "c",
        "]": "S",
        "{": "e",
        "|": "u",
        "}": "e",
        "~": '"',
    },
    2: {
        "@": "S",
        "[": "A",
        "\\": "O",
        "]": "U",
        "{": "a",
        "|": "o",
        "}": "u",
        "~": "s",
    },
    3: {
        "#": "\u00a3",
    },
    4: {
        "@": "A",
        "[": "AE",
        "\\": "O",
        "]": "A",
        "{": "ae",
        "|": "o",
        "}": "a",
    },
    5: {
        "@": "E",
        "[": "A",
        "\\": "O",
        "]": "A",
        "^": "U",
        "`": "e",
        "{": "a",
        "|": "o",
        "}": "a",
        "~": "u",
    },
    6: {
        "#": "\u00a3",
        "@": "S",
        "[": "deg",
        "\\": "c",
        "]": "e",
        "`": "u",
        "{": "a",
        "|": "o",
        "}": "e",
        "~": "i",
    },
    7: {
        "#": "Pt",
        "@": "a",
        "[": "i",
        "\\": "N",
        "]": "?",
        "{": "deg",
        "|": "n",
        "}": "}",
    },
    8: {
        "\\": "\u00a5",
        "~": "-",
    },
    9: {
        "@": "A",
        "[": "AE",
        "\\": "O",
        "]": "A",
        "`": "e",
        "{": "ae",
        "|": "o",
        "}": "a",
    },
}


def number(value: float) -> str:
    text = f"{value:.3f}"
    return text.rstrip("0").rstrip(".") if "." in text else text


def pdf_hex_text(text: str) -> str:
    return "<" + text.encode("cp1252", "replace").hex().upper() + ">"


@dataclass
class PdfPage:
    width: float
    height: float
    commands: list[str] = field(default_factory=list)
    has_marks: bool = False

    def add_text(
        self,
        x: float,
        y: float,
        text: str,
        font_name: str,
        font_size: float,
        horizontal_scale: float,
    ) -> None:
        if not text:
            return
        self.commands.append(
            "\n".join(
                [
                    "BT",
                    f"/{font_name} {number(font_size)} Tf",
                    f"{number(horizontal_scale)} Tz",
                    f"1 0 0 1 {number(x)} {number(y)} Tm",
                    f"{pdf_hex_text(text)} Tj",
                    "ET",
                ]
            )
        )
        self.has_marks = True

    def add_rect(self, x: float, y: float, width: float, height: float) -> None:
        if width <= 0 or height <= 0:
            return
        self.commands.append(
            f"{number(x)} {number(y)} {number(width)} {number(height)} re f"
        )
        self.has_marks = True


@dataclass
class PdfDocument:
    width: float
    height: float
    pages: list[PdfPage] = field(default_factory=list)

    def add_page(self) -> PdfPage:
        page = PdfPage(self.width, self.height)
        self.pages.append(page)
        return page

    def to_bytes(self) -> bytes:
        if not self.pages:
            self.add_page()

        objects: list[bytes] = []

        def add_object(body: str | bytes) -> int:
            object_id = len(objects) + 1
            if isinstance(body, str):
                body_bytes = body.encode("ascii")
            else:
                body_bytes = body
            objects.append(body_bytes)
            return object_id

        catalog_id = add_object("<< /Type /Catalog /Pages 2 0 R >>")
        pages_id = add_object(b"")
        font_ids = {
            resource: add_object(
                (
                    "<< /Type /Font /Subtype /Type1 "
                    f"/BaseFont /{base_font} /Encoding /WinAnsiEncoding >>"
                )
            )
            for resource, base_font in FONT_OBJECTS.items()
        }

        page_ids: list[int] = []
        for page in self.pages:
            content = "\n".join(page.commands).encode("ascii")
            content_id = add_object(
                b"<< /Length "
                + str(len(content)).encode("ascii")
                + b" >>\nstream\n"
                + content
                + b"\nendstream"
            )
            font_resource = " ".join(
                f"/{resource} {object_id} 0 R"
                for resource, object_id in font_ids.items()
            )
            page_id = add_object(
                (
                    f"<< /Type /Page /Parent {pages_id} 0 R "
                    f"/MediaBox [0 0 {number(page.width)} {number(page.height)}] "
                    f"/Resources << /Font << {font_resource} >> >> "
                    f"/Contents {content_id} 0 R >>"
                )
            )
            page_ids.append(page_id)

        objects[pages_id - 1] = (
            "<< /Type /Pages /Kids ["
            + " ".join(f"{page_id} 0 R" for page_id in page_ids)
            + f"] /Count {len(page_ids)} >>"
        ).encode("ascii")

        output = bytearray(b"%PDF-1.4\n%\xE2\xE3\xCF\xD3\n")
        offsets = [0]
        for object_id, body in enumerate(objects, start=1):
            offsets.append(len(output))
            output.extend(f"{object_id} 0 obj\n".encode("ascii"))
            output.extend(body)
            output.extend(b"\nendobj\n")

        xref_offset = len(output)
        output.extend(f"xref\n0 {len(objects) + 1}\n".encode("ascii"))
        output.extend(b"0000000000 65535 f \n")
        for offset in offsets[1:]:
            output.extend(f"{offset:010d} 00000 n \n".encode("ascii"))
        output.extend(
            (
                "trailer\n"
                f"<< /Size {len(objects) + 1} /Root {catalog_id} 0 R >>\n"
                "startxref\n"
                f"{xref_offset}\n"
                "%%EOF\n"
            ).encode("ascii")
        )
        return bytes(output)

    def write(self, path: Path) -> None:
        path.write_bytes(self.to_bytes())


@dataclass
class Fx80State:
    cpi: float = 10.0
    line_spacing: float = 12.0
    left_margin: float = 0.0
    right_margin: float = DEFAULT_RIGHT_COLUMNS * 7.2
    intercharacter_space: float = 0.0
    condensed: bool = False
    double_width: bool = False
    one_line_double_width: bool = False
    double_height: bool = False
    emphasized: bool = False
    double_strike: bool = False
    italic: bool = False
    underline: bool = False
    superscript: bool = False
    subscript: bool = False
    international_charset: int = 0
    printable_upper_controls: bool = False
    user_defined_selected: bool = False
    horizontal_tabs: list[int] = field(default_factory=list)
    vertical_tabs: list[int] = field(default_factory=list)
    bit_image_modes: dict[int, int] = field(default_factory=lambda: dict(FX_IMAGE_COMMAND_MODES))


class Fx80Renderer:
    """Render an Epson FX-80 byte stream into a simple PDF document."""

    def __init__(
        self,
        page_size: str = "a4",
        log_commands: bool = False,
        log: Callable[[str], None] | None = None,
    ) -> None:
        if page_size not in PAGE_SIZES:
            raise ValueError(f"unknown page size: {page_size}")
        self.page_width, self.page_height = PAGE_SIZES[page_size]
        self.doc = PdfDocument(self.page_width, self.page_height)
        self.page = self.doc.add_page()
        self.log_commands = log_commands
        self.log = log if log is not None else lambda message: print(message, file=sys.stderr)
        self.cursor_x = 0.0
        self.cursor_y = 0.0
        self.page_length = self.page_height - DEFAULT_TOP_EDGE - DEFAULT_BOTTOM_EDGE
        self.state = Fx80State()
        self.user_defined_chars: dict[int, bytes] = {}

    def reset_printer(self) -> None:
        self.state = Fx80State()
        self.page_length = self.page_height - DEFAULT_TOP_EDGE - DEFAULT_BOTTOM_EDGE
        self.cursor_x = self.state.left_margin
        self.cursor_y = 0.0

    def render(self, data: bytes) -> PdfDocument:
        index = 0
        while index < len(data):
            byte = data[index]
            index += 1
            if byte == ESC:
                index = self._handle_escape(data, index)
            else:
                self._handle_byte(byte)
        return self.doc

    def _debug(self, message: str) -> None:
        if self.log_commands:
            self.log(message)

    def _read(self, data: bytes, index: int, count: int, command: str) -> tuple[bytes, int]:
        end = index + count
        if end > len(data):
            self._debug(f"{command}: truncated, expected {count} byte(s)")
            return data[index:], len(data)
        return data[index:end], end

    def _handle_byte(self, byte: int) -> None:
        if byte == 0x00:
            return
        if byte == 0x07:
            self._debug("BEL ignored")
            return
        if byte == 0x08:
            self.backspace()
            return
        if byte == 0x09:
            self.horizontal_tab()
            return
        if byte == 0x0A:
            self.line_feed()
            return
        if byte == 0x0B:
            self.vertical_tab()
            return
        if byte == 0x0C:
            self.form_feed()
            return
        if byte == 0x0D:
            self.carriage_return()
            return
        if byte == 0x0E:
            self.state.one_line_double_width = True
            return
        if byte == 0x0F:
            self.state.condensed = True
            return
        if byte == 0x11:
            return
        if byte == 0x12:
            self.state.condensed = False
            return
        if byte == 0x13:
            self._debug("DC3 ignored")
            return
        if byte == 0x14:
            self.state.one_line_double_width = False
            return
        if byte == 0x18:
            self._debug("CAN ignored")
            return
        if byte == 0x7F:
            self._debug("DEL ignored")
            return
        if byte < 0x20 and not self.state.printable_upper_controls:
            self._debug(f"control byte 0x{byte:02X} ignored")
            return
        self.print_character(byte)

    def _handle_escape(self, data: bytes, index: int) -> int:
        if index >= len(data):
            self._debug("ESC: truncated")
            return index

        command = data[index]
        index += 1

        if command == ord("@"):
            self.reset_printer()
        elif command == ord("0"):
            self.state.line_spacing = POINTS_PER_INCH / 8.0
        elif command == ord("1"):
            self.state.line_spacing = 7.0
        elif command == ord("2"):
            self.state.line_spacing = POINTS_PER_INCH / 6.0
        elif command == ord("3"):
            payload, index = self._read(data, index, 1, "ESC 3")
            if payload:
                self.state.line_spacing = payload[0] / 216.0 * POINTS_PER_INCH
        elif command == ord("A"):
            payload, index = self._read(data, index, 1, "ESC A")
            if payload:
                self.state.line_spacing = float(payload[0])
        elif command == ord("J"):
            payload, index = self._read(data, index, 1, "ESC J")
            if payload:
                self.advance_paper(payload[0] / 216.0 * POINTS_PER_INCH)
        elif command == ord("j"):
            payload, index = self._read(data, index, 1, "ESC j")
            if payload:
                self.cursor_y = max(0.0, self.cursor_y - payload[0] / 216.0 * POINTS_PER_INCH)
        elif command == ord("C"):
            payload, index = self._read(data, index, 1, "ESC C")
            if payload:
                if payload[0] == 0:
                    inches, index = self._read(data, index, 1, "ESC C NUL")
                    if inches:
                        self.page_length = max(self.state.line_spacing, inches[0] * POINTS_PER_INCH)
                else:
                    self.page_length = max(self.state.line_spacing, payload[0] * self.state.line_spacing)
        elif command == ord("N"):
            payload, index = self._read(data, index, 1, "ESC N")
            if payload:
                self._debug(f"ESC N skip-over-perforation {payload[0]} line(s) ignored")
        elif command == ord("O"):
            self._debug("ESC O cancel skip-over-perforation ignored")
        elif command == ord("P"):
            self.state.cpi = 10.0
        elif command == ord("M"):
            self.state.cpi = 12.0
        elif command == ord("g"):
            self.state.cpi = 15.0
        elif command == ord("p"):
            payload, index = self._read(data, index, 1, "ESC p")
            if payload:
                self._debug(f"ESC p proportional mode {payload[0]} ignored")
        elif command == ord("!"):
            payload, index = self._read(data, index, 1, "ESC !")
            if payload:
                self.master_select(payload[0])
        elif command == ord("E"):
            self.state.emphasized = True
        elif command == ord("F"):
            self.state.emphasized = False
        elif command == ord("G"):
            self.state.double_strike = True
        elif command == ord("H"):
            self.state.double_strike = False
        elif command == ord("4"):
            self.state.italic = True
        elif command == ord("5"):
            self.state.italic = False
        elif command == ord("-"):
            payload, index = self._read(data, index, 1, "ESC -")
            if payload:
                self.state.underline = payload[0] not in (0, ord("0"))
        elif command == ord("W"):
            payload, index = self._read(data, index, 1, "ESC W")
            if payload:
                self.state.double_width = payload[0] not in (0, ord("0"))
        elif command == ord("w"):
            payload, index = self._read(data, index, 1, "ESC w")
            if payload:
                self.state.double_height = payload[0] not in (0, ord("0"))
        elif command == ord("S"):
            payload, index = self._read(data, index, 1, "ESC S")
            if payload:
                self.state.superscript = payload[0] in (0, ord("0"))
                self.state.subscript = payload[0] in (1, ord("1"))
        elif command == ord("T"):
            self.state.superscript = False
            self.state.subscript = False
        elif command == ord(" "):
            payload, index = self._read(data, index, 1, "ESC SP")
            if payload:
                self.state.intercharacter_space = payload[0] / 120.0 * POINTS_PER_INCH
        elif command == ord("l"):
            payload, index = self._read(data, index, 1, "ESC l")
            if payload:
                self.state.left_margin = payload[0] * self.base_advance()
                if self.state.right_margin <= self.state.left_margin:
                    self.state.right_margin = self.state.left_margin + DEFAULT_RIGHT_COLUMNS * 7.2
                self.carriage_return()
        elif command == ord("Q"):
            payload, index = self._read(data, index, 1, "ESC Q")
            if payload:
                self.state.right_margin = max(
                    self.state.left_margin + self.base_advance(),
                    payload[0] * self.base_advance(),
                )
        elif command == ord("$"):
            payload, index = self._read(data, index, 2, "ESC $")
            if len(payload) == 2:
                units = payload[0] | (payload[1] << 8)
                self.cursor_x = self.state.left_margin + units / 60.0 * POINTS_PER_INCH
        elif command == ord("\\"):
            payload, index = self._read(data, index, 2, "ESC \\")
            if len(payload) == 2:
                units = payload[0] | (payload[1] << 8)
                if units >= 0x8000:
                    units -= 0x10000
                self.cursor_x = max(
                    self.state.left_margin,
                    self.cursor_x + units / 120.0 * POINTS_PER_INCH,
                )
        elif command == ord("D"):
            stops, index = self._read_tab_stops(data, index, "ESC D")
            self.state.horizontal_tabs = stops
        elif command == ord("B"):
            stops, index = self._read_tab_stops(data, index, "ESC B")
            self.state.vertical_tabs = stops
        elif command == ord("R"):
            payload, index = self._read(data, index, 1, "ESC R")
            if payload:
                self.state.international_charset = payload[0]
        elif command == ord("t"):
            payload, index = self._read(data, index, 1, "ESC t")
            if payload:
                self._debug(f"ESC t character table {payload[0]} selected approximately")
        elif command == ord("6"):
            self.state.printable_upper_controls = True
        elif command == ord("7"):
            self.state.printable_upper_controls = False
        elif command == ord("I"):
            payload, index = self._read(data, index, 1, "ESC I")
            if payload:
                self.state.printable_upper_controls = payload[0] != 0
        elif command == ord("%"):
            payload, index = self._read(data, index, 1, "ESC %")
            if payload:
                self.state.user_defined_selected = payload[0] != 0
                self._debug("user-defined characters are selected but rendered as text placeholders")
        elif command == ord("&"):
            index = self._read_user_defined_characters(data, index)
        elif command == ord(":"):
            _, index = self._read(data, index, 3, "ESC :")
            self._debug("ESC : copy ROM to RAM ignored")
        elif command == ord("?"):
            payload, index = self._read(data, index, 2, "ESC ?")
            if len(payload) == 2:
                self.state.bit_image_modes[payload[0]] = payload[1]
        elif command in FX_IMAGE_COMMAND_MODES:
            index = self._handle_named_bit_image(data, index, command)
        elif command == ord("*"):
            index = self._handle_star_bit_image(data, index)
        elif command == ord("^"):
            index = self._handle_caret_bit_image(data, index)
        elif command == ord("("):
            index = self._handle_esc_paren(data, index)
        elif command in (ord("U"), ord("x"), ord("k")):
            payload, index = self._read(data, index, 1, f"ESC {chr(command)}")
            if payload:
                self._debug(f"ESC {chr(command)} {payload[0]} ignored")
        elif command == ord("="):
            payload, index = self._read(data, index, 1, "ESC =")
            if payload:
                self._debug("ESC = select peripheral ignored")
        else:
            printable = chr(command) if 32 <= command <= 126 else f"0x{command:02X}"
            self._debug(f"unsupported ESC {printable}")
        return index

    def _read_tab_stops(self, data: bytes, index: int, command: str) -> tuple[list[int], int]:
        stops: list[int] = []
        while index < len(data):
            value = data[index]
            index += 1
            if value == 0:
                return stops, index
            stops.append(value)
            if len(stops) >= 32:
                self._debug(f"{command}: more than 32 stops, remaining bytes may be print data")
                return stops, index
        self._debug(f"{command}: truncated before NUL terminator")
        return stops, index

    def _read_user_defined_characters(self, data: bytes, index: int) -> int:
        header, index = self._read(data, index, 3, "ESC &")
        if len(header) != 3:
            return index
        first = header[1]
        last = header[2]
        if last < first:
            self._debug("ESC &: invalid character range")
            return index
        count = last - first + 1
        byte_count = count * 12
        payload, index = self._read(data, index, byte_count, "ESC & data")
        for offset, char_code in enumerate(range(first, last + 1)):
            glyph = payload[offset * 12 : (offset + 1) * 12]
            if len(glyph) == 12:
                self.user_defined_chars[char_code] = glyph[1:]
        self._debug("ESC &: user-defined glyph shapes loaded approximately")
        return index

    def _handle_named_bit_image(self, data: bytes, index: int, command: int) -> int:
        payload, index = self._read(data, index, 2, f"ESC {chr(command)}")
        if len(payload) != 2:
            return index
        count = payload[0] | (payload[1] << 8)
        mode = self.state.bit_image_modes.get(command, FX_IMAGE_COMMAND_MODES[command])
        return self._render_bit_image_payload(data, index, count, mode, f"ESC {chr(command)}")

    def _handle_star_bit_image(self, data: bytes, index: int) -> int:
        payload, index = self._read(data, index, 3, "ESC *")
        if len(payload) != 3:
            return index
        mode = payload[0]
        count = payload[1] | (payload[2] << 8)
        return self._render_bit_image_payload(data, index, count, mode, "ESC *")

    def _handle_caret_bit_image(self, data: bytes, index: int) -> int:
        payload, index = self._read(data, index, 3, "ESC ^")
        if len(payload) != 3:
            return index
        mode = payload[0]
        count = payload[1] | (payload[2] << 8)
        return self._render_bit_image_payload(data, index, count, mode, "ESC ^")

    def _render_bit_image_payload(
        self,
        data: bytes,
        index: int,
        count: int,
        mode: int,
        command: str,
    ) -> int:
        density, bytes_per_column = BIT_IMAGE_MODE_DENSITY.get(mode, (60.0, 1))
        if mode not in BIT_IMAGE_MODE_DENSITY:
            self._debug(f"{command}: unsupported bit-image mode {mode}, using 60 dpi")
        byte_count = count * bytes_per_column
        payload, next_index = self._read(data, index, byte_count, f"{command} data")
        complete_columns = len(payload) // bytes_per_column
        if complete_columns > 0:
            self.draw_bit_image(payload[: complete_columns * bytes_per_column], density, bytes_per_column)
        return next_index

    def _handle_esc_paren(self, data: bytes, index: int) -> int:
        header, index = self._read(data, index, 3, "ESC (")
        if len(header) != 3:
            return index
        subcommand = header[0]
        length = header[1] | (header[2] << 8)
        payload, index = self._read(data, index, length, f"ESC ( {chr(subcommand)}")
        if subcommand == ord("^"):
            for byte in payload:
                self.print_character(byte)
        elif subcommand == ord("C") and len(payload) >= 2:
            units = payload[0] | (payload[1] << 8)
            if units > 0:
                self.page_length = min(
                    units / 360.0 * POINTS_PER_INCH,
                    self.page_height - DEFAULT_TOP_EDGE,
                )
        elif subcommand == ord("U"):
            self._debug("ESC ( U unit definition ignored")
        elif subcommand == ord("t"):
            self._debug("ESC ( t character table assignment ignored")
        elif subcommand == ord("c"):
            self._debug("ESC ( c top/bottom margins ignored")
        else:
            printable = chr(subcommand) if 32 <= subcommand <= 126 else f"0x{subcommand:02X}"
            self._debug(f"ESC ( {printable} skipped {length} byte(s)")
        return index

    def master_select(self, value: int) -> None:
        self.state.cpi = 12.0 if (value & 0x01) else 10.0
        self.state.condensed = (value & 0x04) != 0
        self.state.emphasized = (value & 0x08) != 0
        self.state.double_strike = (value & 0x10) != 0
        self.state.double_width = (value & 0x20) != 0
        self.state.italic = (value & 0x40) != 0
        self.state.underline = (value & 0x80) != 0

    def base_advance(self) -> float:
        return POINTS_PER_INCH / self.state.cpi

    def character_advance(self) -> float:
        advance = self.base_advance() + self.state.intercharacter_space
        if self.state.condensed:
            advance *= 0.6
        if self.state.double_width or self.state.one_line_double_width:
            advance *= 2.0
        return advance

    def font_size(self) -> float:
        size = 12.0
        if self.state.superscript or self.state.subscript:
            size *= 0.67
        if self.state.double_height:
            size *= 2.0
        return size

    def horizontal_scale(self) -> float:
        natural_width = 0.6 * self.font_size()
        if natural_width <= 0:
            return 100.0
        return self.character_advance() / natural_width * 100.0

    def font_resource(self) -> str:
        if self.state.italic and (self.state.emphasized or self.state.double_strike):
            return FONT_NAMES["bold_italic"]
        if self.state.italic:
            return FONT_NAMES["italic"]
        if self.state.emphasized or self.state.double_strike:
            return FONT_NAMES["bold"]
        return FONT_NAMES["regular"]

    def carriage_return(self) -> None:
        self.cursor_x = self.state.left_margin

    def line_feed(self) -> None:
        self.advance_paper(self.state.line_spacing)
        self.state.one_line_double_width = False

    def advance_paper(self, amount: float) -> None:
        self.cursor_y += amount
        if self.cursor_y + self.state.line_spacing > self.page_limit():
            self.form_feed()

    def page_limit(self) -> float:
        media_limit = self.page_height - DEFAULT_TOP_EDGE - DEFAULT_BOTTOM_EDGE
        return min(self.page_length, media_limit)

    def form_feed(self) -> None:
        if self.page.has_marks:
            self.page = self.doc.add_page()
        self.cursor_x = self.state.left_margin
        self.cursor_y = 0.0
        self.state.one_line_double_width = False

    def backspace(self) -> None:
        self.cursor_x = max(self.state.left_margin, self.cursor_x - self.character_advance())

    def horizontal_tab(self) -> None:
        advance = max(self.character_advance(), 0.1)
        if self.state.horizontal_tabs:
            stops = [self.state.left_margin + stop * advance for stop in self.state.horizontal_tabs]
            next_stop = next((stop for stop in stops if stop > self.cursor_x + 0.01), None)
            if next_stop is None:
                return
            self.cursor_x = min(next_stop, self.state.right_margin)
            return

        relative = max(0.0, self.cursor_x - self.state.left_margin)
        current_column = int(relative // advance)
        next_column = ((current_column // 8) + 1) * 8
        self.cursor_x = min(self.state.left_margin + next_column * advance, self.state.right_margin)

    def vertical_tab(self) -> None:
        if self.state.vertical_tabs:
            stops = [stop * self.state.line_spacing for stop in self.state.vertical_tabs]
            next_stop = next((stop for stop in stops if stop > self.cursor_y + 0.01), None)
            if next_stop is not None:
                self.cursor_y = next_stop
                if self.cursor_y + self.state.line_spacing > self.page_limit():
                    self.form_feed()
                return
        self.line_feed()

    def print_character(self, byte: int) -> None:
        if self.state.user_defined_selected and byte in self.user_defined_chars:
            self.print_user_defined_character(byte)
            return

        text = self.character_for_byte(byte)
        if not text:
            return

        advance = self.character_advance() * len(text)
        if self.cursor_x + advance > self.state.right_margin + 0.01:
            self.carriage_return()
            self.line_feed()

        x = DEFAULT_LEFT_EDGE + self.cursor_x
        baseline_from_top = DEFAULT_TOP_EDGE + self.cursor_y + self.font_size() * 0.82
        if self.state.superscript:
            baseline_from_top -= 4.0
        elif self.state.subscript:
            baseline_from_top += 3.0
        y = self.page.height - baseline_from_top
        font_size = self.font_size()
        scale = self.horizontal_scale()
        font = self.font_resource()
        self.page.add_text(x, y, text, font, font_size, scale)
        if self.state.emphasized or self.state.double_strike:
            self.page.add_text(x + 0.45, y, text, font, font_size, scale)
        if self.state.underline:
            self.page.add_rect(x, y - 2.0, max(advance, 0.5), 0.55)
        self.cursor_x += advance

    def character_for_byte(self, byte: int) -> str:
        if 32 <= byte <= 126:
            char = chr(byte)
            return INTERNATIONAL_CHARSETS.get(self.state.international_charset, {}).get(char, char)
        if 160 <= byte <= 255:
            char = bytes([byte]).decode("cp1252", "replace")
            return char if char.isprintable() else "?"
        if self.state.printable_upper_controls and 128 <= byte <= 159:
            return "?"
        return "?"

    def print_user_defined_character(self, byte: int) -> None:
        glyph = self.user_defined_chars[byte]
        advance = self.character_advance()
        if self.cursor_x + advance > self.state.right_margin + 0.01:
            self.carriage_return()
            self.line_feed()

        x_step = advance / max(1, len(glyph))
        vertical_scale = self.font_size() / 12.0
        dot_width = max(0.35, min(x_step, 1.0))
        dot_height = max(0.5, 0.85 * vertical_scale)
        base_x = DEFAULT_LEFT_EDGE + self.cursor_x
        base_top = DEFAULT_TOP_EDGE + self.cursor_y + 1.5
        for column, value in enumerate(glyph):
            for bit in range(8):
                if (value & (0x80 >> bit)) == 0:
                    continue
                dot_top = base_top + bit * vertical_scale
                x = base_x + column * x_step
                y = self.page.height - dot_top - dot_height
                self.page.add_rect(x, y, dot_width, dot_height)
        if self.state.underline:
            baseline = self.page.height - (
                DEFAULT_TOP_EDGE + self.cursor_y + self.font_size() * 0.82
            )
            self.page.add_rect(base_x, baseline - 2.0, max(advance, 0.5), 0.55)
        self.cursor_x += advance

    def draw_bit_image(self, payload: bytes, horizontal_density: float, bytes_per_column: int) -> None:
        if bytes_per_column <= 0:
            return
        columns = len(payload) // bytes_per_column
        x_step = POINTS_PER_INCH / horizontal_density
        dot_width = max(0.35, min(x_step, 1.0))
        dot_height = 0.85
        vertical_step = 1.0
        base_x = DEFAULT_LEFT_EDGE + self.cursor_x
        base_top = DEFAULT_TOP_EDGE + self.cursor_y

        for column in range(columns):
            for byte_index in range(bytes_per_column):
                value = payload[column * bytes_per_column + byte_index]
                for bit in range(8):
                    if (value & (0x80 >> bit)) == 0:
                        continue
                    dot_top = base_top + (byte_index * 8 + bit) * vertical_step
                    x = base_x + column * x_step
                    y = self.page.height - dot_top - dot_height
                    self.page.add_rect(x, y, dot_width, dot_height)
        self.cursor_x += columns * x_step


def render_to_pdf_bytes(
    data: bytes,
    page_size: str = "a4",
    log_commands: bool = False,
    log: Callable[[str], None] | None = None,
) -> bytes:
    renderer = Fx80Renderer(page_size=page_size, log_commands=log_commands, log=log)
    renderer.render(data)
    return renderer.doc.to_bytes()


def convert_file(
    input_path: Path,
    output_path: Path,
    page_size: str = "a4",
    log_commands: bool = False,
) -> None:
    data = input_path.read_bytes()
    output_path.write_bytes(
        render_to_pdf_bytes(data, page_size=page_size, log_commands=log_commands)
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert raw Epson FX-80/ESC/P printer bytes to PDF."
    )
    parser.add_argument("input", type=Path, help="raw printer byte stream, e.g. printer.txt")
    parser.add_argument("output", type=Path, help="output PDF path")
    parser.add_argument(
        "--page-size",
        choices=sorted(PAGE_SIZES),
        default="a4",
        help="PDF page size, default: a4",
    )
    parser.add_argument(
        "--log-commands",
        action="store_true",
        help="log unsupported or approximate ESC/P command handling to stderr",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        convert_file(args.input, args.output, args.page_size, args.log_commands)
    except OSError as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

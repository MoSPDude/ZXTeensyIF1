#!/usr/bin/env python3
"""Split a DivMMC HDF/IMG image into ZXTeensyIF1 segment files.

The firmware treats a selected image path as segment 0, then appends files
named from the same stem with numeric suffixes: image.hdf, image.001,
image.002, and so on. Only the first file carries any HDF header/prefix bytes;
continuation files contain whole 512-byte sectors.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


SECTOR_SIZE = 512
FAT32_MAX_FILE_SIZE = (1 << 32) - 1
MAX_SEGMENTS = 1000
COPY_BUFFER_SIZE = 1024 * 1024


@dataclass(frozen=True)
class Segment:
    index: int
    path: Path
    source_offset: int
    length: int


def parse_size(value: str) -> int:
    units = {
        "": 1,
        "b": 1,
        "k": 1000,
        "kb": 1000,
        "m": 1000**2,
        "mb": 1000**2,
        "g": 1000**3,
        "gb": 1000**3,
        "ki": 1024,
        "kib": 1024,
        "mi": 1024**2,
        "mib": 1024**2,
        "gi": 1024**3,
        "gib": 1024**3,
    }
    match = re.fullmatch(r"\s*(\d+)\s*([a-zA-Z]*)\s*", value)
    if match is None:
        raise argparse.ArgumentTypeError(f"invalid size: {value!r}")

    number = int(match.group(1))
    suffix = match.group(2).lower()
    if suffix not in units:
        raise argparse.ArgumentTypeError(f"unknown size suffix: {suffix!r}")
    return number * units[suffix]


def format_size(size: int) -> str:
    if size >= 1024**3:
        return f"{size / (1024**3):.2f} GiB"
    if size >= 1024**2:
        return f"{size / (1024**2):.2f} MiB"
    if size >= 1024:
        return f"{size / 1024:.2f} KiB"
    return f"{size} bytes"


def segment_path(base_path: Path, index: int) -> Path:
    if index == 0:
        return base_path
    return base_path.with_suffix(f".{index:03d}")


def normalized_path(path: Path) -> str:
    return os.path.normcase(str(path.resolve(strict=False)))


def hdf_sector_offset(source_path: Path) -> int:
    with source_path.open("rb") as source:
        header = source.read(16)
    if len(header) >= 16 and header.startswith(b"RS-IDE"):
        return header[9] | (header[10] << 8)
    return 0


def build_plan(source_path: Path, output_base: Path, max_file_size: int) -> list[Segment]:
    source_size = source_path.stat().st_size
    sector_offset = hdf_sector_offset(source_path)
    if source_size < sector_offset:
        raise ValueError(
            f"HDF sector offset {sector_offset} is beyond source size {source_size}"
        )
    if max_file_size <= 0:
        raise ValueError("--max-file-size must be positive")
    if sector_offset > max_file_size:
        raise ValueError(
            "the HDF header/prefix is larger than the requested maximum file size"
        )

    data_size = source_size - sector_offset
    if (data_size % SECTOR_SIZE) != 0:
        raise ValueError(
            f"sector data size {data_size} is not a multiple of {SECTOR_SIZE}"
        )

    first_payload_max = ((max_file_size - sector_offset) // SECTOR_SIZE) * SECTOR_SIZE
    continuation_payload_max = (max_file_size // SECTOR_SIZE) * SECTOR_SIZE
    if data_size > 0 and first_payload_max <= 0:
        raise ValueError("maximum file size leaves no room for data sectors")
    if data_size > first_payload_max and continuation_payload_max <= 0:
        raise ValueError("maximum file size leaves no room for continuation sectors")

    plan: list[Segment] = []
    remaining = data_size
    source_offset = sector_offset

    first_payload = min(remaining, first_payload_max)
    plan.append(
        Segment(
            index=0,
            path=segment_path(output_base, 0),
            source_offset=0,
            length=sector_offset + first_payload,
        )
    )
    remaining -= first_payload
    source_offset += first_payload

    segment_index = 1
    while remaining > 0:
        if segment_index >= MAX_SEGMENTS:
            raise ValueError(
                f"image needs more than {MAX_SEGMENTS} files, but firmware scans .001-.999"
            )
        payload = min(remaining, continuation_payload_max)
        plan.append(
            Segment(
                index=segment_index,
                path=segment_path(output_base, segment_index),
                source_offset=source_offset,
                length=payload,
            )
        )
        remaining -= payload
        source_offset += payload
        segment_index += 1

    return plan


def existing_contiguous_segments(output_base: Path) -> list[Path]:
    paths: list[Path] = []
    for index in range(1, MAX_SEGMENTS):
        path = segment_path(output_base, index)
        if not path.exists():
            break
        paths.append(path)
    return paths


def check_outputs(source_path: Path, plan: Iterable[Segment], stale_paths: Iterable[Path], force: bool) -> None:
    source_normalized = normalized_path(source_path)
    for segment in plan:
        if normalized_path(segment.path) == source_normalized:
            raise ValueError("output path must not be the same as the source path")
        if segment.path.exists() and not force:
            raise ValueError(f"output already exists: {segment.path}")

    stale = list(stale_paths)
    for stale_path in stale:
        if normalized_path(stale_path) == source_normalized:
            raise ValueError("a stale continuation file is the source path")

    if stale and not force:
        raise ValueError(
            "stale continuation files already exist; use --force to replace/remove them: "
            + ", ".join(str(path) for path in stale)
        )


def copy_range(source, destination: Path, offset: int, length: int) -> Path:
    temp_path = destination.with_name(
        f"{destination.name}.split-tmp-{os.getpid()}"
    )
    if temp_path.exists():
        raise ValueError(f"temporary output already exists: {temp_path}")

    source.seek(offset)
    remaining = length
    with temp_path.open("wb") as output:
        while remaining > 0:
            chunk = source.read(min(COPY_BUFFER_SIZE, remaining))
            if not chunk:
                raise IOError("unexpected end of source file")
            output.write(chunk)
            remaining -= len(chunk)
    return temp_path


def write_segments(source_path: Path, plan: list[Segment], stale_paths: list[Path]) -> None:
    temp_paths: list[tuple[Path, Path]] = []
    try:
        plan[0].path.parent.mkdir(parents=True, exist_ok=True)
        with source_path.open("rb") as source:
            for segment in plan:
                segment.path.parent.mkdir(parents=True, exist_ok=True)
                temp_path = copy_range(
                    source, segment.path, segment.source_offset, segment.length
                )
                temp_paths.append((temp_path, segment.path))

        for temp_path, final_path in temp_paths:
            os.replace(temp_path, final_path)

        for stale_path in stale_paths:
            stale_path.unlink()
    except Exception:
        for temp_path, _ in temp_paths:
            try:
                if temp_path.exists():
                    temp_path.unlink()
            except OSError:
                pass
        raise


def print_plan(source_path: Path, plan: list[Segment], stale_paths: list[Path], dry_run: bool) -> None:
    action = "Would write" if dry_run else "Writing"
    print(f"{action} {len(plan)} file(s) from {source_path}:")
    for segment in plan:
        print(
            f"  {segment.path}  "
            f"{format_size(segment.length)}  "
            f"from offset {segment.source_offset}"
        )
    if stale_paths:
        print("Stale continuation files to remove:")
        for path in stale_paths:
            print(f"  {path}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Split a monolithic HDF/IMG file into ZXTeensyIF1 segment files "
            "such as esximage.hdf, esximage.001, esximage.002."
        )
    )
    parser.add_argument("source", type=Path, help="source monolithic image")
    parser.add_argument(
        "output",
        type=Path,
        help="output base image path, for example /media/sd/esximage.hdf",
    )
    parser.add_argument(
        "--max-file-size",
        type=parse_size,
        default=FAT32_MAX_FILE_SIZE,
        help=(
            "maximum output file size; accepts bytes or K/M/G/KiB/MiB/GiB "
            f"suffixes (default: {FAT32_MAX_FILE_SIZE})"
        ),
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="replace existing output files and remove stale continuation files",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="show the files that would be written without writing them",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="only print errors",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    source_path = args.source
    output_base = args.output

    try:
        if not source_path.is_file():
            raise ValueError(f"source is not a file: {source_path}")

        plan = build_plan(source_path, output_base, args.max_file_size)
        planned_paths = {normalized_path(segment.path) for segment in plan}
        stale_paths = [
            path for path in existing_contiguous_segments(output_base)
            if normalized_path(path) not in planned_paths
        ]
        check_outputs(source_path, plan, stale_paths, args.force)

        if not args.quiet:
            print_plan(source_path, plan, stale_paths, args.dry_run)

        if not args.dry_run:
            write_segments(source_path, plan, stale_paths)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

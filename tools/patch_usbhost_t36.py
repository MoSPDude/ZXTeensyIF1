"""Preview/apply the Teensy 1.60.0 USBHost_t36 fixes needed by HDMI output.

Run with --apply to update the installed library, or --check to verify it.
Original files are saved beside them with a .zxteensy-hdmi.bak suffix.
"""

import argparse
import difflib
import os
from pathlib import Path


def function_span(source, signature):
    start = source.index(signature)
    end = source.index("\n}", start) + 2
    return start, end


def patch_timers(source):
    for name, arguments in (("start", "uint32_t microseconds"), ("stop", "")):
        start, end = function_span(source, "void USBDriverTimer::%s(%s)" % (name, arguments))
        body = source[start:end]
        disable = "\t\t" if name == "start" else "\t"
        original = disable + "__disable_irq();"
        replacement = (disable + "// Keep the Spectrum GPIO bus interrupt running.\n" +
                       disable + "bool irq_was_enabled = NVIC_IS_ENABLED(IRQ_USBHS);\n" +
                       disable + "NVIC_DISABLE_IRQ(IRQ_USBHS);")
        restore = disable + "if (irq_was_enabled) NVIC_ENABLE_IRQ(IRQ_USBHS);"
        if original in body:
            if body.count(original) != 1 or body.count("__enable_irq();") != 1:
                raise ValueError("Unexpected USBDriverTimer::" + name + " implementation")
            body = body.replace(original, replacement)
            body = body.replace(disable + "__enable_irq();", restore)
        elif replacement not in body or restore not in body:
            raise ValueError("Unrecognised USBDriverTimer::" + name + " implementation")
        if "__disable_irq" in body or "__enable_irq" in body:
            raise ValueError("Global interrupt masking remains in USBDriverTimer::" + name)
        source = source[:start] + body + source[end:]
    return source


def patch_serial(source):
    start, end = function_span(source, "void USBSerialBase::tx_data(const Transfer_t *transfer)")
    body = source[start:end]
    original = "\telse txstate &= ~(mask | 4);"
    replacement = "\telse txstate &= ~4;"
    # Match an active statement, rather than an old statement in a comment.
    if "\n" + original in body:
        body = body.replace("\n" + original, "\n" + replacement, 1)
    elif "\n" + replacement not in body:
        raise ValueError("Unrecognised USBSerialBase::tx_data implementation")
    return source[:start] + body + source[end:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, default=Path(os.environ.get("USERPROFILE", str(Path.home()))) /
                        "AppData/Local/Arduino15/packages/teensy/hardware/avr/1.60.0/libraries/USBHost_t36")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args()
    changes = []
    # Validate both files before writing either of them.
    for name, patch in (("ehci.cpp", patch_timers), ("serial.cpp", patch_serial)):
        path = args.library / name
        original = path.read_bytes()
        source = original.decode("utf-8").replace("\r\n", "\n")
        updated = patch(source)
        if updated != source:
            newline = "\r\n" if b"\r\n" in original else "\n"
            changes.append((path, original, updated.replace("\n", newline).encode("utf-8")))
            if not args.check:
                print("".join(difflib.unified_diff(source.splitlines(True), updated.splitlines(True),
                                                 fromfile=str(path), tofile=str(path))), end="")
    if args.check:
        if changes:
            parser.exit(1, "USBHost_t36 HDMI fixes are missing: " + ", ".join(p.name for p, _, _ in changes) + "\n")
        print("USBHost_t36 HDMI fixes verified.")
    elif args.apply:
        for path, original, updated in changes:
            backup = path.with_name(path.name + ".zxteensy-hdmi.bak")
            if not backup.exists():
                backup.write_bytes(original)
            path.write_bytes(updated)
        print("Updated %d USBHost_t36 file(s)." % len(changes))
    elif changes:
        print("Preview only. Run again with --apply to update the installed library.")
    else:
        print("USBHost_t36 HDMI fixes are already applied.")


if __name__ == "__main__":
    main()

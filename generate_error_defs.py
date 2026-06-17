#!/usr/bin/env python3

import re
import sys
from pathlib import Path

DEF_FILE = Path(__file__).parent / "errors.def"
OUT_H = Path(__file__).parent / "esp/include/errors_generated.h"
OUT_TXT = Path(__file__).parent / "VGUI2/vcom/error_codes.txt"

LINE_RE = re.compile(r"^BIT(\d+)\s*=\s*([A-Za-z_][A-Za-z0-9_]*)\s*(?::\s*(.*))?$")


def parse(def_path: Path):
    bits = []
    seen_bits = set()
    seen_names = set()

    for lineno, raw in enumerate(def_path.read_text().splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        m = LINE_RE.match(line)
        if not m:
            sys.exit(f"[generate_errors] Line {lineno}: malformed entry: {raw!r}")

        bit = int(m.group(1))
        name = m.group(2)
        desc = (m.group(3) or "").strip()

        if not (0 <= bit <= 31):
            sys.exit(f"[generate_errors] Line {lineno}: BIT{bit} out of range 0-31")
        if bit in seen_bits:
            sys.exit(f"[generate_errors] Line {lineno}: BIT{bit} defined more than once")
        if name in seen_names:
            sys.exit(f"[generate_errors] Line {lineno}: name '{name}' reused")

        seen_bits.add(bit)
        seen_names.add(name)
        bits.append((bit, name, desc))

    bits.sort(key=lambda b: b[0])
    return bits


def emit_cpp(bits):
    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("// =========================================================")
    lines.append("// AUTO-GENERATED FILE -- DO NOT EDIT BY HAND")
    lines.append("// Source: errors.def")
    lines.append("// =========================================================")
    lines.append("")
    lines.append("enum ErrorBit : uint8_t")
    lines.append("{")
    for bit, name, desc in bits:
        comment = f"  // {desc}" if desc else ""
        lines.append(f"    ERR_{name.ljust(14)}= {bit},{comment}")
    lines.append("};")
    lines.append("")
    lines.append(f"#define ERROR_BIT_COUNT {len(bits)}")
    lines.append("")
    lines.append("struct ErrorBitInfo")
    lines.append("{")
    lines.append("    uint8_t bit;")
    lines.append("    const char* name;")
    lines.append("    const char* description;")
    lines.append("};")
    lines.append("")
    lines.append("static const ErrorBitInfo ERROR_BIT_TABLE[ERROR_BIT_COUNT] = {")
    for bit, name, desc in bits:
        lines.append(f'    {{ {bit}, "{name}", "{desc}" }},')
    lines.append("};")
    lines.append("")
    OUT_H.write_text("\n".join(lines) + "\n")

def emit_txt(bits):
    lines = []
    lines.append("# AUTO-GENERATED FILE -- DO NOT EDIT BY HAND")
    lines.append("# Source: errors.def")
    lines.append("# Format BIT<N>=SHORT_NAME : Description (optional)")
    lines.append("# Bits 0 - 31 supported")
    lines.append("")
    for bit, name, desc in bits:
        if desc:
            lines.append(f"BIT{bit}={name} : {desc}")
        else:
            lines.append(f"BIT{bit}={name}")
    OUT_TXT.write_text("\n".join(lines) + "\n")


def main():
    bits = parse(DEF_FILE)
    emit_cpp(bits)
    emit_txt(bits)
    print(f"[generate_errors] Parsed {len(bits)} bits from {DEF_FILE.name}")
    print(f"[generate_errors] Wrote {OUT_H.name}")
    print(f"[generate_errors] Wrote {OUT_TXT.name}")


if __name__ == "__main__":
    main()
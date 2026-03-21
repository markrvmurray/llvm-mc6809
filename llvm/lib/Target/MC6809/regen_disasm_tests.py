#!/usr/bin/env python3
"""Regenerate disassembler test data by running hex bytes through llvm-mc."""

import os
import re
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Navigate from lib/Target/MC6809/ up to llvm/, then into the build dir
LLVM_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
LLVM_MC = os.path.join(LLVM_DIR, "cmake-build-debug-system", "bin", "llvm-mc")

FILES = [
    {
        "path": os.path.join(SCRIPT_DIR, "MC6809InstrDisassembler6809Tests.txt"),
        "args": [LLVM_MC, "-triple=mc6809", "-disassemble"],
    },
    {
        "path": os.path.join(SCRIPT_DIR, "MC6809InstrDisassembler6309Tests.txt"),
        "args": [LLVM_MC, "-triple=mc6809", "-mcpu=hd6309", "-disassemble"],
    },
]

# Matches a data line: leading whitespace, hex bytes, then # separator
DATA_LINE_RE = re.compile(r'^(\s*)(0x[0-9A-Fa-f?]{2}(?:\s+0x[0-9A-Fa-f?]{2})*)\s+(#)\t(.*)$')

# To split the assembly part from the description:
# The description is separated from operands by 2+ consecutive spaces.
# Format after #: \t<mnemonic>\t<operands>    <description>
# Or:            \t<mnemonic>             (no operands, description may follow after spaces)
ASSEMBLY_DESC_RE = re.compile(r'^(\S+(?:\t\S.+?)?)(\s{2,}.+)?$')


def disassemble(hex_bytes_str, args):
    """Run hex bytes through llvm-mc and return the disassembled output."""
    try:
        result = subprocess.run(
            args,
            input=hex_bytes_str + "\n",
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode != 0:
            return None
        output = result.stdout.strip()
        if not output:
            return None
        # Output format: \t<mnemonic>\t<operands>
        # Strip the leading tab
        if output.startswith('\t'):
            output = output[1:]
        return output
    except (subprocess.TimeoutExpired, OSError):
        return None


def process_file(file_info):
    path = file_info["path"]
    args = file_info["args"]
    filename = os.path.basename(path)

    print(f"\nProcessing: {filename}")

    with open(path, 'r') as f:
        lines = f.readlines()

    updated = 0
    unchanged = 0
    skipped = 0
    errors = 0
    new_lines = []

    for line_no, line in enumerate(lines, 1):
        stripped = line.rstrip('\n')

        # Check if it's a data line
        m = DATA_LINE_RE.match(stripped)
        if not m:
            # Comment/marker line or blank — keep as-is
            new_lines.append(line)
            continue

        leading_ws = m.group(1)
        hex_bytes = m.group(2)
        separator = m.group(3)  # '#'
        after_hash = m.group(4)

        # Skip lines with 0x?? placeholder bytes
        if '0x??' in hex_bytes:
            new_lines.append(line)
            skipped += 1
            continue

        # Parse existing assembly and description from after the hash
        # after_hash is like: mnemonic\toperands    description
        asm_desc_m = ASSEMBLY_DESC_RE.match(after_hash)
        if asm_desc_m:
            old_asm = asm_desc_m.group(1)
            description = asm_desc_m.group(2) if asm_desc_m.group(2) else ""
        else:
            old_asm = after_hash
            description = ""

        # Run through disassembler
        disasm_output = disassemble(hex_bytes, args)
        if disasm_output is None:
            # Disassembler failed — keep original
            new_lines.append(line)
            errors += 1
            continue

        # disasm_output is like: mnemonic\toperands
        new_asm = disasm_output

        if new_asm == old_asm:
            new_lines.append(line)
            unchanged += 1
        else:
            # Reconstruct the line
            # Figure out how wide the hex bytes column is (pad to align #)
            # Original format preserves column alignment. Let's measure from the original.
            hash_pos = stripped.index('#')
            hex_part = leading_ws + hex_bytes
            padding = " " * (hash_pos - len(hex_part))
            new_line = hex_part + padding + "#\t" + new_asm + description + "\n"
            new_lines.append(new_line)
            updated += 1

    # Write back
    with open(path, 'w') as f:
        f.writelines(new_lines)

    print(f"  Updated:   {updated}")
    print(f"  Unchanged: {unchanged}")
    print(f"  Skipped:   {skipped} (placeholder bytes)")
    print(f"  Errors:    {errors} (disassembler failed, kept original)")
    print(f"  Total:     {len(lines)} lines")

    return updated


def main():
    if not os.path.isfile(LLVM_MC):
        print(f"ERROR: llvm-mc not found at {LLVM_MC}")
        sys.exit(1)

    total_updated = 0
    for file_info in FILES:
        total_updated += process_file(file_info)

    print(f"\nDone. Total lines updated across all files: {total_updated}")


if __name__ == "__main__":
    main()

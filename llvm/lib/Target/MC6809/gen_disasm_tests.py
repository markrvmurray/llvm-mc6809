#!/usr/bin/env python3
"""Generate LLVM Lit disassembler tests from MC6809 disassembler test data.

Reads the disassembler test data files, runs each through llvm-mc -disassemble
to get the actual output, and generates .txt files with CHECK lines.

Usage:
    python3 gen_disasm_tests.py [path-to-llvm-mc]
"""

import os
import re
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))

INPUT_6809 = os.path.join(SCRIPT_DIR, 'MC6809InstrDisassembler6809Tests.txt')
INPUT_6309 = os.path.join(SCRIPT_DIR, 'MC6809InstrDisassembler6309Tests.txt')

OUTPUT_DIR = os.path.join(REPO_ROOT, 'test', 'MC', 'Disassembler', 'MC6809')
OUTPUT_6809 = os.path.join(OUTPUT_DIR, 'disasm-6809.txt')
OUTPUT_6309 = os.path.join(OUTPUT_DIR, 'disasm-6309.txt')

# Instructions to skip. Currently empty — both hcf and reset are now
# defined in the .td and disassemble cleanly.
SKIP_MNEMONICS = set()


def parse_line(line):
    """Parse a disassembler test line. Returns (hex_bytes_str, mnemonic) or None."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None

    hash_idx = line.find('#')
    if hash_idx < 0:
        return None

    hex_part = line[:hash_idx].strip()
    asm_part = line[hash_idx + 1:]

    if not hex_part or not asm_part:
        return None

    hex_tokens = hex_part.split()
    for token in hex_tokens:
        if '?' in token:
            return None
        if not token.startswith('0x') and not token.startswith('0X'):
            return None

    if not hex_tokens:
        return None

    # Extract mnemonic from assembly part.
    parts = asm_part.split('\t')
    parts = [p for p in parts if p.strip()]
    if not parts:
        return None

    mnemonic = parts[0].strip()
    if mnemonic in SKIP_MNEMONICS:
        return None

    # Format hex bytes for disassembler input: "0xNN 0xNN ..." or "0xnn,0xnn,..."
    # llvm-mc -disassemble accepts space-separated hex bytes.
    hex_str = ' '.join(t.lower() for t in hex_tokens)

    return (hex_str, mnemonic)


def get_disasm_output(hex_str, llvm_mc, triple, mcpu=None):
    """Run hex bytes through llvm-mc -disassemble and return the output line."""
    cmd = [llvm_mc, f'-triple={triple}', '-disassemble']
    if mcpu:
        cmd.append(f'-mcpu={mcpu}')

    try:
        result = subprocess.run(cmd, input=hex_str + '\n',
                                capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            return None
        output = result.stdout.strip()
        if not output:
            return None
        # Output format: \t<mnemonic>\t<operands>
        # Strip leading tab for CHECK comparison.
        if output.startswith('\t'):
            output = output[1:]
        return output
    except (subprocess.TimeoutExpired, OSError):
        return None


def convert_file(input_path, output_path, run_line, llvm_mc, triple, mcpu=None):
    """Convert a disassembler test data file to a lit test file."""
    entries = []

    with open(input_path, 'r') as f:
        for line in f:
            result = parse_line(line)
            if result:
                entries.append(result)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    written = 0
    errors = 0

    with open(output_path, 'w') as f:
        f.write(f'{run_line}\n')
        f.write('\n')

        for hex_str, mnemonic in entries:
            disasm = get_disasm_output(hex_str, llvm_mc, triple, mcpu)
            if disasm:
                f.write(f'# CHECK: {disasm}\n')
                f.write(f'{hex_str}\n')
                f.write('\n')
                written += 1
            else:
                f.write(f'# SKIP (disassembler error): {hex_str}\n')
                f.write('\n')
                errors += 1

    print(f'Generated {output_path} ({written} instructions, {errors} errors)')


def main():
    if len(sys.argv) > 1:
        llvm_mc = sys.argv[1]
    else:
        candidates = [
            os.path.join(REPO_ROOT, 'cmake-build-debug-system', 'bin', 'llvm-mc'),
            os.path.join(REPO_ROOT, 'build', 'bin', 'llvm-mc'),
        ]
        llvm_mc = None
        for c in candidates:
            if os.path.isfile(c):
                llvm_mc = c
                break
        if not llvm_mc:
            print(f'Error: llvm-mc not found. Tried: {candidates}', file=sys.stderr)
            sys.exit(1)

    print(f'Using llvm-mc: {llvm_mc}')

    run_6809 = '# RUN: llvm-mc -triple=mc6809 -disassemble %s | FileCheck %s'
    run_6309 = '# RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -disassemble %s | FileCheck %s'

    convert_file(INPUT_6809, OUTPUT_6809, run_6809, llvm_mc, 'mc6809')
    convert_file(INPUT_6309, OUTPUT_6309, run_6309, llvm_mc, 'mc6809', mcpu='hd6309')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
cmoc2gas.py — Convert CMOC Motorola-style 6809 assembly to GAS syntax for llvm-mc.

Strips CMOC startup code (SECTION start) and library imports (INILIB, _exit).
Only emits the code section functions, suitable for linking with our runtime.inc.

Usage: cmoc2gas.py < input.s > output.s
"""

import re
import sys

# Sections to skip entirely (CMOC runtime, not needed with our runtime.inc)
SKIP_SECTIONS = {'start', 'initgl_start', 'initgl', 'initgl_end', 'program_end'}

def convert(lines):
    in_skip_section = False
    output = []

    for line in lines:
        stripped = line.rstrip()

        # Track sections to skip
        m = re.match(r'\s+SECTION\s+(\w+)', stripped, re.IGNORECASE)
        if m:
            name = m.group(1)
            if name in SKIP_SECTIONS:
                in_skip_section = True
                continue
            in_skip_section = False
            if name == 'bss':
                output.append('\t.section .bss')
            else:
                output.append('\t.section .rom,"ax",@progbits')
            continue

        if in_skip_section:
            continue

        # ENDSECTION / END — skip
        if re.match(r'\s+ENDSECTION\s*$', stripped, re.IGNORECASE):
            continue
        if re.match(r'\s+END\s*$', stripped, re.IGNORECASE):
            continue

        # Empty lines
        if not stripped:
            output.append('')
            continue

        # Comment-only lines: * → ;
        if stripped.startswith('*'):
            output.append(';' + stripped[1:])
            continue

        # label EXPORT → .globl label
        m = re.match(r'(\w+)\s+EXPORT\s*$', stripped, re.IGNORECASE)
        if m:
            output.append(f'\t.globl\t{m.group(1)}')
            continue

        # label IMPORT → skip
        m = re.match(r'(\w+)\s+IMPORT\s*$', stripped, re.IGNORECASE)
        if m:
            continue

        # label EQU * [comment] → label:
        m = re.match(r'([\w.]+)\s+EQU\s+\*(.*)', stripped, re.IGNORECASE)
        if m:
            output.append(f'{m.group(1)}:')
            continue

        # label EQU expr → .set label, expr
        m = re.match(r'([\w.]+)\s+EQU\s+(.+)', stripped, re.IGNORECASE)
        if m:
            output.append(f'\t.set\t{m.group(1)}, {m.group(2)}')
            continue

        # Indented line = instruction
        m = re.match(r'\t(.+)', stripped)
        if m:
            content = m.group(1)
            # CMOC format: MNEMONIC\tOPERAND\tCOMMENT (tab-separated fields)
            fields = content.split('\t')
            mnemonic = fields[0].strip().lower()
            operand = fields[1].strip() if len(fields) > 1 else ''
            comment = ('\t; ' + ' '.join(f.strip() for f in fields[2:])) if len(fields) > 2 else ''
            # PCR → pc (CMOC PC-relative addressing)
            operand = re.sub(r',PCR\b', ',pc', operand)
            # Translate Motorola pseudo-ops to GAS equivalents
            if mnemonic == 'fcc':
                instr_part = f'.ascii\t{operand}'
            elif mnemonic == 'fcb':
                instr_part = f'.byte\t{operand}'
            elif mnemonic == 'fdb':
                instr_part = f'.word\t{operand}'
            elif mnemonic == 'rmb':
                instr_part = f'.space\t{operand}'
            else:
                instr_part = f'{mnemonic}\t{operand}' if operand else mnemonic
            output.append(f'\t{instr_part}{comment}')
            continue

        # Non-indented with content (label + instruction on same line)
        m = re.match(r'([\w.]+)\s+(.*)', stripped)
        if m:
            label = m.group(1)
            rest = m.group(2).strip()
            if rest:
                output.append(f'{label}:\t{rest.lower()}')
            else:
                output.append(f'{label}:')
            continue

        output.append(stripped)

    return output


def main():
    lines = sys.stdin.readlines()
    for line in convert(lines):
        print(line)


if __name__ == '__main__':
    main()

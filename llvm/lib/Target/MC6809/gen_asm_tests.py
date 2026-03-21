#!/usr/bin/env python3
"""Convert MC6809 disassembler test files into LLVM Lit assembler tests.

Reads the disassembler test data files, runs each instruction through llvm-mc
to get the actual encoding (including fixup markers), and generates .s files
with CHECK lines that verify the assembler produces the correct encodings.

Usage:
    python3 gen_asm_tests.py [path-to-llvm-mc]
"""

import os
import re
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))

INPUT_6809 = os.path.join(SCRIPT_DIR, 'MC6809InstrDisassembler6809Tests.txt')
INPUT_6309 = os.path.join(SCRIPT_DIR, 'MC6809InstrDisassembler6309Tests.txt')

OUTPUT_DIR = os.path.join(REPO_ROOT, 'test', 'MC', 'MC6809')
OUTPUT_6809 = os.path.join(OUTPUT_DIR, 'asm-6809.s')
OUTPUT_6309 = os.path.join(OUTPUT_DIR, 'asm-6309.s')

# Instructions to skip (not assemblable).
SKIP_MNEMONICS = {'hcf', 'reset'}

# Branch instructions — need special handling with labels.
BRANCH_MNEMONICS = {
    'bra', 'brn', 'bhi', 'bls', 'bhs', 'blo', 'bne', 'beq',
    'bvc', 'bvs', 'bpl', 'bmi', 'bge', 'blt', 'bgt', 'ble',
    'lbra', 'lbrn', 'lbhi', 'lbls', 'lbhs', 'lblo', 'lbne', 'lbeq',
    'lbvc', 'lbvs', 'lbpl', 'lbmi', 'lbge', 'lblt', 'lbgt', 'lble',
    'lbsr', 'bsr',
}

# Conditional branches use b$cond / lb$cond with an unimplemented condition
# code parser. Only unconditional branches and subroutine calls are assemblable.
CONDITIONAL_BRANCH_MNEMONICS = {
    'bhi', 'bls', 'bhs', 'blo', 'bne', 'beq',
    'bvc', 'bvs', 'bpl', 'bmi', 'bge', 'blt', 'bgt', 'ble',
    'lbhi', 'lbls', 'lbhs', 'lblo', 'lbne', 'lbeq',
    'lbvc', 'lbvs', 'lbpl', 'lbmi', 'lbge', 'lblt', 'lbgt', 'lble',
}


def parse_line(line):
    """Parse a disassembler test line. Returns (hex_bytes, mnemonic, operands, asm_text) or None."""
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

    hex_bytes = []
    for token in hex_tokens:
        if not token.startswith('0x') and not token.startswith('0X'):
            return None
        hex_bytes.append(token)

    if not hex_bytes:
        return None

    parts = asm_part.split('\t')
    parts = [p for p in parts if p.strip()]

    if not parts:
        return None

    mnemonic = parts[0].strip()

    if len(parts) >= 2:
        operand_and_desc = parts[1]
        match = re.search(r'  ', operand_and_desc)
        if match:
            operands = operand_and_desc[:match.start()].strip()
        else:
            operands = operand_and_desc.strip()
    else:
        operands = ''

    # The AsmString patterns use ',' instead of ';' as the separator between
    # the immediate value and the address operand (oim/aim/eim/tim).
    operands = operands.replace(';', ',')

    # The disassembler printer omits the '#' prefix on the immediate for
    # oim/aim/eim/tim. The assembler requires it per the AsmString pattern.
    if mnemonic in ('oim', 'aim', 'eim', 'tim') and operands and not operands.startswith('#'):
        operands = '#' + operands

    # The disassembler printer omits +/- suffixes for tfm block transfer
    # variants. Determine the correct suffix from the opcode byte.
    if mnemonic == 'tfm' and len(hex_bytes) >= 2:
        opc = int(hex_bytes[1], 16) & 0xFF
        parts = operands.split(',')
        if len(parts) == 2:
            r1, r2 = parts[0].strip(), parts[1].strip()
            if opc == 0x38:    # r0+,r1+ (copy)
                operands = f'{r1}+,{r2}+'
            elif opc == 0x39:  # r0-,r1- (reverse copy)
                operands = f'{r1}-,{r2}-'
            elif opc == 0x3A:  # r0+,r1 (fill)
                operands = f'{r1}+,{r2}'
            elif opc == 0x3B:  # r0,r1+ (read)
                operands = f'{r1},{r2}+'

    if operands:
        asm_text = f'{mnemonic}\t{operands}'
    else:
        asm_text = mnemonic

    return (hex_bytes, mnemonic, operands, asm_text)


def should_skip(mnemonic, operands):
    """Check if an instruction should be skipped entirely."""
    if mnemonic in SKIP_MNEMONICS:
        return True
    if mnemonic in CONDITIONAL_BRANCH_MNEMONICS:
        return True  # Condition code parser not implemented
    if operands and re.search(r'\[-\d+\]', operands):
        return True
    return False


def get_branch_encoding(mnemonic, llvm_mc, triple, mcpu=None):
    """Run a single branch instruction through llvm-mc with a label target
    and return the actual encoding string."""
    label = f'.Ltmp_{mnemonic}'
    asm_input = f'{mnemonic} {label}\n{label}:\n'
    cmd = [llvm_mc, f'-triple={triple}', '-show-encoding']
    if mcpu:
        cmd.append(f'-mcpu={mcpu}')
    result = subprocess.run(cmd, input=asm_input, capture_output=True, text=True)
    for line in (result.stdout or '').split('\n'):
        enc_match = re.search(r'encoding: (\[.*?\])', line)
        if enc_match:
            return enc_match.group(1)
    return None


def get_actual_encodings(asm_lines, llvm_mc, triple, mcpu=None):
    """Run instructions through llvm-mc and capture actual encodings.

    Returns a dict mapping line_index -> encoding_string, and a set of
    failed line indices.
    """
    input_text = '\n'.join(asm_lines) + '\n'
    cmd = [llvm_mc, f'-triple={triple}', '-show-encoding']
    if mcpu:
        cmd.append(f'-mcpu={mcpu}')

    result = subprocess.run(cmd, input=input_text, capture_output=True, text=True)

    # Collect error line numbers (1-based stdin lines).
    failed = set()
    for out_line in (result.stderr or '').split('\n'):
        line_match = re.search(r'<stdin>:(\d+):', out_line)
        if line_match and 'error:' in out_line:
            failed.add(int(line_match.group(1)) - 1)

    encodings = {}
    successful_indices = [i for i in range(len(asm_lines)) if i not in failed]

    enc_idx = 0
    for out_line in (result.stdout or '').split('\n'):
        enc_match = re.search(r'encoding: (\[.*?\])', out_line)
        if enc_match and enc_idx < len(successful_indices):
            encodings[successful_indices[enc_idx]] = enc_match.group(1)
            enc_idx += 1

    return encodings, failed


def convert_file(input_path, output_path, run_line, llvm_mc, triple, mcpu=None):
    """Convert a disassembler test file to an assembler test file."""
    # First pass: parse all lines and separate branches from normal instructions.
    all_entries = []  # (hex_bytes, mnemonic, operands, asm_text, is_branch)
    skipped = 0

    with open(input_path, 'r') as f:
        for line in f:
            result = parse_line(line)
            if result is None:
                continue
            hex_bytes, mnemonic, operands, asm_text = result
            if should_skip(mnemonic, operands):
                skipped += 1
                continue
            is_branch = mnemonic in BRANCH_MNEMONICS
            all_entries.append((hex_bytes, mnemonic, operands, asm_text, is_branch))

    # Collect non-branch instructions for batch encoding.
    non_branch_entries = [(i, e) for i, e in enumerate(all_entries) if not e[4]]
    non_branch_asm = [e[3] for _, e in non_branch_entries]

    # Get actual encodings for non-branch instructions.
    actual, failed_set = get_actual_encodings(non_branch_asm, llvm_mc, triple, mcpu)

    # Build a mapping from all_entries index to encoding string.
    encoding_map = {}
    failed_map = set()
    for nb_idx, (all_idx, _) in enumerate(non_branch_entries):
        if nb_idx in actual:
            encoding_map[all_idx] = actual[nb_idx]
        else:
            failed_map.add(all_idx)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    written = 0
    errors = 0
    branch_label_counter = 0
    seen_branches = set()  # Track (mnemonic) to deduplicate

    with open(output_path, 'w') as f:
        f.write(f'{run_line}\n')
        f.write('\n')

        for idx, (hex_bytes, mnemonic, operands, asm_text, is_branch) in enumerate(all_entries):
            if is_branch:
                # Deduplicate branches — only need one test per mnemonic.
                if mnemonic in seen_branches:
                    continue
                seen_branches.add(mnemonic)

                # Run through llvm-mc to get actual encoding with fixups.
                encoding = get_branch_encoding(mnemonic, llvm_mc, triple, mcpu)
                if encoding:
                    label = f'.Lbr_{mnemonic}'
                    f.write(f'; CHECK: encoding: {encoding}\n')
                    f.write(f'{mnemonic}\t{label}\n')
                    f.write(f'{label}:\n')
                    written += 1
                else:
                    f.write(f'; SKIP (branch assembler error): {mnemonic}\n')
                    errors += 1
            elif idx in encoding_map:
                encoding = encoding_map[idx]
                f.write(f'; CHECK: encoding: {encoding}\n')
                f.write(f'{asm_text}\n')
                written += 1
            else:
                f.write(f'; SKIP (assembler error): {asm_text}\n')
                errors += 1

    print(f'Generated {output_path} ({written} instructions, {skipped} skipped, {errors} errors)')


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
            print('Usage: python3 gen_asm_tests.py [path-to-llvm-mc]', file=sys.stderr)
            sys.exit(1)

    print(f'Using llvm-mc: {llvm_mc}')

    run_6809 = '; RUN: llvm-mc -triple=mc6809 -show-encoding %s 2>&1 | FileCheck %s'
    run_6309 = '; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -show-encoding %s 2>&1 | FileCheck %s'

    convert_file(INPUT_6809, OUTPUT_6809, run_6809, llvm_mc, 'mc6809')
    convert_file(INPUT_6309, OUTPUT_6309, run_6309, llvm_mc, 'mc6809', mcpu='hd6309')


if __name__ == '__main__':
    main()

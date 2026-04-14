//===-- MC6809CallingConv.td - Calling Conventions MC6809 --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MC6809 calling convention, matching gcc6809 (m6809-unknown-gcc).
// See ~/.claude/shared-mc6809/gcc6809-calling-convention.md for the
// canonical specification — derived empirically and cross-checked
// against gcc-4.6.4/gcc/config/m6809/{m6809.h,m6809.c}.
//
// Summary (functions returning ≤ 2 bytes):
//   - First i8 arg in B, first i16/ptr arg in IX
//   - All other args (including ALL i32+) on stack, big-endian
//   - i8 stack slots are 1 byte (NOT padded to 2)
//   - i8 return in B, i16/ptr return in IX
//   - Callee-saved: U (frame pointer), Y. Caller-saved: A, B, X, CC
//
// Summary (functions returning > 2 bytes — i32, structs, doubles):
//   - IX = sret pointer (caller-allocated buffer for the return value)
//   - First i8 arg in B (B is independent of the IX sret reservation)
//   - All other args on stack — even the first i16/ptr, since IX
//     is taken by the sret pointer
//   - Callee writes the result through IX (hi at IX+0, lo at IX+2)
//   - The sret transformation lives in MC6809CallLowering — these
//     CC tables only see the synthetic ptr arg in IX, the rest of
//     the user args naturally fall through to the stack rules
//
// Variadic functions (printf, etc.):
//   - ALL args on stack, including named ones (CC_MC6809_VarArgs)
//
//===----------------------------------------------------------------------===//

/// CCIfSubtarget - Match if the current subtarget has a feature F.
class CCIfSubtarget<string F, CCAction A>
    : CCIf<!strconcat("static_cast<const MC6809Subtarget&>"
                       "(State.getMachineFunction().getSubtarget()).", F), A>;
class CCIfNotSubtarget<string F, CCAction A>
    : CCIf<!strconcat("!static_cast<const MC6809Subtarget&>"
                       "(State.getMachineFunction().getSubtarget()).", F), A>;

//===----------------------------------------------------------------------===//
// Return Value Calling Convention
//===----------------------------------------------------------------------===//

// Return value convention (gcc6809-compatible).
//
// i8 → B, i16/ptr → IX. Anything larger (i32, structs, …) is
// returned via the gcc6809 sret convention: caller passes a buffer
// pointer in IX, callee writes through it. The sret transformation
// is performed in MC6809CallLowering::lowerReturn / ::lowerCall
// BEFORE this CC table is consulted, so RetCC_MC6809 here only ever
// sees i1/i8/i16/ptr return values — never i32.
let Entry = 1 in
def RetCC_MC6809 : CallingConv<[
  CCIfType<[i1],  CCPromoteToType<i8>>,
  CCIfType<[i8],  CCAssignToReg<[AB]>>,
  CCIfPtr<CCAssignToReg<[IX]>>,
  CCIfType<[i16], CCAssignToReg<[IX]>>,
  CCAssignToStack<0, 1>,
]>;

//===----------------------------------------------------------------------===//
// Argument Calling Convention
//===----------------------------------------------------------------------===//

// gcc6809 argument convention.
//
//   - First 8-bit arg in B (always — even when sret is active)
//   - First 16-bit/ptr arg in IX (unless IX is taken by an sret
//     pointer prepended by MC6809CallLowering for long-returning
//     callees)
//   - i8 args use 1-byte stack slots (NOT padded to a word)
//   - All i32+ args go on the stack, regardless of register
//     availability. Since splitToValueTypes splits an i32 into
//     two i16 pieces before this table is consulted, we use the
//     CCIfSplit / CCIfSplitEnd flags to detect those pieces and
//     force them to the stack — otherwise the first i16 piece
//     would land in IX and break ABI compatibility with gcc6809
//     for void- or short-returning functions that take i32 args.
//   - Callee-saved: U (frame pointer), Y
let Entry = 1 in
def CC_MC6809 : CallingConv<[
  CCIfPtr<CCAssignToReg<[IX]>>,
  CCIfType<[i1],  CCPromoteToType<i8>>,
  CCIfType<[i8],  CCAssignToReg<[AB]>>,
  CCIfType<[i8],  CCAssignToStack<1, 1>>,
  // Pieces of a split i32 (or larger) → always stack. The first
  // piece carries ArgFlags.isSplit() and the last piece carries
  // ArgFlags.isSplitEnd(). For i32 → 2*i16 there are no middle
  // pieces; for i64+ middle pieces have neither flag set and need
  // an additional rule (not yet exercised by any test).
  CCIfSplit<CCAssignToStack<2, 1>>,
  CCIf<"ArgFlags.isSplitEnd()", CCAssignToStack<2, 1>>,
  CCIfType<[i16], CCAssignToReg<[IX]>>,
  CCAssignToStack<0, 1>,
]>;

// Calling convention for the variable section of a variadic function call.
// Named arguments in such functions still use the above calling convention.
let Entry = 1 in
def CC_MC6809_VarArgs : CallingConv<[
  CCAssignToStack<0, 1>,
]>;

// Callee-saved registers: U (frame pointer) and Y.
// A, B, X, CC are caller-saved.
def MC6809_CSR : CalleeSavedRegs<(add IY, SU)>;

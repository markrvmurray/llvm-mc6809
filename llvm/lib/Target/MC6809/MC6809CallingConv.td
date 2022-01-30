//===-- MC6809CallingConv.td - Calling Conventions MC6809 --------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the MC6809 architecture.
//
//===----------------------------------------------------------------------===//
//
// The C calling convention is mostly described inline. There are a few
// additional points worth mentioning:
//
// - A, X, Y, C, N, V, Z, and RS1-RS9 are caller-saved. PC, S, D, I, SS, and
//   RS10-RS15 are callee-saved.
//
// - Aggregate types (structs, arrays, etc.) are passed by pointer. The pointer
//   is managed entirely by the caller, and may or may not be on the soft stack.
//   The callee is free to write to the memory; the caller must consider the
//   memory overwritten by the call. This is handled directly by Clang; LLVM
//   itself should never see aggregates.
//
// - Aggregate types are returned by a pointer passed as an implicit first
//   argument. The resulting function returns void. This is handled directly by
//   Clang; LLVM itself should never see aggregates.
//
// - Variable arguments (those within the ellipses of the argument list) are
//   passed through the stack. Named arguments before the variable arguments are
//   passed as usual: first in registers, then stack. Note that the variable
//   argument and regular calling convention differ; thus, variable argument
//   functions must only be called if prototyped. The C standard requires this,
//   but many platforms do not; their variable argument and regular calling
//   conventions are identical. A notable exception is Apple ARM64.
//
// - Values may be returned on the soft stack if insufficiently many registers
//   are available. Callers must reserve sufficient space for this as they do
//   for arguments. The space reserved for arguments may overlap freely with the
//   space used for return values; thus only enough space for the larger of the
//   two need be allocated.
//
// This calling convention was roughly based on that of RISC-V.
//
// For insight into the design of performant calling conventions, see the
// following work by Davidson and Whalley. By their convention, this plaftorm
// uses the "smarter hybrid" method, since LLVM performs both shrink wrapping
// and caller save-restore placement optimizations, while using both
// callee-saved and caller-saved registers when appropriate.
//
// Methods for Saving and Restoring Register Values across Function Calls
// Software--Practice and Experience Vol 21(2), 149-165 (February 1991)
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.14.4669&rep=rep1&type=pdf
//
//===----------------------------------------------------------------------===//

/// CCIfSubtarget - Match if the current subtarget has a feature F.
class CCIfSubtarget<string F, CCAction A>
    : CCIf<!strconcat("static_cast<const MC6809Subtarget&>"
                       "(State.getMachineFunction().getSubtarget()).",
                     F),
          A>;

class CCIfNotSubtarget<string F, CCAction A>
    : CCIf<!strconcat("!static_cast<const MC6809Subtarget&>"
                       "(State.getMachineFunction().getSubtarget()).",
                     F),
          A>;

let Entry = 1 in {
def CC_MC6809 : CallingConv<[
//  // 1- and 8-bit values are promited to 16-bit.
//  CCIfType<[i1, i8], CCPromoteToType<i16>>,

  // 8-bit values are assigned to B, A, F, E for HD6309
  CCIfType<[i8], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AB, AA, AF, AE]>>>,

  // 8-bit values are assigned to B, A only for MC6809
  CCIfType<[i8], CCIfNotSubtarget<"isHD6309()", CCAssignToReg<[AB, AA]>>>,

  // 16-bit values are assigned to D, W, then X, Y for HD6309
  CCIfType<[i16], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AD, AW, IX, IY]>>>,

  // 16-bit values are assigned to D, then X, Y for MC6809
  CCIfType<[i16], CCIfNotSubtarget<"isHD6309()", CCAssignToReg<[AD, IX, IY]>>>,

  // 32-bit values are assigned to Q for HD6309
  CCIfType<[i32], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AQ]>>>,

  // Pointers are preferentially assigned to X, Y so indirect
  // addressing modes work without additional copying.
  CCIfPtr<CCAssignToReg<[IX, IY]>>,

  // Values that wouldn't fit anywhere else are passed directly on the stack.
  CCAssignToStack<0, 1>,
]>;

// Calling convention for the variable section of a variadic function call.
// Named arguments in such functions still use the above calling convention.
def CC_MC6809_VarArgs : CallingConv<[
  // All values are passed directly on the stack.
  CCAssignToStack<0, 1>,
]>;
}

def MC6809_CSR : CalleeSavedRegs<(add SU)>;

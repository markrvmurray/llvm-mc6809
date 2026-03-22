//===-- MC6809CallingConv.td - Calling Conventions MC6809 --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MC6809 calling convention, compatible with CMOC's __gcccall convention:
//   - First 8-bit arg in B, first 16-bit/ptr arg in X
//   - Remaining args as 16-bit words on stack (right-to-left, caller cleanup)
//   - 8-bit return in B, 16-bit return in X
//   - Callee-saved: U (frame pointer), Y
//   - Caller-saved: A, B, X, CC
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

// Return value convention.
// Stage 1: i8 in B, i16 in D (TODO: move i16 to X for full CMOC GCC compat).
let Entry = 1 in
def RetCC_MC6809 : CallingConv<[
  CCIfType<[i1],  CCPromoteToType<i8>>,
  CCIfType<[i8],  CCAssignToReg<[AB]>>,
  CCIfPtr<CCAssignToReg<[IX]>>,
  CCIfType<[i16], CCAssignToReg<[AD]>>,
  CCIfType<[i32], CCIfSubtarget<"has6309()", CCAssignToReg<[AQ]>>>,
  CCAssignToStack<0, 1>,
]>;

//===----------------------------------------------------------------------===//
// Argument Calling Convention
//===----------------------------------------------------------------------===//

// Stage 1 toward CMOC GCC convention:
//   - First 8-bit arg in B, first 16-bit/ptr arg in D (TODO: X)
//   - Remaining args as 16-bit words on stack (2-byte aligned)
//   - Callee-saved: U, Y
let Entry = 1 in
def CC_MC6809 : CallingConv<[
  CCIfPtr<CCAssignToReg<[IX]>>,
  CCIfType<[i1],  CCPromoteToType<i8>>,
  CCIfType<[i8],  CCAssignToReg<[AB]>>,
  CCIfType<[i16], CCAssignToReg<[AD]>>,
  CCIfType<[i32], CCIfSubtarget<"has6309()", CCAssignToReg<[AQ]>>>,
  // TODO: promote i8 stack args to i16 words for full CMOC GCC compat.
  // Requires CallLowering changes to handle promotion on load.
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

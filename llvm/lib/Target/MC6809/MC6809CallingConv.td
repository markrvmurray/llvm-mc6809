//===-- MC6809CallingConv.td - Calling Conventions MC6809 --------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the MC6809
// architectures.
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

let Entry = 1 in
def RetCC_MC6809 : CallingConv<[
  CCIfPtr<CCAssignToReg<[IX, IY]>>,
  CCIfType<[i1], CCPromoteToType<i8>>,
  CCIfType<[i8], CCIfNotSubtarget<"isHD6309()", CCAssignToReg<[AB, AA]>>>,
  CCIfType<[i8], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AB, AA, AF, AE]>>>,
  CCIfType<[i16], CCIfNotSubtarget<"isHD6309()", CCAssignToReg<[AD, IX, IY]>>>,
  CCIfType<[i16], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AD, AW, IX, IY]>>>,
  CCIfType<[i32], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AQ]>>>,
  CCAssignToStack<0, 1>,
]>;

// Simple calling convention for MC6809 ELF
let Entry = 1 in
def CC_MC6809 : CallingConv<[
  CCIfPtr<CCAssignToReg<[IX, IY]>>,
  CCIfType<[i1],  CCPromoteToType<i8>>,
  CCIfType<[i8], CCIfNotSubtarget<"isHD6309()", CCAssignToReg<[AB, AA]>>>,
  CCIfType<[i8], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AB, AA, AF, AE]>>>,
  CCIfType<[i16], CCIfNotSubtarget<"isHD6309()", CCAssignToReg<[AD, IX, IY]>>>,
  CCIfType<[i16], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AD, AW, IX, IY]>>>,
  CCIfType<[i32], CCIfSubtarget<"isHD6309()", CCAssignToReg<[AQ]>>>,
  CCAssignToStack<0, 1>,
]>;

// Calling convention for the variable section of a variadic function call.
// Named arguments in such functions still use the above calling convention.
let Entry = 1 in
def CC_MC6809_VarArgs : CallingConv<[
  CCAssignToStack<0, 1>,
]>;

def MC6809_CSR : CalleeSavedRegs<(add)>;
//def MC6809_CSR : CalleeSavedRegs<(add SU)>;
//def MC6809_CSR : CalleeSavedRegs<(add AD, AW, IX, IY)>;

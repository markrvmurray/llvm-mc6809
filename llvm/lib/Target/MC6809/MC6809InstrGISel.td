//===-- MC6809InstrGISel.td - MC6809 Generic Instructions --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MC6809 Generic Instruction Definitions.
//
// These generic instructions capture aspects of the abstract semantics of 6502
// instructions. Often, an existing generic opcode contains more functionality
// than any one 6502 instruction can actually perform. In such cases, the
// legalizer lowers these constructs into a combination of other generic opcodes
// and the target-specific ones here.
//
// Note that a 6502 instruction may map to some combination of generic opcodes;
// that's fine. The problem occurs whenever a generic instruction would map to a
// large sequence of 6502 instructions. Handling those cases in the instruction
// selector, can't take advantage of the superior optimization opportunities
// afforded by the legalizer, GlobalISel CSE, and post-legalizer combiner
// passes.
//
//===----------------------------------------------------------------------===//

class MC6809GenericInstruction : GenericInstruction {
  let Namespace = "MC6809";
  let hasSideEffects = false;
}

// Generalized load using the absolute addressing mode. The address
// argument isn't a pointer; it's either a global value or a 16-bit immediate.
// Additionally expects a MachineMemOperand.
def G_LOAD_ABS : MC6809GenericInstruction {
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins unknown:$addr);
  let mayLoad = true;
}

// Generalized load using the indexed addressing mode. The index argument is a pointer.
// The offset argument is a scalar. Additionally expects a MachineMemOperand.
def G_LOAD_INDEX_OFFSET : MC6809GenericInstruction {
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins ptype1:$index, type1:$offset);
  let mayLoad = true;
}

// Generalized load using the indirect indexed addressing mode. The index argument is a pointer.
// The offset argument is a scalar. Additionally expects a MachineMemOperand.
def G_LOAD_INDIR_INDEX_OFFSET : MC6809GenericInstruction {
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins ptype1:$index, type1:$offset);
  let mayLoad = true;
}

// Generalized store using the absolute addressing mode. The address
// argument isn't a pointer; it's either a global value or a 16-bit immediate.
// Additionally expects a MachineMemOperand.
def G_STORE_ABS : MC6809GenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, unknown:$addr);
  let hasSideEffects = false;
  let mayStore = true;
}

// Generalized store using the indexed addressing mode. The index argument is a pointer.
// The offset argument is a scalar. Additionally expects a MachineMemOperand.
def G_STORE_INDEX_OFFSET : MC6809GenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, ptype1:$index, type1:$offset);
  let hasSideEffects = false;
  let mayStore = true;
}

// Generalized store using the indirect indexed addressing mode. The index argument is a pointer.
// The offset argument is a scalar. Additionally expects a MachineMemOperand.
def G_STORE_INDIR_INDEX_OFFSET : MC6809GenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, ptype1:$index, type1:$offset);
  let hasSideEffects = false;
  let mayStore = true;
}

// Shift an 8-bit value left one bit, placing $carry_in in the LSB and placing
// the previous MSB bit into $carry_out. This "even" version can be chained to
// perform multi-byte shifts.
//def G_SHLE : MC6809GenericInstruction {
//  let OutOperandList = (outs type0:$dst, type1:$carry_out);
//  let InOperandList = (ins type0:$src, type1:$carry_in);
//}

// Logically shift an 8-bit value right one bit, placing $carry_in in the MSB
// and placing the previous LSB bit into $carry_out. This "even" version can be
// chained to perform multi-byte shifts.
//def G_LSHRE : MC6809GenericInstruction {
//  let OutOperandList = (outs type0:$dst, type1:$carry_out);
//  let InOperandList = (ins type0:$src, type1:$carry_in);
//}

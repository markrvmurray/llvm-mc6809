//===-- MC6809InstrPseudos.td - Describe MC6809 Pseudo-Instrs ---*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes the MC6809 pseudo-instructions.
//
// Pseudo-instructions allow deferring generation of specific instruction
// sequences from one pass until a later pass. This is useful whenever an early
// pass is required to make a decision that it doesn't have the information
// necessary to do well.
//
//===---------------------------------------------------------------------===//

class MC6809Pseudo : Instruction {
  let Namespace = "MC6809";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
}

//===---------------------------------------------------------------------===//
// Post RA Pseudos
//===---------------------------------------------------------------------===//
// These pseudoinstructions defer decisionmaking until after register
// allocation. They're used whenever the best instruction sequence to emit
// depends only on the placement of their arguments and the set of live
// registers before and after the pseudo.
//===---------------------------------------------------------------------===//

// Abstracts LDAZpIdx and LDYIdx. LDXIdx isn't yet supported, since there's no
// good way to save and restore $x on a NMC6809 6502 when lowering $a = LDZpIdx
// $y.
//def LDZpIdx : MC6809Pseudo {
//  dag OutOperandList = (outs AY:$dst);
//  dag InOperandList = (ins i8imm:$addr, Xc:$idx);
//
//  let isReMaterializable = true;
//  let mayLoad = true;
//}

// Abstracts LDAAbsIdx, LDXIdx, and LDYIdx. This is almost a logical instruction,
// but the "LDX 1234,X" and "LDY 1234,Y" cases aren't natively supported, and
// more than one instruction is required to emit them.
//def LDAbsIdx : MC6809Pseudo {
//  dag OutOperandList = (outs GPR:$dst);
//  dag InOperandList = (ins i16imm:$addr, XY:$idx);
//
//  let isReMaterializable = true;
//  let mayLoad = true;
//}

//// Loads a boolean value into C, V or any accumulator's LSB
//def LDImm1 : MC6809Pseudo {
//  dag OutOperandList = (outs BIT1:$dst);
//  dag InOperandList = (ins i1imm:$val);
//
//  let isAsCheapAsAMove = true;
//  let isMoveImm = true;
//  let isReMaterializable = true;
//}

// Loads a 16-bit value into an imaginary pointer pair. This is generally more
// constrained than loading two 8-bit values using a REG_SEQUENCE, but bundling
// this functionality into a single pseudo-instruction allows thee register
// allocator to rematerialize the load at point of use. This is particularly
// important for immediate loads, since GlobalISel places these at the beginning
// of basic blocks, which maximizes the chance that they'll be live across
// calls. Without rematerialization, all these loads would be placed in CSRs,
// for massive spill/reload overhead.
//def LDImm16 : MC6809Pseudo {
//  dag OutOperandList = (outs Imag16:$dst, ALL16:$scratch);
//  dag InOperandList = (ins i16imm:$val);
//  let Constraints = "@earlyclobber $scratch";
//
//  let isAsCheapAsAMove = true;
//  let isMoveImm = true;
//  let isReMaterializable = true;
//}

// Version of LDImm16 that doesn't use a scratch register, which is possible
// on the SPC700 architecture.
//def LDImm16SPC700 : MC6809Pseudo {
//  let Predicates = [HasSPC700];
//  dag OutOperandList = (outs Imag16:$dst);
//  dag InOperandList = (ins i16imm:$val);
//
//  let isAsCheapAsAMove = true;
//  let isMoveImm = true;
//  let isReMaterializable = true;
//}

// Version of LDImm16 that doesn't use a scratch register, to satisfy the
// requirements for rematerialization. Instead, the register scavenger locates a
// scratch register at expansion time.
//def LDImm16Remat : MC6809Pseudo {
//  dag OutOperandList = (outs Imag16:$dst);
//  dag InOperandList = (ins i16imm:$val);
//
//  let isAsCheapAsAMove = true;
//  let isMoveImm = true;
//  let isReMaterializable = true;
//}

// Load zero into GPR.
//def LDZ : MC6809Pseudo {
//  dag OutOperandList = (outs Anyi8:$dst);
//  let Pattern = [(set Anyi8:$dst, 0)];
//
//  let isAsCheapAsAMove = true;
//  let isMoveImm = true;
//  let isReMaterializable = true;
//}

//def CmpZero : MC6809Pseudo {
//  dag InOperandList = (ins Anyi8:$val);
//  let Defs = [C];
//  let isCompare = true;
//}

// On 6502, INC/DEC cannot be directly performed on the A register;
// to do so, one has to use ADC/SBC, which affects the C and V flags.
//def IncNMC6809 : MC6809Pseudo {
//  dag OutOperandList = (outs Anyi8:$dst);
//  dag InOperandList = (ins Anyi8:$src);
//
//  let Defs = [C, V];
//
//  let Constraints = "$dst = $src";
//  let Pattern = [(set Anyi8:$dst, (add Anyi8:$src, 1))];
//  let isAdd = true;
//}

//def DecNMC6809 : MC6809Pseudo {
//  dag OutOperandList = (outs Anyi8:$dst);
//  dag InOperandList = (ins Anyi8:$src);
//
//  let Defs = [C, V];
//
//  let Constraints = "$dst = $src";
//  let Pattern = [(set Anyi8:$dst, (add Anyi8:$src, -1))];
//}

// Pointer increment/decrement.
//class MC6809IncDecPtr : MC6809Pseudo {
//  dag InOperandList = (ins Imag16:$src);
//  let Defs = [C, V];
//  let Constraints = "$dst = $src";
//}

//def IncPtr : MC6809IncDecPtr {
//  dag OutOperandList = (outs Imag16:$dst);
//  let isAdd = true;
//}

//def DecPtr : MC6809IncDecPtr {
//  dag OutOperandList = (outs GPR:$scratch, Imag16:$dst);
//  let Constraints = "$dst = $src,@earlyclobber $scratch";
//}

//def DecDcpPtr : MC6809IncDecPtr {
//  dag OutOperandList = (outs Ac:$scratch, Imag16:$dst);
//  let Constraints = "$dst = $src,@earlyclobber $scratch";
//}

//===---------------------------------------------------------------------===//
// Soft stack
//===---------------------------------------------------------------------===//

// The call-frame pseudoinstructions form a signalling mechanism to pass the
// size of outgoing call frames from calling-convention lowering (in
// IRTranslator) through to the stack frame prolog-epilog insertion. The latter
// examines these pseudoinstructions to find the largest size that an outgoing
// call frame can be, then preallocates that much space to the stack frame if
// possible. This allows these pseudoinstructions to be summarily removed in
// most cases, since we can instead guarantee that a reserved region at the
// bottom of the stack will always have enough free space to set up the
// arguments of any outgoing call.
class MC6809CallFramePseudo : MC6809Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [SS];
  let Uses = [SS];
}

// Call frame setup pseudo.
def ADJCALLSTACKDOWN : MC6809CallFramePseudo;

// Call frame destroy pseudo.
def ADJCALLSTACKUP : MC6809CallFramePseudo;

// Produces the low byte of the address of the given frame index. Produces a
// carry required by the high byte calculation.
//def AddrLostk : MC6809Pseudo {
//  dag OutOperandList = (outs Anyi8:$dst, Cc:$carry, Vc:$deadv);
//  dag InOperandList = (ins Imag16:$base, i16imm:$offset);
//
//  let isReMaterializable = true;
//}

// Produces the high byte of the address of the given frame index. Accepts the
// carry produced by the low byte calculation.
//def AddrHistk : MC6809Pseudo {
//  dag OutOperandList = (outs Anyi8:$dst, Cc:$deadc, Vc:$deadv);
//  dag InOperandList = (ins Imag16:$base, i16imm:$offset, Cc:$carry);
//
//  let isReMaterializable = true;
//}

// LDStk and STStk require an additional pointer register; if the stack access
// is far, this is used to materialize the necessary pointer. This "wastes" a
// register in common cases, but it's better to use register allocator
// mechanisms to keep a pointer register free than to scramble at the last
// second to scavenge one up.

// Loads a value (1, 8, or 16 bits) from the soft stack. Eliminated once the
// offset from the frame register is known.
//def LDStk : MC6809Pseudo {
//  dag OutOperandList = (outs unknown:$dst);
//  dag InOperandList = (ins INDEX16:$base, i16imm:$offset);
//
//  let mayLoad = true;
//}

// Stores a value (1, 8, or 16 bits) to the soft stack. Eliminated once the
// offset from the frame register is known.
//def STStk : MC6809Pseudo {
//  dag OutOperandList = (outs);
//  dag InOperandList = (ins unknown:$src, INDEX16:$base, i16imm:$fi);
//
//  let mayStore = true;
//}

//===---------------------------------------------------------------------===//
// Control flow
//===---------------------------------------------------------------------===//

// Generic conditional branch that can branch on BIT1. Branches to $tgt if $tst = $val.
//def GBR : MC6809Pseudo {
//  dag InOperandList = (ins label:$tgt, BIT1:$tst, i1imm:$val);
//
//  // This may need to lower to a compare, which clobbers C.
//  // FIXME!! MarkM: Other flags?
//  let Defs = [C];
//
//  let isBranch = true;
//  let isTerminator = true;
//}

// It's difficult for the code generator to keep N and Z free for the usual
// multi-byte increments and decrements, so these pseudos bundle arbitrary-width
// increments and decrements into single instructions. Each use corresponds to
// one byte in little-endian order. Bytes may either be registers or absolute
// addresses. For each register, there must be one tied def. Register operands
// must be constrained to Anyi8.
//class MC6809IncDecMB : MC6809Pseudo {
//  dag InOperandList = (ins variable_ops);
//  let Defs = [C, V];
//
//  let usesCustomInserter = true;
//  let mayLoad = true;
//  let mayStore = true;
//}
//def IncMB : MC6809IncDecMB {
//  dag OutOperandList = (outs variable_ops);
//}
//def DecMB : MC6809IncDecMB {
//  dag OutOperandList = (outs GPR:$scratch, variable_ops);
//  let Constraints = "@earlyclobber $scratch";
//}
// DCP-based multi-byte decrement for 6502X. DCP is significantly more compact
// than DEC + CMP, but only supports the A register as scratch.
//def DecDcpMB : MC6809IncDecMB {
//  dag OutOperandList = (outs Ac:$scratch, variable_ops);
//  let Constraints = "@earlyclobber $scratch";
//  let Predicates = [Has6502X];
//}

// Use the diamond control flow pattern to load one of two immediates based on
// the value of a BIT1/Flag.
def SelectImm : MC6809Pseudo {
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins BIT1:$tst, i1imm:$true, i1imm:$false);

  let usesCustomInserter = true;
}

// Use the diamond control flow pattern to load one of two immediates based on
// the value of a conditional - same as is used in the Bbc $cond instruction..
def ConditionalImm : MC6809Pseudo {
  dag OutOperandList = (outs BIT1:$dst);
  dag InOperandList = (ins condcode:$cc, CCond:$tst, i1imm:$true, i1imm:$false);

  let usesCustomInserter = true;
}

// Joined compare-and-branch instructions on N and Z. These flags are not
// explicitly tracked, since they're very difficult to keep alive. Accordingly,
// they are always bundled with their use, which is always a branch.
//class MC6809CmpBr : MC6809Pseudo {
//  let Defs = [C];
//
//  let isCompare = true;
//  let isBranch = true;
//  let isTerminator = true;
//}

//def CmpBrImm : MC6809CmpBr {
//  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, GPR:$l, i8imm:$r);
//}
//def CmpBrImag8 : MC6809CmpBr {
//  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, GPR:$l, Imag8:$r);
//}
//def CmpBrZero : MC6809CmpBr {
//  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, Anyi8:$val);
//}

// (ins label:$tgt, i1imm:$flag_val, Anyi8:$val...)
// Note that the flag is always Z; N only requires the first byte, so it is
// legalized to a single CMP.
//def CmpBrZeroMultiByte : MC6809CmpBr {
//  dag InOperandList = (ins variable_ops);
//  let usesCustomInserter = true;
//}
//
//let mayLoad = true in {
//  def CmpBrZpIdx : MC6809CmpBr {
//    dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, Ac:$l, i8imm:$addr, Xc:$idx);
//  }
//  def CmpBrAbs : MC6809CmpBr {
//    dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, GPR:$l, i16imm:$addr);
//  }
//  def CmpBrAbsIdx : MC6809CmpBr {
//    dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, Ac:$l, i16imm:$addr, XY:$idx);
//  }
//  def CmpBrIndir : MC6809CmpBr {
//    let Predicates = [Has65C02];
//    dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, Ac:$l, Imag16:$addr);
//  }
//  def CmpBrIndirIdx : MC6809CmpBr {
//    dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$flag_val, Ac:$l, Imag16:$addr, Yc:$idx);
//  }
//}

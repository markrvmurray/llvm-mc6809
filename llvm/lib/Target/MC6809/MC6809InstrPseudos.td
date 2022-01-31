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

// Abstracts LDAAbsIdx, LDXAbsIdx, and LDYAbsIdx. This is almost a logical instruction,
// but the "LDX 1234,X" and "LDY 1234,Y" cases aren't natively supported, and
// more than one instruction is required to emit them.
def LDAbsIdx : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins INDEX16:$idx, i16imm:$offset);

//let isReMaterializable = true;
  let mayLoad = true;
}

// Loads a boolean value into a CC bit or an ACC8 LSB
def LDImm1 : MC6809Pseudo {
  dag OutOperandList = (outs BIT1:$dst);
  dag InOperandList = (ins i1imm:$val);

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
//let isReMaterializable = true;
}

// Loads a 16-bit value into an imaginary pointer pair. This is generally more
// constrained than loading two 8-bit values using a REG_SEQUENCE, but bundling
// this functionality into a single pseudo-instruction allows thee register
// allocator to rematerialize the load at point of use. This is particularly
// important for immediate loads, since GlobalISel places these at the beginning
// of basic blocks, which maximizes the chance that they'll be live across
// calls. Without rematerialization, all these loads would be placed in CSRs,
// for massive spill/reload overhead.
//def LDImm16 : MC6809Pseudo {
//  dag OutOperandList = (outs ACC16:$dst);
//  dag InOperandList = (ins i16imm:$val);
//
//  let isAsCheapAsAMove = true;
//  let isMoveImm = true;
//  let isReMaterializable = true;
//}

// Version of LDImm16 that doesn't use a scratch register, to satisfy the
// requirements for rematerialization. Instead, the register scavenger locates a
// scratch register at expansion time.
def LDImm16Remat : MC6809Pseudo {
  dag OutOperandList = (outs ACC16:$dst);
  dag InOperandList = (ins i16imm:$val);

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
//let isReMaterializable = true;
}

// Expands to LDImm8 0 for ACC8 or STZImag8 for Imag8.
def LDZ : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst);
  let Pattern = [(set ACC8:$dst, 0)];

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
//let isReMaterializable = true;
}

def INC : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins ACC8:$src);

  let Constraints = "$dst = $src";
  let Pattern = [(set ACC8:$dst, (add ACC8:$src, 1))];
  let isAdd = true;
}

def DEC : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins ACC8:$src);

  let Constraints = "$dst = $src";
  let Pattern = [(set ACC8:$dst, (add ACC8:$src, -1))];
}

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
def AddrLostk : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst, Cc:$carry, Vc:$deadv);
  dag InOperandList = (ins ACC16:$base, i16imm:$offset);

  let isReMaterializable = true;
}

// Produces the high byte of the address of the given frame index. Accepts the
// carry produced by the low byte calculation.
def AddrHistk : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst, Cc:$deadc, Vc:$deadv);
  dag InOperandList = (ins ACC16:$base, i16imm:$offset, Cc:$carry);

  let isReMaterializable = true;
}

//===---------------------------------------------------------------------===//
// NZVC
//===---------------------------------------------------------------------===//

// These pseudoinstructions bundle the definition and use of N and Z together to
// maintain the invariant that neither can be live across pseudoinstructions. At
// most one of N or Z can be set (the other must be $noreg), since copying N out
// clobbers Z, and vice versa. If neither is set, then this represents a regular
// SBC or CMP.

class MC6809CMPNZVC : MC6809Pseudo {
  dag OutOperandList = (outs Nc:$nout, Zc:$zout, Vc:$vout, Cc:$carryout);
  let isCompare = true;
}

def CMPNZVCImm : MC6809CMPNZVC {
  dag InOperandList = (ins ACC8:$l, i8imm:$r);
}
let mayLoad = true in {
  def CMPNZVCAbs : MC6809CMPNZVC {
    dag InOperandList = (ins ACC8:$l, i16imm:$addr);
  }
  def CMPNZVCAbsIdx : MC6809CMPNZVC {
    dag InOperandList = (ins ACC8:$l, INDEX16:$idx, i16imm:$offset);
  }
  def CMPNZVCIndirIdx : MC6809CMPNZVC {
    dag InOperandList = (ins ACC8:$l, INDEX16:$idx, i16imm:$offset);
  }
}

class MC6809SBCNZVC : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst, Nc:$nout, Zc:$zout, Vc:$vout, Cc:$carryout);
  let Constraints = "$dst = $l, $carryout = $carryin";
  let isCompare = true;
}

def SBCNZVCImm : MC6809SBCNZVC {
  dag InOperandList = (ins ACC8:$l, i8imm:$r, Cc:$carryin);
}

let mayLoad = true in {
  def SBCNZVCAbs : MC6809SBCNZVC {
    dag InOperandList = (ins ACC8:$l, i16imm:$addr, Cc:$carryin);
  }
  def SBCNZVCAbsIdx : MC6809SBCNZVC {
    dag InOperandList = (ins ACC8:$l, INDEX16:$idx, i16imm:$offset, Cc:$carryin);
  }
  def SBCNZVCIndirIdx : MC6809SBCNZVC {
    dag InOperandList = (ins ACC8:$l, INDEX16:$idx, i16imm:$offset, Cc:$carryin);
  }
}

// Terminator versions of CMP. Each of these instructions is ensured to stay
// adjacent to the branch that consumes its NZVC value.
class MC6809CMPTerm : MC6809Pseudo {
  dag OutOperandList = (outs Cc:$carry);

  let Defs = [NZVC];

  let isCompare = true;
  let isTerminator = true;
}

def CMPTermImm : MC6809CMPTerm {
  dag InOperandList = (ins ACC8:$l, i8imm:$r);
}
def CMPTermZ : MC6809CMPTerm {
  dag InOperandList = (ins ACC8:$val);
}
let mayLoad = true in {
  def CMPTermAbs : MC6809CMPTerm {
    dag InOperandList = (ins ACC8:$l, i16imm:$addr);
  }
  def CMPTermIdx : MC6809CMPTerm {
    dag InOperandList = (ins ACC8:$l, INDEX16:$idx, i16imm:$offset);
  }
  def CMPTermIndir : MC6809CMPTerm {
    dag InOperandList = (ins ACC8:$l, INDEX16:$idx, i16imm:$offset);
  }
}

//===---------------------------------------------------------------------===//
// Control flow
//===---------------------------------------------------------------------===//

// Generic BR that can branch on Anyi1. Branches to $tgt if $tst = $val.
def GBR : MC6809Pseudo {
  dag InOperandList = (ins label:$tgt, BIT1:$tst, i1imm:$val);

  let isBranch = true;
  let isTerminator = true;
}

// Use the diamond control flow pattern to load one of two immediates based on
// the value of a flag. The output must either be to CV_GPR_LSB or GPR.
def SelectImm : MC6809Pseudo {
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins BIT1:$tst, i8imm:$true, i8imm:$false);

  let usesCustomInserter = true;
}

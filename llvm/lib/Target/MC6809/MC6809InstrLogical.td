//===-- MC6809InstrLogical.td - Describe MC6809 Logical Instrs --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes the MC6809 logical instruction set.
//
// When viewed logically, the 6502 instruction set is more regular than it might
// appear. For example, consider the three instructions: LDA #imm, LDX #imm, and
// LDY #imm. Each of these instructions takes a single immediate operand, loads
// one of A, X, or Y, and sets NZ based on the immediate value. Even though
// there are three separate opcodes, the only difference in their behavior is
// which register is loaded.
//
// The logical instruction set merges these instructions together into a single
// logical LDImm #imm instruction. The logical instruction takes a "ACC"
// register class as an argument. When the logical instructions are lowered to
// real instructions in AsmPrinter, the register operand is examined to
// determine which real instruction is emitted. This scheme comes with some
// considerable advantages.
//
// First, selecting a LDImm instruction doesn't constrain where the destination
// can be placed other than that it needs to be one of A, X, or Y. This allows
// earlier passes that cannot effectively make such decisions to defer them to
// the register allocator.
//
// Second, the description of the side effects and characteristics of LDImm only
// need to be specified once. Any switch() statements need to have only one
// entry for LDImm, not three for LDAImm, LDXImm, and LDYImm. This considerably
// reduces the amount of code needed to cover the 6502 instruction set.
//
// Third, exposing these regularities to LLVM passes make the 6502 more like a
// regular CISC target (X86-ish). LLVM already has very good code paths for
// dealing with this sort of architecture, and maintaining a logical instruction
// set allows us to stay within those "sane", well-tested code paths, for the
// most part.
//
// By examining its register operands, every logical instruction can be mapped
// to exactly one real target instruction. Pseudoinstructions where this
// property does not hold are recorded in MC6809InstrPseudos.td instead. These are
// more traditional pseudo-instructions; they may expand out to short snippets
// of code required to perform higher-level operations that cannot be expaneded
// inline, for one reason or another.
//
// All instructions take 1-register register classes for fixed operands: e.g.
// ACC8, INDEX16, INDEX16, etc. This allows the each instruction to natively use virtual
// registers that can be resolved by the register allocator or register
// scavenger. NZ is modeled implicitly for the most part; too many instructions
// clobber it for it to be useful outside very specific scenarios.
//
//===---------------------------------------------------------------------===//

class MC6809LogicalInstr : Instruction {
  let Namespace = "MC6809";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
  let isReMaterializable = true;
}

//===---------------------------------------------------------------------===//
// Addition/Subtraction Instructions
//===---------------------------------------------------------------------===//

class MC6809AddSub : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC8:$dst, Cc:$carryout, Vc:$vout);
  let Constraints = "$dst = $l, $carryout = $carryin";
}
class MC6809AddSubOp<Operand op> : MC6809AddSub {
  dag InOperandList = (ins ACC8:$l, op:$r, Cc:$carryin);
}
class MC6809AddSubRC<RegisterClass rc> : MC6809AddSub {
  dag InOperandList = (ins ACC8:$l, rc:$r, Cc:$carryin);
}

let isAdd = true in {
  def ADCImm : MC6809AddSubOp<i8imm>;
  let mayLoad = true in {
    def ADCAbs :
      MC6809AddSubOp<i16imm>;
    def ADCAbsIdx : MC6809AddSub {
      dag InOperandList = (ins ACC8:$l, addr16:$addr, INDEX16:$idx, Cc:$carryin);
    }
    def ADCIndirIdx : MC6809AddSub {
      dag InOperandList = (ins ACC8:$l, ACC16:$addr, INDEX16:$idx, Cc:$carryin);
    }
  }
}

def SBCImm : MC6809AddSubOp<i8imm>;
let mayLoad = true in {
  def SBCAbs : MC6809AddSubOp<i16imm>;
  def SBCAbsIdx : MC6809AddSub {
    dag InOperandList = (ins ACC8:$l, addr16:$addr, INDEX16:$idx, Cc:$carryin);
  }
  def SBCIndirIdx : MC6809AddSub {
    dag InOperandList = (ins ACC8:$l, ACC16:$addr, INDEX16:$idx, Cc:$carryin);
  }
}

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//

class MC6809BitwiseBase : MC6809LogicalInstr {
  let OutOperandList = (outs ACC8:$dst);
  let Constraints = "$dst = $l";
}

multiclass MC6809BitwiseInstr<SDNode node> {
  def Imm : MC6809BitwiseBase {
    let OutOperandList = (outs ACC8:$dst);
    let InOperandList = (ins ACC8:$l, i8imm:$r);

    let Pattern = [(set ACC8:$dst, (node ACC8:$l, imm:$r))];
  }

  let mayLoad = true in {
    def Abs : MC6809BitwiseBase {
      let InOperandList = (ins ACC8:$l, addr16:$r);
    }

    def AbsIdx : MC6809BitwiseBase {
      let InOperandList = (ins ACC8:$l, addr16:$addr, INDEX16:$idx);
    }

    def IndirIdx : MC6809BitwiseBase {
      let InOperandList = (ins ACC8:$l, ACC16:$addr, INDEX16:$idx);
    }
  }
}

defm AND : MC6809BitwiseInstr<and>;
defm EOR : MC6809BitwiseInstr<xor>;
defm ORA : MC6809BitwiseInstr<or>;

//===---------------------------------------------------------------------===//
// Shift/Rotate Instructions
//===---------------------------------------------------------------------===//

class MC6809ShiftRotate : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC8:$dst, Cc:$carryout);
}

class MC6809Shift : MC6809ShiftRotate {
  dag InOperandList = (ins ACC8:$src);
  let Constraints = "$dst = $src";
}
class MC6809Rotate : MC6809ShiftRotate {
  dag InOperandList = (ins ACC8:$src, Cc:$carryin);
  let Constraints = "$dst = $src, $carryout = $carryin";
}

class MC6809ShiftRotateRMW : MC6809LogicalInstr {
  dag OutOperandList = (outs Cc:$carryout);
  let mayLoad = true;
  let mayStore = true;
}

class MC6809ShiftRMW : MC6809ShiftRotateRMW {}
class MC6809RotateRMW : MC6809ShiftRotateRMW {
  let Constraints = "$carryout = $carryin";
}

def ASL : MC6809Shift;
def LSR : MC6809Shift;

def ASLAbs : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr);
}
def ASLAbsIdx : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
}
def LSRAbs : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr);
}
def LSRAbsIdx : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
}

def ROL : MC6809Rotate;
def ROR : MC6809Rotate;

def ROLAbs : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, Cc:$carryin);
}
def ROLAbsIdx : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx, Cc:$carryin);
}
def RORAbs : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, Cc:$carryin);
}
def RORAbsIdx : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx, Cc:$carryin);
}

//===---------------------------------------------------------------------===//
// BIT Instructions
//===---------------------------------------------------------------------===//
// Ever the odd duck.
//===---------------------------------------------------------------------===//

def BITAbs : MC6809LogicalInstr {
  dag OutOperandList = (outs Vc:$v);
  dag InOperandList = (ins ACC8:$l, i16imm:$r);
  let mayLoad = true;
}

//===---------------------------------------------------------------------===//
// Control Flow Instructions
//===---------------------------------------------------------------------===//

class MC6809Branch : MC6809LogicalInstr {
  let isBranch = true;
  let isTerminator = true;
}

// Covers all branch instructions. Branches to $tgt if $flag = $val.
def BR : MC6809Branch {
  dag InOperandList = (ins label:$tgt, CCFlag:$flag, i1imm:$val);
}

class MC6809UnconditionalBranch : MC6809Branch {
  let isBarrier = true;
}

def BRA : MC6809UnconditionalBranch {
  let InOperandList = (ins label:$tgt);
  let Pattern = [(br bb:$tgt)];
}
def JMP : MC6809UnconditionalBranch {
  let InOperandList = (ins label:$tgt);
  let Pattern = [(br bb:$tgt)];
}

def JMPIndir : MC6809UnconditionalBranch {
  let InOperandList = (ins ACC16:$tgt);
  let isIndirectBranch = true;
}

def JSR : MC6809LogicalInstr {
  dag InOperandList = (ins label:$tgt);

  let isCall = true;
}

class MC6809Return : MC6809LogicalInstr {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def RTS : MC6809Return;
def RTI : MC6809Return;

//===---------------------------------------------------------------------===//
// Immediate Load Instructions
//===---------------------------------------------------------------------===//

class MC6809ImmediateLoad<RegisterClass dst, Operand val> : MC6809LogicalInstr {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins val:$val);

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
}

// Set and clear CC bits
def LDNImm : MC6809ImmediateLoad<Nc, i1imm>;
def LDZImm : MC6809ImmediateLoad<Zc, i1imm>;
def LDVImm : MC6809ImmediateLoad<Vc, i1imm>;
def LDCImm : MC6809ImmediateLoad<Cc, i1imm>;

// LDA imm, LDX imm, LDY imm
def LDImm : MC6809ImmediateLoad<ACC8, i8imm>;

//===---------------------------------------------------------------------===//
// Comparison Instructions
//===---------------------------------------------------------------------===//

class MC6809CMP : MC6809LogicalInstr {
  dag OutOperandList = (outs Cc:$carry);
  let isCompare = true;
}

def CMPImm : MC6809CMP { dag InOperandList = (ins ACC8:$l, i8imm:$r); }

let mayLoad = true in {
  def CMPAbs : MC6809CMP { dag InOperandList = (ins ACC8:$l, i16imm:$addr); }
  def CMPAbsIdx : MC6809CMP { dag InOperandList = (ins ACC8:$l, i16imm:$addr, INDEX16:$idx); }
  def CMPIndirIdx : MC6809CMP {
    dag InOperandList = (ins ACC8:$l, ACC16:$addr, INDEX16:$idx);
  }
}

//===---------------------------------------------------------------------===//
// Increment/Decrement Instructions
//===---------------------------------------------------------------------===//

multiclass MC6809IncDec {
  def NAME : MC6809LogicalInstr {
    dag OutOperandList = (outs INDEX16:$dst);
    dag InOperandList = (ins INDEX16:$src);
    let Constraints = "$src = $dst";
  }

  let mayLoad = true, mayStore = true in {
    def CAbs : MC6809LogicalInstr {
      dag InOperandList = (ins i16imm:$addr);
    }
    def CAbsIdx : MC6809LogicalInstr {
      dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
    }
  }
}

// DEX, DEY, DEC
defm DE : MC6809IncDec;

// INX, INY, INC
let isAdd = true in {
  defm IN : MC6809IncDec;
}

//===---------------------------------------------------------------------===//
// Stack Instructions
//===---------------------------------------------------------------------===//

// PHA, PHP
def PH : MC6809LogicalInstr {
  dag InOperandList = (ins ACC:$src);

  let mayStore = true;
}

// PLA, PLP
def PL : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC:$dst);

  let mayLoad = true;
}

//===---------------------------------------------------------------------===//
// Load/Store Instructions
//===---------------------------------------------------------------------===//

class MC6809Load : MC6809LogicalInstr {
  let mayLoad = true;
}

// LDA abs, LDX abs, LDY abs
def LDAbs : MC6809Load {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins i16imm:$src);
}

class MC6809LoadIndexed<RegisterClass dst, RegisterClass idx> : MC6809Load {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins i16imm:$addr, idx:$idx);
}
// LDA abs,x; LDA abs,y
def LDAAbsIdx : MC6809LoadIndexed<ACC8, INDEX16>;
// LDX abs,y
def LDXAbsIdx : MC6809LoadIndexed<INDEX16, INDEX16>;
// LDY abs,x
def LDYAbsIdx : MC6809LoadIndexed<INDEX16, INDEX16>;

// LDA (zp),y
def LDIndirIdx : MC6809Load {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins ACC16:$addr, INDEX16:$offset);
}

class MC6809Store : MC6809LogicalInstr {
  let mayStore = true;
}
// STA abs, STX abs
def STAbs : MC6809Store {
  dag InOperandList = (ins ACC8:$src, i16imm:$dst);
}
// STA abs,x; STA abs,y
def STAbsIdx : MC6809Store {
  dag InOperandList = (ins ACC8:$src, i16imm:$addr, INDEX16:$idx);
}
// STA (zp),y
def STIndirIdx : MC6809Store {
  dag InOperandList = (ins ACC8:$src, ACC16:$addr, INDEX16:$offset);
}

// STZ abs
def STZAbs : MC6809Store {
  dag InOperandList = (ins i16imm:$addr);
}
// STZ abs,x
def STZAbsIdx : MC6809Store {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
}

//===---------------------------------------------------------------------===//
// Addition/Subtraction Patterns
//===---------------------------------------------------------------------===//

def : Pat<(add ACC8:$l, (i8 imm:$imm)), (ADCImm ACC8:$l, imm:$imm, (LDCImm 0))>;
def : Pat<(sub ACC8:$l, (i8 imm:$imm)), (SBCImm ACC8:$l, imm:$imm, (LDCImm -1))>;

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def : Pat<(i8 (anyext BIT1:$s)), (INSERT_SUBREG (IMPLICIT_DEF), BIT1:$s, sub_lsb)>;
def : Pat<(i16 (anyext ACC8:$s)), (INSERT_SUBREG (IMPLICIT_DEF), ACC8:$s, sub_lo_byte)>;
def : Pat<(i16 (anyext BIT1:$s)),
          (INSERT_SUBREG (IMPLICIT_DEF),
                         (INSERT_SUBREG (i8 (IMPLICIT_DEF)), BIT1:$s, sub_lsb),
                         sub_lo_byte)>;

def : Pat<(i1 (trunc ACC8:$s)), (EXTRACT_SUBREG (ANDImm ACC8:$s, 1), sub_lsb)>;
def : Pat<(i8 (trunc ACC16:$s)), (EXTRACT_SUBREG ACC16:$s, sub_lo_byte)>;

//===---------------------------------------------------------------------===//
// Constant Patterns
//===---------------------------------------------------------------------===//

def : Pat<(i1 imm:$s), (LDImm1 imm:$s)>;
def : Pat<(i8 imm:$s), (LDImm imm:$s)>;

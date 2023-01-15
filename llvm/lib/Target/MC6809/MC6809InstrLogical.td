//===-- MC6809InstrLogical.td - Describe MC6809 Logical Instrs --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
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
// Loads and Stores
//===---------------------------------------------------------------------===//
// Load base class
class MC6809LoadBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayLoad = true;
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = operand;
}

multiclass MC6809Load<RegisterClass dst, string inttype> {
  let isAsCheapAsAMove = true, isMoveImm = true, isReMaterializable = true in {
    def Imm : MC6809LoadBase<dst, (ins !cast<Operand>(inttype#"imm"):$imm)> {
      let Pattern = [(set dst:$dst, imm:$imm)];
    }
  }
  def Abs : MC6809LoadBase<dst, (ins addr16:$addr)>;
  def Idx : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
  def Indir : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
}

// All the loads
defm Load8 : MC6809Load<ACC8, "i8">;
defm Load16 : MC6809Load<ACC16, "i16">;
defm Load32 : MC6809Load<ACC32, "i32">;
defm LoadPtr : MC6809Load<INDEX16, "i16">;

// Loads a boolean value into a CC bit or an ACC8 LSB
def Load1Imm : MC6809Pseudo {
  dag OutOperandList = (outs BIT1:$dst);
  dag InOperandList = (ins i1imm:$val);
  let isAsCheapAsAMove = true;
  let isMoveImm = true;
//let isReMaterializable = true;
}

def : Pat<(i1 imm:$s), (Load1Imm imm:$s)>;

def : Pat<(i8 imm:$s), (Load8Imm imm:$s)>;
def : Pat<(i16 imm:$s), (Load16Imm imm:$s)>;
def : Pat<(i32 imm:$s), (Load32Imm imm:$s)>;

def : Pat<(i8 (load (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (Load8Idx INDEX16:$idx, i16imm:$offset)>;
def : Pat<(i8 (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (Load8Idx INDEX16:$idx, ACC8:$offset)>;
def : Pat<(i8 (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (Load8Idx INDEX16:$idx, ACC16:$offset)>;

def : Pat<(i16 (load (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (Load16Idx INDEX16:$idx, i16imm:$offset)>;
def : Pat<(i16 (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (Load16Idx INDEX16:$idx, ACC8:$offset)>;
def : Pat<(i16 (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (Load16Idx INDEX16:$idx, ACC16:$offset)>;

def : Pat<(i32 (load (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (Load32Idx INDEX16:$idx, i16imm:$offset)>;
def : Pat<(i32 (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (Load32Idx INDEX16:$idx, ACC8:$offset)>;
def : Pat<(i32 (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (Load32Idx INDEX16:$idx, ACC16:$offset)>;

def : Pat<(iPTR (load (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (Load32Idx INDEX16:$idx, i16imm:$offset)>;
def : Pat<(iPTR (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (Load32Idx INDEX16:$idx, ACC8:$offset)>;
def : Pat<(iPTR (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (Load32Idx INDEX16:$idx, ACC16:$offset)>;

// Store base class
class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayStore = true;
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins src:$src), operand);
}

multiclass MC6809Store<RegisterClass src> {
  def Abs : MC6809StoreBase<src, (ins addr16:$addr)>;
  def Idx : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
  def Indir : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
}

// All the stores
defm Store8 : MC6809Store<ACC8>;
defm Store16 : MC6809Store<ACC16>;
defm Store32 : MC6809Store<ACC32>;
defm StorePtr : MC6809Store<INDEX16>;

def : Pat<(store (i8 ACC8:$r), (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (Store8Idx ACC8:$r, INDEX16:$idx, i16imm:$offset)>;
def : Pat<(store (i8 ACC8:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset)), (Store8Idx ACC8:$r, INDEX16:$idx, ACC8:$offset)>;
def : Pat<(store (i8 ACC8:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset)), (Store8Idx ACC8:$r, INDEX16:$idx, ACC16:$offset)>;

def : Pat<(store (i16 ACC16:$r), (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (Store16Idx ACC16:$r, INDEX16:$idx, i16imm:$offset)>;
def : Pat<(store (i16 ACC16:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset)), (Store16Idx ACC16:$r, INDEX16:$idx, ACC8:$offset)>;
def : Pat<(store (i16 ACC16:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset)), (Store16Idx ACC16:$r, INDEX16:$idx, ACC16:$offset)>;

def : Pat<(store (i32 ACC32:$r), (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (Store32Idx ACC32:$r, INDEX16:$idx, i16imm:$offset)>;
def : Pat<(store (i32 ACC32:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset)), (Store32Idx ACC32:$r, INDEX16:$idx, ACC8:$offset)>;
def : Pat<(store (i32 ACC32:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset)), (Store32Idx ACC32:$r, INDEX16:$idx, ACC16:$offset)>;

def : Pat<(store (iPTR INDEX16:$r), (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (StorePtrIdx INDEX16:$r, INDEX16:$idx, i16imm:$offset)>;
def : Pat<(store (iPTR INDEX16:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset)), (StorePtrIdx INDEX16:$r, INDEX16:$idx, ACC8:$offset)>;
def : Pat<(store (iPTR INDEX16:$r), (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset)), (StorePtrIdx INDEX16:$r, INDEX16:$idx, ACC16:$offset)>;

//===---------------------------------------------------------------------===//
// Inter-register copies and exchanges
//===---------------------------------------------------------------------===//
class MC6809Transfer<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src);
}

def Copy8 : MC6809Transfer<REGPAIR8>;
def Copy16 : MC6809Transfer<REGPAIR16>;

class MC6809Exchange<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs reg:$dst1, reg:$dst2);
  dag InOperandList = (ins reg:$src1, reg:$src2);
  let Constraints = "$dst1 = $src2, $dst2 = $src1";
}

def Exchange8 : MC6809Exchange<REGPAIR8>;
def Exchange16 : MC6809Exchange<REGPAIR16>;

//===---------------------------------------------------------------------===//
// In-register operations without operands
//===---------------------------------------------------------------------===//
//class MC6809RegisterOp<RegisterClass reg> : MC6809LogicalInstr {
//  dag OutOperandList = (outs reg:$dst);
//  dag InOperandList = (ins reg:$src);
//}

//def Complement8 : MC6809RegisterOp<ACC8>;
//def Complement16 : MC6809RegisterOp<ACC16>;
//def Complement32 : MC6809RegisterOp<ACC32>;

//def Negate8 : MC6809RegisterOp<ACC8>;
//def Negate16 : MC6809RegisterOp<ACC16>;
//def Negate32 : MC6809RegisterOp<ACC32>;

//===---------------------------------------------------------------------===//
// Operandless Accumulator Instructions
//===---------------------------------------------------------------------===//
class MC6809Accumulator<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [NZ, C, V];
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src);
  let Constraints = "$dst = $src";
}

def Neg8 : MC6809Accumulator<ACC8>;
def Neg16 : MC6809Accumulator<ACC16>;
def Neg32 : MC6809Accumulator<ACC32>;

//===---------------------------------------------------------------------===//
// Arithmetic base classes
//===---------------------------------------------------------------------===//

//class MC6809ArithmeticBaseCarry<RegisterClass dst, dag operand> : MC6809LogicalInstr {
//  let Defs = [NZ, V, C];
//  let Uses = [C];
//  dag OutOperandList = (outs dst:$dst);
//  dag InOperandList = !con((ins dst:$src), operand);
//  // let Constraints = "$dst = $src, $carryout = $carryin";
//}

//multiclass MC6809ArithmeticCarry<SDNode node, RegisterClass dst, string inttype> {
//  def Imm : MC6809ArithmeticBaseCarry<dst, (ins !cast<Operand>(inttype#"imm"):$imm)>;
//  def Reg : MC6809ArithmeticBaseCarry<dst, (ins dst:$src2)>;
//  let mayLoad = true in {
//    def Abs : MC6809ArithmeticBaseCarry<dst, (ins addr16:$addr)>;
//    // def Idx : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, unknown:$offset)>;
//    def Idx : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, i16imm:$offset)>;
//    def Indir : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, unknown:$offset)>;
//    def Pop : MC6809ArithmeticBaseCarry<dst, (ins)> {
//      let Defs = [SS, NZ];
//      let Uses = [SS];
//    }
//  }
//}

class MC6809ArithmeticBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins dst:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809Arithmetic<SDNode node, RegisterClass dst, string inttype> {
  def Imm : MC6809ArithmeticBase<dst, (ins !cast<Operand>(inttype#"imm"):$imm)> {
    let Pattern = [(set dst:$dst, (node dst:$src, (am_am_immediate !cast<Operand>(inttype#"imm"):$imm)))];
  }
  def ImmRev : MC6809ArithmeticBase<dst, (ins !cast<Operand>(inttype#"imm"):$imm)> {
    let Pattern = [(set dst:$dst, (node (am_am_immediate !cast<Operand>(inttype#"imm"):$imm), dst:$src))];
  }
  def Reg : MC6809ArithmeticBase<dst, (ins dst:$src2)> {
    let Pattern = [(set dst:$dst, (node dst:$src2, dst:$src))];
  }
  let mayLoad = true in {
    def Abs : MC6809ArithmeticBase<dst, (ins addr16:$addr)>;
    def IdxImm : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, i16imm:$offset)> {
      let Pattern = [(set dst:$dst, (node dst:$src, (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset)))];
    }
    def IdxReg8 : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, ACC8:$offset)> {
      let Pattern = [(set dst:$dst, (node dst:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset)))];
    }
    def IdxReg16 : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, ACC16:$offset)> {
      let Pattern = [(set dst:$dst, (node dst:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset)))];
    }
    def Indir : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809ArithmeticBase<dst, (ins)> {
      let Defs = [SS, NZ, V, C];
      let Uses = [SS];
    }
  }
}

//===---------------------------------------------------------------------===//
// Addition/Subtraction Instructions
//===---------------------------------------------------------------------===//

//let isAdd = true in {
//  defm AddCarry8 : MC6809ArithmeticCarry<addc, ACC8, "i8">;
//  defm AddCarry16 : MC6809ArithmeticCarry<addc, ACC16, "i16">;
//  defm AddCarry32 : MC6809ArithmeticCarry<addc, ACC32, "i32">;
//}
//defm SubBorrow8 : MC6809ArithmeticCarry<subc, ACC8, "i8">;
//defm SubBorrow16 : MC6809ArithmeticCarry<subc, ACC16, "i16">;
//defm SubBorrow32 : MC6809ArithmeticCarry<subc, ACC32, "i32">;

let isAdd = true in {
  defm Add8 : MC6809Arithmetic<add, ACC8, "i8">;
  defm Add16 : MC6809Arithmetic<add, ACC16, "i16">;
  defm Add32 : MC6809Arithmetic<add, ACC32, "i32">;
}

defm Sub8 : MC6809Arithmetic<sub, ACC8, "i8">;
defm Sub16 : MC6809Arithmetic<sub, ACC16, "i16">;
defm Sub32 : MC6809Arithmetic<sub, ACC32, "i32">;

//def : Pat<(sub i8:$a, ACC8:$b), (Neg8 (Sub8Imm ACC8:$b, i8imm:$a))>;
//def : Pat<(sub i16:$a, ACC16:$b), (Neg16 (Sub16Imm ACC16:$b, i16imm:$a))>;
//def : Pat<(sub i32:$a, ACC32:$b), (Neg32 (Sub32Imm ACC32:$b, i32imm:$a))>;

// defm Test8 : MC6809Arithmetic<ACC8>;
// defm Test16 : MC6809Arithmetic<ACC16>;

//def : Pat<(add ACC8:$a, ACC8:$b), (Add8Reg ACC8:$a, ACC8:$b)>;
//def : Pat<(add ACC16:$a, ACC16:$b), (Add16Reg ACC16:$a, ACC16:$b)>;
//def : Pat<(add ACC32:$a, ACC32:$b), (Add32Reg ACC32:$a, ACC32:$b)>;

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//

class MC6809BitwiseBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins dst:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809Bitwise<SDNode node, RegisterClass dst, string inttype> {
  def Imm : MC6809BitwiseBase<dst, (ins !cast<Operand>(inttype#"imm"):$imm)> {
    let Pattern = [(set dst:$dst, (node dst:$src, imm:$imm))];
  }
  def Reg : MC6809BitwiseBase<dst, (ins dst:$src2)>;
  let mayLoad = true in {
    def Abs : MC6809BitwiseBase<dst, (ins addr16:$addr)>;
    def Idx : MC6809BitwiseBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809BitwiseBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809BitwiseBase<dst, (ins)> {
      let Defs = [SS, NZ, V];
      let Uses = [SS];
    }
  }
}

defm AND8 : MC6809Bitwise<and, ACC8, "i8">;
defm AND16 : MC6809Bitwise<and, ACC16, "i16">;
defm OR8 : MC6809Bitwise<or, ACC8, "i8">;
defm OR16 : MC6809Bitwise<or, ACC16, "i16">;
defm XOR8 : MC6809Bitwise<xor, ACC8, "i8">;
defm XOR16 : MC6809Bitwise<xor, ACC16, "i16">;

//===---------------------------------------------------------------------===//
// Comparison Instructions
//===---------------------------------------------------------------------===//
class MC6809CompareBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  let isCompare = true;
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins reg:$reg), operand);
}

multiclass MC6809Compare<RegisterClass dst> {
  def Imm : MC6809CompareBase<dst, (ins i16imm:$imm)>;
  def Reg : MC6809CompareBase<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809CompareBase<dst, (ins addr16:$addr)>;
    def Idx : MC6809CompareBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809CompareBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809CompareBase<dst, (ins)> {
      let Defs = [SS, NZ, V, C];
      let Uses = [SS];
    }
  }
}

defm Compare8 : MC6809Compare<ACC8>;
defm Compare16 : MC6809Compare<ACC16>;
defm ComparePtr : MC6809Compare<INDEX16>;

class MC6809BitBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  let isCompare = true;
  let Defs = [NZ, V];
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins reg:$reg), operand);
}

multiclass MC6809Bit<RegisterClass dst> {
  def Imm : MC6809BitBase<dst, (ins i16imm:$imm)>;
  def Reg : MC6809BitBase<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809BitBase<dst, (ins addr16:$addr)>;
    def Idx : MC6809BitBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809BitBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809BitBase<dst, (ins)> {
      let Defs = [SS, NZ, V];
      let Uses = [SS];
    }
  }
}

defm Bit8 : MC6809Bit<ACC8>;

//===---------------------------------------------------------------------===//
// Funnies
//===---------------------------------------------------------------------===//
class MC6809MultiplyAccumulators<RegisterClass dst, RegisterClass src> : MC6809LogicalInstr {
  let Defs = [Z, C];
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins src:$src1, src:$src2);
  // let Constraints = "$dst1 = $src1, $dst2 = $src2";
  let Pattern = [(set dst:$dst, (mul src:$src1, src:$src2))];
}

def Mul8 : MC6809MultiplyAccumulators<ACC8_B, ACC8_AB> {
  let Defs = [Z, C, AA];
}

class MC6809MultiplyBase<RegisterClass reg_d, RegisterClass reg_w, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, AD];
  dag OutOperandList = (outs reg_w:$dst);
  dag InOperandList = !con((ins reg_d:$src), operand);
  // let Constraints = "$dst2 = $src";
}

multiclass MC6809Multiply<RegisterClass reg_d, RegisterClass reg_w> {
  def Imm : MC6809MultiplyBase<reg_d, reg_w, (ins i16imm:$imm)> {
    let Pattern = [(set reg_w:$dst, (mul reg_d:$src, imm:$imm))];
  }
  let mayLoad = true in {
    def Abs : MC6809MultiplyBase<reg_d, reg_w, (ins addr16:$addr)>;
    def IdxImm : MC6809MultiplyBase<reg_d, reg_w, (ins INDEX16:$idx, i16imm:$offset)> {
      let Pattern = [(set reg_w:$dst, (mul reg_d:$src, (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset)))];
    }
    def IdxReg8 : MC6809MultiplyBase<reg_d, reg_w, (ins INDEX16:$idx, ACC8:$offset)> {
      let Pattern = [(set reg_w:$dst, (mul reg_d:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset)))];
    }
    def IdxReg16 : MC6809MultiplyBase<reg_d, reg_w, (ins INDEX16:$idx, ACC16:$offset)> {
      let Pattern = [(set reg_w:$dst, (mul reg_d:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset)))];
    }
    def Indir : MC6809MultiplyBase<reg_d, reg_w, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809MultiplyBase<reg_d, reg_w, (ins)> {
      let Defs = [SS, NZ];
      let Uses = [SS];
    }
    def Reg : MC6809MultiplyBase<reg_d, reg_w, (ins reg_w:$src2)> {
      let Defs = [SS, NZ];
      let Uses = [SS];
      let Pattern = [(set reg_w:$dst, (mul reg_d:$src, reg_w:$src2))];
    }
  }
}

defm Mul16 : MC6809Multiply<ACC16_D, ACC16_W>;

//===---------------------------------------------------------------------===//
// Control Flow Instructions
//===---------------------------------------------------------------------===//

// Unconditional relative branches

class MC6809Branch : MC6809LogicalInstr {
  let isBranch = true;
  let isTerminator = true;
}

class MC6809UnconditionalBranch : MC6809Branch {
  let isBarrier = true;
  dag InOperandList = (ins label:$tgt);
}

def JumpRelative : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPi_o8PC pcrel8:$tgt)>;
def LongJumpRelative : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPi_o16PC pcrel16:$tgt)>;

def BranchRelative : MC6809UnconditionalBranch, PseudoInstExpansion<(Bbc 0, pcrel8:$tgt)> {
  let Pattern = [(br bb:$tgt)];
}
def LongBranchRelative : MC6809UnconditionalBranch, PseudoInstExpansion<(LBlbc 0, pcrel16:$tgt)>;

// Conditional relative branches

class MC6809ConditionalBranch : MC6809Branch {
  // FIXME: MarkM: The below could use some optimisation in analyzeBranch() to more
  // precisely set the Uses flags depending on the actual condition used.
  let Uses = [NZ, V, C];
  dag InOperandList = (ins condcode:$cond, label:$tgt);
}

def ConditionalBranchRelative : MC6809ConditionalBranch, PseudoInstExpansion<(Bbc condcode:$cond, pcrel8:$tgt)>;
def ConditionalLongBranchRelative : MC6809ConditionalBranch, PseudoInstExpansion<(LBlbc condcode:$cond, pcrel16:$tgt)>;

// Subroutines

def BranchSubroutine : MC6809LogicalInstr, PseudoInstExpansion<(BSRb pcrel8:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isCall = true;
}
def LongBranchSubroutine : MC6809LogicalInstr, PseudoInstExpansion<(LBSRlb pcrel16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isCall = true;
}

def JumpSubroutine : MC6809LogicalInstr, PseudoInstExpansion<(JSRi_o8PC pcrel8:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isCall = true;
}
def LongJumpSubroutine : MC6809LogicalInstr, PseudoInstExpansion<(JSRi_o16PC pcrel16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isCall = true;
}
def IndirectJumpSubroutine : MC6809LogicalInstr, PseudoInstExpansion<(JMPi_eI addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isCall = true;
  let isIndirectBranch = true;
}

class MC6809Return<Instruction Opcode> : MC6809LogicalInstr, PseudoInstExpansion<(Opcode)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def ReturnImplicit : MC6809Return<RTSr>;
def ReturnIRQImplicit : MC6809Return<RTIr>;

// Other unconditional jumps

def JumpAbsolute : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPe addr16:$tgt)>;
def JumpIndir : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPi_eI addr16:$tgt)> {
  let InOperandList = (ins addr16:$tgt);
  let isIndirectBranch = true;
}

def TailJump : MC6809LogicalInstr, PseudoInstExpansion<(JMPe addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isBarrier = true;
  let isTerminator = true;
  let isCall = true;
  let isReturn = true;
}

//===---------------------------------------------------------------------===//
// Pointer instructions
//===---------------------------------------------------------------------===//

class MC6809PtrAdd<RegisterClass idx> : MC6809LogicalInstr {
  dag OutOperandList = (outs idx:$dst);
  dag InOperandList = (ins idx:$src, unknown:$val);
  let isAdd = true;
}

// LEAr n,r or LEAr a,r
def LEAPtrAdd : MC6809PtrAdd<INDEX16>;
//def LEAPtrAddReg8 : MC6809PtrAddReg<INDEX16, ACC8>;
//def LEAPtrAddReg16 : MC6809PtrAddReg<INDEX16, ACC16>;

//===---------------------------------------------------------------------===//
// Stack instructions. These will be squashed together later.
//===---------------------------------------------------------------------===//

class MC6809StackPush<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs);
  dag InOperandList = (ins reg:$reg);
  let mayStore = true;
}

class MC6809StackPull<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [SS];
  let Uses = [SS];
  dag OutOperandList = (outs reg:$reg);
  dag InOperandList = (ins);
  let mayLoad = true;
}

def Push8 : MC6809StackPush<ACC8>;
def Push16 : MC6809StackPush<ACC16>;
def Push32 : MC6809StackPush<ACC32>;
def PushPtr : MC6809StackPush<INDEX16>;
def Pull8 : MC6809StackPull<ACC8>;
def Pull16 : MC6809StackPull<ACC16>;
def Pull32 : MC6809StackPull<ACC32>;
def PullPtr : MC6809StackPull<INDEX16>;

// The "Op" variants are intended to couple with <Instr>Pop instructions.
class MC6809StackPushOp<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs);
  dag InOperandList = (ins reg:$reg);
  let mayStore = true;
}

def PushOp8 : MC6809StackPushOp<ACC8>;
def PushOp16 : MC6809StackPushOp<ACC16>;
def PushOp32 : MC6809StackPushOp<ACC32>;
def PushOpPtr : MC6809StackPushOp<INDEX16>;

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def SEX8Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins BIT1:$src);
  dag OutOperandList = (outs ACC8:$dst);
  let Pattern = [(set ACC8:$dst, (sext BIT1:$src))];
}

def SEX16Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ACC8_B:$src);
  dag OutOperandList = (outs ACC16_D:$dst);
  let Pattern = [(set ACC16_D:$dst, (sext ACC8_B:$src))];
}

def SEX32Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ACC16_W:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (sext ACC16_W:$src))];
}

def ZEX8Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins BIT1:$src);
  dag OutOperandList = (outs ACC8:$dst);
  let Pattern = [(set ACC8:$dst, (zext BIT1:$src))];
}

def ZEX16Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ACC8:$src);
  dag OutOperandList = (outs ACC16:$dst);
  let Pattern = [(set ACC16:$dst, (zext ACC8:$src))];
}

def ZEX32Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ACC16_W:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (zext ACC16_W:$src))];
}

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def : Pat<(i8 (anyext BIT1:$s)), (INSERT_SUBREG (IMPLICIT_DEF), BIT1:$s, sub_lsb)>;
def : Pat<(i16 (anyext ACC8:$s)), (INSERT_SUBREG (IMPLICIT_DEF), ACC8:$s, sub_lo_byte)>;
def : Pat<(i16 (anyext BIT1:$s)), (INSERT_SUBREG (IMPLICIT_DEF), (INSERT_SUBREG (i8 (IMPLICIT_DEF)), BIT1:$s, sub_lsb), sub_lo_byte)>;

def : Pat<(i1 (trunc ACC8:$s)), (EXTRACT_SUBREG ACC8:$s, sub_lsb)>;
def : Pat<(i8 (trunc ACC16:$s)), (EXTRACT_SUBREG ACC16:$s, sub_lo_byte)>;
def : Pat<(i16 (trunc ACC32:$s)), (EXTRACT_SUBREG ACC32:$s, sub_lo_word)>;

//===---------------------------------------------------------------------===//
// Constant Patterns
//===---------------------------------------------------------------------===//

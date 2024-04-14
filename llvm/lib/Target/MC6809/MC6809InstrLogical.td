//===-- MC6809InstrLogical.td - Describe MC6809 Logical Instrs --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// By examining its register operands, every logical instruction can be mapped
// to exactly one real target instruction. Pseudoinstructions where this
// property does not hold are recorded in MC6809InstrPseudos.td instead. These are
// more traditional pseudo-instructions; they may expand out to short snippets
// of code required to perform higher-level operations that cannot be expaneded
// inline, for one reason or another.
//
// There are some exceptions; Compare is tied to CondtionalBranch where possible,
// and some wide instructions are treated as single instructions.
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
// Inter-register copies and exchanges
//===---------------------------------------------------------------------===//
class MC6809Transfer<dag ins, dag outs, string constraints = ""> : MC6809LogicalInstr {
  dag OutOperandList = outs;
  dag InOperandList = ins;
  let Constraints = constraints;
}

def Copy8 : MC6809Transfer<(ins REGPAIR8:$src), (outs REGPAIR8:$dst), "">;
def Copy16 : MC6809Transfer<(ins REGPAIR16:$src), (outs REGPAIR16:$dst), "">;

def Exchange8 : MC6809Transfer<(ins REGPAIR8:$src1, REGPAIR8:$src2), (outs REGPAIR8:$dst1, REGPAIR8:$dst2), "$dst1 = $src2, $dst2 = $src1">;
def Exchange16 : MC6809Transfer<(ins REGPAIR16:$src1, REGPAIR16:$src2), (outs REGPAIR16:$dst1, REGPAIR16:$dst2), "$dst1 = $src2, $dst2 = $src1">;

//===---------------------------------------------------------------------===//
// Operandless Accumulator Instructions
//===---------------------------------------------------------------------===//
class MC6809Accumulator<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [NZ, C, V];
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src);
  let Constraints = "$dst = $src";
}

def Negate_i8 : MC6809Accumulator<ACC8>;
def Negate_i16 : MC6809Accumulator<ACC16>;
let Predicates = [IsHD6309] in {
  def Negate_i32 : MC6809Accumulator<ACC32>;
}

multiclass MC6809NegatePat<ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(vtype (sub (vtype 0), reg:$src)), (!cast<Instruction>("Negate_"#vtype) reg:$src)>;
}

defm NegPat_i8 : MC6809NegatePat<i8, ACC8>;
defm NegPat_ii6 : MC6809NegatePat<i16, ACC16>;
let Predicates = [IsHD6309] in {
  defm NegPat_i32 : MC6809NegatePat<i32, ACC32>;
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

multiclass MC6809Load<RegisterClass dst, ValueType vtype> {
  let isAsCheapAsAMove = true, isMoveImm = true in {
    def _Imm : MC6809LoadBase<dst, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
//  def _Abs : MC6809LoadBase<dst, (ins addr16:$addr)>;
//  def _Idx : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
//  def _Idx_Inc : MC6809LoadBase<dst, src, (ins INDEX16:$idx)>;
//  def _Dec_Idx : MC6809LoadBase<dst, src, (ins INDEX16:$idx)>;
//  def _Indir : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
  let mayLoad = true in {
    def _Abs : MC6809LoadBase<dst, (ins addr16:$addr)>;
    def _Idx_Imm : MC6809LoadBase<dst, (ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809LoadBase<dst, (ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809LoadBase<dst, (ins INDEX16:$idx, ACC16:$offset)>;
    def _Idx_Inc : MC6809LoadBase<dst, (ins INDEX16:$idx)>;
    def _Dec_Idx : MC6809LoadBase<dst, (ins INDEX16:$idx)>;
    def _Indir : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
  }
}

// All the loads
defm Load_i8 : MC6809Load<ACC8, i8>;
defm Load_i16 : MC6809Load<ACC16, i16>;
defm Load_iPtr : MC6809Load<INDEX16, i16>;
let Predicates = [IsHD6309] in {
  defm Load_i32 : MC6809Load<ACC32, i32>;
}

// Loads a boolean value into a NZVC bit or an ACC8 LSB
def Load_i1_Imm : MC6809LogicalInstr {
  dag OutOperandList = (outs BIT1:$dst);
  dag InOperandList = (ins i1imm:$val);
  let isAsCheapAsAMove = true;
  let isMoveImm = true;
}

def : Pat<(i1 imm:$s), (Load_i1_Imm imm:$s)>;

multiclass MC6809LoadPat<ValueType vtype> {
  def _Imm : Pat<(vtype imm:$val), (!cast<Instruction>("Load_"#vtype#"_Imm") imm:$val)>;
  def _Idx_Zero : Pat<(vtype (load INDEX16:$idx)), (!cast<Instruction>("Load_"#vtype#"_Idx_Imm") INDEX16:$idx, (i16 0))>;
  def _Idx_Imm : Pat<(vtype (load (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_Idx_Imm") INDEX16:$idx, i16imm:$offset)>;
  def _Idx_Reg8 : Pat<(vtype (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>("Load_"#vtype#"_Idx_Reg8") INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(vtype (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>("Load_"#vtype#"_Idx_Reg16") INDEX16:$idx, ACC16:$offset)>;
}

defm LoadPat_i8 : MC6809LoadPat<i8>;
defm LoadPat_i16 : MC6809LoadPat<i16>;
let Predicates = [IsHD6309] in {
  defm LoadPat_i32 : MC6809LoadPat<i32>;
}
// Store base class
class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayStore = true;
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins src:$src), operand);
}

multiclass MC6809Store<RegisterClass src> {
  def _Abs : MC6809StoreBase<src, (ins addr16:$addr)>;
  def _Idx : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
  def _Indir : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
}

// All the stores
defm Store_i8 : MC6809Store<ACC8>;
defm Store_i16 : MC6809Store<ACC16>;
defm Store_iPtr : MC6809Store<INDEX16>;
let Predicates = [IsHD6309] in {
  defm Store_i32 : MC6809Store<ACC32>;
}

multiclass MC6809StorePat<ValueType vtype, RegisterClass reg> {
  def _#vtype#_Idx_Zero : Pat<(store reg:$src, INDEX16:$idx), (!cast<Instruction>("Store_"#vtype#"_Idx") reg:$src, INDEX16:$idx, (i64 0))>;
  def _#vtype#_Idx_Imm : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (!cast<Instruction>("Store_"#vtype#"_Idx") reg:$src, INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Idx_Reg8 : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset)), (!cast<Instruction>("Store_"#vtype#"_Idx") reg:$src, INDEX16:$idx, ACC8:$offset)>;
  def _#vtype#_Idx_Reg16 : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset)), (!cast<Instruction>("Store_"#vtype#"_Idx") reg:$src, INDEX16:$idx, ACC16:$offset)>;
}

defm StorePat_i8 : MC6809StorePat<i8, ACC8>;
defm StorePat_i16 : MC6809StorePat<i16, ACC16>;
defm StorePat_iPtr : MC6809StorePat<i16, INDEX16>;
let Predicates = [IsHD6309] in {
  defm StorePat_i32 : MC6809StorePat<i32, ACC32>;
}

//===---------------------------------------------------------------------===//
// Set/Clear CC bits
//===---------------------------------------------------------------------===//

class MC6809ImmediateLoad<RegisterClass dst, Operand val> : MC6809LogicalInstr {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins val:$val);
  let isAsCheapAsAMove = true;
  let isMoveImm = true;
}

// Clear/set overflow
//def CLV : MC6809ImmediateLoad<Vc, i1imm>, PseudoInstExpansion<(CLV_Implied)> {
//  dag InOperandList = (ins);
//}

// Clear/set carry
//def LoadCarryImm : MC6809ImmediateLoad<Cc, i1imm>;

//===---------------------------------------------------------------------===//
// Addition/subtraction
//===---------------------------------------------------------------------===//

// Ignoring carry

class MC6809ArithmeticBase<RegisterClass dst, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins src:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809Arithmetic<ValueType vtype, RegisterClass dst, RegisterClass src> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBase<dst, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Abs : MC6809ArithmeticBase<dst, src, (ins addr16:$addr)>;
    def _Idx_Imm : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx, ACC16:$offset)>;
    def _Idx_Inc : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx)>;
    def _Dec_Idx : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx)>;
    def _Indir : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    def _Reg : MC6809ArithmeticBase<dst, src, (ins src:$src2)>;
  }
}

let isAdd = true in {
  defm Add_i8 : MC6809Arithmetic<i8, ACC8, ACC8>;
  defm Add_i16 : MC6809Arithmetic<i16, ACC16, ACC16>;
  let Predicates = [IsHD6309] in {
    defm Add_i32 : MC6809Arithmetic<i32, ACC32, ACC32>;
  }
}
defm Sub_i8 : MC6809Arithmetic<i8, ACC8, ACC8>;
defm Sub_i16 : MC6809Arithmetic<i16, ACC16, ACC16>;
let Predicates = [IsHD6309] in {
  defm Sub_i32 : MC6809Arithmetic<i32, ACC32, ACC32>;
}

// Setting carry

class MC6809ArithmeticBaseCarry<RegisterClass dst, RegisterClass carry, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs dst:$dst, carry:$carry);
  dag InOperandList = !con((ins src:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809ArithmeticCarry<ValueType vtype, RegisterClass dst, RegisterClass carry, RegisterClass src> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBaseCarry<dst, carry, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Abs : MC6809ArithmeticBaseCarry<dst, carry, src, (ins addr16:$addr)>;
    def _Idx_Imm : MC6809ArithmeticBaseCarry<dst, carry, src, (ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809ArithmeticBaseCarry<dst, carry, src, (ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809ArithmeticBaseCarry<dst, carry, src, (ins INDEX16:$idx, ACC16:$offset)>;
    def _Idx_Inc : MC6809ArithmeticBaseCarry<dst, carry, src, (ins INDEX16:$idx)>;
    def _Dec_Idx : MC6809ArithmeticBaseCarry<dst, carry, src, (ins INDEX16:$idx)>;
    def _Indir : MC6809ArithmeticBaseCarry<dst, carry, src, (ins INDEX16:$idx, unknown:$offset)>;
    def _Reg : MC6809ArithmeticBaseCarry<dst, carry, src, (ins src:$src2)>;
  }
}

let isAdd = true in {
  defm AddSetCarry_i8 : MC6809ArithmeticCarry<i8, ACC8, CARRY, ACC8>;
  defm AddSetCarry_i16 : MC6809ArithmeticCarry<i16, ACC16, CARRY, ACC16>;
  let Predicates = [IsHD6309] in {
    defm AddSetCarry_i32 : MC6809ArithmeticCarry<i32, ACC32, CARRY, ACC32>;
  }
}
defm SubSetCarry_i8 : MC6809ArithmeticCarry<i8, ACC8, CARRY, ACC8>;
defm SubSetCarry_i16 : MC6809ArithmeticCarry<i16, ACC16, CARRY, ACC16>;
let Predicates = [IsHD6309] in {
  defm SubSetCarry_i32 : MC6809ArithmeticCarry<i32, ACC32, CARRY, ACC32>;
}

let Defs = [NZ], Constraints = "$carry = $src" in {
  defm MulSetCarry_i16 : MC6809ArithmeticCarry<i16, AWc, ADc, ADc>;
}
let Defs = [NZ], Constraints = "$dst = $src" in {
  defm MulHSetCarry_i16 : MC6809ArithmeticCarry<i16, ADc, AWc, ADc>;
}

multiclass MC6809ArithPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Imm : Pat<(vtype (node reg:$src, imm:$val)), (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val)>;
  def _Idx_Imm : Pat<(vtype (node reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Imm") reg:$src, INDEX16:$idx, imm:$offset)>;
  def _Idx_Reg8 : Pat<(vtype (node reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Reg8") reg:$src, INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(vtype (node reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Reg16") reg:$src, INDEX16:$idx, ACC16:$offset)>;
  if !not(!eq(vtype, i32)) then {
    def _Reg : Pat<(vtype (node reg:$src, reg:$src2)), (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, reg:$src2)>;
  }
}

multiclass MC6809ArithRevPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Imm : Pat<(vtype (node (am_am_immediate !cast<Operand>(vtype#"imm"):$val), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val))>;
  //def _Imm : Pat<(vtype (node (am_am_immediate i32imm:$val), reg:$src)), (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val)>;
}

defm AddPat_i8 : MC6809ArithPat<add, i8, ACC8, "Add">;
defm SubPat_i8 : MC6809ArithPat<sub, i8, ACC8, "Sub">;
defm SubRevPat_i8 : MC6809ArithRevPat<sub, i8, ACC8, "Sub">;
defm AddPat_ii6 : MC6809ArithPat<add, i16, ACC16, "Add">;
defm SubPat_ii6 : MC6809ArithPat<sub, i16, ACC16, "Sub">;
defm SubRevPat_ii6 : MC6809ArithRevPat<sub, i16, ACC16, "Sub">;
let Predicates = [IsHD6309] in {
  defm AddPat_i32 : MC6809ArithPat<add, i32, ACC32, "Add">;
  defm SubPat_i32 : MC6809ArithPat<sub, i32, ACC32, "Sub">;
  defm SubRevPat_i32 : MC6809ArithRevPat<sub, i32, ACC32, "Sub">;
}
//def SubRevPat_i32_Imm : Pat<(i32 (sub (am_am_immediate i32imm:$val), ACC32:$src)), (Negate_i32 (Sub_i32_Imm ACC32:$src, imm:$val))>;

let Predicates = [IsHD6309] in {
  defm MulPat_i16 : MC6809ArithPat<mul, i16, ADc, "MulSetCarry">;
  defm MulHSPat_i16 : MC6809ArithPat<mulhs, i16, ADc, "MulHSetCarry">;
  defm MulHUPat_i16 : MC6809ArithPat<mulhu, i16, ADc, "MulHSetCarry">;
}

// Setting and consuming carry

class MC6809ArithmeticBaseCarryUse<RegisterClass dst, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs dst:$dst, CARRY:$carry);
  dag InOperandList = !con((ins src:$src, CARRY:$carry_in), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809ArithmeticCarryUse<ValueType vtype, RegisterClass dst, RegisterClass src> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBaseCarryUse<dst, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Abs : MC6809ArithmeticBaseCarryUse<dst, src, (ins addr16:$addr)>;
    def _Idx_Imm : MC6809ArithmeticBaseCarryUse<dst, src, (ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809ArithmeticBaseCarryUse<dst, src, (ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809ArithmeticBaseCarryUse<dst, src, (ins INDEX16:$idx, ACC16:$offset)>;
    def _Idx_Inc : MC6809ArithmeticBaseCarryUse<dst, src, (ins INDEX16:$idx)>;
    def _Dec_Idx : MC6809ArithmeticBaseCarryUse<dst, src, (ins INDEX16:$idx)>;
    def _Indir : MC6809ArithmeticBaseCarryUse<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    def _Reg : MC6809ArithmeticBaseCarryUse<dst, src, (ins src:$src2)>;
  }
}

let isAdd = true in {
  defm AddSetCarryUse_i8 : MC6809ArithmeticCarryUse<i8, ACC8, ACC8>;
  defm AddSetCarryuse_i16 : MC6809ArithmeticCarryUse<i16, ACC16, ACC16>;
  let Predicates = [IsHD6309] in {
    defm AddSetCarryuse_i32 : MC6809ArithmeticCarryUse<i32, ACC32, ACC32>;
  }
}

defm SubSetCarryUse_i8 : MC6809ArithmeticCarryUse<i8, ACC8, ACC8>;
defm SubSetCarryuse_i16 : MC6809ArithmeticCarryUse<i16, ACC16, ACC16>;
let Predicates = [IsHD6309] in {
  defm SubSetCarryuse_i32 : MC6809ArithmeticCarryUse<i32, ACC32, ACC32>;
}

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//

class MC6809BitwiseBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = !con((ins reg:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809Bitwise<ValueType vtype, RegisterClass reg> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809BitwiseBase<reg, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
  let Predicates = [IsHD6309] in {
    def _Reg : MC6809BitwiseBase<reg, (ins reg:$src2)>;
  }
  let mayLoad = true in {
    def _Abs : MC6809BitwiseBase<reg, (ins addr16:$addr)>;
    def _Idx_Imm : MC6809BitwiseBase<reg, (ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809BitwiseBase<reg, (ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809BitwiseBase<reg, (ins INDEX16:$idx, ACC16:$offset)>;
    def _Idx_Inc : MC6809BitwiseBase<reg, (ins INDEX16:$idx)>;
    def _Dec_Idx : MC6809BitwiseBase<reg, (ins INDEX16:$idx)>;
    def _Indir : MC6809BitwiseBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm AND_i1 : MC6809Bitwise<i8, BIT1>;
defm OR_i1 : MC6809Bitwise<i8, BIT1>;
defm XOR_i1 : MC6809Bitwise<i8, BIT1>;
defm AND_i8 : MC6809Bitwise<i8, ACC8_AB>;
defm OR_i8 : MC6809Bitwise<i8, ACC8_AB>;
defm XOR_i8 : MC6809Bitwise<i8, ACC8_AB>;
let Predicates = [IsHD6309] in {
    defm AND_i16 : MC6809Bitwise<i16, ADc>;
    defm OR_i16 : MC6809Bitwise<i16, ADc>;
    defm XOR_i16 : MC6809Bitwise<i16, ADc>;
}

defm ANDPat_i8 : MC6809ArithPat<and, i8, ACC8_AB, "AND">;
defm ORPat_i8 : MC6809ArithPat<or, i8, ACC8_AB, "OR">;
defm XORPat_i8 : MC6809ArithPat<xor, i8, ACC8_AB, "XOR">;
defm ANDPat_i16 : MC6809ArithPat<and, i16, ADc, "AND">;
defm ORPat_i16 : MC6809ArithPat<or, i16, ADc, "OR">;
defm XORPat_i16 : MC6809ArithPat<xor, i16, ADc, "XOR">;

// Bit-testing

//class MC6809BitBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
//  let isCompare = true;
//  // let Defs = [NZ, V];
//  dag OutOperandList = (outs i1:$cc);
//  dag InOperandList = !con((ins reg:$reg), operand);
//}

//multiclass MC6809Bit<RegisterClass reg> {
//  def Imm : MC6809BitBase<reg, (ins i16imm:$imm)>;
//  def Reg : MC6809BitBase<reg, (ins reg:$src)>;
//  let mayLoad = true in {
//    def Abs : MC6809BitBase<reg, (ins addr16:$addr)>;
//    def Idx : MC6809BitBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
//    def Indir : MC6809BitBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
//  }
//}

//defm Bit_i8 : MC6809Bit<ACC8_AB>;
//defm Bit_i16 : MC6809Bit<ADc>;

//===---------------------------------------------------------------------===//
// Shift/Rotate instructions
//===---------------------------------------------------------------------===//

class MC6809ShiftBase<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src, i8imm:$val);
  let Constraints = "$dst = $src";
}

multiclass MC6809Shift<RegisterClass reg> {
  def _Reg : MC6809ShiftBase<reg>;
//  let mayLoad = true in {
//    def _Abs : MC6809ShiftBase<reg, (ins addr16:$addr)>;
//    def _Idx_Imm : MC6809ShiftBase<reg, (ins INDEX16:$idx, i16imm:$offset)>;
//    def _Idx_Reg8 : MC6809ShiftBase<reg, (ins INDEX16:$idx, ACC8:$offset)>;
//    def _Idx_Reg16 : MC6809ShiftBase<reg, (ins INDEX16:$idx, ACC16:$offset)>;
//    def _Indir : MC6809ShiftBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
//  }
}

defm LSL_i8 : MC6809Shift<ACC8_AB>;
defm LSL_i16 : MC6809Shift<ADc>;

defm ASR_i8 : MC6809Shift<ACC8_AB>;
defm ASR_i16 : MC6809Shift<ADc>;
defm LSR_i8 : MC6809Shift<ACC8_AB>;
defm LSR_i16 : MC6809Shift<ADc>;

multiclass MC6809ShiftPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Reg : Pat<(vtype (node reg:$src, (i8 imm:$val))), (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, imm:$val)>;
//  def _Idx_Imm : Pat<(vtype (node reg:$src, (i1 1))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Imm") reg:$src, INDEX16:$idx, imm:$offset)>;
//  def _Idx_Reg8 : Pat<(vtype (node reg:$src, (i1 1))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Reg8") reg:$src, INDEX16:$idx, ACC8:$offset)>;
//  def _Idx_Reg16 : Pat<(vtype (node reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Reg16") reg:$src, INDEX16:$idx, ACC16:$offset)>;
}

defm LSL_Pat_i8 : MC6809ShiftPat<shl, i8, ACC8_AB, "LSL">;
defm LSL_Pat_i16 : MC6809ShiftPat<shl, i16, ADc, "LSL">;

defm ASR_Pat_i8 : MC6809ShiftPat<sra, i8, ACC8_AB, "ASR">;
defm ASR_Pat_i16 : MC6809ShiftPat<sra, i16, ADc, "ASR">;
defm LSR_Pat_i8 : MC6809ShiftPat<srl, i8, ACC8_AB, "LSR">;
defm LSR_Pat_i16 : MC6809ShiftPat<srl, i16, ADc, "LSR">;

//===---------------------------------------------------------------------===//
// CC register
//===---------------------------------------------------------------------===//

class MC6809ConditionCodes<RegisterClass cc> : MC6809LogicalInstr {
  dag OutOperandList = (outs cc:$dst);
  dag InOperandList = (ins cc:$src, i8imm:$val);
  let Constraints = "$dst = $src";
}

def ORCC_Imm : MC6809ConditionCodes<CCond>;
def ANDCC_Imm : MC6809ConditionCodes<CCond>;

//===---------------------------------------------------------------------===//
// Multiply
//===---------------------------------------------------------------------===//
class MC6809MultiplyAccumulatorsLow : MC6809LogicalInstr {
  let Defs = [Z, C, AA, AD, AQ];
  dag OutOperandList = (outs ABc:$dst);
  dag InOperandList = (ins ABc:$src1, AAc:$src2);
  let isCommutable = true;
}

def Mul_i8_i8 : MC6809MultiplyAccumulatorsLow;

class MC6809MultiplyAccumulatorsHigh : MC6809LogicalInstr {
  let Defs = [Z, C, AB, AD, AQ];
  dag OutOperandList = (outs AAc:$dst);
  dag InOperandList = (ins ABc:$src1, AAc:$src2);
  let isCommutable = true;
}

def MulH_i8_i8 : MC6809MultiplyAccumulatorsHigh;

class MC6809MultiplyAccumulatorsExtend : MC6809LogicalInstr {
  let Defs = [Z, C, AA, AB, AQ];
  dag OutOperandList = (outs ADc:$dst);
  dag InOperandList = (ins ABc:$src1, AAc:$src2);
  let isCommutable = true;
}

def Mul_i8_i16 : MC6809MultiplyAccumulatorsExtend;

//def Mul_i8_i8 : MC6809MultiplyAccumulators<ABc> {
//   let Defs = [Z, C, AA];
//}

//def MulH_i8_i8 : MC6809MultiplyAccumulators<AAc> {
//   let Defs = [Z, C, AB];
//}

//def Mul_i8_i16 : MC6809MultiplyAccumulators<ADc>;

def MulPat_i8_i8 : Pat<(i8 (mul ABc:$src1, AAc:$src2)), (Mul_i8_i8 ABc:$src1, AAc:$src2)>;
def MulHUPat_i8_i8 : Pat<(i8 (mulhu ABc:$src1, AAc:$src2)), (MulH_i8_i8 ABc:$src1, AAc:$src2)>;
def MulHSPat_i8_i8 : Pat<(i8 (mulhs ABc:$src1, AAc:$src2)), (MulH_i8_i8 ABc:$src1, AAc:$src2)>;
def MulPat_s_i8_i16 : Pat<(i16 (mul (sext ABc:$src1), (sext AAc:$src2))), (Mul_i8_i16 ABc:$src1, AAc:$src2)>;
def MulPat_u_i8_i16 : Pat<(i16 (mul (zext ABc:$src1), (zext AAc:$src2))), (Mul_i8_i16 ABc:$src1, AAc:$src2)>;

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

def BranchRelative : MC6809UnconditionalBranch {
  let Pattern = [(br bb:$tgt)];
}
def LongBranchRelative : MC6809UnconditionalBranch;

def JumpRelative : MC6809UnconditionalBranch;
def LongJumpRelative : MC6809UnconditionalBranch;

// Covers all branch instructions. Branches to $tgt if $flag = $val.
def BranchFlagIs : MC6809Branch {
  dag InOperandList = (ins label:$tgt, CCFlag:$flag, i1imm:$val);
}

// Conditional relative branches

class MC6809ConditionalBranch : MC6809Branch {
  dag InOperandList = (ins condcode:$cc, label:$tgt, CCond:$bits);
}

// Covers all conditional branch instructions. Branches to $tgt if $cc is true.
def ConditionalBranchRelative : MC6809ConditionalBranch;
def ConditionalLongBranchRelative : MC6809ConditionalBranch;

//def BranchCondBit : Pat<(brcond i1:$bit, bb:$tgt), (ConditionalBranchRelative (i4 5), bb:$tgt, BIT1:$bit)>;

// Subroutines

class MC6809Subroutine : MC6809LogicalInstr {
  let isCall = true;
  dag InOperandList = (ins label:$tgt);
}

def BranchSubroutine : MC6809Subroutine, PseudoInstExpansion<(BSRb pcrel8:$tgt)>;
def LongBranchSubroutine : MC6809Subroutine, PseudoInstExpansion<(LBSRlb pcrel16:$tgt)>;

def JumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_o8PC pcrel8:$tgt)>;
def LongJumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_o16PC pcrel16:$tgt)>;
def IndirectJumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_eI addr16:$tgt)> {
  let isIndirectBranch = true;
  dag InOperandList = (ins addr16:$tgt);
}

class MC6809Return<Instruction Opcode> : MC6809LogicalInstr, PseudoInstExpansion<(Opcode)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def ReturnImplicit : MC6809Return<RTSr>;
def ReturnIRQImplicit : MC6809Return<RTIr>;

// Other unconditional jumps

def JumpAbsolute : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPe addr16:$tgt)> {
  let InOperandList = (ins label:$tgt);
}
def JumpIndir : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPi_eI addr16:$tgt)> {
  let isIndirectBranch = true;
  let InOperandList = (ins label:$tgt);
}

def TailJump : MC6809LogicalInstr, PseudoInstExpansion<(LBRAlb pcrel16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isBarrier = true;
  let isTerminator = true;
  let isCall = true;
  let isReturn = true;
}

//===---------------------------------------------------------------------===//
// Comparison Instructions
//===---------------------------------------------------------------------===//

class MC6809CompareBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  //let Defs = [NZVC, NZ, N, Z, V, C];
  dag OutOperandList = (outs CCond:$dst);
  dag InOperandList = !con((ins i8imm:$cc, reg:$src), operand);
  let isCompare = true;
}

multiclass MC6809Compare<ValueType vtype, RegisterClass reg> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809CompareBase<reg, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
  def _Reg : MC6809CompareBase<reg, (ins reg:$src2)>;
  let mayLoad = true in {
    def _Abs : MC6809CompareBase<reg, (ins addr16:$addr)>;
    def _Idx_Imm : MC6809CompareBase<reg, (ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809CompareBase<reg, (ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809CompareBase<reg, (ins INDEX16:$idx, ACC16:$offset)>;
    def _Indir : MC6809CompareBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm Compare_i8 : MC6809Compare<i8, ACC8>;
defm Compare_i16 : MC6809Compare<i16, ACC16>;
let Predicates = [IsHD6309] in {
  defm Compare_i32 : MC6809Compare<i32, ACC32>;
}

multiclass MC6809CompPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Imm : Pat<(i8 (setcc reg:$src, imm:$val, CC)), (!cast<Instruction>("Compare_"#vtype#"_Imm") cc, reg:$src, imm:$val)>;
  def _Idx_Imm : Pat<(i8 (setcc reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (!cast<Instruction>("Compare_"#vtype#"_Idx_Imm") cc, reg:$src, INDEX16:$idx, i16imm:$offset)>;
  def _Idx_Reg8 : Pat<(i8 (setcc reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), (!cast<Instruction>("Compare_"#vtype#"_Idx_Reg8") cc, reg:$src, INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(i8 (setcc reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), (!cast<Instruction>("Compare_"#vtype#"_Idx_Reg16") cc, reg:$src, INDEX16:$idx, ACC16:$offset)>;
}

multiclass MC6809CompCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809CompPat<CC, ccode, i8, ACC8>;
  defm _i16 : MC6809CompPat<CC, ccode, i16, ACC16>;
  let Predicates = [IsHD6309] in {
    defm _i32 : MC6809CompPat<CC, ccode, i32, ACC32>;
  }
}

defm CompPat_HI : MC6809CompCondPat<SETUGT, 2>;
defm CompPat_HS : MC6809CompCondPat<SETUGE, 4>;
defm CompPat_LS : MC6809CompCondPat<SETULE, 3>;
defm CompPat_LO : MC6809CompCondPat<SETULT, 5>;
defm CompPat_GT : MC6809CompCondPat<SETGT, 14>;
defm CompPat_GE : MC6809CompCondPat<SETGE, 12>;
defm CompPat_LE : MC6809CompCondPat<SETLE, 15>;
defm CompPat_LT : MC6809CompCondPat<SETLT, 13>;
defm CompPat_EQ : MC6809CompCondPat<SETEQ, 7>;
defm CompPat_NE : MC6809CompCondPat<SETNE, 6>;

// Test instructions - like compare, but the comparison is with an implied zero

class MC6809TestBase<dag operand> : MC6809LogicalInstr {
  //let Defs = [NZVC, NZ, N, Z, V, C];
  dag OutOperandList = (outs CCond:$dst);
  dag InOperandList = !con((ins i8imm:$cc), operand);
  let isCompare = true;
}

multiclass MC6809Test<RegisterClass reg> {
  def _Reg : MC6809TestBase<(ins reg:$src)>;
  let mayLoad = true in {
    def _Abs : MC6809TestBase<(ins addr16:$addr)>;
    def _Idx_Imm : MC6809TestBase<(ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809TestBase<(ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809TestBase<(ins INDEX16:$idx, ACC16:$offset)>;
    def _Indir : MC6809TestBase<(ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm Test_i8 : MC6809Test<ACC8_AB>;
defm Test_i16 : MC6809Test<ACC16>;
defm Test_i32 : MC6809Test<ACC32>;

multiclass MC6809TestPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(i8 (setcc (vtype 0), reg:$src, CC)), (!cast<Instruction>("Test_"#vtype#"_Reg") cc, reg:$src)>;
  def _Idx_Imm : Pat<(i8 (setcc (vtype 0), (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (!cast<Instruction>("Test_"#vtype#"_Idx_Imm") cc, INDEX16:$idx, i16imm:$offset)>;
  def _Idx_Reg8 : Pat<(i8 (setcc (vtype 0), (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), (!cast<Instruction>("Test_"#vtype#"_Idx_Reg8") cc, INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(i8 (setcc (vtype 0), (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), (!cast<Instruction>("Test_"#vtype#"_Idx_Reg16") cc, INDEX16:$idx, ACC16:$offset)>;
}

multiclass MC6809TestCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809TestPat<CC, ccode, i8, ACC8_AB>;
  defm _i16 : MC6809TestPat<CC, ccode, i16, ACC16>;
  defm _i32 : MC6809TestPat<CC, ccode, i32, ACC32>;
}

defm TestPat_HI : MC6809TestCondPat<SETUGT, 2>;
defm TestPat_HS : MC6809TestCondPat<SETUGE, 4>;
defm TestPat_LS : MC6809TestCondPat<SETULE, 3>;
defm TestPat_LO : MC6809TestCondPat<SETULT, 5>;
defm TestPat_GT : MC6809TestCondPat<SETGT, 14>;
defm TestPat_GE : MC6809TestCondPat<SETGE, 12>;
defm TestPat_LE : MC6809TestCondPat<SETLE, 15>;
defm TestPat_LT : MC6809TestCondPat<SETLT, 13>;
defm TestPat_EQ : MC6809TestCondPat<SETEQ, 7>;
defm TestPat_NE : MC6809TestCondPat<SETNE, 6>;

// Bit instructions - like compare, but the comparison operation is AND rather than subtract.

class MC6809BitTestBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  //let Defs = [NZVC, NZ, N, Z, V, C];
  dag OutOperandList = (outs CCond:$dst);
  dag InOperandList = !con((ins i8imm:$cc, reg:$src), operand);
  let isCompare = true;
}

multiclass MC6809BitTest<ValueType vtype, RegisterClass reg> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809BitTestBase<reg, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
  def _Reg : MC6809BitTestBase<reg, (ins reg:$src2)>;
  let mayLoad = true in {
    def _Abs : MC6809BitTestBase<reg, (ins addr16:$addr)>;
    def _Idx_Imm : MC6809BitTestBase<reg, (ins INDEX16:$idx, i16imm:$offset)>;
    def _Idx_Reg8 : MC6809BitTestBase<reg, (ins INDEX16:$idx, ACC8:$offset)>;
    def _Idx_Reg16 : MC6809BitTestBase<reg, (ins INDEX16:$idx, ACC16:$offset)>;
    def _Indir : MC6809BitTestBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm BitTest_i8 : MC6809BitTest<i8, ACC8_AB>;
let Predicates = [IsHD6309] in {
  defm BitTest_i16 : MC6809BitTest<i16, ACC16_D>;
}

//multiclass MC6809BitTestPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
//  def _Imm : Pat<(i8 (setcc (vtype 1), imm:$val, CC)), (!cast<Instruction>("BitTest_"#vtype#"_Imm") cc, reg:$src, imm:$val)>;
//  def _Idx_Imm : Pat<(i8 (setcc (vtype 1), (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (!cast<Instruction>("BitTest_"#vtype#"_Idx_Imm") cc, (Load_i8_Imm, INDEX16:$idx, i16imm:$offset)>;
//  def _Idx_Reg8 : Pat<(i8 (setcc (vtype 1), (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), (!cast<Instruction>("BitTest_"#vtype#"_Idx_Reg8") cc, reg:$src, INDEX16:$idx, ACC8:$offset)>;
//  def _Idx_Reg16 : Pat<(i8 (setcc (vtype 1), (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), (!cast<Instruction>("BitTest_"#vtype#"_Idx_Reg16") cc, reg:$src, INDEX16:$idx, ACC16:$offset)>;
//}

//multiclass MC6809BitTestCondPat<CondCode CC, int ccode> {
//  defm _i8 : MC6809BitTestPat<CC, ccode, i8, ACC8_AB>;
//  let Predicates = [IsHD6309] in {
//    defm _i16 : MC6809BitTestPat<CC, ccode, i16, ACC16_D>;
//  }
//}

//defm BitTestPat_HI : MC6809BitTestCondPat<SETUGT, 2>;
//defm BitTestPat_HS : MC6809BitTestCondPat<SETUGE, 4>;
//defm BitTestPat_LS : MC6809BitTestCondPat<SETULE, 3>;
//defm BitTestPat_LO : MC6809BitTestCondPat<SETULT, 5>;
//defm BitTestPat_GT : MC6809BitTestCondPat<SETGT, 14>;
//defm BitTestPat_GE : MC6809BitTestCondPat<SETGE, 12>;
//defm BitTestPat_LE : MC6809BitTestCondPat<SETLE, 15>;
//defm BitTestPat_LT : MC6809BitTestCondPat<SETLT, 13>;
//defm BitTestPat_EQ : MC6809BitTestCondPat<SETEQ, 7>;
//defm BitTestPat_NE : MC6809BitTestCondPat<SETNE, 6>;

//===---------------------------------------------------------------------===//
// Fused Compare/Branch patterns
//===---------------------------------------------------------------------===//

multiclass MC6809CompBraPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Imm : Pat<(brcond (i1 (setcc reg:$src, imm:$val, CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Imm") cc, reg:$src, imm:$val))>;
  def _Idx_Imm : Pat<(brcond (i1 (setcc reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Idx_Imm") cc, reg:$src, INDEX16:$idx, i16imm:$offset))>;
  def _Idx_Reg8 : Pat<(brcond (i1 (setcc reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Idx_Reg8") cc, reg:$src, INDEX16:$idx, ACC8:$offset))>;
  def _Idx_Reg16 : Pat<(brcond (i1 (setcc reg:$src, (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Idx_Reg16") cc, reg:$src, INDEX16:$idx, ACC16:$offset))>;
}

multiclass MC6809CompBraCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809CompBraPat<CC, ccode, i8, ACC8>;
  defm _i16 : MC6809CompBraPat<CC, ccode, i16, ACC16>;
  let Predicates = [IsHD6309] in {
    defm _i32 : MC6809CompBraPat<CC, ccode, i32, ACC32>;
  }
}

defm CompBraPat_HI : MC6809CompBraCondPat<SETUGT, 2>;
defm CompBraPat_HS : MC6809CompBraCondPat<SETUGE, 4>;
defm CompBraPat_LS : MC6809CompBraCondPat<SETULE, 3>;
defm CompBraPat_LO : MC6809CompBraCondPat<SETULT, 5>;
defm CompBraPat_GT : MC6809CompBraCondPat<SETGT, 14>;
defm CompBraPat_GE : MC6809CompBraCondPat<SETGE, 12>;
defm CompBraPat_LE : MC6809CompBraCondPat<SETLE, 15>;
defm CompBraPat_LT : MC6809CompBraCondPat<SETLT, 13>;
defm CompBraPat_EQ : MC6809CompBraCondPat<SETEQ, 7>;
defm CompBraPat_NE : MC6809CompBraCondPat<SETNE, 6>;

// Fused Test/Branch patterns. Same as Compare/Branch, but the register is
// replaced by an implied zero.

multiclass MC6809TestBraPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(brcond (i1 (setcc reg:$src, (vtype 0), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Reg") cc, reg:$src))>;
  def _Idx_Imm : Pat<(brcond (i1 (setcc (vtype 0), (am_am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Idx_Imm") cc, INDEX16:$idx, i16imm:$offset))>;
  def _Idx_Reg8 : Pat<(brcond (i1 (setcc (vtype 0), (am_am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Idx_Reg8") cc, INDEX16:$idx, ACC8:$offset))>;
  def _Idx_Reg16 : Pat<(brcond (i1 (setcc (vtype 0), (am_am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Idx_Reg16") cc, INDEX16:$idx, ACC16:$offset))>;
}

multiclass MC6809TestBraCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809TestBraPat<CC, ccode, i8, ACC8_AB>;
  defm _i16 : MC6809TestBraPat<CC, ccode, i16, ACC16>;
  defm _i32 : MC6809TestBraPat<CC, ccode, i32, ACC32>;
}

defm TestBraPat_HI : MC6809TestBraCondPat<SETUGT, 2>;
defm TestBraPat_HS : MC6809TestBraCondPat<SETUGE, 4>;
defm TestBraPat_LS : MC6809TestBraCondPat<SETULE, 3>;
defm TestBraPat_LO : MC6809TestBraCondPat<SETULT, 5>;
defm TestBraPat_GT : MC6809TestBraCondPat<SETGT, 14>;
defm TestBraPat_GE : MC6809TestBraCondPat<SETGE, 12>;
defm TestBraPat_LE : MC6809TestBraCondPat<SETLE, 15>;
defm TestBraPat_LT : MC6809TestBraCondPat<SETLT, 13>;
defm TestBraPat_EQ : MC6809TestBraCondPat<SETEQ, 7>;
defm TestBraPat_NE : MC6809TestBraCondPat<SETNE, 6>;

//===---------------------------------------------------------------------===//
// Pointer instructions
//===---------------------------------------------------------------------===//

class MC6809PtrAdd<RegisterClass idx, dag operand> : MC6809LogicalInstr {
  dag OutOperandList = (outs idx:$dst);
  dag InOperandList = !con((ins idx:$src), operand);
  let isAdd = true;
  let Constraints = "$dst = $src";
}

// LEAr n,r or LEAr a,r
def LEAPtrAdd_Imm : MC6809PtrAdd<INDEX16, (ins i16imm:$offset)>;
def LEAPtrAdd_Reg8 : MC6809PtrAdd<INDEX16, (ins ACC8:$offset)>;
def LEAPtrAdd_Reg16 : MC6809PtrAdd<INDEX16, (ins ACC16:$offset)>;

// def LEAPtrAddPatImm : Pat<(i16 (ptradd INDEX16:$src, imm:$offset)), (LEAPtrAdd_Imm INDEX16:$src, imm:$offset)>;
def LEAPtrAddPatReg8 : Pat<(i16 (ptradd INDEX16:$src, ACC8:$offset)), (LEAPtrAdd_Reg8 INDEX16:$src, ACC8:$offset)>;
def LEAPtrAddPatReg16 : Pat<(i16 (ptradd INDEX16:$src, ACC16:$offset)), (LEAPtrAdd_Reg16 INDEX16:$src, ACC16:$offset)>;

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
def Pull8 : MC6809StackPull<ACC8>;
def Push16 : MC6809StackPush<ACC16>;
def Pull16 : MC6809StackPull<ACC16>;
def PushPtr : MC6809StackPush<INDEX16>;
def PullPtr : MC6809StackPull<INDEX16>;
let Predicates = [IsHD6309] in {
  def Push32 : MC6809StackPush<ACC32>;
  def Pull32 : MC6809StackPull<ACC32>;
}

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def SEXCC8Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins CCFlag:$bit);
  dag OutOperandList = (outs ACC8:$dst);
  //let Pattern = [(set ACC8:$dst, (sext CCFlag:$bit))];
}

def SEX8Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins BIT1:$src);
  dag OutOperandList = (outs ACC8:$dst);
  let Pattern = [(set ACC8:$dst, (sext BIT1:$src))];
}

def SEX16Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ABc:$src);
  dag OutOperandList = (outs ADc:$dst);
  let Pattern = [(set ADc:$dst, (sext ABc:$src))];
}

def SEX32Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins AWc:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (sext AWc:$src))];
  let Predicates = [IsHD6309];
}

def ZEXCC8Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins CCFlag:$bit);
  dag OutOperandList = (outs ACC8:$dst);
  //let Pattern = [(set ACC8:$dst, (zext CCFlag:$bit))];
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
  dag InOperandList = (ins AWc:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (zext AWc:$src))];
  let Predicates = [IsHD6309];
}

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def : Pat<(i8 (anyext BIT1:$bit)), (INSERT_SUBREG (IMPLICIT_DEF), BIT1:$bit, sub_lsb)>;
def : Pat<(i8 (zext CARRY:$bit)), (INSERT_SUBREG (IMPLICIT_DEF), CARRY:$bit, sub_lsb)>;
def : Pat<(i16 (anyext ACC8:$byte)), (INSERT_SUBREG (IMPLICIT_DEF), ACC8:$byte, sub_lo_byte)>;
def : Pat<(i16 (anyext BIT1:$bit)), (INSERT_SUBREG (IMPLICIT_DEF), (INSERT_SUBREG (i8 (IMPLICIT_DEF)), BIT1:$bit, sub_lsb), sub_lo_byte)>;

//def : Pat<(i1 (trunc ACC8:$byte)), (EXTRACT_SUBREG ACC8:$byte, sub_lsb)>;
def : Pat<(i8 (trunc ACC16:$word)), (EXTRACT_SUBREG ACC16:$word, sub_lo_byte)>;
let Predicates = [IsHD6309] in {
  def : Pat<(i16 (trunc ACC32:$dword)), (EXTRACT_SUBREG ACC32:$dword, sub_lo_word)>;
}

//===---------------------------------------------------------------------===//
// Constant Patterns
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Oddball instructions
//===---------------------------------------------------------------------===//

//def LoadCC : MC6809LogicalInstr {
//  dag InOperandList = (ins ACC8:$src);
//  dag OutOperandList = (outs CCond:$dst);
//}

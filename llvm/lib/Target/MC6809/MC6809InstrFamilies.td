//===-- MC6809InstrFamilies.td - Instruction Family Groupings --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Groups pseudo-instructions, concrete instructions, and patterns for each
// instruction family together for maintainability. Pseudo-instructions are
// register-agnostic (used by GlobalISel), concrete instructions are
// register-specific (used by assembler/disassembler), and patterns connect
// LLVM IR operations to pseudo-instructions.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Load Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

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
  let mayLoad = true in {
    def _Mem : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
  }
}

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

// --- Concrete instructions (register-specific, assembler/disassembler) ---

// 8-bit loads: LDA/LDB (6809 Page 1)
defm LDA : Load8Concrete<"lda", Opcode<0x86,1>, Opcode<0x96,1>,
                          Opcode<0xA6,1>, Opcode<0xB6,1>,
                          [NZ,N,Z,V,AA,AD,AQ]>;
defm LDB : Load8Concrete<"ldb", Opcode<0xC6,1>, Opcode<0xD6,1>,
                          Opcode<0xE6,1>, Opcode<0xF6,1>,
                          [NZ,N,Z,V,AB,AD,AQ]>;

// 16-bit loads: LDD/LDX/LDU (6809 Page 1)
defm LDD : Load16Concrete<"ldd", Opcode<0xCC,1>, Opcode<0xDC,1>,
                           Opcode<0xEC,1>, Opcode<0xFC,1>,
                           [NZ,N,Z,V,AA,AB,AD,AQ]>;
defm LDX : Load16Concrete<"ldx", Opcode<0x8E,1>, Opcode<0x9E,1>,
                           Opcode<0xAE,1>, Opcode<0xBE,1>,
                           [NZ,N,Z,V,IX]>;
defm LDU : Load16Concrete<"ldu", Opcode<0xCE,1>, Opcode<0xDE,1>,
                           Opcode<0xEE,1>, Opcode<0xFE,1>,
                           [NZ,N,Z,V,SU]>;

// 8-bit loads: LDE/LDF (6309 Page 3)
defm LDE : Load8Concrete_P3<"lde", Opcode<0x86,3>, Opcode<0x96,3>,
                             Opcode<0xA6,3>, Opcode<0xB6,3>,
                             [NZ,N,Z,V,AE,AW,AQ]>;
defm LDF : Load8Concrete_P3<"ldf", Opcode<0xC6,3>, Opcode<0xD6,3>,
                             Opcode<0xE6,3>, Opcode<0xF6,3>,
                             [NZ,N,Z,V,AF,AW,AQ]>;

// 16-bit loads: LDY/LDS (6809 Page 2), LDW (6309 Page 2)
defm LDY : Load16Concrete_P2<"ldy", Opcode<0x8E,2>, Opcode<0x9E,2>,
                              Opcode<0xAE,2>, Opcode<0xBE,2>,
                              [NZ,N,Z,V,IY]>;
defm LDS : Load16Concrete_P2<"lds", Opcode<0xCE,2>, Opcode<0xDE,2>,
                              Opcode<0xEE,2>, Opcode<0xFE,2>,
                              [NZ,N,Z,V,SS]>;
let Predicates = [IsHD6309] in {
  defm LDW : Load16Concrete_P2<"ldw", Opcode<0x86,2>, Opcode<0x96,2>,
                                Opcode<0xA6,2>, Opcode<0xB6,2>,
                                [NZ,N,Z,V,AE,AF,AW,AQ]>;
}

// LDQ (6309, 32-bit): immediate on Page 1, dir/idx/ext on Page 2.
let Predicates = [IsHD6309], mayLoad = true in {
  def LDQi32 : MC6809Immediate32Bit_P1<(outs), "ldq", Opcode<0xCD,1>,
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
  def LDQd   : MC6809Direct_P2<(outs), "ldq", Opcode<0xDC,2>,
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
  defm LDQi  : MC6809Indexed_P2<"ldq", Opcode<0xEC,2>, (outs),
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
  def LDQe   : MC6809Extended_P2<(outs), "ldq", Opcode<0xFC,2>,
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
}

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

def : Pat<(i1 imm:$s), (Load_i1_Imm imm:$s)>;

multiclass MC6809LoadPat<ValueType vtype> {
  def _#vtype#_Imm : Pat<(vtype imm:$val), (!cast<Instruction>("Load_"#vtype#"_Imm") imm:$val)>;
  def _#vtype#_Idx_Zero : Pat<(vtype (load INDEX16:$idx)), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, (i16 0))>;
  def _#vtype#_Idx_Imm : Pat<(vtype (load (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Unmerge_Idx_Imm : Pat<(vtype (load (am_ls_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Idx_Reg8 : Pat<(vtype (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, ACC8:$offset)>;
  def _#vtype#_Idx_Reg16 : Pat<(vtype (load (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, ACC16:$offset)>;
  def _#vtype#_Frame_Index : Pat<(vtype (load (am_ls_frame_index INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, i16imm:$offset)>;
}

defm LoadPat_i8 : MC6809LoadPat<i8>;
defm LoadPat_i16 : MC6809LoadPat<i16>;
let Predicates = [IsHD6309] in {
  defm LoadPat_i32 : MC6809LoadPat<i32>;
}

//===----------------------------------------------------------------------===//
// Store Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayStore = true;
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins src:$src), operand);
}

multiclass MC6809Store<RegisterClass src> {
  def _Mem : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
}

defm Store_i8 : MC6809Store<ACC8>;
defm Store_i16 : MC6809Store<ACC16>;
defm Store_iPtr : MC6809Store<INDEX16>;
let Predicates = [IsHD6309] in {
  defm Store_i32 : MC6809Store<ACC32>;
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---

// 8-bit stores: STA/STB (6809 Page 1)
defm STA : Store8Concrete<"sta", Opcode<0x97,1>, Opcode<0xA7,1>, Opcode<0xB7,1>>;
defm STB : Store8Concrete<"stb", Opcode<0xD7,1>, Opcode<0xE7,1>, Opcode<0xF7,1>>;

// 16-bit stores: STD/STX/STU (6809 Page 1)
defm STD : Store16Concrete<"std", Opcode<0xDD,1>, Opcode<0xED,1>, Opcode<0xFD,1>>;
defm STX : Store16Concrete<"stx", Opcode<0x9F,1>, Opcode<0xAF,1>, Opcode<0xBF,1>>;
defm STU : Store16Concrete<"stu", Opcode<0xDF,1>, Opcode<0xEF,1>, Opcode<0xFF,1>>;

// 8-bit stores: STE/STF (6309 Page 3)
defm STE : Store8Concrete_P3<"ste", Opcode<0x97,3>, Opcode<0xA7,3>, Opcode<0xB7,3>>;
defm STF : Store8Concrete_P3<"stf", Opcode<0xD7,3>, Opcode<0xE7,3>, Opcode<0xF7,3>>;

// 16-bit stores: STY/STS (6809 Page 2), STW (6309 Page 2)
defm STY : Store16Concrete_P2<"sty", Opcode<0x9F,2>, Opcode<0xAF,2>, Opcode<0xBF,2>>;
defm STS : Store16Concrete_P2<"sts", Opcode<0xDF,2>, Opcode<0xEF,2>, Opcode<0xFF,2>>;
let Predicates = [IsHD6309] in {
  defm STW : Store16Concrete_P2<"stw", Opcode<0x97,2>, Opcode<0xA7,2>, Opcode<0xB7,2>>;
}

// STQ (6309, 32-bit): dir/idx/ext on Page 2.
let Predicates = [IsHD6309], mayStore = true in {
  def STQd  : MC6809Direct_P2<(outs), "stq", Opcode<0xDD,2>, [NZ,N,Z,V], []>;
  defm STQi : MC6809Indexed_P2<"stq", Opcode<0xED,2>, (outs), [NZ,N,Z,V], []>;
  def STQe  : MC6809Extended_P2<(outs), "stq", Opcode<0xFD,2>, [NZ,N,Z,V], []>;
}

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809StorePat<ValueType vtype, RegisterClass reg> {
  def _#vtype#_Idx_Zero : Pat<(store reg:$src, INDEX16:$idx), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, (i16 0))>;
  def _#vtype#_Idx_Imm : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Unmerge_Idx_Imm : Pat<(store reg:$src, (am_ls_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Idx_Reg8 : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC8:$offset)>;
  def _#vtype#_Idx_Reg16 : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC16:$offset)>;
  def _#vtype#_Frame_Index : Pat<(store reg:$src, (am_ls_frame_index INDEX16:$idx, ACC16:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, i16imm:$offset)>;
}

defm StorePat_i8 : MC6809StorePat<i8, ACC8>;
defm StorePat_i16 : MC6809StorePat<i16, ACC16>;
defm StorePat_iPtr : MC6809StorePat<i16, INDEX16>;
let Predicates = [IsHD6309] in {
  defm StorePat_i32 : MC6809StorePat<i32, ACC32>;
}

//===----------------------------------------------------------------------===//
// Arithmetic (Add/Sub) Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

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
    def _Mem : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809ArithmeticBase<dst, src, (ins STACK16:$idx)>;
    let Predicates = [IsHD6309] in {
      def _Reg : MC6809ArithmeticBase<dst, src, (ins src:$src2)>;
    }
  }
}

let isAdd = true in {
  defm Add_i8 : MC6809Arithmetic<i8, ACC8, ACC8>;
  defm Add_i16 : MC6809Arithmetic<i16, ACC16, ACC16>;
}
defm Sub_i8 : MC6809Arithmetic<i8, ACC8, ACC8>;
defm Sub_i16 : MC6809Arithmetic<i16, ACC16, ACC16>;

// --- Carry-setting variants ---

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
    def _Mem : MC6809ArithmeticBaseCarry<dst, carry, src, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809ArithmeticBaseCarry<dst, carry, src, (ins STACK16:$idx)>;
    let Predicates = [IsHD6309] in {
      def _Reg : MC6809ArithmeticBaseCarry<dst, carry, src, (ins src:$src2)>;
    }
  }
}

let isAdd = true in {
  defm AddSetCarry_i8 : MC6809ArithmeticCarry<i8, ACC8, BIT1, ACC8>;
  defm AddSetCarry_i16 : MC6809ArithmeticCarry<i16, ACC16, BIT1, ACC16>;
}
defm SubSetCarry_i8 : MC6809ArithmeticCarry<i8, ACC8, BIT1, ACC8>;
defm SubSetCarry_i16 : MC6809ArithmeticCarry<i16, ACC16, BIT1, ACC16>;

// --- Carry-using variants ---

class MC6809ArithmeticBaseCarryUse<RegisterClass dst, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs dst:$dst, BIT1:$carry);
  dag InOperandList = !con((ins src:$src, BIT1:$carry_in), operand);
  let Constraints = "$dst = $src, $carry = $carry_in";
}

multiclass MC6809ArithmeticCarryUse<ValueType vtype, RegisterClass dst, RegisterClass src> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBaseCarryUse<dst, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809ArithmeticBaseCarryUse<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    let Predicates = [IsHD6309] in {
      def _Reg : MC6809ArithmeticBaseCarryUse<dst, src, (ins src:$src2)>;
    }
  }
}

let isAdd = true in {
  defm AddSetCarryUse_i8 : MC6809ArithmeticCarryUse<i8, ACC8, ACC8>;
  defm AddSetCarryUse_i16 : MC6809ArithmeticCarryUse<i16, ACC16, ACC16>;
}

defm SubSetCarryUse_i8 : MC6809ArithmeticCarryUse<i8, ACC8, ACC8>;
defm SubSetCarryUse_i16 : MC6809ArithmeticCarryUse<i16, ACC16, ACC16>;

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete arithmetic instructions (ADDA, ADDB, ADDD, SUBA, SUBB, SUBD, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809ArithPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Imm : Pat<(vtype (node reg:$src, imm:$val)), (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val)>;
  def _Idx_Imm : Pat<(vtype (node reg:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset)>;
  def _Unmerge_Idx_Imm : Pat<(vtype (node reg:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset)>;
  def _Idx_Reg8 : Pat<(vtype (node reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(vtype (node reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC16:$offset)>;
  let Predicates = [IsHD6309] in {
    def _Reg : Pat<(vtype (node reg:$src, reg:$src2)), (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, reg:$src2)>;
  }
  let Predicates = [NotHD6309] in {
    def _Push_Pull : Pat<(vtype (node reg:$src, reg:$src2)), (!cast<Instruction>(inst#"_"#vtype#"_Pull") reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2))>;
  }
}

multiclass MC6809ArithRevPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Imm : Pat<(vtype (node imm:$val, reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val))>;
  def _Idx_Imm : Pat<(vtype (node (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset))>;
  def _Unmerge_Idx_Imm : Pat<(vtype (node (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset))>;
  def _Idx_Reg8 : Pat<(vtype (node (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC8:$offset))>;
  def _Idx_Reg16 : Pat<(vtype (node (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC16:$offset))>;
  let Predicates = [IsHD6309] in {
    def _Reg : Pat<(vtype (node reg:$src2, reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, reg:$src2)))>;
  }
  let Predicates = [NotHD6309] in {
    def _Push_Pull : Pat<(vtype (node reg:$src2, reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Pull") reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2))))>;
  }
}

//multiclass MC6809ArithRevPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
//  def _Imm : Pat<(vtype (node (am_immediate !cast<Operand>(vtype#"imm"):$val), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val))>;
//  let Predicates = [IsHD6309] in {
//    def _Reg : Pat<(vtype (node reg:$src, reg:$src2)), (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, reg:$src2)>;
//  }
//  let Predicates = [NotHD6309] in {
//    def _Push_Pull : Pat<(vtype (node reg:$src, reg:$src2)), (!cast<Instruction>(inst#"_"#vtype#"_Pull") reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2))>;
//  }
//}

defm AddPat_i8 : MC6809ArithPat<add, i8, ACC8, "Add">;
defm SubPat_i8 : MC6809ArithPat<sub, i8, ACC8, "Sub">;
defm SubRevPat_i8 : MC6809ArithRevPat<sub, i8, ACC8, "Sub">;

defm AddPat_ii6 : MC6809ArithPat<add, i16, ACC16, "Add">;
defm SubPat_ii6 : MC6809ArithPat<sub, i16, ACC16, "Sub">;
defm SubRevPat_ii6 : MC6809ArithRevPat<sub, i16, ACC16, "Sub">;

// LEA-based i16 add/sub for INDEX registers is handled in
// MC6809InstructionSelector.cpp (hand-lowering required for GlobalISel
// because constants are in registers, not bare immediates).

//===----------------------------------------------------------------------===//
// Bitwise (AND/OR/XOR) Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

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
//  let Predicates = [IsHD6309] in {
    def _Reg : MC6809BitwiseBase<reg, (ins reg:$src2)>;
//  }
  let mayLoad = true in {
    def _Mem : MC6809BitwiseBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809BitwiseBase<reg, (ins STACK16:$idx)>;
  }
}

defm AND_i1 : MC6809Bitwise<i8, BIT1>;
defm OR_i1 : MC6809Bitwise<i8, BIT1>;
defm XOR_i1 : MC6809Bitwise<i8, BIT1>;
defm AND_i8 : MC6809Bitwise<i8, ACC8>;
defm OR_i8 : MC6809Bitwise<i8, ACC8>;
defm XOR_i8 : MC6809Bitwise<i8, ACC8>;
let Predicates = [IsHD6309] in {
    defm AND_i16 : MC6809Bitwise<i16, ACC16>;
    defm OR_i16 : MC6809Bitwise<i16, ACC16>;
    defm XOR_i16 : MC6809Bitwise<i16, ACC16>;
}

// Bit-testing (commented out — preserved for future use)

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

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete bitwise instructions (ANDA, ANDB, ORA, ORB, EORA, EORB, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---
// Bitwise patterns reuse MC6809ArithPat from the Arithmetic family above.

defm ANDPat_i8 : MC6809ArithPat<and, i8, ACC8, "AND">;
defm ORPat_i8 : MC6809ArithPat<or, i8, ACC8, "OR">;
defm XORPat_i8 : MC6809ArithPat<xor, i8, ACC8, "XOR">;
defm ANDPat_i16 : MC6809ArithPat<and, i16, ACC16, "AND">;
defm ORPat_i16 : MC6809ArithPat<or, i16, ACC16, "OR">;
defm XORPat_i16 : MC6809ArithPat<xor, i16, ACC16, "XOR">;

//===----------------------------------------------------------------------===//
// Shift/Rotate Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

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

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete shift instructions (ASLA, ASLB, ASRA, ASRB, LSLA, LSLB, LSRA, LSRB, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809ShiftPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Reg : Pat<(vtype (node reg:$src, (i8 imm:$val))), (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, imm:$val)>;
//  def _Idx_Imm : Pat<(vtype (node reg:$src, (i1 1))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Imm") reg:$src, INDEX16:$idx, imm:$offset)>;
//  def _Idx_Reg8 : Pat<(vtype (node reg:$src, (i1 1))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Reg8") reg:$src, INDEX16:$idx, ACC8:$offset)>;
//  def _Idx_Reg16 : Pat<(vtype (node reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Idx_Reg16") reg:$src, INDEX16:$idx, ACC16:$offset)>;
}

defm LSL_Pat_i8 : MC6809ShiftPat<shl, i8, ACC8_AB, "LSL">;
defm LSL_Pat_i16 : MC6809ShiftPat<shl, i16, ADc, "LSL">;

defm ASR_Pat_i8 : MC6809ShiftPat<sra, i8, ACC8_AB, "ASR">;
defm ASR_Pat_i16 : MC6809ShiftPat<sra, i16, ADc, "ASR">;
defm LSR_Pat_i8 : MC6809ShiftPat<srl, i8, ACC8_AB, "LSR">;
defm LSR_Pat_i16 : MC6809ShiftPat<srl, i16, ADc, "LSR">;

//===----------------------------------------------------------------------===//
// Compare Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

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
    def _Mem : MC6809CompareBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809CompareBase<reg, (ins STACK16:$idx)>;
  }
}

defm Compare_i8 : MC6809Compare<i8, ACC8>;
defm Compare_i16 : MC6809Compare<i16, ACC16>;
defm Compare_ptr: MC6809Compare<i16, INDEX16>;

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete compare instructions (CMPA, CMPB, CMPD, CMPX, CMPY, CMPU, CMPS, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809SetIfCondPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _i1_Imm : Pat<(i1 (setcc reg:$src, imm:$val, CC)), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Imm") cc, reg:$src, imm:$val), 1, 0)>;
  def _i1_Idx_Imm : Pat<(i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, imm:$offset), 1, 0)>;
  def _i1_Unmerge_Idx_Imm : Pat<(i1 (setcc reg:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, imm:$offset), 1, 0)>;
  def _i1_Idx_Reg8 : Pat<(i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC8:$offset), 1, 0)>;
  def _i1_Idx_Reg16 : Pat<(i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC16:$offset), 1, 0)>;
  let Predicates = [IsHD6309] in {
    def _i1_Reg : Pat<(i1 (setcc reg:$src, reg:$src2, CC)), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Reg") cc, reg:$src, reg:$src2), 1, 0)>;
  }
  let Predicates = [NotHD6309] in {
    def _i1_Push_Pull : Pat<(i1 (setcc reg:$src, reg:$src2, CC)), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Pull") cc, reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2)), 1, 0)>;
  }
}

multiclass MC6809SetIfCompCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809SetIfCondPat<CC, ccode, i8, ACC8>;
  defm _i16 : MC6809SetIfCondPat<CC, ccode, i16, ACC16>;
  defm _ptr : MC6809SetIfCondPat<CC, ccode, i16, INDEX16>;
}

//defm CompPat_HI : MC6809CompCondPat<SETUGT, 2>;
//defm CompPat_HS : MC6809CompCondPat<SETUGE, 4>;
//defm CompPat_LS : MC6809CompCondPat<SETULE, 3>;
//defm CompPat_LO : MC6809CompCondPat<SETULT, 5>;
//defm CompPat_GT : MC6809CompCondPat<SETGT, 14>;
//defm CompPat_GE : MC6809CompCondPat<SETGE, 12>;
//defm CompPat_LE : MC6809CompCondPat<SETLE, 15>;
//defm CompPat_LT : MC6809CompCondPat<SETLT, 13>;
//defm CompPat_EQ : MC6809CompCondPat<SETEQ, 7>;
//defm CompPat_NE : MC6809CompCondPat<SETNE, 6>;

defm SetIfCompPat_HI : MC6809SetIfCompCondPat<SETUGT, 2>;
defm SetIfCompPat_HS : MC6809SetIfCompCondPat<SETUGE, 4>;
defm SetIfCompPat_LS : MC6809SetIfCompCondPat<SETULE, 3>;
defm SetIfCompPat_LO : MC6809SetIfCompCondPat<SETULT, 5>;
defm SetIfCompPat_GT : MC6809SetIfCompCondPat<SETGT, 14>;
defm SetIfCompPat_GE : MC6809SetIfCompCondPat<SETGE, 12>;
defm SetIfCompPat_LE : MC6809SetIfCompCondPat<SETLE, 15>;
defm SetIfCompPat_LT : MC6809SetIfCompCondPat<SETLT, 13>;
defm SetIfCompPat_EQ : MC6809SetIfCompCondPat<SETEQ, 7>;
defm SetIfCompPat_NE : MC6809SetIfCompCondPat<SETNE, 6>;

//===----------------------------------------------------------------------===//
// Test Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809TestBase<dag operand> : MC6809LogicalInstr {
  //let Defs = [NZVC, NZ, N, Z, V, C];
  dag OutOperandList = (outs CCond:$dst);
  dag InOperandList = !con((ins i8imm:$cc), operand);
  let isCompare = true;
}

multiclass MC6809Test<RegisterClass reg> {
  def _Reg : MC6809TestBase<(ins reg:$src)>;
  let mayLoad = true in {
    def _Mem : MC6809TestBase<(ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm Test_i8 : MC6809Test<ACC8_AB>;
let Predicates = [IsHD6309] in {
  defm Test_i16 : MC6809Test<ACC16>;
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete test instructions (TSTA, TSTB, TSTD, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809TestPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(i1 (setcc (vtype 0), reg:$src, CC)), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Reg") cc, reg:$src), 1, 0)>;
  def _Idx_Imm : Pat<(i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset), 1, 0)>;
  def _Unmerge_Idx_Imm : Pat<(i1 (setcc (vtype 0), (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset), 1, 0)>;
  def _Idx_Reg8 : Pat<(i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, ACC8:$offset), 1, 0)>;
  def _Idx_Reg16 : Pat<(i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, ACC16:$offset), 1, 0)>;
}

multiclass MC6809TestCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809TestPat<CC, ccode, i8, ACC8_AB>;
  let Predicates = [IsHD6309] in {
    defm _i16 : MC6809TestPat<CC, ccode, i16, ACC16>;
  }
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

//===----------------------------------------------------------------------===//
// BitTest Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

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
    def _Mem : MC6809BitTestBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm BitTest_i8 : MC6809BitTest<i8, ACC8_AB>;
let Predicates = [IsHD6309] in {
  defm BitTest_i16 : MC6809BitTest<i16, ACC16_D>;
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete bit-test instructions (BITA, BITB, BITD, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

//multiclass MC6809BitTestPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
//  def _Imm : Pat<(i8 (setcc (vtype 1), imm:$val, CC)), (!cast<Instruction>("BitTest_"#vtype#"_Imm") cc, reg:$src, imm:$val)>;
//  def _Idx_Imm : Pat<(i8 (setcc (vtype 1), (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (!cast<Instruction>("BitTest_"#vtype#"_Mem") cc, (Load_i8_Imm, INDEX16:$idx, i16imm:$offset)>;
//  def _Unmerge_Idx_Imm : Pat<(i8 (setcc (vtype 1), (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), (!cast<Instruction>("BitTest_"#vtype#"_Mem") cc, (Load_i8_Imm, INDEX16:$idx, i16imm:$offset)>;
//  def _Idx_Reg8 : Pat<(i8 (setcc (vtype 1), (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), (!cast<Instruction>("BitTest_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC8:$offset)>;
//  def _Idx_Reg16 : Pat<(i8 (setcc (vtype 1), (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), (!cast<Instruction>("BitTest_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC16:$offset)>;
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

//===----------------------------------------------------------------------===//
// Multiply Family
//===----------------------------------------------------------------------===//

// --- 8-bit multiply pseudo-instruction ---
// The 6809 MUL instruction multiplies A × B → D (16-bit result).
// We model this as a single pseudo taking D as input/output to avoid
// the D/A/B register aliasing problem: if A and B are separate operands,
// the register allocator can't handle the fact that loading one (via LDD
// or TFR) clobbers the other through D.
//
// Patterns use REG_SEQUENCE to pack the two i8 operands into D, then
// EXTRACT_SUBREG to pull the result byte out.

class MC6809MultiplyD : MC6809LogicalInstr {
  // Always clobbers the real accumulator (AA, AB, AD) because the expansion
  // loads into D, does MUL, then stores back — even when the operand is a
  // spill register. Without these implicit defs, the allocator could leave
  // values in A/B that get destroyed during the MUL expansion.
  let Defs = [Z, C, AA, AB, AD];
  dag OutOperandList = (outs ADc:$dst);
  dag InOperandList = (ins ADc:$src);
  let Constraints = "$dst = $src";
}

def MUL_D : MC6809MultiplyD;

// --- 8-bit multiply patterns ---
// Pack two i8 operands into D via REG_SEQUENCE, multiply, extract result.
// mul  → low byte (B = sub_lo_byte)
// mulhu/mulhs → high byte (A = sub_hi_byte)
def MulPat_i8 : Pat<(i8 (mul ACC8:$b, ACC8:$a)),
  (EXTRACT_SUBREG
    (MUL_D (REG_SEQUENCE ADc, ACC8:$b, sub_lo_byte, ACC8:$a, sub_hi_byte)),
    sub_lo_byte)>;

def MulHUPat_i8 : Pat<(i8 (mulhu ACC8:$b, ACC8:$a)),
  (EXTRACT_SUBREG
    (MUL_D (REG_SEQUENCE ADc, ACC8:$b, sub_lo_byte, ACC8:$a, sub_hi_byte)),
    sub_hi_byte)>;

def MulHSPat_i8 : Pat<(i8 (mulhs ACC8:$b, ACC8:$a)),
  (EXTRACT_SUBREG
    (MUL_D (REG_SEQUENCE ADc, ACC8:$b, sub_lo_byte, ACC8:$a, sub_hi_byte)),
    sub_hi_byte)>;

// --- 16-bit multiply pseudo-instructions (6309 only) ---

class MC6809MultiplyBase<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, AD];
  dag InOperandList = !con((ins ADc:$src), operand);
  dag OutOperandList = (outs AWc:$dst);
}

multiclass MC6809Multiply {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809MultiplyBase<(ins i16imm:$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809MultiplyBase<(ins INDEX16:$idx, unknown:$offset)>;
    def _Reg : MC6809MultiplyBase<(ins ACC16:$src2)>;
  }
}

let Predicates = [IsHD6309] in {
  defm Multiply_i16 : MC6809Multiply;
}

class MC6809MultiplyHighBase<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, AW];
  dag OutOperandList = (outs ADc:$dst);
  dag InOperandList = !con((ins ADc:$src), operand);
}

multiclass MC6809MultiplyHigh {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809MultiplyHighBase<(ins i16imm:$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809MultiplyHighBase<(ins INDEX16:$idx, unknown:$offset)>;
    def _Reg : MC6809MultiplyHighBase<(ins ACC16:$src2)>;
  }
}

let Predicates = [IsHD6309] in {
  defm MultiplyHigh_i16 : MC6809MultiplyHigh;
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete multiply instructions (MUL, MULD, etc.)
// are defined in MC6809InstrInfo.td.

// --- 16-bit multiply patterns ---

multiclass MC6809MultiplyPat<SDNode node, string inst> {
  def _Imm : Pat<(i16 (node ADc:$src, imm:$val)), (!cast<Instruction>(inst#"_i16_Imm") ADc:$src, imm:$val)>;
  def _Idx_Imm : Pat<(i16 (node ADc:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, imm:$offset)>;
  def _Unmerge_Idx_Imm : Pat<(i16 (node ADc:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, imm:$offset)>;
  def _Idx_Reg8 : Pat<(i16 (node ADc:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(i16 (node ADc:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, ACC16:$offset)>;
  def _Reg : Pat<(i16 (node ADc:$src, ACC16:$src2)), (!cast<Instruction>(inst#"_i16_Reg") ADc:$src, ACC16:$src2)>;
}

let Predicates = [IsHD6309] in {
  defm MulPat_i16 : MC6809MultiplyPat<mul, "Multiply">;
  defm MulHSPat_i16 : MC6809MultiplyPat<mulhs, "MultiplyHigh">;
  defm MulHUPat_i16 : MC6809MultiplyPat<mulhu, "MultiplyHigh">;
}

//===----------------------------------------------------------------------===//
// Fused Compare/Branch Patterns
//===----------------------------------------------------------------------===//

multiclass MC6809CompBraPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Imm : Pat<(brcond (i1 (setcc reg:$src, imm:$val, CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Imm") cc, reg:$src, imm:$val))>;
  def _Idx_Imm : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, i16imm:$offset))>;
  def _Unmerge_Idx_Imm : Pat<(brcond (i1 (setcc reg:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, imm:$offset))>;
  def _Idx_Reg8 : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC8:$offset))>;
  def _Idx_Reg16 : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC16:$offset))>;
  let Predicates = [IsHD6309] in {
    def _Reg : Pat<(brcond (i1 (setcc reg:$src, reg:$src2, CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Reg") cc, reg:$src, reg:$src2))>;
  }
  let Predicates = [NotHD6309] in {
    def _Push_Pull : Pat<(brcond (i1 (setcc reg:$src, reg:$src2, CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Compare_"#vtype#"_Pull") cc, reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2)))>;
  }
}

multiclass MC6809CompBraCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809CompBraPat<CC, ccode, i8, ACC8>;
  defm _i16 : MC6809CompBraPat<CC, ccode, i16, ACC16>;
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

//===----------------------------------------------------------------------===//
// Fused Test/Branch Patterns
//===----------------------------------------------------------------------===//

multiclass MC6809TestBraPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(brcond (i1 (setcc reg:$src, (vtype 0), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Reg") cc, reg:$src))>;
  def _Idx_Imm : Pat<(brcond (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset))>;
  def _Unmerge_Idx_Imm : Pat<(brcond (i1 (setcc (vtype 0), (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset))>;
  def _Idx_Reg8 : Pat<(brcond (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, ACC8:$offset))>;
  def _Idx_Reg16 : Pat<(brcond (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), bb:$tgt), (ConditionalBranchRelative cc, bb:$tgt, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, ACC16:$offset))>;
}

multiclass MC6809TestBraCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809TestBraPat<CC, ccode, i8, ACC8_AB>;
  defm _i16 : MC6809TestBraPat<CC, ccode, i16, ACC16>;
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

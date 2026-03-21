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

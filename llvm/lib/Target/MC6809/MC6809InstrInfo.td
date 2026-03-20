//===-- MC6809InstrInfo.td - MC6809 Instruction defs -------------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the MC6809 instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

/// These instruction format definitions exist, thanks to Neil Parker's
/// analysis of the 65xx instruction set, at:
/// http://nparker.llx.com/a2/opcodes.html
/// Parker's analysis is closely reflected in the structure of this file, and
/// his words are quoted liberally herein.  Parker's sense-making of the 6502
/// and family was critical in creating these format definitions, and we're
/// indebted to him for the excellent analysis.
///
/// These instruction definitions provide the instruction set for the LLVM MC
/// layer. They correspond to the real instruction set of the 6502. Given their
/// irregularity, the side effects and behavior of these instructions are not
/// themselves modeled. Instead, a regularized virtual instruction set was formed
/// out of pseudo-instructions, and these were fully modelled for the code
/// generator's purposes. Lowering of these pseudoinstructions occurs in
/// AsmPrinter, which is the boundary between the CodeGen and MC layers.

include "MC6809InstrFormats.td"

// clang-format off
// MRVM START MARKER 1
// ============================== PAGE MC6809ImmediateIndexed_P1 ==================================
multiclass MC6809ImmediateIndexed_P1<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {
  def _o0 : MC6809ImmediateIndexedOffset0_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o5 : MC6809ImmediateIndexedOffset5_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o8 : MC6809ImmediateIndexedOffset8_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16 : MC6809ImmediateIndexedOffset16_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oA : MC6809ImmediateIndexedOffsetA_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])> {
    let Predicates = [IsHD6309];
  }
  def _oB : MC6809ImmediateIndexedOffsetB_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])> {
    let Predicates = [IsHD6309];
  }
  def _oE : MC6809ImmediateIndexedOffsetE_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809ImmediateIndexedOffsetF_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809ImmediateIndexedOffsetD_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])> {
    let Predicates = [IsHD6309];
  }
  def _oW : MC6809ImmediateIndexedOffsetW_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc1 : MC6809ImmediateIndexedInc1_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2 : MC6809ImmediateIndexedInc2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Dec1 : MC6809ImmediateIndexedDec1_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Dec2 : MC6809ImmediateIndexedDec2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o8PC : MC6809ImmediateIndexedOffset8PC_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16PC : MC6809ImmediateIndexedOffset16PC_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o0W : MC6809ImmediateIndexedOffset0W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809ImmediateIndexedOffset16W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809ImmediateIndexedIncW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809ImmediateIndexedDecW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o0I : MC6809ImmediateIndexedOffset0Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o8I : MC6809ImmediateIndexedOffset8Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16I : MC6809ImmediateIndexedOffset16Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oAI : MC6809ImmediateIndexedOffsetAInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])> {
    let Predicates = [IsHD6309];
  }
  def _oBI : MC6809ImmediateIndexedOffsetBInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])> {
    let Predicates = [IsHD6309];
  }
  def _oEI : MC6809ImmediateIndexedOffsetEInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809ImmediateIndexedOffsetFInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809ImmediateIndexedOffsetDInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])> {
    let Predicates = [IsHD6309];
  }
  def _oWI : MC6809ImmediateIndexedOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2I : MC6809ImmediateIndexedInc2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Dec2I : MC6809ImmediateIndexedDec2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o8PCI : MC6809ImmediateIndexedOffset8PCInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16PCI : MC6809ImmediateIndexedOffset16PCInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _eI : MC6809ImmediateIndexedExtendedInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _0oWI : MC6809ImmediateIndexedNoOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809ImmediateIndexedOffset16WInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809ImmediateIndexedIncW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809ImmediateIndexedDecW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== PAGE MC6809Indexed_P1 ==================================
multiclass MC6809Indexed_P1<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {
  def _o0 : MC6809IndexedOffset0_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o5 : MC6809IndexedOffset5_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8 : MC6809IndexedOffset8_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16 : MC6809IndexedOffset16_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oA : MC6809IndexedOffsetA_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])>;
  def _oB : MC6809IndexedOffsetB_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])>;
  def _oE : MC6809IndexedOffsetE_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809IndexedOffsetF_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809IndexedOffsetD_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])>;
  def _oW : MC6809IndexedOffsetW_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc1 : MC6809IndexedInc1_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Inc2 : MC6809IndexedInc2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec1 : MC6809IndexedDec1_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2 : MC6809IndexedDec2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PC : MC6809IndexedOffset8PC_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PC : MC6809IndexedOffset16PC_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o0W : MC6809IndexedOffset0W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809IndexedOffset16W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809IndexedIncW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809IndexedDecW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o0I : MC6809IndexedOffset0Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8I : MC6809IndexedOffset8Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16I : MC6809IndexedOffset16Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oAI : MC6809IndexedOffsetAInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])>;
  def _oBI : MC6809IndexedOffsetBInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])>;
  def _oEI : MC6809IndexedOffsetEInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809IndexedOffsetFInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809IndexedOffsetDInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])>;
  def _oWI : MC6809IndexedOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2I : MC6809IndexedInc2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2I : MC6809IndexedDec2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PCI : MC6809IndexedOffset8PCInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PCI : MC6809IndexedOffset16PCInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _eI : MC6809IndexedExtendedInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _0oWI : MC6809IndexedNoOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809IndexedOffset16WInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809IndexedIncW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809IndexedDecW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== Load8 multiclass ======================================
// Produces all concrete forms of an 8-bit load: immediate, direct, indexed (all
// variants), and extended. Use with defm to generate e.g. LDAi8, LDAd, LDAi_o0,
// ..., LDAe from a single line.
// See also: Load_i8 pseudo in MC6809InstrLogical.td (register-agnostic version
// used by GlobalISel).
multiclass Load8Concrete<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                         Opcode IdxOpc, Opcode ExtOpc,
                         list<Register> Defs> {
  let mayLoad = true in {
    def i8 : MC6809Immediate8Bit_P1<(outs), Mnem, ImmOpc, Defs, []>;
    def d  : MC6809Direct_P1<(outs), Mnem, DirOpc, Defs, []>;
    defm i : MC6809Indexed_P1<Mnem, IdxOpc, (outs), Defs, []>;
    def e  : MC6809Extended_P1<(outs), Mnem, ExtOpc, Defs, []>;
  }
}

// 16-bit load: immediate/direct/indexed/extended (Page 1).
multiclass Load16Concrete<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                          Opcode IdxOpc, Opcode ExtOpc,
                          list<Register> Defs> {
  let mayLoad = true in {
    def i16 : MC6809Immediate16Bit_P1<(outs), Mnem, ImmOpc, Defs, []>;
    def d   : MC6809Direct_P1<(outs), Mnem, DirOpc, Defs, []>;
    defm i  : MC6809Indexed_P1<Mnem, IdxOpc, (outs), Defs, []>;
    def e   : MC6809Extended_P1<(outs), Mnem, ExtOpc, Defs, []>;
  }
}

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

// Store: direct/indexed/extended (Page 1). No immediate form.
multiclass Store8Concrete<string Mnem, Opcode DirOpc, Opcode IdxOpc,
                          Opcode ExtOpc> {
  let mayStore = true in {
    def d  : MC6809Direct_P1<(outs), Mnem, DirOpc, [NZ,N,Z,V], []>;
    defm i : MC6809Indexed_P1<Mnem, IdxOpc, (outs), [NZ,N,Z,V], []>;
    def e  : MC6809Extended_P1<(outs), Mnem, ExtOpc, [NZ,N,Z,V], []>;
  }
}

multiclass Store16Concrete<string Mnem, Opcode DirOpc, Opcode IdxOpc,
                           Opcode ExtOpc> {
  let mayStore = true in {
    def d  : MC6809Direct_P1<(outs), Mnem, DirOpc, [NZ,N,Z,V], []>;
    defm i : MC6809Indexed_P1<Mnem, IdxOpc, (outs), [NZ,N,Z,V], []>;
    def e  : MC6809Extended_P1<(outs), Mnem, ExtOpc, [NZ,N,Z,V], []>;
  }
}

// 8-bit stores: STA/STB (6809 Page 1)
defm STA : Store8Concrete<"sta", Opcode<0x97,1>, Opcode<0xA7,1>, Opcode<0xB7,1>>;
defm STB : Store8Concrete<"stb", Opcode<0xD7,1>, Opcode<0xE7,1>, Opcode<0xF7,1>>;

// 16-bit stores: STD/STX/STU (6809 Page 1)
defm STD : Store16Concrete<"std", Opcode<0xDD,1>, Opcode<0xED,1>, Opcode<0xFD,1>>;
defm STX : Store16Concrete<"stx", Opcode<0x9F,1>, Opcode<0xAF,1>, Opcode<0xBF,1>>;
defm STU : Store16Concrete<"stu", Opcode<0xDF,1>, Opcode<0xEF,1>, Opcode<0xFF,1>>;

// ALU 8-bit: immediate/direct/indexed/extended (Page 1).
// Used for ADD, ADC, SUB, SBC, AND, OR, EOR, BIT, CMP on 8-bit accumulators.
multiclass ALU8Concrete<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                        Opcode IdxOpc, Opcode ExtOpc,
                        list<Register> Defs, list<Register> Uses,
                        bit IsCompare = 0, bit IsAdd = 0> {
  let mayLoad = true, isCompare = IsCompare, isAdd = IsAdd in {
    def i8 : MC6809Immediate8Bit_P1<(outs), Mnem, ImmOpc, Defs, Uses>;
    def d  : MC6809Direct_P1<(outs), Mnem, DirOpc, Defs, Uses>;
    defm i : MC6809Indexed_P1<Mnem, IdxOpc, (outs), Defs, Uses>;
    def e  : MC6809Extended_P1<(outs), Mnem, ExtOpc, Defs, Uses>;
  }
}

// ALU 16-bit: immediate/direct/indexed/extended (Page 1).
multiclass ALU16Concrete<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                         Opcode IdxOpc, Opcode ExtOpc,
                         list<Register> Defs, list<Register> Uses,
                         bit IsCompare = 0, bit IsAdd = 0> {
  let mayLoad = true, isCompare = IsCompare, isAdd = IsAdd in {
    def i16 : MC6809Immediate16Bit_P1<(outs), Mnem, ImmOpc, Defs, Uses>;
    def d   : MC6809Direct_P1<(outs), Mnem, DirOpc, Defs, Uses>;
    defm i  : MC6809Indexed_P1<Mnem, IdxOpc, (outs), Defs, Uses>;
    def e   : MC6809Extended_P1<(outs), Mnem, ExtOpc, Defs, Uses>;
  }
}

// 8-bit ALU: A accumulator (Page 1)
defm SUBA : ALU8Concrete<"suba", Opcode<0x80,1>, Opcode<0x90,1>, Opcode<0xA0,1>, Opcode<0xB0,1>,
                          [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA]>;
defm CMPA : ALU8Concrete<"cmpa", Opcode<0x81,1>, Opcode<0x91,1>, Opcode<0xA1,1>, Opcode<0xB1,1>,
                          [NZ,N,Z,V,C], [AA], /*IsCompare=*/1>;
defm SBCA : ALU8Concrete<"sbca", Opcode<0x82,1>, Opcode<0x92,1>, Opcode<0xA2,1>, Opcode<0xB2,1>,
                          [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA,C]>;
defm ANDA : ALU8Concrete<"anda", Opcode<0x84,1>, Opcode<0x94,1>, Opcode<0xA4,1>, Opcode<0xB4,1>,
                          [AA,NZ,N,Z,V,AA,AD,AQ], [AA]>;
defm BITA : ALU8Concrete<"bita", Opcode<0x85,1>, Opcode<0x95,1>, Opcode<0xA5,1>, Opcode<0xB5,1>,
                          [NZ,N,Z,V], [AA], /*IsCompare=*/1>;
defm EORA : ALU8Concrete<"eora", Opcode<0x88,1>, Opcode<0x98,1>, Opcode<0xA8,1>, Opcode<0xB8,1>,
                          [AA,NZ,N,Z,V,AA,AD,AQ], [AA]>;
defm ADCA : ALU8Concrete<"adca", Opcode<0x89,1>, Opcode<0x99,1>, Opcode<0xA9,1>, Opcode<0xB9,1>,
                          [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA,C], /*IsCompare=*/0, /*IsAdd=*/1>;
defm ORA  : ALU8Concrete<"ora",  Opcode<0x8A,1>, Opcode<0x9A,1>, Opcode<0xAA,1>, Opcode<0xBA,1>,
                          [AA,NZ,N,Z,V,AA,AD,AQ], [AA]>;
defm ADDA : ALU8Concrete<"adda", Opcode<0x8B,1>, Opcode<0x9B,1>, Opcode<0xAB,1>, Opcode<0xBB,1>,
                          [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA], /*IsCompare=*/0, /*IsAdd=*/1>;

// 8-bit ALU: B accumulator (Page 1)
defm SUBB : ALU8Concrete<"subb", Opcode<0xC0,1>, Opcode<0xD0,1>, Opcode<0xE0,1>, Opcode<0xF0,1>,
                          [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB]>;
defm CMPB : ALU8Concrete<"cmpb", Opcode<0xC1,1>, Opcode<0xD1,1>, Opcode<0xE1,1>, Opcode<0xF1,1>,
                          [NZ,N,Z,V,C], [AB], /*IsCompare=*/1>;
defm SBCB : ALU8Concrete<"sbcb", Opcode<0xC2,1>, Opcode<0xD2,1>, Opcode<0xE2,1>, Opcode<0xF2,1>,
                          [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB,C]>;
defm ANDB : ALU8Concrete<"andb", Opcode<0xC4,1>, Opcode<0xD4,1>, Opcode<0xE4,1>, Opcode<0xF4,1>,
                          [AB,NZ,N,Z,V,AB,AD,AQ], [AB]>;
defm BITB : ALU8Concrete<"bitb", Opcode<0xC5,1>, Opcode<0xD5,1>, Opcode<0xE5,1>, Opcode<0xF5,1>,
                          [NZ,N,Z,V], [AB], /*IsCompare=*/1>;
defm EORB : ALU8Concrete<"eorb", Opcode<0xC8,1>, Opcode<0xD8,1>, Opcode<0xE8,1>, Opcode<0xF8,1>,
                          [AB,NZ,N,Z,V,AB,AD,AQ], [AB]>;
defm ADCB : ALU8Concrete<"adcb", Opcode<0xC9,1>, Opcode<0xD9,1>, Opcode<0xE9,1>, Opcode<0xF9,1>,
                          [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB,C], /*IsCompare=*/0, /*IsAdd=*/1>;
defm ORB  : ALU8Concrete<"orb",  Opcode<0xCA,1>, Opcode<0xDA,1>, Opcode<0xEA,1>, Opcode<0xFA,1>,
                          [AB,NZ,N,Z,V,AB,AD,AQ], [AB]>;
defm ADDB : ALU8Concrete<"addb", Opcode<0xCB,1>, Opcode<0xDB,1>, Opcode<0xEB,1>, Opcode<0xFB,1>,
                          [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB], /*IsCompare=*/0, /*IsAdd=*/1>;

// 16-bit ALU: D accumulator (Page 1)
defm SUBD : ALU16Concrete<"subd", Opcode<0x83,1>, Opcode<0x93,1>, Opcode<0xA3,1>, Opcode<0xB3,1>,
                           [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]>;
defm ADDD : ALU16Concrete<"addd", Opcode<0xC3,1>, Opcode<0xD3,1>, Opcode<0xE3,1>, Opcode<0xF3,1>,
                           [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD], /*IsCompare=*/0, /*IsAdd=*/1>;

// 16-bit CMP: X, Y, U, S (Page 1 and 2)
defm CMPX : ALU16Concrete<"cmpx", Opcode<0x8C,1>, Opcode<0x9C,1>, Opcode<0xAC,1>, Opcode<0xBC,1>,
                           [NZ,N,Z,V,C], [IX], /*IsCompare=*/1>;

// Unary memory-modify: direct/indexed/extended (Page 1). No immediate.
// mayLoad and mayStore since they read-modify-write memory.
multiclass Unary1Concrete<string Mnem, Opcode DirOpc, Opcode IdxOpc,
                          Opcode ExtOpc,
                          list<Register> Defs, list<Register> Uses> {
  let mayLoad = true, mayStore = true in {
    def d  : MC6809Direct_P1<(outs), Mnem, DirOpc, Defs, Uses>;
    defm i : MC6809Indexed_P1<Mnem, IdxOpc, (outs), Defs, Uses>;
    def e  : MC6809Extended_P1<(outs), Mnem, ExtOpc, Defs, Uses>;
  }
}

// Unary accumulator pair (A and B forms, Page 1).
multiclass UnaryAcc8_AB<string MnemA, Opcode OpcA, list<Register> DefsA, list<Register> UsesA,
                        string MnemB, Opcode OpcB, list<Register> DefsB, list<Register> UsesB> {
  def Aa : MC6809Accumulator_P1<(outs), MnemA, OpcA, DefsA, UsesA>;
  def Ba : MC6809Accumulator_P1<(outs), MnemB, OpcB, DefsB, UsesB>;
}

// Memory-modify unary ops (Page 1): NEG, COM, LSR, ROR, ASR, ASL, ROL, DEC, INC, TST, CLR
defm NEG : Unary1Concrete<"neg", Opcode<0x00,1>, Opcode<0x60,1>, Opcode<0x70,1>, [NZ,N,Z,V,C], []>;
defm COM : Unary1Concrete<"com", Opcode<0x03,1>, Opcode<0x63,1>, Opcode<0x73,1>, [NZ,N,Z,V,C], []>;
defm LSR : Unary1Concrete<"lsr", Opcode<0x04,1>, Opcode<0x64,1>, Opcode<0x74,1>, [NZ,N,Z,C], []>;
defm ROR : Unary1Concrete<"ror", Opcode<0x06,1>, Opcode<0x66,1>, Opcode<0x76,1>, [NZ,N,Z,C], [C]>;
defm ASR : Unary1Concrete<"asr", Opcode<0x07,1>, Opcode<0x67,1>, Opcode<0x77,1>, [NZ,N,Z,C], [C]>;
defm ASL : Unary1Concrete<"asl", Opcode<0x08,1>, Opcode<0x68,1>, Opcode<0x78,1>, [NZ,N,Z,V,C], []>;
defm ROL : Unary1Concrete<"rol", Opcode<0x09,1>, Opcode<0x69,1>, Opcode<0x79,1>, [NZ,N,Z,V,C], [C]>;
defm DEC : Unary1Concrete<"dec", Opcode<0x0A,1>, Opcode<0x6A,1>, Opcode<0x7A,1>, [NZ,N,Z,V], []>;
defm INC : Unary1Concrete<"inc", Opcode<0x0C,1>, Opcode<0x6C,1>, Opcode<0x7C,1>, [NZ,N,Z,V], []>;
defm TST : Unary1Concrete<"tst", Opcode<0x0D,1>, Opcode<0x6D,1>, Opcode<0x7D,1>, [NZ,N,Z,V], []>;
defm CLR : Unary1Concrete<"clr", Opcode<0x0F,1>, Opcode<0x6F,1>, Opcode<0x7F,1>, [NZ,N,Z,V,C], []>;

// Accumulator unary ops (Page 1): A and B forms
// Same defm prefix as memory ops — no conflict since suffixes differ (Aa/Ba vs d/i/e)
defm NEG : UnaryAcc8_AB<"nega", Opcode<0x40,1>, [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA],
                         "negb", Opcode<0x50,1>, [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB]>;
defm COM : UnaryAcc8_AB<"coma", Opcode<0x43,1>, [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA],
                         "comb", Opcode<0x53,1>, [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB]>;
defm LSR : UnaryAcc8_AB<"lsra", Opcode<0x44,1>, [AA,NZ,N,Z,C,AA,AD,AQ], [AA],
                         "lsrb", Opcode<0x54,1>, [AB,NZ,N,Z,C,AB,AD,AQ], [AB]>;
defm ROR : UnaryAcc8_AB<"rora", Opcode<0x46,1>, [AA,NZ,N,Z,C,AA,AD,AQ], [AA,C],
                         "rorb", Opcode<0x56,1>, [AB,NZ,N,Z,C,AB,AD,AQ], [AB,C]>;
defm ASR : UnaryAcc8_AB<"asra", Opcode<0x47,1>, [AA,NZ,N,Z,C,AA,AD,AQ], [AA,C],
                         "asrb", Opcode<0x57,1>, [AB,NZ,N,Z,C,AB,AD,AQ], [AB,C]>;
defm ASL : UnaryAcc8_AB<"asla", Opcode<0x48,1>, [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA],
                         "aslb", Opcode<0x58,1>, [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB]>;
defm ROL : UnaryAcc8_AB<"rola", Opcode<0x49,1>, [AA,NZ,N,Z,V,C,AA,AD,AQ], [AA,C],
                         "rolb", Opcode<0x59,1>, [AB,NZ,N,Z,V,C,AB,AD,AQ], [AB,C]>;
defm DEC : UnaryAcc8_AB<"deca", Opcode<0x4A,1>, [AA,NZ,N,Z,V,AA,AD,AQ], [AA],
                         "decb", Opcode<0x5A,1>, [AB,NZ,N,Z,V,AB,AD,AQ], [AB]>;
defm INC : UnaryAcc8_AB<"inca", Opcode<0x4C,1>, [AA,NZ,N,Z,V,AA,AD,AQ], [AA],
                         "incb", Opcode<0x5C,1>, [AB,NZ,N,Z,V,AB,AD,AQ], [AB]>;
defm CLR : UnaryAcc8_AB<"clra", Opcode<0x4F,1>, [AA,NZ,N,Z,V,C,AA,AD,AQ], [],
                         "clrb", Opcode<0x5F,1>, [AB,NZ,N,Z,V,C,AB,AD,AQ], []>;

// LEA: indexed only (Page 1).
multiclass LEAConcrete<string Mnem, Opcode IdxOpc, list<Register> Defs> {
  defm i : MC6809Indexed_P1<Mnem, IdxOpc, (outs), Defs, []>;
}

defm LEAX : LEAConcrete<"leax", Opcode<0x30,1>, [IX]>;
defm LEAY : LEAConcrete<"leay", Opcode<0x31,1>, [IY]>;
defm LEAS : LEAConcrete<"leas", Opcode<0x32,1>, [SS]>;
defm LEAU : LEAConcrete<"leau", Opcode<0x33,1>, [SU]>;

// Immediate-direct/indexed/extended ops (6309): OIM, AIM, EIM, TIM.
// Read-modify-write with an immediate operand. TIM is test-only (no store).
// Read-modify-write variant (OIM, AIM, EIM).
multiclass ImmRMWConcrete<string Mnem, Opcode DirOpc, Opcode IdxOpc,
                          Opcode ExtOpc> {
  let Predicates = [IsHD6309], mayLoad = true, mayStore = true in {
    def id  : MC6809ImmediateDirect_P1<(outs), Mnem, DirOpc, [NZ,N,Z,V], []>;
    defm ii : MC6809ImmediateIndexed_P1<Mnem, IdxOpc, (outs), [NZ,N,Z,V], []>;
    def ie  : MC6809ImmediateExtended_P1<(outs), Mnem, ExtOpc, [NZ,N,Z,V], []>;
  }
}

// Test-only variant (TIM) — no store, isCompare.
multiclass ImmTestConcrete<string Mnem, Opcode DirOpc, Opcode IdxOpc,
                           Opcode ExtOpc> {
  let Predicates = [IsHD6309], mayLoad = true, isCompare = true in {
    def id  : MC6809ImmediateDirect_P1<(outs), Mnem, DirOpc, [NZ,N,Z,V], []>;
    defm ii : MC6809ImmediateIndexed_P1<Mnem, IdxOpc, (outs), [NZ,N,Z,V], []>;
    def ie  : MC6809ImmediateExtended_P1<(outs), Mnem, ExtOpc, [NZ,N,Z,V], []>;
  }
}

defm OIM : ImmRMWConcrete<"oim", Opcode<0x01,1>, Opcode<0x61,1>, Opcode<0x71,1>>;
defm AIM : ImmRMWConcrete<"aim", Opcode<0x02,1>, Opcode<0x62,1>, Opcode<0x72,1>>;
defm EIM : ImmRMWConcrete<"eim", Opcode<0x05,1>, Opcode<0x65,1>, Opcode<0x75,1>>;
defm TIM : ImmTestConcrete<"tim", Opcode<0x0B,1>, Opcode<0x6B,1>, Opcode<0x7B,1>>;

// LDE/LDF/STE/STF and ALU/Unary Page 2/3 ops are defined after their
// respective indexed multiclasses below.

// ============================== PAGE MC6809Indexed_P2 ==================================
multiclass MC6809Indexed_P2<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {
  def _o0 : MC6809IndexedOffset0_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o5 : MC6809IndexedOffset5_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8 : MC6809IndexedOffset8_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16 : MC6809IndexedOffset16_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oA : MC6809IndexedOffsetA_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])>;
  def _oB : MC6809IndexedOffsetB_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])>;
  def _oE : MC6809IndexedOffsetE_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809IndexedOffsetF_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809IndexedOffsetD_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])>;
  def _oW : MC6809IndexedOffsetW_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc1 : MC6809IndexedInc1_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Inc2 : MC6809IndexedInc2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec1 : MC6809IndexedDec1_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2 : MC6809IndexedDec2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PC : MC6809IndexedOffset8PC_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PC : MC6809IndexedOffset16PC_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o0W : MC6809IndexedOffset0W_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809IndexedOffset16W_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809IndexedIncW2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809IndexedDecW2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o0I : MC6809IndexedOffset0Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8I : MC6809IndexedOffset8Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16I : MC6809IndexedOffset16Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oAI : MC6809IndexedOffsetAInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])>;
  def _oBI : MC6809IndexedOffsetBInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])>;
  def _oEI : MC6809IndexedOffsetEInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809IndexedOffsetFInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809IndexedOffsetDInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])>;
  def _oWI : MC6809IndexedOffsetWInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2I : MC6809IndexedInc2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2I : MC6809IndexedDec2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PCI : MC6809IndexedOffset8PCInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PCI : MC6809IndexedOffset16PCInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _eI : MC6809IndexedExtendedInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _0oWI : MC6809IndexedNoOffsetWInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809IndexedOffset16WInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809IndexedIncW2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809IndexedDecW2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== PAGE MC6809Indexed_P3 ==================================
multiclass MC6809Indexed_P3<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {
  def _o0 : MC6809IndexedOffset0_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o5 : MC6809IndexedOffset5_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8 : MC6809IndexedOffset8_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16 : MC6809IndexedOffset16_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oA : MC6809IndexedOffsetA_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])>;
  def _oB : MC6809IndexedOffsetB_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])>;
  def _oE : MC6809IndexedOffsetE_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809IndexedOffsetF_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809IndexedOffsetD_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])>;
  def _oW : MC6809IndexedOffsetW_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc1 : MC6809IndexedInc1_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Inc2 : MC6809IndexedInc2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec1 : MC6809IndexedDec1_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2 : MC6809IndexedDec2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PC : MC6809IndexedOffset8PC_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PC : MC6809IndexedOffset16PC_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o0W : MC6809IndexedOffset0W_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809IndexedOffset16W_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809IndexedIncW2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809IndexedDecW2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _o0I : MC6809IndexedOffset0Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8I : MC6809IndexedOffset8Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16I : MC6809IndexedOffset16Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oAI : MC6809IndexedOffsetAInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AA])>;
  def _oBI : MC6809IndexedOffsetBInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AB])>;
  def _oEI : MC6809IndexedOffsetEInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AE])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809IndexedOffsetFInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AF])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809IndexedOffsetDInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AD])>;
  def _oWI : MC6809IndexedOffsetWInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2I : MC6809IndexedInc2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2I : MC6809IndexedDec2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PCI : MC6809IndexedOffset8PCInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PCI : MC6809IndexedOffset16PCInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _eI : MC6809IndexedExtendedInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _0oWI : MC6809IndexedNoOffsetWInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809IndexedOffset16WInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809IndexedIncW2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809IndexedDecW2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, [AW]), !listconcat(usesin, [AW])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== END PAGES ==================================
// MRVM END MARKER 1
// clang-format on

// Page 3 (HD6309, prefix 0x11) variant of Load8Concrete.
multiclass Load8Concrete_P3<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                            Opcode IdxOpc, Opcode ExtOpc,
                            list<Register> Defs> {
  let Predicates = [IsHD6309], mayLoad = true in {
    def i8 : MC6809Immediate8Bit_P3<(outs), Mnem, ImmOpc, Defs, []>;
    def d  : MC6809Direct_P3<(outs), Mnem, DirOpc, Defs, []>;
    defm i : MC6809Indexed_P3<Mnem, IdxOpc, (outs), Defs, []>;
    def e  : MC6809Extended_P3<(outs), Mnem, ExtOpc, Defs, []>;
  }
}

// 8-bit loads: LDE/LDF (6309 Page 3)
defm LDE : Load8Concrete_P3<"lde", Opcode<0x86,3>, Opcode<0x96,3>,
                             Opcode<0xA6,3>, Opcode<0xB6,3>,
                             [NZ,N,Z,V,AE,AW,AQ]>;
defm LDF : Load8Concrete_P3<"ldf", Opcode<0xC6,3>, Opcode<0xD6,3>,
                             Opcode<0xE6,3>, Opcode<0xF6,3>,
                             [NZ,N,Z,V,AF,AW,AQ]>;

// 16-bit load (Page 2, prefix 0x10).
multiclass Load16Concrete_P2<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                             Opcode IdxOpc, Opcode ExtOpc,
                             list<Register> Defs> {
  let mayLoad = true in {
    def i16 : MC6809Immediate16Bit_P2<(outs), Mnem, ImmOpc, Defs, []>;
    def d   : MC6809Direct_P2<(outs), Mnem, DirOpc, Defs, []>;
    defm i  : MC6809Indexed_P2<Mnem, IdxOpc, (outs), Defs, []>;
    def e   : MC6809Extended_P2<(outs), Mnem, ExtOpc, Defs, []>;
  }
}

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

// Store (Page 3, HD6309).
multiclass Store8Concrete_P3<string Mnem, Opcode DirOpc, Opcode IdxOpc,
                             Opcode ExtOpc> {
  let Predicates = [IsHD6309], mayStore = true in {
    def d  : MC6809Direct_P3<(outs), Mnem, DirOpc, [NZ,N,Z,V], []>;
    defm i : MC6809Indexed_P3<Mnem, IdxOpc, (outs), [NZ,N,Z,V], []>;
    def e  : MC6809Extended_P3<(outs), Mnem, ExtOpc, [NZ,N,Z,V], []>;
  }
}

// Store (Page 2).
multiclass Store16Concrete_P2<string Mnem, Opcode DirOpc, Opcode IdxOpc,
                              Opcode ExtOpc> {
  let mayStore = true in {
    def d  : MC6809Direct_P2<(outs), Mnem, DirOpc, [NZ,N,Z,V], []>;
    defm i : MC6809Indexed_P2<Mnem, IdxOpc, (outs), [NZ,N,Z,V], []>;
    def e  : MC6809Extended_P2<(outs), Mnem, ExtOpc, [NZ,N,Z,V], []>;
  }
}

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

// ALU 8-bit (Page 3, HD6309).
multiclass ALU8Concrete_P3<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                           Opcode IdxOpc, Opcode ExtOpc,
                           list<Register> Defs, list<Register> Uses,
                           bit IsCompare = 0, bit IsAdd = 0> {
  let Predicates = [IsHD6309], mayLoad = true, isCompare = IsCompare, isAdd = IsAdd in {
    def i8 : MC6809Immediate8Bit_P3<(outs), Mnem, ImmOpc, Defs, Uses>;
    def d  : MC6809Direct_P3<(outs), Mnem, DirOpc, Defs, Uses>;
    defm i : MC6809Indexed_P3<Mnem, IdxOpc, (outs), Defs, Uses>;
    def e  : MC6809Extended_P3<(outs), Mnem, ExtOpc, Defs, Uses>;
  }
}

// ALU 16-bit (Page 2).
multiclass ALU16Concrete_P2<string Mnem, Opcode ImmOpc, Opcode DirOpc,
                            Opcode IdxOpc, Opcode ExtOpc,
                            list<Register> Defs, list<Register> Uses,
                            bit IsCompare = 0, bit IsAdd = 0> {
  let mayLoad = true, isCompare = IsCompare, isAdd = IsAdd in {
    def i16 : MC6809Immediate16Bit_P2<(outs), Mnem, ImmOpc, Defs, Uses>;
    def d   : MC6809Direct_P2<(outs), Mnem, DirOpc, Defs, Uses>;
    defm i  : MC6809Indexed_P2<Mnem, IdxOpc, (outs), Defs, Uses>;
    def e   : MC6809Extended_P2<(outs), Mnem, ExtOpc, Defs, Uses>;
  }
}

// 16-bit CMP: Y, U, S (Page 2)
defm CMPY : ALU16Concrete_P2<"cmpy", Opcode<0x8C,2>, Opcode<0x9C,2>, Opcode<0xAC,2>, Opcode<0xBC,2>,
                              [NZ,N,Z,V,C], [IY], /*IsCompare=*/1>;
defm CMPU : ALU16Concrete_P2<"cmpu", Opcode<0x83,3>, Opcode<0x93,3>, Opcode<0xA3,3>, Opcode<0xB3,3>,
                              [NZ,N,Z,V,C], [SU], /*IsCompare=*/1>;
defm CMPS : ALU16Concrete_P2<"cmps", Opcode<0x8C,3>, Opcode<0x9C,3>, Opcode<0xAC,3>, Opcode<0xBC,3>,
                              [NZ,N,Z,V,C], [SS], /*IsCompare=*/1>;

// 16-bit ALU: D register (Page 2, HD6309)
let Predicates = [IsHD6309] in {
  defm SBCD : ALU16Concrete_P2<"sbcd", Opcode<0x82,2>, Opcode<0x92,2>, Opcode<0xA2,2>, Opcode<0xB2,2>,
                                [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD,C]>;
  defm CMPD : ALU16Concrete_P2<"cmpd", Opcode<0x83,2>, Opcode<0x93,2>, Opcode<0xA3,2>, Opcode<0xB3,2>,
                                [NZ,N,Z,V,C], [AD], /*IsCompare=*/1>;
  defm ANDD : ALU16Concrete_P2<"andd", Opcode<0x84,2>, Opcode<0x94,2>, Opcode<0xA4,2>, Opcode<0xB4,2>,
                                [AD,NZ,N,Z,V,AA,AB,AD,AQ], [AD]>;
  defm BITD : ALU16Concrete_P2<"bitd", Opcode<0x85,2>, Opcode<0x95,2>, Opcode<0xA5,2>, Opcode<0xB5,2>,
                                [NZ,N,Z,V], [AD], /*IsCompare=*/1>;
  defm EORD : ALU16Concrete_P2<"eord", Opcode<0x88,2>, Opcode<0x98,2>, Opcode<0xA8,2>, Opcode<0xB8,2>,
                                [AD,NZ,N,Z,V,AA,AB,AD,AQ], [AD]>;
  defm ADCD : ALU16Concrete_P2<"adcd", Opcode<0x89,2>, Opcode<0x99,2>, Opcode<0xA9,2>, Opcode<0xB9,2>,
                                [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD,C], /*IsCompare=*/0, /*IsAdd=*/1>;
  defm ORD  : ALU16Concrete_P2<"ord",  Opcode<0x8A,2>, Opcode<0x9A,2>, Opcode<0xAA,2>, Opcode<0xBA,2>,
                                [AD,NZ,N,Z,V,AA,AB,AD,AQ], [AD]>;
}

// 16-bit ALU: W register (Page 2, HD6309)
let Predicates = [IsHD6309] in {
  defm SUBW : ALU16Concrete_P2<"subw", Opcode<0x80,2>, Opcode<0x90,2>, Opcode<0xA0,2>, Opcode<0xB0,2>,
                                [AW,NZ,N,Z,V,C,AE,AF,AW,AQ], [AW]>;
  defm CMPW : ALU16Concrete_P2<"cmpw", Opcode<0x81,2>, Opcode<0x91,2>, Opcode<0xA1,2>, Opcode<0xB1,2>,
                                [NZ,N,Z,V,C], [AW], /*IsCompare=*/1>;
  defm ADDW : ALU16Concrete_P2<"addw", Opcode<0x8B,2>, Opcode<0x9B,2>, Opcode<0xAB,2>, Opcode<0xBB,2>,
                                [AW,NZ,N,Z,V,C,AE,AF,AW,AQ], [AW], /*IsCompare=*/0, /*IsAdd=*/1>;
}

// 8-bit ALU: E accumulator (Page 3, HD6309)
defm SUBE : ALU8Concrete_P3<"sube", Opcode<0x80,3>, Opcode<0x90,3>, Opcode<0xA0,3>, Opcode<0xB0,3>,
                             [AE,NZ,N,Z,V,C,AE,AW,AQ], [AE]>;
defm CMPE : ALU8Concrete_P3<"cmpe", Opcode<0x81,3>, Opcode<0x91,3>, Opcode<0xA1,3>, Opcode<0xB1,3>,
                             [NZ,N,Z,V,C], [AE], /*IsCompare=*/1>;
defm ADDE : ALU8Concrete_P3<"adde", Opcode<0x8B,3>, Opcode<0x9B,3>, Opcode<0xAB,3>, Opcode<0xBB,3>,
                             [AE,NZ,N,Z,V,C,AE,AW,AQ], [AE], /*IsCompare=*/0, /*IsAdd=*/1>;

// 8-bit ALU: F accumulator (Page 3, HD6309)
defm SUBF : ALU8Concrete_P3<"subf", Opcode<0xC0,3>, Opcode<0xD0,3>, Opcode<0xE0,3>, Opcode<0xF0,3>,
                             [AF,NZ,N,Z,V,C,AF,AW,AQ], [AF]>;
defm CMPF : ALU8Concrete_P3<"cmpf", Opcode<0xC1,3>, Opcode<0xD1,3>, Opcode<0xE1,3>, Opcode<0xF1,3>,
                             [NZ,N,Z,V,C], [AF], /*IsCompare=*/1>;
defm ADDF : ALU8Concrete_P3<"addf", Opcode<0xCB,3>, Opcode<0xDB,3>, Opcode<0xEB,3>, Opcode<0xFB,3>,
                             [AF,NZ,N,Z,V,C,AF,AW,AQ], [AF], /*IsCompare=*/0, /*IsAdd=*/1>;

// Unary accumulator ops: D and W registers (Page 2, HD6309)
let Predicates = [IsHD6309] in {
  def NEGDa : MC6809Accumulator_P2<(outs), "negd", Opcode<0x40,2>, [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]>;
  def COMDa : MC6809Accumulator_P2<(outs), "comd", Opcode<0x43,2>, [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]>;
  def LSRDa : MC6809Accumulator_P2<(outs), "lsrd", Opcode<0x44,2>, [AD,NZ,N,Z,C,AA,AB,AD,AQ], [AD]>;
  def RORDa : MC6809Accumulator_P2<(outs), "rord", Opcode<0x46,2>, [AD,NZ,N,Z,C,AA,AB,AD,AQ], [AD,C]>;
  def ASRDa : MC6809Accumulator_P2<(outs), "asrd", Opcode<0x47,2>, [AD,NZ,N,Z,C,AA,AB,AD,AQ], [AD,C]>;
  def ASLDa : MC6809Accumulator_P2<(outs), "asld", Opcode<0x48,2>, [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]>;
  def ROLDa : MC6809Accumulator_P2<(outs), "rold", Opcode<0x49,2>, [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], [AD,C]>;
  def DECDa : MC6809Accumulator_P2<(outs), "decd", Opcode<0x4A,2>, [AD,NZ,N,Z,V,AA,AB,AD,AQ], [AD]>;
  def INCDa : MC6809Accumulator_P2<(outs), "incd", Opcode<0x4C,2>, [AD,NZ,N,Z,V,AA,AB,AD,AQ], [AD]>;
  def TSTDa : MC6809Accumulator_P2<(outs), "tstd", Opcode<0x4D,2>, [NZ,N,Z,V], [AD]>;
  def CLRDa : MC6809Accumulator_P2<(outs), "clrd", Opcode<0x4F,2>, [AD,NZ,N,Z,V,C,AA,AB,AD,AQ], []>;
  def COMWa : MC6809Accumulator_P2<(outs), "comw", Opcode<0x53,2>, [AW,NZ,N,Z,V,C,AE,AF,AW,AQ], [AW]>;
  def LSRWa : MC6809Accumulator_P2<(outs), "lsrw", Opcode<0x54,2>, [AW,NZ,N,Z,C,AE,AF,AW,AQ], [AW]>;
  def RORWa : MC6809Accumulator_P2<(outs), "rorw", Opcode<0x56,2>, [AW,NZ,N,Z,C,AE,AF,AW,AQ], [AW,C]>;
  def ROLWa : MC6809Accumulator_P2<(outs), "rolw", Opcode<0x59,2>, [AW,NZ,N,Z,V,C,AE,AF,AW,AQ], [AW,C]>;
  def DECWa : MC6809Accumulator_P2<(outs), "decw", Opcode<0x5A,2>, [AW,NZ,N,Z,V,AE,AF,AW,AQ], [AW]>;
  def INCWa : MC6809Accumulator_P2<(outs), "incw", Opcode<0x5C,2>, [AW,NZ,N,Z,V,AE,AF,AW,AQ], [AW]>;
  def TSTWa : MC6809Accumulator_P2<(outs), "tstw", Opcode<0x5D,2>, [NZ,N,Z,V], [AW]>;
  def CLRWa : MC6809Accumulator_P2<(outs), "clrw", Opcode<0x5F,2>, [AW,NZ,N,Z,V,C,AE,AF,AW,AQ], []>;
}

// Unary accumulator ops: E and F registers (Page 3, HD6309)
let Predicates = [IsHD6309] in {
  def COMEa : MC6809Accumulator_P3<(outs), "come", Opcode<0x43,3>, [AE,NZ,N,Z,V,C,AE,AW,AQ], [AE]>;
  def DECEa : MC6809Accumulator_P3<(outs), "dece", Opcode<0x4A,3>, [AE,NZ,N,Z,V,AE,AW,AQ], [AE]>;
  def INCEa : MC6809Accumulator_P3<(outs), "ince", Opcode<0x4C,3>, [AE,NZ,N,Z,V,AE,AW,AQ], [AE]>;
  def TSTEa : MC6809Accumulator_P3<(outs), "tste", Opcode<0x4D,3>, [NZ,N,Z,V], [AE]>;
  def CLREa : MC6809Accumulator_P3<(outs), "clre", Opcode<0x4F,3>, [AE,NZ,N,Z,V,C,AE,AW,AQ], []>;
  def COMFa : MC6809Accumulator_P3<(outs), "comf", Opcode<0x53,3>, [AF,NZ,N,Z,V,C,AF,AW,AQ], [AF]>;
  def DECFa : MC6809Accumulator_P3<(outs), "decf", Opcode<0x5A,3>, [AF,NZ,N,Z,V,AF,AW,AQ], [AF]>;
  def INCFa : MC6809Accumulator_P3<(outs), "incf", Opcode<0x5C,3>, [AF,NZ,N,Z,V,AF,AW,AQ], [AF]>;
  def TSTFa : MC6809Accumulator_P3<(outs), "tstf", Opcode<0x5D,3>, [NZ,N,Z,V], [AF]>;
  def CLRFa : MC6809Accumulator_P3<(outs), "clrf", Opcode<0x5F,3>, [AF,NZ,N,Z,V,C,AF,AW,AQ], []>;
}

// clang-format off
// MRVM START MARKER 2
// NEGd now generated by Unary1Concrete multiclass above

// OIMid now generated by ImmRMWConcrete multiclass above

// AIMid now generated by ImmRMWConcrete multiclass above

// COMd now generated by Unary1Concrete multiclass above

// LSRd now generated by Unary1Concrete multiclass above

// EIMid now generated by ImmRMWConcrete multiclass above

// RORd now generated by Unary1Concrete multiclass above

// ASRd now generated by Unary1Concrete multiclass above

// ASLd now generated by Unary1Concrete multiclass above

// ROLd now generated by Unary1Concrete multiclass above

// DECd now generated by Unary1Concrete multiclass above

// TIMid now generated by ImmTestConcrete multiclass above

// INCd now generated by Unary1Concrete multiclass above

// TSTd now generated by Unary1Concrete multiclass above

def JMPd : MC6809Direct_P1<(outs), "jmp", Opcode<0x0E,1>, [], []> {
  let isBranch = true;
  let isBarrier = true;
  let isTerminator = true;
}

// CLRd now generated by Unary1Concrete multiclass above

def NOPx : MC6809Inherent_P1<(outs), "nop", Opcode<0x12,1>, [], []>;

def SYNCx : MC6809Inherent_P1<(outs), "sync", Opcode<0x13,1>, [], []>;

def SEXWx : MC6809Inherent_P1<(outs), "sexw", Opcode<0x14,1>, [NZ,N,Z,AA,AB,AD,AE,AF,AW,AQ], [AD]> {
  let Predicates = [IsHD6309];
}

def HCFx : MC6809Inherent_P1<(outs), "hcf", Opcode<0x15,1>, [], []>;

def LBRAlb : MC6809LongBranch_P1<(outs), "lbra", Opcode<0x16,1>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = true;
  let isTerminator = true;
  let isBarrier = true;
}

def LBSRlb : MC6809LongBranch_P1<(outs), "lbsr", Opcode<0x17,1>, [], [SS]> {
  let mayLoad = false;
  let mayStore = false;
  let isCall = true;
}

def DAAx : MC6809Inherent_P1<(outs), "daa", Opcode<0x19,1>, [AA,NZ,N,Z,C,AA,AD,AQ], [AA]>;

def ORCCi8 : MC6809Immediate8Bit_P1<(outs), "orcc", Opcode<0x1A,1>, [NZ,N,Z,V,C], [NZ,N,Z,V,C]>;

def ANDCCi8 : MC6809Immediate8Bit_P1<(outs), "andcc", Opcode<0x1C,1>, [NZ,N,Z,V,C], [NZ,N,Z,V,C]>;

def SEXx : MC6809Inherent_P1<(outs), "sex", Opcode<0x1D,1>, [NZ,N,Z,AA,AB,AD,AQ], [AB]>;

def EXGp : MC6809RegisterPairSwap_P1<(outs anyregister:$dst2, anyregister:$dst1), "exg", Opcode<0x1E,1>, [], []> {
  let Constraints = "$dst2 = $reg1, $dst1 = $reg2";
}

def TFRp : MC6809RegisterPairCopy_P1<(outs anyregister:$reg2), "tfr", Opcode<0x1F,1>, [], []>;

def BRAb : MC6809ShortBranch_P1<(outs), "bra", Opcode<0x20,1>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = true;
  let isTerminator = true;
  let isBarrier = true;
}

def BRNb : MC6809ShortBranch_P1<(outs), "brn", Opcode<0x21,1>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = false;
  let isTerminator = false;
}

def Bbc : MC6809ShortConditionalBranch_P1<(outs), "b$cond", Opcode<0x02,1>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = true;
  let isTerminator = true;
}

// LEAXi/LEAYi/LEASi/LEAUi now generated by LEAConcrete multiclass above

def PSHSs : MC6809Stack_P1<(outs), "pshs", Opcode<0x34,1>, [SS], [SS]>;

def PULSs : MC6809Stack_P1<(outs), "puls", Opcode<0x35,1>, [SS], [SS]>;

def PSHUs : MC6809Stack_P1<(outs), "pshu", Opcode<0x36,1>, [SU], [SU]>;

def PULUs : MC6809Stack_P1<(outs), "pulu", Opcode<0x37,1>, [SU], [SU]>;

def RTSr : MC6809Return_P1<(outs), "rts", Opcode<0x39,1>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isReturn = true;
  let isTerminator = true;
  let isBarrier = true;
}

def ABXx : MC6809Inherent_P1<(outs), "abx", Opcode<0x3A,1>, [], []>;

def RTIr : MC6809Return_P1<(outs), "rti", Opcode<0x3B,1>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isReturn = true;
  let isTerminator = true;
  let isBarrier = true;
}

def CWAIi8 : MC6809Immediate8Bit_P1<(outs), "cwai", Opcode<0x3C,1>, [], []>;

def MULx : MC6809Inherent_P1<(outs), "mul", Opcode<0x3D,1>, [Z,C,AA,AB,AD,AQ], [AA,AB]>;

def RESETx : MC6809Inherent_P1<(outs), "reset", Opcode<0x3E,1>, [], []>;

def SWIx : MC6809Inherent_P1<(outs), "swi", Opcode<0x3F,1>, [], []>;

// NEGAa now generated by Unary1Concrete multiclass above

// COMAa now generated by Unary1Concrete multiclass above

// LSRAa now generated by Unary1Concrete multiclass above

// RORAa now generated by Unary1Concrete multiclass above

// ASRAa now generated by Unary1Concrete multiclass above

// ASLAa now generated by Unary1Concrete multiclass above

// ROLAa now generated by Unary1Concrete multiclass above

// DECAa now generated by Unary1Concrete multiclass above

// INCAa now generated by Unary1Concrete multiclass above

// TSTAa now generated by Unary1Concrete multiclass above

// CLRAa now generated by Unary1Concrete multiclass above

// NEGBa now generated by Unary1Concrete multiclass above

// COMBa now generated by Unary1Concrete multiclass above

// LSRBa now generated by Unary1Concrete multiclass above

// RORBa now generated by Unary1Concrete multiclass above

// ASRBa now generated by Unary1Concrete multiclass above

// ASLBa now generated by Unary1Concrete multiclass above

// ROLBa now generated by Unary1Concrete multiclass above

// DECBa now generated by Unary1Concrete multiclass above

// INCBa now generated by Unary1Concrete multiclass above

// TSTBa now generated by Unary1Concrete multiclass above

// CLRBa now generated by Unary1Concrete multiclass above

// NEGi now generated by Unary1Concrete multiclass above

// OIMii now generated by ImmRMWConcrete multiclass above

// AIMii now generated by ImmRMWConcrete multiclass above

// COMi now generated by Unary1Concrete multiclass above

// LSRi now generated by Unary1Concrete multiclass above

// EIMii now generated by ImmRMWConcrete multiclass above

// RORi now generated by Unary1Concrete multiclass above

// ASRi now generated by Unary1Concrete multiclass above

// ASLi now generated by Unary1Concrete multiclass above

// ROLi now generated by Unary1Concrete multiclass above

// DECi now generated by Unary1Concrete multiclass above

// TIMii now generated by ImmTestConcrete multiclass above

// INCi now generated by Unary1Concrete multiclass above

// TSTi now generated by Unary1Concrete multiclass above

let isBarrier = true, isBranch = true, isTerminator = true in {
  defm JMPi : MC6809Indexed_P1<"jmp", Opcode<0x6E, 1>, (outs), [], []>;
}

// CLRi now generated by Unary1Concrete multiclass above

// NEGe now generated by Unary1Concrete multiclass above

// OIMie now generated by ImmRMWConcrete multiclass above

// AIMie now generated by ImmRMWConcrete multiclass above

// COMe now generated by Unary1Concrete multiclass above

// LSRe now generated by Unary1Concrete multiclass above

// EIMie now generated by ImmRMWConcrete multiclass above

// RORe now generated by Unary1Concrete multiclass above

// ASRe now generated by Unary1Concrete multiclass above

// ASLe now generated by Unary1Concrete multiclass above

// ROLe now generated by Unary1Concrete multiclass above

// DECe now generated by Unary1Concrete multiclass above

// TIMie now generated by ImmTestConcrete multiclass above

// INCe now generated by Unary1Concrete multiclass above

// TSTe now generated by Unary1Concrete multiclass above

def JMPe : MC6809Extended_P1<(outs), "jmp", Opcode<0x7E,1>, [], []> {
  let isBranch = true;
  let isBarrier = true;
  let isTerminator = true;
}

// CLRe now generated by Unary1Concrete multiclass above

// SUBAi8 now generated by ALU8Concrete multiclass above

// CMPAi8 now generated by ALU8Concrete multiclass above

// SBCAi8 now generated by ALU8Concrete multiclass above

// SUBDi16 now generated by ALU16Concrete multiclass above

// ANDAi8 now generated by ALU8Concrete multiclass above

// BITAi8 now generated by ALU8Concrete multiclass above

// LDAi8 now generated by Load8Concrete multiclass above

// EORAi8 now generated by ALU8Concrete multiclass above

// ADCAi8 now generated by ALU8Concrete multiclass above

// ORAi8 now generated by ALU8Concrete multiclass above

// ADDAi8 now generated by ALU8Concrete multiclass above

// CMPXi16 now generated by ALU16Concrete multiclass above

def BSRb : MC6809ShortBranch_P1<(outs), "bsr", Opcode<0x8D,1>, [], [SS]> {
  let mayLoad = false;
  let mayStore = false;
  let isCall = true;
}

// LDXi16 now generated by Load16Concrete multiclass above

// SUBAd now generated by ALU8Concrete multiclass above

// CMPAd now generated by ALU8Concrete multiclass above

// SBCAd now generated by ALU8Concrete multiclass above

// SUBDd now generated by ALU16Concrete multiclass above

// ANDAd now generated by ALU8Concrete multiclass above

// BITAd now generated by ALU8Concrete multiclass above

// LDAd now generated by Load8Concrete multiclass above

// STAd now generated by Store8Concrete multiclass above

// EORAd now generated by ALU8Concrete multiclass above

// ADCAd now generated by ALU8Concrete multiclass above

// ORAd now generated by ALU8Concrete multiclass above

// ADDAd now generated by ALU8Concrete multiclass above

// CMPXd now generated by ALU16Concrete multiclass above

def JSRd : MC6809Direct_P1<(outs), "jsr", Opcode<0x9D,1>, [], []> {
  let isCall = true;
}

// LDXd now generated by Load16Concrete multiclass above

// STXd now generated by Store8Concrete multiclass above

// SUBAi now generated by ALU8Concrete multiclass above

// CMPAi now generated by ALU8Concrete multiclass above

// SBCAi now generated by ALU8Concrete multiclass above

// SUBDi now generated by ALU16Concrete multiclass above

// ANDAi now generated by ALU8Concrete multiclass above

// BITAi now generated by ALU8Concrete multiclass above

// LDAi now generated by Load8Concrete multiclass above

// STAi now generated by Store8Concrete multiclass above

// EORAi now generated by ALU8Concrete multiclass above

// ADCAi now generated by ALU8Concrete multiclass above

// ORAi now generated by ALU8Concrete multiclass above

// ADDAi now generated by ALU8Concrete multiclass above

// CMPXi now generated by ALU16Concrete multiclass above

let isCall = true in {
  defm JSRi : MC6809Indexed_P1<"jsr", Opcode<0xAD, 1>, (outs), [], []>;
}

// defm LDXi now generated by Load16Concrete multiclass above

// STXi now generated by Store8Concrete multiclass above

// SUBAe now generated by ALU8Concrete multiclass above

// CMPAe now generated by ALU8Concrete multiclass above

// SBCAe now generated by ALU8Concrete multiclass above

// SUBDe now generated by ALU16Concrete multiclass above

// ANDAe now generated by ALU8Concrete multiclass above

// BITAe now generated by ALU8Concrete multiclass above

// LDAe now generated by Load8Concrete multiclass above

// STAe now generated by Store8Concrete multiclass above

// EORAe now generated by ALU8Concrete multiclass above

// ADCAe now generated by ALU8Concrete multiclass above

// ORAe now generated by ALU8Concrete multiclass above

// ADDAe now generated by ALU8Concrete multiclass above

// CMPXe now generated by ALU16Concrete multiclass above

def JSRe : MC6809Extended_P1<(outs), "jsr", Opcode<0xBD,1>, [], []> {
  let isCall = true;
}

// LDXe now generated by Load16Concrete multiclass above

// STXe now generated by Store8Concrete multiclass above

// SUBBi8 now generated by ALU8Concrete multiclass above

// CMPBi8 now generated by ALU8Concrete multiclass above

// SBCBi8 now generated by ALU8Concrete multiclass above

// ADDDi16 now generated by ALU16Concrete multiclass above

// ANDBi8 now generated by ALU8Concrete multiclass above

// BITBi8 now generated by ALU8Concrete multiclass above

// LDBi8 now generated by Load8Concrete multiclass above

// EORBi8 now generated by ALU8Concrete multiclass above

// ADCBi8 now generated by ALU8Concrete multiclass above

// ORBi8 now generated by ALU8Concrete multiclass above

// ADDBi8 now generated by ALU8Concrete multiclass above

// LDDi16 now generated by Load16Concrete multiclass above

// LDQi32 now generated by Load16Concrete multiclass above

// LDUi16 now generated by Load16Concrete multiclass above

// SUBBd now generated by ALU8Concrete multiclass above

// CMPBd now generated by ALU8Concrete multiclass above

// SBCBd now generated by ALU8Concrete multiclass above

// ADDDd now generated by ALU16Concrete multiclass above

// ANDBd now generated by ALU8Concrete multiclass above

// BITBd now generated by ALU8Concrete multiclass above

// LDBd now generated by Load8Concrete multiclass above

// STBd now generated by Store8Concrete multiclass above

// EORBd now generated by ALU8Concrete multiclass above

// ADCBd now generated by ALU8Concrete multiclass above

// ORBd now generated by ALU8Concrete multiclass above

// ADDBd now generated by ALU8Concrete multiclass above

// LDDd now generated by Load16Concrete multiclass above

// STDd now generated by Store8Concrete multiclass above

// LDUd now generated by Load16Concrete multiclass above

// STUd now generated by Store8Concrete multiclass above

// SUBBi now generated by ALU8Concrete multiclass above

// CMPBi now generated by ALU8Concrete multiclass above

// SBCBi now generated by ALU8Concrete multiclass above

// ADDDi now generated by ALU16Concrete multiclass above

// ANDBi now generated by ALU8Concrete multiclass above

// BITBi now generated by ALU8Concrete multiclass above

// LDBi now generated by Load8Concrete multiclass above

// STBi now generated by Store8Concrete multiclass above

// EORBi now generated by ALU8Concrete multiclass above

// ADCBi now generated by ALU8Concrete multiclass above

// ORBi now generated by ALU8Concrete multiclass above

// ADDBi now generated by ALU8Concrete multiclass above

// defm LDDi now generated by Load16Concrete multiclass above

// STDi now generated by Store8Concrete multiclass above

// defm LDUi now generated by Load16Concrete multiclass above

// STUi now generated by Store8Concrete multiclass above

// SUBBe now generated by ALU8Concrete multiclass above

// CMPBe now generated by ALU8Concrete multiclass above

// SBCBe now generated by ALU8Concrete multiclass above

// ADDDe now generated by ALU16Concrete multiclass above

// ANDBe now generated by ALU8Concrete multiclass above

// BITBe now generated by ALU8Concrete multiclass above

// LDBe now generated by Load8Concrete multiclass above

// STBe now generated by Store8Concrete multiclass above

// EORBe now generated by ALU8Concrete multiclass above

// ADCBe now generated by ALU8Concrete multiclass above

// ORBe now generated by ALU8Concrete multiclass above

// ADDBe now generated by ALU8Concrete multiclass above

// LDDe now generated by Load16Concrete multiclass above

// STDe now generated by Store8Concrete multiclass above

// LDUe now generated by Load16Concrete multiclass above

// STUe now generated by Store8Concrete multiclass above

def LBRNlb : MC6809LongBranch_P2<(outs), "lbrn", Opcode<0x21,2>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = false;
  let isTerminator = false;
}

def LBlbc : MC6809LongConditionalBranch_P2<(outs), "lb$cond", Opcode<0x02,2>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = true;
  let isTerminator = true;
}

def ADDRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "addr", Opcode<0x30,2>, [NZ,N,Z,V,C], []> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let isAdd = true;
}

def ADCRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "adcr", Opcode<0x31,2>, [NZ,N,Z,V,C], [C]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let isAdd = true;
}

def SUBRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "subr", Opcode<0x32,2>, [NZ,N,Z,V,C], []> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
}

def SBCRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "sbcr", Opcode<0x33,2>, [NZ,N,Z,V,C], [C]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
}

def ANDRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "andr", Opcode<0x34,2>, [NZ,N,Z,V], []> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
}

def ORRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "orr", Opcode<0x35,2>, [NZ,N,Z,V], []> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
}

def EORRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "eorr", Opcode<0x36,2>, [NZ,N,Z,V], []> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
}

def CMPRp : MC6809RegisterPairCompare_P2<(outs), "cmpr", Opcode<0x37,2>, [NZ,N,Z,V], []> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def PSHSWx : MC6809Inherent_P2<(outs), "pshsw", Opcode<0x38,2>, [SS], [SS]> {
  let Predicates = [IsHD6309];
}

def PULSWx : MC6809Inherent_P2<(outs), "pulsw", Opcode<0x39,2>, [SS], [SS]> {
  let Predicates = [IsHD6309];
}

def PSHUWx : MC6809Inherent_P2<(outs), "pshuw", Opcode<0x3A,2>, [SU], [SU]> {
  let Predicates = [IsHD6309];
}

def PULUWx : MC6809Inherent_P2<(outs), "puluw", Opcode<0x3B,2>, [SU], [SU]> {
  let Predicates = [IsHD6309];
}

def SWI2x : MC6809Inherent_P2<(outs), "swi2", Opcode<0x3F,2>, [], []>;

// NEGDa now generated by Unary1Concrete multiclass above

// COMDa now generated by Unary1Concrete multiclass above

// LSRDa now generated by Unary1Concrete multiclass above

// RORDa now generated by Unary1Concrete multiclass above

// ASRDa now generated by Unary1Concrete multiclass above

// ASLDa now generated by Unary1Concrete multiclass above

// ROLDa now generated by Unary1Concrete multiclass above

// DECDa now generated by Unary1Concrete multiclass above

// INCDa now generated by Unary1Concrete multiclass above

// TSTDa now generated by Unary1Concrete multiclass above

// CLRDa now generated by Unary1Concrete multiclass above

// COMWa now generated by Unary1Concrete multiclass above

// LSRWa now generated by Unary1Concrete multiclass above

// RORWa now generated by Unary1Concrete multiclass above

// ROLWa now generated by Unary1Concrete multiclass above

// DECWa now generated by Unary1Concrete multiclass above

// INCWa now generated by Unary1Concrete multiclass above

// TSTWa now generated by Unary1Concrete multiclass above

// CLRWa now generated by Unary1Concrete multiclass above

// SUBWi16 now generated by ALU16Concrete multiclass above

// CMPWi16 now generated by ALU16Concrete multiclass above

// SBCDi16 now generated by ALU16Concrete multiclass above

// CMPDi16 now generated by ALU16Concrete multiclass above

// ANDDi16 now generated by ALU16Concrete multiclass above

// BITDi16 now generated by ALU16Concrete multiclass above

// LDWi16 now generated by Load16Concrete multiclass above

// EORDi16 now generated by ALU16Concrete multiclass above

// ADCDi16 now generated by ALU16Concrete multiclass above

// ORDi16 now generated by ALU16Concrete_P2 multiclass above

// ADDWi16 now generated by ALU16Concrete multiclass above

// CMPYi16 now generated by ALU16Concrete multiclass above

// LDYi16 now generated by Load16Concrete multiclass above

// SUBWd now generated by ALU16Concrete multiclass above

// CMPWd now generated by ALU16Concrete multiclass above

// SBCDd now generated by ALU16Concrete multiclass above

// CMPDd now generated by ALU16Concrete multiclass above

// ANDDd now generated by ALU16Concrete multiclass above

// BITDd now generated by ALU16Concrete multiclass above

// LDWd now generated by Load16Concrete multiclass above

// STWd now generated by Store8Concrete multiclass above

// EORDd now generated by ALU16Concrete multiclass above

// ADCDd now generated by ALU16Concrete multiclass above

// ORDd now generated by ALU16Concrete_P2 multiclass above

// ADDWd now generated by ALU16Concrete multiclass above

// CMPYd now generated by ALU16Concrete multiclass above

// LDYd now generated by Load16Concrete multiclass above

// STYd now generated by Store8Concrete multiclass above

// SUBWi now generated by ALU16Concrete multiclass above

// CMPWi now generated by ALU16Concrete multiclass above

// SBCDi now generated by ALU16Concrete multiclass above

// CMPDi now generated by ALU16Concrete multiclass above

// ANDDi now generated by ALU16Concrete multiclass above

// BITDi now generated by ALU16Concrete multiclass above

// defm LDWi now generated by Load16Concrete multiclass above

// STWi now generated by Store8Concrete multiclass above

// EORDi now generated by ALU16Concrete multiclass above

// ADCDi now generated by ALU16Concrete multiclass above

// ORDi now generated by ALU16Concrete_P2 multiclass above

// ADDWi now generated by ALU16Concrete multiclass above

// CMPYi now generated by ALU16Concrete multiclass above

// defm LDYi now generated by Load16Concrete multiclass above

// STYi now generated by Store8Concrete multiclass above

// SUBWe now generated by ALU16Concrete multiclass above

// CMPWe now generated by ALU16Concrete multiclass above

// SBCDe now generated by ALU16Concrete multiclass above

// CMPDe now generated by ALU16Concrete multiclass above

// ANDDe now generated by ALU16Concrete multiclass above

// BITDe now generated by ALU16Concrete multiclass above

// LDWe now generated by Load16Concrete multiclass above

// STWe now generated by Store8Concrete multiclass above

// EORDe now generated by ALU16Concrete multiclass above

// ADCDe now generated by ALU16Concrete multiclass above

// ORDe now generated by ALU16Concrete_P2 multiclass above

// ADDWe now generated by ALU16Concrete multiclass above

// CMPYe now generated by ALU16Concrete multiclass above

// LDYe now generated by Load16Concrete multiclass above

// STYe now generated by Store8Concrete multiclass above

// LDSi16 now generated by Load16Concrete multiclass above

// LDQd now generated by Load16Concrete multiclass above

// STQd now generated by Store8Concrete multiclass above

// LDSd now generated by Load16Concrete multiclass above

// STSd now generated by Store8Concrete multiclass above

// defm LDQi now generated by Load16Concrete multiclass above

// STQi now generated by Store8Concrete multiclass above

// defm LDSi now generated by Load16Concrete multiclass above

// STSi now generated by Store8Concrete multiclass above

// LDQe now generated by Load16Concrete multiclass above

// STQe now generated by Store8Concrete multiclass above

// LDSe now generated by Load16Concrete multiclass above

// STSe now generated by Store8Concrete multiclass above

def BANDbd : MC6809BitDirect_P3<(outs BIT8:$dst), "band", Opcode<0x30,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def BIANDbd : MC6809BitDirect_P3<(outs BIT8:$dst), "biand", Opcode<0x31,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def BORbd : MC6809BitDirect_P3<(outs BIT8:$dst), "bor", Opcode<0x32,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def BIORbd : MC6809BitDirect_P3<(outs BIT8:$dst), "bior", Opcode<0x33,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def BEORbd : MC6809BitDirect_P3<(outs BIT8:$dst), "beor", Opcode<0x34,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def BIEORbd : MC6809BitDirect_P3<(outs BIT8:$dst), "bieor", Opcode<0x35,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def LDBTbd : MC6809BitDirect_P3<(outs BIT8:$dst), "ldbt", Opcode<0x36,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def STBTbd : MC6809BitDirect_P3<(outs BIT8:$dst), "stbt", Opcode<0x37,3>, [], []> {
  let Constraints = "$dst = $reg";
  let Predicates = [IsHD6309];
}

def TFM0pp : MC6809BlockRegisterPair0_P3<(outs), "tfm", Opcode<0x38,3>, [], []> {
  let Predicates = [IsHD6309];
}

def TFM1pp : MC6809BlockRegisterPair1_P3<(outs), "tfm", Opcode<0x39,3>, [], []> {
  let Predicates = [IsHD6309];
}

def TFM2pp : MC6809BlockRegisterPair2_P3<(outs), "tfm", Opcode<0x3A,3>, [], []> {
  let Predicates = [IsHD6309];
}

def TFM3pp : MC6809BlockRegisterPair3_P3<(outs), "tfm", Opcode<0x3B,3>, [], []> {
  let Predicates = [IsHD6309];
}

def BITMDi8 : MC6809Immediate8Bit_P3<(outs), "bitmd", Opcode<0x3C,3>, [NZ,N,Z,V], [AD]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
  let mayLoad = true;
}

def LDMDi8 : MC6809Immediate8Bit_P3<(outs), "ldmd", Opcode<0x3D,3>, [], []> {
  let Predicates = [IsHD6309];
}

def SWI3x : MC6809Inherent_P3<(outs), "swi3", Opcode<0x3F,3>, [], []>;

// COMEa now generated by Unary1Concrete multiclass above

// DECEa now generated by Unary1Concrete multiclass above

// INCEa now generated by Unary1Concrete multiclass above

// TSTEa now generated by Unary1Concrete multiclass above

// CLREa now generated by Unary1Concrete multiclass above

// COMFa now generated by Unary1Concrete multiclass above

// DECFa now generated by Unary1Concrete multiclass above

// INCFa now generated by Unary1Concrete multiclass above

// TSTFa now generated by Unary1Concrete multiclass above

// CLRFa now generated by Unary1Concrete multiclass above

// SUBEi8 now generated by ALU8Concrete multiclass above

// CMPEi8 now generated by ALU8Concrete multiclass above

// CMPUi16 now generated by ALU16Concrete multiclass above

// LDEi8 now generated by Load8Concrete_P3 multiclass above

// ADDEi8 now generated by ALU8Concrete multiclass above

// CMPSi16 now generated by ALU16Concrete multiclass above

def DIVDi8 : MC6809Immediate8Bit_P3<(outs), "divd", Opcode<0x8D,3>, [NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def DIVQi16 : MC6809Immediate16Bit_P3<(outs), "divq", Opcode<0x8E,3>, [NZ,N,Z,V,C,AA,AB,AD,AE,AF,AW,AQ], [AQ]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def MULDi16 : MC6809Immediate16Bit_P3<(outs), "muld", Opcode<0x8F,3>, [NZ,N,Z,AA,AB,AD,AE,AF,AW,AQ], [AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

// SUBEd now generated by ALU8Concrete multiclass above

// CMPEd now generated by ALU8Concrete multiclass above

// CMPUd now generated by ALU16Concrete multiclass above

// LDEd now generated by Load8Concrete_P3 multiclass above

// STEd now generated by Store8Concrete multiclass above

// ADDEd now generated by ALU8Concrete multiclass above

// CMPSd now generated by ALU16Concrete multiclass above

def DIVDd : MC6809Direct_P3<(outs), "divd", Opcode<0x9D,3>, [NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def DIVQd : MC6809Direct_P3<(outs), "divq", Opcode<0x9E,3>, [NZ,N,Z,V,C,AA,AB,AD,AE,AF,AW,AQ], [AQ]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def MULDd : MC6809Direct_P3<(outs), "muld", Opcode<0x9F,3>, [NZ,N,Z,AA,AB,AD,AE,AF,AW,AQ], [AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

// SUBEi now generated by ALU8Concrete multiclass above

// CMPEi now generated by ALU8Concrete multiclass above

// CMPUi now generated by ALU16Concrete multiclass above

// LDEi now generated by Load8Concrete_P3 multiclass above

// STEi now generated by Store8Concrete multiclass above

// ADDEi now generated by ALU8Concrete multiclass above

// CMPSi now generated by ALU16Concrete multiclass above

let Predicates = [IsHD6309], mayLoad = true in {
  defm DIVDi : MC6809Indexed_P3<"divd", Opcode<0xAD, 3>, (outs), [NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm DIVQi : MC6809Indexed_P3<"divq", Opcode<0xAE, 3>, (outs), [NZ,N,Z,V,C,AA,AB,AD,AE,AF,AW,AQ], [AQ]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm MULDi : MC6809Indexed_P3<"muld", Opcode<0xAF, 3>, (outs), [NZ,N,Z,AA,AB,AD,AE,AF,AW,AQ], [AD]>;
}

// SUBEe now generated by ALU8Concrete multiclass above

// CMPEe now generated by ALU8Concrete multiclass above

// CMPUe now generated by ALU16Concrete multiclass above

// LDEe now generated by Load8Concrete_P3 multiclass above

// STEe now generated by Store8Concrete multiclass above

// ADDEe now generated by ALU8Concrete multiclass above

// CMPSe now generated by ALU16Concrete multiclass above

def DIVDe : MC6809Extended_P3<(outs), "divd", Opcode<0xBD,3>, [NZ,N,Z,V,C,AA,AB,AD,AQ], [AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def DIVQe : MC6809Extended_P3<(outs), "divq", Opcode<0xBE,3>, [NZ,N,Z,V,C,AA,AB,AD,AE,AF,AW,AQ], [AQ]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def MULDe : MC6809Extended_P3<(outs), "muld", Opcode<0xBF,3>, [NZ,N,Z,AA,AB,AD,AE,AF,AW,AQ], [AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

// SUBFi8 now generated by ALU8Concrete multiclass above

// CMPFi8 now generated by ALU8Concrete multiclass above

// LDFi8 now generated by Load8Concrete_P3 multiclass above

// ADDFi8 now generated by ALU8Concrete multiclass above

// SUBFd now generated by ALU8Concrete multiclass above

// CMPFd now generated by ALU8Concrete multiclass above

// LDFd now generated by Load8Concrete_P3 multiclass above

// STFd now generated by Store8Concrete multiclass above

// ADDFd now generated by ALU8Concrete multiclass above

// SUBFi now generated by ALU8Concrete multiclass above

// CMPFi now generated by ALU8Concrete multiclass above

// LDFi now generated by Load8Concrete_P3 multiclass above

// STFi now generated by Store8Concrete multiclass above

// ADDFi now generated by ALU8Concrete multiclass above

// SUBFe now generated by ALU8Concrete multiclass above

// CMPFe now generated by ALU8Concrete multiclass above

// LDFe now generated by Load8Concrete_P3 multiclass above

// STFe now generated by Store8Concrete multiclass above

// ADDFe now generated by ALU8Concrete multiclass above

// MRVM END MARKER 2
// clang-format on

include "MC6809InstrInfoTables.td"

include "MC6809InstrPseudos.td"
include "MC6809InstrLogical.td"
include "MC6809InstrGISel.td"

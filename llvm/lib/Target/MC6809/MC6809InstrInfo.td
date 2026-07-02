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

// Load concrete defm lines are in MC6809InstrFamilies.td

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

// Store concrete defm lines are in MC6809InstrFamilies.td

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
                          [AA,NZ,N,Z,V,C,AA], [AA]>;
defm CMPA : ALU8Concrete<"cmpa", Opcode<0x81,1>, Opcode<0x91,1>, Opcode<0xA1,1>, Opcode<0xB1,1>,
                          [NZ,N,Z,V,C], [AA], /*IsCompare=*/1>;
defm SBCA : ALU8Concrete<"sbca", Opcode<0x82,1>, Opcode<0x92,1>, Opcode<0xA2,1>, Opcode<0xB2,1>,
                          [AA,NZ,N,Z,V,C,AA], [AA,C]>;
defm ANDA : ALU8Concrete<"anda", Opcode<0x84,1>, Opcode<0x94,1>, Opcode<0xA4,1>, Opcode<0xB4,1>,
                          [AA,NZ,N,Z,V,AA], [AA]>;
defm BITA : ALU8Concrete<"bita", Opcode<0x85,1>, Opcode<0x95,1>, Opcode<0xA5,1>, Opcode<0xB5,1>,
                          [NZ,N,Z,V], [AA], /*IsCompare=*/1>;
defm EORA : ALU8Concrete<"eora", Opcode<0x88,1>, Opcode<0x98,1>, Opcode<0xA8,1>, Opcode<0xB8,1>,
                          [AA,NZ,N,Z,V,AA], [AA]>;
defm ADCA : ALU8Concrete<"adca", Opcode<0x89,1>, Opcode<0x99,1>, Opcode<0xA9,1>, Opcode<0xB9,1>,
                          [AA,NZ,N,Z,V,C,AA], [AA,C], /*IsCompare=*/0, /*IsAdd=*/1>;
defm ORA  : ALU8Concrete<"ora",  Opcode<0x8A,1>, Opcode<0x9A,1>, Opcode<0xAA,1>, Opcode<0xBA,1>,
                          [AA,NZ,N,Z,V,AA], [AA]>;
defm ADDA : ALU8Concrete<"adda", Opcode<0x8B,1>, Opcode<0x9B,1>, Opcode<0xAB,1>, Opcode<0xBB,1>,
                          [AA,NZ,N,Z,V,C,AA], [AA], /*IsCompare=*/0, /*IsAdd=*/1>;

// 8-bit ALU: B accumulator (Page 1)
defm SUBB : ALU8Concrete<"subb", Opcode<0xC0,1>, Opcode<0xD0,1>, Opcode<0xE0,1>, Opcode<0xF0,1>,
                          [AB,NZ,N,Z,V,C,AB], [AB]>;
defm CMPB : ALU8Concrete<"cmpb", Opcode<0xC1,1>, Opcode<0xD1,1>, Opcode<0xE1,1>, Opcode<0xF1,1>,
                          [NZ,N,Z,V,C], [AB], /*IsCompare=*/1>;
defm SBCB : ALU8Concrete<"sbcb", Opcode<0xC2,1>, Opcode<0xD2,1>, Opcode<0xE2,1>, Opcode<0xF2,1>,
                          [AB,NZ,N,Z,V,C,AB], [AB,C]>;
defm ANDB : ALU8Concrete<"andb", Opcode<0xC4,1>, Opcode<0xD4,1>, Opcode<0xE4,1>, Opcode<0xF4,1>,
                          [AB,NZ,N,Z,V,AB], [AB]>;
defm BITB : ALU8Concrete<"bitb", Opcode<0xC5,1>, Opcode<0xD5,1>, Opcode<0xE5,1>, Opcode<0xF5,1>,
                          [NZ,N,Z,V], [AB], /*IsCompare=*/1>;
defm EORB : ALU8Concrete<"eorb", Opcode<0xC8,1>, Opcode<0xD8,1>, Opcode<0xE8,1>, Opcode<0xF8,1>,
                          [AB,NZ,N,Z,V,AB], [AB]>;
defm ADCB : ALU8Concrete<"adcb", Opcode<0xC9,1>, Opcode<0xD9,1>, Opcode<0xE9,1>, Opcode<0xF9,1>,
                          [AB,NZ,N,Z,V,C,AB], [AB,C], /*IsCompare=*/0, /*IsAdd=*/1>;
defm ORB  : ALU8Concrete<"orb",  Opcode<0xCA,1>, Opcode<0xDA,1>, Opcode<0xEA,1>, Opcode<0xFA,1>,
                          [AB,NZ,N,Z,V,AB], [AB]>;
defm ADDB : ALU8Concrete<"addb", Opcode<0xCB,1>, Opcode<0xDB,1>, Opcode<0xEB,1>, Opcode<0xFB,1>,
                          [AB,NZ,N,Z,V,C,AB], [AB], /*IsCompare=*/0, /*IsAdd=*/1>;

// 16-bit ALU: D accumulator (Page 1)
defm SUBD : ALU16Concrete<"subd", Opcode<0x83,1>, Opcode<0x93,1>, Opcode<0xA3,1>, Opcode<0xB3,1>,
                           [AD,NZ,N,Z,V,C,AA,AB,AD], [AD]>;
defm ADDD : ALU16Concrete<"addd", Opcode<0xC3,1>, Opcode<0xD3,1>, Opcode<0xE3,1>, Opcode<0xF3,1>,
                           [AD,NZ,N,Z,V,C,AA,AB,AD], [AD], /*IsCompare=*/0, /*IsAdd=*/1>;

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
defm NEG : UnaryAcc8_AB<"nega", Opcode<0x40,1>, [AA,NZ,N,Z,V,C,AA], [AA],
                         "negb", Opcode<0x50,1>, [AB,NZ,N,Z,V,C,AB], [AB]>;
defm COM : UnaryAcc8_AB<"coma", Opcode<0x43,1>, [AA,NZ,N,Z,V,C,AA], [AA],
                         "comb", Opcode<0x53,1>, [AB,NZ,N,Z,V,C,AB], [AB]>;
defm LSR : UnaryAcc8_AB<"lsra", Opcode<0x44,1>, [AA,NZ,N,Z,C,AA], [AA],
                         "lsrb", Opcode<0x54,1>, [AB,NZ,N,Z,C,AB], [AB]>;
defm ROR : UnaryAcc8_AB<"rora", Opcode<0x46,1>, [AA,NZ,N,Z,C,AA], [AA,C],
                         "rorb", Opcode<0x56,1>, [AB,NZ,N,Z,C,AB], [AB,C]>;
// ASR is arithmetic: it fills bit 7 from the sign (bit 7), NOT from carry-in,
// and shifts bit 0 out into carry. So it DEFs C but does not USE it. Modelling
// a carry use here left the first ASRA of a multi-byte arithmetic right shift
// reading an undefined $c (verifier error at -Og's -verify-machineinstrs).
defm ASR : UnaryAcc8_AB<"asra", Opcode<0x47,1>, [AA,NZ,N,Z,C,AA], [AA],
                         "asrb", Opcode<0x57,1>, [AB,NZ,N,Z,C,AB], [AB]>;
defm ASL : UnaryAcc8_AB<"asla", Opcode<0x48,1>, [AA,NZ,N,Z,V,C,AA], [AA],
                         "aslb", Opcode<0x58,1>, [AB,NZ,N,Z,V,C,AB], [AB]>;
// Bug #227: LSL is the same encoding as ASL on MC6809/HD6309 (opcode 0x58
// for accumulator B, 0x08/0x68/0x78 for memory). Both spellings appear in
// hand-written sources and external assemblers (lwasm, asm6809) accept
// each. MnemonicAlias rewrites the input mnemonic before instruction
// matching, so all addressing-mode variants are covered without touching
// the per-form record names.
def : MnemonicAlias<"lsla", "asla">;
def : MnemonicAlias<"lslb", "aslb">;
def : MnemonicAlias<"lsl",  "asl">;
// HD6309 D-register variant. (HD6309 has no ASL/LSL on W or E/F.)
def : MnemonicAlias<"lsld", "asld">;
defm ROL : UnaryAcc8_AB<"rola", Opcode<0x49,1>, [AA,NZ,N,Z,V,C,AA], [AA,C],
                         "rolb", Opcode<0x59,1>, [AB,NZ,N,Z,V,C,AB], [AB,C]>;
defm DEC : UnaryAcc8_AB<"deca", Opcode<0x4A,1>, [AA,NZ,N,Z,V,AA], [AA],
                         "decb", Opcode<0x5A,1>, [AB,NZ,N,Z,V,AB], [AB]>;
defm INC : UnaryAcc8_AB<"inca", Opcode<0x4C,1>, [AA,NZ,N,Z,V,AA], [AA],
                         "incb", Opcode<0x5C,1>, [AB,NZ,N,Z,V,AB], [AB]>;
defm CLR : UnaryAcc8_AB<"clra", Opcode<0x4F,1>, [AA,NZ,N,Z,V,C,AA], [],
                         "clrb", Opcode<0x5F,1>, [AB,NZ,N,Z,V,C,AB], []>;

// TST accumulator forms — separate because isCompare = true and defs differ.
let isCompare = true in {
  def TSTAa : MC6809Accumulator_P1<(outs), "tsta", Opcode<0x4D,1>, [NZ,N,Z,V], [AA]>;
  def TSTBa : MC6809Accumulator_P1<(outs), "tstb", Opcode<0x5D,1>, [NZ,N,Z,V], [AB]>;
}

// LEA: indexed only (Page 1).
//
// CC-clobber asymmetry (Motorola MC6809E spec): LEAX and LEAY set Z based
// on the computed effective address (Z=1 iff result is zero, else Z=0);
// N, V, C are unchanged. LEAS and LEAU intentionally do NOT touch any
// condition codes — that asymmetry is what lets the stack pointers be
// updated without disturbing surrounding flag state. Reflecting this in
// Defs lets the scheduler see the Z-clobber on LEAX/LEAY (so a LEAX
// between a compare and a Z-reading branch can no longer be moved into
// the gap), while leaving LEAS/LEAU as scheduling-free address moves.
// (bug #165 phase A1) NOTE: most LEAX/LEAY reach the MC layer via the
// LEAPtrAdd_* pseudo, whose expansion (expandLEAPtrAdd) uses setDesc — which
// does NOT materialise these implicit Defs — so it re-adds implicit-def $z
// itself (Bug #365).
multiclass LEAConcrete<string Mnem, Opcode IdxOpc, list<Register> Defs> {
  defm i : MC6809Indexed_P1<Mnem, IdxOpc, (outs), Defs, []>;
}

defm LEAX : LEAConcrete<"leax", Opcode<0x30,1>, [IX, Z]>;
defm LEAY : LEAConcrete<"leay", Opcode<0x31,1>, [IY, Z]>;
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

// Stack push/pull (Page 1).
def PSHSs : MC6809Stack_P1<(outs), "pshs", Opcode<0x34,1>, [SS], [SS]>;
def PULSs : MC6809Stack_P1<(outs), "puls", Opcode<0x35,1>, [SS], [SS]>;
def PSHUs : MC6809Stack_P1<(outs), "pshu", Opcode<0x36,1>, [SU], [SU]>;
def PULUs : MC6809Stack_P1<(outs), "pulu", Opcode<0x37,1>, [SU], [SU]>;

// Stack push/pull W register (Page 2, HD6309).
let Predicates = [IsHD6309] in {
  def PSHSWx : MC6809Inherent_P2<(outs), "pshsw", Opcode<0x38,2>, [SS], [SS]>;
  def PULSWx : MC6809Inherent_P2<(outs), "pulsw", Opcode<0x39,2>, [SS], [SS]>;
  def PSHUWx : MC6809Inherent_P2<(outs), "pshuw", Opcode<0x3A,2>, [SU], [SU]>;
  def PULUWx : MC6809Inherent_P2<(outs), "puluw", Opcode<0x3B,2>, [SU], [SU]>;
}

// Register pair ops (Page 1): EXG, TFR.
def EXGp : MC6809RegisterPairSwap_P1<(outs anyregister:$dst2, anyregister:$dst1), "exg", Opcode<0x1E,1>, [], []> {
  let Constraints = "$dst2 = $reg1, $dst1 = $reg2";
}
def TFRp : MC6809RegisterPairCopy_P1<(outs anyregister:$reg2), "tfr", Opcode<0x1F,1>, [], []>;

// Register pair arithmetic (Page 2, HD6309).
let Predicates = [IsHD6309] in {
  let Constraints = "$dst = $reg2" in {
    def ADDRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "addr", Opcode<0x30,2>, [NZ,N,Z,V,C], []> { let isAdd = true; }
    def ADCRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "adcr", Opcode<0x31,2>, [NZ,N,Z,V,C], [C]> { let isAdd = true; }
    def SUBRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "subr", Opcode<0x32,2>, [NZ,N,Z,V,C], []>;
    def SBCRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "sbcr", Opcode<0x33,2>, [NZ,N,Z,V,C], [C]>;
    def ANDRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "andr", Opcode<0x34,2>, [NZ,N,Z,V], []>;
    def ORRp  : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "orr",  Opcode<0x35,2>, [NZ,N,Z,V], []>;
    def EORRp : MC6809RegisterPairArithmetic_P2<(outs anyregister:$dst), "eorr", Opcode<0x36,2>, [NZ,N,Z,V], []>;
  }
  // Bug #205: CMPR is a subtraction-based compare and DOES affect carry
  // (matches every other CMPx in this file: CMPA/B/D/X/Y/U/S/W/E/F all
  // declare [NZ,N,Z,V,C]). The previous [NZ,N,Z,V] omitted C, leading
  // the verifier to flag $c as "undefined" at any LBlbc downstream of
  // CMPRp.
  def CMPRp : MC6809RegisterPairCompare_P2<(outs), "cmpr", Opcode<0x37,2>, [NZ,N,Z,V,C], []> { let isCompare = true; }
}

// JMP: direct/indexed/extended (Page 1). Unconditional jump.
let isBranch = true, isBarrier = true, isTerminator = true in {
  def JMPd  : MC6809Direct_P1<(outs), "jmp", Opcode<0x0E,1>, [], []>;
  defm JMPi : MC6809Indexed_P1<"jmp", Opcode<0x6E,1>, (outs), [], []>;
  def JMPe  : MC6809Extended_P1<(outs), "jmp", Opcode<0x7E,1>, [], []>;
}

// JSR: direct/indexed/extended (Page 1). Subroutine call.
let isCall = true in {
  def JSRd  : MC6809Direct_P1<(outs), "jsr", Opcode<0x9D,1>, [], []>;
  defm JSRi : MC6809Indexed_P1<"jsr", Opcode<0xAD,1>, (outs), [], []>;
  def JSRe  : MC6809Extended_P1<(outs), "jsr", Opcode<0xBD,1>, [], []>;
}

// Branches.
// Unconditional: short (BRA) + long (LBRA). isBarrier because execution
// never falls through.
let mayLoad = false, mayStore = false, isBranch = true, isTerminator = true,
    isBarrier = true in {
  def BRAb   : MC6809ShortBranch_P1<(outs), "bra",  Opcode<0x20,1>, [], []>;
  def LBRAlb : MC6809LongBranch_P1<(outs),  "lbra", Opcode<0x16,1>, [], []>;
}

// Never-branch (effectively NOP): short (BRN) + long (LBRN).
let mayLoad = false, mayStore = false, isBranch = false, isTerminator = false in {
  def BRNb   : MC6809ShortBranch_P1<(outs), "brn",  Opcode<0x21,1>, [], []>;
  def LBRNlb : MC6809LongBranch_P2<(outs),  "lbrn", Opcode<0x21,2>, [], []>;
}

// Conditional: short (Bcc) + long (LBcc). Condition encoded in operand.
// Bug #137: Uses=[N,Z,V,C] is mandatory. Without it the post-RA scheduler
// will happily insert flag-clobbering instructions (LDD, ANDA, NEG, etc.)
// between the CMP that sets the flags and the conditional branch that
// reads them. Pre-RA the dependency rides through a CCond:$bits virtual-
// register operand on the ConditionalBranchRelative pseudo, but
// expandPostRAPseudo strips that operand (it's not part of the assembler
// encoding) — so the post-RA MachineInstr must declare the implicit CC
// reads here, otherwise the scheduler sees a "free" branch.
//
// The union of all condition codes' flag dependencies is N, Z, V, C —
// equivalently the composite NZVC register. Declaring the union here
// (not the per-condition subset) is intentional: $cond is an immediate
// operand whose value the scheduler can't see at instruction-info-class
// granularity, and overconstraining is strictly safer than under-
// constraining (worst case we lose a per-condition reordering opportunity
// the scheduler had no business taking).
let mayLoad = false, mayStore = false, isBranch = true, isTerminator = true in {
  def Bbc   : MC6809ShortConditionalBranch_P1<(outs), "b$cond",  Opcode<0x02,1>, [], [N, Z, V, C]>;
  def LBlbc : MC6809LongConditionalBranch_P2<(outs),  "lb$cond", Opcode<0x02,2>, [], [N, Z, V, C]>;

  // Bug #206: NoC variants for cc values that don't actually consume C
  // at runtime ({EQ, NE, VC, VS, PL, MI, GE, LT, GT, LE, RA, RN}).
  // Encoding-equivalent to Bbc/LBlbc (same opcode byte, same cc-immediate
  // encoding); marked `isCodeGenOnly = 1` so the disassembler always
  // produces the canonical Bbc/LBlbc, not these variants. The verifier
  // sees the tighter Uses = [N, Z, V] and stops complaining about $c
  // being undefined after a TST predecessor (which sets N/Z/V but
  // hardware-preserves C).
  let isCodeGenOnly = 1 in {
    def Bbc_NoC   : MC6809ShortConditionalBranch_P1<(outs), "b$cond",  Opcode<0x02,1>, [], [N, Z, V]>;
    def LBlbc_NoC : MC6809LongConditionalBranch_P2<(outs),  "lb$cond", Opcode<0x02,2>, [], [N, Z, V]>;

    // Bug #271 cat-1 residual: OnlyC variants for cc values that consume
    // ONLY $c at runtime ({HS/CC=4, LO/CS=5}). Encoding-equivalent to
    // Bbc/LBlbc; the verifier sees the tighter Uses = [C] and stops
    // complaining about $n/$z/$v being undefined after a producer chain
    // that includes an intervening Store_i*_Mem (which sets $nz/$v via
    // hardware side-effect but preserves $c). Pattern from
    // c8rtomb / strftime / xdr at -Og.
    def Bbc_OnlyC   : MC6809ShortConditionalBranch_P1<(outs), "b$cond",  Opcode<0x02,1>, [], [C]>;
    def LBlbc_OnlyC : MC6809LongConditionalBranch_P2<(outs),  "lb$cond", Opcode<0x02,2>, [], [C]>;
  }
}

// Subroutine call: short (BSR) + long (LBSR).
let mayLoad = false, mayStore = false, isCall = true in {
  def BSRb   : MC6809ShortBranch_P1<(outs), "bsr",  Opcode<0x8D,1>, [], [SS]>;
  def LBSRlb : MC6809LongBranch_P1<(outs),  "lbsr", Opcode<0x17,1>, [], [SS]>;
}

// Block transfer modes (Page 3, HD6309).
// TFM r0+,r1+ (copy), r0-,r1- (reverse copy), r0+,r1 (fill), r0,r1+ (read)
// TFM uses W as count and reads/writes memory until W reaches 0; mark
// it accordingly so dead-MI-elim doesn't drop it and so the verifier
// is happy when a memoperand is attached.
let Predicates = [IsHD6309], mayLoad = true, mayStore = true,
    hasSideEffects = true, isReMaterializable = false,
    Defs = [AW], Uses = [AW] in {
  // The [AW],[AW] template args matter: the class body's `let Defs = defs`
  // overrides the enclosing let, so empty args would silently erase the W
  // count modeling (TFM reads the count from W and burns it down to zero).
  def TFM0pp : MC6809BlockRegisterPair0_P3<(outs), "tfm", Opcode<0x38,3>, [AW], [AW]>;
  def TFM1pp : MC6809BlockRegisterPair1_P3<(outs), "tfm", Opcode<0x39,3>, [AW], [AW]>;
  def TFM2pp : MC6809BlockRegisterPair2_P3<(outs), "tfm", Opcode<0x3A,3>, [AW], [AW]>;
  def TFM3pp : MC6809BlockRegisterPair3_P3<(outs), "tfm", Opcode<0x3B,3>, [AW], [AW]>;
}

// Bit-direct ops (Page 3, HD6309): BAND, BIAND, BOR, BIOR, BEOR, BIEOR, LDBT, STBT.
let Predicates = [IsHD6309], Constraints = "$dst = $reg" in {
  def BANDbd  : MC6809BitDirect_P3<(outs BIT8:$dst), "band",  Opcode<0x30,3>, [], []>;
  def BIANDbd : MC6809BitDirect_P3<(outs BIT8:$dst), "biand", Opcode<0x31,3>, [], []>;
  def BORbd   : MC6809BitDirect_P3<(outs BIT8:$dst), "bor",   Opcode<0x32,3>, [], []>;
  def BIORbd  : MC6809BitDirect_P3<(outs BIT8:$dst), "bior",  Opcode<0x33,3>, [], []>;
  def BEORbd  : MC6809BitDirect_P3<(outs BIT8:$dst), "beor",  Opcode<0x34,3>, [], []>;
  def BIEORbd : MC6809BitDirect_P3<(outs BIT8:$dst), "bieor", Opcode<0x35,3>, [], []>;
  def LDBTbd  : MC6809BitDirect_P3<(outs BIT8:$dst), "ldbt",  Opcode<0x36,3>, [], []>;
  def STBTbd  : MC6809BitDirect_P3<(outs BIT8:$dst), "stbt",  Opcode<0x37,3>, [], []>;
}

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

// Load defm instantiations are in MC6809InstrFamilies.td

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

// Store P2/P3/STQ concrete defm lines are in MC6809InstrFamilies.td

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

// 16-bit CMP: D, Y, U, S (Page 2)
defm CMPD : ALU16Concrete_P2<"cmpd", Opcode<0x83,2>, Opcode<0x93,2>, Opcode<0xA3,2>, Opcode<0xB3,2>,
                              [NZ,N,Z,V,C], [AD], /*IsCompare=*/1>;
defm CMPY : ALU16Concrete_P2<"cmpy", Opcode<0x8C,2>, Opcode<0x9C,2>, Opcode<0xAC,2>, Opcode<0xBC,2>,
                              [NZ,N,Z,V,C], [IY], /*IsCompare=*/1>;
defm CMPU : ALU16Concrete_P2<"cmpu", Opcode<0x83,3>, Opcode<0x93,3>, Opcode<0xA3,3>, Opcode<0xB3,3>,
                              [NZ,N,Z,V,C], [SU], /*IsCompare=*/1>;
defm CMPS : ALU16Concrete_P2<"cmps", Opcode<0x8C,3>, Opcode<0x9C,3>, Opcode<0xAC,3>, Opcode<0xBC,3>,
                              [NZ,N,Z,V,C], [SS], /*IsCompare=*/1>;

// 16-bit ALU: D register (Page 2, HD6309)
let Predicates = [IsHD6309] in {
  defm SBCD : ALU16Concrete_P2<"sbcd", Opcode<0x82,2>, Opcode<0x92,2>, Opcode<0xA2,2>, Opcode<0xB2,2>,
                                [AD,NZ,N,Z,V,C,AA,AB,AD], [AD,C]>;
  defm ANDD : ALU16Concrete_P2<"andd", Opcode<0x84,2>, Opcode<0x94,2>, Opcode<0xA4,2>, Opcode<0xB4,2>,
                                [AD,NZ,N,Z,V,AA,AB,AD], [AD]>;
  defm BITD : ALU16Concrete_P2<"bitd", Opcode<0x85,2>, Opcode<0x95,2>, Opcode<0xA5,2>, Opcode<0xB5,2>,
                                [NZ,N,Z,V], [AD], /*IsCompare=*/1>;
  defm EORD : ALU16Concrete_P2<"eord", Opcode<0x88,2>, Opcode<0x98,2>, Opcode<0xA8,2>, Opcode<0xB8,2>,
                                [AD,NZ,N,Z,V,AA,AB,AD], [AD]>;
  defm ADCD : ALU16Concrete_P2<"adcd", Opcode<0x89,2>, Opcode<0x99,2>, Opcode<0xA9,2>, Opcode<0xB9,2>,
                                [AD,NZ,N,Z,V,C,AA,AB,AD], [AD,C], /*IsCompare=*/0, /*IsAdd=*/1>;
  defm ORD  : ALU16Concrete_P2<"ord",  Opcode<0x8A,2>, Opcode<0x9A,2>, Opcode<0xAA,2>, Opcode<0xBA,2>,
                                [AD,NZ,N,Z,V,AA,AB,AD], [AD]>;
}

// 16-bit ALU: W register (Page 2, HD6309)
let Predicates = [IsHD6309] in {
  defm SUBW : ALU16Concrete_P2<"subw", Opcode<0x80,2>, Opcode<0x90,2>, Opcode<0xA0,2>, Opcode<0xB0,2>,
                                [AW,NZ,N,Z,V,C,AE,AF,AW], [AW]>;
  defm CMPW : ALU16Concrete_P2<"cmpw", Opcode<0x81,2>, Opcode<0x91,2>, Opcode<0xA1,2>, Opcode<0xB1,2>,
                                [NZ,N,Z,V,C], [AW], /*IsCompare=*/1>;
  defm ADDW : ALU16Concrete_P2<"addw", Opcode<0x8B,2>, Opcode<0x9B,2>, Opcode<0xAB,2>, Opcode<0xBB,2>,
                                [AW,NZ,N,Z,V,C,AE,AF,AW], [AW], /*IsCompare=*/0, /*IsAdd=*/1>;
}

// 8-bit ALU: E accumulator (Page 3, HD6309)
defm SUBE : ALU8Concrete_P3<"sube", Opcode<0x80,3>, Opcode<0x90,3>, Opcode<0xA0,3>, Opcode<0xB0,3>,
                             [AE,NZ,N,Z,V,C,AE], [AE]>;
defm CMPE : ALU8Concrete_P3<"cmpe", Opcode<0x81,3>, Opcode<0x91,3>, Opcode<0xA1,3>, Opcode<0xB1,3>,
                             [NZ,N,Z,V,C], [AE], /*IsCompare=*/1>;
defm ADDE : ALU8Concrete_P3<"adde", Opcode<0x8B,3>, Opcode<0x9B,3>, Opcode<0xAB,3>, Opcode<0xBB,3>,
                             [AE,NZ,N,Z,V,C,AE], [AE], /*IsCompare=*/0, /*IsAdd=*/1>;

// 8-bit ALU: F accumulator (Page 3, HD6309)
defm SUBF : ALU8Concrete_P3<"subf", Opcode<0xC0,3>, Opcode<0xD0,3>, Opcode<0xE0,3>, Opcode<0xF0,3>,
                             [AF,NZ,N,Z,V,C,AF], [AF]>;
defm CMPF : ALU8Concrete_P3<"cmpf", Opcode<0xC1,3>, Opcode<0xD1,3>, Opcode<0xE1,3>, Opcode<0xF1,3>,
                             [NZ,N,Z,V,C], [AF], /*IsCompare=*/1>;
defm ADDF : ALU8Concrete_P3<"addf", Opcode<0xCB,3>, Opcode<0xDB,3>, Opcode<0xEB,3>, Opcode<0xFB,3>,
                             [AF,NZ,N,Z,V,C,AF], [AF], /*IsCompare=*/0, /*IsAdd=*/1>;

// Unary accumulator ops: D and W registers (Page 2, HD6309)
let Predicates = [IsHD6309] in {
  def NEGDa : MC6809Accumulator_P2<(outs), "negd", Opcode<0x40,2>, [AD,NZ,N,Z,V,C,AA,AB,AD], [AD]>;
  def COMDa : MC6809Accumulator_P2<(outs), "comd", Opcode<0x43,2>, [AD,NZ,N,Z,V,C,AA,AB,AD], [AD]>;
  def LSRDa : MC6809Accumulator_P2<(outs), "lsrd", Opcode<0x44,2>, [AD,NZ,N,Z,C,AA,AB,AD], [AD]>;
  def RORDa : MC6809Accumulator_P2<(outs), "rord", Opcode<0x46,2>, [AD,NZ,N,Z,C,AA,AB,AD], [AD,C]>;
  def ASRDa : MC6809Accumulator_P2<(outs), "asrd", Opcode<0x47,2>, [AD,NZ,N,Z,C,AA,AB,AD], [AD]>;  // arithmetic: defs C, does not use it (see ASR)
  def ASLDa : MC6809Accumulator_P2<(outs), "asld", Opcode<0x48,2>, [AD,NZ,N,Z,V,C,AA,AB,AD], [AD]>;
  def ROLDa : MC6809Accumulator_P2<(outs), "rold", Opcode<0x49,2>, [AD,NZ,N,Z,V,C,AA,AB,AD], [AD,C]>;
  def DECDa : MC6809Accumulator_P2<(outs), "decd", Opcode<0x4A,2>, [AD,NZ,N,Z,V,AA,AB,AD], [AD]>;
  def INCDa : MC6809Accumulator_P2<(outs), "incd", Opcode<0x4C,2>, [AD,NZ,N,Z,V,AA,AB,AD], [AD]>;
  def TSTDa : MC6809Accumulator_P2<(outs), "tstd", Opcode<0x4D,2>, [NZ,N,Z,V], [AD]>;
  def CLRDa : MC6809Accumulator_P2<(outs), "clrd", Opcode<0x4F,2>, [AD,NZ,N,Z,V,C,AA,AB,AD], []>;
  def COMWa : MC6809Accumulator_P2<(outs), "comw", Opcode<0x53,2>, [AW,NZ,N,Z,V,C,AE,AF,AW], [AW]>;
  def LSRWa : MC6809Accumulator_P2<(outs), "lsrw", Opcode<0x54,2>, [AW,NZ,N,Z,C,AE,AF,AW], [AW]>;
  def RORWa : MC6809Accumulator_P2<(outs), "rorw", Opcode<0x56,2>, [AW,NZ,N,Z,C,AE,AF,AW], [AW,C]>;
  def ROLWa : MC6809Accumulator_P2<(outs), "rolw", Opcode<0x59,2>, [AW,NZ,N,Z,V,C,AE,AF,AW], [AW,C]>;
  def DECWa : MC6809Accumulator_P2<(outs), "decw", Opcode<0x5A,2>, [AW,NZ,N,Z,V,AE,AF,AW], [AW]>;
  def INCWa : MC6809Accumulator_P2<(outs), "incw", Opcode<0x5C,2>, [AW,NZ,N,Z,V,AE,AF,AW], [AW]>;
  def TSTWa : MC6809Accumulator_P2<(outs), "tstw", Opcode<0x5D,2>, [NZ,N,Z,V], [AW]>;
  def CLRWa : MC6809Accumulator_P2<(outs), "clrw", Opcode<0x5F,2>, [AW,NZ,N,Z,V,C,AE,AF,AW], []>;
}

// Unary accumulator ops: E and F registers (Page 3, HD6309)
let Predicates = [IsHD6309] in {
  def COMEa : MC6809Accumulator_P3<(outs), "come", Opcode<0x43,3>, [AE,NZ,N,Z,V,C,AE], [AE]>;
  def DECEa : MC6809Accumulator_P3<(outs), "dece", Opcode<0x4A,3>, [AE,NZ,N,Z,V,AE], [AE]>;
  def INCEa : MC6809Accumulator_P3<(outs), "ince", Opcode<0x4C,3>, [AE,NZ,N,Z,V,AE], [AE]>;
  def TSTEa : MC6809Accumulator_P3<(outs), "tste", Opcode<0x4D,3>, [NZ,N,Z,V], [AE]>;
  def CLREa : MC6809Accumulator_P3<(outs), "clre", Opcode<0x4F,3>, [AE,NZ,N,Z,V,C,AE], []>;
  def COMFa : MC6809Accumulator_P3<(outs), "comf", Opcode<0x53,3>, [AF,NZ,N,Z,V,C,AF], [AF]>;
  def DECFa : MC6809Accumulator_P3<(outs), "decf", Opcode<0x5A,3>, [AF,NZ,N,Z,V,AF], [AF]>;
  def INCFa : MC6809Accumulator_P3<(outs), "incf", Opcode<0x5C,3>, [AF,NZ,N,Z,V,AF], [AF]>;
  def TSTFa : MC6809Accumulator_P3<(outs), "tstf", Opcode<0x5D,3>, [NZ,N,Z,V], [AF]>;
  def CLRFa : MC6809Accumulator_P3<(outs), "clrf", Opcode<0x5F,3>, [AF,NZ,N,Z,V,C,AF], []>;
}

def NOPx : MC6809Inherent_P1<(outs), "nop", Opcode<0x12,1>, [], []>;

def SYNCx : MC6809Inherent_P1<(outs), "sync", Opcode<0x13,1>, [], []>;

// Bug #247: SEXW sign-extends W into D — it READS AW and WRITES AD.
// Old definition listed Uses=[AD], which told regalloc SEXW *consumed*
// AD's current value. With AD already holding an in-flight value (e.g.,
// the `actual` operand of an i16 != i16 compare in test-timegm), regalloc
// thought it was safe to schedule SEXW there: the value would be
// "consumed" by the read. But SEXW actually clobbers AD with sign(W).
// The prior value was lost, downstream compare/printf saw 0 instead of
// the real value -> "got 0 want 43" pattern across all of test-timegm's
// h/m/s checks. Fix: Uses=[AW] (the real input). Defs trimmed to what
// SEXW actually clobbers: NZ + N + Z + V + (AA, AB, AD) = D's pieces.
// W (AE, AF, AW) is preserved by hardware; listing it in Defs was both
// wrong and obstructed regalloc.
def SEXWx : MC6809Inherent_P1<(outs), "sexw", Opcode<0x14,1>, [NZ,N,Z,V,AA,AB,AD], [AW]> {
  let Predicates = [IsHD6309];
}

def HCFx : MC6809Inherent_P1<(outs), "hcf", Opcode<0x15,1>, [], []>;

def DAAx : MC6809Inherent_P1<(outs), "daa", Opcode<0x19,1>, [AA,NZ,N,Z,C,AA], [AA]>;

def ORCCi8 : MC6809Immediate8Bit_P1<(outs), "orcc", Opcode<0x1A,1>, [NZ,N,Z,V,C], [NZ,N,Z,V,C]>;

def ANDCCi8 : MC6809Immediate8Bit_P1<(outs), "andcc", Opcode<0x1C,1>, [NZ,N,Z,V,C], [NZ,N,Z,V,C]>;

def SEXx : MC6809Inherent_P1<(outs), "sex", Opcode<0x1D,1>, [NZ,N,Z,AA], [AB]>;

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

def MULx : MC6809Inherent_P1<(outs), "mul", Opcode<0x3D,1>, [Z,C,AA,AB,AD], [AA,AB]>;

def RESETx : MC6809Inherent_P1<(outs), "reset", Opcode<0x3E,1>, [], []>;

def SWIx : MC6809Inherent_P1<(outs), "swi", Opcode<0x3F,1>, [], []>;

def SWI2x : MC6809Inherent_P2<(outs), "swi2", Opcode<0x3F,2>, [], []>;

def BITMDi8 : MC6809Immediate8Bit_P3<(outs), "bitmd", Opcode<0x3C,3>, [NZ,N,Z,V], [AD]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
  let mayLoad = true;
}

def LDMDi8 : MC6809Immediate8Bit_P3<(outs), "ldmd", Opcode<0x3D,3>, [], []> {
  let Predicates = [IsHD6309];
}

def SWI3x : MC6809Inherent_P3<(outs), "swi3", Opcode<0x3F,3>, [], []>;

def DIVDi8 : MC6809Immediate8Bit_P3<(outs), "divd", Opcode<0x8D,3>, [NZ,N,Z,V,C,AA,AB,AD], [AD]> {
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

def DIVDd : MC6809Direct_P3<(outs), "divd", Opcode<0x9D,3>, [NZ,N,Z,V,C,AA,AB,AD], [AD]> {
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

let Predicates = [IsHD6309], mayLoad = true in {
  defm DIVDi : MC6809Indexed_P3<"divd", Opcode<0xAD, 3>, (outs), [NZ,N,Z,V,C,AA,AB,AD], [AD]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm DIVQi : MC6809Indexed_P3<"divq", Opcode<0xAE, 3>, (outs), [NZ,N,Z,V,C,AA,AB,AD,AE,AF,AW,AQ], [AQ]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm MULDi : MC6809Indexed_P3<"muld", Opcode<0xAF, 3>, (outs), [NZ,N,Z,AA,AB,AD,AE,AF,AW,AQ], [AD]>;
}

def DIVDe : MC6809Extended_P3<(outs), "divd", Opcode<0xBD,3>, [NZ,N,Z,V,C,AA,AB,AD], [AD]> {
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

include "MC6809InstrInfoTables.td"

include "MC6809InstrPseudos.td"
include "MC6809InstrLogical.td"
include "MC6809InstrGISel.td"
include "MC6809InstrFamilies.td"

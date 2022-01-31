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
  def _oA : MC6809ImmediateIndexedOffsetA_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oB : MC6809ImmediateIndexedOffsetB_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oE : MC6809ImmediateIndexedOffsetE_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809ImmediateIndexedOffsetF_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809ImmediateIndexedOffsetD_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oW : MC6809ImmediateIndexedOffsetW_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
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
  def _o0W : MC6809ImmediateIndexedOffset0W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809ImmediateIndexedOffset16W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809ImmediateIndexedIncW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809ImmediateIndexedDecW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
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
  def _oAI : MC6809ImmediateIndexedOffsetAInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oBI : MC6809ImmediateIndexedOffsetBInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oEI : MC6809ImmediateIndexedOffsetEInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809ImmediateIndexedOffsetFInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809ImmediateIndexedOffsetDInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oWI : MC6809ImmediateIndexedOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
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
  def _0oWI : MC6809ImmediateIndexedNoOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809ImmediateIndexedOffset16WInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809ImmediateIndexedIncW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809ImmediateIndexedDecW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== PAGE MC6809Indexed_P1 ==================================
multiclass MC6809Indexed_P1<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {
  def _o0 : MC6809IndexedOffset0_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o5 : MC6809IndexedOffset5_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8 : MC6809IndexedOffset8_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16 : MC6809IndexedOffset16_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oA : MC6809IndexedOffsetA_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oB : MC6809IndexedOffsetB_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oE : MC6809IndexedOffsetE_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809IndexedOffsetF_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809IndexedOffsetD_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oW : MC6809IndexedOffsetW_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Inc1 : MC6809IndexedInc1_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Inc2 : MC6809IndexedInc2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec1 : MC6809IndexedDec1_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2 : MC6809IndexedDec2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PC : MC6809IndexedOffset8PC_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PC : MC6809IndexedOffset16PC_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o0W : MC6809IndexedOffset0W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809IndexedOffset16W_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809IndexedIncW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809IndexedDecW2_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o0I : MC6809IndexedOffset0Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8I : MC6809IndexedOffset8Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16I : MC6809IndexedOffset16Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oAI : MC6809IndexedOffsetAInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oBI : MC6809IndexedOffsetBInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oEI : MC6809IndexedOffsetEInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809IndexedOffsetFInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809IndexedOffsetDInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oWI : MC6809IndexedOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2I : MC6809IndexedInc2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2I : MC6809IndexedDec2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PCI : MC6809IndexedOffset8PCInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PCI : MC6809IndexedOffset16PCInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _eI : MC6809IndexedExtendedInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _0oWI : MC6809IndexedNoOffsetWInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809IndexedOffset16WInd_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809IndexedIncW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809IndexedDecW2Ind_P1<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== PAGE MC6809Indexed_P2 ==================================
multiclass MC6809Indexed_P2<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {
  def _o0 : MC6809IndexedOffset0_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o5 : MC6809IndexedOffset5_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8 : MC6809IndexedOffset8_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16 : MC6809IndexedOffset16_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oA : MC6809IndexedOffsetA_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oB : MC6809IndexedOffsetB_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oE : MC6809IndexedOffsetE_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809IndexedOffsetF_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809IndexedOffsetD_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oW : MC6809IndexedOffsetW_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Inc1 : MC6809IndexedInc1_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Inc2 : MC6809IndexedInc2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec1 : MC6809IndexedDec1_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2 : MC6809IndexedDec2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PC : MC6809IndexedOffset8PC_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PC : MC6809IndexedOffset16PC_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o0W : MC6809IndexedOffset0W_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809IndexedOffset16W_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809IndexedIncW2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809IndexedDecW2_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o0I : MC6809IndexedOffset0Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8I : MC6809IndexedOffset8Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16I : MC6809IndexedOffset16Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oAI : MC6809IndexedOffsetAInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oBI : MC6809IndexedOffsetBInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oEI : MC6809IndexedOffsetEInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809IndexedOffsetFInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809IndexedOffsetDInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oWI : MC6809IndexedOffsetWInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2I : MC6809IndexedInc2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2I : MC6809IndexedDec2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PCI : MC6809IndexedOffset8PCInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PCI : MC6809IndexedOffset16PCInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _eI : MC6809IndexedExtendedInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _0oWI : MC6809IndexedNoOffsetWInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809IndexedOffset16WInd_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809IndexedIncW2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809IndexedDecW2Ind_P2<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== PAGE MC6809Indexed_P3 ==================================
multiclass MC6809Indexed_P3<string Mnemonic, Opcode opcode, dag outdag, list<Register> defsin, list<Register> usesin> {
  def _o0 : MC6809IndexedOffset0_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o5 : MC6809IndexedOffset5_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8 : MC6809IndexedOffset8_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16 : MC6809IndexedOffset16_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oA : MC6809IndexedOffsetA_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oB : MC6809IndexedOffsetB_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oE : MC6809IndexedOffsetE_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oF : MC6809IndexedOffsetF_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oD : MC6809IndexedOffsetD_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oW : MC6809IndexedOffsetW_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Inc1 : MC6809IndexedInc1_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Inc2 : MC6809IndexedInc2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec1 : MC6809IndexedDec1_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2 : MC6809IndexedDec2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PC : MC6809IndexedOffset8PC_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PC : MC6809IndexedOffset16PC_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o0W : MC6809IndexedOffset0W_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o16W : MC6809IndexedOffset16W_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2 : MC6809IndexedIncW2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2 : MC6809IndexedDecW2_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _o0I : MC6809IndexedOffset0Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8I : MC6809IndexedOffset8Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16I : MC6809IndexedOffset16Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oAI : MC6809IndexedOffsetAInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oBI : MC6809IndexedOffsetBInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oEI : MC6809IndexedOffsetEInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oFI : MC6809IndexedOffsetFInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _oDI : MC6809IndexedOffsetDInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _oWI : MC6809IndexedOffsetWInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _Inc2I : MC6809IndexedInc2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _Dec2I : MC6809IndexedDec2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o8PCI : MC6809IndexedOffset8PCInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _o16PCI : MC6809IndexedOffset16PCInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _eI : MC6809IndexedExtendedInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])>;
  def _0oWI : MC6809IndexedNoOffsetWInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _16oWI : MC6809IndexedOffset16WInd_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _IncW2I : MC6809IndexedIncW2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
  def _DecW2I : MC6809IndexedDecW2Ind_P3<!con(outdag, (outs)), Mnemonic, opcode, !listconcat(defsin, []), !listconcat(usesin, [])> {
    let Predicates = [IsHD6309];
  }
}
// ============================== END PAGES ==================================
// MRVM END MARKER 1
// clang-format on

// clang-format off
// MRVM START MARKER 2
def NEGd : MC6809Direct_P1<(outs), "neg", Opcode<0x00,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def OIMid : MC6809ImmediateDirect_P1<(outs), "oim", Opcode<0x01,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def AIMid : MC6809ImmediateDirect_P1<(outs), "aim", Opcode<0x02,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def COMd : MC6809Direct_P1<(outs), "com", Opcode<0x03,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def LSRd : MC6809Direct_P1<(outs), "lsr", Opcode<0x04,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def EIMid : MC6809ImmediateDirect_P1<(outs), "eim", Opcode<0x05,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def RORd : MC6809Direct_P1<(outs), "ror", Opcode<0x06,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def ASRd : MC6809Direct_P1<(outs), "asr", Opcode<0x07,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def ASLd : MC6809Direct_P1<(outs), "asl", Opcode<0x08,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def ROLd : MC6809Direct_P1<(outs), "rol", Opcode<0x09,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def DECd : MC6809Direct_P1<(outs), "dec", Opcode<0x0A,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def TIMid : MC6809ImmediateDirect_P1<(outs), "tim", Opcode<0x0B,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def INCd : MC6809Direct_P1<(outs), "inc", Opcode<0x0C,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def TSTd : MC6809Direct_P1<(outs), "tst", Opcode<0x0D,1>, [NZVC], []> {
  let isCompare = true;
}

def JMPd : MC6809Direct_P1<(outs), "jmp", Opcode<0x0E,1>, [], []> {
  let isBranch = true;
}

def CLRd : MC6809Direct_P1<(outs), "clr", Opcode<0x0F,1>, [NZVC], []> {
  let mayStore = true;
}

def NOPx : MC6809Inherent_P1<(outs), "nop", Opcode<0x12,1>, [], []>;

def SYNCx : MC6809Inherent_P1<(outs), "sync", Opcode<0x13,1>, [], []>;

def SEXWx : MC6809Inherent_P1<(outs), "sexw", Opcode<0x14,1>, [NZVC,AF], [NZVC,AW]> {
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

def LBSRlb : MC6809LongBranch_P1<(outs), "lbsr", Opcode<0x17,1>, [SU], [SS]> {
  let mayLoad = false;
  let mayStore = false;
  let isCall = true;
}

def DAAx : MC6809Inherent_P1<(outs), "daa", Opcode<0x19,1>, [NZVC], [NZVC]>;

def ORCCi8 : MC6809Immediate8Bit_P1<(outs), "orcc", Opcode<0x1A,1>, [NZVC], [NZVC]>;

def ANDCCi8 : MC6809Immediate8Bit_P1<(outs), "andcc", Opcode<0x1C,1>, [NZVC], [NZVC]>;

def SEXx : MC6809Inherent_P1<(outs), "sex", Opcode<0x1D,1>, [NZVC,AD], [NZVC,AB]>;

def EXGp : MC6809RegisterPair_P1<(outs anyregister:$dst), "exg", Opcode<0x1E,1>, [], []>;

def TFRp : MC6809RegisterPair_P1<(outs anyregister:$dst), "tfr", Opcode<0x1F,1>, [], []>;

def Bbc : MC6809ShortConditionalBranch_P1<(outs), "b$cond", Opcode<0x02,1>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = true;
  let isTerminator = true;
}

defm LEAXi : MC6809Indexed_P1<"leax", Opcode<0x30, 1>, (outs), [IX], []>;

defm LEAYi : MC6809Indexed_P1<"leay", Opcode<0x31, 1>, (outs), [IY], []>;

defm LEASi : MC6809Indexed_P1<"leas", Opcode<0x32, 1>, (outs), [SS], []>;

defm LEAUi : MC6809Indexed_P1<"leau", Opcode<0x33, 1>, (outs), [SU], []>;

def PSHSs : MC6809Stack_P1<(outs), "pshs", Opcode<0x34,1>, [], []>;

def PULSs : MC6809Stack_P1<(outs), "puls", Opcode<0x35,1>, [], []>;

def PSHUs : MC6809Stack_P1<(outs), "pshu", Opcode<0x36,1>, [], []>;

def PULUs : MC6809Stack_P1<(outs), "pulu", Opcode<0x37,1>, [], []>;

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

def MULx : MC6809Inherent_P1<(outs), "mul", Opcode<0x3D,1>, [NZVC], [NZVC]>;

def RESETx : MC6809Inherent_P1<(outs), "reset", Opcode<0x3E,1>, [], []>;

def SWIx : MC6809Inherent_P1<(outs), "swi", Opcode<0x3F,1>, [], []>;

def NEGAa : MC6809Accumulator_P1<(outs), "nega", Opcode<0x40,1>, [NZVC,AA], [NZVC,AA]>;

def COMAa : MC6809Accumulator_P1<(outs), "coma", Opcode<0x43,1>, [NZVC,AA], [NZVC,AA]>;

def LSRAa : MC6809Accumulator_P1<(outs), "lsra", Opcode<0x44,1>, [NZVC,AA], [NZVC,AA]>;

def RORAa : MC6809Accumulator_P1<(outs), "rora", Opcode<0x46,1>, [NZVC,AA], [NZVC,AA]>;

def ASRAa : MC6809Accumulator_P1<(outs), "asra", Opcode<0x47,1>, [NZVC,AA], [NZVC,AA]>;

def ASLAa : MC6809Accumulator_P1<(outs), "asla", Opcode<0x48,1>, [NZVC,AA], [NZVC,AA]>;

def ROLAa : MC6809Accumulator_P1<(outs), "rola", Opcode<0x49,1>, [NZVC,AA], [NZVC,AA]>;

def DECAa : MC6809Accumulator_P1<(outs), "deca", Opcode<0x4A,1>, [NZVC,AA], [NZVC,AA]>;

def INCAa : MC6809Accumulator_P1<(outs), "inca", Opcode<0x4C,1>, [NZVC,AA], [NZVC,AA]>;

def TSTAa : MC6809Accumulator_P1<(outs), "tsta", Opcode<0x4D,1>, [NZVC], [AA]> {
  let isCompare = true;
}

def CLRAa : MC6809Accumulator_P1<(outs AAc:$reg), "clra", Opcode<0x4F,1>, [NZVC], []> {
  let mayLoad = true;
}

def NEGBa : MC6809Accumulator_P1<(outs), "negb", Opcode<0x50,1>, [NZVC,AB], [NZVC,AB]>;

def COMBa : MC6809Accumulator_P1<(outs), "comb", Opcode<0x53,1>, [NZVC,AB], [NZVC,AB]>;

def LSRBa : MC6809Accumulator_P1<(outs), "lsrb", Opcode<0x54,1>, [NZVC,AB], [NZVC,AB]>;

def RORBa : MC6809Accumulator_P1<(outs), "rorb", Opcode<0x56,1>, [NZVC,AB], [NZVC,AB]>;

def ASRBa : MC6809Accumulator_P1<(outs), "asrb", Opcode<0x57,1>, [NZVC,AB], [NZVC,AB]>;

def ASLBa : MC6809Accumulator_P1<(outs), "aslb", Opcode<0x58,1>, [NZVC,AB], [NZVC,AB]>;

def ROLBa : MC6809Accumulator_P1<(outs), "rolb", Opcode<0x59,1>, [NZVC,AB], [NZVC,AB]>;

def DECBa : MC6809Accumulator_P1<(outs), "decb", Opcode<0x5A,1>, [NZVC,AB], [NZVC,AB]>;

def INCBa : MC6809Accumulator_P1<(outs), "incb", Opcode<0x5C,1>, [NZVC,AB], [NZVC,AB]>;

def TSTBa : MC6809Accumulator_P1<(outs), "tstb", Opcode<0x5D,1>, [NZVC], [AB]> {
  let isCompare = true;
}

def CLRBa : MC6809Accumulator_P1<(outs ABc:$reg), "clrb", Opcode<0x5F,1>, [NZVC], []> {
  let mayLoad = true;
}

let mayLoad = true, mayStore = true in {
  defm NEGi : MC6809Indexed_P1<"neg", Opcode<0x60, 1>, (outs), [NZVC], [NZVC]>;
}

let Predicates = [IsHD6309], mayLoad = true, mayStore = true in {
  defm OIMii : MC6809ImmediateIndexed_P1<"oim", Opcode<0x61, 1>, (outs), [NZVC], [NZVC]>;
}

let Predicates = [IsHD6309], mayLoad = true, mayStore = true in {
  defm AIMii : MC6809ImmediateIndexed_P1<"aim", Opcode<0x62, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm COMi : MC6809Indexed_P1<"com", Opcode<0x63, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm LSRi : MC6809Indexed_P1<"lsr", Opcode<0x64, 1>, (outs), [NZVC], [NZVC]>;
}

let Predicates = [IsHD6309], mayLoad = true, mayStore = true in {
  defm EIMii : MC6809ImmediateIndexed_P1<"eim", Opcode<0x65, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm RORi : MC6809Indexed_P1<"ror", Opcode<0x66, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm ASRi : MC6809Indexed_P1<"asr", Opcode<0x67, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm ASLi : MC6809Indexed_P1<"asl", Opcode<0x68, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm ROLi : MC6809Indexed_P1<"rol", Opcode<0x69, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm DECi : MC6809Indexed_P1<"dec", Opcode<0x6A, 1>, (outs), [NZVC], [NZVC]>;
}

let Predicates = [IsHD6309], mayLoad = true, mayStore = true in {
  defm TIMii : MC6809ImmediateIndexed_P1<"tim", Opcode<0x6B, 1>, (outs), [NZVC], [NZVC]>;
}

let mayLoad = true, mayStore = true in {
  defm INCi : MC6809Indexed_P1<"inc", Opcode<0x6C, 1>, (outs), [NZVC], [NZVC]>;
}

let isCompare = true in {
  defm TSTi : MC6809Indexed_P1<"tst", Opcode<0x6D, 1>, (outs), [NZVC], []>;
}

let isBranch = true in {
  defm JMPi : MC6809Indexed_P1<"jmp", Opcode<0x6E, 1>, (outs), [], []>;
}

let mayStore = true in {
  defm CLRi : MC6809Indexed_P1<"clr", Opcode<0x6F, 1>, (outs), [NZVC], []>;
}

def NEGe : MC6809Extended_P1<(outs), "neg", Opcode<0x70,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def OIMie : MC6809ImmediateExtended_P1<(outs), "oim", Opcode<0x71,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def AIMie : MC6809ImmediateExtended_P1<(outs), "aim", Opcode<0x72,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def COMe : MC6809Extended_P1<(outs), "com", Opcode<0x73,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def LSRe : MC6809Extended_P1<(outs), "lsr", Opcode<0x74,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def EIMie : MC6809ImmediateExtended_P1<(outs), "eim", Opcode<0x75,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def RORe : MC6809Extended_P1<(outs), "ror", Opcode<0x76,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def ASRe : MC6809Extended_P1<(outs), "asr", Opcode<0x77,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def ASLe : MC6809Extended_P1<(outs), "asl", Opcode<0x78,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def ROLe : MC6809Extended_P1<(outs), "rol", Opcode<0x79,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def DECe : MC6809Extended_P1<(outs), "dec", Opcode<0x7A,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def TIMie : MC6809ImmediateExtended_P1<(outs), "tim", Opcode<0x7B,1>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
  let mayStore = true;
}

def INCe : MC6809Extended_P1<(outs), "inc", Opcode<0x7C,1>, [NZVC], [NZVC]> {
  let mayLoad = true;
  let mayStore = true;
}

def TSTe : MC6809Extended_P1<(outs), "tst", Opcode<0x7D,1>, [NZVC], []> {
  let isCompare = true;
}

def JMPe : MC6809Extended_P1<(outs), "jmp", Opcode<0x7E,1>, [], []> {
  let isBranch = true;
}

def CLRe : MC6809Extended_P1<(outs), "clr", Opcode<0x7F,1>, [NZVC], []> {
  let mayStore = true;
}

def SUBAi8 : MC6809Immediate8Bit_P1<(outs), "suba", Opcode<0x80,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def CMPAi8 : MC6809Immediate8Bit_P1<(outs), "cmpa", Opcode<0x81,1>, [NZVC], [AA]> {
  let isCompare = true;
}

def SBCAi8 : MC6809Immediate8Bit_P1<(outs), "sbca", Opcode<0x82,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def SUBDi16 : MC6809Immediate16Bit_P1<(outs), "subd", Opcode<0x83,1>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def ANDAi8 : MC6809Immediate8Bit_P1<(outs), "anda", Opcode<0x84,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def BITAi8 : MC6809Immediate8Bit_P1<(outs), "bita", Opcode<0x85,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def LDAi8 : MC6809Immediate8Bit_P1<(outs AAc:$reg), "lda", Opcode<0x86,1>, [NZVC], []> {
  let mayLoad = true;
}

def EORAi8 : MC6809Immediate8Bit_P1<(outs), "eora", Opcode<0x88,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ADCAi8 : MC6809Immediate8Bit_P1<(outs), "adca", Opcode<0x89,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ORAi8 : MC6809Immediate8Bit_P1<(outs), "ora", Opcode<0x8A,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ADDAi8 : MC6809Immediate8Bit_P1<(outs), "adda", Opcode<0x8B,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def CMPXi16 : MC6809Immediate16Bit_P1<(outs), "cmpx", Opcode<0x8C,1>, [NZVC], [IX]> {
  let isCompare = true;
}

def BSRb : MC6809ShortBranch_P1<(outs), "bsr", Opcode<0x8D,1>, [SU], [SS]> {
  let mayLoad = false;
  let mayStore = false;
  let isCall = true;
}

def LDXi16 : MC6809Immediate16Bit_P1<(outs IXc:$reg), "ldx", Opcode<0x8E,1>, [NZVC], []> {
  let mayLoad = true;
}

def SUBAd : MC6809Direct_P1<(outs), "suba", Opcode<0x90,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def CMPAd : MC6809Direct_P1<(outs), "cmpa", Opcode<0x91,1>, [NZVC], [AA]> {
  let isCompare = true;
}

def SBCAd : MC6809Direct_P1<(outs), "sbca", Opcode<0x92,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def SUBDd : MC6809Direct_P1<(outs), "subd", Opcode<0x93,1>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def ANDAd : MC6809Direct_P1<(outs), "anda", Opcode<0x94,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def BITAd : MC6809Direct_P1<(outs), "bita", Opcode<0x95,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def LDAd : MC6809Direct_P1<(outs AAc:$reg), "lda", Opcode<0x96,1>, [NZVC], []> {
  let mayLoad = true;
}

def STAd : MC6809Direct_P1<(outs), "sta", Opcode<0x97,1>, [NZVC], []> {
  let mayStore = true;
}

def EORAd : MC6809Direct_P1<(outs), "eora", Opcode<0x98,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ADCAd : MC6809Direct_P1<(outs), "adca", Opcode<0x99,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ORAd : MC6809Direct_P1<(outs), "ora", Opcode<0x9A,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ADDAd : MC6809Direct_P1<(outs), "adda", Opcode<0x9B,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def CMPXd : MC6809Direct_P1<(outs), "cmpx", Opcode<0x9C,1>, [NZVC], [IX]> {
  let isCompare = true;
}

def JSRd : MC6809Direct_P1<(outs), "jsr", Opcode<0x9D,1>, [], []> {
  let isCall = true;
}

def LDXd : MC6809Direct_P1<(outs IXc:$reg), "ldx", Opcode<0x9E,1>, [NZVC], []> {
  let mayLoad = true;
}

def STXd : MC6809Direct_P1<(outs), "stx", Opcode<0x9F,1>, [NZVC], []> {
  let mayStore = true;
}

let mayLoad = true in {
  defm SUBAi : MC6809Indexed_P1<"suba", Opcode<0xA0, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let isCompare = true in {
  defm CMPAi : MC6809Indexed_P1<"cmpa", Opcode<0xA1, 1>, (outs), [NZVC], [AA]>;
}

let mayLoad = true in {
  defm SBCAi : MC6809Indexed_P1<"sbca", Opcode<0xA2, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let mayLoad = true in {
  defm SUBDi : MC6809Indexed_P1<"subd", Opcode<0xA3, 1>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let mayLoad = true in {
  defm ANDAi : MC6809Indexed_P1<"anda", Opcode<0xA4, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let mayLoad = true in {
  defm BITAi : MC6809Indexed_P1<"bita", Opcode<0xA5, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let mayLoad = true in {
  defm LDAi : MC6809Indexed_P1<"lda", Opcode<0xA6, 1>, (outs AAc:$reg), [NZVC], []>;
}

let mayStore = true in {
  defm STAi : MC6809Indexed_P1<"sta", Opcode<0xA7, 1>, (outs), [NZVC], []>;
}

let mayLoad = true in {
  defm EORAi : MC6809Indexed_P1<"eora", Opcode<0xA8, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let mayLoad = true in {
  defm ADCAi : MC6809Indexed_P1<"adca", Opcode<0xA9, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let mayLoad = true in {
  defm ORAi : MC6809Indexed_P1<"ora", Opcode<0xAA, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let mayLoad = true in {
  defm ADDAi : MC6809Indexed_P1<"adda", Opcode<0xAB, 1>, (outs), [NZVC,AA], [NZVC,AA]>;
}

let isCompare = true in {
  defm CMPXi : MC6809Indexed_P1<"cmpx", Opcode<0xAC, 1>, (outs), [NZVC], [IX]>;
}

let isCall = true in {
  defm JSRi : MC6809Indexed_P1<"jsr", Opcode<0xAD, 1>, (outs), [], []>;
}

let mayLoad = true in {
  defm LDXi : MC6809Indexed_P1<"ldx", Opcode<0xAE, 1>, (outs IXc:$reg), [NZVC], []>;
}

let mayStore = true in {
  defm STXi : MC6809Indexed_P1<"stx", Opcode<0xAF, 1>, (outs), [NZVC], []>;
}

def SUBAe : MC6809Extended_P1<(outs), "suba", Opcode<0xB0,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def CMPAe : MC6809Extended_P1<(outs), "cmpa", Opcode<0xB1,1>, [NZVC], [AA]> {
  let isCompare = true;
}

def SBCAe : MC6809Extended_P1<(outs), "sbca", Opcode<0xB2,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def SUBDe : MC6809Extended_P1<(outs), "subd", Opcode<0xB3,1>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def ANDAe : MC6809Extended_P1<(outs), "anda", Opcode<0xB4,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def BITAe : MC6809Extended_P1<(outs), "bita", Opcode<0xB5,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def LDAe : MC6809Extended_P1<(outs AAc:$reg), "lda", Opcode<0xB6,1>, [NZVC], []> {
  let mayLoad = true;
}

def STAe : MC6809Extended_P1<(outs), "sta", Opcode<0xB7,1>, [NZVC], []> {
  let mayStore = true;
}

def EORAe : MC6809Extended_P1<(outs), "eora", Opcode<0xB8,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ADCAe : MC6809Extended_P1<(outs), "adca", Opcode<0xB9,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ORAe : MC6809Extended_P1<(outs), "ora", Opcode<0xBA,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def ADDAe : MC6809Extended_P1<(outs), "adda", Opcode<0xBB,1>, [NZVC,AA], [NZVC,AA]> {
  let mayLoad = true;
}

def CMPXe : MC6809Extended_P1<(outs), "cmpx", Opcode<0xBC,1>, [NZVC], [IX]> {
  let isCompare = true;
}

def JSRe : MC6809Extended_P1<(outs), "jsr", Opcode<0xBD,1>, [], []> {
  let isCall = true;
}

def LDXe : MC6809Extended_P1<(outs IXc:$reg), "ldx", Opcode<0xBE,1>, [NZVC], []> {
  let mayLoad = true;
}

def STXe : MC6809Extended_P1<(outs), "stx", Opcode<0xBF,1>, [NZVC], []> {
  let mayStore = true;
}

def SUBBi8 : MC6809Immediate8Bit_P1<(outs), "subb", Opcode<0xC0,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def CMPBi8 : MC6809Immediate8Bit_P1<(outs), "cmpb", Opcode<0xC1,1>, [NZVC], [AB]> {
  let isCompare = true;
}

def SBCBi8 : MC6809Immediate8Bit_P1<(outs), "sbcb", Opcode<0xC2,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADDDi16 : MC6809Immediate16Bit_P1<(outs), "addd", Opcode<0xC3,1>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def ANDBi8 : MC6809Immediate8Bit_P1<(outs), "andb", Opcode<0xC4,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def BITBi8 : MC6809Immediate8Bit_P1<(outs), "bitb", Opcode<0xC5,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def LDBi8 : MC6809Immediate8Bit_P1<(outs ABc:$reg), "ldb", Opcode<0xC6,1>, [NZVC], []> {
  let mayLoad = true;
}

def EORBi8 : MC6809Immediate8Bit_P1<(outs), "eorb", Opcode<0xC8,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADCBi8 : MC6809Immediate8Bit_P1<(outs), "adcb", Opcode<0xC9,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ORBi8 : MC6809Immediate8Bit_P1<(outs), "orb", Opcode<0xCA,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADDBi8 : MC6809Immediate8Bit_P1<(outs), "addb", Opcode<0xCB,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def LDDi16 : MC6809Immediate16Bit_P1<(outs ADc:$reg), "ldd", Opcode<0xCC,1>, [NZVC], []> {
  let mayLoad = true;
}

def LDQi32 : MC6809Immediate32Bit_P1<(outs AQc:$reg), "ldq", Opcode<0xCD,1>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def LDUi16 : MC6809Immediate16Bit_P1<(outs SUc:$reg), "ldu", Opcode<0xCE,1>, [NZVC], []> {
  let mayLoad = true;
}

def SUBBd : MC6809Direct_P1<(outs), "subb", Opcode<0xD0,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def CMPBd : MC6809Direct_P1<(outs), "cmpb", Opcode<0xD1,1>, [NZVC], [AB]> {
  let isCompare = true;
}

def SBCBd : MC6809Direct_P1<(outs), "sbcb", Opcode<0xD2,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADDDd : MC6809Direct_P1<(outs), "addd", Opcode<0xD3,1>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def ANDBd : MC6809Direct_P1<(outs), "andb", Opcode<0xD4,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def BITBd : MC6809Direct_P1<(outs), "bitb", Opcode<0xD5,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def LDBd : MC6809Direct_P1<(outs ABc:$reg), "ldb", Opcode<0xD6,1>, [NZVC], []> {
  let mayLoad = true;
}

def STBd : MC6809Direct_P1<(outs), "stb", Opcode<0xD7,1>, [NZVC], []> {
  let mayStore = true;
}

def EORBd : MC6809Direct_P1<(outs), "eorb", Opcode<0xD8,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADCBd : MC6809Direct_P1<(outs), "adcb", Opcode<0xD9,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ORBd : MC6809Direct_P1<(outs), "orb", Opcode<0xDA,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADDBd : MC6809Direct_P1<(outs), "addb", Opcode<0xDB,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def LDDd : MC6809Direct_P1<(outs ADc:$reg), "ldd", Opcode<0xDC,1>, [NZVC], []> {
  let mayLoad = true;
}

def STDd : MC6809Direct_P1<(outs), "std", Opcode<0xDD,1>, [NZVC], []> {
  let mayStore = true;
}

def LDUd : MC6809Direct_P1<(outs SUc:$reg), "ldu", Opcode<0xDE,1>, [NZVC], []> {
  let mayLoad = true;
}

def STUd : MC6809Direct_P1<(outs), "stu", Opcode<0xDF,1>, [NZVC], []> {
  let mayStore = true;
}

let mayLoad = true in {
  defm SUBBi : MC6809Indexed_P1<"subb", Opcode<0xE0, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let isCompare = true in {
  defm CMPBi : MC6809Indexed_P1<"cmpb", Opcode<0xE1, 1>, (outs), [NZVC], [AB]>;
}

let mayLoad = true in {
  defm SBCBi : MC6809Indexed_P1<"sbcb", Opcode<0xE2, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let mayLoad = true in {
  defm ADDDi : MC6809Indexed_P1<"addd", Opcode<0xE3, 1>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let mayLoad = true in {
  defm ANDBi : MC6809Indexed_P1<"andb", Opcode<0xE4, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let mayLoad = true in {
  defm BITBi : MC6809Indexed_P1<"bitb", Opcode<0xE5, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let mayLoad = true in {
  defm LDBi : MC6809Indexed_P1<"ldb", Opcode<0xE6, 1>, (outs ABc:$reg), [NZVC], []>;
}

let mayStore = true in {
  defm STBi : MC6809Indexed_P1<"stb", Opcode<0xE7, 1>, (outs), [NZVC], []>;
}

let mayLoad = true in {
  defm EORBi : MC6809Indexed_P1<"eorb", Opcode<0xE8, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let mayLoad = true in {
  defm ADCBi : MC6809Indexed_P1<"adcb", Opcode<0xE9, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let mayLoad = true in {
  defm ORBi : MC6809Indexed_P1<"orb", Opcode<0xEA, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let mayLoad = true in {
  defm ADDBi : MC6809Indexed_P1<"addb", Opcode<0xEB, 1>, (outs), [NZVC,AB], [NZVC,AB]>;
}

let mayLoad = true in {
  defm LDDi : MC6809Indexed_P1<"ldd", Opcode<0xEC, 1>, (outs ADc:$reg), [NZVC], []>;
}

let mayStore = true in {
  defm STDi : MC6809Indexed_P1<"std", Opcode<0xED, 1>, (outs), [NZVC], []>;
}

let mayLoad = true in {
  defm LDUi : MC6809Indexed_P1<"ldu", Opcode<0xEE, 1>, (outs SUc:$reg), [NZVC], []>;
}

let mayStore = true in {
  defm STUi : MC6809Indexed_P1<"stu", Opcode<0xEF, 1>, (outs), [NZVC], []>;
}

def SUBBe : MC6809Extended_P1<(outs), "subb", Opcode<0xF0,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def CMPBe : MC6809Extended_P1<(outs), "cmpb", Opcode<0xF1,1>, [NZVC], [AB]> {
  let isCompare = true;
}

def SBCBe : MC6809Extended_P1<(outs), "sbcb", Opcode<0xF2,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADDDe : MC6809Extended_P1<(outs), "addd", Opcode<0xF3,1>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def ANDBe : MC6809Extended_P1<(outs), "andb", Opcode<0xF4,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def BITBe : MC6809Extended_P1<(outs), "bitb", Opcode<0xF5,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def LDBe : MC6809Extended_P1<(outs ABc:$reg), "ldb", Opcode<0xF6,1>, [NZVC], []> {
  let mayLoad = true;
}

def STBe : MC6809Extended_P1<(outs), "stb", Opcode<0xF7,1>, [NZVC], []> {
  let mayStore = true;
}

def EORBe : MC6809Extended_P1<(outs), "eorb", Opcode<0xF8,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADCBe : MC6809Extended_P1<(outs), "adcb", Opcode<0xF9,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ORBe : MC6809Extended_P1<(outs), "orb", Opcode<0xFA,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def ADDBe : MC6809Extended_P1<(outs), "addb", Opcode<0xFB,1>, [NZVC,AB], [NZVC,AB]> {
  let mayLoad = true;
}

def LDDe : MC6809Extended_P1<(outs ADc:$reg), "ldd", Opcode<0xFC,1>, [NZVC], []> {
  let mayLoad = true;
}

def STDe : MC6809Extended_P1<(outs), "std", Opcode<0xFD,1>, [NZVC], []> {
  let mayStore = true;
}

def LDUe : MC6809Extended_P1<(outs SUc:$reg), "ldu", Opcode<0xFE,1>, [NZVC], []> {
  let mayLoad = true;
}

def STUe : MC6809Extended_P1<(outs), "stu", Opcode<0xFF,1>, [NZVC], []> {
  let mayStore = true;
}

def LBlbc : MC6809LongConditionalBranch_P2<(outs), "lb$cond", Opcode<0x02,2>, [], []> {
  let mayLoad = false;
  let mayStore = false;
  let isBranch = true;
  let isTerminator = true;
}

def ADDRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "addr", Opcode<0x30,2>, [NZVC], [NZVC]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADCRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "adcr", Opcode<0x31,2>, [NZVC], [NZVC]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def SUBRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "subr", Opcode<0x32,2>, [NZVC], [NZVC]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def SBCRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "sbcr", Opcode<0x33,2>, [NZVC], [NZVC]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ANDRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "andr", Opcode<0x34,2>, [NZVC], [NZVC]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ORRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "orr", Opcode<0x35,2>, [NZVC], [NZVC]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def EORRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "eorr", Opcode<0x36,2>, [NZVC], [NZVC]> {
  let Constraints = "$dst = $reg2";
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPRp : MC6809RegisterPair_P2<(outs anyregister:$dst), "cmpr", Opcode<0x37,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def PSHSWx : MC6809Inherent_P2<(outs), "pshsw", Opcode<0x38,2>, [], []> {
  let Predicates = [IsHD6309];
}

def PULSWx : MC6809Inherent_P2<(outs), "pulsw", Opcode<0x39,2>, [], []> {
  let Predicates = [IsHD6309];
}

def PSHUWx : MC6809Inherent_P2<(outs), "pshuw", Opcode<0x3A,2>, [], []> {
  let Predicates = [IsHD6309];
}

def PULUWx : MC6809Inherent_P2<(outs), "puluw", Opcode<0x3B,2>, [], []> {
  let Predicates = [IsHD6309];
}

def SWI2x : MC6809Inherent_P2<(outs), "swi2", Opcode<0x3F,2>, [], []>;

def NEGDa : MC6809Accumulator_P2<(outs), "negd", Opcode<0x40,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def COMDa : MC6809Accumulator_P2<(outs), "comd", Opcode<0x43,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def LSRDa : MC6809Accumulator_P2<(outs), "lsrd", Opcode<0x44,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def RORDa : MC6809Accumulator_P2<(outs), "rord", Opcode<0x46,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def ASRDa : MC6809Accumulator_P2<(outs), "asrd", Opcode<0x47,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def ASLDa : MC6809Accumulator_P2<(outs), "asld", Opcode<0x48,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def ROLDa : MC6809Accumulator_P2<(outs), "rold", Opcode<0x49,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def DECDa : MC6809Accumulator_P2<(outs), "decd", Opcode<0x4A,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def INCDa : MC6809Accumulator_P2<(outs), "incd", Opcode<0x4C,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def TSTDa : MC6809Accumulator_P2<(outs), "tstd", Opcode<0x4D,2>, [NZVC], [AD]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def CLRDa : MC6809Accumulator_P2<(outs ADc:$reg), "clrd", Opcode<0x4F,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def COMWa : MC6809Accumulator_P2<(outs), "comw", Opcode<0x53,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
}

def LSRWa : MC6809Accumulator_P2<(outs), "lsrw", Opcode<0x54,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
}

def RORWa : MC6809Accumulator_P2<(outs), "rorw", Opcode<0x56,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
}

def ROLWa : MC6809Accumulator_P2<(outs), "rolw", Opcode<0x59,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
}

def DECWa : MC6809Accumulator_P2<(outs), "decw", Opcode<0x5A,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
}

def INCWa : MC6809Accumulator_P2<(outs), "incw", Opcode<0x5C,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
}

def TSTWa : MC6809Accumulator_P2<(outs), "tstw", Opcode<0x5D,2>, [NZVC], [AW]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def CLRWa : MC6809Accumulator_P2<(outs AWc:$reg), "clrw", Opcode<0x5F,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def SUBWi16 : MC6809Immediate16Bit_P2<(outs), "subw", Opcode<0x80,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPWi16 : MC6809Immediate16Bit_P2<(outs), "cmpw", Opcode<0x81,2>, [NZVC], [AW]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def SBCDi16 : MC6809Immediate16Bit_P2<(outs), "sbcd", Opcode<0x82,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPDi16 : MC6809Immediate16Bit_P2<(outs), "cmpd", Opcode<0x83,2>, [NZVC], [AD]> {
  let isCompare = true;
}

def ANDDi16 : MC6809Immediate16Bit_P2<(outs), "andd", Opcode<0x84,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def BITDi16 : MC6809Immediate16Bit_P2<(outs), "bitd", Opcode<0x85,2>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def LDWi16 : MC6809Immediate16Bit_P2<(outs AWc:$reg), "ldw", Opcode<0x86,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def EORDi16 : MC6809Immediate16Bit_P2<(outs), "eord", Opcode<0x88,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADCDi16 : MC6809Immediate16Bit_P2<(outs), "adcd", Opcode<0x89,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ORDi16 : MC6809Immediate16Bit_P2<(outs), "ord", Opcode<0x8A,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADDWi16 : MC6809Immediate16Bit_P2<(outs), "addw", Opcode<0x8B,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPYi16 : MC6809Immediate16Bit_P2<(outs), "cmpy", Opcode<0x8C,2>, [NZVC], [IY]> {
  let isCompare = true;
}

def LDYi16 : MC6809Immediate16Bit_P2<(outs IYc:$reg), "ldy", Opcode<0x8E,2>, [NZVC], []> {
  let mayLoad = true;
}

def SUBWd : MC6809Direct_P2<(outs), "subw", Opcode<0x90,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPWd : MC6809Direct_P2<(outs), "cmpw", Opcode<0x91,2>, [NZVC], [AW]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def SBCDd : MC6809Direct_P2<(outs), "sbcd", Opcode<0x92,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPDd : MC6809Direct_P2<(outs), "cmpd", Opcode<0x93,2>, [NZVC], [AD]> {
  let isCompare = true;
}

def ANDDd : MC6809Direct_P2<(outs), "andd", Opcode<0x94,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def BITDd : MC6809Direct_P2<(outs), "bitd", Opcode<0x95,2>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def LDWd : MC6809Direct_P2<(outs AWc:$reg), "ldw", Opcode<0x96,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STWd : MC6809Direct_P2<(outs), "stw", Opcode<0x97,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def EORDd : MC6809Direct_P2<(outs), "eord", Opcode<0x98,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADCDd : MC6809Direct_P2<(outs), "adcd", Opcode<0x99,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ORDd : MC6809Direct_P2<(outs), "ord", Opcode<0x9A,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADDWd : MC6809Direct_P2<(outs), "addw", Opcode<0x9B,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPYd : MC6809Direct_P2<(outs), "cmpy", Opcode<0x9C,2>, [NZVC], [IY]> {
  let isCompare = true;
}

def LDYd : MC6809Direct_P2<(outs IYc:$reg), "ldy", Opcode<0x9E,2>, [NZVC], []> {
  let mayLoad = true;
}

def STYd : MC6809Direct_P2<(outs), "sty", Opcode<0x9F,2>, [NZVC], []> {
  let mayStore = true;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm SUBWi : MC6809Indexed_P2<"subw", Opcode<0xA0, 2>, (outs), [NZVC,AW], [NZVC,AW]>;
}

let Predicates = [IsHD6309], isCompare = true in {
  defm CMPWi : MC6809Indexed_P2<"cmpw", Opcode<0xA1, 2>, (outs), [NZVC], [AW]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm SBCDi : MC6809Indexed_P2<"sbcd", Opcode<0xA2, 2>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let isCompare = true in {
  defm CMPDi : MC6809Indexed_P2<"cmpd", Opcode<0xA3, 2>, (outs), [NZVC], [AD]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm ANDDi : MC6809Indexed_P2<"andd", Opcode<0xA4, 2>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let mayLoad = true in {
  defm BITDi : MC6809Indexed_P2<"bitd", Opcode<0xA5, 2>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm LDWi : MC6809Indexed_P2<"ldw", Opcode<0xA6, 2>, (outs AWc:$reg), [NZVC], []>;
}

let Predicates = [IsHD6309], mayStore = true in {
  defm STWi : MC6809Indexed_P2<"stw", Opcode<0xA7, 2>, (outs), [NZVC], []>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm EORDi : MC6809Indexed_P2<"eord", Opcode<0xA8, 2>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm ADCDi : MC6809Indexed_P2<"adcd", Opcode<0xA9, 2>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm ORDi : MC6809Indexed_P2<"ord", Opcode<0xAA, 2>, (outs), [NZVC,AD], [NZVC,AD]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm ADDWi : MC6809Indexed_P2<"addw", Opcode<0xAB, 2>, (outs), [NZVC,AW], [NZVC,AW]>;
}

let isCompare = true in {
  defm CMPYi : MC6809Indexed_P2<"cmpy", Opcode<0xAC, 2>, (outs), [NZVC], [IY]>;
}

let mayLoad = true in {
  defm LDYi : MC6809Indexed_P2<"ldy", Opcode<0xAE, 2>, (outs IYc:$reg), [NZVC], []>;
}

let mayStore = true in {
  defm STYi : MC6809Indexed_P2<"sty", Opcode<0xAF, 2>, (outs), [NZVC], []>;
}

def SUBWe : MC6809Extended_P2<(outs), "subw", Opcode<0xB0,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPWe : MC6809Extended_P2<(outs), "cmpw", Opcode<0xB1,2>, [NZVC], [AW]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def SBCDe : MC6809Extended_P2<(outs), "sbcd", Opcode<0xB2,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPDe : MC6809Extended_P2<(outs), "cmpd", Opcode<0xB3,2>, [NZVC], [AD]> {
  let isCompare = true;
}

def ANDDe : MC6809Extended_P2<(outs), "andd", Opcode<0xB4,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def BITDe : MC6809Extended_P2<(outs), "bitd", Opcode<0xB5,2>, [NZVC,AD], [NZVC,AD]> {
  let mayLoad = true;
}

def LDWe : MC6809Extended_P2<(outs AWc:$reg), "ldw", Opcode<0xB6,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STWe : MC6809Extended_P2<(outs), "stw", Opcode<0xB7,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def EORDe : MC6809Extended_P2<(outs), "eord", Opcode<0xB8,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADCDe : MC6809Extended_P2<(outs), "adcd", Opcode<0xB9,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ORDe : MC6809Extended_P2<(outs), "ord", Opcode<0xBA,2>, [NZVC,AD], [NZVC,AD]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADDWe : MC6809Extended_P2<(outs), "addw", Opcode<0xBB,2>, [NZVC,AW], [NZVC,AW]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPYe : MC6809Extended_P2<(outs), "cmpy", Opcode<0xBC,2>, [NZVC], [IY]> {
  let isCompare = true;
}

def LDYe : MC6809Extended_P2<(outs IYc:$reg), "ldy", Opcode<0xBE,2>, [NZVC], []> {
  let mayLoad = true;
}

def STYe : MC6809Extended_P2<(outs), "sty", Opcode<0xBF,2>, [NZVC], []> {
  let mayStore = true;
}

def LDSi16 : MC6809Immediate16Bit_P2<(outs SSc:$reg), "lds", Opcode<0xCE,2>, [NZVC], []> {
  let mayLoad = true;
}

def LDQd : MC6809Direct_P2<(outs AQc:$reg), "ldq", Opcode<0xDC,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STQd : MC6809Direct_P2<(outs), "stq", Opcode<0xDD,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def LDSd : MC6809Direct_P2<(outs SSc:$reg), "lds", Opcode<0xDE,2>, [NZVC], []> {
  let mayLoad = true;
}

def STSd : MC6809Direct_P2<(outs), "sts", Opcode<0xDF,2>, [NZVC], []> {
  let mayStore = true;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm LDQi : MC6809Indexed_P2<"ldq", Opcode<0xEC, 2>, (outs AQc:$reg), [NZVC], []>;
}

let Predicates = [IsHD6309], mayStore = true in {
  defm STQi : MC6809Indexed_P2<"stq", Opcode<0xED, 2>, (outs), [NZVC], []>;
}

let mayLoad = true in {
  defm LDSi : MC6809Indexed_P2<"lds", Opcode<0xEE, 2>, (outs SSc:$reg), [NZVC], []>;
}

let mayStore = true in {
  defm STSi : MC6809Indexed_P2<"sts", Opcode<0xEF, 2>, (outs), [NZVC], []>;
}

def LDQe : MC6809Extended_P2<(outs AQc:$reg), "ldq", Opcode<0xFC,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STQe : MC6809Extended_P2<(outs), "stq", Opcode<0xFD,2>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def LDSe : MC6809Extended_P2<(outs SSc:$reg), "lds", Opcode<0xFE,2>, [NZVC], []> {
  let mayLoad = true;
}

def STSe : MC6809Extended_P2<(outs), "sts", Opcode<0xFF,2>, [NZVC], []> {
  let mayStore = true;
}

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

def TFM0pp : MC6809BlockRegisterPair0_P3<(outs anyregister:$dst), "tfm", Opcode<0x38,3>, [], []> {
  let Predicates = [IsHD6309];
}

def TFM1pp : MC6809BlockRegisterPair1_P3<(outs anyregister:$dst), "tfm", Opcode<0x39,3>, [], []> {
  let Predicates = [IsHD6309];
}

def TFM2pp : MC6809BlockRegisterPair2_P3<(outs anyregister:$dst), "tfm", Opcode<0x3A,3>, [], []> {
  let Predicates = [IsHD6309];
}

def TFM3pp : MC6809BlockRegisterPair3_P3<(outs anyregister:$dst), "tfm", Opcode<0x3B,3>, [], []> {
  let Predicates = [IsHD6309];
}

def BITMDi8 : MC6809Immediate8Bit_P3<(outs), "bitmd", Opcode<0x3C,3>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
}

def LDMDi8 : MC6809Immediate8Bit_P3<(outs), "ldmd", Opcode<0x3D,3>, [], []> {
  let Predicates = [IsHD6309];
}

def SWI3x : MC6809Inherent_P3<(outs), "swi3", Opcode<0x3F,3>, [], []>;

def COMEa : MC6809Accumulator_P3<(outs), "come", Opcode<0x43,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
}

def DECEa : MC6809Accumulator_P3<(outs), "dece", Opcode<0x4A,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
}

def INCEa : MC6809Accumulator_P3<(outs), "ince", Opcode<0x4C,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
}

def TSTEa : MC6809Accumulator_P3<(outs), "tste", Opcode<0x4D,3>, [NZVC], [AE]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def CLREa : MC6809Accumulator_P3<(outs AEc:$reg), "clre", Opcode<0x4F,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def COMFa : MC6809Accumulator_P3<(outs), "comf", Opcode<0x53,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
}

def DECFa : MC6809Accumulator_P3<(outs), "decf", Opcode<0x5A,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
}

def INCFa : MC6809Accumulator_P3<(outs), "incf", Opcode<0x5C,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
}

def TSTFa : MC6809Accumulator_P3<(outs), "tstf", Opcode<0x5D,3>, [NZVC], [AF]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def CLRFa : MC6809Accumulator_P3<(outs AFc:$reg), "clrf", Opcode<0x5F,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def SUBEi8 : MC6809Immediate8Bit_P3<(outs), "sube", Opcode<0x80,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPEi8 : MC6809Immediate8Bit_P3<(outs), "cmpe", Opcode<0x81,3>, [NZVC], [AE]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def CMPUi16 : MC6809Immediate16Bit_P3<(outs), "cmpu", Opcode<0x83,3>, [NZVC], [SU]> {
  let isCompare = true;
}

def LDEi8 : MC6809Immediate8Bit_P3<(outs AEc:$reg), "lde", Opcode<0x86,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADDEi8 : MC6809Immediate8Bit_P3<(outs), "adde", Opcode<0x8B,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPSi16 : MC6809Immediate16Bit_P3<(outs), "cmps", Opcode<0x8C,3>, [NZVC], [SS]> {
  let isCompare = true;
}

def DIVDi8 : MC6809Immediate8Bit_P3<(outs), "divd", Opcode<0x8D,3>, [NZVC,AA,AB], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def DIVQi16 : MC6809Immediate16Bit_P3<(outs), "divq", Opcode<0x8E,3>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
}

def MULDi16 : MC6809Immediate16Bit_P3<(outs), "muld", Opcode<0x8F,3>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
}

def SUBEd : MC6809Direct_P3<(outs), "sube", Opcode<0x90,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPEd : MC6809Direct_P3<(outs), "cmpe", Opcode<0x91,3>, [NZVC], [AE]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def CMPUd : MC6809Direct_P3<(outs), "cmpu", Opcode<0x93,3>, [NZVC], [SU]> {
  let isCompare = true;
}

def LDEd : MC6809Direct_P3<(outs AEc:$reg), "lde", Opcode<0x96,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STEd : MC6809Direct_P3<(outs), "ste", Opcode<0x97,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def ADDEd : MC6809Direct_P3<(outs), "adde", Opcode<0x9B,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPSd : MC6809Direct_P3<(outs), "cmps", Opcode<0x9C,3>, [NZVC], [SS]> {
  let isCompare = true;
}

def DIVDd : MC6809Direct_P3<(outs), "divd", Opcode<0x9D,3>, [NZVC,AA,AB], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def DIVQd : MC6809Direct_P3<(outs), "divq", Opcode<0x9E,3>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
}

def MULDd : MC6809Direct_P3<(outs), "muld", Opcode<0x9F,3>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm SUBEi : MC6809Indexed_P3<"sube", Opcode<0xA0, 3>, (outs), [NZVC,AE], [NZVC,AE]>;
}

let Predicates = [IsHD6309], isCompare = true in {
  defm CMPEi : MC6809Indexed_P3<"cmpe", Opcode<0xA1, 3>, (outs), [NZVC], [AE]>;
}

let isCompare = true in {
  defm CMPUi : MC6809Indexed_P3<"cmpu", Opcode<0xA3, 3>, (outs), [NZVC], [SU]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm LDEi : MC6809Indexed_P3<"lde", Opcode<0xA6, 3>, (outs AEc:$reg), [NZVC], []>;
}

let Predicates = [IsHD6309], mayStore = true in {
  defm STEi : MC6809Indexed_P3<"ste", Opcode<0xA7, 3>, (outs), [NZVC], []>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm ADDEi : MC6809Indexed_P3<"adde", Opcode<0xAB, 3>, (outs), [NZVC,AE], [NZVC,AE]>;
}

let isCompare = true in {
  defm CMPSi : MC6809Indexed_P3<"cmps", Opcode<0xAC, 3>, (outs), [NZVC], [SS]>;
}

let Predicates = [IsHD6309] in {
  defm DIVDi : MC6809Indexed_P3<"divd", Opcode<0xAD, 3>, (outs), [NZVC,AA,AB], [NZVC,AD]>;
}

let Predicates = [IsHD6309] in {
  defm DIVQi : MC6809Indexed_P3<"divq", Opcode<0xAE, 3>, (outs), [NZVC], [NZVC]>;
}

let Predicates = [IsHD6309] in {
  defm MULDi : MC6809Indexed_P3<"muld", Opcode<0xAF, 3>, (outs), [NZVC], [NZVC]>;
}

def SUBEe : MC6809Extended_P3<(outs), "sube", Opcode<0xB0,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPEe : MC6809Extended_P3<(outs), "cmpe", Opcode<0xB1,3>, [NZVC], [AE]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def CMPUe : MC6809Extended_P3<(outs), "cmpu", Opcode<0xB3,3>, [NZVC], [SU]> {
  let isCompare = true;
}

def LDEe : MC6809Extended_P3<(outs AEc:$reg), "lde", Opcode<0xB6,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STEe : MC6809Extended_P3<(outs), "ste", Opcode<0xB7,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def ADDEe : MC6809Extended_P3<(outs), "adde", Opcode<0xBB,3>, [NZVC,AE], [NZVC,AE]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPSe : MC6809Extended_P3<(outs), "cmps", Opcode<0xBC,3>, [NZVC], [SS]> {
  let isCompare = true;
}

def DIVDe : MC6809Extended_P3<(outs), "divd", Opcode<0xBD,3>, [NZVC,AA,AB], [NZVC,AD]> {
  let Predicates = [IsHD6309];
}

def DIVQe : MC6809Extended_P3<(outs), "divq", Opcode<0xBE,3>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
}

def MULDe : MC6809Extended_P3<(outs), "muld", Opcode<0xBF,3>, [NZVC], [NZVC]> {
  let Predicates = [IsHD6309];
}

def SUBFi8 : MC6809Immediate8Bit_P3<(outs), "subf", Opcode<0xC0,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPFi8 : MC6809Immediate8Bit_P3<(outs), "cmpf", Opcode<0xC1,3>, [NZVC], [AF]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def LDFi8 : MC6809Immediate8Bit_P3<(outs AFc:$reg), "ldf", Opcode<0xC6,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def ADDFi8 : MC6809Immediate8Bit_P3<(outs), "addf", Opcode<0xCB,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def SUBFd : MC6809Direct_P3<(outs), "subf", Opcode<0xD0,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPFd : MC6809Direct_P3<(outs), "cmpf", Opcode<0xD1,3>, [NZVC], [AF]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def LDFd : MC6809Direct_P3<(outs AFc:$reg), "ldf", Opcode<0xD6,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STFd : MC6809Direct_P3<(outs), "stf", Opcode<0xD7,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def ADDFd : MC6809Direct_P3<(outs), "addf", Opcode<0xDB,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm SUBFi : MC6809Indexed_P3<"subf", Opcode<0xE0, 3>, (outs), [NZVC,AF], [NZVC,AF]>;
}

let Predicates = [IsHD6309], isCompare = true in {
  defm CMPFi : MC6809Indexed_P3<"cmpf", Opcode<0xE1, 3>, (outs), [NZVC], [AF]>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm LDFi : MC6809Indexed_P3<"ldf", Opcode<0xE6, 3>, (outs AFc:$reg), [NZVC], []>;
}

let Predicates = [IsHD6309], mayStore = true in {
  defm STFi : MC6809Indexed_P3<"stf", Opcode<0xE7, 3>, (outs), [NZVC], []>;
}

let Predicates = [IsHD6309], mayLoad = true in {
  defm ADDFi : MC6809Indexed_P3<"addf", Opcode<0xEB, 3>, (outs), [NZVC,AF], [NZVC,AF]>;
}

def SUBFe : MC6809Extended_P3<(outs), "subf", Opcode<0xF0,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def CMPFe : MC6809Extended_P3<(outs), "cmpf", Opcode<0xF1,3>, [NZVC], [AF]> {
  let Predicates = [IsHD6309];
  let isCompare = true;
}

def LDFe : MC6809Extended_P3<(outs AFc:$reg), "ldf", Opcode<0xF6,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

def STFe : MC6809Extended_P3<(outs), "stf", Opcode<0xF7,3>, [NZVC], []> {
  let Predicates = [IsHD6309];
  let mayStore = true;
}

def ADDFe : MC6809Extended_P3<(outs), "addf", Opcode<0xFB,3>, [NZVC,AF], [NZVC,AF]> {
  let Predicates = [IsHD6309];
  let mayLoad = true;
}

// MRVM END MARKER 2
// clang-format on

include "MC6809InstrInfoTables.td"
include "MC6809InstrPseudos.td"
include "MC6809InstrLogical.td"
include "MC6809InstrGISel.td"

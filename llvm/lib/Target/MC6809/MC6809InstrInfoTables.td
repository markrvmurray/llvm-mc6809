//===- MC6809InstrInfoTables.td - MC6809 Instruction tables -*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes MC6809 constant tables in TableGen format.
//
//===----------------------------------------------------------------------===//

include "llvm/TableGen/SearchableTable.td"

/// ---------------------------------------------------------------------------

/// ---------------------------------------------------------------------------
/// Addressing forms: for each instruction, its family and its form -- an
/// indexed offset (constant or by register), direct page, immediate or
/// inherent -- and, for a load, store, arithmetic, logical, compare, LEA or
/// test, the register it works on and what it does with it. Looked up by
/// opcode, by family and form to find a sibling, and by operation, register
/// and form to choose an instruction (MC6809InstrInfo::buildOpcodeMaps).
def MC6809IndexedForms : GenericTable {
  let FilterClass = "IndexedFormInfo";
  let CppTypeName = "IndexedForm";
  let Fields = ["Inst", "Family", "Form", "IdxValueEnc", "IdxAccess", "ArithOp"];
  let PrimaryKey = ["Inst"];
  let PrimaryKeyName = "getIndexedForm";
}

/// A frame-slot pseudo's static-stack sibling, and back (StaticStackForm).
def getStaticSymForm : InstrMapping {
  let FilterClass = "StaticStackForm";
  let RowFields = ["StaticFamily"];
  let ColFields = ["StaticAccess"];
  let KeyCol = ["Mem"];
  let ValueCols = [["Sym"]];
}

def getFrameSlotForm : InstrMapping {
  let FilterClass = "StaticStackForm";
  let RowFields = ["StaticFamily"];
  let ColFields = ["StaticAccess"];
  let KeyCol = ["Sym"];
  let ValueCols = [["Mem"]];
}

def getIndexedFormByFamily : SearchIndex {
  let Table = MC6809IndexedForms;
  let Key = ["Family", "Form"];
}

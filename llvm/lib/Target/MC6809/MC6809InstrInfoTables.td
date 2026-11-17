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
/// Constant-offset indexed forms: for each instruction, its family and its
/// offset form; looked up by opcode, or by family and form to find a sibling.
def MC6809IndexedForms : GenericTable {
  let FilterClass = "IndexedFormInfo";
  let CppTypeName = "IndexedForm";
  let Fields = ["Inst", "Family", "Form"];
  let PrimaryKey = ["Inst"];
  let PrimaryKeyName = "getIndexedForm";
}

def getIndexedFormByFamily : SearchIndex {
  let Table = MC6809IndexedForms;
  let Key = ["Family", "Form"];
}

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
/// A MC6809 Branch/LongBranch relaxation entry.  Used to relax 8 bit
/// branch instructions that cannot reach their targets, to long branch
/// equivalents.
class BranchInstructionRelaxationEntry< MC6809Inst from, MC6809Inst to> {
  MC6809Inst From = from;
  MC6809Inst To = to;
}

class BRIRE< MC6809Inst from, MC6809Inst to> :
	BranchInstructionRelaxationEntry< from, to >;

def : BRIRE< Bbc, LBlbc >;

/// ---------------------------------------------------------------------------
/// The MC6809 branch instruction lowering table.
def MC6809BranchInstructionRelaxationTable : GenericTable {
  let FilterClass = "BranchInstructionRelaxationEntry";
  let Fields = ["From", "To"];

  let PrimaryKey = ["From"];
  let PrimaryKeyName = "getBranchInstructionRelaxationEntry";
}

/// Direct page section entry.  If a symbol exists in one of these sections,
/// it belongs in direct page.
class MC6809BranchSectionEntry< string name > {
  string Name = name;
}

class BRSE< string name > : MC6809BranchSectionEntry< name >;

def : BRSE<".dp">;
def : BRSE<".directpage">;

def MC6809BranchSectionTable : GenericTable {
  let FilterClass = "MC6809BranchSectionEntry";
  let CppTypeName = "StringRef";
  let Fields = ["Name"];
}

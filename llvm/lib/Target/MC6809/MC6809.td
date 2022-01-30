//===-- MC6809.td - Describe the MC6809 Target Machine ----------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
// This is the top level entry point for the MC6809 target.
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===---------------------------------------------------------------------===//

include "llvm/Target/Target.td"
include "llvm/Target/GlobalISel/Combine.td"

//===---------------------------------------------------------------------===//
// MC6809 Device Definitions
//===---------------------------------------------------------------------===//

include "MC6809Devices.td"
include "MC6809Features.td"
include "MC6809RegisterInfo.td"
include "MC6809RegisterBanks.td"
include "MC6809InstrInfo.td"
include "MC6809Combine.td"

def MC6809InstrInfo : InstrInfo {
  let guessInstructionProperties = 0;
}

include "MC6809CallingConv.td"

def MC6809AsmWriter : AsmWriter {
 string AsmWriterClassName = "InstPrinter";
 bit isMCAsmWriter = 1;
}

def MC6809AsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let ReportMultipleNearMisses = 0;
  string RegisterPrefix = "";
  string TokenizingCharacters = "#(),[]*!";
}

def MC6809AsmParserVariantDefault : AsmParserVariant {
  int Variant = 0;
  let Name = "generic";
}

//===---------------------------------------------------------------------===//
// Target Declaration
//===---------------------------------------------------------------------===//

def MC6809 : Target {
  let InstructionSet         = MC6809InstrInfo;
  let AssemblyWriters        = [MC6809AsmWriter];
  let AllowRegisterRenaming = 1;

  let AssemblyParsers        = [MC6809AsmParser];
  let AssemblyParserVariants = [MC6809AsmParserVariantDefault];
}

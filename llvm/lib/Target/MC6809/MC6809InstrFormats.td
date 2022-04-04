//===-- MC6809InstFormats.td - MC6809 Instruction Formats ----------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MC6809 Instruction Format Definitions.
//
//===----------------------------------------------------------------------===//

/// Predicates. Useful for limiting instructions to particular hardware modes
/// or particular hardware implementations.
def IsHD6309 : Predicate<"Subtarget->isHD6309()">,
               AssemblerPredicate<(all_of Feature6309), "Feature6309"> {
  let PredicateName = "IsHD6309";
}

/// Makes sure that immediate mode operands fit into the required bit field
/// size.  See also *isImm_8 functions in MC6809 code.
class MC6809AsmOperand<string name> : AsmOperandClass {
  let Name = name;
  let DiagnosticType = !strconcat("Invalid", name);
}

class ImmediateAsmOperand<string name> : MC6809AsmOperand<name> {
  let RenderMethod = "addImmOperands";
  let DiagnosticType = "Invalid" # name;
}

class PCRelativeOperand<string name> : MC6809AsmOperand<name>;

class RelativeOperand<string name> : MC6809AsmOperand<name>;

class Addr8Operand<string name> : MC6809AsmOperand<name>;

class Addr16Operand<string name> : MC6809AsmOperand<name>;

def imm3 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm3">;
  let EncoderMethod = "encodeImm3";
}

def imm8 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm8">;
  let EncoderMethod = "encodeImm<MC6809::Imm_8, 1>";
}

def imm16 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm16">;
  let EncoderMethod = "encodeImm<MC6809::Imm16, 1>";
}

def pcrel8 : Operand<i32> {
  let ParserMatchClass = PCRelativeOperand<"PCRel8">;
  let EncoderMethod = "encodeImm<MC6809::PCRel8, 1>";
}

def pcrel16 : Operand<i32> {
  let ParserMatchClass = PCRelativeOperand<"PCRel16">;
  let EncoderMethod = "encodeImm<MC6809::PCRel16, 1>";
}

def offset5 : Operand<i32> {
  let ParserMatchClass = RelativeOperand<"Rel5">;
  let EncoderMethod = "encodeImm<MC6809::Rel5, 1>";
}

def offset8 : Operand<i32> {
  let ParserMatchClass = RelativeOperand<"Rel8">;
  let EncoderMethod = "encodeImm<MC6809::Rel8, 1>";
}

def offset16 : Operand<i32> {
  let ParserMatchClass = RelativeOperand<"Rel16">;
  let EncoderMethod = "encodeImm<MC6809::Rel16, 1>";
}

def addr8 : Operand<i32> {
  let ParserMatchClass = Addr8Operand<"Addr8">;
  let EncoderMethod = "encodeImm<MC6809::Addr8, 1>";
}

def addr16 : Operand<i32> {
  let ParserMatchClass = Addr16Operand<"Addr16">;
  let EncoderMethod = "encodeImm<MC6809::Addr16, 1>";
}

def imm8to16 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm8To16">;
  let EncoderMethod = "encodeImm<MC6809::Imm16, 1>";
}

def label : Operand<OtherVT>;

class RegAsmOperand<string type> : MC6809AsmOperand<"Register"> {
  let Name = "Reg" # type;
  let RenderMethod = "addReg" # type # "Operands";
  let DiagnosticType = "Invalid" # type # "Register";
}

def anyregister : Operand<i8> {
  let EncoderMethod = "encodeRegOpValue";
  let ParserMatchClass = RegAsmOperand<"Any">;
}

// A list of registers separated by comma. Used by load/store multiple.
def RegListAsmOperand : MC6809AsmOperand<"RegList">;
def reglist : Operand<i8> {
  let EncoderMethod = "encodeRegListOpValue";
  let ParserMatchClass = RegListAsmOperand;
  let PrintMethod = "printRegisterList";
  let DecoderMethod = "DecodeRegListOperand";
}

class CondCodeAsmOperand<string name> : MC6809AsmOperand<name> {
  let Name = name;
  let RenderMethod = "addCondCodeOperands";
  let DiagnosticType = "InvalidCondCode";
}

def condcode : Operand<i8> {
  let EncoderMethod = "encodeCondCodeOpValue";
  let ParserMatchClass = CondCodeAsmOperand<"CondCode">;
  let PrintMethod = "printCondCode";
}

/// All non-virtual MC6809 opcodes are 8 or 16 bits long.
class Opcode<bits<8> opcode = 0x00, int page = 0> {
  bits<8> Pre = !if(!eq(page, 2), 0x10, !if(!eq(page, 3), 0x11, 0x00));
  bits<8> Op = opcode;
  int Page = page;
  int OpcodeSize = !if(!eq(page, 1), 1, !if(!eq(page, 2), 2, !if(!eq(page, 3), 2, 0)));
  int PrefixOffset = !if(!eq(page, 1), 0, 8);
}
def DefaultOpcode : Opcode<0, 0>;

/// Addressing modes
/// ----------------
/// Addressing modes on 6x09 do not map to bit fields, except that the
/// indexed mode post-bytes are regular.
///
/// We define the concepts of the addressing modes here, and define how
/// the operands should be parsed per addressing mode.
///
/// The operands are in the a i r y format because tablegen seems to
/// like to collapse multiple characters into one token if they are not
/// separated by spaces. We'll remove those spaces when we print out the
/// operands for any instruction. See also MC6809InstPrinter.cpp.

class MC6809Inst<string mnemonic, Opcode opcode> : Instruction {
  let Namespace = "MC6809";
  bits<64> SoftFail = 0;
  bits<64> Inst = 0;
  bits<8> Pre = opcode.Pre;
  bits<8> Opc = opcode.Op;
  int Size = ?;
  int Page = opcode.Page;
  int OpcodeSize = opcode.OpcodeSize;
  bits<64> P1Inst = 0;
  bits<64> P23Inst = 0;
  string Mode = ?;
  int OperandSize = ?;
  string OperandsStr = ?;
  dag InOperandList = ?;
  string Mnemonic;
  let Mnemonic = mnemonic;
  let AsmString = ?;
  let mayStore = 0;
  let mayLoad = 0;
  let hasSideEffects = true;
}

// clang-format off
// MRVM START MARKER 1
class Accumulator<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{7-0} = Opc;
  let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "a";
  let OperandSize = 0;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins);
  let AsmString = !strconcat(mnemonic, "");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class BitDirect<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> addr;
  bits<2> reg;
  bits<3> srcbit;
  bits<3> dstbit;
  let P1Inst{23-16} = addr{7-0}; let P1Inst{15-14} = reg{1-0}; let P1Inst{13-11} = srcbit; let P1Inst{10-8} = dstbit; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = addr{7-0}; let P23Inst{23-22} = reg{1-0}; let P23Inst{21-19} = srcbit; let P23Inst{18-16} = dstbit; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "bd";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins BIT8:$reg, imm3:$dstbit, imm3:$srcbit, addr8:$addr);
  let AsmString = !strconcat(mnemonic, " $dst , $srcbit , $dstbit , < $addr");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class BlockRegisterPair0<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "pp";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1, anyregister:$reg2);
  let AsmString = !strconcat(mnemonic, " $reg1 + , $reg2 +");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class BlockRegisterPair1<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "pp";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1, anyregister:$reg2);
  let AsmString = !strconcat(mnemonic, " $reg1 - , $reg2 -");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class BlockRegisterPair2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "pp";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1, anyregister:$reg2);
  let AsmString = !strconcat(mnemonic, " $reg1 + , $reg2");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class BlockRegisterPair3<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "pp";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1, anyregister:$reg2);
  let AsmString = !strconcat(mnemonic, " $reg1 , $reg2 +");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Direct<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> addr;
  let P1Inst{15-8} = addr{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = addr{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "d";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins addr8:$addr);
  let AsmString = !strconcat(mnemonic, " < $addr");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Extended<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> addr;
  let P1Inst{23-16} = addr{7-0}; let P1Inst{15-8} = addr{15-8}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = addr{7-0}; let P23Inst{23-16} = addr{15-8}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "e";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins addr16:$addr);
  let AsmString = !strconcat(mnemonic, " $addr");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Immediate16Bit<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> val;
  let P1Inst{23-16} = val{7-0}; let P1Inst{15-8} = val{15-8}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = val{7-0}; let P23Inst{23-16} = val{15-8}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i16";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i16imm:$val);
  let AsmString = !strconcat(mnemonic, " # $val");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Immediate32Bit<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<32> val;
  let P1Inst{39-32} = val{7-0}; let P1Inst{31-24} = val{15-8}; let P1Inst{23-16} = val{23-16}; let P1Inst{15-8} = val{31-24}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = val{7-0}; let P23Inst{39-32} = val{15-8}; let P23Inst{31-24} = val{23-16}; let P23Inst{23-16} = val{31-24}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i32";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i32imm:$val);
  let AsmString = !strconcat(mnemonic, " # $val");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Immediate8Bit<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> val;
  let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i8";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val);
  let AsmString = !strconcat(mnemonic, " # $val");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateDirect<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> addr;
  bits<8> val;
  let P1Inst{23-16} = addr{7-0}; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = addr{7-0}; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "id";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins addr8:$addr, i8imm:$val);
  let AsmString = !strconcat(mnemonic, " # $val ; < $addr");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateExtended<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> addr;
  bits<8> val;
  let P1Inst{31-24} = addr{7-0}; let P1Inst{23-16} = addr{15-8}; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = addr{7-0}; let P23Inst{31-24} = addr{15-8}; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ie";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins addr16:$addr, i8imm:$val);
  let AsmString = !strconcat(mnemonic, " # $val ; $addr");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedDec1<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00010; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00010; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_Dec1";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , - $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedDec2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00011; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00011; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_Dec2";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , -- $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedDec2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b10011; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b10011; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_Dec2I";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ , -- $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedDecW2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> val;
  let P1Inst{23-16} = 0b11101111; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = 0b11101111; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_DecW2";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , -- w");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedDecW2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> val;
  let P1Inst{23-16} = 0b11110000; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = 0b11110000; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_DecW2I";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ , -- w ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedExtendedInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> addr;
  bits<8> val;
  let P1Inst{39-32} = addr{7-0}; let P1Inst{31-24} = addr{15-8}; let P1Inst{23} = 0b1; let P1Inst{22-21} = 0b00; let P1Inst{20-16} = 0b11111; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = addr{7-0}; let P23Inst{39-32} = addr{15-8}; let P23Inst{31} = 0b1; let P23Inst{30-29} = 0b00; let P23Inst{28-24} = 0b11111; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_eI";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, addr16:$addr);
  let AsmString = !strconcat(mnemonic, " $val ; [ $addr ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedInc1<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00000; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00000; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_Inc1";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , $ireg +");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedInc2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00001; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00001; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_Inc2";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , $ireg ++");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedInc2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b10001; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b10001; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_Inc2I";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ , $ireg ++ ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedIncW2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> val;
  let P1Inst{23-16} = 0b11001111; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = 0b11001111; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_IncW2";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , w ++");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedIncW2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> val;
  let P1Inst{23-16} = 0b11010000; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = 0b11010000; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_IncW2I";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ , w ++ ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedNoOffsetWInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> val;
  let P1Inst{23-16} = 0b10010000; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = 0b10010000; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_0oWI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ , w ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset0<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00100; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00100; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o0";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset0Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b10100; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b10100; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o0I";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset0W<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> val;
  let P1Inst{23-16} = 0b10001111; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = 0b10001111; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o0W";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; , w");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset16<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  bits<8> val;
  let P1Inst{39-32} = offset{7-0}; let P1Inst{31-24} = offset{15-8}; let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b01001; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = offset{7-0}; let P23Inst{39-32} = offset{15-8}; let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b01001; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o16";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, offset16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; $offset , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset16Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  bits<8> val;
  let P1Inst{39-32} = offset{7-0}; let P1Inst{31-24} = offset{15-8}; let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b11001; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = offset{7-0}; let P23Inst{39-32} = offset{15-8}; let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b11001; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o16I";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, offset16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ $offset , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset16PC<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  bits<8> val;
  let P1Inst{39-32} = offset{7-0}; let P1Inst{31-24} = offset{15-8}; let P1Inst{23} = 0b1; let P1Inst{22-21} = 0b00; let P1Inst{20-16} = 0b01101; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = offset{7-0}; let P23Inst{39-32} = offset{15-8}; let P23Inst{31} = 0b1; let P23Inst{30-29} = 0b00; let P23Inst{28-24} = 0b01101; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o16PC";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, pcrel16:$offset);
  let AsmString = !strconcat(mnemonic, " $val ; $offset , pc");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset16PCInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  bits<8> val;
  let P1Inst{39-32} = offset{7-0}; let P1Inst{31-24} = offset{15-8}; let P1Inst{23} = 0b1; let P1Inst{22-21} = 0b00; let P1Inst{20-16} = 0b11101; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = offset{7-0}; let P23Inst{39-32} = offset{15-8}; let P23Inst{31} = 0b1; let P23Inst{30-29} = 0b00; let P23Inst{28-24} = 0b11101; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o16PCI";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, pcrel16:$offset);
  let AsmString = !strconcat(mnemonic, " $val ; [ $offset , pc ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset16W<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  bits<8> val;
  let P1Inst{39-32} = offset{7-0}; let P1Inst{31-24} = offset{15-8}; let P1Inst{23-16} = 0b10101111; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = offset{7-0}; let P23Inst{39-32} = offset{15-8}; let P23Inst{31-24} = 0b10101111; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o16W";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, offset16:$offset, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; $offset , w");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset16WInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  bits<8> val;
  let P1Inst{39-32} = offset{7-0}; let P1Inst{31-24} = offset{15-8}; let P1Inst{23-16} = 0b10110000; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{47-40} = offset{7-0}; let P23Inst{39-32} = offset{15-8}; let P23Inst{31-24} = 0b10110000; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_16oWI";
  let OperandSize = 4;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, offset16:$offset, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ $offset , w ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset5<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  bits<8> val;
  let P1Inst{23} = 0b0; let P1Inst{22-21} = ireg; let P1Inst{20-16} = offset{4-0}; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b0; let P23Inst{30-29} = ireg; let P23Inst{28-24} = offset{4-0}; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o5";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, offset5:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; $offset , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset8<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  bits<8> val;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b01000; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b01000; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o8";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, offset8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; $offset , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset8Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  bits<8> val;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b11000; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b11000; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o8I";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, offset8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ $offset , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset8PC<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  bits<8> val;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23} = 0b1; let P1Inst{22-21} = 0b00; let P1Inst{20-16} = 0b01100; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31} = 0b1; let P23Inst{30-29} = 0b00; let P23Inst{28-24} = 0b01100; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o8PC";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, pcrel8:$offset);
  let AsmString = !strconcat(mnemonic, " $val ; $offset , pc");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffset8PCInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  bits<8> val;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23} = 0b1; let P1Inst{22-21} = 0b00; let P1Inst{20-16} = 0b11100; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31} = 0b1; let P23Inst{30-29} = 0b00; let P23Inst{28-24} = 0b11100; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_o8PCI";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, pcrel8:$offset);
  let AsmString = !strconcat(mnemonic, " $val ; [ $offset , pc ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetA<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00110; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00110; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oA";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; a , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetAInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b10110; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b10110; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oAI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ a , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetB<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00101; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00101; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oB";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; b , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetBInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b10101; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b10101; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oBI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ b , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetD<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b01011; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b01011; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oD";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; d , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetDInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b11011; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b11011; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oDI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ d , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetE<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b00111; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b00111; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oE";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; e , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetEInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b10111; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b10111; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oEI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ e , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetF<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b01010; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b01010; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oF";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; f , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetFInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b11010; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b11010; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oFI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ f , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetW<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b01110; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b01110; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oW";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; w , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ImmediateIndexedOffsetWInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<8> val;
  let P1Inst{23} = 0b1; let P1Inst{22-21} = ireg; let P1Inst{20-16} = 0b11110; let P1Inst{15-8} = val{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{31} = 0b1; let P23Inst{30-29} = ireg; let P23Inst{28-24} = 0b11110; let P23Inst{23-16} = val{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "ii_oWI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins i8imm:$val, ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $val ; [ w , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedDec1<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00010; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00010; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_Dec1";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " , - $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedDec2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00011; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00011; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_Dec2";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " , -- $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedDec2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b10011; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b10011; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_Dec2I";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ , -- $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedDecW2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{15-8} = 0b11101111; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = 0b11101111; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_DecW2";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " , -- w");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedDecW2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{15-8} = 0b11110000; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = 0b11110000; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_DecW2I";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ , -- w ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedExtendedInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> addr;
  let P1Inst{31-24} = addr{7-0}; let P1Inst{23-16} = addr{15-8}; let P1Inst{15} = 0b1; let P1Inst{14-13} = 0b00; let P1Inst{12-8} = 0b11111; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = addr{7-0}; let P23Inst{31-24} = addr{15-8}; let P23Inst{23} = 0b1; let P23Inst{22-21} = 0b00; let P23Inst{20-16} = 0b11111; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_eI";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins addr16:$addr);
  let AsmString = !strconcat(mnemonic, " [ $addr ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedInc1<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00000; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00000; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_Inc1";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " , $ireg +");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedInc2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00001; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00001; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_Inc2";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " , $ireg ++");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedInc2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b10001; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b10001; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_Inc2I";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ , $ireg ++ ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedIncW2<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{15-8} = 0b11001111; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = 0b11001111; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_IncW2";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " , w ++");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedIncW2Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{15-8} = 0b11010000; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = 0b11010000; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_IncW2I";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ , w ++ ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedNoOffsetWInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{15-8} = 0b10010000; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = 0b10010000; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_0oWI";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ , w ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset0<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00100; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00100; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o0";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset0Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b10100; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b10100; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o0I";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset0W<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{15-8} = 0b10001111; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = 0b10001111; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o0W";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " , w");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset16<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23-16} = offset{15-8}; let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b01001; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31-24} = offset{15-8}; let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b01001; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o16";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins offset16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $offset , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset16Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23-16} = offset{15-8}; let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b11001; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31-24} = offset{15-8}; let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b11001; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o16I";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins offset16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ $offset , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset16PC<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23-16} = offset{15-8}; let P1Inst{15} = 0b1; let P1Inst{14-13} = 0b00; let P1Inst{12-8} = 0b01101; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31-24} = offset{15-8}; let P23Inst{23} = 0b1; let P23Inst{22-21} = 0b00; let P23Inst{20-16} = 0b01101; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o16PC";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins pcrel16:$offset);
  let AsmString = !strconcat(mnemonic, " $offset , pc");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset16PCInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23-16} = offset{15-8}; let P1Inst{15} = 0b1; let P1Inst{14-13} = 0b00; let P1Inst{12-8} = 0b11101; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31-24} = offset{15-8}; let P23Inst{23} = 0b1; let P23Inst{22-21} = 0b00; let P23Inst{20-16} = 0b11101; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o16PCI";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins pcrel16:$offset);
  let AsmString = !strconcat(mnemonic, " [ $offset , pc ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset16W<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23-16} = offset{15-8}; let P1Inst{15-8} = 0b10101111; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31-24} = offset{15-8}; let P23Inst{23-16} = 0b10101111; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o16W";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins offset16:$offset, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " $offset , w");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset16WInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  let P1Inst{31-24} = offset{7-0}; let P1Inst{23-16} = offset{15-8}; let P1Inst{15-8} = 0b10110000; let P1Inst{7-0} = Opc;
  let P23Inst{39-32} = offset{7-0}; let P23Inst{31-24} = offset{15-8}; let P23Inst{23-16} = 0b10110000; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_16oWI";
  let OperandSize = 3;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins offset16:$offset, ACC16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ $offset , w ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset5<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  let P1Inst{15} = 0b0; let P1Inst{14-13} = ireg; let P1Inst{12-8} = offset{4-0}; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b0; let P23Inst{22-21} = ireg; let P23Inst{20-16} = offset{4-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o5";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins offset5:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $offset , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset8<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  let P1Inst{23-16} = offset{7-0}; let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b01000; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = offset{7-0}; let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b01000; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o8";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins offset8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " $offset , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset8Ind<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  bits<16> offset;
  let P1Inst{23-16} = offset{7-0}; let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b11000; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = offset{7-0}; let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b11000; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o8I";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins offset8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ $offset , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset8PC<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  let P1Inst{23-16} = offset{7-0}; let P1Inst{15} = 0b1; let P1Inst{14-13} = 0b00; let P1Inst{12-8} = 0b01100; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = offset{7-0}; let P23Inst{23} = 0b1; let P23Inst{22-21} = 0b00; let P23Inst{20-16} = 0b01100; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o8PC";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins pcrel8:$offset);
  let AsmString = !strconcat(mnemonic, " $offset , pc");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffset8PCInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> offset;
  let P1Inst{23-16} = offset{7-0}; let P1Inst{15} = 0b1; let P1Inst{14-13} = 0b00; let P1Inst{12-8} = 0b11100; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = offset{7-0}; let P23Inst{23} = 0b1; let P23Inst{22-21} = 0b00; let P23Inst{20-16} = 0b11100; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_o8PCI";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins pcrel8:$offset);
  let AsmString = !strconcat(mnemonic, " [ $offset , pc ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetA<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00110; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00110; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oA";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " a , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetAInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b10110; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b10110; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oAI";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ a , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetB<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00101; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00101; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oB";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " b , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetBInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b10101; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b10101; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oBI";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ b , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetD<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b01011; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b01011; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oD";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " d , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetDInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b11011; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b11011; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oDI";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ d , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetE<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b00111; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b00111; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oE";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " e , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetEInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b10111; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b10111; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oEI";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ e , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetF<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b01010; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b01010; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oF";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " f , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetFInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b11010; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b11010; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oFI";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC8:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ f , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetW<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b01110; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b01110; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oW";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " w , $ireg");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class IndexedOffsetWInd<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<2> ireg;
  let P1Inst{15} = 0b1; let P1Inst{14-13} = ireg; let P1Inst{12-8} = 0b11110; let P1Inst{7-0} = Opc;
  let P23Inst{23} = 0b1; let P23Inst{22-21} = ireg; let P23Inst{20-16} = 0b11110; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "i_oWI";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins ACC16:$offset, INDEX16:$ireg);
  let AsmString = !strconcat(mnemonic, " [ w , $ireg ]");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Inherent<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{7-0} = Opc;
  let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "x";
  let OperandSize = 0;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins);
  let AsmString = !strconcat(mnemonic, "");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class LongBranch<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> tgt;
  let P1Inst{23-16} = tgt{7-0}; let P1Inst{15-8} = tgt{15-8}; let P1Inst{7-0} = Opc;
  let P23Inst{31-24} = tgt{7-0}; let P23Inst{23-16} = tgt{15-8}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "lb";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins label:$tgt);
  let AsmString = !strconcat(mnemonic, " $tgt");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class LongConditionalBranch<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> tgt;
  bits<4> cond;
  let P1Inst{23-16} = tgt{7-0}; let P1Inst{15-8} = tgt{15-8}; let P1Inst{7-4} = Opc{3-0}; let P1Inst{3-0} = cond;
  let P23Inst{31-24} = tgt{7-0}; let P23Inst{23-16} = tgt{15-8}; let P23Inst{15-12} = Opc{3-0}; let P23Inst{11-8} = cond; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "lbc";
  let OperandSize = 2;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins condcode:$cond, label:$tgt);
  let AsmString = !strconcat(mnemonic, " $tgt");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class RegisterPairArithmetic<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "p";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1, anyregister:$reg2);
  let AsmString = !strconcat(mnemonic, " $reg1 , $reg2");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class RegisterPairCompare<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "p";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1, anyregister:$reg2);
  let AsmString = !strconcat(mnemonic, " $reg1 , $reg2");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class RegisterPairCopy<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "p";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1);
  let AsmString = !strconcat(mnemonic, " $reg1 , $reg2");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class RegisterPairSwap<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<4> reg1;
  bits<4> reg2;
  let P1Inst{15-12} = reg1; let P1Inst{11-8} = reg2; let P1Inst{7-0} = Opc;
  let P23Inst{23-20} = reg1; let P23Inst{19-16} = reg2; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "p";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins anyregister:$reg1, anyregister:$reg2);
  let AsmString = !strconcat(mnemonic, " $reg1 , $reg2");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Return<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  let P1Inst{7-0} = Opc;
  let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "r";
  let OperandSize = 0;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins);
  let AsmString = !strconcat(mnemonic, "");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ShortBranch<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> tgt;
  let P1Inst{15-8} = tgt{7-0}; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = tgt{7-0}; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "b";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins label:$tgt);
  let AsmString = !strconcat(mnemonic, " $tgt");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class ShortConditionalBranch<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<16> tgt;
  bits<4> cond;
  let P1Inst{15-8} = tgt{7-0}; let P1Inst{7-4} = Opc{3-0}; let P1Inst{3-0} = cond;
  let P23Inst{23-16} = tgt{7-0}; let P23Inst{15-12} = Opc{3-0}; let P23Inst{11-8} = cond; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "bc";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins condcode:$cond, label:$tgt);
  let AsmString = !strconcat(mnemonic, " $tgt");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

class Stack<string mnemonic, Opcode opcode> : MC6809Inst<mnemonic, opcode> {
  bits<8> regs;
  let P1Inst{15-8} = regs; let P1Inst{7-0} = Opc;
  let P23Inst{23-16} = regs; let P23Inst{15-8} = Opc; let P23Inst{7-0} = Pre{7-0};
  let DecoderNamespace = "_Page_" # Page # "_Size_";
  let Mode = "s";
  let OperandSize = 1;
  let Size = !add(OperandSize, OpcodeSize);
  let InOperandList = (ins reglist:$regs, variable_ops);
  let AsmString = !strconcat(mnemonic, " $regs");
  let Inst = !if(!eq(Page, 1), P1Inst, P23Inst);
}

// MRVM END MARKER 1
// clang-format on

// clang-format off
// MRVM START MARKER 2
class MC6809Accumulator_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Accumulator<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Accumulator_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Accumulator<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Accumulator_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Accumulator<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809BitDirect_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : BitDirect<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809BlockRegisterPair0_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : BlockRegisterPair0<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809BlockRegisterPair1_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : BlockRegisterPair1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809BlockRegisterPair2_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : BlockRegisterPair2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809BlockRegisterPair3_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : BlockRegisterPair3<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Direct_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Direct<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Direct_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Direct<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Direct_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Direct<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Extended_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Extended<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Extended_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Extended<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Extended_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Extended<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Immediate16Bit_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Immediate16Bit<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Immediate16Bit_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Immediate16Bit<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Immediate16Bit_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Immediate16Bit<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Immediate32Bit_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Immediate32Bit<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Immediate8Bit_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Immediate8Bit<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Immediate8Bit_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Immediate8Bit<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateDirect_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateDirect<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateExtended_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateExtended<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedDec1_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedDec1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedDec2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedDec2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedDec2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedDec2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedDecW2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedDecW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedDecW2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedDecW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedExtendedInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedExtendedInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedInc1_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedInc1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedInc2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedInc2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedInc2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedInc2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedIncW2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedIncW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedIncW2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedIncW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedNoOffsetWInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedNoOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset0Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset0Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset0W_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset0W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset0_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset0<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset16Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset16Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset16PCInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset16PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset16PC_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset16PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset16WInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset16WInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset16W_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset16W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset16_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset16<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset5_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset5<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset8Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset8Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset8PCInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset8PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset8PC_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset8PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffset8_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffset8<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetAInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetAInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetA_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetA<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetBInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetBInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetB_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetB<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetDInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetDInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetD_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetD<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetEInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetEInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetE_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetE<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetFInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetFInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetF_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetF<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetWInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ImmediateIndexedOffsetW_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ImmediateIndexedOffsetW<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec1_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec1_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec1_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec2Ind_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec2Ind_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec2_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDec2_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDec2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDecW2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDecW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDecW2Ind_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDecW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDecW2Ind_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDecW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDecW2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDecW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDecW2_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDecW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedDecW2_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedDecW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedExtendedInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedExtendedInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedExtendedInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedExtendedInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedExtendedInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedExtendedInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc1_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc1_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc1_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc1<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc2Ind_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc2Ind_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc2_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedInc2_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedInc2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedIncW2Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedIncW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedIncW2Ind_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedIncW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedIncW2Ind_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedIncW2Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedIncW2_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedIncW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedIncW2_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedIncW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedIncW2_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedIncW2<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedNoOffsetWInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedNoOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedNoOffsetWInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedNoOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedNoOffsetWInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedNoOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0Ind_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0Ind_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0W_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0W_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0W_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset0_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset0<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16Ind_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16Ind_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16PCInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16PCInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16PCInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16PC_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16PC_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16PC_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16WInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16WInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16WInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16WInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16WInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16WInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16W_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16W_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16W_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16W<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset16_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset16<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset5_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset5<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset5_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset5<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset5_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset5<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8Ind_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8Ind_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8Ind_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8Ind<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8PCInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8PCInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8PCInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8PCInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8PC_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8PC_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8PC_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8PC<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffset8_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffset8<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetAInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetAInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetAInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetAInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetAInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetAInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetA_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetA<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetA_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetA<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetA_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetA<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetBInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetBInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetBInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetBInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetBInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetBInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetB_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetB<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetB_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetB<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetB_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetB<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetDInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetDInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetDInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetDInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetDInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetDInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetD_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetD<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetD_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetD<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetD_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetD<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetEInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetEInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetEInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetEInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetEInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetEInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetE_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetE<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetE_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetE<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetE_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetE<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetFInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetFInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetFInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetFInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetFInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetFInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetF_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetF<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetF_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetF<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetF_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetF<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetWInd_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetWInd_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetWInd_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetWInd<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetW_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetW<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetW_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetW<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809IndexedOffsetW_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : IndexedOffsetW<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Inherent_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Inherent<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Inherent_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Inherent<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Inherent_P3<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Inherent<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809LongBranch_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : LongBranch<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809LongConditionalBranch_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : LongConditionalBranch<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809RegisterPairArithmetic_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : RegisterPairArithmetic<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809RegisterPairCompare_P2<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : RegisterPairCompare<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809RegisterPairCopy_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : RegisterPairCopy<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809RegisterPairSwap_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : RegisterPairSwap<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Return_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Return<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ShortBranch_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ShortBranch<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809ShortConditionalBranch_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : ShortConditionalBranch<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

class MC6809Stack_P1<dag outs, string mnemonic, Opcode opc, list<Register> defs, list<Register> uses>
  : Stack<mnemonic, opc> {
    let Defs = defs;
    let Uses = uses;
    let OutOperandList = outs;
}

// MRVM END MARKER 2
// clang-format on

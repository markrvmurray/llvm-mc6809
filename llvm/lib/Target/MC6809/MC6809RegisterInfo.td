//===-- MC6809RegisterInfo.td - MC6809 Register defs -------------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Subregister indices
//===----------------------------------------------------------------------===//

class MC6809SubRegIndex<int size, int offset = 0>: SubRegIndex<size, offset> {
  let Namespace = "MC6809";
}

// Bytes of a 16-bit word
def sub_lo_byte : MC6809SubRegIndex<8>;
def sub_hi_byte : MC6809SubRegIndex<8, 8>;

// 16-bit half-words of a 32-bit word
def sub_lo_word : MC6809SubRegIndex<16>;
def sub_hi_word : MC6809SubRegIndex<16, 16>;

// Bits of the 8-bit processor status register.
def sub_entire : MC6809SubRegIndex<1, 7>;
def sub_firq : MC6809SubRegIndex<1, 6>;
def sub_half : MC6809SubRegIndex<1, 5>;
def sub_irq : MC6809SubRegIndex<1, 4>;
def sub_negative : MC6809SubRegIndex<1, 3>;
def sub_zero : MC6809SubRegIndex<1, 2>;
def sub_overflow : MC6809SubRegIndex<1, 1>;
def sub_carry : MC6809SubRegIndex<1, 0>;

// Bits of the 8-bit processor status register set by most instructions.
def sub_nzvc : MC6809SubRegIndex<4, 0>;

// Least-significant bit of an 8-bit register. Writes to this are considered to
// write zeroes to the other bits (for now), so we can reduce LSB writes to
// zero-extension.
def sub_lsb : MC6809SubRegIndex<1>;

//===----------------------------------------------------------------------===//
//  Real registers
//===----------------------------------------------------------------------===//

class MC6809Reg<bits<16> num, string name>
  : Register<name>, DwarfRegNum<[num]> {
  field bits<16> Num = num;
  let HWEncoding = num;
  let Namespace = "MC6809";
  let AltNames = [name];
}

//class MC6809Reg8<bits<16> num, string name> : MC6809Reg<num, name>;

// An 8-bit value register (GPR or imaginary). Contains a 1-bit LSB subregister
// to allow them to store boolean values.
//multiclass MC6809Reg8<bits<16> num, string name> {
//  def _LSB : MC6809Reg<!add(num, 128), name#"_LSB">;
//
//  def NAME : MC6809Reg<num, name> {
//    let SubRegs = [!cast<Register>(NAME#_LSB)];
//    let SubRegIndices = [sub_lsb];
//  }
//}

class MC6809Reg1<bits<16> num, string name>
  : MC6809Reg<num, name> {
    let SubRegs = [];
    let SubRegIndices = [];
    let CoveredBySubRegs = true;
}

class MC6809Reg8<bits<16> num, string name, list<Register> subregs = [], list<SubRegIndex> subregindices = []>
  : MC6809Reg<num, name> {
    let SubRegs = subregs;
    let SubRegIndices = subregindices;
    let CoveredBySubRegs = false;
}

class MC6809Reg16<bits<16> num, string name, list<Register> subregs = [], list<SubRegIndex> subregindices = []>
  : MC6809Reg<num, name> {
    let SubRegs = subregs;
    let SubRegIndices = subregindices;
    let CoveredBySubRegs = true;
}

class MC6809Reg32<bits<16> num, string name, list<Register> subregs = [], list<SubRegIndex> subregindices = []>
  : MC6809Reg<num, name> {
    let SubRegs = subregs;
    let SubRegIndices = subregindices;
    let CoveredBySubRegs = true;
}

def E : MC6809Reg1<48, "E">;
def F : MC6809Reg1<49, "F">;
def H : MC6809Reg1<50, "H">;
def I : MC6809Reg1<51, "I">;
def N : MC6809Reg1<52, "N">;
def Z : MC6809Reg1<53, "Z">;
def V : MC6809Reg1<54, "V">;
def C : MC6809Reg1<55, "C">;

def NZVC : MC6809Reg<65, "NZVC"> {
  let SubRegs = [N, Z, V, C];
  let SubRegIndices = [sub_negative, sub_zero, sub_overflow, sub_carry];
  let CoveredBySubRegs = true;
}

def CC : MC6809Reg8<10, "cc", [E, F, H, I, NZVC], [sub_entire, sub_firq, sub_half, sub_irq, sub_nzvc]>;

def AA_LSB : MC6809Reg1<66, "a_lsb">;
def AB_LSB : MC6809Reg1<67, "b_lsb">;
def AE_LSB : MC6809Reg1<68, "e_lsb">;
def AF_LSB : MC6809Reg1<69, "f_lsb">;

def IX : MC6809Reg16<1,  "x" >;
def IY : MC6809Reg16<2,  "y" >;
def SU : MC6809Reg16<3,  "u" >;
def SS : MC6809Reg16<4,  "s" >;
def PC : MC6809Reg16<5,  "pc">;
def AV : MC6809Reg16<7,  "v" >;
def AA : MC6809Reg8<8,   "a", [AA_LSB], [sub_lsb]>;
def AB : MC6809Reg8<9,   "b", [AB_LSB], [sub_lsb]>;
def DP : MC6809Reg8<11,  "dp">;
def A0 : MC6809Reg16<12, "0" >;
def AE : MC6809Reg8<14,  "e", [AE_LSB], [sub_lsb]>;
def AF : MC6809Reg8<15,  "f", [AF_LSB], [sub_lsb]>;

def AD : MC6809Reg16<0,  "d", [AA, AB], [sub_hi_byte, sub_lo_byte]>;
def AW : MC6809Reg16<6,  "w", [AE, AF], [sub_hi_byte, sub_lo_byte]>;

def AQ : MC6809Reg32<16, "q", [AD, AW], [sub_hi_word, sub_lo_word]>;

def MD : MC6809Reg8<17,  "md">;

//===----------------------------------------------------------------------===//
//  Register classes
//===----------------------------------------------------------------------===//

class MC6809RegClass<list<ValueType> regTypes, int alignment, dag regList>
     : RegisterClass<"MC6809", regTypes, alignment, regList>;

class MC6809Reg1Class<dag RegList>: MC6809RegClass<[i1], 8, RegList> {
  // The register is 1-bit wide, but takes 8 bits to spill.
  let RegInfos = RegInfoByHwMode<[DefaultMode], [RegInfo<1, 8, 8>]>;
}
class MC6809Reg8Class<dag RegList>: MC6809RegClass<[i8], 8, RegList>;
class MC6809Reg16Class<dag RegList>: MC6809RegClass<[i16], 8, RegList>;
class MC6809Reg32Class<dag RegList>: MC6809RegClass<[i32], 8, RegList>;

class MC6809RegAnyClass<dag RegList>: MC6809RegClass<[i32, i16, i8], 8, RegList>;

// "General purpose" register classes.
def BIT1 : MC6809Reg1Class<(add AA_LSB, AB_LSB, AE_LSB, AF_LSB)>;
def ACC8 : MC6809Reg8Class<(add AA, AB, AE, AF)>;
def BIT8 : MC6809Reg8Class<(add AA, AB, CC)>;
def ACC16 : MC6809Reg16Class<(add AD, AW)>;
def INDEX16 : MC6809Reg16Class<(add IX, IY, SU, SS)>;
def REG16 : MC6809Reg16Class<(add AD, AW, IX, IY, SU, SS)>;
def ACC32 : MC6809Reg32Class<(add AQ)>;

def PTR16 : MC6809Reg16Class<(add IX, IY, SU, SS)>;

def ANY1 : MC6809Reg1Class<(add BIT1, N, Z, V, C)>;
//def ACC : MC6809RegAnyClass<(add AA, AB, AD, AE, AF, AW, AQ)>;

let isAllocatable = false in {
  def CCFlag : MC6809Reg1Class<(add E, F, H, I, N, Z, V, C)>;
  def CCond : MC6809Reg8Class<(add CC)>;
  def MDCond : MC6809Reg8Class<(add MD)>;
  def DtPage : MC6809Reg8Class<(add DP)>;
  def STACK16 : MC6809Reg16Class<(add SU, SS)>;
}

// Single register classes.
let isPressureFineGrained = true in {
  def AAc : MC6809Reg8Class<(add AA)>;
  def ABc : MC6809Reg8Class<(add AB)>;
  def ADc : MC6809Reg16Class<(add AD)>;
  def AEc : MC6809Reg8Class<(add AE)>;
  def AFc : MC6809Reg8Class<(add AF)>;
  def AWc : MC6809Reg16Class<(add AW)>;
  def AQc : MC6809Reg16Class<(add AQ)>;
  def IXc : MC6809Reg16Class<(add IX)>;
  def IYc : MC6809Reg16Class<(add IY)>;
  def SUc : MC6809Reg16Class<(add SU)>;
  def SSc : MC6809Reg16Class<(add SS)>;
  def PCc : MC6809Reg16Class<(add PC)>;
  def CCc : MC6809Reg8Class<(add CC)>;
  def NZVCc : MC6809Reg8Class<(add NZVC)>;
  def Nc : MC6809Reg1Class<(add N)>;
  def Zc : MC6809Reg1Class<(add Z)>;
  def Vc : MC6809Reg1Class<(add V)>;
  def Cc : MC6809Reg1Class<(add C)>;
}

// MC6809 assembly mnemonics may only have the following registers as parameters
// following the comma in a mnemonic, e.g. lda hello, x
//def MC6809AsmParamRegClass : MC6809RegClass<[i16, i8], 8, (add AA, AB, AD, AE, AF, AW, IX, IY, SU, SS, PC)>;

// While not directly used, unless this is present, the TableGen-erated intersection
// register class between BIT1 and CC flags will not be allocatable.
//def AllocatableFlags : MC6809Reg1Class<(add N, Z, V, C)>;

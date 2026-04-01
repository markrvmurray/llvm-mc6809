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
def sub_negative : MC6809SubRegIndex<2, 2>;
def sub_zero : MC6809SubRegIndex<1, 3>;
def sub_overflow : MC6809SubRegIndex<1, 1>;
def sub_carry : MC6809SubRegIndex<1, 0>;

// 2-bit NZ subregister
def sub_nz_negative : MC6809SubRegIndex<1, 1>;
def sub_nz_zero : MC6809SubRegIndex<1, 0>;

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

// An 8-bit value register (GPR or imaginary). Contains a 1-bit LSB subregister
// to allow them to store boolean values.
multiclass MC6809Reg8<bits<16> num, string name> {
  def LSB : MC6809Reg1<!add(num, 128), name#"LSB">;
  def NAME : MC6809Reg8<num, name, [!cast<Register>(NAME#LSB)], [sub_lsb]>;
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

//===----------------------------------------------------------------------===//
//  Imaginary registers
//===----------------------------------------------------------------------===//

// This section describes the dimension of the imaginary registers for the MC6809
// targets. Imaginary registers are locations in the direct page that llvm can
// treat as physical registers for the purpose of register allocation.

// Imaginary registers are reserved in pairs, which allows them to be used as
// pointers with the 6809's various addressing modes. The number of imaginary
// register pointer pairs is configurable at code generation time via the
// --num-imag-ptrs flag. See MC6809RegisterInfo.cpp for details.

// The two bytes of any given pointer must be contiguous, but otherwise, there
// are no constraints on the relative layout of the pointers. For example, RS0
// could be allocated to 0x34 and 0x35, while RS1 is allocated to 0x12 and 0x13.
// The code generator emits generic symbol references for imaginary registers,
// and the linker script for a given memory layout later assigns these to their
// real locations.
//
// 32-bit registers are also created, similarly to the 16-bit ones described
// above. These will be used for 32-bit arithmetic and for floating-point.

// 8-bit imaginary registers.
multiclass MC6809ImagReg8<bits<16> num, string name> : MC6809Reg8<num, name>;

// 16-bit imaginary registers (consecutive pairs of 8-bit registers).
class MC6809ImagReg16<bits<16> num, string name, list<Register> subregs>
    : MC6809Reg<num, name> {
  let SubRegs = subregs;
  let SubRegIndices = [sub_lo_byte, sub_hi_byte];
  let CoveredBySubRegs = 1;
}

// 32-bit imaginary registers (consecutive pairs of 16-bit registers).
class MC6809ImagReg32<bits<16> num, string name, list<Register> subregs>
    : MC6809Reg<num, name> {
  let SubRegs = subregs;
  let SubRegIndices = [sub_lo_word, sub_hi_word];
  let CoveredBySubRegs = 1;
}

defvar MaxImag8Regs = 256;

// The starting DWARF number for the imaginary registers.
defvar Imag8RegsOffset = 0x1000;

defvar MaxImag16Regs = !sra(MaxImag8Regs, 1);
defvar Imag16RegsOffset = !add(Imag8RegsOffset, !shl(MaxImag16Regs,1));

defvar MaxImag32Regs = !sra(MaxImag16Regs, 1);
defvar Imag32RegsOffset = !add(Imag16RegsOffset, !shl(MaxImag16Regs,1));

// Now we enumerate the imaginary registers.
// Imaginary 8-bit registers, starting with the prefix rc
foreach I = 0...!add(MaxImag8Regs, -1) in {
  // There exist MaxImag8Regs rcXX registers...
  defm RC#I: MC6809ImagReg8<!add(!shl(I,1), Imag8RegsOffset), "rc"#!cast<string>(I)>;
}

// Imaginary 16-bit registers, starting with the prefix rs
foreach I = 0...!add(MaxImag16Regs, -1) in {
  // There exist MaxImag16Regs rsXX registers...
  def RS#I: MC6809ImagReg16<!add(I, Imag16RegsOffset), "rs"#!cast<string>(I),
      [!cast<Register>("RC"#!shl(I, 1)),
       !cast<Register>("RC"#!add(!shl(I,1),1))]>;
}

// Imaginary 32-bit registers, starting with the prefix rw
foreach I = 0...!add(MaxImag32Regs, -1) in {
  // There exist MaxImag32Regs rwXX registers...
  def RW#I: MC6809ImagReg32<!add(I, Imag32RegsOffset), "rw"#!cast<string>(I),
      [!cast<Register>("RS"#!shl(I, 1)),
       !cast<Register>("RS"#!add(!shl(I,1),1))]>;
}

//===----------------------------------------------------------------------===//
//  Define the actual real registers.
//===----------------------------------------------------------------------===//

def E : MC6809Reg1<48, "E">;
def F : MC6809Reg1<49, "F">;
def H : MC6809Reg1<50, "H">;
def I : MC6809Reg1<51, "I">;
def N : MC6809Reg1<52, "N">;
def Z : MC6809Reg1<53, "Z">;
def V : MC6809Reg1<54, "V">;
def C : MC6809Reg1<55, "C">;

def NZ : MC6809Reg<65, "NZ"> {
  let SubRegs = [N, Z];
  let SubRegIndices = [sub_nz_negative, sub_nz_zero];
  let CoveredBySubRegs = true;
}

def NZVC : MC6809Reg<66, "NZVC"> {
  let SubRegs = [N, Z, V, C];
  let SubRegIndices = [sub_negative, sub_zero, sub_overflow, sub_carry];
  let CoveredBySubRegs = true;
}

def CC : MC6809Reg<10, "cc"> {
  let SubRegs = [E, F, H, I, N, Z, V, C];
  let SubRegIndices = [sub_entire, sub_firq, sub_half, sub_irq, sub_negative, sub_zero, sub_overflow, sub_carry];
  let CoveredBySubRegs = true;
}


def IX : MC6809Reg16<1,  "x" >;
def IY : MC6809Reg16<2,  "y" >;
def SU : MC6809Reg16<3,  "u" >;
def SS : MC6809Reg16<4,  "s" >;

def PC : MC6809Reg16<5,  "pc">;

def AV : MC6809Reg16<7,  "v" >;

defm AA : MC6809Reg8<8,   "a">;
defm AB : MC6809Reg8<9,   "b">;

def AD : MC6809Reg16<0,  "d", [AB, AA], [sub_lo_byte, sub_hi_byte]>;

def DP : MC6809Reg8<11,  "dp">;

def A0 : MC6809Reg16<12, "0" >;

defm AE : MC6809Reg8<14,  "e">;
defm AF : MC6809Reg8<15,  "f">;

def AW : MC6809Reg16<6,  "w", [AF, AE], [sub_lo_byte, sub_hi_byte]>;

def AQ : MC6809Reg32<16, "q", [AW, AD], [sub_lo_word, sub_hi_word]>;

def MD : MC6809Reg8<17,  "md">;

//===----------------------------------------------------------------------===//
//  Stack-backed spill pseudo-registers
//===----------------------------------------------------------------------===//
// These pseudo-registers are backed by fixed stack frame locations.
// They give the register allocator extra "registers" for 1-register classes
// (ADc, AAc, ABc) that would otherwise cause allocation failures.
// Values assigned to spill registers are accessed via S-indexed loads/stores.
// DWARF numbers in 0x2000+ range to avoid collision with imaginary regs.

defm SPILL_A0 : MC6809Reg8<0x2000, "spill_a0">;
defm SPILL_A1 : MC6809Reg8<0x2002, "spill_a1">;
defm SPILL_A2 : MC6809Reg8<0x2004, "spill_a2">;
defm SPILL_A3 : MC6809Reg8<0x2006, "spill_a3">;
defm SPILL_A4 : MC6809Reg8<0x2008, "spill_a4">;
defm SPILL_A5 : MC6809Reg8<0x200A, "spill_a5">;
defm SPILL_A6 : MC6809Reg8<0x200C, "spill_a6">;
defm SPILL_A7 : MC6809Reg8<0x200E, "spill_a7">;

defm SPILL_B0 : MC6809Reg8<0x2001, "spill_b0">;
defm SPILL_B1 : MC6809Reg8<0x2003, "spill_b1">;
defm SPILL_B2 : MC6809Reg8<0x2005, "spill_b2">;
defm SPILL_B3 : MC6809Reg8<0x2007, "spill_b3">;
defm SPILL_B4 : MC6809Reg8<0x2009, "spill_b4">;
defm SPILL_B5 : MC6809Reg8<0x200B, "spill_b5">;
defm SPILL_B6 : MC6809Reg8<0x200D, "spill_b6">;
defm SPILL_B7 : MC6809Reg8<0x200F, "spill_b7">;

def SPILL_D0 : MC6809Reg16<0x2000, "spill_d0", [SPILL_B0, SPILL_A0], [sub_lo_byte, sub_hi_byte]>;
def SPILL_D1 : MC6809Reg16<0x2002, "spill_d1", [SPILL_B1, SPILL_A1], [sub_lo_byte, sub_hi_byte]>;
def SPILL_D2 : MC6809Reg16<0x2004, "spill_d2", [SPILL_B2, SPILL_A2], [sub_lo_byte, sub_hi_byte]>;
def SPILL_D3 : MC6809Reg16<0x2006, "spill_d3", [SPILL_B3, SPILL_A3], [sub_lo_byte, sub_hi_byte]>;
def SPILL_D4 : MC6809Reg16<0x2008, "spill_d4", [SPILL_B4, SPILL_A4], [sub_lo_byte, sub_hi_byte]>;
def SPILL_D5 : MC6809Reg16<0x200A, "spill_d5", [SPILL_B5, SPILL_A5], [sub_lo_byte, sub_hi_byte]>;
def SPILL_D6 : MC6809Reg16<0x200C, "spill_d6", [SPILL_B6, SPILL_A6], [sub_lo_byte, sub_hi_byte]>;
def SPILL_D7 : MC6809Reg16<0x200E, "spill_d7", [SPILL_B7, SPILL_A7], [sub_lo_byte, sub_hi_byte]>;

//===----------------------------------------------------------------------===//
//  Register classes
//===----------------------------------------------------------------------===//

class MC6809RegClass<list<ValueType> regTypes, int alignment, dag regList>
    : RegisterClass<"MC6809", regTypes, alignment, regList> {
        let Namespace = "MC6809";
}

class MC6809Reg1Class<dag RegList>: MC6809RegClass<[i1], 1, RegList> {
  // The register is 1-bit wide, but takes 8 bits to spill.
  let RegInfos = RegInfoByHwMode<[DefaultMode], [RegInfo<1, 8, 8>]>;
}
class MC6809Reg2Class<dag RegList>  : MC6809RegClass<[i2], 8, RegList>;
class MC6809Reg4Class<dag RegList>  : MC6809RegClass<[i4], 8, RegList>;
class MC6809Reg8Class<dag RegList>  : MC6809RegClass<[i8], 8, RegList>;
class MC6809Reg16Class<dag RegList> : MC6809RegClass<[i16], 8, RegList>;
class MC6809Reg32Class<dag RegList> : MC6809RegClass<[i32], 8, RegList>;

// Register grouping classes.
def CARRY : MC6809Reg1Class<(add C)>;
def BIT1 : MC6809Reg1Class<(add AALSB, ABLSB, AELSB, AFLSB)> {
    let AltOrders = [(add AALSB, ABLSB)];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
def ACC8 : MC6809Reg8Class<(add AB, AA, AF, AE, SPILL_B0, SPILL_B1, SPILL_B2, SPILL_B3, SPILL_B4, SPILL_B5, SPILL_B6, SPILL_B7, SPILL_A0, SPILL_A1, SPILL_A2, SPILL_A3, SPILL_A4, SPILL_A5, SPILL_A6, SPILL_A7)> {
    let AltOrders = [(add AB, AA, SPILL_B0, SPILL_B1, SPILL_B2, SPILL_B3, SPILL_B4, SPILL_B5, SPILL_B6, SPILL_B7, SPILL_A0, SPILL_A1, SPILL_A2, SPILL_A3, SPILL_A4, SPILL_A5, SPILL_A6, SPILL_A7)];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
def ACC8_AB : MC6809Reg8Class<(add AB, AA)>;
def BIT8 : MC6809Reg8Class<(add AA, AB, CC)>; // Valid registers for BAND, BEOR, BOR, BIAND, BIEOR, BIOR, LDBT, STBT
def ACC16 : MC6809Reg16Class<(add AD, AW, SPILL_D0, SPILL_D1, SPILL_D2, SPILL_D3, SPILL_D4, SPILL_D5, SPILL_D6, SPILL_D7)> {
    let AltOrders = [(add AD, SPILL_D0, SPILL_D1, SPILL_D2, SPILL_D3, SPILL_D4, SPILL_D5, SPILL_D6, SPILL_D7)];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
def ACC16_D : MC6809Reg8Class<(add AD)>;
def ACC32 : MC6809Reg32Class<(add AQ)> {
    let AltOrders = [(add)];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
def REGLIST8 : MC6809Reg8Class<(add CC, AA, AB, DP)>; // Registers which can be pushed/popped
def REGLIST16 : MC6809Reg16Class<(add AD, AW, IX, IY, SU, SS, PC)>; // Registers which can be pushed/popped
def REGPAIR8 : MC6809Reg8Class<(add AA, AB, CC, DP, A0, AE, AF)>; // Registers for EXG, TFR
def REGPAIR16 : MC6809Reg16Class<(add AD, IX, IY, SU, SS, PC, AW, AV)>; // Registers for EXG, TFR
def REGTFM : MC6809Reg16Class<(add AD, IX, IY, SU, SS)>; // Registers for TFM

// Pointer and stack registers
def INDEX16 : MC6809Reg16Class<(add IX, IY, SU, SS)>;

// Stack registers for internal push/pull operations.
// SU is always reserved (frame pointer), so only SS will be allocated.
// Stack registers for Push/Pull operations.
// The allocator may assign SU but the expansion always forces SS.
def STACK16 : MC6809Reg16Class<(add SU, SS)>;

def CCond : MC6809Reg8Class<(add CC)>;

def Imag8 : MC6809Reg8Class<(sequence "RC%u", 0, !add(MaxImag8Regs, -1))>;
def Imag16 : MC6809Reg16Class<(sequence "RS%u", 0, !add(MaxImag16Regs, -1))>;
def Imag32 : MC6809Reg32Class<(sequence "RW%u", 0, !add(MaxImag32Regs, -1))>;

def AllCCFlag : MC6809Reg1Class<(add E, F, H, I, N, Z, V, C)>;
def CCFlag : MC6809Reg1Class<(add N, Z, V, C)>;
def ArithFlag : MC6809Reg2Class<(add NZ)>;
def AllArithFlag : MC6809Reg2Class<(add NZVC)>;

let isAllocatable = false in {
  def MDCond : MC6809Reg8Class<(add MD)>;
  def DtPage : MC6809Reg8Class<(add DP)>;
}

// Single register classes.
    def AAc : MC6809Reg8Class<(add AA, SPILL_A0, SPILL_A1, SPILL_A2, SPILL_A3, SPILL_A4, SPILL_A5, SPILL_A6, SPILL_A7)>;
    def AALSBc : MC6809Reg1Class<(add AALSB, SPILL_A0LSB, SPILL_A1LSB, SPILL_A2LSB, SPILL_A3LSB, SPILL_A4LSB, SPILL_A5LSB, SPILL_A6LSB, SPILL_A7LSB)>;
    def ABc : MC6809Reg8Class<(add AB, SPILL_B0, SPILL_B1, SPILL_B2, SPILL_B3, SPILL_B4, SPILL_B5, SPILL_B6, SPILL_B7)>;
    def ABLSBc : MC6809Reg1Class<(add ABLSB, SPILL_B0LSB, SPILL_B1LSB, SPILL_B2LSB, SPILL_B3LSB, SPILL_B4LSB, SPILL_B5LSB, SPILL_B6LSB, SPILL_B7LSB)>;
    def ADc : MC6809Reg16Class<(add AD, SPILL_D0, SPILL_D1, SPILL_D2, SPILL_D3, SPILL_D4, SPILL_D5, SPILL_D6, SPILL_D7)>;
    def AEc : MC6809Reg8Class<(add AE)>;
    def AELSBc : MC6809Reg1Class<(add AELSB)>;
    def AFc : MC6809Reg8Class<(add AF)>;
    def AFLSBc : MC6809Reg1Class<(add AFLSB)>;
    def AWc : MC6809Reg16Class<(add AW)>;
    def AQc : MC6809Reg32Class<(add AQ)>;
    def IXc : MC6809Reg16Class<(add IX)>;
    def IYc : MC6809Reg16Class<(add IY)>;
    def SUc : MC6809Reg16Class<(add SU)>;
    def SSc : MC6809Reg16Class<(add SS)>;
    def PCc : MC6809Reg16Class<(add PC)>;
    def CCc : MC6809Reg8Class<(add CC)>;
    def NZVCc : MC6809Reg8Class<(add NZVC)>;
    def NZc : MC6809Reg8Class<(add NZ)>;
    def Nc : MC6809Reg1Class<(add N)>;
    def Zc : MC6809Reg1Class<(add Z)>;
    def Vc : MC6809Reg1Class<(add V)>;
    def Cc : MC6809Reg1Class<(add C)>;

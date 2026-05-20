//===- MC6809RegisterBanks.td - MC6809 Register Bank Description -*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the GlobalISel Register Banks for MC6809.
//
//===----------------------------------------------------------------------===//

/// General Purpose Registers: AA, AD, etc
def ACCUMRegBank : RegisterBank<"ACCUM", [ACC64, ACC32, ACC16, ACC8]>;

/// Index Registers: IX, IY, SU, SS
def INDEXRegBank : RegisterBank<"INDEX", [INDEX16]>;

/// Imaginary (DP) registers: Imag8, Imag16, Imag32;
def IMAGRegBank : RegisterBank<"IMAG", [Imag32]>;

/// Conditional register: CC bits
def FLAGSRegBank : RegisterBank<"FLAGS", [CCFlag, CCond]>;

/// PHANTOM_CARRY — scheduling-phantom carry-out vregs of the SetCarry /
/// SetCarryUse / SetOverflow / SetOverflowUse pseudos.  Lives in its own
/// bank so the cost models for ACCUM (real bytes/words) and PHANTOM
/// (synthetic 1-bit slots) stay disjoint and regalloc never spills a
/// phantom into a real byte register.  See the comment block in
/// MC6809RegisterInfo.td at the PHANTOM_CARRY_0..7 physreg defs.
def PHANTOMRegBank : RegisterBank<"PHANTOM", [PHANTOM_CARRY]>;

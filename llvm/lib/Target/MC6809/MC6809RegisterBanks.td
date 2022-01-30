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
def ACCRegBank : RegisterBank<"GPR", [ACC32]>;

/// Index Registers: IX, IY, SU, SS
def INDEXRegBank : RegisterBank<"INDEX", [INDEX16]>;

/// Conditional register: CC
def CCRegBank : RegisterBank<"FLAGS", [CCond]>;

/// Direct page: DP
def DPRegBank : RegisterBank<"DP", [DtPage]>;

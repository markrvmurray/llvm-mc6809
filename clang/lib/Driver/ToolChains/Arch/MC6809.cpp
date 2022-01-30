//===--- MC6809.cpp - MC6809 Helpers for Tools ------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "clang/Driver/Options.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang::driver::tools;
using namespace llvm::opt;
using namespace llvm;

/// getMC6809TargetCPU - Get the (LLVM) name of the MC6809 cpu we are targeting.
std::string mc6809::getMC6809TargetCPU(const ArgList &Args) {
  if (Arg *A = Args.getLastArg(clang::driver::options::OPT_mcpu_EQ)) {
    StringRef CPUName = A->getValue();

    return llvm::StringSwitch<const char *>(CPUName)
        .Cases("mc6809", "6809", "mc6809")
        .Cases("hd6309", "6309", "hd6309")
        .Default("");
  }

  return "";
}

//===--- MC6809.cpp - MC6809 Helpers for Tools ------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "clang/Driver/Driver.h"
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

void mc6809::getMC6809TargetFeatures(const Driver &D, const ArgList &Args,
                               std::vector<StringRef> &Features) {
  if (Args.hasArg(clang::driver::options::OPT_mcpu_EQ) &&
      getMC6809TargetCPU(Args).empty()) {
    D.Diag(diag::err_drv_clang_unsupported)
        << Args.getLastArg(clang::driver::options::OPT_mcpu_EQ)
               ->getAsString(Args);
  }

  if (Arg *A = Args.getLastArg(options::OPT_fstatic_stack,
                               options::OPT_fno_static_stack)) {
    if (A->getOption().matches(options::OPT_fstatic_stack))
      Features.push_back("+static-stack");
    else
      Features.push_back("-static-stack");
  }
}

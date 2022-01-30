//===--- MC6809.h - MC6809-specific Tool Helpers ----------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_MC6809_H
#define LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_MC6809_H

#include "llvm/Option/ArgList.h"
#include <string>

namespace clang {
namespace driver {
namespace tools {
namespace mc6809 {

std::string getMC6809TargetCPU(const llvm::opt::ArgList &Args);

} // end namespace mc6809
} // end namespace tools
} // end namespace driver
} // end namespace clang

#endif // LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_ARCH_MC6809_H

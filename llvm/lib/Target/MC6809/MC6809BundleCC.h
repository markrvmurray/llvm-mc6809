#ifndef LLVM_LIB_TARGET_MC6809_MC6809BUNDLECC_H
#define LLVM_LIB_TARGET_MC6809_MC6809BUNDLECC_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809BundleCCPass();

} // namespace llvm

#endif

#ifndef LLVM_LIB_TARGET_MC6809_MC6809MATERIALIZESPILLS_H
#define LLVM_LIB_TARGET_MC6809_MC6809MATERIALIZESPILLS_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809MaterializeSpillsPass();

} // namespace llvm

#endif

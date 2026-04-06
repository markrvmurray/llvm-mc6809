#ifndef LLVM_LIB_TARGET_MC6809_MC6809SPILLDSAVERESTORE_H
#define LLVM_LIB_TARGET_MC6809_MC6809SPILLDSAVERESTORE_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809SpillDSaveRestorePass();

} // namespace llvm

#endif

//===-- MC6809PreferPage1Index.cpp - Prefer X over Y where free ----------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// On the MC6809, IX and IU are page-1 index registers (no prefix) while IY and
// IS are page-2 ($10 prefix) -- every LDY/STY/LEAY/CMPY costs one more byte and
// one more cycle than the IX equivalent. The register allocator already lists
// IX ahead of IY, but IX is frequently busy (it carries arg0 / the return value
// / the sret pointer, and spill staging deliberately uses IY), so general index
// values routinely land in the prefixed IY.
//
// This post-RA peephole reclaims the cheap register locally: when a value's
// whole IY live-range sits inside one basic block, is call-free, and IX is dead
// across the entire range, the range is renamed IY -> IX (page-2 opcodes -> the
// page-1 siblings). This is the "reload-into-X" idea: re-home a Y live-range to
// X wherever X happens to be free, without any frame/allocation change.
//
// Correctness rests on three checks per candidate range [def, lastUse]:
//   * IY is dead *before* def (a fresh value -- avoids retroactively renaming a
//     prior IY value still live elsewhere),
//   * IY is dead *after* lastUse and never escapes the block (so every read of
//     the value is inside the range and gets renamed with it),
//   * IX is dead at every point across the range (so putting the value in IX
//     clobbers nothing; this also means nothing in the range reads/writes IX,
//     so ranges that use IX as a base -- e.g. `ldy ,x` -- are excluded).
// Every IY-referencing instruction in the range must have a page-1 sibling
// (all 160 do); if one somehow doesn't, the range is skipped wholesale.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809InstrInfo.h"
#include "MC6809Subtarget.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-prefer-page1-index"

STATISTIC(NumRanges, "Number of IY live-ranges renamed to IX");
STATISTIC(NumInstrs, "Number of page-2 instructions demoted to page-1");

// Default ON. The rename is gated by a conservative computeRegisterLiveness
// cross-check (see processBlock) on top of the local IX/IY dataflow, so it only
// fires where IX is *provably* dead -- bailing on the call-argument /
// control-flow-save cases the dataflow alone under-reported.
static cl::opt<bool>
    EnablePreferPage1("mc6809-prefer-page1-index", cl::init(true), cl::Hidden,
                      cl::desc("Rename call-free IY ranges to IX where IX is "
                               "free (MC6809 page-1 preference)"));

namespace {

class MC6809PreferPage1Index : public MachineFunctionPass {
public:
  static char ID;
  MC6809PreferPage1Index() : MachineFunctionPass(ID) {
    initializeMC6809PreferPage1IndexPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().set(
        MachineFunctionProperties::Property::NoVRegs);
  }

private:
  const TargetRegisterInfo *TRI = nullptr;
  const MC6809InstrInfo *TII = nullptr;
  // Page-2 (IY/IS-based) opcode -> page-1 (IX-based) sibling.
  DenseMap<unsigned, unsigned> YtoX;
  // Per-block {IX, IY} live-out, from computeLiveOuts.
  DenseMap<const MachineBasicBlock *, std::pair<bool, bool>> LiveOutXY;

  void buildOpcodeMap();
  void computeLiveOuts(MachineFunction &MF);
  bool processBlock(MachineBasicBlock &MBB);
};

} // namespace

char MC6809PreferPage1Index::ID = 0;

INITIALIZE_PASS(MC6809PreferPage1Index, DEBUG_TYPE,
                "MC6809 prefer page-1 index register", false, false)

MachineFunctionPass *llvm::createMC6809PreferPage1IndexPass() {
  return new MC6809PreferPage1Index();
}

// Build IY-form -> IX-form by name: the mnemonics differ only in the index
// letter right after the LD/ST/LEA/CMP prefix (LDYi_o8 -> LDXi_o8, etc.).
void MC6809PreferPage1Index::buildOpcodeMap() {
  StringMap<unsigned> NameToOpc;
  unsigned N = TII->getNumOpcodes();
  for (unsigned Op = 0; Op < N; ++Op)
    NameToOpc[TII->getName(Op)] = Op;

  static const StringRef Prefixes[] = {"LDY", "STY", "LEAY", "CMPY"};
  for (unsigned Op = 0; Op < N; ++Op) {
    StringRef Name = TII->getName(Op);
    for (StringRef P : Prefixes) {
      if (!Name.starts_with(P))
        continue;
      // Replace the trailing 'Y' of the prefix with 'X'.
      std::string XName = (P.drop_back() + "X" + Name.drop_front(P.size())).str();
      auto It = NameToOpc.find(XName);
      if (It != NameToOpc.end())
        YtoX[Op] = It->second;
      break;
    }
  }
}

// Backward live-variable dataflow for just IX and IY, computed straight from
// the instruction stream (independent of the stored block live-in lists).
// use(B,R)  = R is read before any write in B (upward-exposed use)
// def(B,R)  = R is written before any read in B (locally killed)
// LiveIn(B) = use(B) | (LiveOut(B) & ~def(B));  LiveOut(B) = | LiveIn(succ)
// The boolean lattice makes the fixpoint monotonic, so it always converges.
void MC6809PreferPage1Index::computeLiveOuts(MachineFunction &MF) {
  LiveOutXY.clear();
  const Register XY[2] = {MC6809::IX, MC6809::IY};
  DenseMap<const MachineBasicBlock *, std::pair<bool, bool>> Use, Def;
  for (MachineBasicBlock &MBB : MF) {
    bool U[2] = {false, false}, D[2] = {false, false};
    for (int R = 0; R < 2; ++R)
      for (MachineInstr &MI : MBB) {
        if (MI.readsRegister(XY[R], TRI)) {
          U[R] = true;
          break;
        }
        if (MI.modifiesRegister(XY[R], TRI)) {
          D[R] = true;
          break;
        }
      }
    Use[&MBB] = {U[0], U[1]};
    Def[&MBB] = {D[0], D[1]};
    LiveOutXY[&MBB] = {false, false};
  }

  bool Changed = true;
  while (Changed) {
    Changed = false;
    for (MachineBasicBlock &MBB : MF) {
      bool LO[2] = {false, false};
      for (MachineBasicBlock *S : MBB.successors()) {
        auto U = Use[S], D = Def[S], SLO = LiveOutXY[S];
        LO[0] |= U.first || (SLO.first && !D.first);
        LO[1] |= U.second || (SLO.second && !D.second);
      }
      auto &Cur = LiveOutXY[&MBB];
      if (LO[0] != Cur.first || LO[1] != Cur.second) {
        Cur = {LO[0], LO[1]};
        Changed = true;
      }
    }
  }
}

bool MC6809PreferPage1Index::runOnMachineFunction(MachineFunction &MF) {
  if (!EnablePreferPage1 || skipFunction(MF.getFunction()))
    return false;
  TRI = MF.getSubtarget().getRegisterInfo();
  TII = static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());
  if (YtoX.empty())
    buildOpcodeMap();

  // Compute IX/IY live-out per block ourselves. We can't trust the stored
  // block live-in lists (earlier post-RA passes -- pseudo expansion,
  // MC6809PostRASpillOpt -- add/delete instructions without refreshing them, so
  // LivePhysRegs::addLiveOuts under-reports liveness and we'd rename a
  // loop-carried IY that is really live-out). LLVM's fullyRecomputeLiveIns
  // doesn't converge on MC6809's heavily aliased register file. A direct
  // two-register backward dataflow over the instructions is exact and always
  // converges.
  computeLiveOuts(MF);

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF)
    Changed |= processBlock(MBB);
  return Changed;
}

bool MC6809PreferPage1Index::processBlock(MachineBasicBlock &MBB) {
  // Skip blocks with no terminator-free body quickly.
  if (MBB.empty())
    return false;

  // Index the block and record, after each instruction, whether IX / IY are
  // live -- computed by a precise backward LivePhysRegs walk from the block's
  // live-outs. liveAfterIX[k] / liveAfterIY[k] hold the state immediately AFTER
  // instruction k.
  SmallVector<MachineInstr *, 64> Instrs;
  for (MachineInstr &MI : MBB)
    Instrs.push_back(&MI);
  unsigned NumMI = Instrs.size();
  SmallVector<bool, 64> LiveAfterIX(NumMI), LiveAfterIY(NumMI);

  // Seed from this block's IX/IY live-out (computed by computeLiveOuts) and walk
  // backward, applying each instruction's read/write transfer: a read makes the
  // register live before the instruction, a write-without-read makes it dead.
  auto LO = LiveOutXY.lookup(&MBB);
  bool LiveIX = LO.first, LiveIY = LO.second;
  for (int k = (int)NumMI - 1; k >= 0; --k) {
    LiveAfterIX[k] = LiveIX;
    LiveAfterIY[k] = LiveIY;
    MachineInstr *MI = Instrs[k];
    if (MI->readsRegister(MC6809::IX, TRI))
      LiveIX = true;
    else if (MI->modifiesRegister(MC6809::IX, TRI))
      LiveIX = false;
    if (MI->readsRegister(MC6809::IY, TRI))
      LiveIY = true;
    else if (MI->modifiesRegister(MC6809::IY, TRI))
      LiveIY = false;
  }

  auto RefsIY = [&](const MachineInstr *MI) {
    return MI->readsRegister(MC6809::IY, TRI) ||
           MI->modifiesRegister(MC6809::IY, TRI);
  };

  bool Changed = false;
  for (unsigned k = 0; k < NumMI;) {
    MachineInstr *MI = Instrs[k];
    // A fresh IY value begins where an instruction defines IY and IY was dead
    // immediately before it.
    bool IYDeadBefore = (k == 0) || !LiveAfterIY[k - 1];
    if (!(MI->definesRegister(MC6809::IY, TRI) && IYDeadBefore)) {
      ++k;
      continue;
    }

    // Extend the range while IY stays live (LiveAfterIY[j] true means the value
    // is still live after instruction j). The loop stops at the instruction
    // that kills IY -- its last use, after which IY is dead -- or at the block
    // end if the value is live-out.
    unsigned End = k;
    while (End < NumMI && LiveAfterIY[End])
      ++End;
    if (End == NumMI) {
      // IY is live after the last instruction: the value escapes to a
      // successor, so not every read is in this block -- cannot rename.
      ++k;
      continue;
    }
    // End is the last-use instruction (IY dead after it); include it.
    unsigned Last = End;

    // Validate the range.
    bool OK = true;
    for (unsigned j = k; j <= Last && OK; ++j) {
      MachineInstr *R = Instrs[j];
      if (R->isCall())
        OK = false; // calls clobber X; renamed value wouldn't survive.
      // IX must be dead across the whole span, including just before the def.
      if (LiveAfterIX[j])
        OK = false;
      // Every IY-referencing instruction must have a page-1 sibling.
      if (RefsIY(R) && !YtoX.count(R->getOpcode()))
        OK = false;
    }
    // IX dead immediately before the def too (don't clobber a live IX).
    if (k > 0 && LiveAfterIX[k - 1])
      OK = false;

    if (!OK) {
      ++k;
      continue;
    }

    // Conservative liveness cross-check. The two-register dataflow above can
    // under-report IX liveness for a value that is consumed by a call argument
    // or a control-flow save the local scan does not model -- renaming onto it
    // would clobber that value (observed as a frame/return corruption).
    // computeRegisterLiveness accounts for every use (implicit operands,
    // regmask clobbers) and answers LQR_Unknown when it cannot be sure; require
    // a definite LQR_Dead, and treat anything else as a reason to bail:
    //   * IX must be provably dead entering the range (the rename defines IX
    //     there, so a live incoming IX value must not exist), and
    //   * IY must be provably dead just after the range (the value really does
    //     not escape to a later use this block's scan missed).
    MachineBasicBlock::const_iterator AtDef(Instrs[k]);
    if (MBB.computeRegisterLiveness(TRI, MC6809::IX, AtDef,
                                    /*Neighborhood=*/32) !=
        MachineBasicBlock::LQR_Dead) {
      ++k;
      continue;
    }
    MachineBasicBlock::const_iterator AfterLast =
        std::next(MachineBasicBlock::const_iterator(Instrs[Last]));
    if (MBB.computeRegisterLiveness(TRI, MC6809::IY, AfterLast,
                                    /*Neighborhood=*/32) !=
        MachineBasicBlock::LQR_Dead) {
      ++k;
      continue;
    }

    // Rename the range: IY-form opcodes -> IX-form, IY operands -> IX.
    for (unsigned j = k; j <= Last; ++j) {
      MachineInstr *R = Instrs[j];
      if (!RefsIY(R))
        continue;
      R->setDesc(TII->get(YtoX[R->getOpcode()]));
      for (MachineOperand &MO : R->operands())
        if (MO.isReg() && MO.getReg() == MC6809::IY)
          MO.setReg(MC6809::IX);
      ++NumInstrs;
    }
    LLVM_DEBUG(dbgs() << "  PreferPage1: renamed IY range [" << k << "," << Last
                      << "] to IX in " << MBB.getName() << "\n");
    ++NumRanges;
    Changed = true;
    k = End + 1;
  }
  return Changed;
}

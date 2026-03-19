//===-- MC6809AsmBackend.h - MC6809 Asm Backend  --------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// \file The MC6809 assembly backend implementation.
//
//===----------------------------------------------------------------------===//
//

#ifndef LLVM_MC6809_ASM_BACKEND_H
#define LLVM_MC6809_ASM_BACKEND_H

#include "MCTargetDesc/MC6809FixupKinds.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/TargetParser/Triple.h"

namespace llvm {

class MCAssembler;
class Target;

struct MCFixupKindInfo;

class MC6809ObjectTargetWriter : public MCELFObjectTargetWriter {
public:
  MC6809ObjectTargetWriter()
      : MCELFObjectTargetWriter(false, 0, ELF::EM_MC6809, false) {
    val = ELF::EM_MC6809;
  }

  ~MC6809ObjectTargetWriter() override { val = 0; }

  unsigned getRelocType(const MCFixup &Fixup, const MCValue &Target,
                        bool IsPCRel) const override {
    return 0;
  }

  Triple::ObjectFormatType getFormat() const override {
    return Triple::ObjectFormatType::ELF;
  }

  int val;
};

/// Utilities for manipulating generated MC6809 machine code.
class MC6809AsmBackend : public MCAsmBackend {
public:
  MC6809AsmBackend(Triple::OSType OSType)
      : llvm::MCAsmBackend(endianness::little), OSType(OSType) {}

  std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override;

  /// Simple predicate for targets where !Resolved implies requiring relaxation
  bool fixupNeedsRelaxation(const MCFixup &Fixup,
                            uint64_t Value) const override;
  /// Carefully determine whether the instruction in question requires
  /// relaxation.  This implementation considers the fixup as well as
  /// the section that the symbol points to.
  bool fixupNeedsRelaxationAdvanced(const MCFragment &, const MCFixup &Fixup,
                                    const MCValue &Target, uint64_t Value,
                                    bool Resolved) const override;
  MCFixupKindInfo getFixupKindInfo(MCFixupKind Kind) const override;

  bool shouldForceRelocation(const MCFixup &, const MCValue &);

  /// Apply the \p Value for given \p Fixup into the provided data fragment, at
  /// the offset specified by the fixup and following the fixup kind as
  /// appropriate.
  void applyFixup(const MCFragment &F, const MCFixup &Fixup,
                  const MCValue &Target, uint8_t *Data, uint64_t Value,
                  bool IsResolved) override;

  /// Check whether the given instruction may need relaxation.
  bool mayNeedRelaxation(unsigned Opcode, ArrayRef<MCOperand> Operands,
                         const MCSubtargetInfo &STI) const override;

  /// Relax the instruction in the given fragment to the next wider instruction.
  ///
  /// \param Inst The instruction to relax, which may be the same as the
  /// output.
  /// \param STI the subtarget information for the associated instruction.
  /// \param [out] Res On return, the relaxed instruction.
  void relaxInstruction(MCInst &Inst,
                        const MCSubtargetInfo &STI) const override;

  /// If the instruction can be relaxed, return the opcode of the instruction
  /// that this instruction can be relaxed to. If the instruction cannot be
  /// relaxed, return zero. When 65816 subtarget is active and the instruction
  /// is relaxed to Addr24, BankRelax is set to true.
  static unsigned relaxInstructionTo(unsigned Opcode,
                                     const MCSubtargetInfo &STI,
                                     bool &BankRelax);
  static unsigned relaxInstructionTo(unsigned Opcode,
                                     const MCSubtargetInfo &STI) {
    bool BankRelax = false;
    return relaxInstructionTo(Opcode, STI, BankRelax);
  }

  /// If the provided instruction contains an out-of-range immediate in a
  /// relaxable opcode, perform the relaxation now. MC6809AsmPrinter calls this at
  /// the end of lowering so it does not have to deal with the relaxation
  /// itself.
  static void relaxForImmediate(MCInst &Inst, const MCSubtargetInfo &STI);

  /// Write an (optimal) nop sequence of Count bytes to the given output. If the
  /// target cannot generate such a sequence, it should return an error.
  ///
  /// \return - True on success.
  bool writeNopData(raw_ostream &OS, uint64_t Count,
                    const MCSubtargetInfo *STI) const override;

private:
  Triple::OSType OSType;
  mutable unsigned RelaxedOpcode = 0;
  mutable const MCSubtargetInfo *RelaxedSTI = nullptr;
};

} // end namespace llvm

#endif // LLVM_MC6809_ASM_BACKEND_H

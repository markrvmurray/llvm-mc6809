//===-- ABIMC6809.cpp -----------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Minimal MC6809 ABI plugin for lldb (Motorola 6809 / Hitachi 6309).
//
// Register layout matches the DWARF numbering defined in
// llvm/lib/Target/MC6809/MC6809RegisterInfo.td:
//
//   D  =  0 (16-bit; composed of A:B)
//   X  =  1
//   Y  =  2
//   U  =  3  (user stack / frame pointer)
//   S  =  4  (hardware stack pointer)
//   PC =  5
//   A  =  8  (high byte of D)
//   B  =  9  (low  byte of D)
//   CC = 10  (condition codes)
//   DP = 11  (direct-page register)
//
// The standard MC6809 calling convention used by llvm-mc6809:
//   - S is the active hardware stack pointer.
//   - U is used as a frame pointer when a function needs one.
//   - JSR / BSR pushes a 2-byte big-endian return address onto S;
//     PC high at S+0, PC low at S+1.
//   - Prologue: PSHS U; TFR S,U; LEAS -N,S
//     Frame layout post-prologue:
//       [return addr hi] <- U+0
//       [return addr lo] <- U+1
//       [old U hi]       <- U+2
//       [old U lo]       <- U+3
//       [locals...]      <- U+4...
//     CFA at function entry = S + 2 (caller's PC just below S).
//
//===----------------------------------------------------------------------===//

#include "ABIMC6809.h"

#include "lldb/Core/Module.h"
#include "lldb/Core/PluginManager.h"
#include "lldb/Core/Value.h"
#include "lldb/Symbol/UnwindPlan.h"
#include "lldb/Target/Process.h"
#include "lldb/Target/RegisterContext.h"
#include "lldb/Target/StackFrame.h"
#include "lldb/Target/Target.h"
#include "lldb/Target/Thread.h"
#include "lldb/Utility/ConstString.h"
#include "lldb/Utility/DataExtractor.h"
#include "lldb/Utility/Log.h"
#include "lldb/Utility/RegisterValue.h"
#include "lldb/ValueObject/ValueObjectConstResult.h"
#include "lldb/ValueObject/ValueObjectMemory.h"
#include "lldb/ValueObject/ValueObjectRegister.h"

#include "llvm/IR/DerivedTypes.h"
#include "llvm/TargetParser/Triple.h"

using namespace lldb;
using namespace lldb_private;

LLDB_PLUGIN_DEFINE(ABIMC6809)

// DWARF register numbers (must match MC6809RegisterInfo.td).
enum dwarf_regnums {
  dwarf_d   =  0,  // D (16-bit, A:B)
  dwarf_x   =  1,  // X
  dwarf_y   =  2,  // Y
  dwarf_u   =  3,  // U (frame pointer)
  dwarf_s   =  4,  // S (stack pointer)
  dwarf_pc  =  5,  // PC
  dwarf_a   =  8,  // A (high byte of D)
  dwarf_b   =  9,  // B (low  byte of D)
  dwarf_cc  = 10,  // CC
  dwarf_dp  = 11,  // DP
};

static const RegisterInfo g_register_infos[] = {
    // name   alt    size  offset  encoding      format
    {"d",   "d",   2, 0, eEncodingUint, eFormatHex,
     {dwarf_d,  dwarf_d,  LLDB_INVALID_REGNUM,     LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"x",   "x",   2, 2, eEncodingUint, eFormatHex,
     {dwarf_x,  dwarf_x,  LLDB_INVALID_REGNUM,     LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"y",   "y",   2, 4, eEncodingUint, eFormatHex,
     {dwarf_y,  dwarf_y,  LLDB_INVALID_REGNUM,     LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"u",   "u",   2, 6, eEncodingUint, eFormatHex,
     {dwarf_u,  dwarf_u,  LLDB_REGNUM_GENERIC_FP,  LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"s",   "sp",  2, 8, eEncodingUint, eFormatHex,
     {dwarf_s,  dwarf_s,  LLDB_REGNUM_GENERIC_SP,  LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"pc",  "pc",  2, 10, eEncodingUint, eFormatHex,
     {dwarf_pc, dwarf_pc, LLDB_REGNUM_GENERIC_PC,  LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    // Padding to maintain byte_offset alignment with DWARF register numbers.
    // Slots 6 and 7 have no DWARF-defined registers.
    {"__pad6", "", 1, 12, eEncodingUint, eFormatHex,
     {LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"__pad7", "", 1, 13, eEncodingUint, eFormatHex,
     {LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"a",   "a",   1, 14, eEncodingUint, eFormatHex,
     {dwarf_a,  dwarf_a,  LLDB_REGNUM_GENERIC_ARG1, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"b",   "b",   1, 15, eEncodingUint, eFormatHex,
     {dwarf_b,  dwarf_b,  LLDB_INVALID_REGNUM,     LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"cc",  "cc",  1, 16, eEncodingUint, eFormatHex,
     {dwarf_cc, dwarf_cc, LLDB_REGNUM_GENERIC_FLAGS, LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
    {"dp",  "dp",  1, 17, eEncodingUint, eFormatHex,
     {dwarf_dp, dwarf_dp, LLDB_INVALID_REGNUM,     LLDB_INVALID_REGNUM, LLDB_INVALID_REGNUM},
     nullptr, nullptr, nullptr},
};

static const uint32_t k_num_register_infos =
    sizeof(g_register_infos) / sizeof(RegisterInfo);

const RegisterInfo *ABIMC6809::GetRegisterInfoArray(uint32_t &count) {
  count = k_num_register_infos;
  return g_register_infos;
}

ABISP ABIMC6809::CreateInstance(lldb::ProcessSP process_sp,
                                 const ArchSpec &arch) {
  if (arch.GetTriple().getArch() == llvm::Triple::mc6809)
    return ABISP(new ABIMC6809(std::move(process_sp), MakeMCRegisterInfo(arch)));
  return ABISP();
}

bool ABIMC6809::PrepareTrivialCall(Thread &, addr_t, addr_t, addr_t,
                                    llvm::ArrayRef<addr_t>) const {
  return false;
}

bool ABIMC6809::GetArgumentValues(Thread &, ValueList &) const {
  return false;
}

Status ABIMC6809::SetReturnValueObject(StackFrameSP &, ValueObjectSP &) {
  return Status();
}

ValueObjectSP ABIMC6809::GetReturnValueObjectImpl(Thread &,
                                                   CompilerType &) const {
  return ValueObjectSP();
}

// At function entry the stack pointer (S) points at the 2-byte return
// address pushed by JSR/BSR.  CFA = S + 2; return address = mem[S].
UnwindPlanSP ABIMC6809::CreateFunctionEntryUnwindPlan() {
  UnwindPlan::Row row;
  row.GetCFAValue().SetIsRegisterPlusOffset(dwarf_s, 2);
  row.SetRegisterLocationToAtCFAPlusOffset(dwarf_pc, -2, true);
  row.SetRegisterLocationToIsCFAPlusOffset(dwarf_s, 0, true);

  auto plan_sp = std::make_shared<UnwindPlan>(eRegisterKindDWARF);
  plan_sp->AppendRow(std::move(row));
  plan_sp->SetSourceName("mc6809 at-func-entry default");
  plan_sp->SetSourcedFromCompiler(eLazyBoolNo);
  return plan_sp;
}

// Default unwind: same as function-entry — CFA = S + 2.
UnwindPlanSP ABIMC6809::CreateDefaultUnwindPlan() {
  UnwindPlan::Row row;
  row.GetCFAValue().SetIsRegisterPlusOffset(dwarf_s, 2);
  row.SetRegisterLocationToAtCFAPlusOffset(dwarf_pc, -2, true);
  row.SetRegisterLocationToIsCFAPlusOffset(dwarf_s, 0, true);
  row.SetRegisterLocationToUnspecified(dwarf_u, true);

  auto plan_sp = std::make_shared<UnwindPlan>(eRegisterKindDWARF);
  plan_sp->AppendRow(std::move(row));
  plan_sp->SetSourceName("mc6809 default unwind plan");
  plan_sp->SetSourcedFromCompiler(eLazyBoolNo);
  plan_sp->SetUnwindPlanValidAtAllInstructions(eLazyBoolNo);
  return plan_sp;
}

// MC6809 llvm ABI callee-saves: U, S (and CC, DP in practice, but
// for unwinding the volatile set is D, X, Y, A, B.
bool ABIMC6809::RegisterIsVolatile(const RegisterInfo *reg_info) {
  if (!reg_info)
    return false;
  const char *name = reg_info->name;
  if (!name)
    return false;
  // Callee-saved: u, s, dp — everything else is volatile.
  return !(strcmp(name, "u") == 0 || strcmp(name, "s") == 0 ||
           strcmp(name, "dp") == 0);
}

void ABIMC6809::Initialize() {
  PluginManager::RegisterPlugin(GetPluginNameStatic(),
                                "ABI for Motorola MC6809 / Hitachi HD6309",
                                CreateInstance);
}

void ABIMC6809::Terminate() {
  PluginManager::UnregisterPlugin(CreateInstance);
}

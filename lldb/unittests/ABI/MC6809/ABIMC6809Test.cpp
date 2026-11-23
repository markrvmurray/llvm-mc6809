//===-- ABIMC6809Test.cpp -------------------------------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Plugins/ABI/MC6809/ABIMC6809.h"
#include "lldb/Symbol/UnwindPlan.h"
#include "lldb/Utility/ArchSpec.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/TargetSelect.h"
#include "gtest/gtest.h"
#include <cstring>

using namespace lldb_private;
using namespace lldb;

extern "C" void LLVMInitializeMC6809TargetInfo();
extern "C" void LLVMInitializeMC6809TargetMC();

namespace {

// DWARF register numbers, as MC6809RegisterInfo.td defines them.
enum : uint32_t { DwarfS = 4, DwarfPC = 5 };

class ABIMC6809Test : public testing::Test {
public:
  static void SetUpTestCase() {
    LLVMInitializeMC6809TargetInfo();
    LLVMInitializeMC6809TargetMC();
    ABIMC6809::Initialize();
  }
  static void TearDownTestCase() {
    ABIMC6809::Terminate();
    llvm::llvm_shutdown();
  }

protected:
  ABISP abi() { return ABI::FindPlugin(ProcessSP(), ArchSpec("mc6809")); }
};

} // namespace

// The default plan stands for a frame lldb knows nothing else about, so a
// register it does not describe must read as undefined in the caller.  lldb
// asserts on this in RegisterContextUnwind, and without it a debug build
// stopped at the first stop of every gdb-remote session.
TEST_F(ABIMC6809Test, DefaultUnwindPlanLeavesUnspecifiedRegistersUndefined) {
  ABISP abi_sp = abi();
  ASSERT_TRUE(abi_sp);
  UnwindPlanSP plan = abi_sp->CreateDefaultUnwindPlan();
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->GetRowCount(), 1);
  const UnwindPlan::Row *row = plan->GetRowAtIndex(0);
  ASSERT_TRUE(row);
  EXPECT_TRUE(row->GetUnspecifiedRegistersAreUndefined());
}

// Both plans start from the same frame: S points at the return address JSR
// or BSR pushed, so the CFA is S+2, the caller's PC is at CFA-2, and the
// caller's S is the CFA.
TEST_F(ABIMC6809Test, UnwindPlansFindTheReturnAddress) {
  ABISP abi_sp = abi();
  ASSERT_TRUE(abi_sp);
  for (UnwindPlanSP plan : {abi_sp->CreateDefaultUnwindPlan(),
                            abi_sp->CreateFunctionEntryUnwindPlan()}) {
    ASSERT_TRUE(plan);
    const UnwindPlan::Row *row = plan->GetRowAtIndex(0);
    ASSERT_TRUE(row);
    EXPECT_TRUE(row->GetCFAValue().IsRegisterPlusOffset());
    EXPECT_EQ(row->GetCFAValue().GetRegisterNumber(), DwarfS);
    EXPECT_EQ(row->GetCFAValue().GetOffset(), 2);

    UnwindPlan::Row::AbstractRegisterLocation pc;
    ASSERT_TRUE(row->GetRegisterInfo(DwarfPC, pc));
    EXPECT_TRUE(pc.IsAtCFAPlusOffset());
    EXPECT_EQ(pc.GetOffset(), -2);

    UnwindPlan::Row::AbstractRegisterLocation s;
    ASSERT_TRUE(row->GetRegisterInfo(DwarfS, s));
    EXPECT_TRUE(s.IsCFAPlusOffset());
    EXPECT_EQ(s.GetOffset(), 0);
  }
}

// The compiler's calling convention (MC6809_CSR) preserves Y and U across
// a call; S comes back where it was and compiled code never changes DP.
// D (A:B), X, CC and PC are the callee's.  A register wrongly called
// volatile is discarded in every caller's frame.
TEST_F(ABIMC6809Test, VolatileRegistersMatchTheCallingConvention) {
  ABISP abi_sp = abi();
  ASSERT_TRUE(abi_sp);
  uint32_t count = 0;
  const RegisterInfo *regs =
      static_cast<ABIMC6809 *>(abi_sp.get())->GetRegisterInfoArray(count);
  ASSERT_TRUE(regs);

  struct Expect {
    const char *name;
    bool is_volatile;
  };
  const Expect expected[] = {
      {"y", false},  {"u", false}, {"s", false},  {"dp", false},
      {"d", true},   {"a", true},  {"b", true},   {"x", true},
      {"cc", true},  {"pc", true},
  };
  for (const Expect &e : expected) {
    const RegisterInfo *found = nullptr;
    for (uint32_t i = 0; i < count; ++i)
      if (regs[i].name && std::strcmp(regs[i].name, e.name) == 0)
        found = &regs[i];
    ASSERT_TRUE(found) << "no register named " << e.name;
    EXPECT_EQ(abi_sp->RegisterIsVolatile(found), e.is_volatile) << e.name;
  }
}

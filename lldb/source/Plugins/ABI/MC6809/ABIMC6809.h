//===-- ABIMC6809.h -----------------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_SOURCE_PLUGINS_ABI_MC6809_ABIMC6809_H
#define LLDB_SOURCE_PLUGINS_ABI_MC6809_ABIMC6809_H

#include "lldb/Target/ABI.h"
#include "lldb/Utility/Log.h"

class ABIMC6809 : public lldb_private::RegInfoBasedABI {
public:
  ~ABIMC6809() override = default;

  static void Initialize();
  static void Terminate();

  static lldb::ABISP CreateInstance(lldb::ProcessSP process_sp,
                                    const lldb_private::ArchSpec &arch);

  static llvm::StringRef GetPluginNameStatic() { return "mc6809"; }

  llvm::StringRef GetPluginName() override { return GetPluginNameStatic(); }

  bool PrepareTrivialCall(lldb_private::Thread &thread, lldb::addr_t sp,
                          lldb::addr_t pc, lldb::addr_t ra,
                          llvm::ArrayRef<lldb::addr_t> args) const override;

  bool GetArgumentValues(lldb_private::Thread &thread,
                         lldb_private::ValueList &values) const override;

  lldb_private::Status
  SetReturnValueObject(lldb::StackFrameSP &frame_sp,
                       lldb::ValueObjectSP &new_value_sp) override;

  lldb::ValueObjectSP
  GetReturnValueObjectImpl(lldb_private::Thread &thread,
                           lldb_private::CompilerType &type) const override;

  lldb::UnwindPlanSP CreateFunctionEntryUnwindPlan() override;

  lldb::UnwindPlanSP CreateDefaultUnwindPlan() override;

  bool RegisterIsVolatile(const lldb_private::RegisterInfo *reg_info) override;

  bool CallFrameAddressIsValid(lldb::addr_t cfa) override {
    return cfa != 0 && cfa != LLDB_INVALID_ADDRESS;
  }

  bool CodeAddressIsValid(lldb::addr_t pc) override {
    return pc != LLDB_INVALID_ADDRESS;
  }

  const lldb_private::RegisterInfo *
  GetRegisterInfoArray(uint32_t &count) override;

  size_t GetRedZoneSize() const override { return 0; }

private:
  using lldb_private::RegInfoBasedABI::RegInfoBasedABI;
};

#endif // LLDB_SOURCE_PLUGINS_ABI_MC6809_ABIMC6809_H

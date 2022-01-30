//===-- MC6809Features.td - MC6809 Features ---------------------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
// MC6809 Features
//===---------------------------------------------------------------------===//

def FeatureAltRegisterNamesOnly
    : SubtargetFeature<"mc6809-long-register-names", "LongRegisterNames",
                        "true",
                        "Requires llvm_mc6809_* prefixes to all registers. "#
                        "Useful if your code has variable names that conflict "#
                        "with llvm-mc6809 register names">;

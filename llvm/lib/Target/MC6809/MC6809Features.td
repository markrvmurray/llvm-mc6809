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

// Bug #387: lay the frames of provably-non-reentrant functions out in
// statically-allocated global memory (extended/absolute addressing) instead
// of on the dynamic U-relative stack. Opt-in for now; default off.
def FeatureStaticStack
    : SubtargetFeature<"static-stack", "StaticStack", "true",
                        "Allocate non-reentrant function frames to static "#
                        "memory rather than the dynamic stack">;

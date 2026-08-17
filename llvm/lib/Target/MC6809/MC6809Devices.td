//===-- MC6809Devices.td - MC6809 Devices -----------------------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
// MC6809 Device Definitions
//===---------------------------------------------------------------------===//

def Feature6809
    : SubtargetFeature<"mc6809-insns-6809", "Has6809Insns", "true",
                       "The original Motorola MC6809 instruction set">;

def Feature6309
    : SubtargetFeature<"mc6809-insns-6309", "Has6309Insns", "true",
                       "The extended Hitachi HD6309 instruction set">;

// A feature set aggregates features, grouping them. We don't want to create a
// new member in MC6809Subtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
  : SubtargetFeature<name, "FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, string desc, list<SubtargetFeature> i>
  : FeatureSet<name, desc, i>;


// FeatureStaticStack is enabled by default for every CPU: a provably
// non-reentrant leaf function (no calls, no frame address taken, no
// variable-sized objects) lays its frame in static global memory rather than
// the dynamic U-relative stack. That frees the frame pointer entirely -- no
// `pshs u; tfr s,u`, U becomes a third index register -- which is where the win
// lives (hot string/loop leaves: strcmp/memmove -17..-26% cycles at -Os on
// HD6309). Functions that keep their frame pointer anyway are left on the
// dynamic frame: for them a static frame only costs (extended addressing is a
// byte longer and a cycle slower than a short `n,u` access) and it widens the
// whole-program static_stack, so restricting to leaves measured better on
// every tier of the picolibc bench than static-stacking every non-reentrant
// function. -mc6809-static-stack-dp-avail sizes the page-0 window used for
// 1-byte direct-page frame addressing; -mattr=-static-stack disables the
// feature for a translation unit.
def Family6809
    : Family<
      "mc6809",
      "Generic MC6809 processor",
      [Feature6809, FeatureStaticStack]>;

def Family6309
    : Family<"hd6309",
      "Extended HD6309 processor",
      [Feature6809, Feature6309, FeatureStaticStack]>;

//===---------------------------------------------------------------------===//
// MC6809 Families
//===---------------------------------------------------------------------===//

def MC6809SchedModel : SchedMachineModel {
  // Prevent instructions from being added to the pending queue, since latency
  // doesn't matter. The remaining latency concerns are disabled in the C++
  // scheduling strategy.
  let MicroOpBufferSize = 1;
  let CompleteModel = false;
  // Bug #165 Phase D: enable the LEGACY post-RA list scheduler. The existing
  // branch instructions already carry `Uses = [N, Z, V, C]` (per bug #137's
  // load-bearing fix at MC6809InstrInfo.td:507-526), and the per-instruction
  // CC clobbers were audited clean in #165 phase A2. So the scheduler has
  // accurate dataflow on the compare→branch dependency edge without needing
  // a full Lanai-style WriteRes / ProcResource model. If reordering reveals
  // missing CC annotations in practice, those will surface as bench
  // regressions and can be fixed point-wise.
  let PostRAScheduler = 1;
}

class Device<string Name, Family Fam,
             list<SubtargetFeature> ExtraFeatures = []>
  : ProcessorModel<Name, MC6809SchedModel, !listconcat([Fam],ExtraFeatures)>;

def : Device<"mc6809", Family6809>;
def : Device<"hd6309", Family6309>;

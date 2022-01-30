//===-- MC6809Devices.td - MC6809 Devices -----------------------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
// MC6809 Device Definitions
//===---------------------------------------------------------------------===//

def Feature6309 : SubtargetFeature<"mc6809-insns-6309", "Has6309Insns", "true",
                       "The new instructions and registers on the Hitachi HD6309">;

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

def Family6809 : Family<"mc6809", "Generic MC6809", []>;

def Family6309 : Family<"hd6309", "Extended HD6309", [Feature6309]>;

//===---------------------------------------------------------------------===//
// MC6809 Families
//===---------------------------------------------------------------------===//

def MC6809SchedModel : SchedMachineModel {
  // Prevent instructions from being added to the pending queue, since latency
  // doesn't matter. The remaining latency concerns are disabled in the C++
  // scheduling strategy.
  let MicroOpBufferSize = 1;
  let CompleteModel = false;
}

class Device<string Name, Family Fam,
             list<SubtargetFeature> ExtraFeatures = []>
  : ProcessorModel<Name, MC6809SchedModel, !listconcat([Fam],ExtraFeatures)>;

def : Device<"mc6809", Family6809>;
def : Device<"hd6309", Family6309>;

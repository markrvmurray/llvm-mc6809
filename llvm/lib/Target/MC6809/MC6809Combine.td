//===-- MC6809Combine.td - Describe MC6809 combiner -------------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes which combine operations are included in the MC6809
// combiner.
//
//===---------------------------------------------------------------------===//

// ============================================================================
def fold_global_offset_matchdata : GIDefMatchData<"std::pair<const MachineOperand*, int64_t>">;
def fold_global_offset : GICombineRule<
  (defs root:$root, fold_global_offset_matchdata:$matchinfo),
  (match (wip_match_opcode G_PTR_ADD):$root, [{ return matchFoldGlobalOffset(*${root}, MRI, ${matchinfo}); }]),
  (apply [{ return applyFoldGlobalOffset(*${root}, MRI, B, Observer, ${matchinfo});}])
>;

//  %2:_(s16) = G_SEXT %1:_(s8)
//  %3:_(p0) = G_PTR_ADD %0:_, %2:_(s16)
//   =>
//  %3:_(s8) = G_PTR_ADD %0:_, %1:_(s8)
def fold_pointer_sext_offset_matchdata : GIDefMatchData<"std::tuple<MachineInstr *>">;
def fold_pointer_sext_offset : GICombineRule<
  (defs root:$root, fold_pointer_sext_offset_matchdata:$matchinfo),
  (match (wip_match_opcode G_PTR_ADD):$root, [{ return matchFoldPointerSExtOffset(*${root}, MRI, ${matchinfo}); }]),
  (apply [{ return applyFoldPointerSExtOffset(*${root}, MRI, B, Observer, ${matchinfo});}])
>;

// ============================================================================
def MC6809CombinerHelper: GICombinerHelper<"MC6809GenCombinerHelper",
                                           [all_combines,
                                            fold_global_offset,
                                            fold_pointer_sext_offset
                                            ]> {
  let DisableRuleOption = "mc6809combiner-disable-rule";
  let StateClass = "MC6809CombinerHelperState";
  let AdditionalArguments = [];
}

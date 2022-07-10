//===-- MC6809Combine.td - Describe MC6809 combiner -------*- tablegen -*-===//
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
//  %2:_(p0) = G_GLOBAL_VALUE @fold_global_offset
//  %3:_(s16) = G_CONSTANT i16 42
//  %1:_(p0) = G_PTR_ADD %2, %3(s16)
//   =>
//  %1:_(p0) = G_GLOBAL_VALUE @fold_global_offset + 42
def fold_global_offset_matchdata : GIDefMatchData<"std::pair<const MachineOperand *, int64_t>">;
def fold_global_offset : GICombineRule<
  (defs root:$root, fold_global_offset_matchdata:$matchinfo),
  (match (wip_match_opcode G_PTR_ADD):$root, [{ return matchFoldGlobalOffset(*${root}, MRI, ${matchinfo}); }]),
  (apply [{ return applyFoldGlobalOffset(*${root}, MRI, B, Observer, ${matchinfo});}])
>;

// ============================================================================
//  %2:_(p0) = G_GLOBAL_VALUE @fold_global_copy
//  %3:_(s16) = COPY %2:_(p0)
//   =>
//  %3:_(s16) = G_GLOBAL_VALUE @fold_global_copy
def fold_global_copy_matchdata : GIDefMatchData<"std::pair<const MachineOperand *, MachineInstr *>">;
def fold_global_copy : GICombineRule<
  (defs root:$root, fold_global_copy_matchdata:$matchinfo),
  (match (wip_match_opcode COPY):$root, [{ return matchFoldGlobalCopy(*${root}, MRI, ${matchinfo}); }]),
  (apply [{ return applyFoldGlobalCopy(*${root}, MRI, B, Observer, ${matchinfo});}])
>;

// ============================================================================
//  %2:_(s16) = G_[SZ]SEXT %1:_(s8)
//  %3:_(p0) = G_PTR_ADD %0:_, %2:_(s16)
//   =>
//  %3:_(s8) = G_PTR_ADD %0:_, %1:_(s8)
def fold_pointer_ext_offset_matchdata : GIDefMatchData<"std::pair<MachineInstr *, MachineInstr *>">;
def fold_pointer_ext_offset : GICombineRule<
  (defs root:$root, fold_pointer_ext_offset_matchdata:$matchinfo),
  (match (wip_match_opcode G_PTR_ADD):$root, [{ return matchFoldPointerExtOffset(*${root}, MRI, ${matchinfo}); }]),
  (apply [{ return applyFoldPointerExtOffset(*${root}, MRI, B, Observer, ${matchinfo});}])
>;

// ============================================================================
//  liveins: $ab(, $af)
//  %0:_(s8) = COPY $ab
//  ( %1:_(s8) = COPY $af // Antipattern )
//  %5:_(p0) = G_FRAME_INDEX %fixed-stack.3
//  %1:_(s8) = G_LOAD %5:_(p0) :: (invariant load (s8) from %fixed-stack.3, align 2)
//  %9:_(s8) = G_ADD %1:_, %0:_ ; The known physical register is on the wrong side
//   =>
//  :
//  %3:_(s8) = G_ADD %0:_, %1:_ ; Can make use of addressing modes
def swap_physreg_to_lhs_matchdata : GIDefMatchData<"MachineInstr *">;
def swap_physreg_to_lhs : GICombineRule<
  (defs root:$root, swap_physreg_to_lhs_matchdata:$matchinfo),
  (match (wip_match_opcode G_ADD, G_SADDO, G_SADDE, G_UADDO, G_UADDE):$root,
    [{ return matchSwapPhysregToLhs(*${root}, MRI, ${matchinfo}); }]),
  (apply [{ return applySwapPhysregToLhs(*${root}, MRI, B, Observer, ${matchinfo});}])
>;

//  %0:accum(s8) = COPY $ab
//  :
//  %11:accum(s8) = G_ADD %10:accum, %4:accum
//  %12:accum(s8) = G_SUB %0:accum, %11:accum
//   =>
//  %0:accum(s8) = COPY $ab
//  :
//  %12:accum(s8) = G_SUB %0:accum, %4:accum
//  %13:accum(s8) = G_SUB %12:accum, %10:accum
def switch_add_to_subtract_matchdata : GIDefMatchData<"MachineInstr *">;
def switch_add_to_subtract : GICombineRule<
  (defs root:$root, switch_add_to_subtract_matchdata:$matchinfo),
  (match (wip_match_opcode G_SUB):$root,
    [{ return matchSwitchAddToSubtract(*${root}, MRI, ${matchinfo}); }]),
  (apply [{ return applySwitchAddToSubtract(*${root}, MRI, B, Observer, ${matchinfo});}])
>;

// ============================================================================
def MC6809CombinerHelper: GICombinerHelper<"MC6809GenCombinerHelper",
                                           [all_combines,
                                            fold_global_offset,
                                            // fold_global_copy,
                                            fold_pointer_ext_offset,
                                            // swap_physreg_to_lhs,
                                            switch_add_to_subtract,
                                            ]> {
  let DisableRuleOption = "mc6809combiner-disable-rule";
  let StateClass = "MC6809CombinerHelperState";
  let AdditionalArguments = [];
}

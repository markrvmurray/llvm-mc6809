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

def fold_global_offset_matchdata : GIDefMatchData<"std::pair<const MachineOperand*, int64_t>">;
def fold_global_offset : GICombineRule<
  (defs root:$root, fold_global_offset_matchdata:$matchinfo),
  (match (wip_match_opcode G_PTR_ADD):$root,
          [{ return matchFoldGlobalOffset(*${root}, MRI, ${matchinfo}); }]),
  (apply [{  return applyFoldGlobalOffset(*${root}, MRI, B, Observer, ${matchinfo});}])>;

def extract_low_bit_matchdata : GIDefMatchData<"MachineInstr*">;
def extract_low_bit : GICombineRule<
  (defs root:$root, extract_low_bit_matchdata:$matchinfo),
  (match (wip_match_opcode G_TRUNC, G_ICMP):$root,
          [{ return matchExtractLowBit(*${root}, MRI, ${matchinfo}); }]),
  (apply [{  return applyExtractLowBit(*${root}, MRI, B, Observer, ${matchinfo});}])>;

def MC6809CombinerHelper: GICombinerHelper<
  "MC6809GenCombinerHelper", [all_combines, fold_global_offset, extract_low_bit]> {
  let DisableRuleOption = "mc6809combiner-disable-rule";
  let StateClass = "MC6809CombinerHelperState";
  let AdditionalArguments = [];
}

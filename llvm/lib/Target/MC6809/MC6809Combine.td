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

def fold_global_offset_matchdata : GIDefMatchData<"std::pair<const MachineOperand*, int64_t>">;
def fold_global_offset : GICombineRule<
  (defs root:$root, fold_global_offset_matchdata:$matchinfo),
  (match (wip_match_opcode G_PTR_ADD):$root,
          [{ return matchFoldGlobalOffset(*${root}, ${matchinfo}); }]),
  (apply [{ applyFoldGlobalOffset(*${root}, ${matchinfo});}])>;

def unmerge_high_from_load_matchdata : GIDefMatchData<"MachineOperand *">;
def unmerge_high_from_load : GICombineRule<
  (defs root:$root, unmerge_high_from_load_matchdata:$matchinfo),
  (match (wip_match_opcode G_UNMERGE_VALUES):$root,
          [{ return matchUnmergeHighFromLoad(*${root}, ${matchinfo}); }]),
  (apply [{ applyUnmergeHighFromLoad(*${root}, ${matchinfo});}])>;

def unmerge_low_from_load_matchdata : GIDefMatchData<"MachineOperand *">;
def unmerge_low_from_load : GICombineRule<
  (defs root:$root, unmerge_low_from_load_matchdata:$matchinfo),
  (match (wip_match_opcode G_UNMERGE_VALUES):$root,
          [{ return matchUnmergeLowFromLoad(*${root}, ${matchinfo}); }]),
  (apply [{ applyUnmergeLowFromLoad(*${root}, ${matchinfo});}])>;

def extract_low_bit_matchdata : GIDefMatchData<"MachineInstr*">;
def extract_low_bit : GICombineRule<
  (defs root:$root, extract_low_bit_matchdata:$matchinfo),
  (match (wip_match_opcode G_TRUNC, G_ICMP):$root,
          [{ return matchExtractLowBit(*${root}, ${matchinfo}); }]),
  (apply [{ applyExtractLowBit(*${root}, ${matchinfo});}])>;

def uaddo_1 : GICombineRule<
  (defs root:$root),
  (match (wip_match_opcode G_UADDO):$root,
          [{ return matchUAddO1(*${root}); }]),
  (apply [{ applyUAddO1(*${root});}])>;

def load_store_to_memcpy_matchdata : GIDefMatchData<"GLoad*">;
def load_store_to_memcpy : GICombineRule<
  (defs root:$root, load_store_to_memcpy_matchdata:$matchinfo),
  (match (wip_match_opcode G_STORE):$root,
          [{ return matchLoadStoreToMemcpy(*${root}, ${matchinfo}); }]),
  (apply [{ applyLoadStoreToMemcpy(*${root}, ${matchinfo});}])>;

def store_to_memset_matchdata : GIDefMatchData<"uint8_t">;
def store_to_memset : GICombineRule<
  (defs root:$root, store_to_memset_matchdata:$matchinfo),
  (match (wip_match_opcode G_STORE):$root,
          [{ return matchStoreToMemset(*${root}, ${matchinfo}); }]),
  (apply [{ applyStoreToMemset(*${root}, ${matchinfo});}])>;

def fold_adde : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_UADDE, G_SADDE):$root,
          [{ return matchFoldAddE(*${root}, ${matchinfo}); }]),
  (apply [{ Helper.applyBuildFn(*${root}, ${matchinfo}); }])>;

def fold_shift : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_SHLE, G_LSHRE):$root,
          [{ return matchFoldShift(*${root}, ${matchinfo}); }]),
  (apply [{ Helper.applyBuildFn(*${root}, ${matchinfo}); }])>;

def shift_unused_carry_in : GICombineRule<
  (defs root:$root, build_fn_matchinfo:$matchinfo),
  (match (wip_match_opcode G_SHLE, G_LSHRE):$root,
          [{ return matchShiftUnusedCarryIn(*${root}, ${matchinfo}); }]),
  (apply [{ Helper.applyBuildFnNoErase(*${root}, ${matchinfo}); }])>;

// Custom all_combines without hoist_logic_op_with_same_opcode_hands,
// which causes an infinite loop on MC6809 (bug #10): the transformation
// creates a pattern that re-matches the same rule.
def mc6809_all_combines : GICombineGroup<[integer_reassoc_combines,
    trivial_combines, vector_ops_combines, freeze_combines, cast_combines,
    insert_vec_elt_combines, extract_vec_elt_combines, combines_for_extload,
    combine_extracted_vector_load,
    undef_combines, identity_combines, phi_combines,
    simplify_add_to_sub, /* hoist_logic_op_with_same_opcode_hands, */ shifts_too_big,
    reassocs, ptr_add_immed_chain, cmp_combines,
    shl_ashr_to_sext_inreg, neg_and_one_to_sext_inreg, sext_inreg_of_load,
    width_reduction_combines, select_combines,
    known_bits_simplifications, trunc_shift,
    not_cmp_fold, opt_brcond_by_inverting_cond,
    const_combines, xor_of_and_with_same_reg, ptr_add_with_zero,
    shift_immed_chain, shift_of_shifted_logic_chain, load_or_combine,
    div_rem_to_divrem, funnel_shift_combines, bitreverse_shift, commute_shift,
    form_bitfield_extract, constant_fold_binops, constant_fold_fma,
    constant_fold_cast_op, fabs_fneg_fold,
    mulh_combines, redundant_neg_operands,
    and_or_disjoint_mask, fma_combines, fold_binop_into_select,
    intrem_combines, intdiv_combines, fdiv_repeated_divison,
    sub_add_reg, select_to_minmax,
    fsub_to_fneg, commute_constant_to_rhs, match_ands, match_ors,
    simplify_neg_minmax, combine_concat_vector,
    sext_trunc, zext_trunc, prefer_sign_combines, shuffle_combines,
    combine_use_vector_truncate, merge_combines, overflow_combines,
    truncsat_combines, lshr_of_trunc_of_lshr, ctls_combines, add_shift,
    sub_one_from_sub]>;

def MC6809Combiner : GICombiner<"MC6809CombinerImpl", [
    mc6809_all_combines,
    // unmerge_high_from_load,
    // unmerge_low_from_load,
    // fold_global_offset,
    // extract_low_bit,
    // uaddo_1,
    // load_store_to_memcpy,
    // store_to_memset,
    // fold_adde,
    // fold_shift,
    // shift_unused_carry_in
]>;

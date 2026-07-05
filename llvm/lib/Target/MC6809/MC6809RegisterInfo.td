//===-- MC6809RegisterInfo.td - MC6809 Register defs -------------*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Subregister indices
//===----------------------------------------------------------------------===//

class MC6809SubRegIndex<int size, int offset = 0>: SubRegIndex<size, offset> {
  let Namespace = "MC6809";
}

// Bytes of a 16-bit word
def sub_lo_byte : MC6809SubRegIndex<8>;
def sub_hi_byte : MC6809SubRegIndex<8, 8>;

// 16-bit half-words of a 32-bit word
def sub_lo_word : MC6809SubRegIndex<16>;
def sub_hi_word : MC6809SubRegIndex<16, 16>;

// Bits of the 8-bit processor status register.
def sub_entire : MC6809SubRegIndex<1, 7>;
def sub_firq : MC6809SubRegIndex<1, 6>;
def sub_half : MC6809SubRegIndex<1, 5>;
def sub_irq : MC6809SubRegIndex<1, 4>;
def sub_negative : MC6809SubRegIndex<2, 2>;
def sub_zero : MC6809SubRegIndex<1, 3>;
def sub_overflow : MC6809SubRegIndex<1, 1>;
def sub_carry : MC6809SubRegIndex<1, 0>;

// 2-bit NZ subregister
def sub_nz_negative : MC6809SubRegIndex<1, 1>;
def sub_nz_zero : MC6809SubRegIndex<1, 0>;

// Bits of the 8-bit processor status register set by most instructions.
def sub_nzvc : MC6809SubRegIndex<4, 0>;

//===----------------------------------------------------------------------===//
//  Real registers
//===----------------------------------------------------------------------===//

class MC6809Reg<bits<16> num, string name>
  : Register<name>, DwarfRegNum<[num]> {
  field bits<16> Num = num;
  let HWEncoding = num;
  let Namespace = "MC6809";
  let AltNames = [name];
}

class MC6809Reg1<bits<16> num, string name>
  : MC6809Reg<num, name> {
    let SubRegs = [];
    let SubRegIndices = [];
    let CoveredBySubRegs = true;
}

class MC6809Reg8<bits<16> num, string name, list<Register> subregs = [], list<SubRegIndex> subregindices = []>
  : MC6809Reg<num, name> {
    let SubRegs = subregs;
    let SubRegIndices = subregindices;
    let CoveredBySubRegs = false;
}

// An 8-bit value register (GPR or imaginary).  i1 values live in
// ACC8 directly — the byte's bit 0 carries the boolean.
// MC6809ISelLowering binds MVT::i1 to ACC8RegClass.
multiclass MC6809Reg8<bits<16> num, string name> {
  def NAME : MC6809Reg8<num, name, [], []>;
}

class MC6809Reg16<bits<16> num, string name, list<Register> subregs = [], list<SubRegIndex> subregindices = []>
  : MC6809Reg<num, name> {
    let SubRegs = subregs;
    let SubRegIndices = subregindices;
    let CoveredBySubRegs = true;
}

class MC6809Reg32<bits<16> num, string name, list<Register> subregs = [], list<SubRegIndex> subregindices = []>
  : MC6809Reg<num, name> {
    let SubRegs = subregs;
    let SubRegIndices = subregindices;
    let CoveredBySubRegs = true;
}

//===----------------------------------------------------------------------===//
//  Imaginary registers
//===----------------------------------------------------------------------===//

// This section describes the dimension of the imaginary registers for the MC6809
// targets. Imaginary registers are locations in the direct page that llvm can
// treat as physical registers for the purpose of register allocation.

// Imaginary registers are reserved in pairs, which allows them to be used as
// pointers with the 6809's various addressing modes. The number of imaginary
// register pointer pairs is configurable at code generation time via the
// --num-imag-ptrs flag. See MC6809RegisterInfo.cpp for details.

// The two bytes of any given pointer must be contiguous, but otherwise, there
// are no constraints on the relative layout of the pointers. For example, RS0
// could be allocated to 0x34 and 0x35, while RS1 is allocated to 0x12 and 0x13.
// The code generator emits generic symbol references for imaginary registers,
// and the linker script for a given memory layout later assigns these to their
// real locations.
//
// 32-bit registers are also created, similarly to the 16-bit ones described
// above. These will be used for 32-bit arithmetic and for floating-point.

// 8-bit imaginary registers.
multiclass MC6809ImagReg8<bits<16> num, string name> : MC6809Reg8<num, name>;

// 16-bit imaginary registers — independent, no sub-register aliasing with
// 8-bit imaginary registers. Memory-backed registers don't physically alias,
// so sub-register composition would only create artificial regalloc pressure.
class MC6809ImagReg16<bits<16> num, string name>
    : MC6809Reg<num, name>;

// 32-bit imaginary registers — independent, no sub-register aliasing.
class MC6809ImagReg32<bits<16> num, string name>
    : MC6809Reg<num, name>;

defvar MaxImag8Regs = 256;

// The starting DWARF number for the imaginary registers.
defvar Imag8RegsOffset = 0x1000;

defvar MaxImag16Regs = !sra(MaxImag8Regs, 1);
defvar Imag16RegsOffset = !add(Imag8RegsOffset, !shl(MaxImag16Regs,1));

defvar MaxImag32Regs = !sra(MaxImag16Regs, 1);
defvar Imag32RegsOffset = !add(Imag16RegsOffset, !shl(MaxImag16Regs,1));

// Now we enumerate the imaginary registers.
// Imaginary 8-bit registers, starting with the prefix rc
foreach I = 0...!add(MaxImag8Regs, -1) in {
  // There exist MaxImag8Regs rcXX registers...
  let CostPerUse = [1] in
  defm RC#I: MC6809ImagReg8<!add(!shl(I,1), Imag8RegsOffset), "rc"#!cast<string>(I)>;
}

// Imaginary 16-bit registers, starting with the prefix rs.
//
// RS0..RS3 (the registers actually in allocation classes ACC16/ADc)
// have byte-level sub-register structure so that TableGen-synthesised
// intersection classes like ACC16_with_sub_hi_byte include them.
// Without sub_hi_byte, the regalloc silently excludes RS from any
// instruction that extracts a byte half — the entire 32-bit arithmetic
// path. This was the Layer 1 root cause of regalloc-pressure bugs
// #82b/#85b.
//
// The sub-regs are NOT in any allocatable 8-bit class (ACC8, AAc, ABc).
// They exist only as metadata for TableGen's constraint-class synthesis.
// The regalloc never assigns vregs to them directly — it assigns to
// RS0..RS3 at the 16-bit level, and the existing materializeReg/
// dematerializeReg handles those via LDD/STD to the direct-page symbol.

// Byte sub-registers for RS0..RS3. Plain MC6809Reg — NOT in any
// 8-bit allocatable class. Adding RS bytes to ACC8/AAc/ABc
// destabilises the greedy regalloc's heuristics system-wide
// (different allocation choices → cascade → miscompiles at O1+).
// The sub-regs exist ONLY to give RS sub_hi_byte/sub_lo_byte indices.
//
// Layout (interleaved hi/lo in DWARF numbering):
//   RS{I}HI = 0x3000 + I*2,   RS{I}LO = 0x3001 + I*2
//   RS{I}   = Imag16RegsOffset + I   (16-bit composed)
// CostPerUse: the imaginaries are direct-page MEMORY -- every access is
// a real load/store, and post-RA staging brackets multiply that cost in
// hot code (a rematerialised constant re-stored to __rs0 per loop
// iteration cost strcmp +52% cycles on HD6309). Bias regalloc to real
// registers whenever one is free; RS remain available under genuine
// pressure.
let CostPerUse = [1] in
foreach I = 0...3 in {
  def RS#I#HI : MC6809Reg<!add(0x3000, !mul(I, 2)),     "rs"#I#"hi">;
  def RS#I#LO : MC6809Reg<!add(0x3001, !mul(I, 2)),     "rs"#I#"lo">;
  def RS#I    : MC6809Reg16<!add(Imag16RegsOffset, I),  "rs"#I,
      [!cast<Register>("RS"#I#"LO"), !cast<Register>("RS"#I#"HI")],
      [sub_lo_byte, sub_hi_byte]>;
}

// Bug-#207 follow-up audit (2026-05-02): RS4..RS127 were trimmed
// after llvm-readelf across 564 .o files (full picolibc HD6309 -Oz
// libc.a + semihost + every test binary) showed ZERO references to
// any `__rs4`..`__rs127` symbol. The pool was sized for round-14
// worst-case pressure (i64-multiply chains) which the subsequent
// fused-cmp-branch and TFM work has materially relieved. Trimming
// reclaims ~248 bytes of direct-page address space for user code
// (`__attribute__((directpage))` per bugs #178/#192/#193).
//
// RS0..RS3 are kept: they have byte sub-regs the EXTRACT_LO/HI_i16
// pseudo expansions and the ADc class still depend on, and several
// .cpp source sites enumerate them by name (sub-reg-parent map, the
// post-RA materialisation helpers).
//
// To restore the larger pool if needed: re-add the foreach below
// AND add the names back to ACC16's `add`+`AltOrders` lists below.

// Imaginary 32-bit registers, starting with the prefix rw
foreach I = 0...!add(MaxImag32Regs, -1) in {
  def RW#I: MC6809ImagReg32<!add(I, Imag32RegsOffset), "rw"#!cast<string>(I)>;
}

//===----------------------------------------------------------------------===//
//  Define the actual real registers.
//===----------------------------------------------------------------------===//

def E : MC6809Reg1<48, "E">;
def F : MC6809Reg1<49, "F">;
def H : MC6809Reg1<50, "H">;
def I : MC6809Reg1<51, "I">;
def N : MC6809Reg1<52, "N">;
def Z : MC6809Reg1<53, "Z">;
def V : MC6809Reg1<54, "V">;
def C : MC6809Reg1<55, "C">;

def NZ : MC6809Reg<65, "NZ"> {
  let SubRegs = [N, Z];
  let SubRegIndices = [sub_nz_negative, sub_nz_zero];
  let CoveredBySubRegs = true;
}

def NZVC : MC6809Reg<66, "NZVC"> {
  let SubRegs = [N, Z, V, C];
  let SubRegIndices = [sub_negative, sub_zero, sub_overflow, sub_carry];
  let CoveredBySubRegs = true;
}

def CC : MC6809Reg<10, "cc"> {
  let SubRegs = [E, F, H, I, N, Z, V, C];
  let SubRegIndices = [sub_entire, sub_firq, sub_half, sub_irq, sub_negative, sub_zero, sub_overflow, sub_carry];
  let CoveredBySubRegs = true;
}


def IX : MC6809Reg16<1,  "x" >;
def IY : MC6809Reg16<2,  "y" >;
def SU : MC6809Reg16<3,  "u" >;
def SS : MC6809Reg16<4,  "s" >;

def PC : MC6809Reg16<5,  "pc">;

def AV : MC6809Reg16<7,  "v" >;

defm AA : MC6809Reg8<8,   "a">;
defm AB : MC6809Reg8<9,   "b">;

def AD : MC6809Reg16<0,  "d", [AB, AA], [sub_lo_byte, sub_hi_byte]>;

def DP : MC6809Reg8<11,  "dp">;

def A0 : MC6809Reg16<12, "0" >;

defm AE : MC6809Reg8<14,  "e">;
defm AF : MC6809Reg8<15,  "f">;

def AW : MC6809Reg16<6,  "w", [AF, AE], [sub_lo_byte, sub_hi_byte]>;

def AQ : MC6809Reg32<16, "q", [AW, AD], [sub_lo_word, sub_hi_word]>;

def MD : MC6809Reg8<17,  "md">;

// The stack-backed spill pseudo-registers (SPILL_A0..7, SPILL_B0..7,
// SPILL_D0..7, SPILL_X0..3, SPILL_Q0..31 with their half-word and byte
// sub-regs) are GONE: every class spills through the stock frame-index
// path (storeRegToStackSlot / loadRegFromStackSlot), with
// foldMemoryOperandImpl's _Mem folds keeping the single-register classes
// allocatable when two values meet at one instruction.

//===----------------------------------------------------------------------===//
//  Bug #302 follow-up: PHANTOM_CARRY physregs
//===----------------------------------------------------------------------===//
//
// These eight 1-bit "registers" exist purely as regalloc bookkeeping slots
// for the scheduling-phantom carry-out vregs of the SetCarry / SetCarryUse /
// SetOverflow / SetOverflowUse pseudos.  The phantoms have no runtime
// value — CC.C / CC.V is the source of truth (see the long comment on
// MC6809ArithmeticBaseCarry in MC6809InstrFamilies.td).  Their job is to
// thread a producer→consumer SSA dependency that survives regalloc and
// scheduling without competing with real byte / word allocation.
//
// PHANTOM_CARRY_0..PHANTOM_CARRY_7 satisfy that need without competing with
// any real allocation pool: no sub-register relationships, no hardware
// encoding, no MC instruction takes them as an operand.  AsmPrinter ignores
// implicit operands at MCInst conversion, so they never appear in the
// emitted asm.  Eight phantoms is generous — even an i64 add chain has at
// most 1 live at a time per chain (consecutive *CarryUse_* consume their
// predecessor's phantom and produce a new one); multiple chains in the same
// BB push to 2-3.
//
// CRITICAL INVARIANT — DO NOT REFERENCE PHANTOM_CARRY IN ANY (ins ...) OR
// (outs ...) LIST OF ANY PSEUDO / INSTRUCTION.  The only valid way for a
// phantom to enter MIR is via the selector's hand-written
// `.addDef(CarryOut, RegState::ImplicitDefine)` / `.addUse(Carry,
// RegState::Implicit)` calls in selectAddO / selectSubO / selectAddE /
// selectSubE.  An explicit operand of PHANTOM_CARRY class would cause
// PHANTOM_CARRY to leak into emission paths.  The late
// MC6809PhantomCarryGuard pass (added in this same change) walks every MI
// before final lowering and aborts compilation with a clear diagnostic
// if any explicit PHANTOM_CARRY operand is seen — see
// MC6809PhantomCarryGuard.cpp.
//
// DWARF / HWEncoding numbers in 0x3F00..0x3F07 -- well outside the real
// MC6809 register encoding space (0..63) and the spill / imaginary ranges
// (0x1000+, 0x2000+).  HWEncoding must fit in bits<16>.  Any accidental
// debug-info or MC-emission reference will land on a deliberately
// suggestive number (0x3FXX -- a "phantom" looking address) that is easy
// to spot in disassembly or DWARF dumps.
foreach I = 0...7 in {
  def PHANTOM_CARRY_#I : MC6809Reg1<!add(0x3F00, I), "phantom_carry_"#I>;
}

//===----------------------------------------------------------------------===//
//  Register classes
//===----------------------------------------------------------------------===//

class MC6809RegClass<list<ValueType> regTypes, int alignment, dag regList>
    : RegisterClass<"MC6809", regTypes, alignment, regList> {
        let Namespace = "MC6809";
}

class MC6809Reg1Class<dag RegList>: MC6809RegClass<[i1], 1, RegList> {
  // The register is 1-bit wide, but takes 8 bits to spill.
  let RegInfos = RegInfoByHwMode<[DefaultMode], [RegInfo<1, 8, 8>]>;
}
class MC6809Reg2Class<dag RegList>  : MC6809RegClass<[i2], 8, RegList>;
class MC6809Reg4Class<dag RegList>  : MC6809RegClass<[i4], 8, RegList>;
class MC6809Reg8Class<dag RegList>  : MC6809RegClass<[i8], 8, RegList>;
class MC6809Reg16Class<dag RegList> : MC6809RegClass<[i16], 8, RegList>;
class MC6809Reg32Class<dag RegList> : MC6809RegClass<[i32], 8, RegList>;
class MC6809Reg64Class<dag RegList> : MC6809RegClass<[i64], 8, RegList>;

// Register grouping classes.
def CARRY : MC6809Reg1Class<(add C)>;
// Single-flag class for the inline-asm 'v' (CC.V overflow) constraint, the
// overflow analogue of CARRY for the 'c' constraint.
def OVFLAG : MC6809Reg1Class<(add V)>;

// Bug #302 follow-up: PHANTOM_CARRY is the regalloc home for the
// scheduling-phantom carry-out vregs produced by SetCarry / SetCarryUse /
// SetOverflow / SetOverflowUse.  See the long comment block at the
// PHANTOM_CARRY_0..7 physreg definitions above for the invariant.
def PHANTOM_CARRY : MC6809Reg1Class<(sequence "PHANTOM_CARRY_%u", 0, 7)>;
// ACC8 — the byte allocation pool.  i1 values share this class with
// i8: the byte's LSB carries the boolean.  Includes the four real
// byte registers and 16 RC* imaginary (DP) registers, all addressable
// as a single byte.  The SPILL_A*/SPILL_B* stack-backed pseudo-registers
// were removed from byte allocation: byte spills now go through the
// stock frame-index spiller, whose second-operand reloads fold into the
// _Mem forms via foldMemoryOperandImpl (see memFoldSibling).
def ACC8 : MC6809Reg8Class<(add AB, AA, AF, AE,
                                (sequence "RC%u",      0, 15))> {
    // AE and AF are HD6309-only. Exclude from allocation on 6809.
    let AltOrders = [(add AB, AA,
                         (sequence "RC%u",      0, 15))];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
// Compare-operand byte pool: real accumulators only, no RC direct-page
// bytes. RC is admitted to ACC8 for one shape only — a byte defined
// alongside a live i32 in AQ (an i32 carry-out materialised to a byte,
// e.g. lround) needs an AQ-aliasing-free direct-page home; that byte is
// always TESTed, never COMPAREd. So the compare pools have no reason to
// admit RC, and admitting it hurt: under byte pressure the allocator
// parked a hot compare operand in RC (a per-access ldb/stage) where the
// real-register pool would have kept it live.
def ACC8_CMP : MC6809Reg8Class<(add AB, AA, AF, AE)> {
    let AltOrders = [(add AB, AA)];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
def ACC8_AB : MC6809Reg8Class<(add AB, AA)>;

// ACC8 minus the HD6309 E/F accumulators, but keeping the spill slots and
// direct-page imaginary regs so allocation never fails. Used as the
// tied-accumulator class for the byte indexed/pull arith/logical pseudos
// (_Mem / _MemIndirect / _Pull), which have no `ANDF`/`ADDE n,x` style
// encoding — if regalloc parked their accumulator in E/F the post-RA expander
// (expandIdxImm) hit an llvm_unreachable. Excluding E/F here keeps those ops
// off the unencodable registers on both CPUs (on 6809 this already matches
// ACC8's AltOrder, so 6809 codegen is unchanged).
def ACC8_AB_SP : MC6809Reg8Class<(add AB, AA,
                                     (sequence "RC%u",      0, 15))>;

// Bug #217: per-half byte register class for the _Reg variants of
// MC6809Arithmetic{Base,Carry,SetCarryUse,SetOverflowUse}. The byte
// _Reg expansion via emit6809RegByteFromMem clobbers ONE accumulator
// half (whichever half the LHS lands in). Constraining dst to the
// B-half resources (AB + SPILL_B*) means regalloc commits up-front to
// AB; the pseudo's Defs can then declare ONLY $ab as clobbered,
// instead of the union $aa+$ab. Halves the over-claim and recovers
// the regalloc pressure that bug #217 reported as catastrophic across
// non-LTO opt levels.
//
// AB is the historical default for byte arithmetic on the 6809 (most
// of the byte ALU ops have B variants). Forcing dst to B is a
// reasonable convention: regalloc inserts COPY $ab, $aa when
// upstream values live in A.
//
// Bug #271 cat-1 residual (2026-05-11): the per-half "Bonly"/"Aonly"
// classes formerly existed as ACC8_Bonly / ACC8_Aonly with contents
// identical to ABc / AAc above. TableGen treated them as DISTINCT
// classes despite identical register sets, so an operand declared
// ACC8_Bonly could not be satisfied by an ABc-class vreg (verifier
// flagged "Expected ACC8_Bonly, got ABc"). The duplicate defs were
// removed; the pseudos that previously referenced them now use ABc
// (B-half) and AAc (A-half). This eliminates the verifier flag with
// zero codegen change — the register sets are byte-identical.
def BIT8 : MC6809Reg8Class<(add AA, AB, CC)>; // Valid registers for BAND, BEOR, BOR, BIAND, BIEOR, BIOR, LDBT, STBT
// Enable fine-grained pressure tracking on constrained i16 classes.
// On basic 6809, ACC16 has 1 real physreg (AD) + 8 stack-backed spills +
// 4 imaginary = 13 total, but the D-only ALU constraint means instructions
// often need the ADc intersection (9 slots). Fine-grained tracking lets
// the scheduler see per-class pressure and avoid peaks that blow regalloc.
// Follows the MOS pattern (MOSRegisterInfo.td:165-188).
//
// History: Bug #161 round 14 grew the ACC16 pool from 14 → 74 slots
// by adding RS4..RS63 (60 imaginary 16-bit regs, no byte sub-regs)
// to absorb partial-product pressure from i64-multiply-heavy
// functions. Bug-#207 follow-up audit (2026-05-02) showed
// llvm-readelf across 564 .o files (full picolibc HD6309 -Oz
// libc.a + semihost + every test binary) found ZERO references to
// `__rs4`..`__rs63`. The fused-cmp-branch (#203) and TFM work
// since round 14 cut pressure enough that RS4+ have become pure
// pool padding. Trimmed back to RS0..RS3 to reclaim ~120 bytes of
// direct-page address space for user `__attribute__((directpage))`.
let isPressureFineGrained = true in
// The SPILL_D* stack-backed pseudo-registers were removed from i16
// allocation with the stock-frame-index spill migration: i16 spills go
// through the ordinary spiller and spilled second sources fold into the
// _Mem forms (see memFoldSibling). The RS direct-page imaginaries remain
// (a separate, retained mechanism).
def ACC16 : MC6809Reg16Class<(add AD, AW,
                                  (sequence "RS%u",      0, 3))> {
    let AltOrders = [(add AD,
                          (sequence "RS%u",      0, 3))];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
// ACC16_D removed 2026-04-16 as the Layer-2 regalloc experiment (bug #118).
// The coalescer was tightening i16 vregs to this 1-register class when
// eliminating `COPY $ad`, producing a regalloc bottleneck on any function
// with multiple simultaneously-live i16 values. With ACC16_D gone, the
// smallest subset of ACC16 containing AD is now ADc (13 regs: AD +
// SPILL_D0..7 + RS0..3) — the coalescer can still tighten but the result
// is no longer a 1-register class. See project_regalloc_two_layer_bottleneck.md.
// The single in-source user (HD6309 BitTest_i16) switched to ACC16;
// frame lowering switched to direct physreg $ad copies.
// The SPILL_Q* stack-backed pseudo-registers were removed from i32
// allocation (stock-spilling migration): i32 values spill through ordinary
// frame-index slots, with foldMemoryOperandImpl reading a spilled second
// source in place (Add_i32_Mem and friends) and folding a spilled
// Extract16 source to a one-word load — the escape valve is no longer
// needed to make the single-register {AQ} pool allocatable.
def ACC32 : MC6809Reg32Class<(add AQ)> {
    let AltOrders = [(add)];
    let AltOrderSelect = [{return MF.getSubtarget<MC6809Subtarget>().has6309() ? 0 : 1;}];
}
// ACC64: non-allocatable class with a dummy member register. Exists solely
// to set the ACCUM register bank's maximum size to 64 bits (for i64
// support). No physical 64-bit registers exist on MC6809 — i64
// decomposes to 4×i16 via the legaliser.
def PHANTOM64 : MC6809Reg<0xFFFF, "">;
let isAllocatable = false in
def ACC64 : MC6809Reg64Class<(add PHANTOM64)>;
def REGLIST8 : MC6809Reg8Class<(add CC, AA, AB, DP)>; // Registers which can be pushed/popped
def REGLIST16 : MC6809Reg16Class<(add AD, AW, IX, IY, SU, SS, PC)>; // Registers which can be pushed/popped
def REGPAIR8 : MC6809Reg8Class<(add AA, AB, CC, DP, A0, AE, AF)>; // Registers for EXG, TFR
def REGPAIR16 : MC6809Reg16Class<(add AD, IX, IY, SU, SS, PC, AW, AV)>; // Registers for EXG, TFR
// Index registers TFM codegen can allocate. The TFM *hardware* accepts
// D/X/Y/U/S, but for GISel-emitted block copies only IX/IY are usable: D is in
// the ACCUM bank (a cross-bank pointer vreg can't be assigned a single bank) and
// U/S are reserved. Restricting to {IX,IY} makes this a subclass of INDEX16, so
// the pointer operands map cleanly to INDEXRegBank and regalloc picks either
// IX/IY assignment (the postbyte records whichever it got -- no forced swap).
// Hand-written asm keeps the full D/X/Y/U/S set via TFM0pp's anyregister.
def REGTFM : MC6809Reg16Class<(add IX, IY)>; // Index regs TFM codegen allocates

// Pointer and stack registers
let isPressureFineGrained = true in
// The SPILL_X* stack-backed pseudo-registers were removed from INDEX16
// allocation with the stock-frame-index spill migration: pointer spills go
// through the ordinary spiller, and spilled second sources of the index
// reg-reg compares fold into the _Mem forms (see memFoldSibling).
def INDEX16 : MC6809Reg16Class<(add IX, IY,
                                    SU, SS)>;

// Stack registers for internal push/pull operations.
def CCond : MC6809Reg8Class<(add CC)>;

def Imag8 : MC6809Reg8Class<(add (sequence "RC%u", 0, !add(MaxImag8Regs, -1)))>;
// Imag16 enumerates the existing RS regs only. Trimmed to RS0..RS3
// (was RS0..RS127) per the bug-#207 follow-up audit; see ACC16 above.
def Imag16 : MC6809Reg16Class<(add RS0, RS1, RS2, RS3)>;
def Imag32 : MC6809Reg32Class<(sequence "RW%u", 0, !add(MaxImag32Regs, -1))>;

def AllCCFlag : MC6809Reg1Class<(add E, F, H, I, N, Z, V, C)>;
def CCFlag : MC6809Reg1Class<(add N, Z, V, C)>;
def ArithFlag : MC6809Reg2Class<(add NZ)>;
def AllArithFlag : MC6809Reg2Class<(add NZVC)>;

let isAllocatable = false in {
  def MDCond : MC6809Reg8Class<(add MD)>;
  def DtPage : MC6809Reg8Class<(add DP)>;
}

// Single register classes.
// AAc/ABc are genuine one-register classes: an instruction with two
// simultaneously-live operands in one of them is allocatable only because
// foldMemoryOperandImpl can fold the spilled second source into the op's
// _Mem sibling (read from the frame slot in place). See memFoldSibling in
// MC6809InstrInfo.cpp.
    def AAc : MC6809Reg8Class<(add AA)>;
    def ABc : MC6809Reg8Class<(add AB)>;
    let isPressureFineGrained = true in
    def ADc : MC6809Reg16Class<(add AD,
                                    (sequence "RS%u",      0, 3))>;
    def AEc : MC6809Reg8Class<(add AE)>;
    def AFc : MC6809Reg8Class<(add AF)>;
    def AWc : MC6809Reg16Class<(add AW)>;
    // AQc — i32 allocation pool: just the real AQ (the SPILL_Q escape
    // registers are retired; i32 spills through stock frame-index slots,
    // with the _Mem folds keeping two-source ops allocatable).
    // Accumulator Defs are scoped to only the half they actually clobber
    // (LDD → AD, LDW → AW) so long-lived i32 values can sit in AQ across
    // half-width ops.
    def AQc : MC6809Reg32Class<(add AQ)>;
    let isPressureFineGrained = true in
    def IXc : MC6809Reg16Class<(add IX)>;
    let isPressureFineGrained = true in
    def IYc : MC6809Reg16Class<(add IY)>;
    def SUc : MC6809Reg16Class<(add SU)>;
    def SSc : MC6809Reg16Class<(add SS)>;
    def PCc : MC6809Reg16Class<(add PC)>;
    def CCc : MC6809Reg8Class<(add CC)>;
    def NZVCc : MC6809Reg8Class<(add NZVC)>;
    def NZc : MC6809Reg8Class<(add NZ)>;
    def Nc : MC6809Reg1Class<(add N)>;
    def Zc : MC6809Reg1Class<(add Z)>;
    def Vc : MC6809Reg1Class<(add V)>;
    def Cc : MC6809Reg1Class<(add C)>;

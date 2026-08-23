This is an LLVM fork adding backends for the Motorola 6809 and Hitachi
6309. Almost all of the work is in `llvm/lib/Target/MC6809/`, with hooks in
clang's driver, lld and compiler-rt. When reviewing a change here, the
hazards below are the ones that have actually produced miscompiles in this
tree, and they are worth more attention than generic LLVM review.

**Accumulator sub-registers overlap.** `AA` and `AB` are halves of `AD`;
`AE` and `AF` are halves of `AW`; all of them live inside `AQ`. A byte
operation therefore defines only part of a register, and code that treats
such a def as covering the whole thing makes liveness and copy propagation
unsound. Ask of any new definition: which sub-registers does this really
write, and which does it leave alive?

**A pseudo instruction must declare everything its expansion touches.**
Every register and condition-code bit the expansion clobbers belongs in the
pseudo's `Defs`, or the register allocator will place a live value where
the expansion will destroy it. Note that `MI.setDesc` does not carry
implicit operands across — they must be re-added by hand.

**Condition codes are frequently implicit and easily wrong.** Whether an
instruction sets Z, and whether a compare can be elided because a previous
operation already set the flags, is a recurring source of wrong branches.
`LEAX` in particular does not set Z, which has been assumed more than once.
Changes that remove a compare or a `TST` need to be argued from the
architecture manual, not from what the tests happen to accept.

**Inline-asm register names go through `getRegAsmName`.** Clobber lists are
matched against those names; if a register's assembly spelling and its
TableGen record name diverge, clobbers are silently ignored and the asm
corrupts its caller's registers.

**There are very few registers.** Changes that increase 16-bit pressure
tend to produce spills rather than better code, and a change that looks
like an optimisation can lose to the staging it forces around it. Size and
cycle counts are measured, not reasoned about.

**Position independence is the default.** Bare-metal, OS-9 and DECB targets
all rely on it; an absolute address baked into a code sequence will work in
a test and fail in an OS-9 module, which is relocated at load time.

**Instruction selection is GlobalISel only.** There is no SelectionDAG
fallback path, so a pattern that is not selected is a hard failure rather
than a slow path.

**Debug information deserves the same scrutiny as code.** Control-flow
changes around branches and calls are the usual way DWARF line and location
data becomes wrong, and this backend's debug output is already an area with
known gaps.

Correctness comes before code size in every case. A change to the backend
is validated by running the benchmark suite across all optimisation levels
and comparing cycle counts and output bytes — the lit tests alone do not
establish that codegen is unchanged.

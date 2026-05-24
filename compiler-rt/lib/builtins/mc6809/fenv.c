/* Bug #289 (2026-05-24): C99 <fenv.h> for MC6809 via the MC6839 FPCBs.
 *
 * Picolibc on MC6809 previously fell through to its softfloat <fenv.h>
 * stub (libc/include/machine/fenv-softfloat.h) — every function silently
 * succeeded but tracked nothing.  This file replaces the stub with real
 * functionality backed by the MC6839 ROM's Floating-Point Control
 * Blocks (FPCBs), which already maintain:
 *
 *  - sticky exception status (FPCB[FP_STAT])  — accumulated across all
 *    FP libcalls, never auto-cleared by the ROM
 *  - rounding mode + precision (FPCB[FP_CTRL] bits 7:5 / 2:1)
 *
 * Four per-precision FPCBs live in compiler-rt/lib/builtins/mc6809/
 * fp_data.S:
 *   __mc6839_fpcb_f32        — single, round-to-nearest
 *   __mc6839_fpcb_f64        — double, round-to-nearest
 *   __mc6839_fpcb_f32_trunc  — single, round-toward-zero (used by (int)x)
 *   __mc6839_fpcb_f64_trunc  — double, round-toward-zero
 *
 * Critical invariant: FPCB[FP_CTRL] is set ONCE at .data load time and
 * the libcalls never rewrite it per call (only the ROM address is
 * passed in D).  Therefore `fesetround()` modifying FPCB[FP_CTRL] takes
 * effect on subsequent libcalls without any libcall-side changes.
 *
 * The `_trunc` FPCBs intentionally remain at RZ — because `(int)x` is
 * truncation by definition per C99 regardless of FP rounding mode.  A
 * `fesetround(FE_DOWNWARD)` followed by `(int)x` still uses RZ, which
 * is correct.
 *
 * Out of scope (separate bugs if/when wanted):
 *   - Exception trapping (FPCB[FP_TRAPEN] / FPCB[FP_TRAPVEC]) — needs
 *     a trap handler installer
 *   - GNU extensions feenableexcept / fedisableexcept / fegetexcept
 *
 * Co-located with this file: fp_trampoline.S, which provides the
 * non-standard `__mc6839_set_rom_base(void*)` consumer (the
 * __mc6839_fpreg_ptr / __mc6839_fpstak_ptr globals it updates).
 */

#include <stdint.h>

/* FE_* constants — match the picolibc machine/fenv.h header.  The
 * exception bits match the FPCB[FP_STAT] hardware layout (see
 * compiler-rt/lib/builtins/mc6809/mc6839_abi.inc and the canonical
 * nfp09-abi.s status-byte block) so all fenv ops on exception masks
 * are trivial AND/OR on the FPCB byte. */
#define FE_INVALID    0x01  /* FP_STAT_IOP */
#define FE_OVERFLOW   0x02  /* FP_STAT_OVF */
#define FE_UNDERFLOW  0x04  /* FP_STAT_UNF */
#define FE_DIVBYZERO  0x08  /* FP_STAT_DZ  */
#define FE_INEXACT    0x40  /* FP_STAT_INX */
#define FE_ALL_EXCEPT 0x4F  /* IOP|OVF|UNF|DZ|INX (no UN, IOV — non-C99) */

/* Rounding modes — match FPCB[FP_CTRL] bits 2:1 hardware encoding so
 * fesetround / fegetround are trivial mask/OR operations. */
#define FE_TONEAREST  0x00
#define FE_TOWARDZERO 0x02
#define FE_UPWARD     0x04
#define FE_DOWNWARD   0x06

/* Bitmasks within FPCB[FP_CTRL] (other bits: precision in 7:5, NRM in
 * bit 3, closure in bit 0).  fesetround must preserve them. */
#define FP_CTRL_ROUND_MASK 0x06  /* bits 2:1 */

typedef unsigned char fexcept_t;

typedef struct {
    unsigned char round_mode;    /* FPCB[FP_CTRL] bits 2:1, in the
                                  * hardware-bit positions (0/2/4/6) */
    unsigned char except_flags;  /* FE_* mask */
} fenv_t;

/* Default FP environment — round-to-nearest, no pending exceptions.
 * picolibc's machine/fenv.h exposes &__fe_dfl_env as FE_DFL_ENV. */
const fenv_t __fe_dfl_env = { FE_TONEAREST, 0 };

/* The four FPCBs live in compiler-rt/lib/builtins/mc6809/fp_data.S.
 * Each is 6 bytes; we touch bytes [FP_CTRL=0] and [FP_STAT=2]. */
extern unsigned char __mc6839_fpcb_f32[6];
extern unsigned char __mc6839_fpcb_f64[6];
extern unsigned char __mc6839_fpcb_f32_trunc[6];
extern unsigned char __mc6839_fpcb_f64_trunc[6];

/* The runtime entry-point pointers live in fp_trampoline.S.
 * `__mc6839_set_rom_base` updates all three. */
extern void *__mc6839_rom_base;
extern void *__mc6839_fpreg_ptr;
extern void *__mc6839_fpstak_ptr;

/* MC6839 ROM entry-point offsets relative to ROM base (verified from
 * Float09.bin / romsrc per mc6839_rom.S header). */
#define MC6839_FPREG_OFFSET  0x3D
#define MC6839_FPSTAK_OFFSET 0x3F

/* ------------------------------------------------------------------ */
/* Internal helpers                                                   */
/* ------------------------------------------------------------------ */

/* OR of the four FPCBs' FP_STAT bytes.  Captures all exceptions
 * accumulated across any precision / any normal-vs-trunc op since the
 * last feclearexcept. */
static unsigned char aggregate_status(void) {
    return __mc6839_fpcb_f32[2]
         | __mc6839_fpcb_f64[2]
         | __mc6839_fpcb_f32_trunc[2]
         | __mc6839_fpcb_f64_trunc[2];
}

/* Clear `mask` bits in FP_STAT of all 4 FPCBs (so subsequent
 * aggregate_status sees them cleared). */
static void clear_status_bits(unsigned char mask) {
    unsigned char keep = (unsigned char)~mask;
    __mc6839_fpcb_f32[2]       &= keep;
    __mc6839_fpcb_f64[2]       &= keep;
    __mc6839_fpcb_f32_trunc[2] &= keep;
    __mc6839_fpcb_f64_trunc[2] &= keep;
}

/* Set `mask` bits in FP_STAT of every FPCB (so a subsequent test
 * sees them set regardless of which precision the consumer queries). */
static void set_status_bits(unsigned char mask) {
    __mc6839_fpcb_f32[2]       |= mask;
    __mc6839_fpcb_f64[2]       |= mask;
    __mc6839_fpcb_f32_trunc[2] |= mask;
    __mc6839_fpcb_f64_trunc[2] |= mask;
}

/* ------------------------------------------------------------------ */
/* C99 fenv exception manipulation                                    */
/* ------------------------------------------------------------------ */

int feclearexcept(int excepts) {
    clear_status_bits((unsigned char)(excepts & FE_ALL_EXCEPT));
    return 0;
}

int fetestexcept(int excepts) {
    return aggregate_status() & (unsigned char)(excepts & FE_ALL_EXCEPT);
}

int fegetexceptflag(fexcept_t *flagp, int excepts) {
    *flagp = aggregate_status() & (unsigned char)(excepts & FE_ALL_EXCEPT);
    return 0;
}

int fesetexceptflag(const fexcept_t *flagp, int excepts) {
    unsigned char mask = (unsigned char)(excepts & FE_ALL_EXCEPT);
    unsigned char val  = *flagp & mask;
    /* Clear the masked bits everywhere, then re-set them from the
     * saved snapshot.  Symmetric across all 4 FPCBs. */
    clear_status_bits(mask);
    set_status_bits(val);
    return 0;
}

int feraiseexcept(int excepts) {
    set_status_bits((unsigned char)(excepts & FE_ALL_EXCEPT));
    return 0;
}

/* ------------------------------------------------------------------ */
/* C99 fenv rounding-mode control                                     */
/* ------------------------------------------------------------------ */

int fegetround(void) {
    return __mc6839_fpcb_f32[0] & FP_CTRL_ROUND_MASK;
}

int fesetround(int round) {
    unsigned char r = (unsigned char)(round & FP_CTRL_ROUND_MASK);
    if (r != FE_TONEAREST && r != FE_TOWARDZERO &&
        r != FE_UPWARD    && r != FE_DOWNWARD)
        return -1;   /* C99 says non-zero on failure */
    /* Update FP_CTRL bits 2:1 in the f32 and f64 FPCBs; leave the
     * _trunc variants alone (they must always RZ — (int)x is
     * truncation regardless of fenv state). */
    unsigned char ctrl_keep = (unsigned char)~FP_CTRL_ROUND_MASK;
    __mc6839_fpcb_f32[0] = (__mc6839_fpcb_f32[0] & ctrl_keep) | r;
    __mc6839_fpcb_f64[0] = (__mc6839_fpcb_f64[0] & ctrl_keep) | r;
    return 0;
}

/* ------------------------------------------------------------------ */
/* C99 fenv environment save/restore                                  */
/* ------------------------------------------------------------------ */

int fegetenv(fenv_t *envp) {
    envp->round_mode   = __mc6839_fpcb_f32[0] & FP_CTRL_ROUND_MASK;
    envp->except_flags = aggregate_status();
    return 0;
}

int fesetenv(const fenv_t *envp) {
    fesetround(envp->round_mode);
    clear_status_bits(FE_ALL_EXCEPT);
    set_status_bits(envp->except_flags & FE_ALL_EXCEPT);
    return 0;
}

int feupdateenv(const fenv_t *envp) {
    unsigned char held = aggregate_status();
    fesetenv(envp);
    set_status_bits(held);
    return 0;
}

int feholdexcept(fenv_t *envp) {
    fegetenv(envp);
    clear_status_bits(FE_ALL_EXCEPT);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Non-standard: MC6839 ROM runtime entry-point setup                 */
/* ------------------------------------------------------------------ */

/* Update the three pointer globals declared in fp_trampoline.S to
 * reflect a new ROM base address.  For bare-metal, the linker has
 * already initialised these to `__mc6839_rom_start + offset` and no
 * call is needed.  For OS-9, crt0 calls this after F$Link("Float09")
 * returns the dynamic load address. */
void __mc6839_set_rom_base(void *base) {
    char *p = (char *)base;
    __mc6839_rom_base   = p;
    __mc6839_fpreg_ptr  = p + MC6839_FPREG_OFFSET;
    __mc6839_fpstak_ptr = p + MC6839_FPSTAK_OFFSET;
}

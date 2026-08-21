# Third-party code in an MC6809 toolchain

One piece of this toolchain was not written for it, and travels in every
bundle.  This says what it is, where it came from and on what footing, so
that nobody has to work it out from a hex dump.

## The MC6839 floating-point ROM

**File**: `Float09.bin`, 8192 bytes.
**SHA-256**: `c5332caac4cda960d240b059b6fccc9f450140ce94a0ebf5893c1f8f0fb83813`.
**Written**: for Motorola by Joel Boney, 1980; the image carries
`COPYRIGHT (C) MOTOROLA 1982` and is revision `$82`.

All floating-point arithmetic on this target is done by this ROM.  The
compiler emits calls to small wrappers — `compiler-rt/lib/builtins/mc6809/
fp_*.S` — which hand their arguments to it through the register interface at
`ROM_BASE+$3D`.  Nothing here reimplements IEEE 754.

### Where it appears in a bundle

The same 8192 bytes, unaltered, in two places, because the two targets reach
it differently:

* `lib/clang/<ver>/lib/mc6809-unknown-unknown/libclang_rt.builtins.a` — the
  image is `.incbin`-ed into an object (`mc6839_rom.S`) and linked into a
  program that does floating point.  A program that does none carries none:
  the reference is weak.  DECB links the same library.
* `lib/clang/<ver>/lib/mc6809-unknown-os9/FPO9` — a straight copy of the file.
  Motorola shipped the ROM *as* an OS-9 module (it begins `87 CD`), so no
  wrapping is needed; an OS-9 program links it at run time by name, and it
  has to be present on the target machine.

### Footing

Rich Kottke's 1989 documentation for the ROM — preserved as `doc/kottke.txt`
in the reconstruction repository — states on its title page:

>                                 F P O 9
>
>                  Floating point routines for the 6809
>
>                Written for Motorola by Joel Boney, 1980
>           **Released into the public domain by Motorola in 1988**
>      Docs and apps for Tandy Color Computer by Rich Kottke, 1989

and explains how he came by it: *"The 6839 floating point ROM was never
available from Motorola, but I recently noticed that the code from the ROM
was available on Motorola's PD software BBS so I downloaded it."*  He closes
by thanking "Motorola for releasing it into the Public Domain".

That is the basis on which it is included here: a contemporaneous account of
a public-domain release by the copyright holder.  It is an account, not a
licence document, and this note makes no legal claim beyond repeating it.

### Chain of custody

Motorola's public-domain BBS (by 1988) → Rich Kottke's posting, archived on
Usenet in 1994 → [github.com/brouhaha/fp09](https://github.com/brouhaha/fp09)
→ a fork carrying a byte-identical reconstruction from the original Motorola
sources → `compiler-rt/lib/builtins/mc6809/Float09.bin` in this tree.

The reconstruction matters for confidence rather than for provenance: the
original `.sa` sources, adapted for `lwasm`, assemble to an image identical
to this one, which is a strong check that the file is the ROM and nothing
else.  Five differences between the published sources and the shipped ROM
were found and corrected in doing it — a truncated `VALID` routine, a
revision and copyright year, `FSQRT` using `SNORM` and `ASRA`, a module
missing from the build manifest, and zero-offset indexed encodings.

### If you would rather not ship it

Nothing else in the toolchain depends on it.  A build without floating point
never links it, and the default libraries are integer-only, so a bundle whose
users do not use `-mlibc=float` never puts it in a program.  Replacing it
would mean building compiler-rt's portable C soft-float for this target
instead — which is a real piece of work, and slower and larger on a machine
where both matter.

; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809-unknown-os9 -relocation-model=pic -O2 %s -o - | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809-unknown-os9 -relocation-model=pic -O0 %s -o - | FileCheck %s --check-prefix=INDEXED

target triple = "mc6809-unknown-os9"

; Direct-page (address-space 1) objects on OS-9 live in the first page of the
; process data area, which the kernel sets DP to (DP = U >> 8). They are reached
; by direct addressing whose one-byte operand is the object's data-area offset,
; known only to the linker: the mc6809_os9_data8 relocation.
@dp_flags = internal addrspace(1) global i8 3, align 1
@dp_count = internal addrspace(1) global i16 0, align 1

define i8 @poke(i8 %x) {
entry:
  %f = load i8, ptr addrspace(1) @dp_flags, align 1
  %n = xor i8 %f, %x
  store i8 %n, ptr addrspace(1) @dp_flags, align 1
  ret i8 %n
}

; CHECK-LABEL: poke:
; CHECK:     lda <mc6809_os9_data8(dp_flags)
; CHECK:     stb <mc6809_os9_data8(dp_flags)
; CHECK-NOT: __dp_base_addr
; CHECK:     rts

; A run-time index into a direct-page object needs the page base in an index
; register: on OS-9 that is U itself, never the bare-metal __dp_base_addr.
define i8 @at(i8 %i) {
entry:
  %p = getelementptr inbounds i8, ptr addrspace(1) @dp_count, i8 %i
  %v = load i8, ptr addrspace(1) %p, align 1
  ret i8 %v
}

; The page offset is an 8-bit immediate (the same relocation), the base is U.
; INDEXED-LABEL: at:
; INDEXED-NOT: __dp_base_addr
; INDEXED: tfr u,x
; INDEXED: lda #mc6809_os9_data8(dp_count)
; INDEXED: ldb d,x
; INDEXED-NOT: __dp_base_addr
; INDEXED: rts
; CHECK-LABEL: at:
; CHECK-NOT: __dp_base_addr
; CHECK:     lda #mc6809_os9_data8(dp_count)
; CHECK:     ldb d,u
; CHECK-NOT: __dp_base_addr
; CHECK:     rts

; The objects themselves go to the direct-page sections the OS-9 linker
; script images into the first page of the data area.
; CHECK: .section .dp.data
; CHECK: dp_flags:
; CHECK: .section .dp.bss
; CHECK: dp_count:

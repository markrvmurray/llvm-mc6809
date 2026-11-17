; Interrupt handlers and the code they interrupt. A handler returns with RTI
; and must leave the direct-page registers as it found them: they are scratch
; to every other function, so the interrupted code may be using them.
; clang -O2 output for:
;
;   #define ACIA_CTRL (*(volatile unsigned char *)0xFFC4)
;   #define ACIA_DATA (*(volatile unsigned char *)0xFFC5)
;   volatile unsigned char irqs, swis;
;   unsigned long irq_sum, swi_sum;
;   volatile unsigned long seed;
;   void mix(unsigned long *p) { *p = *p + seed + 0x00000101UL; }  /* noinline */
;   __attribute__((interrupt)) void irq_handler(void) {
;     ACIA_CTRL = 0x15;                  /* transmit interrupt off */
;     (void)ACIA_DATA;                   /* and the request cleared */
;     irq_sum = irq_sum + seed + 0x01020304UL;
;     irqs++;
;   }
;   __attribute__((interrupt)) void swi_handler(void) { mix(&swi_sum); swis++; }
;   unsigned long sum_across_irq(unsigned long a, unsigned long b, unsigned long c) {
;     unsigned long t = a + b;
;     ACIA_CTRL = 0x35;                  /* transmit interrupt on: IRQ here */
;     t += c;
;     return t;
;   }
;   unsigned long sum_across_swi(... the same, with __asm__("swi") ...)

target triple = "mc6809-unknown-unknown"

@seed = global i32 0, align 1
@irq_sum = global i32 0, align 1
@irqs = global i8 0, align 1
@swi_sum = global i32 0, align 1
@swis = global i8 0, align 1

define void @mix(ptr noundef %p) noinline {
entry:
  %0 = load i32, ptr %p, align 1
  %1 = load volatile i32, ptr @seed, align 1
  %add = add i32 %0, 257
  %add1 = add i32 %add, %1
  store i32 %add1, ptr %p, align 1
  ret void
}

define void @irq_handler() "interrupt" {
entry:
  store volatile i8 21, ptr inttoptr (i16 -60 to ptr), align 4
  %0 = load volatile i8, ptr inttoptr (i16 -59 to ptr), align 1
  %1 = load i32, ptr @irq_sum, align 1
  %2 = load volatile i32, ptr @seed, align 1
  %add = add i32 %1, 16909060
  %add1 = add i32 %add, %2
  store i32 %add1, ptr @irq_sum, align 1
  %3 = load volatile i8, ptr @irqs, align 1
  %inc = add i8 %3, 1
  store volatile i8 %inc, ptr @irqs, align 1
  ret void
}

define void @swi_handler() "interrupt" {
entry:
  tail call void @mix(ptr noundef nonnull @swi_sum)
  %0 = load volatile i8, ptr @swis, align 1
  %inc = add i8 %0, 1
  store volatile i8 %inc, ptr @swis, align 1
  ret void
}

define noundef i32 @sum_across_irq(i32 noundef %a, i32 noundef %b, i32 noundef %c) {
entry:
  %add = add i32 %b, %a
  store volatile i8 53, ptr inttoptr (i16 -60 to ptr), align 4
  %add1 = add i32 %add, %c
  ret i32 %add1
}

define noundef i32 @sum_across_swi(i32 noundef %a, i32 noundef %b, i32 noundef %c) {
entry:
  %add = add i32 %b, %a
  tail call void asm sideeffect "swi", ""()
  %add1 = add i32 %add, %c
  ret i32 %add1
}


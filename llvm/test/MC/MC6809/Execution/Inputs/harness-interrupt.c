/* Harness for codegen-interrupt.ll and irq-probe.inc. */
#include "harness-common.h"

extern volatile unsigned char irqs, swis, firqs;
extern unsigned long irq_sum, swi_sum, firq_sum;
extern volatile unsigned long seed;
extern unsigned long sum_across_irq(unsigned long a, unsigned long b, unsigned long c);
extern unsigned long sum_across_swi(unsigned long a, unsigned long b, unsigned long c);
extern void firq_leaf(void);
extern void firq_calls(void);
extern void irq_enable(void);
extern unsigned rs_survive_irq(void);
extern unsigned rs_survive_swi(void);
extern unsigned rs_survive_firq(void);
extern void (*firq_target)(void);

void test_main(void) {
    seed = 0x00010203UL;
    irq_enable();
    h_putx(rs_survive_irq()); h_putnl();                 /* 0: everything survived */
    h_putx(rs_survive_swi()); h_putnl();
    firq_target = firq_leaf;
    h_putx(rs_survive_firq()); h_putnl();
    firq_target = firq_calls;
    h_putx(rs_survive_firq()); h_putnl();
    h_putlong(sum_across_irq(0x01020304UL, 0x10203040UL, 0x00010001UL)); h_putnl();
    h_putlong(sum_across_swi(0x01020304UL, 0x10203040UL, 0x00010001UL)); h_putnl();
    h_puthex(irqs); h_puthex(swis); h_puthex(firqs); h_putnl();   /* two of each */
    h_putlong(irq_sum); h_putnl();                       /* and each ran to the end */
    h_putlong(swi_sum); h_putnl();
    h_putlong(firq_sum); h_putnl();
}

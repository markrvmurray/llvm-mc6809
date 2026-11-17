// RUN: %clang_cc1 -triple mc6809-unknown-unknown -fsyntax-only -verify %s

// Interrupt handlers are not implemented for this target, so the attributes
// that ask for one are errors. A handler compiled as an ordinary function
// returns with RTS and saves nothing, which fails only when the interrupt
// arrives.

__attribute__((interrupt)) void isr(void); // expected-error {{'interrupt' attribute is not supported on 'mc6809'}}
__attribute__((m6809_interrupt_norecurse)) void isr_once(void); // expected-error {{'m6809_interrupt_norecurse' attribute is not supported on 'mc6809'}}
__attribute__((m6809_no_isr)) void not_isr(void); // expected-error {{'m6809_no_isr' attribute is not supported on 'mc6809'}}

void ordinary(void);

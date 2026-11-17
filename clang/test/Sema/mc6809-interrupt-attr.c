// RUN: %clang_cc1 -triple mc6809-unknown-unknown -fsyntax-only -verify=bare %s
// RUN: %clang_cc1 -triple mc6809-unknown-os9 -fsyntax-only -verify=os9 %s
// RUN: %clang_cc1 -triple mc6809-unknown-decb -fsyntax-only -verify=decb %s

// A bare-metal program owns its vectors, so it may have interrupt handlers.
// Under OS-9 and Color BASIC the system fields the interrupt, and the
// attribute is refused.

__attribute__((interrupt)) void isr(void);
// os9-error@-1 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-2 {{'interrupt' attribute is not supported on 'decb'}}

// A handler is passed nothing and returns nothing.
__attribute__((interrupt)) void takes_arg(int n);
// bare-error@-1 {{'interrupt' attribute only applies to functions that take no parameters and return 'void'}}
// os9-error@-2 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-3 {{'interrupt' attribute is not supported on 'decb'}}
__attribute__((interrupt)) int returns_value(void);
// bare-error@-1 {{'interrupt' attribute only applies to functions that take no parameters and return 'void'}}
// os9-error@-2 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-3 {{'interrupt' attribute is not supported on 'decb'}}

// The argument names the interrupt. FIRQ has a handler of its own kind; the
// others are all entered alike.
__attribute__((interrupt("irq"))) void on_irq(void);
// os9-error@-1 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-2 {{'interrupt' attribute is not supported on 'decb'}}
__attribute__((interrupt("nmi"))) void on_nmi(void);
// os9-error@-1 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-2 {{'interrupt' attribute is not supported on 'decb'}}
__attribute__((interrupt("swi2"))) void on_swi2(void);
// os9-error@-1 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-2 {{'interrupt' attribute is not supported on 'decb'}}
__attribute__((interrupt("firq"))) void on_firq(void);
// os9-error@-1 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-2 {{'interrupt' attribute is not supported on 'decb'}}

// An unknown one is an error, not a warning that leaves an ordinary function.
__attribute__((interrupt("reset"))) void unknown_kind(void);
// bare-error@-1 {{'interrupt' attribute only applies to the interrupts 'irq', 'nmi', 'swi', 'swi2', 'swi3' and 'firq'}}
// os9-error@-2 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-3 {{'interrupt' attribute is not supported on 'decb'}}
__attribute__((interrupt(1))) void not_a_string(void);
// bare-error@-1 {{'interrupt' attribute requires a string}}
// os9-error@-2 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-3 {{'interrupt' attribute is not supported on 'decb'}}
__attribute__((interrupt("irq", "firq"))) void two_kinds(void);
// bare-error@-1 {{'interrupt' attribute takes no more than 1 argument}}
// os9-error@-2 {{'interrupt' attribute is not supported on 'os9'}}
// decb-error@-3 {{'interrupt' attribute is not supported on 'decb'}}

// The variants inherited with the attribute do nothing, so they are refused
// everywhere.
__attribute__((m6809_interrupt_norecurse)) void isr_once(void);
// bare-error@-1 {{'m6809_interrupt_norecurse' attribute is not supported on 'mc6809'}}
// os9-error@-2 {{'m6809_interrupt_norecurse' attribute is not supported on 'mc6809'}}
// decb-error@-3 {{'m6809_interrupt_norecurse' attribute is not supported on 'mc6809'}}
__attribute__((m6809_no_isr)) void not_isr(void);
// bare-error@-1 {{'m6809_no_isr' attribute is not supported on 'mc6809'}}
// os9-error@-2 {{'m6809_no_isr' attribute is not supported on 'mc6809'}}
// decb-error@-3 {{'m6809_no_isr' attribute is not supported on 'mc6809'}}

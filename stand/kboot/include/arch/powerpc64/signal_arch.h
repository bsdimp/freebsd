/*-
 * Copyright (c) 2005-2024 Rich Felker, et al.
 * Copyright (c) 2024, Netflix, Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 * Note: From the musl project, with minor tweaks
 * arch/powerpc64/bits/signal.h
 */

#pragma once

/*
 * hack to get the size right... Not sure I'll finish powerpc64 It's supposed to
 * be a minimal set so one can do signals, including void * arg in signal
 * handlers. I'm not doing tracebacks on segv/bus for powerpc, so I'm not
 * importing the full complexity from musl here.
 */
typedef struct {
	long __regs[4+4+48+33+1+34+34+32+1];
} host_mcontext_t;

typedef struct host__ucontext {
	unsigned long uc_flags;
	struct host__ucontext *uc_link;
	host_stack_t uc_stack;
	host_sigset_t uc_sigmask;
	host_mcontext_t uc_mcontext;
} host_ucontext_t;

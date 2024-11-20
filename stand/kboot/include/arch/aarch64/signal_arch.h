/*-
 * Copyright (c) 2005-2024 Rich Felker, et al.
 * Copyright (c) 2024, Netflix, Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 * Note: From the musl project, with minor tweaks
 * arch/aarch64/bits/signal.h
 */

#pragma once

/*
 * Minimal set of arch dependent signal definitions for aarch64, including void *
 * arg in signal handlers.
 */

typedef struct host_sigcontext {
	unsigned long fault_address;
	unsigned long regs[31];
	unsigned long sp, pc, pstate;
	long long __reserved[256 * 2];	/* long double is 16 bytes, long long 8 */
} host_mcontext_t;

typedef struct host__ucontext {
	unsigned long uc_flags;
	struct host__ucontext *uc_link;
	host_stack_t uc_stack;
	host_sigset_t uc_sigmask;
	host_mcontext_t uc_mcontext;
} host_ucontext_t;

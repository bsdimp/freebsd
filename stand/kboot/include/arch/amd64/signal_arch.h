/*-
 * Copyright (c) 2005-2024 Rich Felker, et al.
 * Copyright (c) 2024, Netflix, Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 * Note: From the musl project
 * arch/x86_64/bits/signal.h
 */

#pragma once

/*
 * Minimal set of arch dependent signal definitions for amd64, including void *
 * arg in signal handlers.
 */

enum {
	REG_R8 = 0,
	REG_R9,
	REG_R10,
	REG_R11,
	REG_R12,
	REG_R13,	/*  5 */
	REG_R14,
	REG_R15,
	REG_RSI,
	REG_RDI,
	REG_RBP,	/* 10 */
	REG_RBX,
	REG_RDX,
	REG_RAX,
	REG_RCX,
	REG_RSP,	/* 15 */
	REG_RIP,
	REG_EFL,
	REG_CSGSFS,
	REG_ERR,
	REG_TRAPNO,	/* 20 */
	REG_OLDMASK,
	REG_CR2		/* 22 */
};

typedef long long host_greg_t, host_gregset_t[23];
typedef void *host_fpregset_t;	/* Yes, pointer to storage */
typedef struct {
	host_gregset_t gregs;
	host_fpregset_t fpregs;
	unsigned long long __reserved1[8];
} host_mcontext_t;

typedef struct host__ucontext {
	unsigned long uc_flags;
	struct host__ucontext *uc_link;
	host_stack_t uc_stack;
	host_mcontext_t uc_mcontext;
	host_sigset_t uc_sigmask;
	unsigned long __fpregs_mem[64];
} host_ucontext_t;

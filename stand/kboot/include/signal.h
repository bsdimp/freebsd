/*-
 * Copyright (c) 2005-2024 Rich Felker, et al.
 * Copyright (c) 2024, Netflix, Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 * From musl, in part, abstracted from the relevant arch/$ARCH/bits/signal.h
 * files with the common bits landing here.
 */

#pragma once

/*
 * Note, on Linux, there's a separate user and kernel sigaction. For this
 * library, we use the kernel sigset_t set size. We don't define a real
 * siginfo_t, since we don't need it. And the kernel doesn't use sigset_t. I
 * also define stack_t differently than musl. The names are slighly different
 * than linux, but it's ABI-ly the same.
 */
typedef long host_siginfo_t;
struct host_sigaction {
	union {
		void	(*sa_handler)(int);
		void	(*sa_sigaction)(int, host_siginfo_t *, void *);
	};
	unsigned long sa_flags;
	void (*sa_restorer)(void);
	unsigned mask[2];
};
	
typedef struct host__sigset {
	unsigned long _bits[128 / sizeof(unsigned long)];
} host_sigset_t;

typedef struct host_sigaltstack {
	void *ss_sp;
	int ss_flags;
	unsigned long ss_size;
} host_stack_t;

#include "signal_arch.h"

/* flags */
#define HOST_SA_NOCLDSTOP	1U
#define HOST_SA_NOCLDWAIT	2U
#define HOST_SA_SIGINFO		4U
#define HOST_SA_ONSTACK		0x08000000U
#define HOST_SA_RESTART		0x10000000U
#define HOST_SA_NODEFER		0x40000000U
#define HOST_SA_RESETHAND	0x80000000U
#define HOST_SA_RESTORER	0x04000000U

int host_sigaction(int sig, struct host_sigaction *restrict act,
    struct host_sigaction *restrict oact);
void __rt_restore(void);

/* sig */
#define HOST_SIGHUP	 1
#define HOST_SIGINT	 2
#define HOST_SIGQUIT	 3
#define HOST_SIGILL	 4
#define HOST_SIGTRAP	 5
#define HOST_SIGABRT	 6
#define HOST_SIGBUS	 7
#define HOST_SIGFPE	 8
#define HOST_SIGKILL	 9
#define HOST_SIGUSR1	10
#define HOST_SIGSEGV	11
#define HOST_SIGUSR2	12
#define HOST_SIGPIPE	13
#define HOST_SIGALRM	14
#define HOST_SIGTERM	15
#define HOST_SIGSTKFLT	16
#define HOST_SIGCHLD	17
#define HOST_SIGCONT	18
#define HOST_SIGSTOP	19
#define HOST_SIGTSTP	20
#define HOST_SIGTTIN	21
#define HOST_SIGTTOU	22
#define HOST_SIGURG	23
#define HOST_SIGXCPU	24
#define HOST_SIGXFSZ	25
#define HOST_SIGVTALRM	26
#define HOST_SIGPROF	27
#define HOST_SIGWINCH	28
#define HOST_SIGIO	29
#define HOST_SIGPWR	30
#define HOST_SIGSYS	31
#define HOST_NSIG	65	/* Size of mask in bits + 1 */

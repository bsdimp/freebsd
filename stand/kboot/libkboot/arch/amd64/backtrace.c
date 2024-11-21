/*-
 * Copyright (c) 2024, Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "backtrace.h"
#include "signal.h"
#include "stand.h"

extern void *stack_upper_limit;

struct amd64_frame
{
	struct amd64_frame *frame;
	uint64_t rip;
};

void
text_crash_dump_ctx(void *ctx)
{
	host_ucontext_t *ucp = ctx;
	struct amd64_frame *frame = (struct amd64_frame *)ucp->uc_mcontext.gregs[REG_RBP];
	uintptr_t first = (uintptr_t)frame;
	uintptr_t upper = (uintptr_t)stack_upper_limit;

	printf("XXX CRASH XXX Should dump the registers too XXX CRASH XXX\n");

	printf("%#llx (frame %p)\n", ucp->uc_mcontext.gregs[REG_RIP], frame);

	while ((uintptr_t)frame >= first && (uintptr_t)frame < upper) {
		printf("%#lx (frame %p)\n", frame->rip, frame);
		frame = frame->frame;
	}
}

void
text_crash_init(const char *prog)
{
}

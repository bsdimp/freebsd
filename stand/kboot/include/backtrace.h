/*-
 * Copyright (c) 2024, Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/**
 * Print a 'crash dump' report. ctx is the void * arg is the context of the
 * exception / trap we're printing information about. It's exact meaning is both
 * execution environment and machine architecture dependent. For FreeBSD and
 * Linux binaries, it's the third argment of a signal hanlder (ucontext_t). This
 * routine is expected to print the machine state and a stack traceback, as best
 * it can.
 */
void text_crash_dump_ctx(void *ctx);

/**
 * Setup any state you need to do a crash dump later. Typically, this will read
 * in any symbols and pre-allocate any space necessary to produce the crash
 * dump.
 */
void text_crash_init(const char *prog);

/**
 * crt1 sets the top of the stack for us. This is a 'reasonable upper bound' not
 * an absolutely correct value as to what, exactly is in the stack. Since we
 * trace back through the linked list of frames, we just need to know the sane
 * limits of the next frame to terminate if we hit a frame that's non-conformant
 * or we run off the end w/o proper termination. crt1 should ensure we're always
 * terminated, but unknown buffer overflows and/or wild memory writes means we
 * need to walk defensively.
 */
extern void *stack_upper_limit;

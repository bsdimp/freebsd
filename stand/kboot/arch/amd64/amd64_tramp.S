/*-
 * Copyright (c) 2022 Netflix, Inc
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * This is the trampoline that starts the FreeBSD kernel. Since the Linux kernel
 * calls this routine with no args, and has a different environment than the boot
 * loader provides and that the kernel expects, this code is responsible for setting
 * all that up and calling the normal kernel entry point. It's analogous ot the
 * "purgatory" code in the linux kernel. Details about these operations are
 * contained in comments below. On amd64, the kernel will start all the APs so
 * we don't have to worry about them here.
 */

#include <machine/asmacros.h>

/*
 * Keep in sync with elf64_freebsd.c. Kexec starts tramp w/o any
 * parameters, so store them here.
 *
 * struct trampoline_data {
 *	uint64_t	entry;			//  0 (VA > KERNBASE)
 *	uint64_t	pt4;			//  8 PA of page tables
 *	uint64_t	modulep;		// 16 module metadata
 *	uint64_t	kernend;		// 24 kernel end
 * };
 *
 * Construct a world that btext expects, which is arguments on the stack, not in
 * registers, and these args are 32-bit not 64. We rely on being little endian
 * and some shift magic to achieve this layour in the code below:
 *
 * 0(%rsp) = 32 bit return address (cannot be used -- will be 0) ignored by btext
 * 4(%rsp) = 32 bit modulep
 * 8(%rsp) = 32 bit kernend
 * 12(%rsp) = upper 32 bit of kernend: ignored by code at btext, need room for it
 *
 * Processor is already in long mode when we're called, paging is enabled and
 * boot loader loads things such that:
 * - kernel mapped at KERNBASE, aligned to 2MB, below 4GB, contiguous memory
 * - there is a 2M hole at KERNBASE (KERNSTART = KERNBASE + 2M)
 * - kernel is mapped with 2M superpages
 * - The kernel, modules and metadata is in first 4GB which is unity mapped
 * - There's additional memory after loader provided data for early allocations
 *
 * Unlike EFI, we don't support copying the staging area. We tell Linux to land
 * the kernel in its final location with the needed alignment, etc. We copy the
 * trampoline code to 1MB offset above KERNBASE since that memory is otherwise free
 * and safely above the lower 1MB swamp we inherited from IBM PC, though this code
 * makes no assumptions about where that might be because we pass in the PA of
 * where this code will land as a result of the kexec (and we assume unity mapping
 * in calculating that address, so it has to be in the low 4GB).
 *
 * This trampoline installs sets up the arguments the kernel expects, installs
 * the page tables and returns to the entry point of the kernel (btext) (to ensure
 * the new page tables are active). The Linux purgatory loader starts out running
 * in 64-bit mode, but then winds the CPU back to 16-bit mode to jump into Linux's
 * startup code. Since this replaces that boot loader, we start in 64-bit mode.
 * We assume that the gdt/idt that Linux provides us with is adequate to access
 * memory that we need to do to make the handoff to the FreeBSD kernel.
 */
	.text
	.globl	tramp
tramp:
	cli				/* Make sure we don't get interrupted. */
	leaq	tramp_end(%rip), %rsp
	movq	tramp_entry(%rip), %r15	/* Kernel entry point */
	movq	tramp_pt4(%rip), %r14	/* Page tables */
	movq	tramp_modulep(%rip), %r13 /* Modulep */
	movq	tramp_kernend(%rip), %r12 /* Kernel end pointer */
	pushq	%r12			/* Push kernend offset. (upper 32bits ignored) */
	salq	$32, %r13		/* 32bit lsb modulep, 32bits 0 (retaddr) */
	pushq	%r13			/* Push modulep offset and 0 dword */
	pushq	%r15			/* Push the entry address. */
	movq	%r14, %cr3		/* Switch page tables. */
	ret				/* "Return" to kernel entry (btext). */
	.p2align	3
trampoline_data:
tramp_entry:	.quad	0
tramp_pt4:	.quad	0
tramp_modulep:	.quad	0
tramp_kernend:	.quad	0
	.space		0xa8	/* min 16 bytes for args +8 for pushq/popfq */
tramp_end:

	.data

	.type   tramp_size,@object
	.globl	tramp_size
tramp_size:
	.long	tramp_end-tramp
	.size	tramp_size, 4

	.type   tramp_data_offset,@object
	.globl	tramp_data_offset
tramp_data_offset:
	.long	trampoline_data-tramp
	.size	tramp_data_offset, 4

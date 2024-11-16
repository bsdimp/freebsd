/*-
 * Copyright (c) 2024, Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#ifndef __ASSEMBLER__

/*
 * We don't really use this, this is mostly to document the protocol.  The field
 * names are the same as linux for clarity.
 */

#include <sys/types.h>
struct efi_info {
	uint32_t efi_loader_signature;	/* 0x00 */
	uint32_t efi_systab;		/* 0x04 */
	uint32_t efi_memdesc_size;	/* 0x08 */
	uint32_t efi_memdesc_version;	/* 0x0c */
	uint32_t efi_memmap;		/* 0x10 */
	uint32_t efi_memmap_size;	/* 0x14 */
	uint32_t efi_systab_hi;		/* 0x18 */
	uint32_t efi_memmap_hi;		/* 0x1c */
} __packed;

/*
 * Abbreviated x86 Linux struct boot_param for the so-called zero-page.  At
 * present, all we need is access to the systbl elements of efi_info, so that's
 * all that's defined. FreeBSD's boot loader passes the address to write this
 * information to the trampoline and we pass it in just like a normal systbl. We
 * also pass in a few PAs that are in the setup_data EFI data, but we can get
 * that from userland so we don't have to have the crazy setup_data element to
 * walk the linked list to find that, which keeps the trampoline code simpler.
 */
struct linux_boot_params {
	uint8_t _pad1[0x1c0];				/* 0x000 */
	struct linux_efi_info efi_info;			/* 0x1c0 */
	uint8_t _pad2[0x1000 = 0x1c0 - sizeof(linux_efi_info)]; /* 0x1e0 */
} __packed;	/* Total size 4k, the page size on x86 */
#endif

#define SYSTAB_LO	(0x1c0 + 0x04)
#define MEMMAP_DSIZE	(0x1c0 + 0x08)
#define MEMMAP_DVERS	(0x1c0 + 0x0c)
#define MEMMAP_LO	(0x1c0 + 0x10)
#define MEMMAP_SIZE	(0x1c0 + 0x14)
#define SYSTAB_HI	(0x1c0 + 0x18)
#define MEMMAP_HI	(0x1c0 + 0x1c)

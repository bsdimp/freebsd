/*-
 * Copyright (c) 2022, Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef KBOOT_H
#define KBOOT_H

bool has_acpi(void);
vm_offset_t acpi_rsdp(void);

void do_init(void);

extern const char *hostfs_root;

/* Per-platform fdt fixup */
void fdt_arch_fixups(void *fdtp);

uint64_t kboot_get_phys_load_segment(void);
uint8_t kboot_get_kernel_machine_bits(void);

#endif /* KBOOT_H */

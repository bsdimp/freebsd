/*-
 * Copyright (c) 2022 Netflix, Inc
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>

#include "kboot.h"
#include "bootstrap.h"

uint64_t
kboot_get_phys_load_segment(void)
{
	return 0x40000000 | 0x4200000;
}

void
bi_loadsmap(struct preloaded_file *kfp)
{
}

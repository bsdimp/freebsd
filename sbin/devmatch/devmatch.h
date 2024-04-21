/*-
 * Copyright (c) 2024 Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#define DM_ALL		0x01
#define DM_DUMP		0x02
#define DM_QUIET	0x04
#define DM_UNBOUND	0x08
#define DM_VERBOSE	0x10

#define IS_(a, b)	((b & DM_ ## a) != 0)
#define IS_ALL(f)	IS_(ALL, f)
#define IS_DUMP(f)	IS_(DUMP, f)
#define IS_QUIET(f)	IS_(QUIET, f)
#define IS_UNBOUND(f)	IS_(UNBOUND, f)
#define IS_VERBOSE(f)	IS_(VERBOSE, f)

struct devmatch
{
	const char *linker_hints;
	uint32_t flags;

	void *hints;
	void *hints_end;
	struct devinfo_dev *root;
};

struct devmatch *devmatch_init(uint32_t flags, const char *linker_hints);
void devmatch_fini(struct devmatch *dm);
void devmatch_find_nomatch(struct devmatch *dm, char *nomatch);
void devmatch_find(struct devmatch *dm);

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

struct devinfo_dev;
class devmatch
{
public:
	devmatch(uint32_t f, const char *l)
	    : linker_hints(l), flags(f), hints(NULL), hints_end(NULL), root(NULL) {
		init();
	}
	~devmatch();
	int find_nomatch(char *nomatch);
	int find();
	int search_hints(const char *bus, const char *dev, const char *pnpinfo);
private:
	void init();
	void read_linker_hints();
	static int find_unmatched(struct devinfo_dev *dev, void *arg);

	const char *linker_hints;
	uint32_t flags;

	void *hints;
	void *hints_end;
	struct devinfo_dev *root;
};


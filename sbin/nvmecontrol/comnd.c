/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2018 Netflix, Inc
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
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/ioccom.h>

#include <ctype.h>
#include <dirent.h>
#include <dlfcn.h>
#include <err.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "comnd.h"

CMD_DECLARE(top, struct cmd_function);

static void
print_usage(const struct cmd_function *f)
{
	const char *cp;
	char ch;
	bool need_prefix = true;

	cp = f->usage;
	while (*cp) {
		ch = *cp++;
		if (need_prefix) {
			if (ch != ' ')
				fputs("        nvmecontrol ", stderr);
			else
				fputs("                    ", stderr);
		}
		fputc(ch, stderr);
		need_prefix = (ch == '\n');
	}
	if (!need_prefix)
		fputc('\n', stderr);
}

void
gen_usage_set(const struct cmd_function * const *f, const struct cmd_function * const *flimit)
{

	fprintf(stderr, "usage:\n");
	while (f < flimit) {
		print_usage(*f);
		f++;
	}
	exit(1);
}

void
usage(const struct cmd_function *f)
{

	fprintf(stderr, "usage:\n");
	print_usage(f);
	exit(1);
}

void
dispatch_set(int argc, char *argv[], const struct cmd_function * const *tbl,
    const struct cmd_function * const *tbl_limit)
{
	const struct cmd_function * const *f = tbl;

	if (argv[1] == NULL) {
		gen_usage_set(tbl, tbl_limit);
		return;
	}

	while (f < tbl_limit) {
		if (strcmp(argv[1], (*f)->name) == 0) {
			(*f)->fn(*f, argc-1, &argv[1]);
			return;
		}
		f++;
	}

	fprintf(stderr, "Unknown command: %s\n", argv[1]);
	gen_usage_set(tbl, tbl_limit);
}

void
set_concat_add(struct set_concat *m, void *b, void *e)
{
	void **bp, **ep;
	int add_n, cur_n;

	if (b == NULL)
		return;
	/*
	 * Args are really pointers to arrays of pointers, but C's
	 * casting rules kinda suck since you can't directly cast
	 * struct foo ** to a void **.
	 */
	bp = (void **)b;
	ep = (void **)e;
	add_n = ep - bp;
	cur_n = 0;
	if (m->begin != NULL)
		cur_n = m->limit - m->begin;
	m->begin = reallocarray(m->begin, cur_n + add_n, sizeof(void *));
	if (m->begin == NULL)
		err(1, "expanding concat set");
	memcpy(m->begin + cur_n, bp, add_n * sizeof(void *));
	m->limit = m->begin + cur_n + add_n;
}

static void
arg_suffix(char *buf, size_t len, arg_type at)
{
	switch (at) {
	case arg_none:
		break;
	case arg_string:
		strlcat(buf, "=<STRING>", len);
		break;
	case arg_path:
		strlcat(buf, "=<FILE>", len);
		break;
	default:
		strlcat(buf, "=<NUM>", len);
		break;
	}
}

void
arg_help(int argc __unused, char * const *argv, const char *desc, const struct opts *opts)
{
	int i;
	char buf[41];
	char *cp;

	if (argv[optind])
		fprintf(stderr, "Unknown argument: %s\n", argv[optind]);
	fprintf(stderr, "Usage:\n        nvmecontrol %s device%s\n", argv[0],
	    opts ? " <args>" : "");
	fprintf(stderr, "\n%s\n", desc);
	if (opts != NULL) {
		fprintf(stderr, "Options:\n");
		for (i = 0; opts[i].long_arg != NULL; i++) {
			strlcpy(buf, "  --", sizeof(buf));
			strlcat(buf, opts[i].long_arg, sizeof(buf));
			arg_suffix(buf, sizeof(buf), opts[i].at);
			if (isprint(opts[i].short_arg)) {
				strlcat(buf, ", -", sizeof(buf));
				cp = buf + strlen(buf);
				if (cp - buf < (ptrdiff_t)sizeof(buf) - 2) {
					*cp++ = opts[i].short_arg;
					*cp++ = '\0';
				}
				arg_suffix(buf, sizeof(buf), opts[i].at);
			}
			fprintf(stderr, "%-40.40s - %s\n", buf, opts[i].descr);
		}
	}
	exit(1);
}

int
arg_parse(int argc, char * const * argv, const char *desc, const struct opts *opts)
{
	int i, n, idx, ch;
	unsigned long v;
	struct option *lopts;
	char *shortopts, *p;

	if (opts == NULL)
		n = 0;
	else
		for (n = 0; opts[n].long_arg != NULL;)
			n++;
	lopts = malloc((n + 2) * sizeof(struct option));
	if (lopts == NULL)
		err(1, "option memory");
	p = shortopts = malloc((n + 3) * sizeof(char));
	if (shortopts == NULL)
		err(1, "shortopts memory");
	for (i = 0; i < n; i++) {
		lopts[i].name = opts[i].long_arg;
		lopts[i].has_arg = opts[i].at == arg_none ? no_argument : required_argument;
		lopts[i].flag = NULL;
		lopts[i].val = opts[i].short_arg;
		if (isprint(opts[i].short_arg)) {
			*p++ = opts[i].short_arg;
			if (lopts[i].has_arg)
				*p++ = ':';
		}
	}
	lopts[n].name = "help";
	lopts[n].has_arg = no_argument;
	lopts[n].flag = NULL;
	lopts[n].val = '?';
	*p++ = '?';
	*p++ = '\0';
	memset(lopts + n + 1, 0, sizeof(struct option));
	while ((ch = getopt_long(argc, argv, shortopts, lopts, &idx)) != -1) {
		if (idx == n)
			arg_help(argc, argv, desc, opts);
		switch (opts[idx].at) {
		case arg_none:
			*(bool *)opts[idx].ptr = true;
			break;
		case arg_string:
		case arg_path:
			*(const char **)opts[idx].ptr = optarg;
			break;
		case arg_uint8:
			v = strtoul(optarg, NULL, 0);
			if (v > 0xff)
				goto bad_arg;
			*(uint8_t *)opts[idx].ptr = v;
			break;
		case arg_uint16:
			v = strtoul(optarg, NULL, 0);
			if (v > 0xffff)
				goto bad_arg;
			*(uint16_t *)opts[idx].ptr = v;
			break;
		case arg_uint32:
			v = strtoul(optarg, NULL, 0);
			if (v > 0xffffffff)
				goto bad_arg;
			*(uint32_t *)opts[idx].ptr = v;
			break;
		}
	}
	free(lopts);
	return (0);
bad_arg:
	fprintf(stderr, "Bad value to --%s: %s\n", opts[idx].long_arg, optarg);
	exit(1);
}

/*
 * Loads all the .so's from the specified directory.
 */
void
cmd_load_dir(const char *dir, cmd_load_cb_t cb, void *argp)
{
	DIR *d;
	struct dirent *dent;
	char *path = NULL;
	void *h, *begin, *limit;

	d = opendir(dir);
	if (d == NULL)
		return;
	for (dent = readdir(d); dent != NULL; dent = readdir(d)) {
		if (strcmp(".so", dent->d_name + dent->d_namlen - 3) != 0)
			continue;
		asprintf(&path, "%s/%s", dir, dent->d_name);
		if (path == NULL)
			err(1, "Can't malloc for path, giving up.");
		if ((h = dlopen(path, RTLD_NOW | RTLD_GLOBAL)) == NULL)
			warnx("Can't load %s: %s", path, dlerror());
		else {
			/*
			 * Add in the top (for cli commands) and logpage (for
			 * logpage parsing) linker sets. We have to do this by
			 * hand because linker sets aren't automatically merged.
			 */
			begin = dlsym(h, "__start_set_top");
			limit = dlsym(h, "__stop_set_top");
			if (begin != NULL)
				add_to_top(begin, limit);
			if (cb != NULL)
				cb(argp, h);
		}
		free(path);
		path = NULL;
	}
	closedir(d);
}

void
cmd_init(void)
{

	add_to_top(CMD_BEGIN(top), CMD_LIMIT(top));
}

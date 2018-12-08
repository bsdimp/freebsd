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
#include <err.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "opts.h"

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

static void
arg_help(int argc __unused, char * const *argv, const char *desc, const struct args *args)
{
	int i;
	char buf[41];
	char *cp;

	if (argv[optind])
		fprintf(stderr, "Unknown argument: %s\n", argv[optind]);
	fprintf(stderr, "Usage:\n        nvmecontrol %s device <args>\n", argv[0]);
	fprintf(stderr, "\n%s\n", desc);
	fprintf(stderr, "Options:\n");
	for (i = 0; args[i].long_arg != NULL; i++) {
		strlcpy(buf, "  [ --", sizeof(buf));
		strlcat(buf, args[i].long_arg, sizeof(buf));
		arg_suffix(buf, sizeof(buf), args[i].at);
		if (isprint(args[i].short_arg)) {
			strlcat(buf, ", -", sizeof(buf));
			cp = buf + strlen(buf);
			if (cp - buf < (ptrdiff_t)sizeof(buf) - 2) {
				*cp++ = args[i].short_arg;
				*cp++ = '\0';
			}
			arg_suffix(buf, sizeof(buf), args[i].at);
		}
		strlcat(buf, " ]", sizeof(buf));
		fprintf(stderr, "%-40.40s - %s\n", buf, args[i].descr);
	}
	exit(1);
}

int
arg_parse(int argc, char * const * argv, const char *desc, const struct args *args)
{
	int i, n, idx, ch;
	unsigned long v;
	struct option *opts;
	char *shortopts, *p;

	for (n = 0; args[n].long_arg != NULL;)
		n++;
	opts = malloc((n + 2) * sizeof(struct option));
	if (opts == NULL)
		err(1, "option memory");
	p = shortopts = malloc((n + 3) * sizeof(char));
	if (shortopts == NULL)
		err(1, "shortopts memory");
	for (i = 0; i < n; i++) {
		opts[i].name = args[i].long_arg;
		opts[i].has_arg = args[i].at == arg_none ? no_argument : required_argument;
		opts[i].flag = NULL;
		opts[i].val = args[i].short_arg;
		if (isprint(args[i].short_arg))
			*p++ = args[i].short_arg;
	}
	opts[n].name = "help";
	opts[n].has_arg = no_argument;
	opts[n].flag = NULL;
	opts[n].val = '?';
	*p++ = '?';
	*p++ = '\0';
	memset(opts + n + 1, 0, sizeof(struct option));
	while ((ch = getopt_long(argc, argv, shortopts, opts, &idx)) != -1) {
		if (idx == n)
			arg_help(argc, argv, desc, args);
		switch (args[idx].at) {
		case arg_none:
			*(bool *)args[idx].ptr = true;
			break;
		case arg_string:
		case arg_path:
			*(const char **)args[idx].ptr = optarg;
			break;
		case arg_uint8:
			v = strtoul(optarg, NULL, 0);
			if (v > 0xff)
				goto bad_arg;
			*(uint8_t *)args[idx].ptr = v;
			break;
		case arg_uint16:
			v = strtoul(optarg, NULL, 0);
			if (v > 0xffff)
				goto bad_arg;
			*(uint16_t *)args[idx].ptr = v;
			break;
		case arg_uint32:
			v = strtoul(optarg, NULL, 0);
			if (v > 0xffffffff)
				goto bad_arg;
			*(uint32_t *)args[idx].ptr = v;
			break;
		}
	}
	return (0);
bad_arg:
	fprintf(stderr, "Bad value to --%s: %s\n", args[idx].long_arg, optarg);
	exit(1);
}

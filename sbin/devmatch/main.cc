/*-
 * Copyright (c) 2017 Netflix, Inc.
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

#include <err.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>

#include <exception>
#include <iostream>

#include "devmatch.h"

/* options descriptor */
static struct option longopts[] = {
	{ "all",		no_argument,		NULL,	'a' },
	{ "dump",		no_argument,		NULL,	'd' },
	{ "hints",		required_argument,	NULL,	'h' },
	{ "nomatch",		required_argument,	NULL,	'p' },
	{ "quiet",		no_argument,		NULL,	'q' },
	{ "unbound",		no_argument,		NULL,	'u' },
	{ "verbose",		no_argument,		NULL,	'v' },
	{ NULL,			0,			NULL,	0 }
};

static void
usage(void)
{

	errx(1, "devmatch [-adv] [-p nomatch] [-h linker-hints]");
}

int
main(int argc, char **argv)
{
	int ch, rv = 0;
	uint32_t flags = 0;
	const char *linker_hints = NULL;
	char *nomatch_str = NULL;

	while ((ch = getopt_long(argc, argv, "adh:p:quv",
		    longopts, NULL)) != -1) {
		switch (ch) {
		case 'a':
			flags |= DM_ALL;
			break;
		case 'd':
			flags |= DM_DUMP;
			break;
		case 'h':
			linker_hints = optarg;
			break;
		case 'p':
			nomatch_str = optarg;
			break;
		case 'q':
			flags |= DM_QUIET;
			break;
		case 'u':
			flags |= DM_UNBOUND;
			break;
		case 'v':
			flags |= DM_VERBOSE;
			break;
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;

	if (argc >= 1)
		usage();

	try {
		devmatch dm(flags, linker_hints);

		if (IS_DUMP(flags))
			rv = dm.search_hints(NULL, NULL, NULL);
		else if (nomatch_str != NULL)
			rv = dm.find_nomatch(nomatch_str);
		else
			rv = dm.find();
	}
	catch (std::exception e) {
		std::cerr << e.what() << std::endl;
		::exit(1);
	}
	return(rv);
}

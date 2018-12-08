/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2012-2013 Intel Corporation
 * All rights reserved.
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

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "nvmecontrol.h"
#include "comnd.h"

#define ADMIN_USAGE \
	"admin-passthru >>args<<\n"
#define ADMIN_ARGS \
	"<controller-id>"
#define ADMIN_DESCR \
	"Send a raw administrative command to the controller via a passthrough interface."
#define IO_USAGE \
	"io-passthru >>args<<\n"
#define IO_ARGS \
	"<namespace-id>"
#define IO_DESCR \
	"Send a raw I/O command to the controller via a passthrough interface."

static void
passthru(const struct cmd_function *nf __unused, int argc, char *argv[], const char *desc)
{
	int	fd = -1, ifd = -1;
	void	*data = NULL, *metadata = NULL;
	struct nvme_pt_command	pt;

	struct options {
		uint8_t		opcode;
		uint8_t		flags;
		uint16_t	rsvd;
		uint32_t	nsid;
		uint32_t	data_len;
		uint32_t	metadata_len;
		uint32_t	timeout;
		uint32_t	cdw2;
		uint32_t	cdw3;
		uint32_t	cdw10;
		uint32_t	cdw11;
		uint32_t	cdw12;
		uint32_t	cdw13;
		uint32_t	cdw14;
		uint32_t	cdw15;
		const char	*ifn;
		bool		binary;
		bool		show_command;
		bool		dry_run;
		bool		read;
		bool		write;
		uint8_t		prefill;
	} opt = {
		.binary = false,
		.cdw10 = 0,
		.cdw11 = 0,
		.cdw12 = 0,
		.cdw13 = 0,
		.cdw14 = 0,
		.cdw15 = 0,
		.cdw2 = 0,
		.cdw3 = 0,
		.data_len = 0,
		.dry_run = false,
		.flags = 0,
		.ifn = "",
		.metadata_len = 0,
		.nsid = 0,
		.opcode = 0,
		.prefill = 0,
		.read = false,
		.rsvd = 0,
		.show_command = false,
		.timeout = 0,
		.write = false,
	};

	const char *opcode = "NVMe command opcode (required)";
	const char *flags = "NVMe command flags";
	const char *rsvd = "Reserved field value";
	const char *nsid = "Namespace id (ignored on FreeBSD)";
	const char *data_len = "Length of data for I/O (bytes)";
	const char *metadata_len = "Length of metadata segment (bytes) (igored)";
	const char *cdw2 = "Command dword 2 value";
	const char *cdw3 = "Command dword 3 value";
	const char *cdw10 = "Command dword 10 value";
	const char *cdw11 = "Command dword 11 value";
	const char *cdw12 = "Command dword 12 value";
	const char *cdw13 = "Command dword 13 value";
	const char *cdw14 = "Command dword 14 value";
	const char *cdw15 = "Command dword 15 value";
	const char *inf = "Input file to send (default stdin)";
	const char *timeout = "Comand timeout (ms)";
	const char *raw = "Output in binary format";
	const char *prefill = "Value to prefill payload with";
	const char *dry_run = "Don't actually execute the command";
	const char *rd = "Command reads data from device";
	const char *wr = "Command writes data to device";
	const char *show = "Show all the command values on stdout";
	/*
	 * Argument names and short names selected to match the nvme-cli program
	 * so vendor-siupplied formulas work out of the box on FreeBSD with a simple
	 * s/nvme/nvmecontrol/.
	 */
	const struct opts opts[] = {
		{ "opcode",		'o',	arg_uint8,	&opt.opcode,		opcode},
		{ "cdw2",		'2',	arg_uint32,	&opt.cdw2,		cdw2},
		{ "cdw3",		'3',	arg_uint32,	&opt.cdw3,		cdw3},
		{ "cdw10",		'4',	arg_uint32,	&opt.cdw10,		cdw10},
		{ "cdw11",		'5',	arg_uint32,	&opt.cdw11,		cdw11},
		{ "cdw12",		'6',	arg_uint32,	&opt.cdw12,		cdw12},
		{ "cdw13",		'7',	arg_uint32,	&opt.cdw13,		cdw13},
		{ "cdw14",		'8',	arg_uint32,	&opt.cdw14,		cdw14},
		{ "cdw15",		'9',	arg_uint32,	&opt.cdw15,		cdw15},
		{ "data-len",		'l',	arg_uint32,	&opt.data_len,		data_len},
		{ "metadata-len",	'm',	arg_uint32,	&opt.metadata_len,	metadata_len},
		{ "flags",		'f',	arg_uint8,	&opt.flags,		flags},
		{ "input-file",		'i',	arg_path,	&opt.ifn,		inf},
		{ "namespace-id",	'n',	arg_uint32,	&opt.nsid,		nsid},
		{ "prefill",		'p',	arg_uint8,	&opt.prefill,		prefill},
		{ "rsvd",		'R',	arg_uint16,	&opt.rsvd,		rsvd},
		{ "timeout",		't',	arg_uint32,	&opt.timeout,		timeout},
		{ "raw-binary",		'b',	arg_none,	&opt.binary,		raw},
		{ "dry-run",		'd',	arg_none,	&opt.dry_run,		dry_run},
		{ "read",		'r',	arg_none,	&opt.read,		rd},
		{ "show-command",	's',	arg_none,	&opt.show_command,	show},
		{ "write",		'w',	arg_none,	&opt.write,		wr},
		{ NULL, 0, arg_none, NULL, NULL }
	};

	fd = arg_parse_and_open(argc, argv, desc, opts);
	if (fd == -1)
		err(1, "open %s", argv[optind]);

	if (opt.read && opt.write)
		errx(1, "need exactly one of --read or --write");
	if (opt.data_len != 0 && !opt.read && !opt.write)
		errx(1, "need exactly one of --read or --write");
	if (*opt.ifn && (ifd = open(opt.ifn, O_RDONLY)) == -1) {
		warn("open %s", opt.ifn);
		goto cleanup;
	}
#if notyet	/* No support in kernel for this */
	if (opt.metadata_len != 0) {
		if (posix_memalign(&metadata, getpagesize(), opt.metadata_len)) {
			warn("can't allocate %d bytes for metadata", metadata_len);
			goto cleanup;
		}
	}
#else
	if (opt.metadata_len != 0)
		errx(1, "metadata not supported on FreeBSD");
#endif
	if (opt.data_len) {
		if (posix_memalign(&data, getpagesize(), opt.data_len)) {
			warn("can't allocate %d bytes for data", opt.data_len);
			goto cleanup;
		}
		memset(data, opt.prefill, opt.data_len);
		if (opt.write && read(ifd, data, opt.data_len) < 0) {
			warn("read %s", *opt.ifn ? opt.ifn : "stdin");
			goto cleanup;
		}
	}
	if (opt.show_command) {
		fprintf(stderr, "opcode       : %#02x\n", opt.opcode);
		fprintf(stderr, "flags        : %#02x\n", opt.flags);
		fprintf(stderr, "rsvd1        : %#04x\n", opt.rsvd);
		fprintf(stderr, "nsid         : %#04x\n", opt.nsid);
		fprintf(stderr, "cdw2         : %#08x\n", opt.cdw2);
		fprintf(stderr, "cdw3         : %#08x\n", opt.cdw3);
		fprintf(stderr, "data_len     : %#08x\n", opt.data_len);
		fprintf(stderr, "metadata_len : %#08x\n", opt.metadata_len);
		fprintf(stderr, "data         : %p\n", data);
		fprintf(stderr, "metadata     : %p\n", metadata);
		fprintf(stderr, "cdw10        : %#08x\n", opt.cdw10);
		fprintf(stderr, "cdw11        : %#08x\n", opt.cdw11);
		fprintf(stderr, "cdw12        : %#08x\n", opt.cdw12);
		fprintf(stderr, "cdw13        : %#08x\n", opt.cdw13);
		fprintf(stderr, "cdw14        : %#08x\n", opt.cdw14);
		fprintf(stderr, "cdw15        : %#08x\n", opt.cdw15);
		fprintf(stderr, "timeout_ms   : %d\n", opt.timeout);
	}
	if (opt.dry_run) {
		errno = 0;
		warn("Doing a dry-run, no actual I/O");
		goto cleanup;
	}

	memset(&pt, 0, sizeof(pt));
	pt.cmd.opc = opt.opcode;
	pt.cmd.fuse = opt.flags;
	pt.cmd.cid = htole16(opt.rsvd);
	pt.cmd.nsid = opt.nsid;				/* XXX note: kernel overrides this */
	pt.cmd.rsvd2 = htole32(opt.cdw2);
	pt.cmd.rsvd3 = htole32(opt.cdw3);
	pt.cmd.cdw10 = htole32(opt.cdw10);
	pt.cmd.cdw11 = htole32(opt.cdw11);
	pt.cmd.cdw12 = htole32(opt.cdw12);
	pt.cmd.cdw13 = htole32(opt.cdw13);
	pt.cmd.cdw14 = htole32(opt.cdw14);
	pt.cmd.cdw15 = htole32(opt.cdw15);
	pt.buf = data;
	pt.len = opt.data_len;
	pt.is_read = opt.read;

	errno = 0;
	if (ioctl(fd, NVME_PASSTHROUGH_CMD, &pt) < 0)
		err(1, "passthrough request failed");
	/* XXX report status */
	if (opt.read) {
		if (opt.binary)
			write(STDOUT_FILENO, data, opt.data_len);
		else {
			/* print status here */
			print_hex(data, opt.data_len);
		}
	}
cleanup:
	if (errno)
		exit(1);
}

static void
admin_passthru(const struct cmd_function *nf, int argc, char *argv[])
{
	const char *desc = "Send a pass through Admin command to the specified device\n";

	passthru(nf, argc, argv, desc);
}

static void
io_passthru(const struct cmd_function *nf, int argc, char *argv[])
{
	const char *desc = "Send a pass through I/O command to the specified device\n";

	passthru(nf, argc, argv, desc);
}

CMD_COMMAND(top, admin-passthru, admin_passthru, ADMIN_USAGE, ADMIN_ARGS, ADMIN_DESCR);
CMD_COMMAND(top, admin-passthru, io_passthru, IO_USAGE, IO_ARGS, IO_DESCR);

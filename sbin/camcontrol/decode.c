/*-
 * Copyright (c) 2023 Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * SCSI / ATA / NVME command and error buffer decoding
 *
 * Takes a standard CDB, ADB or NVME command and/or error buffer
 * and decodes it similar to the error printed in the kernel.
 *
 * Alternatively, you can feed it the one-line CAM-produced devd
 * message, and it will decode that as well.
 */

#include <sys/types.h>

#include <err.h>
//#include <inttypes.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
#include <unistd.h>

#include <cam/cam.h>
#include <cam/cam_debug.h>
#include <cam/cam_ccb.h>
#include <cam/scsi/scsi_all.h>
#include <cam/scsi/scsi_message.h>
#include <camlib.h>
#include "camcontrol.h"

int
decode(struct cam_device *device __unused, int argc, char **argv, char *combinedopt,
    int verbosemode __unused)
{
	int c;
	int error = 0;
	uint8_t cdb[20];
	int cdb_len = 0;
	uint8_t atacmd[12];
	int atacmd_len = 0;

	while ((c = getopt(argc, argv, combinedopt)) != -1) {
		switch (c) {
		case 'a':
			/* Parsing next N args, have to adjust optind */
			amdcmd_len = hex_args_to_buffer(amdcmd, sizeof(amdcmd),
			    argc - optind, argv + optind, &got);
			optind += got;
			break;
		case 'c':
		case 's':	/* accept -s as an alias for scsi */
			/* Parsing next N args, have to adjust optind */
			cdb_len = hex_args_to_buffer(cdb, sizeof(cdb),
			    argc - optind, argv + optind, &got);
			optind += got;
			break;
		case 'd':
		case 'e':
		case 'N':
			errx(1, "Not implemented, you lose.");
		default:
			break;
		}
	}

	return (error);
}

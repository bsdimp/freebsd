/*-
 * Copyright (c) 2017 Netflix, Inc.
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
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/kobj.h>
#include <sys/sbuf.h>

#include <geom/geom.h>
#include <geom/label/g_label.h>
#include <geom/part/g_part.h>

#define	G_LABEL_EFI_DIR	"efi"

struct g_part_efi_entry {
	struct g_part_entry     base;
};

extern char *efi_get_boot_path(void);

static void
g_label_efi_taste(struct g_consumer *cp, char *label, size_t size)
{
	struct g_provider *pp;
	struct g_part_table *tp;
	char *path;

	g_topology_assert_not();
	pp = cp->provider;
	tp = (struct g_part_table *)pp->geom->softc;
	label[0] = '\0';

	path = malloc(4096, M_GEOM, M_NOWAIT);
	if (path == NULL)
		return;

	if (g_geom_efi_devpath(pp->geom, path, 4096) != 0)
		goto out;

	/* Start out only matching where we loaded /boot/loader.efi from */
	if (strcmp(path, efi_get_boot_path()) == 0) {
		strlcpy(label, "root", size);
		goto out;
	}
out:
	free(path, M_GEOM);
	/* Need to figure out how to do boot disk as well maybe -- for now fall through */
}

struct g_label_desc g_label_efi = {
	.ld_taste = g_label_efi_taste,
	.ld_dir = G_LABEL_EFI_DIR,
	.ld_enabled = 1
};

G_LABEL_INIT(efi, g_label_efi, "Create device nodes for EFI labels");

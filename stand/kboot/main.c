/*-
 * Copyright (C) 2010-2014 Nathan Whitehorn
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TOOLS GMBH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <stand.h>
#include <sys/endian.h>
#include <sys/param.h>
#include <fdt_platform.h>

#include <machine/cpufunc.h>
#include <bootstrap.h>
#include "host_syscall.h"
#include "kboot.h"
#include "mem.h"
#include "stand.h"
#ifdef LOADER_ZFS_SUPPORT
#include "libzfs.h"
static void kboot_zfs_probe(void);
static uint64_t pool_guid;
bool hostdisk_zfs_probe(uint64_t *);
#endif

struct arch_switch	archsw;
extern void *_end;

int kboot_getdev(void **vdev, const char *devspec, const char **path);
ssize_t kboot_copyin(const void *src, vm_offset_t dest, const size_t len);
ssize_t kboot_copyout(vm_offset_t src, void *dest, const size_t len);
ssize_t kboot_readin(readin_handle_t fd, vm_offset_t dest, const size_t len);
int kboot_autoload(void);
uint64_t kboot_loadaddr(u_int type, void *data, uint64_t addr);
static void kboot_kseg_get(int *nseg, void **ptr);

extern int command_fdt_internal(int argc, char *argv[]);

#define PA_INVAL (vm_offset_t)-1
static vm_offset_t pa_start = PA_INVAL;
static vm_offset_t padding;
static vm_offset_t offset;

/*
 * Point (dev) at an allocated device specifier matching the string version
 * at the beginning of (devspec).  Return a pointer to the remaining
 * text in (path).
 *
 * In all cases, the beginning of (devspec) is compared to the names
 * of known devices in the device switch, and then any following text
 * is parsed according to the rules applied to the device type.
 */
static int
kboot_parsedev(struct devdesc **dev, const char *devspec, const char **path)
{
	struct devdesc		*idev;
	struct devsw		*dv;
	int			i, unit, err;
	char			*cp;
	const char		*np;

	/* minimum length check */
	if (strlen(devspec) < 2)
		return(EINVAL);

	/* look for a device that matches */
	for (i = 0, dv = NULL; devsw[i] != NULL; i++) {
		if (!strncmp(devspec, devsw[i]->dv_name, strlen(devsw[i]->dv_name))) {
			dv = devsw[i];
			break;
		}
	}
	if (dv == NULL)
		return(ENOENT);

	np = (devspec + strlen(dv->dv_name));
	idev = NULL;
	err = 0;

	switch(dv->dv_type) {
	case DEVT_NONE:
		break;

	case DEVT_HOSTDISK:
		/*
		 * Allocate space for the devspec string after the idev device
		 * so it gets freed automatically. This string is passed to open
		 * and specifies the Linux device to open (eg /dev/sda1 or
		 * /dev/nvme0n1p2).
		 */
		idev = malloc(sizeof(*idev) + strlen(devspec) + 1);
		if (idev == NULL)
			return (ENOMEM);
		idev->d_opendata = (char *)(idev + 1);
		strcpy(idev->d_opendata, devspec);
		cp = strchr(idev->d_opendata, ':');
		if (cp != NULL) {
			*cp++ = '\0';
			if (*cp != '\0') {
				*path = cp;
				printf("hostdisk %s path %s\n", (char *)idev->d_opendata,
				    *path);
			} else {
				printf("hostdisk %s\n", (char *)idev->d_opendata);
			}
		} else {
			printf("hostdisk %s (no colon)\n", (char *)idev->d_opendata);
		}
		break;
#ifdef LOADER_ZFS_SUPPORT
	case DEVT_ZFS:
		idev = malloc(sizeof (struct zfs_devdesc));
		if (idev == NULL)
			return (ENOMEM);

		err = zfs_parsedev((struct zfs_devdesc *)idev, np, path);
		if (err != 0)
			goto fail;
		break;
#endif
	default:
		idev = malloc(sizeof (struct devdesc));
		if (idev == NULL)
			return (ENOMEM);

		unit = 0;
		cp = (char *)np;

		if (*np && (*np != ':')) {
			unit = strtol(np, &cp, 0);	/* get unit number if present */
			if (cp == np) {
				err = EUNIT;
				goto fail;
			}
		}

		if (*cp && (*cp != ':')) {
			err = EINVAL;
			goto fail;
		}

		idev->d_unit = unit;
		if (path != NULL)
			*path = (*cp == 0) ? cp : cp + 1;
		break;
	}
	idev->d_dev = dv;
	if (dev != NULL)
		*dev = idev;
	else
		free(idev);

	return(0);

fail:
	free(idev);
	return(err);
}

/*
 * NB: getdev should likely be identical to this most places, except maybe
 * we should move to storing the length of the platform devdesc.
 */
int
kboot_getdev(void **vdev, const char *devspec, const char **path)
{
	struct devdesc **dev = (struct devdesc **)vdev;
	int				rv;

	/*
	 * If it looks like this is just a path and no
	 * device, go with the current device.
	 */
	if ((devspec == NULL) ||
	    (strchr(devspec, ':') == NULL)) {
		if (((rv = kboot_parsedev(dev, getenv("currdev"), NULL)) == 0) &&
		    (path != NULL))
			*path = devspec;
		return(rv);
	}

	/*
	 * Try to parse the device name off the beginning of the devspec
	 */
	return(kboot_parsedev(dev, devspec, path));
}

static vm_offset_t rsdp;

static vm_offset_t
kboot_rsdp_from_efi(void)
{
	int fd;
	char buffer[512 + 1];
	char *walker, *ep;
	ssize_t len;

	fd = host_open("/sys/firmware/efi/systab", O_RDONLY, 0);
	if (fd == -1)	/* Not an EFI system */
		return (0);
	len = host_read(fd, buffer, sizeof(buffer) - 1);
	close(fd);
	if (len <= 0)
		return (0);
	buffer[len] = '\0';
	ep = buffer + len;
	walker = buffer;
	while (walker < ep) {
		if (strncmp("ACPI20=", walker, 7) == 0)
			return((vm_offset_t)strtoull(walker + 7, NULL, 0));
		if (strncmp("ACPI=", walker, 5) == 0)
			return((vm_offset_t)strtoull(walker + 5, NULL, 0));
		walker += strcspn(walker, "\n");
	}
	return (0);
}

static void
find_acpi()
{
	rsdp = kboot_rsdp_from_efi();
#if 0	/* maybe for amd64 */
	if (rsdp == 0)
		rsdp = find_rsdp_arch();
#endif
}

vm_offset_t
acpi_rsdp()
{
	return (rsdp);
}

bool
has_acpi()
{
	return rsdp != 0;
}

int
main(int argc, const char **argv)
{
	void *heapbase;
	const size_t heapsize = 15*1024*1024;
	const char *bootdev;

	archsw.arch_getdev = kboot_getdev;
	archsw.arch_copyin = kboot_copyin;
	archsw.arch_copyout = kboot_copyout;
	archsw.arch_readin = kboot_readin;
	archsw.arch_autoload = kboot_autoload;
//	archsw.arch_loadaddr = kboot_loadaddr;
	archsw.arch_kexec_kseg_get = kboot_kseg_get;
#if defined(LOADER_ZFS_SUPPORT)
	archsw.arch_zfs_probe = kboot_zfs_probe;
#endif
	
	/* Give us a sane world if we're running as init */
	do_init();

	/*
	 * Setup the heap 15MB should be plenty
	 */
	heapbase = host_getmem(heapsize);
	setheap(heapbase, heapbase + heapsize);

	/*
	 * Set up console.
	 */
	cons_probe();

	for (int i = 0; devsw[i] != NULL; i++) {
		if (devsw[i]->dv_init != NULL) {
			(devsw[i]->dv_init)();
		}
	}

	/* Choose bootdev if provided */
	/* XXX should just set these are command line args */
//	if (argc > 1)
//		bootdev = argv[1];
//	else
		bootdev = "host:";
	if (argc > 2)
		hostfs_root = argv[2];

	printf("Boot device: %s with hostfs_root %s\n", bootdev, hostfs_root);

	printf("\n%s", bootprog_info);

	setenv("currdev", bootdev, 1);
	setenv("loaddev", bootdev, 1);
	setenv("LINES", "24", 1);
//	setenv("usefdt", "1", 1);

	/*
	 * Find acpi, if it exists
	 */
	find_acpi();

	memory_probe();

	interact();			/* doesn't return */

	return (0);
}

void
exit(int code)
{
	host_exit(code);
	__unreachable();
}

void
delay(int usecs)
{
	struct host_timeval tvi, tv;
	uint64_t ti, t;
	host_gettimeofday(&tvi, NULL);
	ti = tvi.tv_sec*1000000 + tvi.tv_usec;
	do {
		host_gettimeofday(&tv, NULL);
		t = tv.tv_sec*1000000 + tv.tv_usec;
	} while (t < ti + usecs);
}

time_t
getsecs(void)
{
	struct host_timeval tv;
	host_gettimeofday(&tv, NULL);
	return (tv.tv_sec);
}

time_t
time(time_t *tloc)
{
	time_t rv;
	
	rv = getsecs();
	if (tloc != NULL)
		*tloc = rv;

	return (rv);
}

struct host_kexec_segment loaded_segments[HOST_KEXEC_SEGMENT_MAX];
int nkexec_segments = 0;

#define SEGALIGN (1ul<<20)
	
static ssize_t
get_phys_buffer(vm_offset_t dest, const size_t len, void **buf)
{
	int i = 0;
	const size_t segsize = 64*1024*1024;

	if (nkexec_segments == HOST_KEXEC_SEGMENT_MAX)
		panic("Tried to load too many kexec segments");
	for (i = 0; i < nkexec_segments; i++) {
		if (dest >= (vm_offset_t)loaded_segments[i].mem &&
		    dest < (vm_offset_t)loaded_segments[i].mem +
		    loaded_segments[i].memsz)
			goto out;
	}

//	printf("GETSEG: Adding segment at %p size %zd\n", (void *)rounddown2(dest,SEGALIGN), segsize);
	
	loaded_segments[nkexec_segments].buf = host_getmem(segsize);
	loaded_segments[nkexec_segments].bufsz = segsize;
	loaded_segments[nkexec_segments].mem = (void *)rounddown2(dest,SEGALIGN);
	loaded_segments[nkexec_segments].memsz = segsize;

	i = nkexec_segments;
	nkexec_segments++;

out:
	*buf = loaded_segments[i].buf + (dest -
	    (vm_offset_t)loaded_segments[i].mem);
	return (min(len,loaded_segments[i].bufsz - (dest -
	    (vm_offset_t)loaded_segments[i].mem)));
}

ssize_t
kboot_copyin(const void *src, vm_offset_t dest, const size_t len)
{
	ssize_t segsize, remainder;
	void *destbuf;

	if (pa_start == PA_INVAL) {
		pa_start = kboot_get_phys_load_segment();
//		padding = 2 << 20;
		padding = 0;
		offset = dest;
//		printf("PA_START set to %#jx dst %#jx\n", (uintmax_t)pa_start, (uintmax_t)dest);
		get_phys_buffer(pa_start, len, &destbuf);
	}

//	printf("COPYIN dest %#jx pa_dst %#jx\n", (uintmax_t)dest, 
//	     (uintmax_t)pa_start + dest);
	remainder = len;
	do {
		segsize = get_phys_buffer(dest + pa_start + padding - offset, remainder, &destbuf);
		bcopy(src, destbuf, segsize);
		remainder -= segsize;
		src += segsize;
		dest += segsize;
	} while (remainder > 0);

	return (len);
}

ssize_t
kboot_copyout(vm_offset_t src, void *dest, const size_t len)
{
	ssize_t segsize, remainder;
	void *srcbuf;

	remainder = len;
	do {
		segsize = get_phys_buffer(src + pa_start + padding - offset, remainder, &srcbuf);
		bcopy(srcbuf, dest, segsize);
		remainder -= segsize;
		src += segsize;
		dest += segsize;
	} while (remainder > 0);

	return (len);
}

ssize_t
kboot_readin(readin_handle_t fd, vm_offset_t dest, const size_t len)
{
	void            *buf;
	size_t          resid, chunk, get;
	ssize_t         got;
	vm_offset_t     p;

//	printf("Reading in %zd bytes at %#jx\n", len, (uintmax_t)dest);
	p = dest;

	chunk = min(PAGE_SIZE, len);
	buf = malloc(chunk);
	if (buf == NULL) {
		printf("kboot_readin: buf malloc failed\n");
		return (0);
	}

	for (resid = len; resid > 0; resid -= got, p += got) {
		get = min(chunk, resid);
		got = VECTX_READ(fd, buf, get);
		if (got <= 0) {
			if (got < 0)
				printf("kboot_readin: read failed\n");
			break;
		}

		kboot_copyin(buf, p, got);
	}

	free (buf);
	return (len - resid);
}

int
kboot_autoload(void)
{

	return (0);
}

uint64_t
kboot_loadaddr(u_int type, void *data, uint64_t addr)
{

//	printf("LOADADDR: want addr %#jx ", (uintmax_t)addr);
	if (addr != 0)
		addr = roundup(addr, PAGE_SIZE);
//	else
//		printf("got addr %#jx\n", (uintmax_t)addr);
	return (addr);
}

static void
kboot_kseg_get(int *nseg, void **ptr)
{
	int a;

	printf("kseg_get: %d segments\n", nkexec_segments);
	printf("VA               SZ       PA               MEMSZ\n");
	printf("---------------- -------- ---------------- -----\n");
	for (a = 0; a < nkexec_segments; a++) {
		printf("%016jx %08jx %016jx %08jx\n",
			(uintmax_t)loaded_segments[a].buf,
			(uintmax_t)loaded_segments[a].bufsz,
			(uintmax_t)loaded_segments[a].mem,
			(uintmax_t)loaded_segments[a].memsz);
	}

	*nseg = nkexec_segments;
	*ptr = &loaded_segments[0];
}

#if defined(LOADER_ZFS_SUPPORT)
static void
kboot_zfs_probe(void)
{
	/*
	 * Open all the disks and partitions we can find to see if there are ZFS
	 * pools on them.
	 */
	hostdisk_zfs_probe(&pool_guid);
}
#endif


/*
 * Since proper fdt command handling function is defined in fdt_loader_cmd.c,
 * and declaring it as extern is in contradiction with COMMAND_SET() macro
 * (which uses static pointer), we're defining wrapper function, which
 * calls the proper fdt handling routine.
 */
static int
command_fdt(int argc, char *argv[])
{

	return (command_fdt_internal(argc, argv));
}
        
COMMAND_SET(fdt, "fdt", "flattened device tree handling", command_fdt);

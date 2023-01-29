/*-
 * Copyright (c) 2022 Netflix, Inc
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/efi.h>
#include <machine/metadata.h>
#include <sys/linker.h>
#include <fdt_platform.h>
#include <libfdt.h>

#include "kboot.h"
#include "bootstrap.h"

static struct memory_segments *segs;
static int nr_seg = 0;
static int segalloc = 0;

/*
 * Info from dtb about the EFI system
 */
vm_paddr_t efi_systbl_phys;
struct efi_map_header *efi_map_hdr;
uint32_t efi_map_size;
vm_paddr_t efi_map_phys_src;	/* From DTB */
vm_paddr_t efi_map_phys_dst;	/* From our memory map metadata module */

enum types {
	system_ram = 1,
	firmware_reserved,
	linux_code,
	linux_data,
	unknown,
};

struct kv
{
	uint64_t	type;
	char *		name;
	int		flags;
#define KV_KEEPER 1
} str2type_kv[] = {
	{ linux_code,		"Kernel code", KV_KEEPER },
	{ linux_data,		"Kernel data", KV_KEEPER },
	{ firmware_reserved,	"reserved" },
	{ 0, NULL },
};

static void
init_avail(void)
{
	if (segs)
		free(segs);
	nr_seg = 0;
	segalloc = 16;
	segs = malloc(sizeof(*segs) * segalloc);
}

/*
 * Make sure at least n items can be accessed in the segs array.  Note the
 * realloc here will invalidate cached pointers (potentially), so addresses
 * into the segs array must be recomputed after this call.
 */
static void
need_avail(int n)
{
	if (n <= segalloc)
		return;

	while (n > segalloc)
		segalloc *= 2;
	segs = realloc(segs, segalloc * sizeof(*segs));
}

/*
 * Always called for a new range, so always just append a range,
 * unless it's continuous with the prior range.
 */
static void
add_avail(uint64_t start, uint64_t end, enum types type)
{
	/*
	 * This range is contiguous with the previous range, and is
	 * the same type: we can collapse the two.
	 */
	if (nr_seg >= 1 &&
	    segs[nr_seg - 1].end + 1 == start &&
	    segs[nr_seg - 1].type == type) {
		segs[nr_seg - 1].end = end;
		return;
	}

	/*
	 * Otherwise we need to add a new range at the end, but don't need to
	 * adjust the current end.
	 */
	need_avail(nr_seg + 1);
	segs[nr_seg].start = start;
	segs[nr_seg].end = end;
	segs[nr_seg].type = type;
	nr_seg++;
}

/*
 * All or part of a prior entry needs to be modified. Given the structure of the
 * code, we know that it will always be modifying the last time and/or extending
 * the one before it if its contiguous.
 */
static void
remove_avail(uint64_t start, uint64_t end, enum types type)
{
	struct memory_segments *s;

	/*
	 * simple case: we are extending a previously removed item.
	 */
	if (nr_seg >= 2) {
		s = &segs[nr_seg - 2];
		if (s->end + 1 == start &&
		    s->type == type) {
			s->end = end;
			/* Now adjust the ending element */
			s++;
			if (s->end == end) {
				/* we've used up the 'free' space */
				nr_seg--;
				return;
			}
			/* Otherwise adjust the 'free' space */
			s->start = end + 1;
			return;
		}
	}

	/*
	 * OK, we have four cases:
	 * (1) The new chunk is at the start of the free space, but didn't catch the above
	 *     folding for whatever reason (different type, start of space). In this case,
	 *     we allocate 1 additional item. The current end is copied to the new end. The
	 *     current end is set to <start, end, type> and the new end's start is set to end + 1.
	 * (2) The new chunk is in the middle of the free space. In this case we allocate 2
	 *     additional items. We copy the current end to the new end, set the new end's start
	 *     to end + 1, the old end's end to start - 1 and the new item is <start, end, type>
	 * (3) The new chunk is at the end of the current end. In this case we allocate 1 more
	 *     and adjust the current end's end to start - 1 and set the new end to <start, end, type>.
	 * (4) The new chunk is exactly the current end, except for type. In this case, we just adjust
	 *     the type.
	 * We can assume we always have at least one chunk since that's created with new_avail() above
	 * necessarily before we are called to subset it.
	 */
	s = &segs[nr_seg - 1];
	if (s->start == start) {
		if (s->end == end) { /* (4) */
			s->type = type;
			return;
		}
		/* chunk at start of old chunk -> (1) */
		need_avail(nr_seg + 1);
		s = &segs[nr_seg - 1];	/* Realloc may change pointers */
		s[1] = s[0];
		s->start = start;
		s->end = end;
		s->type = type;
		s[1].start = end + 1;
		nr_seg++;
		return;
	}
	if (s->end == end) {	/* At end of old chunk (3) */
		need_avail(nr_seg + 1);
		s = &segs[nr_seg - 1];	/* Realloc may change pointers */
		s[1] = s[0];
		s->end = start - 1;
		s[1].start = start;
		s[1].type = type;
		nr_seg++;
		return;
	}
	/* In the middle, need to split things up (2) */
	need_avail(nr_seg + 2);
	s = &segs[nr_seg - 1];	/* Realloc may change pointers */
	s[2] = s[1] = s[0];
	s->end = start - 1;
	s[1].start = start;
	s[1].end = end;
	s[1].type = type;
	s[2].start = end + 1;
	nr_seg += 2;
}

static const char *
parse_line(const char *line, uint64_t *startp, uint64_t *endp)
{
	const char *walker;
	char *next;
	uint64_t start, end;

	/*
	 * Each line is a range followed by a descriptoin of the form:
	 * <hex-number><dash><hex-number><space><colon><space><string>
	 * Bail if we have any parsing errors.
	 */
	walker = line;
	start = strtoull(walker, &next, 16);
	if (start == ULLONG_MAX || walker == next)
		return (NULL);
	walker = next;
	if (*walker != '-')
		return (NULL);
	walker++;
	end = strtoull(walker, &next, 16);
	if (end == ULLONG_MAX || walker == next)
		return (NULL);
	walker = next;
	/* Now eat the ' : ' in front of the string we want to return */
	if (strncmp(walker, " : ", 3) != 0)
		return (NULL);
	*startp = start;
	*endp = end;
	return (walker + 3);
}

static struct kv *
kvlookup(const char *str, struct kv *kvs, size_t nkv)
{
	for (int i = 0; i < nkv; i++)
		if (strcmp(kvs[i].name, str) == 0)
			return (&kvs[i]);

	return (NULL);
}

/* Trim trailing whitespace */
static void
chop(char *line)
{
	char *ep = line + strlen(line) - 1;

	while (ep >= line && isspace(*ep))
		*ep-- = '\0';
}

#define SYSTEM_RAM "System RAM"
#define RESERVED "reserved"

static bool
do_memory_from_fdt(int fd)
{
	struct stat sb;
	char *buf = NULL;
	int len, offset, fd2 = -1;
	uint32_t sz, ver, esz, efisz;
	uint64_t mmap_pa;
	const uint32_t *u32p;
	const uint64_t *u64p;
	struct efi_map_header *efihdr;
	struct efi_md *map;

	if (fstat(fd, &sb) < 0)
		return false;
	buf = malloc(sb.st_size);
	if (buf == NULL)
		return false;
	len = read(fd, buf, sb.st_size);
	/* NB: we're reading this from sysfs, so mismatch OK */
	if (len <= 0)
		goto errout;

	/*
	 * Look for /chosen to find these values:
	 * linux,uefi-system-table	PA of the UEFI System Table.
	 * linux,uefi-mmap-start	PA of the UEFI memory map
	 * linux,uefi-mmap-size		Size of mmap
	 * linux,uefi-mmap-desc-size	Size of each entry of mmap
	 * linux,uefi-mmap-desc-ver	Format version, should be 1
	 */
	offset = fdt_path_offset(buf, "/chosen");
	if (offset <= 0)
		goto errout;
	u64p = fdt_getprop(buf, offset, "linux,uefi-system-table", &len);
	if (u64p == NULL)
		goto errout;
	efi_systbl_phys = fdt64_to_cpu(*u64p);
	u32p = fdt_getprop(buf, offset, "linux,uefi-mmap-desc-ver", &len);
	if (u32p == NULL)
		goto errout;
	ver = fdt32_to_cpu(*u32p);
	u32p = fdt_getprop(buf, offset, "linux,uefi-mmap-desc-size", &len);
	if (u32p == NULL)
		goto errout;
	esz = fdt32_to_cpu(*u32p);
	u32p = fdt_getprop(buf, offset, "linux,uefi-mmap-size", &len);
	if (u32p == NULL)
		goto errout;
	sz = fdt32_to_cpu(*u32p);
	u64p = fdt_getprop(buf, offset, "linux,uefi-mmap-start", &len);
	if (u64p == NULL)
		goto errout;
	mmap_pa = fdt64_to_cpu(*u64p);
	free(buf);

	printf("UEFI MMAP: Ver %d Ent Size %d Tot Size %d PA %#lx\n",
	    ver, esz, sz, mmap_pa);

	/*
	 * We may have no ability to read the PA that this map is in, so pass
	 * the address to FreeBSD via a rather odd flag entry as the first map
	 * so early boot can copy the memory map into this space and have the
	 * rest of the code cope.
	 */
	efisz = (sizeof(*efihdr) + 0xf) & ~0xf;
	buf = malloc(sz + efisz);
	if (buf == NULL)
		return false;
	efihdr = (struct efi_map_header *)buf;
	map = (struct efi_md *)((uint8_t *)efihdr + efisz);
	bzero(map, sz);
	efihdr->memory_size = sz;
	efihdr->descriptor_size = esz;
	efihdr->descriptor_version = ver;

	/*
	 * Save EFI table. Either this will be an empty table filled in by the trampiline,
	 * or we'll read it below. Either way, set these two variables so we share the best
	 * UEFI memory map with the kernel.
	 */
	efi_map_hdr = efihdr;
	efi_map_size = sz + efisz;

	/*
	 * Try to read in the actual UEFI map.
	 */
	fd2 = open("host:/dev/mem", O_RDONLY);
	if (fd2 < 0) {
		printf("Will read UEFI mem map in tramp: no /dev/mem, need CONFIG_DEVMEM=y\n");
		goto no_read;
	}
	if (lseek(fd2, mmap_pa, SEEK_SET) < 0) {
		printf("Will read UEFI mem map in tramp: lseek failed\n");
		goto no_read;
	}
	len = read(fd2, map, sz);
	if (len != sz) {
		if (len < 0 && errno == EPERM)
			printf("Will read UEFI mem map in tramp: kernel needs CONFIG_STRICT_DEVMEM=n\n");
		else
			printf("Will read UEFI mem map in tramp: lean = %d errno = %d\n", len, errno);
		goto no_read;
	}
	printf("Read UEFI mem map from physmem\n");
	efi_map_phys_src = 0; /* Mark MODINFOMD_EFI_MAP as valid */
	close(fd2);
	return true;	/* OK, we really have the memory map */

no_read:
	efi_map_phys_src = mmap_pa;
	close(fd2);
	return true;	/* We can get it the trampoline */

errout:
	close(fd2);
	free(buf);
	return false;
}

uint64_t commit_limit;
uint64_t committed_as;
uint64_t mem_avail;

bool
enumerate_memory_arch(void)
{
	int fd = -1;
	char buf[128];
	const char *str;
	uint64_t start, end;
	struct kv *kv;
	bool rv;

	/*
	 * To properly size the slabs, we need to find how much memory we can
	 * commit to using. commit_limit is the max, while commited_as is the
	 * current total. We can use these later to allocate the largetst amount
	 * of memory possible so we can support larger ram disks than we could
	 * by using fixed segment sizes. We also grab the memory available so
	 * we don't use more than 49% of that.
	 */
	fd = open("host:/proc/meminfo", O_RDONLY);
	if (fd != -1) {
		while (fgetstr(buf, sizeof(buf), fd) > 0) {
			if        (strncmp(buf, "MemAvailable:", 13) == 0) {
				mem_avail = strtoll(buf + 13, NULL, 0);
				mem_avail <<= 10; /* Units are kB */
			} else if (strncmp(buf, "CommitLimit:", 12) == 0) {
				commit_limit = strtoll(buf + 13, NULL, 0);
				commit_limit <<= 10; /* Units are kB */
			} else if (strncmp(buf, "Committed_AS:", 13) == 0) {
				committed_as = strtoll(buf + 14, NULL, 0);
				committed_as <<= 10; /* Units are kB */
			}
		}
	}
	printf("Commit limit: %lld Committed bytes %lld Available %lld\n",
	    (long long)commit_limit, (long long)committed_as,
	    (long long)mem_avail);
	close(fd);

	fd = open("host:/sys/firmware/fdt", O_RDONLY);
	if (fd != -1) {
		rv = do_memory_from_fdt(fd);
		close(fd);
		if (rv)
			printf("Found a physical memory for UEFI memory map\n");
		/*
		 * So, we have physaddr to the memory table. However, we can't
		 * open /dev/mem on some platforms to get the actual table. So
		 * we have to fall through to get it from /proc/iomem.
		 */
	}

	printf("Also reading /proc/iomem to learn of reserved areas\n");
	fd = open("host:/proc/iomem", O_RDONLY);
	if (fd == -1) {
		printf("Can't get memory map\n");
		return false;
	}

	if (fgetstr(buf, sizeof(buf), fd) < 0)
		goto out;	/* Nothing to do ???? */
	init_avail();
	chop(buf);
	while (true) {
		/*
		 * Look for top level items we understand.  Skip anything that's
		 * a continuation, since we don't care here. If we care, we'll
		 * consume them all when we recognize that top level item.
		 */
		if (buf[0] == ' ')	/* Continuation lines? Ignore */
			goto next_line;
		str = parse_line(buf, &start, &end);
		if (str == NULL)	/* Malformed -> ignore */
			goto next_line;
		/*
		 * All we care about is System RAM
		 */
		if (strncmp(str, SYSTEM_RAM, sizeof(SYSTEM_RAM) - 1) == 0)
			add_avail(start, end, system_ram);
		else if (strncmp(str, RESERVED, sizeof(RESERVED) - 1) == 0)
			add_avail(start, end, firmware_reserved);
		else
			goto next_line;	/* Ignore hardware */
		while (fgetstr(buf, sizeof(buf), fd) >= 0 && buf[0] == ' ') {
			chop(buf);
			str = parse_line(buf, &start, &end);
			if (str == NULL)
				break;
			kv = kvlookup(str, str2type_kv, nitems(str2type_kv));
			if (kv == NULL) /* failsafe for new types: igonre */
				remove_avail(start, end, unknown);
			else if ((kv->flags & KV_KEEPER) == 0)
				remove_avail(start, end, kv->type);
			/* Else no need to adjust since it's a keeper */
		}

		/*
		 * if buf[0] == ' ' then we know that the fgetstr failed and we
		 * should break. Otherwise fgetstr succeeded and we have a
		 * buffer we need to examine for being a top level item.
		 */
		if (buf[0] == ' ')
			break;
		chop(buf);
		continue; /* buf has next top level line to parse */
next_line:
		if (fgetstr(buf, sizeof(buf), fd) < 0)
			break;
	}

out:
	close(fd);

	printf("Found %d RAM segments:\n", nr_seg);
	for (int i = 0; i < nr_seg; i++) {
		printf("%#lx-%#lx type %lu\n", segs[i].start, segs[i].end, segs[i].type);
	}

	return true;
}

uint64_t
kboot_get_phys_load_segment(void)
{
#define HOLE_SIZE	(64ul << 20)
#define KERN_ALIGN	(2ul << 20)
	uint64_t	s;
	uint64_t	len;

	for (int i = 0; i < nr_seg; i++) {
		if (segs[i].type != system_ram)	/* Not candiate */
			continue;
		s = roundup(segs[i].start, KERN_ALIGN);
		if (s >= segs[i].end)		/* roundup past end */
			continue;
		len = segs[i].end - s + 1;
		if (len >= HOLE_SIZE) {
			printf("Found a big enough hole at in seg %d at %#lx (%#lx-%#lx)\n",
			    i, s, segs[i].start, segs[i].end);
			return (s);
		}
	}
	s = 0x40000000 | 0x4200000;	/* should never get here */
	printf("Falling back to crazy address %#lx\n", s);
	return (s);
}

void
bi_loadsmap(struct preloaded_file *kfp)
{
	char *buffer;
	struct efi_map_header *efihdr;
	struct efi_md *md;
	uint64_t sz, efisz, attr;
	uint32_t type;

	/*
	 * Make a note of a systbl. This is nearly mandatory on AARCH64.
	 */
	if (efi_systbl_phys)
		file_addmetadata(kfp, MODINFOMD_FW_HANDLE, sizeof(efi_systbl_phys), &efi_systbl_phys);

	/*
	 * If we have efi_map_hdr, then it's a pointer to the PA where this
	 * memory map lives. The trampoline code will copy it over. If we don't
	 * have it, we use whatever we found in /proc/iomap.
	 */
	if (efi_map_hdr != NULL) {
		file_addmetadata(kfp, MODINFOMD_EFI_MAP, efi_map_size, efi_map_hdr);
		return;
	}

	efisz = (sizeof(*efihdr) + 0xf) & ~0xf;
	sz = nr_seg * sizeof(*md);
	buffer = malloc(efisz + sz);
	efihdr = (struct efi_map_header *)buffer;
	md = (struct efi_md *)(buffer + efisz);
	for (int i = 0; i < nr_seg; i++) {
		switch (segs[i].type) {
		case system_ram:
		case linux_code:
		case linux_data:
			type = EFI_MD_TYPE_FREE;
			attr = 0;
			break;
		case firmware_reserved:
		case unknown:
			type = EFI_MD_TYPE_RT_DATA;
			attr = EFI_MD_ATTR_WB;
			break;
		}

		md[i].md_type = type;
		md[i].__pad = 0;
		md[i].md_phys = segs[i].start;
		md[i].md_virt = segs[i].start; /* VA == PA */
		md[i].md_pages = howmany(segs[i].end - segs[i].start + 1,
		    EFI_PAGE_SIZE);
		md[i].md_attr = attr;
	}
	efihdr->memory_size = sz;
	efihdr->descriptor_size = sizeof(*md);
	efihdr->descriptor_version = EFI_MEMORY_DESCRIPTOR_VERSION;
	file_addmetadata(kfp, MODINFOMD_EFI_MAP, efisz + sz, buffer);
	free(buffer);
}

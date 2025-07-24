/*-
 * Copyright (c) 2025 Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <dev/tpm/tpm20.h>
#include <sys/stat.h>

MALLOC_DEFINE(M_TPM20_EVENTLOG, "tpm_event_log_buffer",
    "buffer for tpm event log for tpm 2.0");

static d_open_t		tpm20_eventlog_open;
static d_close_t	tpm20_eventlog_close;
static d_read_t		tpm20_eventlog_read;
static d_ioctl_t	tpm20_eventlog_ioctl;

static struct uuid tcg2_final_event_table = EFI_TCG2_FINAL_EVENTS_TABLE_GUID;
static uint32_t get_minimum_event_size(TCG_PCR_EVENT *event_header);
static void remap_efi_table(struct tpm_eventlog_sc *sc);

static struct cdevsw tpm20_eventlog_cdevsw = {
	.d_version = D_VERSION,
	.d_open = tpm20_eventlog_open,
	.d_close = tpm20_eventlog_close,
	.d_read = tpm20_eventlog_read,
	.d_ioctl = tpm20_eventlog_ioctl,
	.d_name = "tpm20_eventlog",
};

static uint32_t
get_minimum_event_size(TCG_PCR_EVENT *event_header)
{
	TCG_EfiSpecIDEventStruct *efi_spec_id;
	uint32_t i, min_event_size = 0;

	min_event_size = offsetof(TCG_PCR_EVENT2, Digests);
	efi_spec_id = (TCG_EfiSpecIDEventStruct *)event_header->Event;

	for (i = 0; i < efi_spec_id->numberOfAlgorithms; i++) {
		min_event_size += efi_spec_id->digestSizes[i].digestSize;
		min_event_size += sizeof(efi_spec_id->digestSizes[i].digestSize);
	}
	min_event_size += sizeof(((TCG_PCR_EVENT2 *)0)->EventSize);
	return min_event_size;
}

static void
remap_efi_table(struct tpm_eventlog_sc *sc)
{
	pmap_unmapbios(sc->final_tbl_vaddr, sc->tbl_map_pages * PAGE_SIZE);
	sc->tbl_map_pages++;
	sc->final_tbl_vaddr = (EFI_TCG2_FINAL_EVENTS_TABLE *)pmap_mapbios(
	    sc->final_tbl_paddr, sc->tbl_map_pages * PAGE_SIZE);
}

int
tpm20_eventlog_read(struct cdev *dev, struct uio *uio, int flags)
{
	struct tpm_eventlog_sc *sc;
	TCG_PCR_EVENT *event_header;
	uint8_t *log_ptr = NULL, *off = NULL;
	uint32_t total_size = 0, event_size = 0;
	uint32_t i=0, bytes_to_transfer = 0;
	int error = 0;

	sc = (struct tpm_eventlog_sc *)dev->si_drv1;
	sx_xlock(&sc->dev_lock);
	if (sc->pending_data_length == 0 && sc->pending_final_event_log == 0) {
		sc->pending_data_length = sc->preloader_info->size;
		if (sc->final_tbl_vaddr != NULL)
			pmap_unmapbios(sc->final_tbl_vaddr,
			    sc->tbl_map_pages * PAGE_SIZE);
		sc->final_tbl_vaddr = NULL;
		goto exit_lock;
	}
	/* Determine the total size of the event log */
	if (sc->final_tbl_paddr != 0 && sc->final_tbl_vaddr == NULL &&
	    sc->pending_final_event_log == 0) {
		sc->final_tbl_vaddr =
		    (EFI_TCG2_FINAL_EVENTS_TABLE *)pmap_mapbios(sc->final_tbl_paddr,
		    sc->tbl_map_pages * PAGE_SIZE);
		off = (uint8_t *)(&sc->final_tbl_vaddr->Event);
		event_header = (TCG_PCR_EVENT *)(sc->preloader_info->events);
		for (i = 0; i < sc->final_tbl_vaddr->NumberOfEvents; i++) {
			event_size = get_tcg2_event_size((TCG_PCR_EVENT2 *)off,
			    event_header);
			if (event_size == 0) {
				device_printf(sc->parent_dev_sc->dev,
				    "Final Event Logs malformed\n");
				error = EIO;
				break;
			}
			total_size += event_size;
			off += event_size;
			if (((sc->tbl_map_pages * PAGE_SIZE) - sc->min_size) <=
			    total_size)
				remap_efi_table(sc);
		}

		if (total_size < sc->preloader_info->preloader_final_tblsz) {
			device_printf(sc->parent_dev_sc->dev,
			    "Final event table size is malformed\n");
		} else {
			total_size -= sc->preloader_info->preloader_final_tblsz;
			sc->pending_final_event_log = total_size;
		}
	}

	if (sc->pending_data_length > 0) {
		log_ptr = sc->preloader_info->events;
		bytes_to_transfer = MIN(uio->uio_resid, sc->pending_data_length);
		if (bytes_to_transfer > 0) {
			error = uiomove((caddr_t)log_ptr, bytes_to_transfer, uio);
			sc->pending_data_length -= bytes_to_transfer;
			if (error != 0)
				goto exit_lock;
		}
	}

	/* Copy the final event table data */
	if (sc->pending_final_event_log > 0 && sc->pending_data_length == 0) {
		log_ptr =  (uint8_t *)sc->final_tbl_vaddr->Event + \
		    sc->preloader_info->preloader_final_tblsz;
		bytes_to_transfer = MIN(uio->uio_resid, sc->pending_final_event_log);
		if (bytes_to_transfer > 0) {
			error = uiomove((caddr_t)log_ptr, bytes_to_transfer, uio);
			sc->pending_final_event_log -= bytes_to_transfer;
		}
	}

exit_lock:
	sx_xunlock(&sc->dev_lock);
	return error;
}

int
tpm20_eventlog_open(struct cdev *dev, int flag, int mode, struct thread *td)
{
	return (0);
}

int
tpm20_eventlog_close(struct cdev *dev, int flag, int mode, struct thread *td)
{
	return (0);
}

int
tpm20_eventlog_ioctl(struct cdev *dev, u_long cmd, caddr_t data,
    int flags, struct thread *td)
{
	return (ENOTTY);
}

int
tpm20_eventlog_init(struct tpm_sc *parent_dev_sc)
{
	struct make_dev_args args;
	struct tpm_eventlog_sc *sc;
	TCG_PCR_EVENT *event_header;
	EFI_TCG2_EVENT_LOG *info = NULL;
	EFI_TCG2_FINAL_EVENTS_TABLE *final_event_table = NULL;
	int result = 0;

	sc = &parent_dev_sc->eventlog_sc;
	info = (EFI_TCG2_EVENT_LOG *)preload_search_info(preload_kmdp,
	    MODINFO_METADATA | MODINFOMD_TCG2_LOGBUF);
	if (info == NULL) {
		device_printf(parent_dev_sc->dev,
		    "TCG2 event metadata is missing\n");
		return (ENXIO);
	}
	if (info->size == 0) {
		device_printf(parent_dev_sc->dev,
		    "TCG2 event log size must be > 0\n");
		return (ENXIO);
	}
	if (info->version != EFI_TCG2_EVENT_LOG_FORMAT_TCG_2) {
		device_printf(parent_dev_sc->dev,
		    "TCG2 event log unsupported version: %d\n", info->version);
		return (ENXIO);
	}

	sc->preloader_info = info;
	sc->parent_dev_sc = parent_dev_sc;
	sc->pending_data_length = info->size;
	sc->pending_final_event_log = 0;

	result = efi_get_table(&tcg2_final_event_table,
	    (void **)&final_event_table);
	if (result != 0) {
		device_printf(parent_dev_sc->dev,
		    "No final events table registered for TPM\n");
		sc->final_tbl_vaddr = NULL;
		sc->tbl_map_pages = 0;
	} else {
		event_header = (TCG_PCR_EVENT *)(sc->preloader_info->events);
		sc->min_size = get_minimum_event_size(event_header);
		sc->tbl_map_pages = (sc->min_size + PAGE_SIZE - 1) / PAGE_SIZE;
		sc->final_tbl_paddr =(vm_paddr_t)(uintptr_t)final_event_table;
	}

	sx_init(&sc->dev_lock, "TPM Event Log driver lock");
	make_dev_args_init(&args);
	args.mda_devsw = &tpm20_eventlog_cdevsw;
	args.mda_uid = UID_ROOT;
	args.mda_gid = GID_WHEEL;
	args.mda_mode = TPM_CDEV_PERM_FLAG;
	args.mda_si_drv1 = sc;
	result = make_dev_s(&args, &sc->log_cdev, TPM_EVENTLOG_CDEV_NAME);
	if (result != 0)
		tpm20_eventlog_release(parent_dev_sc);
	return (result);
}

void
tpm20_eventlog_release(struct tpm_sc *parent_dev_sc)
{
	if (parent_dev_sc->eventlog_sc.log_cdev != NULL) {
		sx_destroy(&parent_dev_sc->eventlog_sc.dev_lock);
		destroy_dev(parent_dev_sc->eventlog_sc.log_cdev);
	}
}

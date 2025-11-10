/*-
 * Copyright (c) 2025 Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stand.h>
#include <efi.h>
#include <efilib.h>

#include <Protocol/Tcg2Protocol.h>

#define TCG2_SPECID_SIGNATURE "Spec ID Event03"

static EFI_GUID tcg2_final_event_table = EFI_TCG2_FINAL_EVENTS_TABLE_GUID;
static EFI_GUID tcg2_guid = EFI_TCG2_PROTOCOL_GUID;
static uint32_t get_final_table_events_size(EFI_TCG2_FINAL_EVENTS_TABLE *,
	TCG_PCR_EVENT *);


static inline bool
validate_tcg2_event(TCG_PCR_EVENT2 *event,
    TCG_EfiSpecIDEventStruct *efi_spec_id)
{

	if (event->Digest.count != efi_spec_id->numberOfAlgorithms)
		return false;
	return true;
}

/*
 * Calculate the size of the TCG2 event. TCG2 event can contain more than one
 * digests and can also contain variable length of data that is extended and
 * logged. The first event provides metadata information the digest tyoe.
 * Returns 0 if there is a failure in determining the event length.
 */
static inline uint32_t
get_tcg2_event_size(TCG_PCR_EVENT2 *event, TCG_PCR_EVENT *event_header)
{
	TCG_EfiSpecIDEventStruct *efi_spec_id;
	uint8_t *off;
	uint32_t i, j, digests_count;
	uint32_t event_size = 0;
	uint16_t hashid;

	efi_spec_id = (TCG_EfiSpecIDEventStruct *)event_header->Event;
	if (!validate_tcg2_event(event, efi_spec_id)) {
		printf("TCG2 final event validation failed\n");
		return 0;
	}

	off = (uint8_t *)(&event->Digest);
	digests_count = event->Digest.count;
	for (i = 0; i < digests_count; i++) {
		hashid = *((uint16_t *)(off));
		for (j = 0; j < digests_count; j++) {
			if (hashid == efi_spec_id->digestSize[j].algorithmId) {
				off += sizeof(hashid);
				off += efi_spec_id->digestSize[j].digestSize;
				break;
			}
		}
		if (j == digests_count) {
			printf("TCG2 event hash algo ID:(%d) not supported\n",
			    hashid);
			return 0;
		}
	}
	off += *(uint32_t *)(off) + sizeof(event->EventSize);
	event_size = off - (uint8_t *)event;
	return event_size;
}

/*
 * Logic is derived from TCG EFI Protocol Spec Level 00, Revision 00.13
 * Section 7 explains the minimal flow to retrieve event log.
 * https://trustedcomputinggroup.org/wp-content/uploads/EFI-Protocol-Specification-rev13-160330final.pdf
 */
static inline bool
validate_tcg_eventlog_header(TCG_PCR_EVENT *event_header)
{
	TCG_EfiSpecIDEventStruct *efi_spec_id;
	const uint8_t zero_buf[SHA1_DIGEST_SIZE] = {0};

	if (event_header->PCRIndex != 0 ||
	    event_header->EventType != EV_NO_ACTION ||
	    memcmp(event_header->Digest.digest, zero_buf, sizeof(zero_buf)))
		return false;

	efi_spec_id = (TCG_EfiSpecIDEventStruct *)event_header->Event;
	if (memcmp(efi_spec_id->signature, TCG2_SPECID_SIGNATURE,
		sizeof(TCG2_SPECID_SIGNATURE)) ||
		efi_spec_id->numberOfAlgorithms == 0)
		return false;

	return true;
}

/*
 * Get the TCG2 event log buffer from the UEFI firmware. The event log buffer
 * is allocated and populated using the UEFI variables
 * "EFI_TCG2_FINAL_EVENTS_TABLE_GUID" and "EFI_TCG2_PROTOCOL_GUID".
 */
EFI_TCG2_EVENT_LOG *
efitcg_get_event_log(void)
{
	EFI_TCG2_FINAL_EVENTS_TABLE *final_events_table = NULL;
	EFI_TCG2_PROTOCOL *tcg2_protocol = NULL;
	EFI_PHYSICAL_ADDRESS first_event_addr = 0;
	EFI_PHYSICAL_ADDRESS last_event_addr = 0;	/* Begining of event */
	EFI_TCG2_EVENT_LOG *event_log = NULL;
	struct TCG_PCR_EVENT2 *table_event = NULL;
	EFI_STATUS status = 0;
	uint32_t total_event_size = 0;
	uint32_t size = 0, final_tbl_size = 0;
	BOOLEAN event_log_truncated = false;

	status = BS->LocateProtocol(&tcg2_guid, NULL, (VOID **)&tcg2_protocol);
	if (status != EFI_SUCCESS) {
		printf("Failed to locate TCG2 Protocol (%lu)\n",
		    EFI_ERRCODE(status));
		return NULL;
	}
	/* Currently Log format 1.2 is not supported */
	status = tcg2_protocol->GetEventLog(tcg2_protocol,
	    EFI_TCG2_EVENT_LOG_FORMAT_TCG_2, &first_event_addr,
	    &last_event_addr, &event_log_truncated);

	if (status != EFI_SUCCESS || first_event_addr == 0) {
		printf("Failed to get TCG eventlog buffer (%lu)\n",
		    EFI_ERRCODE(status));
		return NULL;
	}

	if (event_log_truncated)
		printf("Warning:TPM event logs were truncated\n");

	if (!validate_tcg_eventlog_header((TCG_PCR_EVENT *)first_event_addr)) {
		printf("TCG event log header validation failed\n");
		return NULL;
	}

	if (last_event_addr != 0) {
		size = get_tcg2_event_size((TCG_PCR_EVENT2 *)last_event_addr,
		    (TCG_PCR_EVENT *)first_event_addr);
		/* If size calculation failed, exclude last event */
		if (size == 0)
			printf("Failed to calculate size of last TCG event.\n");
		total_event_size = last_event_addr - first_event_addr + size;
	}

	printf("TPM event log size: %u\n", total_event_size);
	event_log = malloc(sizeof(*event_log) + total_event_size);
	if (event_log == NULL) {
		printf("Failed to allocate memory for TPM event log struct\n");
		return NULL;
	}

	final_events_table = efi_get_table(&tcg2_final_event_table);
	if (final_events_table == NULL)
		printf("No final events table registered for TPM event log\n");
	else if (final_events_table->NumberOfEvents > 0)
		final_tbl_size = get_final_table_events_size(final_events_table,
		    (TCG_PCR_EVENT *)first_event_addr);
	event_log->preloader_final_tblsz = final_tbl_size;
	event_log->version = EFI_TCG2_EVENT_LOG_FORMAT_TCG_2;
	event_log->size = total_event_size;
	memcpy(event_log->events, (void *)first_event_addr, total_event_size);

	return event_log;
}

static uint32_t
get_final_table_events_size(EFI_TCG2_FINAL_EVENTS_TABLE *final_tbl,
	TCG_PCR_EVENT *event_header)
{
	TCG_PCR_EVENT2 *event;
	uint8_t *offset;
	uint32_t i, event_size = 0, total_events_size = 0;

//	offset = (uint8_t *)&(final_tbl->Event);
	offset = (uint8_t *)&final_tbl[1];
	for (i = 0; i < final_tbl->NumberOfEvents; i++) {
		event = (TCG_PCR_EVENT2 *)offset;
		event_size = get_tcg2_event_size(event, event_header);
		if (event_size == 0) {
			printf("Event log is malformed in final event table\n");
			break;
		}
		total_events_size += event_size;
		offset += event_size;
	}
	return total_events_size;
}

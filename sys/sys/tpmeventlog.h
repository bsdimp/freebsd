/*
 * Copyright (c) 2025 Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_SYS_TPMEVENTLOG_H_
#define	_SYS_TPMEVENTLOG_H_


#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/stddef.h>
#if defined (_KERNEL)
#include <sys/efi.h>
#include "opt_tpm.h"
/* Ask Warner how to handle following header inclusion */
#define INTERFACE_DECL(x)
#define EFIAPI
#include <contrib/dev/acpica/include/actypes.h>
typedef uintptr_t UINTN;
typedef UINT64 EFI_PHYSICAL_ADDRESS;
typedef UINTN EFI_STATUS;
#define IN
#define OUT
#define CONST const
#define OPTIONAL
/* Ask Warner how to handle common efitcg.h */
#include <sys/efitcg.h>
typedef struct {
  uint32_t    Data1;
  uint16_t    Data2;
  uint16_t    Data3;
  uint8_t     Data4[8];
} EFI_GUID;
#elif  defined(_STANDALONE)
#include <stdio.h>
#include <stdint.h>
#include <stand.h>
#include <efi.h>
#include <efilib.h>
#endif

#if defined(_STANDALONE) || defined(DEV_TPM)
typedef struct efi_tcg2_event_log {
	uint32_t	size;
	uint32_t	preloader_final_tblsz;
	uint8_t		version;
	uint8_t		events[];
} EFI_TCG2_EVENT_LOG;

static inline bool validate_tcg2_event(TCG_PCR_EVENT2 *,
    TCG_EfiSpecIDEventStruct *);
static inline uint32_t get_tcg2_event_size(TCG_PCR_EVENT2 *, TCG_PCR_EVENT *);

static inline bool
validate_tcg2_event(TCG_PCR_EVENT2 *event,
    TCG_EfiSpecIDEventStruct *efi_spec_id)
{

	if (event->Count != efi_spec_id->numberOfAlgorithms)
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

	off = (uint8_t *)(&event->Digests);
	digests_count = event->Count;
	for (i = 0; i < digests_count; i++) {
		hashid = *((uint16_t *)(off));
		for (j = 0; j < digests_count; j++) {
			if (hashid == efi_spec_id->digestSizes[j].algorithmId) {
				off += sizeof(hashid);
				off += efi_spec_id->digestSizes[j].digestSize;
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

#endif /* _STANDALONE || DEV_TPM */
#endif /* _SYS_TPMEVENTLOG_H_ */

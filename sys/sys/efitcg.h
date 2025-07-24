/*
 * Copyright (c) 2025 Netflix, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * @file TPM2 Protocol as defined in TCG PC Client Platform EFI Protocol
 * Specification Family "2.0". See http://trustedcomputinggroup.org for
 * the latest specification
 *
 * Copyright (c) 2015 - 2018, Intel Corporation. All rights reserved.
 */

#pragma once

#if defined(_KERNEL)

#define EFI_TCG2_EVENT_LOG_FORMAT_TCG_1_2   0x1
#define EFI_TCG2_EVENT_LOG_FORMAT_TCG_2     0x2
#define TCG2_SPECID_SIGNATURE               "Spec ID Event03"
/* Always set 5 here, because we want to support all hash algo in BIOS */
#define HASH_COUNT  5
#define SHA1_DIGEST_SIZE  20
#define SHA256_DIGEST_SIZE  32
#define SHA384_DIGEST_SIZE  48
#define SHA512_DIGEST_SIZE  64
#define SM3_256_DIGEST_SIZE  32

typedef struct tdEFI_TCG2_PROTOCOL EFI_TCG2_PROTOCOL;

enum TCG_PCR_EVENT2_TYPES {
	EV_PREBOOT_CERT = 0,
	EV_POST_CODE,
	EV_UNUSED,
	EV_NO_ACTION,
	EV_SEPARATOR,
	EV_ACTION,
	EV_EVENT_TAG,
	EV_SCRTM_CONTENTS,
	EV_SCRTM_VERSION,
	EV_CPU_MICROCODE,
	EV_PLATFORM_CONFIG_FLAGS,
	EV_TABLE_OF_DEVICES,
	EV_COMPACT_HASH,
	EV_IPL,
	EV_IPL_PARTITION_DATA,
	EV_NONHOST_CODE,
	EV_NONHOST_CONFIG,
	EV_NONHOST_INFO,
    EV_OMIT_BOOT_DEVICE_EVENTS,
};

typedef struct {
	UINT8 Major;
	UINT8 Minor;
} EFI_TCG2_VERSION;

typedef UINT32 EFI_TCG2_EVENT_ALGORITHM_BITMAP;
typedef UINT32 EFI_TCG2_EVENT_LOG_BITMAP;
typedef UINT32 EFI_TCG2_EVENT_LOG_FORMAT;
typedef UINT32 TCG_PCRINDEX;
typedef UINT32 TCG_EVENTTYPE;

typedef struct {
	UINT8 			Size;
	EFI_TCG2_VERSION	StructureVersion;
	EFI_TCG2_VERSION	ProtocolVersion;
	EFI_TCG2_EVENT_ALGORITHM_BITMAP	HashAlgorithmBitmap;
	EFI_TCG2_EVENT_LOG_BITMAP	SupportedEventLogs;
	BOOLEAN		TPMPresentFlag;
	UINT16		MaxCommandSize;
	UINT16		MaxResponseSize;
	UINT32		ManufacturerID;
	UINT32		NumberOfPcrBanks;
	EFI_TCG2_EVENT_ALGORITHM_BITMAP	ActivePcrBanks;
} EFI_TCG2_BOOT_SERVICE_CAPABILITY;

typedef struct {
	UINT32		HeaderSize;
	UINT16		HeaderVersion;
	TCG_PCRINDEX	PCRIndex;
	TCG_EVENTTYPE	EventType;
} EFI_TCG2_EVENT_HEADER;

typedef struct {
	UINT32			Size;
	EFI_TCG2_EVENT_HEADER	Header;
	UINT8			Event[];
} EFI_TCG2_EVENT;

//
// To avoid a recursive problem for the parsers, the log header is defined to
// be in SHA1 event log entry format.
//
typedef struct {
	TCG_PCRINDEX	PCRIndex;
	TCG_EVENTTYPE	EventType;
	UINT8		Digest[SHA1_DIGEST_SIZE];
	UINT32		EventSize;
	UINT8		Event[];
} TCG_PCR_EVENT;

typedef union {
	UINT8	sha1[SHA1_DIGEST_SIZE];
	UINT8	sha256[SHA256_DIGEST_SIZE];
	UINT8	sm3_256[SM3_256_DIGEST_SIZE];
	UINT8	sha384[SHA384_DIGEST_SIZE];
	UINT8	sha512[SHA512_DIGEST_SIZE];
} TPMU_HA;

typedef struct {
	UINT16	AlgorithmId;
	TPMU_HA	Digest;
} TPMT_HA;

typedef struct {
	TCG_PCRINDEX	PCRIndex;
	TCG_EVENTTYPE	EventType;
	UINT32		Count;
	TPMT_HA		Digests[HASH_COUNT];
	UINT32		EventSize;
	UINT8		Event[];
} TCG_PCR_EVENT2;

typedef struct {
	UINT64		Version;
	UINT64		NumberOfEvents;
	TCG_PCR_EVENT2	Event[];
} EFI_TCG2_FINAL_EVENTS_TABLE;

typedef struct {
	UINT16	algorithmId;
	UINT16	digestSize;
} TCG_EfiSpecIdEventAlgorithmSize;

typedef struct {
	UINT8				signature[16];
	UINT32				platformClass;
	UINT8				specVersionMinor;
	UINT8				specVersionMajor;
	UINT8				specErrata;
	UINT8				uintnSize;
	UINT32				numberOfAlgorithms;
	TCG_EfiSpecIdEventAlgorithmSize	digestSizes[];
} TCG_EfiSpecIDEventStruct;
typedef
EFI_STATUS
(EFIAPI *EFI_TCG2_GET_CAPABILITY) (
	IN EFI_TCG2_PROTOCOL			*This,
	IN OUT EFI_TCG2_BOOT_SERVICE_CAPABILITY	*ProtocolCapability
);

typedef
EFI_STATUS
(EFIAPI *EFI_TCG2_GET_EVENT_LOG) (
	IN  EFI_TCG2_PROTOCOL		*This,
	IN  EFI_TCG2_EVENT_LOG_FORMAT	EventLogFormat,
	OUT EFI_PHYSICAL_ADDRESS	*EventLogLocation,
	OUT EFI_PHYSICAL_ADDRESS	*EventLogLastEntry,
	OUT BOOLEAN			*EventLogTruncated
);

typedef
EFI_STATUS
(EFIAPI * EFI_TCG2_HASH_LOG_EXTEND_EVENT) (
	IN  EFI_TCG2_PROTOCOL	*This,
	IN  UINT64			Flags,
	IN  EFI_PHYSICAL_ADDRESS	DataToHash,
	IN  UINT64			DataToHashLen,
	IN  EFI_TCG2_EVENT		*EfiTcgEvent
);

typedef
EFI_STATUS
(EFIAPI *EFI_TCG2_SUBMIT_COMMAND) (
	IN  EFI_TCG2_PROTOCOL	*This,
	IN  UINT32			InputParameterBlockSize,
	IN  UINT8			*InputParameterBlock,
	IN  UINT32			OutputParameterBlockSize,
	IN  UINT8			*OutputParameterBlock
);

typedef
EFI_STATUS
(EFIAPI *EFI_TCG2_GET_ACTIVE_PCR_BANKS) (
	IN  EFI_TCG2_PROTOCOL	*This,
	OUT UINT32		*ActivePcrBanks
);

typedef
EFI_STATUS
(EFIAPI *EFI_TCG2_SET_ACTIVE_PCR_BANKS) (
	IN  EFI_TCG2_PROTOCOL	*This,
	IN  UINT32			ActivePcrBanks
);

typedef
EFI_STATUS
(EFIAPI * EFI_TCG2_GET_RESULT_OF_SET_ACTIVE_PCR_BANKS) (
	IN  EFI_TCG2_PROTOCOL	*This,
	OUT UINT32		*OperationPresent,
	OUT UINT32		*Response
);

struct tdEFI_TCG2_PROTOCOL {
	EFI_TCG2_GET_CAPABILITY		GetCapability;
	EFI_TCG2_GET_EVENT_LOG		GetEventLog;
	EFI_TCG2_HASH_LOG_EXTEND_EVENT	HashLogExtendEvent;
	EFI_TCG2_SUBMIT_COMMAND		SubmitCommand;
	EFI_TCG2_GET_ACTIVE_PCR_BANKS	GetActivePcrBanks;
	EFI_TCG2_SET_ACTIVE_PCR_BANKS	SetActivePcrBanks;
	EFI_TCG2_GET_RESULT_OF_SET_ACTIVE_PCR_BANKS	GetResultOfSetActivePcrBanks;
};

#define EFI_TCG2_PROTOCOL_GUID \
{ 0x607f766c, 0x7455, 0x42be, {0x93, 0x0b, 0xe4, 0xd7, 0x6d, 0xb2, 0x72, 0x0f} }

#if defined (_KERNEL)
#define EFI_TCG2_FINAL_EVENTS_TABLE_GUID \
{ 0x1e2ed096, 0x30e2, 0x4254, 0xbd, 0x89, {0x86, 0x3b, 0xbe, 0xf8, 0x23, 0x25} }
#elif defined (_STANDALONE)
#define EFI_TCG2_FINAL_EVENTS_TABLE_GUID \
{ 0x1e2ed096, 0x30e2, 0x4254, {0xbd, 0x89, 0x86, 0x3b, 0xbe, 0xf8, 0x23, 0x25} }
#endif /* _KERNEL */
#endif /* _KERNEL */

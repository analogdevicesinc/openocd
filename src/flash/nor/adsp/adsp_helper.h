/* SPDX-License-Identifier: GPL-2.0-or-later */
// Copyright (C) 2022-2026 Analog Devices, Inc.

#ifndef ADSP_HELPER_H
#define ADSP_HELPER_H

#include "spi/adsp_spi.h"
#include <helper/time_support.h>
#include <helper/bits.h>
#include <target/algorithm.h>
#include <target/image.h>
#include "../spi.h"
#include "../imp.h"
#include "helper/binarybuffer.h"
#include <target/xtensa/xtensa_chip.h>
#include <target/xtensa/xtensa.h>
#include <target/armv7a.h>
 #include <string.h>

#define SPI_NAME ".spi"
#define EMMC_NAME ".emmc"
#define OTP_NAME ".otp"

#define PARAMETER_FILE_COUNT 6

#define BYTE_COUNT 8
#define ALGO_READY 0xFFFFFFFF

#define ALGO_TIMEOUT_KEEP_ALIVE 500
#define ALGO_TIMEOUT_MAX 20000
#define ALGO_TIMEOUT_MAX_MASS_ERASE 800000

#define SPI_MAX_READ_COUNT 0xFFFF

#define ROUNDUP(value, limit) ((((value) + (limit) - 1) / (limit)) * (limit))
#define BANK_NAME_IS(bank, str) (strstr((bank)->name, (str)))

#define LOG_ERROR_ALGO_PARAMS(algo_params) \
	LOG_ERROR("Address offset: 0x%8.8" PRIx32 \
				" Length in bytes:0x%8.8" PRIx32 \
				" Flash command: 0x%8.8" PRIx32 \
				" Status: 0x%8.8" PRIx32 \
				" Readiness: 0x%8.8" PRIx32 \
				" Device ID: 0x%8.8" PRIx32, \
				algo_params.address, \
				algo_params.length, \
				algo_params.command, \
				algo_params.status, \
				algo_params.ready, \
				algo_params.device_id)

/*** xtensa resume handling ***/
#define USE_ADDRESS_VAL 0
#define USE_PC_VAL 1

#define HANDLE_BREAKPOINTS 0
#define SKIP_BREAKPOINTS 1

#define NO_DEBUG_EXECUTION 0
#define DEBUG_EXECUTION 1
/******************************/

/* Flash helper algorithm parameter block struct */
#define ADSP_STATUS_OFFSET 0x0C
#define ADSP_READID_OFFSET 0x14

struct custom_algorithm {
	uint8_t *adsp_algo;
	unsigned long algo_start_address;
	unsigned long reset_handler_addr;
	unsigned long parameter_address;  /* Values derived from algorithm for data buffer */
	unsigned long buffer_address;	  /*  address and algo parameter address (g_cfg)*/
	unsigned long version;
	unsigned long size;
	unsigned long buffer_size;
};

struct adsp_algo_params {
	uint32_t address;
	uint32_t length;
	uint32_t command;
	uint32_t status;
	uint32_t ready;
	uint32_t device_id;
} __attribute__((packed, aligned(4)));

/* Internal data structure to allow additional options for flash device */
struct adsp_flash_bank {
	bool probed;				/*! Has the flash device been probed? */
	uint32_t available_space;	/*! Used for sanity checking against memory leaks */
	struct working_area *working_area;
	struct xtensa_algorithm xtensa_info;
	struct flash_device dev;
	struct custom_algorithm adsp_algorithm;
	union {
		struct xtensa_algorithm xtensa_info;
		struct armv7a_algorithm arm7a_info;
		/* extend with other arch structs here */
	} target_info;
	uint32_t sectorsize;
	uint32_t size_in_bytes;
	struct adsp_algo_params algo_params;
};

int adsp_quit(struct flash_bank *bank);
int adsp_wait_algo_done(struct flash_bank *bank, uint32_t params_addr);
int wait_for_breakpoint_and_check_status(struct flash_bank *bank, long long timeout);
int adsp_init(struct flash_bank *bank);
int adsp_run_flash_command(struct flash_bank *bank, long long timeout);
int adsp_target_poll_check_state(struct flash_bank *bank, enum target_state expected_state, long long timeout);

#endif // ADSP_HELPER_H

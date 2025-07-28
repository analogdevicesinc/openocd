/* SPDX-License-Identifier: GPL-2.0-or-later */
// Copyright (C) 2022-2024 Analog Devices, Inc.

#ifndef ADSP2183X_H
#define ADSP2183X_H

struct custom_algorithm {
	uint8_t *adsp2183x_algo;
	unsigned long algo_start_address;
	unsigned long reset_handler_addr;
	unsigned long parameter_address;  /* Values derived from algorithm for data buffer */
	unsigned long buffer_address;	  /*  address and algo parameter address (g_cfg)*/
	unsigned long version;
	unsigned long size;
	unsigned long buffer_size;
};

#define PARAMETER_FILE_COUNT 6

#define SPI_NAME "adsp2183x.spi"

#define BYTE_COUNT 8
#define ALGO_READY 0xFFFFFFFF

#define ALGO_TIMEOUT_KEEP_ALIVE 500
#define ALGO_TIMEOUT_MAX 20000
#define ALGO_TIMEOUT_MAX_MASS_ERASE 500000

enum FLASH_COMMANDS {
	READ_COMMAND = 1,
	PROGRAM_COMMAND = 2,
	LOCK_COMMAND = 3,
	MASS_ERASE_COMMAND = 4,
	SECTOR_ERASE_COMMAND = 5,
	READ_ID_CODE_COMMAND = 6
};

/*** xtensa resume handling ***/
#define USE_ADDRESS_VAL 0
#define USE_PC_VAL 1

#define HANDLE_BREAKPOINTS 0
#define SKIP_BREAKPOINTS 1

#define NO_DEBUG_EXECUTION 0
#define DEBUG_EXECUTION 1
/******************************/

/* Flash helper algorithm parameter block struct */
#define ADSP83X_STATUS_OFFSET 0x0C
#define ADSP83X_READID_OFFSET 0x14

struct adsp2183x_algo_params {
	uint32_t address;
	uint32_t length;
	uint32_t command;
	uint32_t status;
	uint32_t ready;
	uint32_t device_id;
} __attribute__((packed, aligned(4)));

#endif // ADSP2183X_H

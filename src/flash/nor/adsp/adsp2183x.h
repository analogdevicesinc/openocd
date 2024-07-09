/* SPDX-License Identifier: GPL-2.0-or-later
	Copyright (C) 2022-2024 Analog Devices, Inc. */


struct custom_algorithm {
	uint8_t *adsp2183x_algo;
	unsigned long algo_start_address;
	unsigned long reset_handler_addr;
	unsigned long parameter_address;  /* Values derived from algorithm for data buffer */
	unsigned long buffer_address;	  /*  address and algo parameter address (g_cfg)*/
	unsigned long version;
	unsigned long size;
};

#define PARAMETER_FILE_COUNT 5

#define SPI_NAME "adsp2183x.spi"

#define BYTE_COUNT 8
#define ALGO_READY 0xFFFFFFFF

#define ALGO_TIMEOUT_KEEP_ALIVE 500
#define ALGO_TIMEOUT_MAX 20000
#define ALGO_TIMEOUT_MAX_MASS_ERASE 500000

enum FLASH_COMMANDS
{
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
	uint8_t address[4];
	uint8_t length[4];
	uint8_t command[4];
	uint8_t status[4];
	uint8_t ready[4];
	uint8_t device_id[4];
};
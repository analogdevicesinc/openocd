// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
 *	Copyright (C) 2022-2024 Analog Devices, Inc.							*
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "spi/adsp_spi.h"
#include <helper/time_support.h>
#include <helper/bits.h>
#include <target/algorithm.h>
#include <target/image.h>
#include "../spi.h"
#include "helper/binarybuffer.h"
#include <target/xtensa/xtensa_chip.h>
#include <target/xtensa/xtensa.h>
#include "adsp2183x_otp.h"

#define ROUNDUP(value, limit) (((value) + (limit) - ((value) % (limit))))

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

/* Internal data structure to allow additional options for flash device */
struct adsp2183x_otp_bank {
	bool probed;				/*! Has the flash device been probed? */
	uint32_t available_space;	/*! Used for sanity checking against memory leaks */
	uint32_t sector_length;
	struct working_area *working_area;
	struct xtensa_algorithm xtensa_info;
	struct custom_algorithm adsp2183x_algorithm;
	uint32_t sectorsize;
	uint32_t size_in_bytes;
};

static int adsp83x_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	int retval;

	/* Regardless of the algo's status, attempt to halt the target */
	retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	/* Now confirm target halted and clean up from flash helper algorithm */
	retval = target_wait_algorithm(target, 0, NULL, 0, NULL, 0, ALGO_TIMEOUT_MAX,
				&adsp2183x_otp_info->xtensa_info);

	target_free_working_area(target, adsp2183x_otp_info->working_area);
	adsp2183x_otp_info->working_area = NULL;

	return retval;
}

static int adsp83x_wait_algo_done(struct flash_bank *bank, uint32_t params_addr)
{
	struct target *target = bank->target;
	uint32_t status_addr = params_addr + ADSP83X_STATUS_OFFSET;
	uint32_t status = ALGO_READY;
	long long start_ms;
	long long elapsed_ms;
	int retval = ERROR_OK;

	start_ms = timeval_ms();
	while (status == ALGO_READY) {
		retval = target_read_u32(target, status_addr, &status);
		if (retval != ERROR_OK)
			return retval;

		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > ALGO_TIMEOUT_KEEP_ALIVE)
			keep_alive();
		if (elapsed_ms > ALGO_TIMEOUT_MAX)
			break;
	};

	if (status != 0)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int wait_for_breakpoint_and_check_status(struct flash_bank *bank, long long timeout)
{
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	int retval;
	long long start_ms;
	long long elapsed_ms;
	long long timeout_ms;

	timeout_ms = timeout;

	start_ms = timeval_ms();

	// poll target to update state and wait for algorithm to hit breakpoint to halt target
	while (target->state != TARGET_HALTED) {
		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > ALGO_TIMEOUT_KEEP_ALIVE)
			keep_alive();
		if (elapsed_ms > timeout_ms) {
			LOG_ERROR("Timeout during algorithm command execution");
			/* Close down algo */
			(void)adsp83x_quit(bank);
			return ERROR_FAIL;
		}

		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}
	}

	// get status from buffer to determine result of algorithm initialization
	retval = adsp83x_wait_algo_done(bank, adsp2183x_otp_info->adsp2183x_algorithm.parameter_address);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error detected in algorithm command execution. Closing down algorithm.");
		/* Close down algo */
		(void)adsp83x_quit(bank);
	}	else {
		// Resume running algorithm with parameters
		xtensa_resume(target, USE_PC_VAL, 0, SKIP_BREAKPOINTS, DEBUG_EXECUTION);

	/*
	* At this point, the algorithm is running on the target and
	* ready to receive commands and data to flash the target
	*/
	}

	return retval;
}


static int adsp2183x_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	int retval;

	if (!adsp2183x_otp_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	/* Check for working area to use for flash helper algorithm */
	adsp2183x_otp_info->working_area = NULL;

	retval = target_alloc_working_area(target, adsp2183x_otp_info->available_space,
				&adsp2183x_otp_info->working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Working address is not correctly allocated");
		return retval;
	}

	// Need to halt before reads/writes
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Target is not halted!");
		target_free_working_area(target, adsp2183x_otp_info->working_area);
		adsp2183x_otp_info->working_area = NULL;
		return retval;
	}

	// poll target to update state
	retval = target_poll(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adsp2183x_otp_info->working_area);
		adsp2183x_otp_info->working_area = NULL;
		return retval;
	}

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, adsp2183x_otp_info->adsp2183x_algorithm.algo_start_address,
				adsp2183x_otp_info->adsp2183x_algorithm.size, adsp2183x_otp_info->adsp2183x_algorithm.adsp2183x_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load flash helper algorithm");
		target_free_working_area(target, adsp2183x_otp_info->working_area);
		adsp2183x_otp_info->working_area = NULL;
		return retval;
	}

	/* Initialize the Xtensa specific info to run the algorithm */
	adsp2183x_otp_info->xtensa_info.core_mode = XT_MODE_ANY;

	/* Begin executing the flash helper algorithm */
	retval = target_start_algorithm(target, 0, NULL, 0, NULL,
				adsp2183x_otp_info->adsp2183x_algorithm.reset_handler_addr, 0, &adsp2183x_otp_info->xtensa_info);
	if (retval != ERROR_OK) {
		target_free_working_area(target, adsp2183x_otp_info->working_area);
		adsp2183x_otp_info->working_area = NULL;
		LOG_ERROR("Failure starting the algorithm");
		return retval;
	}

	retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

	return retval;
}

/**
 * Erase whole memory
 * Usage:
 * adsp2183x mase_erase bank_id
*/

/**
 * Erase the specified sectors
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	first	The number of the first sector to erase
 * @param	last	The number of the last sector to erase
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/

// No erase for otp
static int adsp2183x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	LOG_ERROR("Erase not available for this device");
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/**
 * Write the 'count' number of bytes from the buffer at the offset specified
 * for otp using the flash bank provided.
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	buffer	Data to write to the otp
 * @param	offset	Offset from base to write to
 * @param	count	Number of bytes to write to otp
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/
static int adsp2183x_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	struct adsp2183x_algo_params algo_params;
	int retval;
	int i, byteCountOrig, byteCountRounded;
	uint32_t convertedHex;

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		retval = ERROR_TARGET_NOT_HALTED;
	} else if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
	} else {
		/* All good to proceed */
		retval = adsp2183x_init(bank);
		if (retval != ERROR_OK)
			return retval;

		byteCountOrig = (count / 2) + (count % 2);

		if (byteCountOrig % 4 != 0)
			byteCountRounded = ROUNDUP(byteCountOrig, 4);
		else
			byteCountRounded = byteCountOrig;

		uint32_t tempBuf[byteCountRounded / 4];
		uint32_t sendBuf[byteCountRounded / 4];

		for (i = 0;	i < byteCountRounded / 4; i++) {
			memcpy(tempBuf, buffer, BYTE_COUNT);
			buffer += BYTE_COUNT;
			convertedHex = (uint32_t)strtoul((void *)tempBuf, NULL, 16);
			sendBuf[i] = convertedHex;
		}

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}

		// poll target to update state
		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}

		// Issue program command to algorithm
		algo_params.command = PROGRAM_COMMAND;

		/* Put next block of data to flash into buffer */
		retval = target_write_buffer(target, adsp2183x_otp_info->adsp2183x_algorithm.buffer_address,
			byteCountRounded, (void *)sendBuf);

		if (byteCountRounded < 4)
			byteCountRounded = 4;

		// write algo parameters

		algo_params.address = offset;
		algo_params.length = byteCountRounded;
		algo_params.ready = ALGO_READY;

		/* Put next block of data to flash into buffer */
		retval = target_write_buffer(target, adsp2183x_otp_info->adsp2183x_algorithm.parameter_address,
					sizeof(algo_params), (uint8_t *)&algo_params);

		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write data to target memory");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}

		// Resume running algorithm with parameters
		xtensa_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

		retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

		if (retval != ERROR_OK) {
			/* Close down algo */
			(void)adsp83x_quit(bank);
			LOG_ERROR_ALGO_PARAMS(algo_params);
			return retval;
		}
	}

	return retval;
}

/**
 * Read the 'count' number of bytes from the buffer at the offset specified
 * in the otp area using the flash bank provided. If called when device has
 * already been probed, previously filled fields will be discard and probe
 * procedure will be done again.
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	buffer	Buffer to read data from the otp area into
 * @param	offset	Offset from base to read from
 * @param	count	Number of bytes to read from otp area
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/
static int adsp2183x_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	struct adsp2183x_algo_params algo_params;

	int retval;
	uint32_t byteCountRounded;

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		retval = ERROR_TARGET_NOT_HALTED;
	} else {
		/* All good to proceed */
		retval = adsp2183x_init(bank);
		if (retval != ERROR_OK)
			return retval;

		if (count % 4 != 0)
			byteCountRounded = ROUNDUP(count, 4);
		else
			byteCountRounded = count;

		uint8_t recBuf[byteCountRounded];

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}

		// poll target to update state
		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}

		// hardcode to issue read command to algorithm
		algo_params.command = READ_COMMAND;

		// write algo parameters
		algo_params.address = offset;
		algo_params.length = byteCountRounded;
		algo_params.ready = ALGO_READY;

		retval = target_write_buffer(target, adsp2183x_otp_info->adsp2183x_algorithm.parameter_address,
					sizeof(algo_params), (uint8_t *)&algo_params);

		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to read data from target memory");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}

		// Resume running algorithm with parameters
		xtensa_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

		// poll target to update state and wait for algorithm to hit breakpoint to halt target
		while (target->state != TARGET_HALTED) {
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adsp2183x_otp_info->working_area);
				adsp2183x_otp_info->working_area = NULL;
				return retval;
			}
		}

		/* Put next block of data from flash into buffer */
		retval = target_read_buffer(target, adsp2183x_otp_info->adsp2183x_algorithm.buffer_address,
		byteCountRounded, recBuf);

		memcpy(buffer, recBuf, count);

		if (retval != ERROR_OK) {
			LOG_ERROR_ALGO_PARAMS(algo_params);
			/* Close down algo */
			(void)adsp83x_quit(bank);
			return retval;
		}

		retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

		if (retval != ERROR_OK) {
			/* Close down algo */
			(void)adsp83x_quit(bank);
			LOG_ERROR_ALGO_PARAMS(algo_params);
			return retval;
		}
	}

	return retval;
}

/**
 * Probe the otp area to set up the target side algorithm and update the bank
 * appropriately.
 *
 * @param	bank	Pointer to the flash bank to use and write to
 *
 * @returns	ERROR_OK if successful otherwise the relevant code.
*/
static int adsp2183x_probe(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	uint32_t sector_length = 0;
	int num_sectors;

	if (!adsp2183x_otp_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	LOG_INFO("Setting up ADSP2183X otp area...");

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	target_free_all_working_areas(target);

	/* Output available working memory on target */
	uint32_t available_space = target_get_working_area_avail(target);
	uint32_t target_start_address = (uint32_t)target->working_area_phys;
	LOG_INFO("Target has %uB of available space.", available_space);
	adsp2183x_otp_info->available_space = available_space;

	// Check algorithm size vs allocated flash bank space
	if (adsp2183x_otp_info->adsp2183x_algorithm.size > adsp2183x_otp_info->available_space) {
		LOG_ERROR("Not enough available space in flash bank %s for corresponding algorithm of size %lu", bank->name,
			adsp2183x_otp_info->adsp2183x_algorithm.size);
		return ERROR_FAIL;
	}

	// Check start address of algorithm with address provided in cfg
	if (adsp2183x_otp_info->adsp2183x_algorithm.algo_start_address < target_start_address
		|| adsp2183x_otp_info->adsp2183x_algorithm.algo_start_address > (target_start_address + adsp2183x_otp_info->available_space)) {
		LOG_ERROR("Start address for corresponding algorithm of %lu is not within the allocated range of %u for flash bank %s",
			adsp2183x_otp_info->adsp2183x_algorithm.algo_start_address, target_start_address + adsp2183x_otp_info->available_space, bank->name);
		return ERROR_FAIL;
	}

	// poll target to update state
	retval = target_poll(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adsp2183x_otp_info->working_area);
		adsp2183x_otp_info->working_area = NULL;
		return ERROR_FAIL;
	}

	num_sectors = 1;

	bank->size = num_sectors * adsp2183x_otp_info->sectorsize;
	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;
	bank->num_sectors = num_sectors;

	sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (!sectors)
		return ERROR_FAIL;

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sector_length;
		sectors[sector].size = sector_length;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;

	adsp2183x_otp_info->probed = true;

	return retval;
}

/**
 * Called by some other commands before proceeding with their main function to
 * ensure device is properly probed and known. Mostly a wrapper from the probe
 * function with a stored probe state.
 *
 * @param	bank	Pointer to the flash bank to use and write to
 *
 * @returns	ERROR_OK if successful otherwise the relevant code.
*/
static int adsp2183x_auto_probe(struct flash_bank *bank)
{
	int retval;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;

	if (!adsp2183x_otp_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	if (adsp2183x_otp_info->probed)
		retval = ERROR_OK;
	else
		retval = adsp2183x_probe(bank);

	return retval;
}

/**
* Not yet supported for otp.
*/
static int adsp2183x_protect_check(struct flash_bank *bank)
{
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/**
* Not yet supported for otp.
*/
static int adsp2183x_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/**
 * Copy the flash device info into the provided buffer.
 *
 * @param	bank		Pointer to the flash bank to print get info about
 * @param   cmd         Pointer to the command invocation
 *
 * @returns	ERROR_OK if successful otherwise the relevant code.
*/
static int adsp2183x_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;

	if (!adsp2183x_otp_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	int retval = adsp2183x_probe(bank);
	if (retval != ERROR_OK)
		return retval;


	command_print(cmd, "ADSP-2183X OTP\n"
			"Size: 0x%X\n",
			adsp2183x_otp_info->sector_length);

	return ERROR_OK;
}

/**
 * Usage:
 * flash bank <name> adsp2183x_otp <base_addr> 0 0 0 <target> sector_size algorithm_file param_file
*/
FLASH_BANK_COMMAND_HANDLER(adsp2183x_otp_bank_command)
{
	struct adsp2183x_otp_bank *adsp2183x_otp_info;
	int byteCount = 0;
	int count = 0;
	uint32_t tempParse;
	char tempStr[2];
	uint8_t convertedHex;
	FILE *algo_file;
	FILE *parameter_file;
	bool insideComment = true;
	char line[256];
	char parameter_file_data[PARAMETER_FILE_COUNT][9];  // Assuming each hex value is of length 8

	/* Check the correct number of arguments have been provided */
	if (CMD_ARGC != 9) {
		LOG_ERROR("Invalid number of flash bank arguments. Usage:\n"
			"flash bank <name> adsp2183x_otp <base_addr> 0 0 0 <target> "
			"sector_size algorithm_file parameter_file");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* Check to see if openocd is being used for non-flashing */
	if (strlen(CMD_ARGV[7]) == 0 && strlen(CMD_ARGV[8]) == 0) {
		LOG_WARNING("Flashing will not work without corresponding .inc files");
		return ERROR_OK;
	}

	adsp2183x_otp_info = malloc(sizeof(struct adsp2183x_otp_bank));
	if (!adsp2183x_otp_info) {
		LOG_ERROR("Not enough memory for local driver information.");
		return ERROR_FAIL;
	}

    // Opening file in reading mode
	algo_file = fopen(CMD_ARGV[7], "r");

	if (NULL == algo_file) {
		LOG_ERROR("File %s can't be opened\n", CMD_ARGV[7]);
		return -1;
	}

     /* Size of file */
	fseek(algo_file, 0, SEEK_END);
	adsp2183x_otp_info->adsp2183x_algorithm.size = ftell(algo_file);
	fseek(algo_file, 0, SEEK_SET);
	adsp2183x_otp_info->adsp2183x_algorithm.adsp2183x_algo = malloc(sizeof(uint8_t) * adsp2183x_otp_info->adsp2183x_algorithm.size);
	// Get 2 characters at a time to form byte. Convert byte and
	// store into spi algorithm buffer
	do {
		tempStr[0] = fgetc(algo_file);
		// read/skip over LF (UNIX) and CR (Windows)
		if (tempStr[0] != '\n' && tempStr[0] != '\r') {
			tempStr[1] = fgetc(algo_file);
			convertedHex = strtoul((const char *)tempStr, NULL, 16);
			adsp2183x_otp_info->adsp2183x_algorithm.adsp2183x_algo[byteCount] = convertedHex;
			byteCount++;
		}
		// Checking if character is not EOF.
		// If it is EOF stop reading.
	} while (tempStr[1] != EOF);

    // Closing the file
	fclose(algo_file);

	// Opening file in reading mode
	parameter_file = fopen(CMD_ARGV[8], "r");

	if (NULL == parameter_file) {
		LOG_ERROR("File %s can't be opened\n", CMD_ARGV[8]);
		return -1;
	}

    // Loop through each line in the file
	while (fgets(line, sizeof(line), parameter_file)) {
		// Assuming the hex values are written one per line
		// You may need to adjust the logic based on the actual file structure
		if (strstr(line, "*/")) {
			insideComment = false;
			continue;
		}

		if (!insideComment) {
			// Copy the last 8 characters (hex value) to the array
			if (sscanf(line, "%8s", parameter_file_data[count]) == 1) {
				count++;

				// Break the loop if we have found the correct number of elements
				if (count == PARAMETER_FILE_COUNT)
					break;
			}
		}
	}

	// Close the file
	fclose(parameter_file);

	adsp2183x_otp_info->adsp2183x_algorithm.parameter_address = strtoul(parameter_file_data[0], NULL, 16);
	adsp2183x_otp_info->adsp2183x_algorithm.buffer_address = strtoul(parameter_file_data[1], NULL, 16);
	adsp2183x_otp_info->adsp2183x_algorithm.reset_handler_addr = strtoul(parameter_file_data[2], NULL, 16);
	adsp2183x_otp_info->adsp2183x_algorithm.algo_start_address = strtoul(parameter_file_data[3], NULL, 16);
	adsp2183x_otp_info->adsp2183x_algorithm.version = strtoul(parameter_file_data[4], NULL, 10);
	adsp2183x_otp_info->adsp2183x_algorithm.buffer_size = strtoul(parameter_file_data[5], NULL, 16);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], tempParse);
	adsp2183x_otp_info->sectorsize = tempParse;

	adsp2183x_otp_info->probed = false;
	bank->driver_priv = adsp2183x_otp_info;

	return ERROR_OK;
}

// No erase for otp
COMMAND_HANDLER(adsp2183x_mass_erase_handler)
{
	LOG_ERROR("Mass erase not available for this device");
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/**
 * Get algorithm version number
 * Usage:
 * adsp2183x get_algorithm_version bank_id
*/
COMMAND_HANDLER(adsp2183x_get_algorithm_version_handler)
{
	struct adsp2183x_otp_bank *adsp2183x_otp_info;
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	adsp2183x_otp_info = bank->driver_priv;

	if (!adsp2183x_otp_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	command_print(CMD, "%lu", adsp2183x_otp_info->adsp2183x_algorithm.version);

	return retval;
}

static const struct command_registration adsp2183x_exec_command_handlers[] = {
	{
		.name		= "mass_erase",
		.handler	= adsp2183x_mass_erase_handler,
		.mode		= COMMAND_EXEC,
		.usage		= "bank_id",
		.help		= "Mass erase entire flash device.",
	},
	{
		.name		= "get_algorithm_version",
		.handler	= adsp2183x_get_algorithm_version_handler,
		.mode		= COMMAND_EXEC,
		.usage		= "bank_id",
		.help		= "Get algorithm version.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration adsp2183x_command_handlers[] = {
	{
		.name	= "adsp2183x_otp",
		.mode	= COMMAND_ANY,
		.help	= "adsp2183x flash command group",
		.usage	= "",
		.chain	= adsp2183x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


const struct flash_driver adsp2183x_otp = {
	.name				= "adsp2183x_otp",
	.commands			= adsp2183x_command_handlers,
	.flash_bank_command	= adsp2183x_otp_bank_command,
	.erase				= adsp2183x_erase,
	.protect			= adsp2183x_protect,
	.write				= adsp2183x_write,
	.read				= adsp2183x_read,
	.probe				= adsp2183x_probe,
	.auto_probe			= adsp2183x_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= adsp2183x_protect_check,
	.info				= adsp2183x_get_info,
	.free_driver_priv	= default_flash_free_driver_priv,
};

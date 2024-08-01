// SPDX-License-Identifier: GPL-2.0+
// Copyright (C) 2022-2024 Analog Devices, Inc.

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "../imp.h"
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "../spi.h"
#include <target/armv7a.h>
#include "adspsc59x.h"

#define SPI_MAX_READ_COUNT 0xFFFF

#define LOG_ERROR_ALGO_PARAMS(algo_params) \
	LOG_ERROR("Address offset: %08X " \
				"Length in bytes: %08X " \
				"Flash command: %08X " \
				"Status: %08X " \
				"Readiness: %08X " \
				"Device ID: %08X ", \
				*(uint32_t *)algo_params.address, \
				*(uint32_t *)algo_params.length, \
				*(uint32_t *)algo_params.command, \
				*(uint32_t *)algo_params.status, \
				*(uint32_t *)algo_params.ready, \
				*(uint32_t *)algo_params.device_id)

/* Internal data structure to allow additional options for flash device */
struct adspsc59x_flash_bank {
	bool probed;				/*! Has the flash device been probed? */
	uint32_t available_space;	/*! Used for sanity checking against memory leaks */
	struct working_area *working_area;
	struct armv7a_algorithm arm7a_info;
	struct flash_device	dev;
	struct custom_algorithm adspsc59x_algorithm;
	uint32_t sectorsize;
	uint32_t size_in_bytes;
};

static int adspsc59x_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	int retval;

	/* Regardless of the algo's status, attempt to halt the target */
	retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	/* Now confirm target halted and clean up from flash helper algorithm */
	retval = target_wait_algorithm(target, 0, NULL, 0, NULL, 0, ALGO_TIMEOUT_MAX,
				&adspsc59x_flash_info->arm7a_info);

	target_free_working_area(target, adspsc59x_flash_info->working_area);
	adspsc59x_flash_info->working_area = NULL;

	return retval;
}

static int adspsc59x_wait_algo_done(struct flash_bank *bank, uint32_t params_addr)
{
	struct target *target = bank->target;
	uint32_t status_addr = params_addr + ADSPSC59X_STATUS_OFFSET;
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
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
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
			return ERROR_FAIL;
		}

		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return retval;
		}
	}

	// get status from buffer to determine result of algorithm initialization
	retval = adspsc59x_wait_algo_done(bank, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error detected in algorithm command execution. Closing down algorithm.");
		/* Close down algo */
		(void)adspsc59x_quit(bank);
	} else {
		// Resume running algorithm with parameters
		target_resume(target, USE_PC_VAL, 0, SKIP_BREAKPOINTS, DEBUG_EXECUTION);

	/*
	 * At this point, the algorithm is running on the target and
	 * ready to receive commands and data to flash the target
	 */
	}

	return retval;
}


static int adspsc59x_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	struct adspsc59x_algo_params algo_params;
	int retval;


	/* Check for working area to use for flash helper algorithm */
	adspsc59x_flash_info->working_area = NULL;

	retval = target_alloc_working_area(target, adspsc59x_flash_info->available_space,
				&adspsc59x_flash_info->working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Working address is not correctly allocated");
		return retval;
	}

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address,
				adspsc59x_flash_info->adspsc59x_algorithm.size, adspsc59x_flash_info->adspsc59x_algorithm.adspsc59x_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load flash helper algorithm");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return retval;
	}

	retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address,
				sizeof(algo_params), (uint8_t *)&algo_params);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load flash helper algorithm");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return retval;
	}

	/* Initialize the ARMV7 specific info to run the algorithm */
	adspsc59x_flash_info->arm7a_info.core_mode = ARM_MODE_SVC;
	adspsc59x_flash_info->arm7a_info.core_state = ARM_STATE_THUMB;
	adspsc59x_flash_info->arm7a_info.common_magic = ARMV7_COMMON_MAGIC;

	/* Begin executing the flash helper algorithm */
	retval = target_start_algorithm(target, 0, NULL, 0, NULL,
				adspsc59x_flash_info->adspsc59x_algorithm.reset_handler_addr, 0, &adspsc59x_flash_info->arm7a_info);
	if (retval != ERROR_OK) {
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		LOG_ERROR("Failure starting the algorithm");
		return retval;
	}

	retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

	if (retval == ERROR_OK) {
		// initialize algorithm parameters to 0
		buf_set_u32(algo_params.command, 0, 32, 0);
		buf_set_u32(algo_params.address, 0, 32, 0);
		buf_set_u32(algo_params.ready,  0, 32, 0);
		buf_set_u32(algo_params.length,  0, 32, 0);
		buf_set_u32(algo_params.status,  0, 32, 0);
		buf_set_u32(algo_params.device_id,  0, 32, 0);
	}

	return retval;
}

/**
 * Erase the specified sectors within the flash
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	first	The number of the first sector to erase
 * @param	last	The number of the last sector to erase
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/
static int adspsc59x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	struct adspsc59x_algo_params algo_params;

	int retval;

	if (!adspsc59x_flash_info->probed) {
		LOG_ERROR("Cannot erase flash as target has not been probed. Please probe target first.");
		retval = ERROR_FLASH_BANK_NOT_PROBED;
	}	else {
		LOG_INFO("Erasing sectors %u to %u (inclusive) in flash", first, last);
		uint32_t address;

		// poll target to update state
		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return ERROR_FAIL;
		}

		// Check if algorithm is running, if not run it
		if (target->state != TARGET_DEBUG_RUNNING) {
			retval = adspsc59x_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		for (unsigned int counter = first; counter <= last; counter++) {
			/* Calculate the address based on the counter and configured sector size */
			address = counter * adspsc59x_flash_info->sectorsize;

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			// poll target to update state
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (TARGET_HALTED != target->state) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}


			// hardcode to issue read command to algorithm
			buf_set_u32(algo_params.command, 0, 32, SECTOR_ERASE_COMMAND);

			// write algo parameters
			buf_set_u32(algo_params.address, 0, 32, address);
			buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

			retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address,
					sizeof(algo_params), (uint8_t *)&algo_params);

			// Resume running algorithm with parameters
			target_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

			retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

			if (retval != ERROR_OK) {
				LOG_ERROR_ALGO_PARAMS(algo_params);
				return retval;
			}
		}
	}

	return retval;
}

/**
 * Write the 'count' number of bytes from the buffer at the offset specified
 * for memory region using the flash bank provided.
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	buffer	Data to write to the spi flash
 * @param	offset	Offset from base to write to
 * @param	count	Number of bytes to write to spi flash
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/
static int adspsc59x_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	struct adspsc59x_algo_params algo_params;
	int retval;
	unsigned write_size = 0;
	unsigned buffer_index = 0;
	uint32_t current_address = offset;

	if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
	}	else {
		// poll target to update state
		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return ERROR_FAIL;
		}

		// Check if algorithm is running, if not run it
		if (target->state != TARGET_DEBUG_RUNNING && target->state != TARGET_RUNNING) {
			retval = adspsc59x_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		/* First write any bytes if the specified offset if not on the sector size boundary */
		if (0 != (current_address % adspsc59x_flash_info->sectorsize)) {
						/* Calculate the write size to use, the modulo remainder of the page size  (unless the specified count is smaller) */
			write_size = adspsc59x_flash_info->sectorsize - (current_address % adspsc59x_flash_info->sectorsize);
			if (write_size > count)
				write_size = count;

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			// poll target to update state
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (TARGET_HALTED != target->state) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}

			// Issue program command to algorithm
			buf_set_u32(algo_params.command, 0, 32, PROGRAM_COMMAND);

			/* Put next block of data to flash into buffer */
			retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.buffer_address,
			write_size, &buffer[buffer_index]);

			// write algo parameters
			buf_set_u32(algo_params.address, 0, 32, current_address);
			buf_set_u32(algo_params.length, 0, 32, write_size);
			buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

			/* Put next block of data to flash into buffer */
			retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address,
					sizeof(algo_params), (uint8_t *)&algo_params);

			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to write data to target memory");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			// Resume running algorithm with parameters
			target_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

			retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

			if (retval != ERROR_OK) {
				LOG_ERROR_ALGO_PARAMS(algo_params);
				return retval;
			}

			/* Increment the index and address */
			current_address += write_size;
			buffer_index += write_size;

			LOG_INFO("Written %u/%u bytes. Current address is 0x%08X", buffer_index, count, current_address);
		}

		/* Write remaining data */
		while (count - buffer_index) {
			/* If the remaining bytes is less than the flash sectot size,
			*  size is just the remaining bytes...
			*/
			if ((count - buffer_index) < adspsc59x_flash_info->sectorsize)
				write_size = count - buffer_index;
			/* Otherwise size is the page size (max size that can be written in one command) */
			else
				write_size = adspsc59x_flash_info->sectorsize;

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			// poll target to update state
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (TARGET_HALTED != target->state) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}

			// Issue program command to algorithm
			buf_set_u32(algo_params.command, 0, 32, PROGRAM_COMMAND);

			/* Put next block of data to flash into buffer */
			retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.buffer_address,
			write_size, &buffer[buffer_index]);


			// write algo parameters

			buf_set_u32(algo_params.address, 0, 32, current_address);
			buf_set_u32(algo_params.length, 0, 32, write_size);
			buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

			/* Put next block of data to flash into buffer */
			retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address,
					sizeof(algo_params), (uint8_t *)&algo_params);

			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to write data to target memory");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}

			// Resume running algorithm with parameters
			target_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

			retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

			if (retval != ERROR_OK) {
				LOG_ERROR_ALGO_PARAMS(algo_params);
				return retval;
			}

			/* Increment the index and address */
			current_address += write_size;
			buffer_index += write_size;
		}
	}

	return retval;
}

/**
 * Read the 'count' number of bytes from the buffer at the offset specified
 * in the corresponding memory region using the flash bank provided. If called when device has
 * already been probed, previously filled fields will be discard and probe
 * procedure will be done again.
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	buffer	Buffer to read data from the memory region into
 * @param	offset	Offset from base to read from
 * @param	count	Number of bytes to read from memory region
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/
static int adspsc59x_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	struct adspsc59x_algo_params algo_params;
	int retval;

	// poll target to update state
	retval = target_poll(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return ERROR_FAIL;
	}

	// Check if algorithm is running, if not run it
	if (target->state != TARGET_DEBUG_RUNNING) {
		retval = adspsc59x_init(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Make sure read is not larger than buffer size to be passed */
	uint32_t read_bytes = 0;
	while (count) {
		/* Maximum read size*/
		uint32_t read_size = SPI_MAX_READ_COUNT;

		/* Then if the actual count is smaller than the theoretical max, use the count */
		if (count < read_size)
			read_size = count;

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return retval;
		}

		// poll target to update state
		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return retval;
		}

		/* Check device is halted and has been probed first */
		if (TARGET_HALTED != target->state) {
			LOG_ERROR("Cannot read from flash. Target is not halted!");
			return ERROR_TARGET_NOT_HALTED;
		}

		/* Check we're not going reading more than we should */
		assert(count >= read_size);


		uint32_t address = offset + read_bytes;

		// hardcode to issue read command to algorithm
		buf_set_u32(algo_params.command, 0, 32, READ_COMMAND);

		// write algo parameters
		buf_set_u32(algo_params.address, 0, 32, address);
		buf_set_u32(algo_params.length, 0, 32, read_size);
		buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

		retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address,
				sizeof(algo_params), (uint8_t *)&algo_params);

		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to read data from target memory");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return retval;
		}

		// Resume running algorithm with parameters
		target_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);
		// poll target to update state and wait for algorithm to hit breakpoint to halt target
		while (target->state != TARGET_HALTED) {
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adspsc59x_flash_info->working_area);
				adspsc59x_flash_info->working_area = NULL;
				return retval;
			}
		}
		/* Put next block of data from flash into buffer */
		retval = target_read_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.buffer_address,
		read_size, &buffer[read_bytes]);
		if (retval != ERROR_OK) {
			LOG_ERROR_ALGO_PARAMS(algo_params);
			/* Close down algo */
			(void)adspsc59x_quit(bank);
			return retval;
		}

		retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

		if (retval != ERROR_OK) {
			LOG_ERROR_ALGO_PARAMS(algo_params);
			return retval;
		}

		/* Increment the index and address */
		read_bytes += read_size;
		count -= read_size;
	}

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
static int adspsc59x_auto_probe(struct flash_bank *bank)
{
	int retval;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	struct target *target = bank->target;

	if (adspsc59x_flash_info->probed)
		return ERROR_OK;

	LOG_INFO("Setting up flash area for %s...", bank->name);

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	target_free_all_working_areas(target);

	/* Output available working memory on target */
	uint32_t available_space = target_get_working_area_avail(target);
	uint32_t target_start_address = (uint32_t)target->working_area_phys;
	LOG_INFO("Target has %uB of available space.", available_space);
	adspsc59x_flash_info->available_space = available_space;

	// Check algorithm size vs allocated flash bank space
	if (adspsc59x_flash_info->adspsc59x_algorithm.size > adspsc59x_flash_info->available_space) {
		LOG_ERROR("Not enough available space in flash bank %s for corresponding algorithm of size %lu", bank->name,
			adspsc59x_flash_info->adspsc59x_algorithm.size);
		return ERROR_FAIL;
	}

	// Check start address of algorithm with address provided in cfg
	if (adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address < target_start_address
		|| adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address > (target_start_address + adspsc59x_flash_info->available_space)) {
		LOG_ERROR("Start address for corresponding algorithm of %lu is not within the allocated range of %u for flash bank %s",
			adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address, target_start_address + adspsc59x_flash_info->available_space, bank->name);
		return ERROR_FAIL;
	}

	// poll target to update state
	retval = target_poll(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return ERROR_FAIL;
	}

	/* Fill the bank info based on the discovered device info */
	bank->num_sectors = (bank->size / adspsc59x_flash_info->sectorsize);

	/* Create and fill the sectors array */
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("Not enough memory available for sectors array.");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * adspsc59x_flash_info->sectorsize;
		sectors[sector].size = adspsc59x_flash_info->sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	adspsc59x_flash_info->probed = true;

	return retval;
}

/**
 * Probe the memory region to set up the target side algorithm and update the bank
 * appropriately.
 *
 * @param	bank	Pointer to the flash bank to use and write to
 *
 * @returns	ERROR_OK if successful otherwise the relevant code.
*/
static int adspsc59x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	struct adspsc59x_algo_params algo_params;
	int retval;
	uint32_t jedec_id = 0u;

	if (strcmp(bank->name, SPI_NAME_594))
		return adspsc59x_auto_probe(bank);

	LOG_INFO("Setting up flash area for %s...", bank->name);

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	target_free_all_working_areas(target);

	/* Output available working memory on target */
	uint32_t available_space = target_get_working_area_avail(target);
	uint32_t target_start_address = (uint32_t)target->working_area_phys;
	LOG_INFO("Target has %uB of available space.", available_space);
	adspsc59x_flash_info->available_space = available_space;

	// Check algorithm size vs allocated flash bank space
	if (adspsc59x_flash_info->adspsc59x_algorithm.size > adspsc59x_flash_info->available_space) {
		LOG_ERROR("Not enough available space in flash bank %s for corresponding algorithm of size %lu", bank->name,
			adspsc59x_flash_info->adspsc59x_algorithm.size);
		return ERROR_FAIL;
	}

	// Check start address of algorithm with address provided in cfg
	if (adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address < target_start_address
		|| adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address > (target_start_address + adspsc59x_flash_info->available_space)) {
		LOG_ERROR("Start address for corresponding algorithm of %lu is not within the allocated range of %u for flash bank %s",
			adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address, target_start_address + adspsc59x_flash_info->available_space, bank->name);
		return ERROR_FAIL;
	}

	// poll target to update state
	retval = target_poll(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return ERROR_FAIL;
	}

	// Check if algorithm is running, if not run it
	if (target->state != TARGET_DEBUG_RUNNING) {
		retval = adspsc59x_init(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	// Need to halt before reads/writes
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Target is not halted!");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return ERROR_FAIL;
	}

	// poll target to update state
	retval = target_poll(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return ERROR_FAIL;
	}

	// hardcode to issue device id read command to algorithm
	buf_set_u32(algo_params.command, 0, 32, READ_ID_CODE_COMMAND);

	// write algo parameters
	buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

	retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address,
				sizeof(algo_params), (uint8_t *)&algo_params);

	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to write algorithm parameters");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return retval;
	}

	// Resume running algorithm with parameters
	target_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

	retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

	if (retval != ERROR_OK) {
		LOG_ERROR_ALGO_PARAMS(algo_params);
		return retval;
	}

	// Need to halt before reads/writes
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Target is not halted!");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return retval;
	}

	// poll target to update state
	retval = target_poll(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return retval;
	}

	retval = target_read_u32(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address + ADSPSC59X_READID_OFFSET, &jedec_id);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adspsc59x_flash_info->working_area);
		adspsc59x_flash_info->working_area = NULL;
		return retval;
	}

	LOG_DEBUG("Got SPI Flash device ID: 0x%08X", jedec_id);

	bool found_device = false;
	for (const struct flash_device *pFlashDevice = flash_devices; pFlashDevice->name; pFlashDevice++) {
		if (pFlashDevice->device_id == jedec_id) {
			adspsc59x_flash_info->dev = *pFlashDevice;
			found_device = true;
			break;
		}
	}

	if (!found_device) {
		LOG_ERROR("No matching SPI Flash definition found for read Device ID: 0x%08X.", jedec_id);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	LOG_INFO("Discovered SPI Flash: %s", adspsc59x_flash_info->dev.name);

	/* Fill the bank info based on the discovered device info */
	bank->size = adspsc59x_flash_info->dev.size_in_bytes;
	bank->num_sectors = (adspsc59x_flash_info->dev.size_in_bytes / adspsc59x_flash_info->dev.sectorsize);
	/* Create and fill the sectors array */
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("Not enough memory available for sectors array.");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * adspsc59x_flash_info->dev.sectorsize;
		sectors[sector].size = adspsc59x_flash_info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	adspsc59x_flash_info->size_in_bytes = bank->size;
	adspsc59x_flash_info->sectorsize = adspsc59x_flash_info->dev.sectorsize;
	bank->sectors = sectors;
	adspsc59x_flash_info->probed = true;

	return ERROR_OK;
}

/**
* Not yet supported for spi flash.
*/
static int adspsc59x_protect_check(struct flash_bank *bank)
{
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/**
* Not yet supported for spi flash.
*/
static int adspsc59x_protect(struct flash_bank *bank, int set,
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
static int adspsc59x_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	int retval = adspsc59x_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	command_print(cmd, "ADSPSC-59x %s\n"
			"Size: 0x%X\n", bank->name,
			bank->size);

	return ERROR_OK;
}

/**
 * Usage:
 * flash bank <name> adspsc59x <base_addr> 0 0 0 <target> sector_size algorithm_file param_file
*/
FLASH_BANK_COMMAND_HANDLER(adspsc59x_flash_bank_command)
{
	struct adspsc59x_flash_bank *adspsc59x_flash_info;
	int byteCount = 0;
	int count = 0;
	uint32_t tempParse;
	char tempStr[3];
	uint8_t convertedHex;
	FILE *algo_file;
	FILE *parameter_file;
	bool insideComment = true;
	char line[256];
	char parameter_file_data[PARAMETER_FILE_COUNT][9];  // Assuming each hex value is of length 8

	/* Check the correct number of arguments have been provided */
	if (CMD_ARGC != 9) {
		LOG_ERROR("Invalid number of flash bank arguments. Usage:\n"
			"flash bank <name> adspsc59x <base_addr> 0 0 0 <target> "
			"sector_size algorithm_file parameter_file");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	adspsc59x_flash_info = malloc(sizeof(struct adspsc59x_flash_bank));
	if (!adspsc59x_flash_info) {
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
	adspsc59x_flash_info->adspsc59x_algorithm.size = ftell(algo_file);
	fseek(algo_file, 0, SEEK_SET);
	adspsc59x_flash_info->adspsc59x_algorithm.adspsc59x_algo = malloc(sizeof(uint8_t) * adspsc59x_flash_info->adspsc59x_algorithm.size);
	// Get 2 characters at a time to form byte. Convert byte and
	// store into spi algorithm buffer
	do {
		tempStr[0] = fgetc(algo_file);
		// read/skip over LF (UNIX) and CR (Windows)
		if (tempStr[0] != '\n' && tempStr[0] != '\r') {
			tempStr[1] = fgetc(algo_file);
			convertedHex = strtoul((const char *)tempStr, NULL, 16);
			adspsc59x_flash_info->adspsc59x_algorithm.adspsc59x_algo[byteCount] = convertedHex;
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

	adspsc59x_flash_info->adspsc59x_algorithm.parameter_address = strtoul(parameter_file_data[0], NULL, 16);
	adspsc59x_flash_info->adspsc59x_algorithm.buffer_address = strtoul(parameter_file_data[1], NULL, 16);
	adspsc59x_flash_info->adspsc59x_algorithm.reset_handler_addr = strtoul(parameter_file_data[2], NULL, 16);

	adspsc59x_flash_info->adspsc59x_algorithm.algo_start_address = strtoul(parameter_file_data[3], NULL, 16);
	adspsc59x_flash_info->adspsc59x_algorithm.version = strtoul(parameter_file_data[4], NULL, 10);

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], tempParse);
	adspsc59x_flash_info->sectorsize = tempParse;

	adspsc59x_flash_info->probed = false;
	bank->driver_priv = adspsc59x_flash_info;

	return ERROR_OK;
}

/**
 * Erase whole memory on SPI flash device.
 * Usage:
 * adspsc59x mase_erase bank_id
*/
COMMAND_HANDLER(adspsc59x_mass_erase_handler)
{
	struct flash_bank *bank;
	struct target *target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info;
	struct adspsc59x_algo_params algo_params;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	adspsc59x_flash_info = bank->driver_priv;

	if (!adspsc59x_flash_info->probed) {
		LOG_ERROR("Cannot erase flash as target has not been probed. Please probe target first.");
		retval = ERROR_FLASH_BANK_NOT_PROBED;
	} else {
		// poll target to update state
		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return ERROR_FAIL;
		}

		// Check if algorithm is running, if not run it
		if (target->state != TARGET_DEBUG_RUNNING) {
			retval = adspsc59x_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return retval;
		}

		// poll target to update state
		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return retval;
		}

		/* Check device is halted and has been probed first */
		if (TARGET_HALTED != target->state) {
			LOG_ERROR("Cannot read from flash. Target is not halted!");
			return ERROR_TARGET_NOT_HALTED;
		}

		// hardcode to issue mass erase command to algorithm
		buf_set_u32(algo_params.command, 0, 32, MASS_ERASE_COMMAND);

		// write algo parameters
		buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

		retval = target_write_buffer(target, adspsc59x_flash_info->adspsc59x_algorithm.parameter_address,
					sizeof(algo_params), (uint8_t *)&algo_params);

		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write algorithm parameters");
			target_free_working_area(target, adspsc59x_flash_info->working_area);
			adspsc59x_flash_info->working_area = NULL;
			return retval;
		}

		// Resume running algorithm with parameters
		target_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

		retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX_MASS_ERASE);

		if (retval != ERROR_OK) {
			LOG_ERROR_ALGO_PARAMS(algo_params);
			return retval;
		}
	}

	return retval;
}

/**
 * Get algorithm version number
 * Usage:
 * adspsc59x get_algorithm_version bank_id
*/
COMMAND_HANDLER(adspsc59x_get_algorithm_version_handler)
{
	struct adspsc59x_flash_bank *adspsc59x_flash_info;
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	adspsc59x_flash_info = bank->driver_priv;

	command_print(CMD, "%lu", adspsc59x_flash_info->adspsc59x_algorithm.version);

	return retval;
}

static const struct command_registration adspsc59x_exec_command_handlers[] = {
	{
		.name		= "mass_erase",
		.handler	= adspsc59x_mass_erase_handler,
		.mode		= COMMAND_EXEC,
		.usage		= "bank_id",
		.help		= "Mass erase entire flash device.",
	},
	{
		.name		= "get_algorithm_version",
		.handler	= adspsc59x_get_algorithm_version_handler,
		.mode		= COMMAND_EXEC,
		.usage		= "bank_id",
		.help		= "Get algorithm version.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration adspsc59x_command_handlers[] = {
	{
		.name	= "adspsc59x",
		.mode	= COMMAND_ANY,
		.help	= "adspsc59x flash command group",
		.usage	= "",
		.chain	= adspsc59x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver adspsc59x_flash = {
	.name				= "adspsc59x",
	.commands			= adspsc59x_command_handlers,
	.flash_bank_command	= adspsc59x_flash_bank_command,
	.erase				= adspsc59x_erase,
	.protect			= adspsc59x_protect,
	.write				= adspsc59x_write,
	.read				= adspsc59x_read,
	.probe				= adspsc59x_probe,
	.auto_probe			= adspsc59x_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= adspsc59x_protect_check,
	.info				= adspsc59x_get_info,
	.free_driver_priv	= default_flash_free_driver_priv,
};

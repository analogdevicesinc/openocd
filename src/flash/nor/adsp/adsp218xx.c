// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
 *	Copyright (C) 2022-2026 Analog Devices, Inc.							*
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
#include "adsp218xx.h"
#include "adsp_helper.h"

/**
 * Erase the specified sectors within the flash
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	first	The number of the first sector to erase
 * @param	last	The number of the last sector to erase
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
 */
static int adsp218xx_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp218xx_flash_info = bank->driver_priv;

	int retval;

	if (BANK_NAME_IS(bank, OTP_NAME) || BANK_NAME_IS(bank, EMMC_NAME)) {
		LOG_ERROR("Erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (!adsp218xx_flash_info->probed) {
		LOG_ERROR("Cannot erase flash as target has not been probed. Please probe target first.");
		retval = ERROR_FLASH_BANK_NOT_PROBED;
	} else {
		/* All good to proceed */
		LOG_INFO("Erasing sectors %u to %u (inclusive) in flash", first, last);
		uint32_t address;

		// Check if algorithm is running, if not run it
		if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
			retval = adsp_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		for (unsigned int counter = first; counter <= last; counter++) {
			/* Calculate the address based on the counter and configured sector size */
			address = counter * adsp218xx_flash_info->sectorsize;

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adsp218xx_flash_info->working_area);
				adsp218xx_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}

			// hardcode to issue sector erase command to algorithm
			adsp218xx_flash_info->algo_params.command = SECTOR_ERASE_COMMAND;

			// write algo parameters
			adsp218xx_flash_info->algo_params.address = address;
			adsp218xx_flash_info->algo_params.ready = ALGO_READY;

			if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX_MASS_ERASE))
				return ERROR_FAIL;
		}
	}

	return retval;
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
static int adsp218xx_write_otp(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp218xx_flash_info = bank->driver_priv;

	int retval;
	uint32_t byte_count_rounded;

	if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
	} else {
		// Check if algorithm is running, if not run it
		if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
			retval = adsp_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		/* The OTP buffer is programmed with raw binary data, rounded up to a
		 * 32-bit word boundary.
		 *
		 * Any padding needed to reach the word boundary is filled with 0x00, the
		 * erased OTP state. Writing 0x00 leaves those OTP bits unprogrammed so the
		 * padded locations can still be written later. */
		byte_count_rounded = ROUNDUP(count, 4);

		uint8_t send_buf[byte_count_rounded];
		memset(send_buf, 0x00, byte_count_rounded);
		memcpy(send_buf, buffer, count);

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adsp218xx_flash_info->working_area);
			adsp218xx_flash_info->working_area = NULL;
			return retval;
		}

		/* Check device is halted and has been probed first */
		if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
			LOG_ERROR("Cannot read from flash. Target is not halted!");
			return ERROR_TARGET_NOT_HALTED;
		}

		/* Put next block of data to flash into buffer */
		retval = target_write_buffer(target, adsp218xx_flash_info->adsp_algorithm.buffer_address,
			byte_count_rounded, (void *)send_buf);

		if (byte_count_rounded < 4)
			byte_count_rounded = 4;

		// Issue program command to algorithm
		adsp218xx_flash_info->algo_params.command = PROGRAM_COMMAND;

		// write algo parameters
		adsp218xx_flash_info->algo_params.address = offset;
		adsp218xx_flash_info->algo_params.length = byte_count_rounded;
		adsp218xx_flash_info->algo_params.ready = ALGO_READY;

		if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX))
			return ERROR_FAIL;
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
static int adsp218xx_read_otp(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp218xx_flash_info = bank->driver_priv;

	int retval;
	uint32_t byte_count_rounded;

	if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
	} else {
		// Check if algorithm is running, if not run it
		if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
			retval = adsp_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		if (count % 4 != 0)
			byte_count_rounded = ROUNDUP(count, 4);
		else
			byte_count_rounded = count;

		uint8_t rec_buf[byte_count_rounded];

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adsp218xx_flash_info->working_area);
			adsp218xx_flash_info->working_area = NULL;
			return retval;
		}

		/* Check device is halted and has been probed first */
		if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
			LOG_ERROR("Cannot read from flash. Target is not halted!");
			return ERROR_TARGET_NOT_HALTED;
		}

		// hardcode to issue read command to algorithm
		adsp218xx_flash_info->algo_params.command = READ_COMMAND;

		// write algo parameters
		adsp218xx_flash_info->algo_params.address = offset;
		adsp218xx_flash_info->algo_params.length = byte_count_rounded;
		adsp218xx_flash_info->algo_params.ready = ALGO_READY;

		if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX))
			return ERROR_FAIL;

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adsp218xx_flash_info->working_area);
			adsp218xx_flash_info->working_area = NULL;
			return retval;
		}

		/* Check device is halted and has been probed first */
		if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
			LOG_ERROR("Cannot read from flash. Target is not halted!");
			return ERROR_TARGET_NOT_HALTED;
		}

		/* Put next block of data from flash into buffer */
		retval = target_read_buffer(target, adsp218xx_flash_info->adsp_algorithm.buffer_address,
		byte_count_rounded, rec_buf);

		memcpy(buffer, rec_buf, count);

		if (retval != ERROR_OK) {
			LOG_ERROR_ALGO_PARAMS(adsp218xx_flash_info->algo_params);
			/* Close down algo */
			(void)adsp_quit(bank);
			return retval;
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
static int adsp218xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp218xx_flash_info = bank->driver_priv;

	int retval;
	unsigned int write_size = 0;
	unsigned int buffer_index = 0;
	uint32_t current_address = offset;
	uint32_t buffer_size = adsp218xx_flash_info->adsp_algorithm.buffer_size;

	if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
	} else {
		// Check if algorithm is running, if not run it
		if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
			retval = adsp_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		/* First write any bytes if the specified offset if not on the buffer size boundary */
		if ((current_address % buffer_size) != 0) {
			/* Calculate the write size to use, the modulo remainder of the buffer size  (unless the specified count is
			 * smaller) */
			write_size = buffer_size - (current_address % buffer_size);
			if (write_size > count)
				write_size = count;

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adsp218xx_flash_info->working_area);
				adsp218xx_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}

			// Issue program command to algorithm
			adsp218xx_flash_info->algo_params.command = PROGRAM_COMMAND;

			/* Put next block of data to flash into buffer */
			retval = target_write_buffer(target, adsp218xx_flash_info->adsp_algorithm.buffer_address, write_size,
										 &buffer[buffer_index]);

			// write algo parameters
			adsp218xx_flash_info->algo_params.address = current_address;
			adsp218xx_flash_info->algo_params.length = write_size;
			adsp218xx_flash_info->algo_params.ready = ALGO_READY;

			if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX))
				return ERROR_FAIL;

			/* Increment the index and address */
			current_address += write_size;
			buffer_index += write_size;

			LOG_INFO("Written %u/%u bytes. Current address is 0x%08X", buffer_index, count, current_address);
		}

		/* Write remaining data */
		while (count - buffer_index) {
			/* If the remaining bytes is less than the buffer size,
			 *  size is just the remaining bytes...
			 */
			if ((count - buffer_index) < buffer_size)
				write_size = count - buffer_index;
			/* Otherwise size is the buffer size (max size that can be written in one command) */
			else
				write_size = buffer_size;

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adsp218xx_flash_info->working_area);
				adsp218xx_flash_info->working_area = NULL;
				return retval;
			}

			// poll target to update state
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adsp218xx_flash_info->working_area);
				adsp218xx_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}

			// Issue program command to algorithm
			adsp218xx_flash_info->algo_params.command = PROGRAM_COMMAND;

			/* Put next block of data to flash into buffer */
			retval = target_write_buffer(target, adsp218xx_flash_info->adsp_algorithm.buffer_address, write_size,
										 &buffer[buffer_index]);

			// write algo parameters

			adsp218xx_flash_info->algo_params.address = current_address;
			adsp218xx_flash_info->algo_params.length = write_size;
			adsp218xx_flash_info->algo_params.ready = ALGO_READY;

			if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX))
				return ERROR_FAIL;

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
static int adsp218xx_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp218xx_flash_info = bank->driver_priv;

	uint32_t buffer_size = adsp218xx_flash_info->adsp_algorithm.buffer_size;
	int retval;

	if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
	} else {
		// Check if algorithm is running, if not run it
		if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
			retval = adsp_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		/* Make sure read is not larger than buffer size to be passed */
		uint32_t read_bytes = 0;
		uint32_t read_size = 0;
		while (count) {
			// read size threshold is only applicable to SPI protocol. For xSPI the current
			// implementation requires a valid read size divisible by 4
			if (!BANK_NAME_IS(bank, SPI_NAME)) {
				read_size = count;
			} else {
				/* Maximum read size*/
				read_size = buffer_size - 1;

				/* Then if the actual count is smaller than the buffer size, use the count */
				if (count < read_size)
					read_size = count;
			}

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adsp218xx_flash_info->working_area);
				adsp218xx_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}

			/* Check we're not going reading more than we should */
			assert(count >= read_size);

			uint32_t address = offset + read_bytes;

			// hardcode to issue read command to algorithm
			adsp218xx_flash_info->algo_params.command = READ_COMMAND;

			// write algo parameters
			adsp218xx_flash_info->algo_params.address = address;
			adsp218xx_flash_info->algo_params.length = read_size;
			adsp218xx_flash_info->algo_params.ready = ALGO_READY;

			if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX))
				return ERROR_FAIL;

			// Need to halt before reads/writes
			retval = target_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Target is not halted!");
				target_free_working_area(target, adsp218xx_flash_info->working_area);
				adsp218xx_flash_info->working_area = NULL;
				return retval;
			}

			/* Check device is halted and has been probed first */
			if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
				LOG_ERROR("Cannot read from flash. Target is not halted!");
				return ERROR_TARGET_NOT_HALTED;
			}

			/* Put next block of data from flash into buffer */
			retval = target_read_buffer(target, adsp218xx_flash_info->adsp_algorithm.buffer_address, read_size,
										&buffer[read_bytes]);

			if (retval != ERROR_OK) {
				/* Close down algo */
				(void)adsp_quit(bank);
				LOG_ERROR_ALGO_PARAMS(adsp218xx_flash_info->algo_params);
				return retval;
			}

			/* Increment the index and address */
			read_bytes += read_size;
			count -= read_size;
		}
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
static int adsp218xx_auto_probe(struct flash_bank *bank)
{
	struct adsp_flash_bank *adsp218xx_flash_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	struct target *target = bank->target;

	if (!adsp218xx_flash_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	if (adsp218xx_flash_info->probed)
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
	adsp218xx_flash_info->available_space = available_space;

	// Check algorithm size vs allocated flash bank space
	if (adsp218xx_flash_info->adsp_algorithm.size > adsp218xx_flash_info->available_space) {
		LOG_ERROR("Not enough available space in flash bank %s for corresponding algorithm of size %lu", bank->name,
				  adsp218xx_flash_info->adsp_algorithm.size);
		return ERROR_FAIL;
	}

	// Check start address of algorithm with address provided in cfg
	if (adsp218xx_flash_info->adsp_algorithm.algo_start_address < target_start_address ||
		adsp218xx_flash_info->adsp_algorithm.algo_start_address >
			(target_start_address + adsp218xx_flash_info->available_space)) {
		LOG_ERROR("Start address for corresponding algorithm of %lu is not within the allocated range of %u for flash "
				  "bank %s",
				  adsp218xx_flash_info->adsp_algorithm.algo_start_address,
				  target_start_address + adsp218xx_flash_info->available_space, bank->name);
		return ERROR_FAIL;
	}

	/* Check device is halted and has been probed first */
	if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Fill the bank info based on the discovered device info */
	bank->num_sectors = (bank->size / adsp218xx_flash_info->sectorsize);

	/* Create and fill the sectors array */
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("Not enough memory available for sectors array.");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * adsp218xx_flash_info->sectorsize;
		sectors[sector].size = adsp218xx_flash_info->sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	adsp218xx_flash_info->probed = true;

	return ERROR_OK;
}

/**
 * Probe the memory region to set up the target side algorithm and update the bank
 * appropriately.
 *
 * @param	bank	Pointer to the flash bank to use and write to
 *
 * @returns	ERROR_OK if successful otherwise the relevant code.
 */
static int adsp218xx_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp218xx_flash_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;

	int retval;
	uint32_t jedec_id = 0u;

	if (!BANK_NAME_IS(bank, SPI_NAME) && !BANK_NAME_IS(bank, EMMC_NAME))
		return adsp218xx_auto_probe(bank);

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
	adsp218xx_flash_info->available_space = available_space;

	// Check algorithm size vs allocated flash bank space
	if (adsp218xx_flash_info->adsp_algorithm.size > adsp218xx_flash_info->available_space) {
		LOG_ERROR("Not enough available space in flash bank %s for corresponding algorithm", bank->name);
		return ERROR_FAIL;
	}

	// Check start address of algorithm with address provided in cfg
	if (adsp218xx_flash_info->adsp_algorithm.algo_start_address < target_start_address ||
		adsp218xx_flash_info->adsp_algorithm.algo_start_address >
			(target_start_address + adsp218xx_flash_info->available_space)) {
		LOG_ERROR("Start address for corresponding algorithm of %lu is not within the allocated range of %u for flash "
				  "bank %s",
				  adsp218xx_flash_info->adsp_algorithm.algo_start_address,
				  target_start_address + adsp218xx_flash_info->available_space, bank->name);
		return ERROR_FAIL;
	}

	// Check if algorithm is running, if not run it
	if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
		retval = adsp_init(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	// Need to halt before reads/writes
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Target is not halted!");
		target_free_working_area(target, adsp218xx_flash_info->working_area);
		adsp218xx_flash_info->working_area = NULL;
		return ERROR_FAIL;
	}

	/* Check device is halted and has been probed first */
	if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	// hardcode to issue device id read command to algorithm
	adsp218xx_flash_info->algo_params.command = READ_ID_CODE_COMMAND;

	// write algo parameters
	adsp218xx_flash_info->algo_params.ready = ALGO_READY;

	if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX))
		return ERROR_FAIL;

	// Need to halt before reads/writes
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Target is not halted!");
		target_free_working_area(target, adsp218xx_flash_info->working_area);
		adsp218xx_flash_info->working_area = NULL;
		return retval;
	}

	/* Check device is halted and has been probed first */
	if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = target_read_u32(target, adsp218xx_flash_info->adsp_algorithm.parameter_address + ADSP_READID_OFFSET, &jedec_id);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to poll target");
		target_free_working_area(target, adsp218xx_flash_info->working_area);
		adsp218xx_flash_info->working_area = NULL;
		return retval;
	}

	LOG_DEBUG("Got SPI Flash device ID: 0x%08X", jedec_id);
	adsp218xx_flash_info->algo_params.device_id = jedec_id;

	// eMMC is part of NAND flash device which is not on the list of supported
	// spi devices in spi.h and spi.c since it does not behave like normal spi flash
	if (!BANK_NAME_IS(bank, EMMC_NAME)) {
		bool found_device = false;
		for (const struct flash_device *flash_device = flash_devices; flash_device->name; flash_device++) {
			if (flash_device->device_id == jedec_id) {
				adsp218xx_flash_info->dev = *flash_device;
				found_device = true;
				break;
			}
		}

		if (!found_device) {
			LOG_ERROR("No matching SPI Flash definition found for read Device ID: 0x%08X.", jedec_id);
			return ERROR_FLASH_OPER_UNSUPPORTED;
		}

		LOG_INFO("Discovered SPI Flash: %s", adsp218xx_flash_info->dev.name);

		/* Fill the bank info based on the discovered device info */
		bank->size = adsp218xx_flash_info->dev.size_in_bytes;
		bank->num_sectors = (adsp218xx_flash_info->dev.size_in_bytes / adsp218xx_flash_info->dev.sectorsize);
		/* Create and fill the sectors array */
		sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		if (!sectors) {
			LOG_ERROR("Not enough memory available for sectors array.");
			return ERROR_FAIL;
		}

		for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
			sectors[sector].offset = sector * adsp218xx_flash_info->dev.sectorsize;
			sectors[sector].size = adsp218xx_flash_info->dev.sectorsize;
			sectors[sector].is_erased = -1;
			sectors[sector].is_protected = 0;
		}

		adsp218xx_flash_info->size_in_bytes = bank->size;
		adsp218xx_flash_info->sectorsize = adsp218xx_flash_info->dev.sectorsize;
		bank->sectors = sectors;
	}

	adsp218xx_flash_info->probed = true;

	return ERROR_OK;
}

/**
 * Copy the flash device info into the provided buffer.
 *
 * @param	bank		Pointer to the flash bank to print get info about
 * @param   cmd         Pointer to the command invocation
 *
 * @returns	ERROR_OK if successful otherwise the relevant code.
 */
static int adsp218xx_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	int retval = adsp218xx_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	command_print(cmd,
				  "ADSP-218XX %s\n"
				  "Size: 0x%X\n",
				  bank->name, bank->size);

	return ERROR_OK;
}

/**
 * Usage:
 * flash bank <name> adsp218xx <base_addr> <size> 0 0 <target> sector_size algorithm_file param_file
 */
FLASH_BANK_COMMAND_HANDLER(adsp218xx_flash_bank_command)
{
	struct adsp_flash_bank *adsp218xx_flash_info;
	int byte_count = 0;
	int count = 0;
	uint32_t temp_parse;
	char temp_str[3];
	uint8_t converted_hex;
	FILE *algo_file;
	FILE *parameter_file;
	bool inside_comment = true;
	char line[256];
	char parameter_file_data[PARAMETER_FILE_COUNT][9]; // Assuming each hex value is of length 8

	/* Check the correct number of arguments have been provided */
	if (CMD_ARGC != 9) {
		LOG_ERROR("Invalid number of flash bank arguments. Usage:\n"
				  "flash bank <name> adsp218xx <base_addr> <size> 0 0 <target> "
				  "sector_size algorithm_file parameter_file");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* Check to see if openocd is being used for non-flashing */
	if (strlen(CMD_ARGV[7]) == 0 && strlen(CMD_ARGV[8]) == 0) {
		LOG_WARNING("Flashing will not work without corresponding .inc files");
		return ERROR_OK;
	}

	adsp218xx_flash_info = malloc(sizeof(struct adsp_flash_bank));
	if (!adsp218xx_flash_info) {
		LOG_ERROR("Not enough memory for local driver information.");
		return ERROR_FAIL;
	}

	// Opening file in reading mode
	algo_file = fopen(CMD_ARGV[7], "r");

	if (!algo_file) {
		LOG_ERROR("File %s can't be opened\n", CMD_ARGV[7]);
		free(adsp218xx_flash_info);
		return ERROR_FAIL;
	}

	/* Size of file */
	fseek(algo_file, 0, SEEK_END);
	adsp218xx_flash_info->adsp_algorithm.size = ftell(algo_file);
	fseek(algo_file, 0, SEEK_SET);
	adsp218xx_flash_info->adsp_algorithm.adsp_algo =
		malloc(sizeof(uint8_t) * adsp218xx_flash_info->adsp_algorithm.size);

	int current_char;
	int hex_digit_count = 0;
	// Parse two hex digits at a time from the full file stream.
	while ((current_char = fgetc(algo_file)) != EOF) {
		if (!isxdigit((unsigned char)current_char))
			continue;

		temp_str[hex_digit_count++] = (char)current_char;
		if (hex_digit_count == 2) {
			temp_str[2] = '\0';
			converted_hex = strtoul((const char *)temp_str, NULL, 16);
			adsp218xx_flash_info->adsp_algorithm.adsp_algo[byte_count] = converted_hex;
			byte_count++;
			hex_digit_count = 0;
		}
	}

	// Closing the file
	fclose(algo_file);

	// Opening file in reading mode
	parameter_file = fopen(CMD_ARGV[8], "r");

	if (!parameter_file) {
		LOG_ERROR("File %s can't be opened\n", CMD_ARGV[8]);
		free(adsp218xx_flash_info->adsp_algorithm.adsp_algo);
		free(adsp218xx_flash_info);
		return ERROR_FAIL;
	}

	// Loop through each line in the file
	while (fgets(line, sizeof(line), parameter_file)) {
		// Assuming the hex values are written one per line
		// You may need to adjust the logic based on the actual file structure
		if (strstr(line, "*/")) {
			inside_comment = false;
			continue;
		}

		if (!inside_comment) {
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

	adsp218xx_flash_info->adsp_algorithm.parameter_address = strtoul(parameter_file_data[0], NULL, 16);
	adsp218xx_flash_info->adsp_algorithm.buffer_address = strtoul(parameter_file_data[1], NULL, 16);
	adsp218xx_flash_info->adsp_algorithm.reset_handler_addr = strtoul(parameter_file_data[2], NULL, 16);
	adsp218xx_flash_info->adsp_algorithm.algo_start_address = strtoul(parameter_file_data[3], NULL, 16);
	adsp218xx_flash_info->adsp_algorithm.version = strtoul(parameter_file_data[4], NULL, 10);
	adsp218xx_flash_info->adsp_algorithm.buffer_size = strtoul(parameter_file_data[5], NULL, 16);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], temp_parse);
	adsp218xx_flash_info->sectorsize = temp_parse;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], temp_parse);
	adsp218xx_flash_info->size_in_bytes = temp_parse;

	// initialize algorithm parameters to zero upon startup
	adsp218xx_flash_info->algo_params = (struct adsp_algo_params){0};

	adsp218xx_flash_info->probed = false;
	bank->driver_priv = adsp218xx_flash_info;

	return ERROR_OK;
}

/**
 * Get algorithm version number
 * Usage:
 * adsp218xx get_algorithm_version bank_id
 */
COMMAND_HANDLER(adsp218xx_get_algorithm_version_handler)
{
	struct adsp_flash_bank *adsp218xx_flash_info;
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	adsp218xx_flash_info = bank->driver_priv;

	if (!adsp218xx_flash_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	command_print(CMD, "%lu", adsp218xx_flash_info->adsp_algorithm.version);

	return retval;
}

/**
 * Erase whole memory on SPI flash device.
 * Usage:
 * adsp218xx mase_erase bank_id
 */
COMMAND_HANDLER(adsp218xx_mass_erase_handler)
{
	struct flash_bank *bank;
	struct target *target;
	struct adsp_flash_bank *adsp218xx_flash_info;

	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;
	adsp218xx_flash_info = bank->driver_priv;

	if (BANK_NAME_IS(bank, OTP_NAME) || BANK_NAME_IS(bank, EMMC_NAME)) {
		LOG_ERROR("Mass erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (!adsp218xx_flash_info->probed) {
		LOG_ERROR("Cannot erase flash as target has not been probed. Please probe target first.");
		retval = ERROR_FLASH_BANK_NOT_PROBED;
	} else {
		// Check if algorithm is running, if not run it
		if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
			retval = adsp_init(bank);
			if (retval != ERROR_OK)
				return retval;
		}

		// Need to halt before reads/writes
		retval = target_halt(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Target is not halted!");
			target_free_working_area(target, adsp218xx_flash_info->working_area);
			adsp218xx_flash_info->working_area = NULL;
			return retval;
		}

		/* Check device is halted and has been probed first */
		if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
			LOG_ERROR("Cannot read from flash. Target is not halted!");
			return ERROR_TARGET_NOT_HALTED;
		}

		// hardcode to issue mass erase command to algorithm
		adsp218xx_flash_info->algo_params.command = MASS_ERASE_COMMAND;

		adsp218xx_flash_info->algo_params.ready = ALGO_READY;

		if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX_MASS_ERASE))
			return ERROR_FAIL;
	}

	return retval;
}

/**
 * Handle adsp218xx emmc commands.
 * Usage:
 * adsp218xx emmc_command bank_id command_val
 */
COMMAND_HANDLER(adsp218xx_emmc_command_handler)
{
	struct flash_bank *bank;
	struct target *target;
	struct adsp_flash_bank *adsp218xx_flash_info;
	uint32_t command_val;

	int retval;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;
	adsp218xx_flash_info = bank->driver_priv;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], command_val);

	if (!BANK_NAME_IS(bank, EMMC_NAME)) {
		LOG_ERROR("Emmc commands are not available for this target");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (!adsp218xx_flash_info->probed) {
		LOG_ERROR("Cannot execute emmc command as target has not been probed. Please probe target first.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	// Check if algorithm is running, if not run it
	if (adsp_target_poll_check_state(bank, TARGET_DEBUG_RUNNING)) {
		retval = adsp_init(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	// Need to halt before reads/writes
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Target is not halted!");
		target_free_working_area(target, adsp218xx_flash_info->working_area);
		adsp218xx_flash_info->working_area = NULL;
		return retval;
	}

	/* Check device is halted and has been probed first */
	if (adsp_target_poll_check_state(bank, TARGET_HALTED)) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	// hardcode to issue emmc command with corresponding command value to algorithm
	adsp218xx_flash_info->algo_params.command = command_val;

	adsp218xx_flash_info->algo_params.ready = ALGO_READY;

	if (adsp_run_flash_command(bank, ALGO_TIMEOUT_MAX))
		return ERROR_FAIL;

	command_print(CMD, "emmc command complete.");

	return retval;
}

/**
 * Handle adsp218xx read device id.
 * Usage:
 * adsp218xx read_device_id bank_id
 */
COMMAND_HANDLER(adsp218xx_read_device_id_handler)
{
	struct flash_bank *bank;
	struct adsp_flash_bank *adsp218xx_flash_info;

	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank_probe_optional, 0, &bank, false);
	if (retval != ERROR_OK)
		return retval;

	adsp218xx_flash_info = bank->driver_priv;

	if (BANK_NAME_IS(bank, SPI_NAME) || BANK_NAME_IS(bank, EMMC_NAME)) {
		retval = adsp218xx_probe(bank);
	} else {
		LOG_ERROR("Reading device id from %s is not supported", bank->name);
		return ERROR_FAIL;
	}

	if (retval == ERROR_OK) {
		command_print(CMD, "Device id is 0x%08X", adsp218xx_flash_info->algo_params.device_id);
	} else {
		LOG_ERROR("Could not read device id from %s", bank->name);
		retval = ERROR_FAIL;
	}

	return retval;
}

static const struct command_registration adsp218xx_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = adsp218xx_mass_erase_handler,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	{
		.name = "get_algorithm_version",
		.handler = adsp218xx_get_algorithm_version_handler,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Get algorithm version.",
	},
	{
		.name = "emmc_command",
		.handler = adsp218xx_emmc_command_handler,
		.mode = COMMAND_EXEC,
		.usage = "bank_id command_val",
		.help = "Execute emmc command.",
	},
	{
		.name = "read_device_id",
		.handler = adsp218xx_read_device_id_handler,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read device id.",
	},
	COMMAND_REGISTRATION_DONE};

static const struct command_registration adsp218xx_command_handlers[] = {
	{
		.name	= "adsp2183x",
		.mode	= COMMAND_ANY,
		.help	= "adsp2183x flash command group",
		.usage	= "",
		.chain	= adsp218xx_exec_command_handlers,
	},
	{
		.name	= "adsp2184x",
		.mode	= COMMAND_ANY,
		.help	= "adsp2184x flash command group",
		.usage	= "",
		.chain	= adsp218xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver adsp2183x_flash = {
	.name = "adsp2183x",
	.commands = adsp218xx_command_handlers,
	.flash_bank_command = adsp218xx_flash_bank_command,
	.erase = adsp218xx_erase,
	.protect = NULL,
	.write = adsp218xx_write,
	.read = adsp218xx_read,
	.probe = adsp218xx_probe,
	.auto_probe = adsp218xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.info = adsp218xx_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver adsp2183x_otp = {
	.name				= "adsp2183x_otp",
	.commands			= adsp218xx_command_handlers,
	.flash_bank_command	= adsp218xx_flash_bank_command,
	.erase				= adsp218xx_erase,
	.protect			= NULL,
	.write				= adsp218xx_write_otp,
	.read				= adsp218xx_read_otp,
	.probe				= adsp218xx_probe,
	.auto_probe			= adsp218xx_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= NULL,
	.info				= adsp218xx_get_info,
	.free_driver_priv	= default_flash_free_driver_priv,
};

const struct flash_driver adsp2184x_flash = {
	.name = "adsp2184x",
	.commands = adsp218xx_command_handlers,
	.flash_bank_command = adsp218xx_flash_bank_command,
	.erase = adsp218xx_erase,
	.protect = NULL,
	.write = adsp218xx_write,
	.read = adsp218xx_read,
	.probe = adsp218xx_probe,
	.auto_probe = adsp218xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.info = adsp218xx_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver adsp2184x_otp = {
	.name				= "adsp2184x_otp",
	.commands			= adsp218xx_command_handlers,
	.flash_bank_command	= adsp218xx_flash_bank_command,
	.erase				= adsp218xx_erase,
	.protect			= NULL,
	.write				= adsp218xx_write_otp,
	.read				= adsp218xx_read_otp,
	.probe				= adsp218xx_probe,
	.auto_probe			= adsp218xx_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= NULL,
	.info				= adsp218xx_get_info,
	.free_driver_priv	= default_flash_free_driver_priv,
};

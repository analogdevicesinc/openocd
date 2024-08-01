// SPDX-License-Identifier: GPL-2.0+

/****************************************************************************
 *	Copyright (C) 2022-2024 Analog Devices, Inc.							*
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "../imp.h"
#include "spi/adsp_spi.h"
#include <target/algorithm.h>
#include <target/armv8.h>
#include <target/armv8_opcodes.h>
#include <target/aarch64.h>
#include <target/armv7a.h>
#include "../spi.h"

/* Configures the number of retries when polling the Write In Progress (WIP) bit */
#define WIP_MAX_RETRIES		(25u)

/* Quad Mode Enable/Disable Commands for IS25LP Flash */
#define IS25_QUAD_IO_ENABLE_CMD		(0x35u)
#define IS25_QUAD_IO_DISABLE_CMD	(0xF5u)

/* Cache enable bit in System Control Register (EL3) */
#define BITP_SCTLR_EL3_C	(2u)
#define BITM_SCTLR_EL3_C BIT(BITP_SCTLR_EL3_C)

/* Undefine this macro to disable use of Quad IO */
#define DO_QUAD_IO

/** Sets the threshold at which word-transfers should be used over bytes.
 *  Should strike the balance between extra calculation required vs quicker
 *  transfers.
*/
#define SPI_WORD_TRANSFER_THRESHOLD		(16u)

/* Set the chunk size with which we write flash, assuming there's enough target
 * memory space for it.
 */
#define WRITE_CHUNK_SIZE (32u * 1024u)

/* Internal data structure to allow additional options for flash device */
struct adspsc59x_flash_bank {
	bool				probed;				/*! Has the flash device been probed? */
	bool				quad_io;			/*! Is the SPI flash currently in Quad IO mode */
	uint32_t			available_space;	/*! Used for sanity checking against memory leaks */
	struct flash_device	dev;				/*! A specific predefined flash device with common cmds and info */
};

const struct flash_driver adspsc59x_a55_flash;

/**
 * Helper function to poll the Busy/Write In Progress (WIP)
 * until either the device is 'free' or the build-time timeout
 * is reached.
 *
 * @param	bank	Flash bank to poll
 *
 * @returns	ERROR_OK if successful, otherwise failure return code
*/
static int adspsc59x_poll(struct flash_bank *bank)
{
	uint8_t read_data;
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	unsigned int counter = 0;
	ADSP_SPI_RESULT result;
	int rc = ERROR_FAIL;

	/** Set the default RC to timeout error. If device is free within set time,
	 *  RC will be changed to success.
	*/
	result = ERROR_TIMEOUT_REACHED;

	while (0 < (WIP_MAX_RETRIES - counter)) {
		/* Read SPI Flash status register */
		struct adsp_spi_flash_cmd flash_cmd = {
			.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
			.instruction	= SPIFLASH_READ_STATUS,
			.address		= 0,
			.address_bytes	= 0,
			.dummy_bytes	= 0,
			.data_out_ptr	= NULL,
			.data_out_bytes	= 0,
			.data_in_ptr	= &read_data,
			.data_in_bytes	= 1,
			.dma_queue		= NULL,
			.quad_io		= adspsc59x_flash_info->quad_io
		};

		result = adsp_spi_command(target, &flash_cmd);

		if (ADSP_SPI_RESULT_SUCCESS == result) {
			/* Check if the busy/WIP bit is set */
			if (0 != (read_data & SPIFLASH_BSY_BIT)) {
				/* Keep looping */
				counter++;

				/* Prevent GDB warnings */
				keep_alive();
			} else {
				rc = ERROR_OK;
				break;
			}
		} else {
			rc = adsp_spi_decode_result(result, "Failed to send read SPI Flash status command.");
			break;
		}
	}

	return rc;
}

/**
 * Helper function to set or clear the 'Write Enable' bit allowing or
 * disallowing the SPI Flash to be written to.
 *
 * @param	bank	Flash bank to change write enable status for
 * @param	enable	If true, enables writes. If false, disables writes.
 *
 * @returns	ERROR_OK if successful, otherwise failure return code
*/
static int adspsc59x_write_enable(struct flash_bank *bank, const bool enable, struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result;
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	int rc;

	struct adsp_spi_flash_cmd flash_cmd = {
		.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
		.instruction	= enable ? SPIFLASH_WRITE_ENABLE : SPIFLASH_WRITE_DISABLE,
		.address		= 0,
		.address_bytes	= 0,
		.dummy_bytes	= 0,
		.data_out_ptr	= NULL,
		.data_out_bytes	= 0,
		.data_in_ptr	= NULL,
		.data_in_bytes	= 0,
		.dma_queue		= queue,
		.quad_io		= adspsc59x_flash_info->quad_io
	};

	result = adsp_spi_command(target, &flash_cmd);

	rc = adsp_spi_decode_result(result, "Failed to send SPI Flash write enable/disable command.");

	return rc;
}

/**
 * Helper function to set or clear the 'Quad IO Enable' bit enabling or
 * disabling Quad IO Mode
 *
 * @param	bank	Flash bank to change Quad IO mode for
 * @param	enable	If true, enables Quad IO. If false, disables Quad IO.
 *
 * @returns	ERROR_OK if successful, otherwise failure return code
*/
static int adspsc59x_quad_io_enable(struct flash_bank *bank, const bool enable)
{
	ADSP_SPI_RESULT result;
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	int rc;

	/* Only bother sending command if the specified option doesn't match current option */
	if (enable != adspsc59x_flash_info->quad_io) {
		/* Quad IO enable param should be inverse of what we're trying to set because that should be
			the current mode. */
		struct adsp_spi_flash_cmd flash_cmd = {
			.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
			.instruction	= enable ? IS25_QUAD_IO_ENABLE_CMD : IS25_QUAD_IO_DISABLE_CMD,
			.address		= 0,
			.address_bytes	= 0,
			.dummy_bytes	= 0,
			.data_out_ptr	= NULL,
			.data_out_bytes	= 0,
			.data_in_ptr	= NULL,
			.data_in_bytes	= 0,
			.dma_queue		= NULL,
			.quad_io		= !enable
		};

		result = adsp_spi_command(target, &flash_cmd);

		rc = adsp_spi_decode_result(result, "Failed to send SPI Flash Quad IO enable/disable command.");

		/* If the operation was successful... */
		if (ERROR_OK == rc) {
			/* ... updated the variable to keep track of whether it's enabled. */
			adspsc59x_flash_info->quad_io = enable;
		}
	} else {
		rc = ERROR_OK;
	}

	return rc;
}

/**
 * Helper function to disable cache on the target. Function is specifically designed
 * for ARMv8-A.
 *
 * @param	target	Target to disable cache on
 *
 * @returns	ERROR_OK if successful, otherwise failure return code
*/
static int adspsc59x_disable_cache(struct target *target)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	int rc = ERROR_OK;
	uint32_t write_instruction = 0;
	uint32_t read_instruction = 0;

	/* SC594 and SC592 use Cortex A Architecture, and cache is disabled
	 * before every memory write by default */
	if ((strcmp(target->cmd_name, "adspsc594.cpu") == 0) ||
		(strcmp(target->cmd_name, "adspsc592.cpu") == 0)) {
		/* If the user has been playing around with cache settings in the board/target
		 * config files, warn them flashing may not work */
		if (armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled) {
			LOG_WARNING("Data Cache enabled for %s. Flashing may be unsuccessful. Try disabling cache "
			 "within the board/target config files.", target->cmd_name);
		} else {
			LOG_INFO("Detected data cache disabled for %s.", target->cmd_name);
		}

		return ERROR_OK;
	}

	/* Clear the cache enable bit */
	aarch64->system_control_reg_curr &= ~BITM_SCTLR_EL3_C;

	enum arm_mode target_mode = ARM_MODE_ANY;
	switch (armv8->arm.core_mode) {
		case ARMV8_64_EL0T:
			target_mode = ARMV8_64_EL1H;
			/* fall through */
		case ARMV8_64_EL1T:
		case ARMV8_64_EL1H:
			write_instruction = ARMV8_MSR_GP(SYSTEM_SCTLR_EL1, 0);
			read_instruction = ARMV8_MRS(SYSTEM_SCTLR_EL1, 0);
			break;
		case ARMV8_64_EL2T:
		case ARMV8_64_EL2H:
			write_instruction = ARMV8_MSR_GP(SYSTEM_SCTLR_EL2, 0);
			read_instruction = ARMV8_MRS(SYSTEM_SCTLR_EL2, 0);
			break;
		case ARMV8_64_EL3H:
		case ARMV8_64_EL3T:
			write_instruction = ARMV8_MSR_GP(SYSTEM_SCTLR_EL3, 0);
			read_instruction = ARMV8_MRS(SYSTEM_SCTLR_EL3, 0);
			break;
		case ARM_MODE_SVC:
		case ARM_MODE_ABT:
		case ARM_MODE_FIQ:
		case ARM_MODE_IRQ:
		case ARM_MODE_HYP:
		case ARM_MODE_UND:
		case ARM_MODE_SYS:
			write_instruction = ARMV4_5_MCR(15, 0, 0, 1, 0, 0);
			read_instruction = ARMV4_5_MRC(15, 0, 0, 1, 0, 0);
			break;
		default:
			LOG_ERROR("Unknown CPU state 0x%x", armv8->arm.core_mode);
			return ERROR_FAIL;
	}

	if (target_mode != ARM_MODE_ANY)
		armv8_dpm_modeswitch(&armv8->dpm, target_mode);

	rc = armv8->dpm.instr_write_data_r0_64(&armv8->dpm, write_instruction, aarch64->system_control_reg_curr);
	if (ERROR_OK != rc) {
		LOG_ERROR("Failed to write instruction to disable data cache. Got rc %d", rc);
		return rc;
	}

	/* Read back system control register to check bit is clear */
	rc = armv8->dpm.instr_read_data_r0_64(&armv8->dpm, read_instruction, &aarch64->system_control_reg);
	if (ERROR_OK != rc) {
		LOG_ERROR("Failed to read back system control register. Got rc %d", rc);
		return rc;
	}

	aarch64->system_control_reg_curr = aarch64->system_control_reg;
	if (0 != (aarch64->system_control_reg & BITM_SCTLR_EL3_C)) {
		LOG_ERROR("Instruction to clear cache enable bit failed. Bit is still set!");
		return ERROR_FAIL;
	}

	if (target_mode != ARM_MODE_ANY)
		armv8_dpm_modeswitch(&armv8->dpm, ARM_MODE_ANY);

	return rc;
}

/**
 * Usage:
 * flash bank <name> adspsc59x <base_addr> 0 0 0 <target>
*/
FLASH_BANK_COMMAND_HANDLER(adspsc59x_flash_bank_command)
{
	struct adspsc59x_flash_bank *poInfo;

	LOG_DEBUG("%s", __func__);

	/* Check the correct number of arguments have been provided */
	if (CMD_ARGC != 6) {
		LOG_ERROR("Invalid number of flash bank arguments. Usage:\n"
			"flash bank <name> adspsc59x <base_addr> 0 0 0 <target>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	poInfo = malloc(sizeof(struct adspsc59x_flash_bank));
	if (!poInfo) {
		LOG_ERROR("Not enough memory for local driver information.");
		return ERROR_FAIL;
	}

	poInfo->probed = false;
	bank->driver_priv = poInfo;

	return ERROR_OK;
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
	ADSP_SPI_RESULT result;
	int rc;

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot erase flash. Target is not halted!");
		rc = ERROR_TARGET_NOT_HALTED;
	} else if (!adspsc59x_flash_info->probed) {
		LOG_ERROR("Cannot erase flash as target has not been probed. Please probe target first.");
		rc = ERROR_FLASH_BANK_NOT_PROBED;
	} else {
	// TODO: OPENOCD-41 - Check if sector is protected, see todo in protect function for more details
	// NOTE: This will only be changed once this file is switched over to target side algo support
	/* All good to proceed */
		LOG_INFO("Erasing sectors %u to %u (inclusive) in flash", first, last);
		uint32_t address;
		for (unsigned int counter = first; counter <= last; counter++) {
			/* Set the write enable. This should be done before each erase instruction as
				WEL (write enable latch) is reset after erase operation.
			*/
			rc = adspsc59x_write_enable(bank, true, NULL);
			if (rc != ERROR_OK)
				return rc;

			/* Calculate the address based on the counter and configured sector size */
			address = counter * adspsc59x_flash_info->dev.sectorsize;

			LOG_INFO("Erasing %u bytes at address: 0x%08X", adspsc59x_flash_info->dev.sectorsize, address);

			struct adsp_spi_flash_cmd flash_cmd = {
				.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
				.instruction	= adspsc59x_flash_info->dev.erase_cmd,
				.address		= address,
				.address_bytes	= 4,
				.dummy_bytes	= 0,
				.data_out_ptr	= NULL,
				.data_out_bytes	= 0,
				.data_in_ptr	= NULL,
				.data_in_bytes	= 0,
				.dma_queue		= NULL,
				.quad_io		= adspsc59x_flash_info->quad_io
			};

			result = adsp_spi_command(target, &flash_cmd);

			rc = adsp_spi_decode_result(result, "Failed to send the SPI flash erase sector command.");
			if (rc != ERROR_OK)
				return rc;

			/* Poll until erase is complete */
			rc = adspsc59x_poll(bank);
			if (rc != ERROR_OK)
				return rc;
		}
	}

	return rc;
}

/**
 * Write the 'count' number of bytes from the buffer at the offset specified
 * in the SPI flash using the flash bank provided.
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	buffer	Data to write to the SPI flash
 * @param	offset	Offset from base to write to
 * @param	count	Number of bytes to write to SPI flash
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/
static int adspsc59x_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	ADSP_SPI_RESULT result;
	int rc;
	unsigned int write_size = 0;
	unsigned int buffer_index = 0;
	uint32_t current_address = offset;

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		rc = ERROR_TARGET_NOT_HALTED;
	} else if (!adspsc59x_flash_info->probed) {
		LOG_ERROR("Cannot read from flash as target has not been probed. Please probe target first.");
		rc = ERROR_FLASH_BANK_NOT_PROBED;
	} else if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		rc = ERROR_FLASH_DST_OUT_OF_BANK;
	} else {
		/* All good to proceed */
		LOG_INFO("Writing %u bytes to address 0x%08X", count, offset);

		struct adsp_dma_queue local_queue;
		struct adsp_dma_queue *queue = NULL;
		uint32_t available_space = target_get_working_area_avail(target);
		uint32_t chunk = WRITE_CHUNK_SIZE;
		ADSP_SPI_DEVICE device = (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55;

		// Assume the descriptors won't take up more space than the data itself!
		while (chunk > (available_space / 2))
			chunk /= 2;

		if (adsp_spi_devices[device].use_tx_dma_chain) {
			queue = &local_queue;
			result = adsp_init_dma_queue(queue, chunk, adspsc59x_flash_info->dev.pagesize);
			if (result != ADSP_SPI_RESULT_SUCCESS)
				return ERROR_FLASH_OPERATION_FAILED;
			queue->keep_descriptors = true; /* until decided otherwise */
		}

		LOG_INFO("Writing %u bytes to address 0x%08X, in chunks of %d bytes", count, offset, chunk);

		adsp_init_tru(target);

		/* Wait till flash is free... */
		rc = adspsc59x_poll(bank);
		if (ERROR_OK != rc)
			return rc;

		/* First write any bytes if the specified offset if not on the page size boundary */
		if (0 != (current_address % adspsc59x_flash_info->dev.pagesize)) {
			if (queue)
				queue->keep_descriptors = false; /* The next transfer will be different */

			/* Set the write enable. This should be done before each program page instruction as
				WEL (write enable latch) is reset after each operation.
			*/
			rc = adspsc59x_write_enable(bank, true, queue);
			if (rc != ERROR_OK)
				return rc;
			/* Calculate the write size to use, the modulo remainder of the page size  (unless the specified count is smaller) */
			write_size = adspsc59x_flash_info->dev.pagesize - (current_address % adspsc59x_flash_info->dev.pagesize);
			if (write_size > count)
				write_size = count;

			struct adsp_spi_flash_cmd flash_cmd = {
				.device			= device,
				.instruction	= adspsc59x_flash_info->dev.pprog_cmd,
				.address		= current_address,
				.address_bytes	= 4,
				.dummy_bytes	= 0,
				.data_out_ptr	= &buffer[buffer_index],
				.data_out_bytes	= write_size,
				.data_in_ptr	= NULL,
				.data_in_bytes	= 0,
				.dma_queue		= queue,
				.quad_io		= adspsc59x_flash_info->quad_io
			};

			result = adsp_spi_command(target, &flash_cmd);

			rc = adsp_spi_decode_result(result, "Failed to send the SPI flash page program command.");
			if (rc != ERROR_OK)
				return rc;

			/* Increment the index and address */
			current_address += write_size;
			buffer_index += write_size;
			LOG_INFO("Written %u/%u bytes. Current address is 0x%08X", buffer_index, count, current_address);

			/* Kick off the transfer so that we can re-use the descriptors on the target next time */
			result = adsp_run_queue(target, queue);
			rc = adsp_spi_decode_result(result, "Failed to run the DMA queue.");
			if (rc != ERROR_OK)
				return rc;

			LOG_INFO("Written %u/%u bytes. Current address is 0x%08X", buffer_index, count, current_address);
			if (queue)
				queue->keep_descriptors = true; /* until decided otherwise */
		}

		/* Write remaining data */
		while (count - buffer_index) {
			uint32_t chunk_count = 0;

			/* If there's less than two whole chunks left, don't keep the descriptors because the next
			 * chunk won't be complete and so will be different to this one.
			 */
			if (queue && count - buffer_index < (2 * chunk))
				queue->keep_descriptors = false;

			while (count - buffer_index && chunk_count < chunk) {
				/* Set the write enable. This should be done before each program page instruction as
					WEL (write enable latch) is reset after each operation.
				*/
				rc = adspsc59x_write_enable(bank, true, queue);
				if (rc != ERROR_OK)
					return rc;

				/* If the remaining bytes is less than the flash page
				*  size is just the remaining bytes...
				*/
				if ((count - buffer_index) < adspsc59x_flash_info->dev.pagesize) {
					write_size = count - buffer_index;
				} else {
					/* Otherwise size is the page size (max size that can be written in one command) */
					write_size = adspsc59x_flash_info->dev.pagesize;
				}

				struct adsp_spi_flash_cmd flash_cmd = {
					.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
					.instruction	= adspsc59x_flash_info->dev.pprog_cmd,
					.address		= current_address,
					.address_bytes	= 4,
					.dummy_bytes	= 0,
					.data_out_ptr	= &buffer[buffer_index],
					.data_out_bytes	= write_size,
					.data_in_ptr	= NULL,
					.data_in_bytes	= 0,
					.dma_queue		= queue,
					.quad_io		= adspsc59x_flash_info->quad_io
				};

				result = adsp_spi_command(target, &flash_cmd);

				rc = adsp_spi_decode_result(result, "Failed to send the SPI flash page program command.");
				if (rc != ERROR_OK)
					return rc;

				/* Increment the index and address */
				current_address += write_size;
				buffer_index += write_size;
				chunk_count += write_size;
			}

			/* Kick off the transfer */
			if (queue) {
				result = adsp_run_queue(target, queue);
				rc = adsp_spi_decode_result(result, "Failed to run the DMA queue.");
				if (rc != ERROR_OK)
					return rc;
			}
			LOG_INFO("Written %u/%u bytes. Current address is 0x%08X", buffer_index, count, current_address);
		}

		/* Done - free the DMA queue */
		if (queue)
			adsp_destroy_dma_queue(queue);
	}

	return rc;
}

/**
 * Read the 'count' number of bytes from the buffer at the offset specified
 * in the SPI flash using the flash bank provided. If called when device has
 * already been probed, previously filled fields will be discard and probe
 * procedure will be done again.
 *
 * @param	bank	Pointer to the flash bank to use
 * @param	buffer	Buffer to read data from the SPI flash into
 * @param	offset	Offset from base to read from
 * @param	count	Number of bytes to read from SPI flash
 *
 * @returns	Return code, ERROR_OK if successful otherwise the relevant code.
*/
static int adspsc59x_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;
	ADSP_SPI_RESULT result;
	int rc;
	spi_instruction_t instruction;
	uint8_t dummy_bytes;

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		rc = ERROR_TARGET_NOT_HALTED;
	} else if (!adspsc59x_flash_info->probed) {
		LOG_ERROR("Cannot read from flash as target has not been probed. Please probe target first.");
		rc = ERROR_FLASH_BANK_NOT_PROBED;
	} else {
		/* All good to proceed */
		/* Output available working memory on target */
		uint32_t available_space = target_get_working_area_avail(target);
		LOG_DEBUG("Target has %uB of available space for read.", available_space);

		/* Check there isn't a memory leak. On first usage param will be zero so ignore in that case */
		if (adspsc59x_flash_info->available_space != 0) {
			if (adspsc59x_flash_info->available_space > available_space) {
				LOG_WARNING("Available space on target seems to be decreasing. This may be a memory leak."
					"Space has decreased by %uB", (adspsc59x_flash_info->available_space - available_space));
			}
		}
		adspsc59x_flash_info->available_space = available_space;

		/* Wait till flash is free... */
		rc = adspsc59x_poll(bank);
		if (ERROR_OK != rc)
			return rc;

		/* Instruction and dummy bytes vary depending if we're in Quad IO mode */
		if (adspsc59x_flash_info->quad_io) {
			instruction = adspsc59x_flash_info->dev.qread_cmd;
			/* Quad Mode needs 6 dummy cycles which would ordinarily mean 6 bits of data,
			 * however as we're in Quad IO mode we transmit 4 bits of data very every cycle,
			 * i.e. 1 byte = 2 cycles
			*/
			dummy_bytes = 3;
		} else {
			instruction = adspsc59x_flash_info->dev.read_cmd;
			dummy_bytes = 0;
		}

		/* Max read has to be smaller than both max SPI transfer size and available space on target */
		uint32_t read_bytes = 0;
		while (count) {
			/* First calculate theoretical maximum */
			uint32_t read_size = MIN((spi_max_word_count * sizeof(uint32_t)),
				(available_space - spi_dma_buffer_minimum_size));

			/* Then if the actual count is smaller than the theoretical max, use the count */
			if (count < read_size)
				read_size = count;

			/** Now adjust read size so we can get as many word transfers as possible.
			 *  Only worth trying to swap to word transfers if it will actually make
			 *  things quicker. Check if we're above a threshold to make it worthwhile.
			*/
			if (read_size > SPI_WORD_TRANSFER_THRESHOLD)
				/* Trim read_size to make it divisible by 4 for word transfers. */
				read_size = read_size - (read_size % 4);

			/* Check we're not going reading more than we should */
			assert(count >= read_size);

			uint32_t address = bank->base + offset + read_bytes;

			LOG_INFO("Reading %u bytes from address 0x%08X", read_size, address);

			struct adsp_spi_flash_cmd flash_cmd = {
				.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
				.instruction	= instruction,
				.address		= address,
				.address_bytes	= 4,
				.dummy_bytes	= dummy_bytes,
				.data_out_ptr	= NULL,
				.data_out_bytes	= 0,
				.data_in_ptr	= &buffer[read_bytes],
				.data_in_bytes	= read_size,
				.dma_queue		= NULL,
				.quad_io		= adspsc59x_flash_info->quad_io
			};

			result = adsp_spi_command(target, &flash_cmd);

			rc = adsp_spi_decode_result(result, "Failed to send SPI Flash read command.");
			if (ERROR_OK != rc) {
				LOG_ERROR("Failed after reading %u bytes.", read_bytes);
			} else {
				/* Increment the read bytes, and decrement the count */
				read_bytes += read_size;
				count -= read_size;
			}
		}
	}

	return rc;
}

/**
 * Probe the SPI flash to discover the model and whether it matches a list of known
 * SPI flash devices. Update the bank appropriately and disable cache on the chip.
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
	ADSP_SPI_RESULT result;
	int rc;

	LOG_INFO("Probing ADSP-SC59X chip...");

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	/* If already probed, reset fields and probe again */
	if (adspsc59x_flash_info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		free(bank->sectors);
		bank->sectors = NULL;
		memset(&adspsc59x_flash_info->dev, 0, sizeof(adspsc59x_flash_info->dev));
		adspsc59x_flash_info->probed = false;
	}

	/** We disable Cache completely to stop cache ruining DMA transfers. Flushing of cache for
	 *  every SPI transfer would take too long. If a target-side algorithm is implemented, the
	 *  SPI driver could be reverted back to core (non-DMA) mode and this sledgehammer solution
	 *  could be removed.
	*/
	rc = adspsc59x_disable_cache(target);
	if (ERROR_OK != rc) {
		LOG_ERROR("Failed to disable cache on target device.");
		return rc;
	}

	target_free_all_working_areas(target);

	/* Output available working memory on target */
	uint32_t available_space = target_get_working_area_avail(target);
	LOG_INFO("Target has %uB of available space.", available_space);
	adspsc59x_flash_info->available_space = available_space;

#ifdef DO_QUAD_IO
	/* Set the global variable to the opposite to force it to send the command first time round */
	adspsc59x_flash_info->quad_io = false;
	rc = adspsc59x_quad_io_enable(bank, true);
	if (ERROR_OK != rc) {
		LOG_ERROR("Failed to enable Quad IO mode");
		return rc;
	}
#else
	/* Set the global variable to the opposite to force it to send the command first time round */
	adspsc59x_flash_info->quad_io = true;
	rc = adspsc59x_quad_io_enable(bank, false);
	if (ERROR_OK != rc) {
		LOG_ERROR("Failed to disable Quad IO mode");
		return rc;
	}
#endif

	rc = adspsc59x_poll(bank);
	if (ERROR_OK != rc) {
		LOG_ERROR("Error polling SPI Flash. Either it is busy or communication is failing altogether.");
		return rc;
	}

	/* Find the device ID first */
	uint32_t jedec_id = 0u;
	struct adsp_spi_flash_cmd flash_cmd = {
		.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
		.instruction	= SPIFLASH_READ_ID,
		.address		= 0,
		.address_bytes	= 0,
		.dummy_bytes	= 0,
		.data_out_ptr	= NULL,
		.data_out_bytes	= 0,
		.data_in_ptr	= (uint8_t *)&jedec_id,
		.data_in_bytes	= 3,
		.dma_queue		= NULL,
		.quad_io		= adspsc59x_flash_info->quad_io
	};

	result = adsp_spi_command(target, &flash_cmd);

	rc = adsp_spi_decode_result(result, "Failed to send the SPI flash read device ID command.");
	if (ERROR_OK != rc)
		return rc;

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

	bank->sectors = sectors;
	adspsc59x_flash_info->probed = true;

	return ERROR_OK;
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
	int rc;
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;

	if (adspsc59x_flash_info->probed)
		rc = ERROR_OK;
	else
		rc = adspsc59x_probe(bank);

	return rc;
}

/**
 * TODO: CCES-24914 - Implement sector level protection.
 *
 * See explanation for adspsc59x_protect(...)
 *
*/
static int adspsc59x_protect_check(struct flash_bank *bank)
{
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/**
 * TODO: CCES-24914 - Implement sector level protection.
 *
 * Block/sector protection method seems to vary widely between
 * flash chips. Even within one chip there are multiple methods
 * that can't be used concurrently. Furthermore, these protection
 * schemes might've already been implemented in a certain way
 * by a boot ROM and OpenOCD should not start altering this.
 *
*/
static int adspsc59x_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/**
 * Erase whole memory on SPI flash device.
*/
COMMAND_HANDLER(adspsc59x_mass_erase_handler)
{
	struct flash_bank *bank;
	struct target *target;
	struct adspsc59x_flash_bank *adspsc59x_flash_info;
	ADSP_SPI_RESULT result;
	int rc;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	rc = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != rc)
		return rc;

	target = bank->target;
	adspsc59x_flash_info = bank->driver_priv;

	/* TODO: CCES-24914 - Check protection status, see todo in protect function for details */

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot erase flash. Target is not halted!");
		rc = ERROR_TARGET_NOT_HALTED;
	} else if (!adspsc59x_flash_info->probed) {
		LOG_ERROR("Cannot erase flash as target has not been probed. Please probe target first.");
		rc = ERROR_FLASH_BANK_NOT_PROBED;
	} else if (adspsc59x_flash_info->dev.chip_erase_cmd == 0x00) {
		LOG_ERROR("Mass erase not available for this device");
		rc = ERROR_FLASH_OPER_UNSUPPORTED;
	} else {
		/* All good to proceed */
		/* Set the write enable */
		rc = adspsc59x_write_enable(bank, true, NULL);
		if (rc != ERROR_OK)
			return rc;

		struct adsp_spi_flash_cmd flash_cmd = {
			.device			= (bank->driver == &adspsc59x_flash) ? ADSP_SPI_DEVICE_SC59X : ADSP_SPI_DEVICE_SC59X_A55,
			.instruction	= adspsc59x_flash_info->dev.chip_erase_cmd,
			.address		= 0,
			.address_bytes	= 0,
			.dummy_bytes	= 0,
			.data_out_ptr	= NULL,
			.data_out_bytes	= 0,
			.data_in_ptr	= NULL,
			.data_in_bytes	= 0,
			.dma_queue		= NULL,
			.quad_io		= adspsc59x_flash_info->quad_io
		};

		result = adsp_spi_command(target, &flash_cmd);

		rc = adsp_spi_decode_result(result, "Failed to send the SPI flash erase chip command.");
		if (rc != ERROR_OK)
			return rc;

		/* Poll until erase is complete */
		rc = adspsc59x_poll(bank);
		if (rc != ERROR_OK)
			return rc;
	}

	return rc;
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
	struct adspsc59x_flash_bank *adspsc59x_flash_info = bank->driver_priv;

	if (!adspsc59x_flash_info->probed) {
		command_print(cmd, "ADSP-SC59X SPI Flash not yet probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	command_print(cmd, "ADSP-SC59X SPI Flash Device: %s\n"
			"Size: 0x%X\n"
			"Page Size: 0x%X\n"
			"Sector Size: 0x%X\n",
			adspsc59x_flash_info->dev.name,
			adspsc59x_flash_info->dev.size_in_bytes,
			adspsc59x_flash_info->dev.pagesize,
			adspsc59x_flash_info->dev.sectorsize);

	return ERROR_OK;
}

static const struct command_registration adspsc59x_exec_command_handlers[] = {
	{
		.name		= "mass_erase",
		.handler	= adspsc59x_mass_erase_handler,
		.mode		= COMMAND_EXEC,
		.usage		= "bank_id",
		.help		= "Mass erase entire flash device.",
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

const struct flash_driver adspsc59x_a55_flash = {
	.name				= "adspsc59x_a55",
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

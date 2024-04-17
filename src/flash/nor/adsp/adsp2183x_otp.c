/* SPDX-License Identifier: GPL-2.0-or-later
	Copyright (C) 2022-2024 Analog Devices, Inc. */

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

static const uint8_t adsp83x_otp_algo[] = {
#include "contrib/loaders/flash/adsp83x/otp/2183x_otp.inc"
};

#define ROUNDUP(value, limit) (value + limit - (value % limit))

#define ERROR_STRING_OTP_SUCCESS "No error. Success."
#define ERROR_STRING_OTP_FAILURE "Generic Failure in OTP algorithm."
#define ERROR_STRING_OTP_INT_FAILURE "Failed to register interrupt handler."
#define ERROR_STRING_OTP_INVALID_HANDLE "The given OTP handle is NULL or invalid."
#define ERROR_STRING_OTP_SEMAPHORE_FAILED "Semaphore related failure occurred."
#define ERROR_STRING_OTP_READ_FAILURE "OTP Read failure."
#define ERROR_STRING_OTP_PROG_FAILURE "OTP Program failure."
#define ERROR_STRING_OTP_ALREADY_PROGRAMMED "OTP ALready Programmed."
#define ERROR_STRING_OTP_PROGRAMMED_WRONG "OTP Programmed Incorrectly."
#define ERROR_STRING_OTP_INVALID_CONFIG "Invalid Configuration."
#define ERROR_STRING_OTP_INVALID_ENUM "Invalid ENUM."
#define ERROR_STRING_OTP_SECURITY_FAILURE "Security Failure."
#define ERROR_STRING_OTP_BOUNDARY_ERROR "OTP memory boundary error."

#define OTP_SIZE 0x490

#define ALGO_RESET_HANDLER 0x20020130

// Values derived from algorithm for data buffer address and algo parameter address (g_cfg)
#define ALGO_PARAMETER_ADDRESS 0x2002B600;
#define ALGO_BUFFER_ADDRESS 0x2002B620;

#define BYTE_COUNT 8

#define ALGO_READY 0xFFFFFFFF

#define ALGO_TIMEOUT_KEEP_ALIVE 500
#define ALGO_TIMEOUT_MAX 10000

#define OTP_READ_COMMAND 1
#define OTP_PROGRAM_COMMAND 2
#define OTP_LOCK_COMMAND 3

#define OTP_PROGRAM_DELAY 5

/*** xtensa resume handling ***/
#define USE_ADDRESS_VAL 0
#define USE_PC_VAL 1

#define HANDLE_BREAKPOINTS 0
#define SKIP_BREAKPOINTS 1

#define NO_DEBUG_EXECUTION 0
#define DEBUG_EXECUTION 1
/******************************/

/*!
 *  \enum ADI_OTP_RESULT
 *   Enumeration used by the service to indicate the return status of requested operation.
 */
typedef enum
{
  ADI_OTP_SUCCESS = 0,
  ADI_OTP_FAILURE,
  ADI_OTP_INT_FAILURE,
  ADI_OTP_INVALID_HANDLE,
  ADI_OTP_SEMAPHORE_FAILED,
  ADI_OTP_READ_FAILURE,
  ADI_OTP_PROG_FAILURE,
  ADI_OTP_ALREADY_PROGRAMMED,
  ADI_OTP_PROGRAMMED_WRONG,
  ADI_OTP_INVALID_CONFIG,
  ADI_OTP_INVALID_ENUM,
  ADI_OTP_SECURITY_FAILURE,
  ADI_OTP_BOUNDARY_ERROR
} ADI_OTP_RESULT;

/* Flash helper algorithm parameter block struct */
#define ADSP83X_STATUS_OFFSET 0x0c

struct adsp83x_algo_params {
	uint8_t address[4];
	uint8_t length[4];
	uint8_t command[4];
	uint8_t status[4];
	uint8_t ready[4];
};

/* Internal data structure to allow additional options for flash device */
struct adsp2183x_otp_bank {
	bool probed;				/*! Has the flash device been probed? */
	uint32_t available_space;	/*! Used for sanity checking against memory leaks */
	uint32_t sector_length;
	struct working_area *working_area;
	struct xtensa_algorithm xtensa_info;
	const uint8_t *algo_data;
	uint32_t algo_size;
	uint32_t algo_start_address;
	uint32_t buffer_addr;
	uint32_t params_addr;
};

static int display_algo_error_code(unsigned char status)
{
	switch(status) {
		case ADI_OTP_FAILURE:
			LOG_ERROR("%s", ERROR_STRING_OTP_FAILURE);
			break;
		case ADI_OTP_INT_FAILURE:
			LOG_ERROR("%s", ERROR_STRING_OTP_INT_FAILURE);
			break;
		case ADI_OTP_INVALID_HANDLE:
			LOG_ERROR("%s", ERROR_STRING_OTP_INVALID_HANDLE);
			break;
		case ADI_OTP_SEMAPHORE_FAILED:
			LOG_ERROR("%s", ERROR_STRING_OTP_SEMAPHORE_FAILED);
			break;
		case ADI_OTP_READ_FAILURE:
			LOG_ERROR("%s", ERROR_STRING_OTP_READ_FAILURE);
			break;
		case ADI_OTP_PROG_FAILURE:
			LOG_ERROR("%s", ERROR_STRING_OTP_PROG_FAILURE);
			LOG_ERROR("OTP area may be corrupt or unable to be programmed. Please hard reset and attempt again.");
			break;
		case ADI_OTP_ALREADY_PROGRAMMED:
			LOG_ERROR("%s", ERROR_STRING_OTP_ALREADY_PROGRAMMED);
			break;
		case ADI_OTP_PROGRAMMED_WRONG:
			LOG_ERROR("%s", ERROR_STRING_OTP_PROGRAMMED_WRONG);
			break;
		case ADI_OTP_INVALID_CONFIG:
			LOG_ERROR("%s", ERROR_STRING_OTP_INVALID_CONFIG);
			break;
		case ADI_OTP_INVALID_ENUM:
			LOG_ERROR("%s", ERROR_STRING_OTP_INVALID_ENUM);
			break;
		case ADI_OTP_SECURITY_FAILURE:
			LOG_ERROR("%s", ERROR_STRING_OTP_SECURITY_FAILURE);
			break;
		case ADI_OTP_BOUNDARY_ERROR:
			LOG_ERROR("%s", ERROR_STRING_OTP_BOUNDARY_ERROR);
			break;
		default:
			LOG_ERROR("Uknown error occurred.");
			break;
	}

	return 0;
}

static int adsp83x_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	int retval;

	/* Regardless of the algo's status, attempt to halt the target */
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		return retval;
	}

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

	if (status != 0) {
		display_algo_error_code(status);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}


static int adsp83x_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	int retval;

	/* Check for working area to use for flash helper algorithm */
	adsp2183x_otp_info->working_area = NULL;

	retval = target_alloc_working_area(target, adsp2183x_otp_info->available_space,
				&adsp2183x_otp_info->working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Working address is not correctly allocated");
		return retval;
	}

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, adsp2183x_otp_info->algo_start_address,
				adsp2183x_otp_info->algo_size, adsp2183x_otp_info->algo_data);
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
				ALGO_RESET_HANDLER, 0, &adsp2183x_otp_info->xtensa_info);
	if (retval != ERROR_OK) {
		target_free_working_area(target, adsp2183x_otp_info->working_area);
		adsp2183x_otp_info->working_area = NULL;
		LOG_ERROR("Failure starting the algorithm");
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

	// get status from buffer to determine result of algorithm initialization
	retval = adsp83x_wait_algo_done(bank, adsp2183x_otp_info->params_addr);

	// Resume running algorithm with parameters
	xtensa_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

	/*
	 * At this point, the algorithm is running on the target and
	 * ready to receive commands and data to flash the target
	 */

	return retval;
}
/**
 * Usage:
 * flash bank <name> adsp2183x <base_addr> 0 0 0 <target>
*/
FLASH_BANK_COMMAND_HANDLER(adsp2183x_otp_bank_command)
{
	struct adsp2183x_otp_bank *poInfo;

	/* Check the correct number of arguments have been provided */
	if (CMD_ARGC != 6) {
		LOG_ERROR("Invalid number of flash bank arguments. Usage:\n"
			"flash bank <name> adsp2183x <base_addr> 0 0 0 <target>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	poInfo = malloc(sizeof(struct adsp2183x_otp_bank));
	if (!poInfo) {
		LOG_ERROR("Not enough memory for local driver information.");
		return ERROR_FAIL;
	}

	poInfo->probed = false;
	bank->driver_priv = poInfo;

	return ERROR_OK;
}

/**
 * Erase whole memory
 * Usage:
 * adsp2183x mase_erase bank_id
*/

// No erase for otp
COMMAND_HANDLER(adsp2183x_mass_erase_handler)
{
	LOG_ERROR("Mass erase not available for this device");
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

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
	struct adsp83x_algo_params algo_params;
	int retval;
	int i, byteCountOrig, byteCountRounded;
	uint32_t convertedHex;

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		retval = ERROR_TARGET_NOT_HALTED;
	}
	else if (offset + count > bank->size) {
		LOG_ERROR("Write would go beyond end of supported flash size.");
		retval = ERROR_FLASH_DST_OUT_OF_BANK;
	}
	/* All good to proceed */
	else {
		retval = adsp83x_init(bank);
		if (retval != ERROR_OK)
			return retval;

		byteCountOrig = (count/2) + (count % 2);

		if(byteCountOrig % 4 != 0){
			byteCountRounded = ROUNDUP(byteCountOrig, 4);
		}
		else
			byteCountRounded = byteCountOrig;

		uint32_t tempBuf[byteCountRounded/4];
		uint32_t sendBuf[byteCountRounded/4];

		for (i = 0;	i < byteCountRounded/4;	i++) {
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
		buf_set_u32(algo_params.command, 0, 32, OTP_PROGRAM_COMMAND);

		/* Put next block of data to flash into buffer */
		retval = target_write_buffer(target, adsp2183x_otp_info->buffer_addr,
			byteCountRounded, (void *)sendBuf);

		if(byteCountRounded < 4)
			byteCountRounded = 4;

		// write algo parameters

		buf_set_u32(algo_params.address, 0, 32, offset);
		buf_set_u32(algo_params.length, 0, 32, byteCountRounded);
		buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

		/* Put next block of data to flash into buffer */
		retval = target_write_buffer(target, adsp2183x_otp_info->params_addr,
					sizeof(algo_params), (uint8_t *)&algo_params);

		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write data to target memory");
			target_free_working_area(target, adsp2183x_otp_info->working_area);
			adsp2183x_otp_info->working_area = NULL;
			return retval;
		}

		// Resume running algorithm with parameters
		xtensa_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

		// poll target to update state and wait for algorithm to hit breakpoint to halt target
		while(target->state != TARGET_HALTED) {
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adsp2183x_otp_info->working_area);
				adsp2183x_otp_info->working_area = NULL;
				return retval;
			}
		}

		// get status from buffer to determine result of programming
		retval = adsp83x_wait_algo_done(bank, adsp2183x_otp_info->params_addr);

		/* Regardless of errors, try to close down algo */
		(void)adsp83x_quit(bank);

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
	struct adsp83x_algo_params algo_params;

	int retval;
	uint32_t byteCountRounded;

	/* Check device is halted and has been probed first */
	if (TARGET_HALTED != target->state)
	{
		LOG_ERROR("Cannot read from flash. Target is not halted!");
		retval = ERROR_TARGET_NOT_HALTED;
	}
	/* All good to proceed */
	else
	{
		retval = adsp83x_init(bank);
		if (retval != ERROR_OK)
			return retval;

		if(count % 4 != 0){
			byteCountRounded = ROUNDUP(count, 4);
		}
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
		buf_set_u32(algo_params.command, 0, 32, OTP_READ_COMMAND);

		// write algo parameters
		buf_set_u32(algo_params.address, 0, 32, offset);
		buf_set_u32(algo_params.length, 0, 32, byteCountRounded);
		buf_set_u32(algo_params.ready,  0, 32, ALGO_READY);

		retval = target_write_buffer(target, adsp2183x_otp_info->params_addr,
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
		while(target->state != TARGET_HALTED) {
			retval = target_poll(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to poll target");
				target_free_working_area(target, adsp2183x_otp_info->working_area);
				adsp2183x_otp_info->working_area = NULL;
				return retval;
			}
		}

		/* Put next block of data from flash into buffer */
		retval = target_read_buffer(target, adsp2183x_otp_info->buffer_addr,
		byteCountRounded, recBuf);

		memcpy(buffer, recBuf, count);

		// get status from buffer to determine result of programming
		retval = adsp83x_wait_algo_done(bank, adsp2183x_otp_info->params_addr);

		/* Regardless of errors, try to close down algo */
		(void)adsp83x_quit(bank);

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
	struct target *target = bank->target;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	uint32_t sector_length;
	int num_sectors;

	LOG_INFO("Setting up ADSP2183X otp area...");

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	target_free_all_working_areas(target);

	/* Output available working memory on target */
	uint32_t available_space = target_get_working_area_avail(target);
	LOG_INFO("Target has %uB of available space.", available_space);
	adsp2183x_otp_info->available_space = available_space;

	// Set up target side algo information (Split into two for now)
	adsp2183x_otp_info->algo_data = adsp83x_otp_algo;
	adsp2183x_otp_info->algo_size = sizeof(adsp83x_otp_algo);
	adsp2183x_otp_info->algo_start_address = target->working_area_phys;

	// Values derived from algorithm for data buffer address and algo parameter address (g_cfg)
	adsp2183x_otp_info->buffer_addr = ALGO_BUFFER_ADDRESS;
	adsp2183x_otp_info->params_addr = ALGO_PARAMETER_ADDRESS;

	num_sectors = 1;

	// end of OTP is 0x490
	sector_length = OTP_SIZE;

	bank->size = num_sectors * sector_length;
	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;
	bank->num_sectors = num_sectors;
	adsp2183x_otp_info->sector_length = sector_length;

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
static int adsp2183x_auto_probe(struct flash_bank *bank)
{
	int retval;
	struct adsp2183x_otp_bank *adsp2183x_otp_info = bank->driver_priv;

	if (adsp2183x_otp_info->probed) {
		retval = ERROR_OK;
	}
	else {
		retval = adsp2183x_probe(bank);
	}

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

	if (!adsp2183x_otp_info->probed) {
		command_print(cmd, "ADSP-2183X SPI Flash not yet probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	command_print(cmd, "ADSP-2183X OTP\n"
			"Size: 0x%X\n",
			adsp2183x_otp_info->sector_length);

	return ERROR_OK;
}

static const struct command_registration adsp2183x_exec_command_handlers[] = {
	{
		.name		= "mass_erase",
		.handler	= adsp2183x_mass_erase_handler,
		.mode		= COMMAND_EXEC,
		.usage		= "bank_id",
		.help		= "Mass erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration adsp2183x_command_handlers[] = {
	{
		.name	= "adsp2183x",
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
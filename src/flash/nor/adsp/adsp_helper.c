// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
 *	Copyright (C) 2022-2026 Analog Devices, Inc.							*
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "adsp_helper.h"

int adsp_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp_flash_info = bank->driver_priv;
	int retval;

	/* Regardless of the algo's status, attempt to halt the target */
	retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	/* Now confirm target halted and clean up from flash helper algorithm */
	retval = target_wait_algorithm(target, 0, NULL, 0, NULL, 0, ALGO_TIMEOUT_MAX, &adsp_flash_info->xtensa_info);

	target_free_working_area(target, adsp_flash_info->working_area);
	adsp_flash_info->working_area = NULL;

	return retval;
}

int adsp_wait_algo_done(struct flash_bank *bank, uint32_t params_addr)
{
	struct target *target = bank->target;
	uint32_t status_addr = params_addr + ADSP_STATUS_OFFSET;
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

int wait_for_breakpoint_and_check_status(struct flash_bank *bank, long long timeout)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp_flash_info = bank->driver_priv;
	int retval;

	// poll target to update state and wait for algorithm to hit breakpoint to halt target
	if (adsp_target_poll_check_state(bank, TARGET_HALTED, timeout))
		return ERROR_FAIL;

	// get status from buffer to determine result of algorithm initialization
	retval = adsp_wait_algo_done(bank, adsp_flash_info->adsp_algorithm.parameter_address);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error detected in algorithm command execution. Closing down algorithm.");
		/* Close down algo */
		(void)adsp_quit(bank);
	} else {
		// Resume running algorithm with parameters
		xtensa_resume(target, USE_PC_VAL, 0, SKIP_BREAKPOINTS, DEBUG_EXECUTION);

		/*
		 * At this point, the algorithm is running on the target and
		 * ready to receive commands and data to flash the target
		 */
	}

	return retval;
}

int adsp_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp_flash_info = bank->driver_priv;
	int retval;

	if (!adsp_flash_info) {
		LOG_ERROR("Flashing commands will fail as flash bank is incomplete without .inc files");
		return ERROR_FAIL;
	}

	/* Check for working area to use for flash helper algorithm */
	adsp_flash_info->working_area = NULL;

	retval =
		target_alloc_working_area(target, adsp_flash_info->available_space, &adsp_flash_info->working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Working address is not correctly allocated");
		return retval;
	}

	// Need to halt before reads/writes
	retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Target is not halted!");
		target_free_working_area(target, adsp_flash_info->working_area);
		adsp_flash_info->working_area = NULL;
		return retval;
	}

	// poll target to update state
	if (adsp_target_poll_check_state(bank, TARGET_HALTED, ALGO_TIMEOUT_MAX)) {
		LOG_ERROR("Cannot initialize target. Target is not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, adsp_flash_info->adsp_algorithm.algo_start_address,
								 adsp_flash_info->adsp_algorithm.size,
								 adsp_flash_info->adsp_algorithm.adsp_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load flash helper algorithm");
		target_free_working_area(target, adsp_flash_info->working_area);
		adsp_flash_info->working_area = NULL;
		return retval;
	}

	/* Initialize the Xtensa specific info to run the algorithm */
	adsp_flash_info->xtensa_info.core_mode = XT_MODE_ANY;

	/* Begin executing the flash helper algorithm */
	retval =
		target_start_algorithm(target, 0, NULL, 0, NULL, adsp_flash_info->adsp_algorithm.reset_handler_addr,
							   0, &adsp_flash_info->xtensa_info);
	if (retval != ERROR_OK) {
		target_free_working_area(target, adsp_flash_info->working_area);
		adsp_flash_info->working_area = NULL;
		LOG_ERROR("Failure starting the algorithm");
		return retval;
	}

	retval = wait_for_breakpoint_and_check_status(bank, ALGO_TIMEOUT_MAX);

	return retval;
}

int adsp_run_flash_command(struct flash_bank *bank, long long timeout)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp_flash_info = bank->driver_priv;
	struct adsp_algo_params *adsp_algo_params = &adsp_flash_info->algo_params;

	int retval;

	retval = target_write_buffer(target, adsp_flash_info->adsp_algorithm.parameter_address,
						sizeof(*adsp_algo_params), (uint8_t *)adsp_algo_params);
	if (retval != ERROR_OK)
		return retval;

	// Resume running algorithm with parameters
	xtensa_resume(target, USE_PC_VAL, 0, HANDLE_BREAKPOINTS, DEBUG_EXECUTION);

	retval = wait_for_breakpoint_and_check_status(bank, timeout);

	if (retval != ERROR_OK) {
		/* Close down algo */
		(void)adsp_quit(bank);
		LOG_ERROR_ALGO_PARAMS(adsp_flash_info->algo_params);
		return retval;
	}

	return retval;
}

int adsp_target_poll_check_state(struct flash_bank *bank, enum target_state expected_state, long long timeout)
{
	struct target *target = bank->target;
	struct adsp_flash_bank *adsp_flash_info = bank->driver_priv;

	int retval = ERROR_OK;
	long long start_ms;
	long long elapsed_ms;
	long long timeout_ms;

	timeout_ms = timeout;

	start_ms = timeval_ms();

	// poll target to update state and wait for algorithm to hit breakpoint to halt target
	while (target->state != expected_state) {
		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > ALGO_TIMEOUT_KEEP_ALIVE)
			keep_alive();
		if (elapsed_ms > timeout_ms) {
			LOG_ERROR("Timeout during algorithm command execution");
			/* Close down algo */
			(void)adsp_quit(bank);
			return ERROR_FAIL;
		}

		retval = target_poll(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to poll target");
			target_free_working_area(target, adsp_flash_info->working_area);
			adsp_flash_info->working_area = NULL;
			return retval;
		}

		if (target->state != expected_state && expected_state == TARGET_DEBUG_RUNNING)
			return ERROR_FAIL;
	}

	return retval;
}

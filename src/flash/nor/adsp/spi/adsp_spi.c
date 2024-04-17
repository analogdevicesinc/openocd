/* SPDX-License-Identifier: GPL-2.0-or-later */

/****************************************************************************
 *	Copyright (C) 2022-2024 Analog Devices, Inc.							*
 ***************************************************************************/

#include "adsp_spi.h"
#include "adsp_spi_device.h"
#include "adsp_spi_dbg.h"
#include "adsp_regs_common.h"

/* The number of writes to make to the wait register to be sure that
 * SPI transfer has completed.
 */
#define WAIT_LOOP_ITERATIONS 100000u

/* This sets a max size for the zero buffer, which is used to zero out
 *  register blocks in one write. Should be set to the largest register
 *  block being zeroed or greater.
*/
#define REG_ZERO_BUFFER_SIZE	(256u)

/* When we run out of space in the DMA queue, increase the size of the
 * size of the alloc'd arrays by this multiple.
*/
#define REALLOC_MULTIPLIER (2)

/* How many MDMA descriptors do we need per page, on average?
*/
#define APPROX_NUM_MDMA_DESCRIPTORS_PER_TRANSFER 10u

/* How many PDMA descriptors do we need per page, on average?
*/
#define APPROX_NUM_PDMA_DESCRIPTORS_PER_TRANSFER 3u

/* How many bytes of MDMA data do we need in total?
*/
#define APPROX_NUM_MDMA_DATA_BYTES 80u

/* How many bytes of PDMA data do we need per page, on average?
*/
#define APPROX_NUM_EXTRA_PDMA_DATA_PER_TRANSFER 8u

static const uint8_t zeroed_buffer[REG_ZERO_BUFFER_SIZE] = {0u};

/* Used to track if pinmux has been configured */
static bool init_done = false;

/* Used to track the current SPI device in use, set in _command */
struct adsp_spi_device device = {0u};

/*! Defines the current maximum word count within the SPI driver */
uint32_t spi_max_word_count;

/*! Defines the required extra space within the target side buffer for DMA transfers */
unsigned spi_dma_buffer_minimum_size;

uint32_t adsp_config_to_descriptor_size(uint32_t config)
{
	/* The NDSIZE field encodes the size of the next descriptor as one
	 * less than its size in words.
	 */
	return ((config & BITM_DMA_CFG_NDSIZE) >> BITP_DMA_CFG_NDSIZE) + 1u;
}

void adsp_init_tru(struct target *target)
{
	/* Enable the TRU */
	target_write_u32(target, device.tru.gctl, BITM_TRU_GCTL_EN);

	/* Route the MDMA to PDMA and vice versa */
	target_write_u32(target, device.tru.mdma_receiver, device.tru.pdma_producer);
	target_write_u32(target, device.tru.pdma_receiver, device.tru.mdma_producer);
}

/**
 * Allocate space for a new DMA channel, given hints about how large it needs to be,
 * and initialise any fields we need to.
 *
 * @param channel			The DMA channel we're allocating buffers in
 * @param descriptors_hint	The expected size of the DMA descriptors required
 * @param data_hint			The expected size of the DMA buffer required, in bytes
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_init_dma_channel(struct adsp_dma_channel_data *channel,
											 uint32_t descriptors_hint, uint32_t data_hint)
{
	if (descriptors_hint > 0)
	{
		channel->max_chain_size = descriptors_hint;
		channel->descriptors = malloc(channel->max_chain_size * sizeof(uint32_t));
		if (!channel->descriptors)
		{
			return ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
		}
	}

	if (data_hint > 0)
	{
		channel->data = malloc(data_hint);
		if (!channel->data)
		{
			return ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
		}
		channel->max_data_size = data_hint;
	}

	channel->last_config = &channel->first_config;

	return ADSP_SPI_RESULT_SUCCESS;
}

ADSP_SPI_RESULT adsp_init_dma_queue(struct adsp_dma_queue *queue,
									uint32_t num_bytes,
									uint32_t page_size)
{
	ADSP_SPI_RESULT result;
	uint32_t expected_mdma_descriptor_size = 0u;
	uint32_t expected_pdma_descriptor_size = 4u;
	uint32_t expected_mdma_data_bytes = 0u;
	uint32_t expected_pdma_data_bytes = num_bytes;
	if (page_size > 0u)
	{
		/* If we come in here, we are building a descriptor chain for transmitting multiple pages.
		* The following calculations don't need to be perfect as we do a realloc if required, but we might
		* as well try to size the buffers correctly to start with.
		*/
		/* We transmit a page at a time, so calculate the number of pages */
		uint32_t num_pages = (num_bytes + page_size - 1u) / page_size;
		/* The MDMA descriptors mostly use 2 words per descriptor */
		expected_mdma_descriptor_size = 2u * num_pages * APPROX_NUM_MDMA_DESCRIPTORS_PER_TRANSFER;
		/* The PDMA descriptors mostly use 4 words per descriptor */
		expected_pdma_descriptor_size = 4u * num_pages * APPROX_NUM_PDMA_DESCRIPTORS_PER_TRANSFER;
		/* The MDMA data comprises register configuration values, with each value only stored once in the buffer */
		expected_mdma_data_bytes = APPROX_NUM_MDMA_DATA_BYTES;
		/* The PDMA data comprises the data we're sending, plus some extra for configuration */
		expected_pdma_data_bytes = num_bytes + (num_pages * APPROX_NUM_EXTRA_PDMA_DATA_PER_TRANSFER);
	}

	memset(queue, 0, sizeof(*queue));

	result = adsp_init_dma_channel(&queue->mdma_tx, expected_mdma_descriptor_size, expected_mdma_data_bytes);
	if (result != ADSP_SPI_RESULT_SUCCESS)
	{
		return result;
	}
	result = adsp_init_dma_channel(&queue->mdma_rx, expected_mdma_descriptor_size, 0);
	if (result != ADSP_SPI_RESULT_SUCCESS)
	{
		return result;
	}
	result = adsp_init_dma_channel(&queue->pdma_tx, expected_pdma_descriptor_size, expected_pdma_data_bytes);
	if (result != ADSP_SPI_RESULT_SUCCESS)
	{
		return result;
	}

	return ADSP_SPI_RESULT_SUCCESS;
}

void adsp_destroy_dma_queue(struct adsp_dma_queue *queue)
{
	free(queue->mdma_tx.descriptors);
	free(queue->mdma_rx.descriptors);
	free(queue->mdma_tx.data);
	free(queue->pdma_tx.descriptors);
	free(queue->pdma_tx.data);
}

/**
 * Allocate space (if necessary) for adding a new DMA descriptor to the channel, and for
 * adding its data to the transfer buffer.
 *
 * @param channel			The DMA channel we're allocating buffers in
 * @param size				The unit size of the transfer in bytes
 * @param count				The number of units we're transferring
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_inc_dma_queue(struct adsp_dma_channel_data *channel, uint32_t size, uint32_t count)
{
	/* First make sure we have room for the descriptor */
	if ((channel->chain_size + sizeof(adsp_1d_dma_array_desc) / sizeof(uint32_t)) > channel->max_chain_size)
	{
		uint32_t last_config_idx = channel->last_config - channel->descriptors;
		channel->max_chain_size *= REALLOC_MULTIPLIER;
		channel->descriptors = realloc(channel->descriptors, channel->max_chain_size * sizeof(uint32_t));
		if (!channel->descriptors)
		{
			return ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
		}
		channel->last_config = channel->descriptors + last_config_idx;
	}
	if (count > 0u)
	{
		/* Next make sure we have room for the data.
		* Add one to count to make sure we have enough space for alignment
		*/
		uint32_t size_needed = channel->data_size + (size * (count + 1));
		if (size_needed > channel->max_data_size)
		{
			channel->max_data_size = size_needed * REALLOC_MULTIPLIER;
			channel->data = realloc(channel->data, channel->max_data_size);
			if (!channel->data)
			{
				return ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
			}
		}
	}
	return ADSP_SPI_RESULT_SUCCESS;
}

/**
 * Start a DMA queue, by transferring the data and descriptors and enabling the DMA channel.
 *
 * @param target			Pointer to the target device to use
 * @param channel			The DMA channel to start
 * @param regs				The structure defining the registers for the DMA channel
 * @param send_data			True if we want to send the data to the target
 * @param is_tx_channel		True if this is a TX channel, and so needs data
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_start_dma_channel(struct target *target, struct adsp_dma_channel_data *channel,
											struct adsp_dma_regs *regs, bool send_data, bool is_tx_channel)
{
	int rc;

	if (channel->num_in_queue > 0)
	{
		if (is_tx_channel)
		{
			/* If we retained the data_target area, we are asserting that the same amount of data will be used again this time using
			* the same addresses, and so the area of memory is used on the target.
			*/
			if (!channel->data_target)
			{
				if (ERROR_OK != target_alloc_working_area(target, channel->data_size, &channel->data_target))
				{
					LOG_ERROR("No available memory on target for DMA buffer.");
					return ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
				}
			}

			if (send_data)
			{
				/* Write the data to the target, even if we previously allocated target space, because the data has changed.
				*/
				rc = target_write_buffer(target, channel->data_target->address, channel->data_size, channel->data);
				/* Check the above memory write was successful */
				if (ERROR_OK != rc)
				{
					LOG_ERROR("Failed to write DMA data buffer to target memory.");
					return ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
				}
			}
		}

		/* If we retained the descriptors_target area, we are asserting that the data there will be the same again this time, and
		 * so does not have to be transferred again to the target.
		 */
		if (!channel->descriptors_target)
		{
			if (is_tx_channel)
			{
				/* Fill in the target addresses for the data sources */
				uint32_t addr_idx = 0;
				uint32_t desc_size = adsp_config_to_descriptor_size(channel->first_config);
				for (uint32_t i = 0; i < channel->num_in_queue; i++)
				{
					adsp_1d_dma_array_desc *desc = (adsp_1d_dma_array_desc *)(&channel->descriptors[addr_idx]);
					desc->address_start += channel->data_target->address;
					addr_idx += desc_size;
					desc_size = adsp_config_to_descriptor_size(desc->config);
				}
			}

			if (ERROR_OK != target_alloc_working_area(target, channel->chain_size * sizeof(uint32_t), &channel->descriptors_target))
			{
				LOG_ERROR("No available memory on target for DMA descriptors.");
				return ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
			}

			/* Write the descriptor to memory */
			rc = target_write_buffer(target, channel->descriptors_target->address, channel->chain_size * sizeof(uint32_t), (uint8_t *)channel->descriptors);
			/* Check the above memory write was successful */
			if (ERROR_OK != rc)
			{
				LOG_ERROR("Failed to write DMA descriptor to target memory.");
				return ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
			}
		}
		else
		{
			/* We use the descriptors already on the target, but make sure we use the same first config value
			 * so that we load the correct first descriptor size.
			 */
			channel->first_config = channel->saved_first_config;
		}

		/* Configure Descriptor Current Register with the location of the first descriptor */
		target_write_u32(target, regs->descriptor_current, channel->descriptors_target->address);

		/* Configure and enable the DMA in one write */
		target_write_u32(target, regs->config, channel->first_config);

		/* Record the first config in case we need it for next time */
		channel->saved_first_config = channel->first_config;
	}

	return ADSP_SPI_RESULT_SUCCESS;
}

/**
 * Free a DMA channel, releasing space we won't use again.
 *
 * @param target			Pointer to the target device to use
 * @param queue				The DMA queue
 * @param channel			The DMA channel to start
*/
static void adsp_free_channel(struct target *target, struct adsp_dma_queue *queue, struct adsp_dma_channel_data *channel)
{
	if (channel->num_in_queue > 0)
	{
		if (!queue->keep_descriptors)
		{
			/* Free the descriptors */
			if (channel->descriptors_target != NULL)
			{
				target_free_working_area(target, channel->descriptors_target);
				channel->descriptors_target = NULL;
			}

			/* Free the data */
			if (channel->data_target)
			{
				target_free_working_area(target, channel->data_target);
				channel->data_target = NULL;
			}
		}

		/* Descriptors now done */
		channel->num_in_queue = 0;
		channel->data_size = 0;
		channel->chain_size = 0;
		channel->last_config = &channel->first_config;
	}
}

ADSP_SPI_RESULT adsp_run_queue(struct target *target, struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result;

	/* Print out the registers before kicking off the DMAs */
	_debug_print_registers(target);

	/* Transfer data and descriptors to the target, and enable the DMAs */
	result = adsp_start_dma_channel(target, &queue->pdma_tx, &device.pdma_tx, true, true);
	if (result != ADSP_SPI_RESULT_SUCCESS)
	{
		return result;
	}
	result = adsp_start_dma_channel(target, &queue->mdma_tx, &device.mdma_tx, !queue->mdma_tx.data_target, true);
	if (result != ADSP_SPI_RESULT_SUCCESS)
	{
		return result;
	}
	result = adsp_start_dma_channel(target, &queue->mdma_rx, &device.mdma_rx, false, false);
	if (result != ADSP_SPI_RESULT_SUCCESS)
	{
		return result;
	}

	uint32_t poll_reg = device.mdma_rx.status;
	/* If there isn't a subsequent dependent MDMA access, wait for PDMA to finish */
	if (queue->pdma_tx.num_in_queue > 0 && (*queue->pdma_tx.last_config & BITM_DMA_CFG_TRIG) != ENUM_DMA_CFG_XCNT_TRIG)
	{
		poll_reg = device.pdma_tx.status;
	}

	/* Form retry counter */
	unsigned retry_counter = MAX_RETRY_TIMEOUT;

	/* Convenient point to print out the descriptors */
	_debug_print_dma_queue(target, queue);

	/* Wait until DMAs are complete */
	uint32_t status;
	uint32_t sleep_ms = 0u;
	do
	{
		if (sleep_ms > 0u)
			alive_sleep(sleep_ms);
		target_read_u32(target, poll_reg, &status);

		if (--retry_counter == 0)
		{
			LOG_ERROR("Timed out waiting for DMA transfer to finish.");
			result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
			_debug_print_registers(target);
			break;
		}
		sleep_ms = 100;
	} while ((status & BITM_DMA_STAT_RUN) != 0u);

	/* Now we've run the queue, reset the queue to default state. It's empty
	* so last_was_pdma is now false.
	*/
	queue->last_was_pdma = false;

	/* Free DMA channel data - they're finished now */
	adsp_free_channel(target, queue, &queue->pdma_tx);
	adsp_free_channel(target, queue, &queue->mdma_tx);
	adsp_free_channel(target, queue, &queue->mdma_rx);

	return result;
}

/**
 * Queue up a new descriptor for a DMA channel.
 *
 * @param channel			The DMA channel to start
 * @param address			The address for the DMA data
 * @param unit_size_enum	The unit size of the transfers
 * @param count				The transfer count for the DMA
 * @param inc				The X increment
 * @param is_tx_channel		True if this is a TX channel, and so needs data
 * @param is_pdma_channel	True if this is a PDMA channel
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static void adsp_queue_dma_descriptor(struct adsp_dma_channel_data *channel, uint32_t address,
									ADSP_SPI_UNIT_SIZE unit_size_enum,
									uint32_t count, uint32_t inc, bool is_tx_channel, bool is_pdma_channel)
{
	uint32_t dma_cfg_reg;

	/* Set the Configuration for the DMA register */
	dma_cfg_reg = (unit_size_enum << BITP_DMA_CFG_MSIZE)
		| (unit_size_enum << BITP_DMA_CFG_PSIZE)
		| (is_tx_channel ? ENUM_DMA_CFG_READ : ENUM_DMA_CFG_WRITE)
		| ENUM_DMA_CFG_ADDR1D
		| (is_pdma_channel ? BITM_DMA_CFG_SYNC : 0)
		| ENUM_DMA_CFG_EN;


	if (channel->num_in_queue == 0)
	{
		channel->first_config = dma_cfg_reg;
	}

	adsp_1d_dma_array_desc descriptor = {
		.address_start = address,
		.config = dma_cfg_reg,
		.x_count = count,
		.x_increment = inc
	};

	/* Work out the minimum descriptor size we need. We don't need to transfer registers
	 * at the end of the descriptor that aren't changing from their current values.
	 */
	uint32_t desc_size = (sizeof(adsp_1d_dma_array_desc)/sizeof(uint32_t));
	uint32_t dma_cfg_fetch_size = ENUM_DMA_CFG_FETCH04;
	if (descriptor.x_increment == channel->current_regs.x_increment)
	{
		desc_size--;
		dma_cfg_fetch_size = ENUM_DMA_CFG_FETCH03;
		if (descriptor.x_count == channel->current_regs.x_count)
		{
			desc_size--;
			dma_cfg_fetch_size = ENUM_DMA_CFG_FETCH02;
		}
	}
	*channel->last_config |= (dma_cfg_fetch_size | ENUM_DMA_CFG_DSCARRAY);

	memcpy(&channel->descriptors[channel->chain_size], &descriptor, desc_size * sizeof(uint32_t));
	adsp_1d_dma_array_desc *desc = (adsp_1d_dma_array_desc *)&channel->descriptors[channel->chain_size];
	channel->last_config = &desc->config;
	channel->chain_size += desc_size;
	channel->num_in_queue++;
	channel->current_regs = descriptor;
}

/**
 * Write a 32-bit value to a target address n times, either immediately (if queue == NULL) or add to the MDMA queue to kick off later.
 *
 * @param target			Pointer to the target device to use
 * @param address			The target address to write to
 * @param count				The number of times to write
 * @param value				The value to write
 * @param queue				The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_write_or_queue_n_x_u32(struct target *target, target_addr_t address,
												uint32_t count, uint32_t value, struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result = ADSP_SPI_RESULT_SUCCESS;

	if (!queue)
	{
		int rc = target_write_u32(target, address, value);
		if (ERROR_OK != rc)
		{
			return ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
		}
	}
	else
	{
		uint32_t offset;

		/* Look for an existing copy of this data in the array. It would be better to hash, but there are very few
		* distinct values.
		*/
		uint32_t idx;
		uint32_t *buffer = (uint32_t *)queue->mdma_tx.data;
		bool allocate_data = true;
		for (idx = 0u; idx < (queue->mdma_tx.data_size / sizeof(uint32_t)); idx++)
		{
			if (value == buffer[idx])
			{
				allocate_data = false;
				break;
			}
		}
		offset = idx * sizeof(uint32_t);

		/* Allocate space for the descriptors, and for the data if we need it */
		result = adsp_inc_dma_queue(&queue->mdma_tx, sizeof(uint32_t), allocate_data ? 1u : 0u);
		if (result != ADSP_SPI_RESULT_SUCCESS)
		{
			LOG_INFO("Failed to realloc MDMA TX queue");
			return result;
		}
		/* Reset buffer just in case the above call did a realloc */
		buffer = (uint32_t *)queue->mdma_tx.data;
		result = adsp_inc_dma_queue(&queue->mdma_rx, 0u, 0u);
		if (result != ADSP_SPI_RESULT_SUCCESS)
		{
			LOG_INFO("Failed to realloc MDMA RX queue");
			return result;
		}

		/* Set the value to transfer in the buffer */
		if (allocate_data)
		{
			buffer[idx] = value;
			queue->mdma_tx.data_size += sizeof(uint32_t);
		}

		/* Queue up a TX descriptor.
		 * Store the offset into buffer as the address for the TX transfer.
		 * We later add the target start address
		 */
		adsp_queue_dma_descriptor(&queue->mdma_tx, offset, ADSP_SPI_UNIT_SIZE_WORD, count, 0, true, false);

		/* Queue up the RX descriptor */
		uint32_t *last_mdma_rx_config = queue->mdma_rx.last_config;
		adsp_queue_dma_descriptor(&queue->mdma_rx, address, ADSP_SPI_UNIT_SIZE_WORD, count, 0, false, false);

		if (queue->last_was_pdma)
		{
			/* Make last PDMA a trigger for this */
			*queue->pdma_tx.last_config |= ENUM_DMA_CFG_XCNT_TRIG;
			/* And wait for that to come through */
			*last_mdma_rx_config |= ENUM_DMA_CFG_TRGWAIT;
		}

		queue->last_was_pdma = false;
	}
	return result;
}

/**
 * Add a descriptor to wait by doing benign writes to memory
 *
 * @param target			Pointer to the target device to use
 * @param count				The number of writes to perform
 * @param queue				The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_dma_wait(struct target *target, uint32_t count, struct adsp_dma_queue *queue)
{
	return adsp_write_or_queue_n_x_u32(target, device.wait_reg, count, 0u, queue);
}

/**
 * Write a 32-bit value to a target address, either immediately (if queue == NULL) or add to the MDMA queue to kick off later.
 *
 * @param target			Pointer to the target device to use
 * @param address			The target address to write to
 * @param value				The value to write
 * @param queue				The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_write_or_queue_u32(struct target *target, target_addr_t address,
											uint32_t value, struct adsp_dma_queue *queue)
{
	return adsp_write_or_queue_n_x_u32(target, address, 1, value, queue);
}

/**
 * Write a buffer to a target address, either immediately (if queue == NULL) or add to the MDMA queue to kick off later.
 *
 * @param target			Pointer to the target device to use
 * @param address			The target address to write to
 * @param size				The unit size of the data
 * @param count				The number of units
 * @param buffer			The buffer of data to write
 * @param queue				The DMA queue to add target writes to, or NULL to commit immediately
*/
static void adsp_write_or_queue_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count,
									const uint8_t *buffer, struct adsp_dma_queue *queue)
{
	if (queue)
	{
		uint32_t i;
		for (i = 0u; i < count; i++)
		{
			switch (size)
			{
				case 1:
					adsp_write_or_queue_u32(target, address + (i * size), buffer[i], queue);
					break;
				case 2:
					adsp_write_or_queue_u32(target, address + (i * size), ((uint16_t *)buffer)[i], queue);
					break;
				case 4:
					adsp_write_or_queue_u32(target, address + (i * size), ((uint32_t *)buffer)[i], queue);
					break;
			}
		}
	}
	else
	{
		target_write_memory(target, address, size, count, buffer);
	}
}

/**
 * Send data to the SPI via DMA, either immediately (if queue == NULL) or add to the PDMA queue to kick off later.
 *
 * @param target			Pointer to the target device to use
 * @param unit_size_enum	The enumeration value for the unit size of the transfers
 * @param count				The number of units to send
 * @param buffer			The buffer of data to send
 * @param queue				The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_send_or_queue_memory(struct target *target, ADSP_SPI_UNIT_SIZE unit_size_enum,
												 uint32_t count, const uint8_t *buffer, struct adsp_dma_queue *queue_in)
{
	ADSP_SPI_RESULT result = ADSP_SPI_RESULT_SUCCESS;
	uint32_t size;

	switch (unit_size_enum)
	{
		case ADSP_SPI_UNIT_SIZE_BYTE:
			size = 1;
			break;
		case ADSP_SPI_UNIT_SIZE_HALFWORD:
			size = 2;
			break;
		case ADSP_SPI_UNIT_SIZE_WORD:
			size = 4;
			break;
		default:
			return ADSP_SPI_RESULT_CONFIGURATION_INVALID;
	}

	struct adsp_dma_queue *queue = queue_in;
	struct adsp_dma_queue local_queue;

	if (!queue)
	{
		result = adsp_init_dma_queue(&local_queue, size * count, 0);
		if (result != ADSP_SPI_RESULT_SUCCESS)
		{
			LOG_INFO("Failed to allocate PDMA queue");
			return result;
		}
		queue = &local_queue;
	}
	else
	{
		result = adsp_inc_dma_queue(&queue->pdma_tx, size, count);
		if (result != ADSP_SPI_RESULT_SUCCESS)
		{
			LOG_INFO("Failed to realloc PDMA queue");
			return result;
		}
	}

	/* Compute the address offset, aligned to the size */
	if (queue->pdma_tx.data_size % size != 0)
	{
		queue->pdma_tx.data_size += (size - (queue->pdma_tx.data_size % size));
	}

	/* Queue up a TX descriptor.
	 * Store the offset into buffer as the address for the TX transfer.
	 * We later add the target start address
	 */
	uint32_t *pdma_tx_last_config = queue->pdma_tx.last_config;
	adsp_queue_dma_descriptor(&queue->pdma_tx, queue->pdma_tx.data_size, unit_size_enum, count, size, true, true);

	/** The SPI Block is configured to transmit data MSByte first, but the SPI flash reads
	 *  data in byte by byte, LSByte first. That means we need to reverse the byte order
	 *  depending on the configured word size. An easier way would be to configure the SPI
	 *  block to be LSByte first but the only option is LSBit first which is no good.
	*/
	uint8_t *queued_buffer = &queue->pdma_tx.data[queue->pdma_tx.data_size];
	if (ADSP_SPI_UNIT_SIZE_WORD == unit_size_enum)
	{
		buf_bswap32(queued_buffer, buffer, size * count);
	}
	else if (ADSP_SPI_UNIT_SIZE_HALFWORD == unit_size_enum)
	{
		buf_bswap16(queued_buffer, buffer, size * count);
	}
	else
	{
		/* If we're in byte mode nothing to do for endianness */
		memcpy(queued_buffer, buffer, size * count);
	}
	queue->pdma_tx.data_size += (size * count);

	if (queue->mdma_rx.num_in_queue > 0)
	{
		/* The assumption is that each PDMA transfer is triggered by, and triggers,
		* an MDMA transfer. In other words, there is MDMA-controlled configuration of
		* MMRs between each PDMA transfer.
		*/
		/* Make last MDMA a trigger for this */
		*queue->mdma_rx.last_config |= ENUM_DMA_CFG_XCNT_TRIG;
		/* And wait for that to come through */
		*pdma_tx_last_config |= ENUM_DMA_CFG_TRGWAIT;
	}

	queue->last_was_pdma = true;

	if (!queue_in)
	{
		/* Kick it off right away */
		result = adsp_run_queue(target, queue);
		adsp_destroy_dma_queue(queue);
	}

	return result;
}

/**
 * Control the Chip Select Manually through Software.
 *
 * @param target	Pointer to the target device to use
 * @param assert	If true, assert the CS line. If false, deassert
 * @param queue		The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_spi_cs_control(struct target *target, const bool assert, struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result;
	if (assert)
	{
		/* Clear the bit, i.e. set pin low */
		result = adsp_write_or_queue_u32(target,
			device.chip_select.data_clear_reg,
			(1uL << device.chip_select.pin_number), queue);
	}
	else
	{
		/* Set the bit, i.e. set pin high */
		result = adsp_write_or_queue_u32(target,
			device.chip_select.data_set_reg,
			(1uL << device.chip_select.pin_number), queue);
	}
	return result;
}

/**
 * Enable/disable the SPI block by setting/clearing the enable bit.
 *
 * @param target	Pointer to the target device to use
 * @param enable	If true, enable the SPI block. If false, disable the SPI block
 * @param queue		The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_spi_enable(struct target *target, const bool enable, struct adsp_dma_queue *queue)
{
	if (enable)
	{
		device.spi.control_value |= BITM_SPI_CTL_EN;
	}
	else
	{
		device.spi.control_value &= ~BITM_SPI_CTL_EN;
	}

	return adsp_write_or_queue_u32(target, device.spi.control, device.spi.control_value, queue);
}

/**
 * Initial setup for SPI device. Does NOT need to be called before
 * every transfer.
 *
 * @param target	Pointer to the target device to use
*/
static void adsp_spi_init(struct target *target)
{
	/* Set the desired pins in the multiplexer register */
	target_write_u32(target,
		device.pinmux.port_mux_reg,
		device.pinmux.port_mux_cfg);

	/* Configure the same pins in the function enable register */
	target_write_u32(target,
		device.pinmux.port_function_reg,
		device.pinmux.port_function_cfg);

	/* Configure direction and set pin high */
	target_write_u32(target,
		device.chip_select.direction_set_reg,
		(1uL << device.chip_select.pin_number));
	target_write_u32(target,
		device.chip_select.data_set_reg,
		(1uL << device.chip_select.pin_number));

	if (device.smpu.secure_ctl != 0) {
		/* Enable read and writes to L2 memory through DMA (SMPU) */
		target_write_u32(target, device.smpu.secure_ctl,
			(BITM_SMPU_SECURECTL_RNSEN | BITM_SMPU_SECURECTL_WNSEN));
	}

	// Allow secure accesses from MDMA
	uint32_t securep;
	if (device.spu.secure_periph)
	{
		target_read_u32(target, device.spu.secure_periph, &securep);
		target_write_u32(target, device.spu.secure_periph, securep | BITM_SPU_SECUREP_MSEC);
	}
}

/**
 * Reset the SPI and DMA registers to their default state.
 *
 * @param target			Pointer to the target device to use
 * @param reset_mdma_chain	Reset the registers used for MDMA chains
 * @param queue				The DMA queue to add target writes to, or NULL to commit immediately
*/
static void adsp_spi_reset_registers(struct target *target, bool reset_mdma_chain, struct adsp_dma_queue *queue)
{
	/* First zero SPI registers */
	if (!queue)
	{
		/* Put everything into a known good state if we're not in a DMA chain.
		 * If we're in a DMA chain, assume things have already been reset as necessary.
		 */
		adsp_write_or_queue_memory(target,
			device.spi.base,
			sizeof(uint32_t),
			(device.spi.size/sizeof(uint32_t)),
			zeroed_buffer,
			queue);

		/* Configure CLK */
		adsp_write_or_queue_u32(target, device.spi.clk, device.spi_baud, queue);
	}

	if (device.use_dma)
	{
		if (!queue)
		{
			/* If we're using DMA, should clear out the configuration registers as well */
			target_write_memory(target,
				device.pdma_tx.base,
				sizeof(uint32_t),
				(device.pdma_tx.size/sizeof(uint32_t)),
				zeroed_buffer);

			target_write_memory(target,
				device.pdma_rx.base,
				sizeof(uint32_t),
				(device.pdma_rx.size/sizeof(uint32_t)),
				zeroed_buffer);

			if (device.use_tx_dma_chain && reset_mdma_chain)
			{
				target_write_memory(target,
					device.mdma_tx.base,
					sizeof(uint32_t),
					(device.mdma_tx.size/sizeof(uint32_t)),
					zeroed_buffer);

				target_write_memory(target,
					device.mdma_rx.base,
					sizeof(uint32_t),
					(device.mdma_rx.size/sizeof(uint32_t)),
					zeroed_buffer);
			}

			// Write 1s to all bits to clear W1C in status registers
			target_write_u32(target, device.pdma_tx.status, UINT32_MAX);
			target_write_u32(target, device.pdma_rx.status, UINT32_MAX);
			if (device.use_tx_dma_chain && reset_mdma_chain)
			{
				target_write_u32(target, device.mdma_tx.status, UINT32_MAX);
				target_write_u32(target, device.mdma_rx.status, UINT32_MAX);
			}

			if (device.smpu.secure_ctl != 0) {
				// Clear SMPU stat register
				target_write_u32(target, device.smpu.status, UINT32_MAX);
			}
		}
	}
	_debug_print_registers(target);
}

/**
 * Pre-transfer configuration of the SPI registers required before each
 * SPI transfer.
 *
 * @param target		Pointer to the target device to use
 * @param unit_size		The unit size to use for the transfer
 * @param transfer_size	The number of units to send in the transfer,
 *						may not be the same as the number of bytes
 * @param receive_size	The number of units to receive in the transfer,
 *						may not be the same as the number of bytes
 * @param quad_io		If true, enable Quad IO mode for the SPI transfer
 * @param queue			The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
static ADSP_SPI_RESULT adsp_spi_pre_transfer(struct target *target,
							const ADSP_SPI_UNIT_SIZE unit_size,
							uint16_t transfer_size,
							uint16_t receive_size,
							const bool dual_io,
							struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result;

	/* Both receive and transmit sizes cannot be non-zero */
	if ((receive_size != 0) && (transfer_size != 0))
	{
		LOG_ERROR("Internal SPI driver error. Tx and Rx size are both non-zero.");
		result = ADSP_SPI_RESULT_DRIVER_ERROR;
	}
	/* Both receive and transmit sizes cannot be zero */
	else if ((receive_size == 0) && (transfer_size == 0))
	{
		LOG_ERROR("Internal SPI driver error. Both Tx and Rx sizes are zero.");
		result = ADSP_SPI_RESULT_DRIVER_ERROR;
	}
	else
	{
		uint32_t control_reg = (BITM_SPI_CTL_FMODE | BITM_SPI_CTL_MSTR | (unit_size << BITP_SPI_CTL_SIZE));

		/* SPI needs to be disabled before modifying transfer count */
		result = adsp_spi_enable(target, false, queue);
		if (ADSP_SPI_RESULT_SUCCESS != result)
		{
			LOG_ERROR("Failed to clear SPI Enable.");
			return result;
		}

		if (dual_io)
		{
			control_reg |= ENUM_SPI_CTL_MIO_QUAD;
		}

		/* Configure Control. This disables SPI, too. */
		device.spi.control_value = control_reg;
		adsp_write_or_queue_u32(target, device.spi.control, control_reg, queue);

		/* If this is a receive transfer... */
		if (receive_size)
		{
			/* Set the transfer size in the RX word count register
			*  N.B. this sets the total units to be transferred which is not
			*  necessarily the number of bytes.
			*/
			target_write_u32(target, device.spi.rx_word_count, receive_size);

			/* Configure RX CTL Register*/
			target_write_u32(target, device.spi.rx_control,
				(ENUM_SPI_RXCTL_RDR_NE |
				ENUM_SPI_RXCTL_RWC_EN |
				ENUM_SPI_RXCTL_DISCARD |
				ENUM_SPI_RXCTL_RTI_EN |
				ENUM_SPI_RXCTL_RX_EN));
		}
		/* or it's a transmit transfer. 'else' possible because of above zero/non-zero checks */
		else
		{
			/* Set the transfer size in the TX word count register
			*  N.B. this sets the total units to be transferred which is not
			*  necessarily the number of bytes.
			*/
			adsp_write_or_queue_u32(target, device.spi.tx_word_count, transfer_size, queue);

			/* Tx CTL should be configured after Rx. From HRM it is advised to program
			*  the non-initiating mode first. For SPI Flash it will always have a
			*  command transferred first so Rx is the non-initiating mode.
			*/
			adsp_write_or_queue_u32(target, device.spi.tx_control,
				(BITM_SPI_TXCTL_TDU |
				ENUM_SPI_TXCTL_TDR_EMPTY |
				BITM_SPI_TXCTL_TWCEN |
				BITM_SPI_TXCTL_TTI |
				ENUM_SPI_TXCTL_TX_EN),
				queue);
		}

		result = ADSP_SPI_RESULT_SUCCESS;
	}

	return result;
}

/**
 * Send data over SPI using DMA. The pre-transfer function is expected
 * to be called before this and the SPI should be disabled. As with
 * transmit, the CS line is not controlled within this function either.
 *
 * @param target			Pointer to the target device to use
 * @param tx_buffer			Pointer to the data to send
 * @param tx_byte_count		The number of bytes of data to send
 * @param transfer_units	The number of transfer units to use
 * @param unit_size			The unit size (e.g. half-word) to use for the transfer
 * @param queue				The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
ADSP_SPI_RESULT adsp_spi_dma_send(struct target *target,
					const uint8_t *tx_buffer,
					const uint32_t tx_byte_count,
					uint32_t transfer_units,
					ADSP_SPI_UNIT_SIZE unit_size,
					struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result;

	/* Enable SPI first to ensure it is ready before kicking off DMA transfer */
	result = adsp_spi_enable(target, true, queue);
	if (ADSP_SPI_RESULT_SUCCESS != result)
	{
		LOG_ERROR("Failed to set SPI Enable.");
		return result;
	}

	result = adsp_send_or_queue_memory(target, unit_size, transfer_units, tx_buffer, queue);
	/* Check the above memory write was successful */
	if (result != ADSP_SPI_RESULT_SUCCESS)
	{
		LOG_ERROR("Failed to write Tx buffer to SPI.");
		return result;
	}

	/* Wait for the transfer to complete before we disable the SPI */
	adsp_dma_wait(target, WAIT_LOOP_ITERATIONS, queue);

	/* Wipe the Tx control register ready for the next transfer */
	adsp_write_or_queue_u32(target, device.spi.tx_control, 0uL, queue);

	return result;
}

/**
 * Receive data over SPI using DMA. The pre-transfer function is expected
 * to be called before this and the SPI should be disabled. As with
 * transmit, the CS line is not controlled within this function either.
 *
 * @param target			Pointer to the target device to use
 * @param rx_buffer			Pointer to the data to send
 * @param rx_byte_count		The number of bytes of data to send
 * @param receive_units		The number of transfer units to use
 * @param unit_size			The unit size (e.g. half-word) to use for the transfer
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
ADSP_SPI_RESULT adsp_spi_dma_receive(struct target *target,
					uint8_t *rx_buffer,
					const uint32_t rx_byte_count,
					uint32_t receive_units,
					ADSP_SPI_UNIT_SIZE unit_size)
{
	ADSP_SPI_RESULT result;
	uint32_t dma_cfg_reg;
	int rc;
	const uint32_t data_align = rx_byte_count/receive_units;

	/** Now allocate space for Rx buffer - add DMA cache alignment size
	 *  to ensure we have enough space when correcting for alignment below.
	*/
	struct working_area *rx_buffer_area;
	if (ERROR_OK != target_alloc_working_area(target, rx_byte_count + data_align, &rx_buffer_area))
	{
		LOG_ERROR("No available memory on target for Rx Buffer.");
		result = ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
	}
	else
	{
		/* Adjust start address to ensure DMA cache alignment */
		target_addr_t rx_start_address = rx_buffer_area->address;
		if (rx_start_address % data_align)
		{
			rx_start_address += (data_align - (rx_start_address % data_align));
		}

		/* Set the Configuration for the DMA register */
		dma_cfg_reg = (unit_size << BITP_DMA_CFG_MSIZE)
			| (unit_size << BITP_DMA_CFG_PSIZE)
			| ENUM_DMA_CFG_STOP
			| ENUM_DMA_CFG_WRITE
			| ENUM_DMA_CFG_ADDR1D
			| ENUM_DMA_CFG_EN
			| ENUM_DMA_CFG_SYNC;

		/* Configure DMA regs */
		target_write_u32(target, device.pdma_rx.address_start, rx_start_address);
		target_write_u32(target, device.pdma_rx.x_count, receive_units);
		target_write_u32(target, device.pdma_rx.x_increment, (rx_byte_count/receive_units));

		/* Enable DMA first to ensure it is ready before kicking off SPI transfer */
		target_write_u32(target, device.pdma_rx.config, dma_cfg_reg);

		/* Enable SPI */
		result = adsp_spi_enable(target, true, NULL);
		if (ADSP_SPI_RESULT_SUCCESS != result)
		{
			LOG_ERROR("Failed to set SPI Enable.");
			return result;
		}

		/* Poll the word counter register till it is 0 */
		uint32_t tmp;
		target_read_u32(target, device.spi.rx_word_count, &tmp);

		/* Set result to success, if we timeout then result will be changed */
		result = ADSP_SPI_RESULT_SUCCESS;

		/* Form retry counter based on size of transfer */
		unsigned retry_counter = MAX_RETRY_TIMEOUT * receive_units;
		while (tmp)
		{
			target_read_u32(target, device.spi.rx_word_count, &tmp);
			keep_alive();

			if (--retry_counter == 0)
			{
				LOG_ERROR("Timed out waiting for Rx transfer to finish."
					"Remaining word count is %u.", tmp);
				_debug_print_registers(target);
				result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
				break;
			}
		}

		if (ADSP_SPI_RESULT_SUCCESS == result)
		{
			/** The SPI Block is configured to receive data MSByte first, but the SPI flash outputs
			 *  data byte by byte, LSByte first. That means we need to reverse the byte order
			 *  depending on the configured word size. An easier way would be to configure the SPI
			 *  block to be LSByte first but the only option is LSBit first which is no good.
			*/
			if (ADSP_SPI_UNIT_SIZE_WORD == unit_size)
			{
				rc = target_read_memory(target, rx_start_address, 4, receive_units, rx_buffer);
				buf_bswap32(rx_buffer, rx_buffer, rx_byte_count);
			}
			else if (ADSP_SPI_UNIT_SIZE_HALFWORD == unit_size)
			{
				rc = target_read_memory(target, rx_start_address, 2, receive_units, rx_buffer);
				buf_bswap16(rx_buffer, rx_buffer, rx_byte_count);
			}
			else
			{
				/* Nothing to do when unit size is byte */
				rc = target_read_memory(target, rx_start_address, 1, receive_units, rx_buffer);
			}

			if (ERROR_OK != rc)
			{
				LOG_ERROR("Failed to read Rx buffer from target memory.");
				result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
			}
		}
	}

	/* Wipe the rx control register ready for the next transfer */
	target_write_u32(target, device.spi.rx_control, 0uL);

	/* Always try free the working area, even if allocation wasn't successful */
	target_free_working_area(target, rx_buffer_area);

	return result;
}


/**
 * Send data over SPI using core read/writes. The pre-transfer function is
 * expected to be called before this and the SPI should be disabled. As with
 * transmit, the CS line is not controlled within this function either.
 *
 * @param target			Pointer to the target device to use
 * @param tx_buffer			Pointer to the data to send
 * @param tx_byte_count		The number of bytes of data to send
 * @param transfer_units	The number of transfer units to use
 * @param unit_size			The unit size (e.g. half-word) to use for the transfer
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
ADSP_SPI_RESULT adsp_spi_core_send(struct target *target,
					const uint8_t *tx_buffer,
					const uint32_t tx_byte_count,
					uint32_t transfer_units,
					ADSP_SPI_UNIT_SIZE unit_size)
{
	ADSP_SPI_RESULT result = ADSP_SPI_RESULT_SUCCESS;
	int rc;

	result = adsp_spi_enable(target, true, NULL);
	if (ADSP_SPI_RESULT_SUCCESS != result)
	{
		LOG_ERROR("Failed to set SPI Enable.");
		return result;
	}

	/** The SPI Block is configured to transmit data MSByte first, but the SPI flash reads
	 *  data in byte by byte, LSByte first. That means we need to reverse the byte order
	 *  depending on the configured word size. An easier way would be to configure the SPI
	 *  block to be LSByte first but the only option is LSBit first which is no good.
	*/
	if (ADSP_SPI_UNIT_SIZE_WORD == unit_size)
	{
		/* Flip byte order and send out the data */
		uint32_t write_data;
		for (unsigned i = 0; i < tx_byte_count; i+=4)
		{
			write_data = ((tx_buffer[i] << 24) +
				(tx_buffer[i+1] << 16) +
				(tx_buffer[i+2] << 8) +
				tx_buffer[i+3]);

			/** Write directly to the Tx Fifo with no polling of whether it is empty
			 *  OpenOCD core read/writes are so slow (order of milliseconds), unless
			 *  there is some bad configuration of the driver the FIFO will be cleared
			 *  long before we write to it again.
			*/
			rc = target_write_u32(target, device.spi.tx_fifo, write_data);
			if (rc != ERROR_OK)
			{
				LOG_ERROR("Failed to write to SPI Tx FIFO");
				result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
				break;
			}
		}
	}
	else if (ADSP_SPI_UNIT_SIZE_HALFWORD == unit_size)
	{
		uint16_t write_data;
		for (unsigned i = 0; i < tx_byte_count; i+=2)
		{
			write_data = ((tx_buffer[i] << 8) + tx_buffer[i+1]);
			rc = target_write_u16(target, device.spi.tx_fifo, write_data);
			if (rc != ERROR_OK)
			{
				LOG_ERROR("Failed to write to SPI Tx FIFO");
				result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
				break;
			}
		}
	}
	else
	{
		/* Match endianess of target vs host */
		for (unsigned i = 0; i < tx_byte_count; i++)
		{
			rc = target_write_u8(target, device.spi.tx_fifo, tx_buffer[i]);
			if (rc != ERROR_OK)
			{
				LOG_ERROR("Failed to write to SPI Tx FIFO");
				result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
				break;
			}
		}
	}

	_debug_print_registers(target);

	/* Wipe the Tx control register ready for the next transfer */
	target_write_u32(target, device.spi.tx_control, 0uL);

	return result;
}

/**
 * Receive data over SPI using core read/writes. The pre-transfer function is
 * expected to be called before this and the SPI should be disabled. As with
 * transmit, the CS line is not controlled within this function either.
 *
 * @param target			Pointer to the target device to use
 * @param tx_buffer			Pointer to the data to send
 * @param tx_byte_count		The number of bytes of data to send
 * @param transfer_units	The number of transfer units to use
 * @param unit_size			The unit size (e.g. half-word) to use for the transfer
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
ADSP_SPI_RESULT adsp_spi_core_receive(struct target *target,
					uint8_t *rx_buffer,
					const uint32_t rx_byte_count,
					uint32_t receive_units,
					ADSP_SPI_UNIT_SIZE unit_size)
{
	ADSP_SPI_RESULT result;
	int rc;

	result = adsp_spi_enable(target, true, NULL);
	if (ADSP_SPI_RESULT_SUCCESS != result)
	{
		LOG_ERROR("Failed to set SPI Enable.");
		return result;
	}

	/* Poll the Rx FIFO empty bit in the status register until there is data to be read */
	uint32_t tmp;
	target_read_u32(target, device.spi.status, &tmp);

	/* Set result to success, if we timeout then result will be changed */
	result = ADSP_SPI_RESULT_SUCCESS;

	/* Form retry counter based on size of transfer */
	unsigned retry_counter = MAX_RETRY_TIMEOUT * receive_units;
	while (0 != (tmp & BITM_SPI_STAT_RFE))
	{
		target_read_u32(target, device.spi.status, &tmp);
		keep_alive();

		if (--retry_counter == 0)
		{
			LOG_ERROR("Timed out waiting for Rx transfer to finish."
				"SPI status register: 0x%08X", tmp);
			result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
			break;
		}
	}

	if (ADSP_SPI_RESULT_SUCCESS == result)
	{
		/** The SPI Block is configured to receive data MSByte first, but the SPI flash outputs
		 *  data byte by byte, LSByte first. That means we need to reverse the byte order
		 *  depending on the configured word size. An easier way would be to configure the SPI
		 *  block to be LSByte first but the only option is LSBit first which is no good.
		*/
		if (ADSP_SPI_UNIT_SIZE_WORD == unit_size)
		{
			uint32_t temp;
			for (unsigned i = 0; i < rx_byte_count; i+=4)
			{
				/** Read from the Rx FIFO, we don't bother polling to see if data has been
				 *  received due to the length of time OpenOCD core read/writes take (milliseconds).
				 *  The check on the status register above confirms that at least the first unit of
				 *  data has been received properly, and at least the transfer should be setup properly.
				 *  There is a risk of just reading zeros for any subsequent zeros but the benefit of
				 *  increased speed by not polling the status for every read outweighs the risk.
				*/
				rc = target_read_u32(target, device.spi.rx_fifo, &temp);
				rx_buffer[i] = ((temp >> 24u) & 0xFF);
				rx_buffer[i+1] = ((temp >> 16u) & 0xFF);
				rx_buffer[i+2] = ((temp >> 8u) & 0xFF);
				rx_buffer[i+3] = (temp & 0xFF);
				if (rc != ERROR_OK)
				{
					LOG_ERROR("Failed to read from SPI Rx FIFO");
					result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
					break;
				}
			}
		}
		else if (ADSP_SPI_UNIT_SIZE_HALFWORD == unit_size)
		{
			uint16_t temp;
			for (unsigned i = 0; i < rx_byte_count; i+=2)
			{
				rc = target_read_u16(target, device.spi.rx_fifo, &temp);
				rx_buffer[i] = ((temp >> 8u) & 0xFF);
				rx_buffer[i+1] = (temp & 0xFF);
				if (rc != ERROR_OK)
				{
					LOG_ERROR("Failed to read from SPI Rx FIFO");
					result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
					break;
				}
			}
		}
		else
		{
			/* Match endianess of target vs host */
			for (unsigned i = 0; i < rx_byte_count; i++)
			{
				rc = target_read_u8(target, device.spi.rx_fifo, &rx_buffer[i]);
				if (rc != ERROR_OK)
				{
					LOG_ERROR("Failed to read from SPI Rx FIFO");
					result = ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE;
					break;
				}
			}
		}
	}

	_debug_print_registers(target);

	/* Wipe the Rx control register ready for the next transfer */
	target_write_u32(target, device.spi.rx_control, 0uL);

	return result;
}

/**
 * This function allows data to be sent or read from the SPI device
 * in half-duplex mode only, i.e. the function can either send data
 * or read data but not both at the same time. Pinmux must be configured
 * before calling this function and the Chip Select (CS) line is expected
 * to be controlled outside of this function.
 *
 * @param target		Pointer to the target device to use
 * @param tx_buffer		Pointer to the data to send
 * @param tx_byte_count	The number of bytes of data to send
 * @param rx_buffer		Pointer to the data buffer to read data into
 * @param rx_byte_count	The number of bytes to read from the SPI device
 * @param quad_io		If true, enable Quad IO mode for the SPI transfer
 * @param queue			The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
ADSP_SPI_RESULT adsp_spi_transmit(struct target *target,
					const uint8_t *tx_buffer,
					const uint32_t tx_byte_count,
					uint8_t *rx_buffer,
					const uint32_t rx_byte_count,
					const bool quad_io,
					struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result = ADSP_SPI_RESULT_SUCCESS;
	uint32_t transfer_units;
	uint32_t receive_units;
	ADSP_SPI_UNIT_SIZE unit_size;

	/* NULL checks. If buffer is NULL but byte count is non-zero */
	if ((!tx_buffer) && (tx_byte_count != 0))
	{
		LOG_ERROR("SPI Tx buffer is NULL when Tx count is non-zero.");
		return ADSP_SPI_RESULT_NULL;
	}
	if ((!rx_buffer) && (rx_byte_count != 0))
	{
		LOG_ERROR("SPI Rx buffer is NULL when Rx count is non-zero.");
		return ADSP_SPI_RESULT_NULL;
	}

	/* SPI Driver doesn't currently support simultaneous read and writes */
	if ((tx_byte_count != 0) && (rx_byte_count != 0))
	{
		LOG_ERROR("Both Tx and Rx byte counts are non-zero. "
			"SPI driver doesn't support transmit of Tx and Rx at the same time.");
		return ADSP_SPI_RESULT_CONFIGURATION_INVALID;
	}

	if (tx_byte_count)
	{
		/* Calculate word size and units based on Tx Count */
		if (tx_byte_count % 4 == 0)
		{
			unit_size = ADSP_SPI_UNIT_SIZE_WORD;
			transfer_units = (tx_byte_count / 4);
			receive_units = 0;
		}
		else if (tx_byte_count % 2 == 0)
		{
			unit_size = ADSP_SPI_UNIT_SIZE_HALFWORD;
			transfer_units = (tx_byte_count / 2);
			receive_units = 0;
		}
		else
		{
			unit_size = ADSP_SPI_UNIT_SIZE_BYTE;
			transfer_units = tx_byte_count;
			receive_units = 0;
		}
	}
	else /* This relies on above check of not allowing both Tx and Rx to be non-zero */
	{
		/* Calculate word size and units based on Rx Count */
		if (rx_byte_count % 4 == 0)
		{
			unit_size = ADSP_SPI_UNIT_SIZE_WORD;
			transfer_units = 0;
			receive_units = (rx_byte_count / 4);
		}
		else if (rx_byte_count % 2 == 0)
		{
			unit_size = ADSP_SPI_UNIT_SIZE_HALFWORD;
			transfer_units = 0;
			receive_units = (rx_byte_count / 2);
		}
		else
		{
			unit_size = ADSP_SPI_UNIT_SIZE_BYTE;
			transfer_units = 0;
			receive_units = rx_byte_count;
		}
	}

	/* Check the transfer limit hasn't been exceeded */
	if ((transfer_units > device.max_word_count) || (receive_units > device.max_word_count))
	{
		LOG_ERROR("SPI Tx or Rx word count is too big. Must be smaller than %u. "
			"Tx size is %u and Rx size %u.", device.max_word_count, transfer_units, receive_units);
		return ADSP_SPI_RESULT_CONFIGURATION_INVALID;
	}

	result = adsp_spi_pre_transfer(target, unit_size, transfer_units, receive_units, quad_io, queue);
	if (ADSP_SPI_RESULT_SUCCESS != result)
	{
		LOG_ERROR("Failed in pre-transfer configuration");
		return result;
	}

	/* Tx transfer */
	if (tx_byte_count)
	{
		if (device.use_dma)
		{
			result = adsp_spi_dma_send(target, tx_buffer, tx_byte_count, transfer_units, unit_size, queue);
		}
		else
		{
			result = adsp_spi_core_send(target, tx_buffer, tx_byte_count, transfer_units, unit_size);
		}
	}
	/* Rx transfer */
	else /* This relies on above check of not allowing both Tx and Rx to be non-zero */
	{
		if (device.use_dma)
		{
			result = adsp_spi_dma_receive(target, rx_buffer, rx_byte_count, receive_units, unit_size);
		}
		else
		{
			result = adsp_spi_core_receive(target, rx_buffer, rx_byte_count, receive_units, unit_size);
		}
	}

	return result;
}

/**
 * Populate the supplied buffer with the given address up to the specified address.
 *
 * @param	buffer			Input buffer to populate address with
 * @param	index_ptr		Pointer to index at which to populate buffer, is incremented
 *							depending on the specific number of address bytes.
 * @param	address_bytes	The size of the address in bytes (either 3 or 4)
 * @param	address			The address which will be added to the buffer
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
ADSP_SPI_RESULT adsp_spi_populate_address(uint8_t *buffer,
					uint32_t *index_ptr,
					const uint8_t address_bytes,
					const uint32_t address)
{
	ADSP_SPI_RESULT result = ADSP_SPI_RESULT_SUCCESS;

	/* Address should be always transmitted MSByte first */
	if (address_bytes == 3)
	{
		buffer[(*index_ptr)++] = (uint8_t)(address >> 16u);
		buffer[(*index_ptr)++] = (uint8_t)(address >> 8u);
		buffer[(*index_ptr)++] = (uint8_t)(address >> 0u);
	}
	else if (address_bytes == 4)
	{
		buffer[(*index_ptr)++] = (uint8_t)(address >> 24u);
		buffer[(*index_ptr)++] = (uint8_t)(address >> 16u);
		buffer[(*index_ptr)++] = (uint8_t)(address >> 8u);
		buffer[(*index_ptr)++] = (uint8_t)(address >> 0u);
	}
	else
	{
		LOG_ERROR("Invalid address size. Supported address modes are 3 or 4 bytes.\n"
			"User specified %u bytes", address_bytes);
		result = ADSP_SPI_RESULT_CONFIGURATION_INVALID;
	}

	return result;
}

/**
 * Forms the header (containing instruction/command, address and any dummy bytes) of a
 * SPI Flash operation and transmits it over the SPI bus.
 *
 * @param	target			Pointer to the target device to use
 * @param	instruction		8-bit SPI Flash Instruction to send
 * @param	address			(Optional) The 3/4 byte address associated with the command.
 *							Ignored if address_bytes is '0'
 * @param	address_bytes	Specifies whether the address is 3 or 4 bytes. Set to 0
 *							to send out no address.
 * @param	dummy_bytes		The number of dummy bytes to send out. Set to 0 to send none.
 * @param	quad_io			If true, Quad IO Mode is enabled, otherwise normal mode is used.
 * @param	queue			The DMA queue to add target writes to, or NULL to commit immediately
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
ADSP_SPI_RESULT adsp_spi_header(struct target *target,
					const spi_instruction_t instruction,
					const uint32_t address,
					const uint8_t address_bytes,
					const uint8_t dummy_bytes,
					const bool quad_io,
					struct adsp_dma_queue *queue)
{
	ADSP_SPI_RESULT result = ADSP_SPI_RESULT_SUCCESS;

	/** Calculate SPI instruction header size. Header consists of:
	 * 1. Instruction byte [required]
	 * 2. Address [optional]
	 * 3. Dummy Bytes [optional]
	*/
	unsigned header_bytes =(sizeof(spi_instruction_t) + address_bytes + dummy_bytes);

	/* Allocate space for the Header Buffer - technically not required but avoids complicated
	*  pointer logic + arithmetic
	*/
	uint8_t *header = malloc(header_bytes);
	if (!header)
	{
		LOG_ERROR("No memory for SPI command header buffer");
		result = ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE;
	}
	else
	{
		/* Form the Tx Buffer - start with SPI Flash Instruction */
		uint32_t header_index = 0;
		header[header_index++] = instruction;

		/* If the number of address bytes is non-zero, add this to the buffer as well */
		if (address_bytes != 0)
		{
			result = adsp_spi_populate_address(header, &header_index, address_bytes, address);
		}

		if (ADSP_SPI_RESULT_SUCCESS != result)
		{
			LOG_ERROR("Failed to add SPI flash address to header");
		}
		else
		{
			/* Finally, we have to fill in any dummy bytes */
			if (dummy_bytes != 0)
			{
				/* Add 'dummy_bytes' number of 0s to the Tx buffer */
				memset(&header[header_index], 0u, dummy_bytes);
				header_index += dummy_bytes;
			}

			/* Just a quick sanity check... */
			assert(header_index == header_bytes);

			/* Filled everything up, now ship it! */
			result = adsp_spi_transmit(target, header, header_bytes, NULL, 0, quad_io, queue);
		}
	}

	/* Always do a free, even if the malloc failed it does no harm */
	free(header);
	return result;
}

ADSP_SPI_RESULT adsp_spi_command(struct target *target, struct adsp_spi_flash_cmd *flash_cmd)
{
	ADSP_SPI_RESULT result = ADSP_SPI_RESULT_SUCCESS;

	/* Do all possible checking first before embarking on the transfer */
	if (flash_cmd->device >= ADSP_SPI_DEVICE_UNKNOWN)
	{
		LOG_ERROR("Invalid ADSP_SPI_DEVICE specifier. Got: 0x%X", flash_cmd->device);
		result = ADSP_SPI_RESULT_INVALID_TARGET;
	}
	else if (flash_cmd->dummy_bytes > ADSP_SPI_MAX_DUMMY)
	{
		LOG_ERROR("Number of dummy bytes specified (%u) exceeds the maximum "
			"supported amount (%u).", flash_cmd->dummy_bytes, ADSP_SPI_MAX_DUMMY);
		result = ADSP_SPI_RESULT_CONFIGURATION_INVALID;
	}
	else if ((flash_cmd->data_out_bytes != 0) && (!flash_cmd->data_out_ptr))
	{
		LOG_ERROR("Internal flash driver error. Supplied data out pointer is null.");
					result = ADSP_SPI_RESULT_NULL;
		result = ADSP_SPI_RESULT_NULL;
	}
	else if ((flash_cmd->data_in_bytes != 0) && (!flash_cmd->data_in_ptr))
	{
		LOG_ERROR("Internal flash driver error. Supplied data in pointer is null.");
					result = ADSP_SPI_RESULT_NULL;
		result = ADSP_SPI_RESULT_NULL;
	}
	else
	{
		/* Set the tracker variable to the correct SPI device definition */
		device = adsp_spi_devices[flash_cmd->device];

		/* And then set global variables accordingly */
		spi_max_word_count = device.max_word_count;
		if (device.use_dma)
		{
			const uint32_t align = 4; /* Maximum MSIZE is 4 bytes */
			spi_dma_buffer_minimum_size =
				(align +			// Size to get up to alignment
				sizeof(spi_instruction_t) +	// Header contents: instruction
				ADSP_SPI_MAX_DUMMY +		// Max number of dummy bytes
				sizeof(uint32_t));			// Max address size
		}
		else
		{
			spi_dma_buffer_minimum_size = 0;
		}

		/* On first run, we need to perform one time setup, such as pin mux */
		if (!init_done)
		{
			/* The first time, do an immediate reset of registers, which will include DMA registers
			 * Always reset the MDMA chain registers, because we won't come in here again.
			 */
			adsp_spi_reset_registers(target, true, NULL);
			adsp_spi_init(target);
			init_done = true;
		}
		else
		{
			/* Subsequently, potentially queue the resets. We only need to reset the MDMA regs
			* if we're doing a write, as only writes use them.
			*/
			bool reset_mdma_regs = (flash_cmd->data_out_bytes != 0);
			adsp_spi_reset_registers(target, reset_mdma_regs, flash_cmd->dma_queue);
		}

		/* Assert CS pin */
		result = adsp_spi_cs_control(target, true, flash_cmd->dma_queue);
		if (ADSP_SPI_RESULT_SUCCESS != result)
		{
			LOG_ERROR("Failed to set SPI Chip Select.");
			return result;
		}

		/* First the SPI header needs to be sent out... */
		result = adsp_spi_header(target, flash_cmd->instruction,
					flash_cmd->address, flash_cmd->address_bytes,
					flash_cmd->dummy_bytes, flash_cmd->quad_io,
					flash_cmd->dma_queue);
		if (ADSP_SPI_RESULT_SUCCESS != result)
		{
			LOG_ERROR("Failed to send SPI Command Header.");
		}
		else
		{
			/* Now the data out side of the transfer, if any data is there to be sent */
			if (flash_cmd->data_out_bytes != 0)
			{
				/* No manipulation required here, just send the raw data supplied */
				result = adsp_spi_transmit(target, flash_cmd->data_out_ptr,
							flash_cmd->data_out_bytes, NULL, 0, flash_cmd->quad_io,
							flash_cmd->dma_queue);
				if (ADSP_SPI_RESULT_SUCCESS != result)
				{
					LOG_ERROR("Failed to send SPI Flash Data.");
				}
			}

			/* The Rx Size of the Transfer */
			if ((flash_cmd->data_in_bytes != 0) && (result == ADSP_SPI_RESULT_SUCCESS))
			{
				result = adsp_spi_transmit(target, NULL,
							0, flash_cmd->data_in_ptr,
							flash_cmd->data_in_bytes, flash_cmd->quad_io,
							flash_cmd->dma_queue);
				if (ADSP_SPI_RESULT_SUCCESS != result)
				{
					LOG_ERROR("Failed to receive SPI Flash Data.");
				}
			}
		}
	}

	result = adsp_spi_enable(target, false, flash_cmd->dma_queue);
	if (ADSP_SPI_RESULT_SUCCESS != result)
	{
		LOG_ERROR("Failed to set SPI Enable.");
		return result;
	}

	result = adsp_spi_cs_control(target, false, flash_cmd->dma_queue);
	if (ADSP_SPI_RESULT_SUCCESS != result)
	{
		LOG_ERROR("Failed to deassert SPI Chip Select.");
	}

	return result;
}

int adsp_spi_decode_result(const ADSP_SPI_RESULT result, const char err_message[])
{
	int rc;

	/* NULL pointer check */
	if (!err_message)
	{
		LOG_ERROR("Internal error. NULL pointer passed when pointer to char array expected.");
		return ERROR_FAIL;
	}

	switch (result)
	{
		case ADSP_SPI_RESULT_SUCCESS:
			rc = ERROR_OK;
			break;
		case ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE:
			LOG_ERROR("%s Target device not running or unresponsive for SPI transfer.", err_message);
			rc = ERROR_TARGET_FAILURE;
			break;
		case ADSP_SPI_RESULT_TIMED_OUT:
			LOG_ERROR("%s SPI Transfer timed out.", err_message);
			rc = ERROR_TARGET_TIMEOUT;
			break;
		case ADSP_SPI_RESULT_DRIVER_ERROR:
			LOG_ERROR("%s Internal driver error. There is probably some bug in the SPI driver.", err_message);
			rc = ERROR_FAIL;
			break;
		case ADSP_SPI_RESULT_NULL:
			LOG_ERROR("%s Supplied NULL pointer. This is likely a flash driver level bug.", err_message);
			rc = ERROR_FAIL;
			break;
		case ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE:
			LOG_ERROR("%s Insufficient space available on host.", err_message);
			rc = ERROR_BUF_TOO_SMALL;
			break;
		case ADSP_SPI_RESULT_CONFIGURATION_INVALID:
			LOG_ERROR("%s Bad SPI configuration. This is likely a flash driver level bug.", err_message);
			rc = ERROR_FAIL;
			break;
		case ADSP_SPI_RESULT_INVALID_TARGET:
			LOG_ERROR("%s No valid SPI device details have been configured for this device. "
				"Device Unsupported.", err_message);
			rc = ERROR_FAIL;
			break;
		default:
			LOG_ERROR("%s Unknown SPI Return Code. Got 0x%X", err_message, result);
			rc = ERROR_FAIL;
			break;
	}

	return rc;
}

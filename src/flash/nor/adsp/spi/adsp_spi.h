/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef ADSP_SPI_H
#define ADSP_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "../../imp.h"
#include "adsp_spi_device.h"

/*! Configure this to set the max number of retries for operations before timing out */
#define MAX_RETRY_TIMEOUT		(100u)

/*! Sets the maximum number of dummy bytes that can be sent with a SPI instruction */
#define ADSP_SPI_MAX_DUMMY		(32u)

/*! Interface designed for SPI Flash Chips with an 8-bit instruction/command */
typedef uint8_t spi_instruction_t;

/*! @struct	adsp_dma_channel_data
 *	@brief		Defines the structure about a particular DMA channel in the queue
*/
struct adsp_dma_channel_data {
	uint32_t				num_in_queue;		/*!< The number of descriptors currently queued */
	uint32_t				*descriptors;		/*!< The memory allocated for the descriptors on the host */
	uint32_t				chain_size;			/*!< The number of words of queued descriptors */
	uint32_t				max_chain_size;		/*!< The number of words of descriptors we have allocated space for on the host */
	struct adsp_1d_dma_array_desc	current_regs;		/*!< The current value of the DMA registers after execution of last descriptor */
	uint32_t				*last_config;		/*!< The pointer to the DMA config field in the last descriptor, or initial value */
	uint32_t				first_config;		/*!< The value of the config register we load to start the DMA descriptor chain going */
	uint32_t				saved_first_config; /*!< The value of the config register we loaded to start the DMA descriptor chain going last time */
	uint8_t					*data;				/*!< Pointer to the buffer of data on the host */
	uint32_t				data_size;			/*!< The total number of bytes currently used for PDMA data on the host */
	uint32_t				max_data_size;		/*!< The number of bytes of memory allocated on the host for the PDMA data */
	struct working_area		*descriptors_target;/*!< The descriptor area on the target, or NULL if not transferred yet */
	struct working_area		*data_target;		/*!< The data area on the target, or NULL if not transferred yet */
};

/*! @struct	adsp_dma_queue
 *	@brief		Defines the structure building up information about the DMA queue to subsequently kick off
*/
struct adsp_dma_queue {
	struct adsp_dma_channel_data	mdma_tx;			/*!< Data for the MDMA TX channel */
	struct adsp_dma_channel_data	mdma_rx;			/*!< Data for the MDMA RX channel */
	struct adsp_dma_channel_data	pdma_tx;			/*!< Data for the PDMA TX channel */
	bool							last_was_pdma;		/*!< Whether the last DMA transfer queued was PDMA or MDMA */
	bool							keep_descriptors;	/*!< The next queue will use the same descriptors, so no need to transfer again to target */
};

/*! @struct	adsp_spi_flash_cmd
 *	@brief		Defines the configurable parameters used for a SPI
 *				flash command.
 *
 * Order of transaction goes:
 * 1) Instruction		(out)	[Required]
 * 2) 3/4 Byte Address	(out)	[Not sent if address_bytes=0]
 * 3) Dummy Bytes		(out)	[Not sent if dummy_bytes=0]
 * 3) Data Out			(out)	[Not sent if data_out_bytes=0]
 * 3) Data In			(in)	[Not sent if data_in_bytes=0]
*/
struct adsp_spi_flash_cmd {
	const enum ADSP_SPI_DEVICE	device;			/*!< Sets the SPI driver device definition to use */
	const spi_instruction_t	instruction;	/*!< 8-bit SPI Flash Instruction to send */
	const uint8_t			address_bytes;	/*!< Specifies whether the address is 3 or 4 bytes. Set to 0
												to send out no address. */
	const uint8_t			dummy_bytes;	/*!< The number of dummy bytes to send out. Set to 0 to send none. */
	const uint32_t			address;		/*!< (Optional) The 3/4 byte address associated with the command.
												Ignored if address_bytes is '0' */
	const uint8_t			*data_out_ptr;	/*!< Pointer to the data to send out. Set data_out_bytes to 0 to send
												out no additional data. */
	const uint32_t			data_out_bytes;	/*!< Size (in bytes) of the data out buffer. Set to 0 to send none. */
	uint8_t					*data_in_ptr;	/*!< Pointer to the receive buffer that will be populated with data
												received from SPI Flash device. Set data_in_bytes to 0 to set
												no receive buffer. */
	const uint32_t			data_in_bytes;	/*!< Size (in bytes) of the data in buffer. Set to 0 to receive none. */
	struct adsp_dma_queue	*dma_queue;		/*!< Pointer to queue for MDMA accesses, or NULL */
	const bool				quad_io;		/*!< If true, Quad IO Mode is enabled, otherwise normal mode is used. */
};

/*! @enum	ADSP_SPI_IO_MODE
*	@brief	Lists the possible IO modes of the SPI Block
*/
enum ADSP_SPI_IO_MODE {
	ADSP_SPI_IO_MODE_NORMAL,	/*!< Normal mode. Two lines; one in, one out */
	ADSP_SPI_IO_MODE_QUAD_TX,	/*!< Quad Mode for transfers only. Receives operate in normal mode */
	ADSP_SPI_IO_MODE_QUAD_RX,	/*!< Quad Mode for receives only. Transfers operate in normal mode */
	ADSP_SPI_IO_MODE_QUAD_FULL	/*!< Quad Mode for both transfers and receives */
};

/*! @enum	ADSP_SPI_UNIT_SIZE
*	@brief	Lists the unit sizes available in the SPI Block
*/
enum ADSP_SPI_UNIT_SIZE {
	ADSP_SPI_UNIT_SIZE_BYTE,		/*!< One unit is one byte (8-bits) */
	ADSP_SPI_UNIT_SIZE_HALFWORD,	/*!< One unit is a half-word (16-bits) */
	ADSP_SPI_UNIT_SIZE_WORD,		/*!< One unit is one word (32-bits) */
	ADSP_SPI_UNIT_SIZE_UNKNOWN		/*!< Unknown word size */
};

/*! @enum	ADSP_SPI_CHANNEL
*	@brief	Lists the possible SPI channels
*/
enum ADSP_SPI_CHANNEL {
	ADSP_SPI_CHANNEL_TX,/*!< The TX (transfer) channel */
	ADSP_SPI_CHANNEL_RX	/*!< The RX (receive) channel */
};

/*! @enum	ADSP_SPI_RESULT
*	@brief	Lists the possible returns values for APIs
*/
enum ADSP_SPI_RESULT {
	ADSP_SPI_RESULT_SUCCESS,				/*!< The API completed successfully */
	ADSP_SPI_RESULT_TARGET_NOT_AVAILABLE,	/*!< The target device is unavailable or unresponsive */
	ADSP_SPI_RESULT_TIMED_OUT,				/*!< The API timed out */
	ADSP_SPI_RESULT_DRIVER_ERROR,			/*!< Internal driver error */
	ADSP_SPI_RESULT_NULL,					/*!< Unexpected NULL pointer received */
	ADSP_SPI_RESULT_SPACE_NOT_AVAILABLE,	/*!< Insufficient space available for the operation */
	ADSP_SPI_RESULT_CONFIGURATION_INVALID,	/*!< User configuration error */
	ADSP_SPI_RESULT_INVALID_TARGET,			/*!< No SPI device defined for this target */
};

/* ====== GLOBALS ========== */

/*! Defines the current maximum word count within the SPI driver */
extern uint32_t spi_max_word_count;

/*! Defines the required extra space within the target side buffer for DMA transfers */
extern unsigned int spi_dma_buffer_minimum_size;

/* ====== APIS ========== */

/**
 * Send a SPI Flash command over the SPI interface through the specified target.
 *
 * @param	target			Pointer to the target device to use
 * @param	flash_cmd		Flash command configuration to use
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
enum ADSP_SPI_RESULT adsp_spi_command(struct target *target, struct adsp_spi_flash_cmd *flash_cmd);

/**
 * Utility function to map the SPI driver return codes to OpenOCD return codes
 * along with additional information outputted to error log.
 *
 * @param	result		SPI driver return code to "decode"
 * @param	err_message	Error message to prepend to error log. Should not be NULL.
 *
 * @returns	Mapped OpenOCD return code.
*/
int adsp_spi_decode_result(const enum ADSP_SPI_RESULT result, const char err_message[]);

/**
 * Initial setup for Trigger Routing Unit, to allow DMA descriptor synchronisation.
 * Does NOT need to be called before every transfer.
 *
 * @param target	Pointer to the target device to use
*/
void adsp_init_tru(struct target *target);

/**
 * Allocate space for a new DMA queue, given hints about how large it needs to be,
 * and initialise any fields we need to.
 *
 * @param queue				The DMA queue we're allocating buffers in
 * @param num_bytes			The number of bytes we transmit before running the queue
 * @param page_size			The number of bytes in a page
 *
 * @returns ADSP_SPI_RESULT_SUCCESS on success
*/
enum ADSP_SPI_RESULT adsp_init_dma_queue(struct adsp_dma_queue *queue,
									uint32_t num_bytes,
									uint32_t page_size);

/**
 * Start an MDMA/PDMA queue, by enabling the DMA channels, and wait for it to complete.
 *
 * @param target			Pointer to the target device to use
 * @param queue				The DMA queue to start
*/
enum ADSP_SPI_RESULT adsp_run_queue(struct target *target, struct adsp_dma_queue *queue);

/**
 * Free the various bits of memory allocated in a DMA queue.
 *
 * @param queue				The DMA queue we're freeing
*/
void adsp_destroy_dma_queue(struct adsp_dma_queue *queue);

/**
 * Utility function to return the size in words of a DMA descriptor loaded
 * in response to a preceding config register.
 *
 * @param config	The preceding config register
 *
 * @return			The size in words of the descriptor
*/
uint32_t adsp_config_to_descriptor_size(uint32_t config);

#endif /* ADSP_SPI_H */

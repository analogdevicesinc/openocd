// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
 *	Copyright (C) 2023-2025 Analog Devices, Inc.							*
 ***************************************************************************/

#include "adsp_spi.h"
#include "adsp_spi_device.h"
#include "adsp_regs_common.h"

//#define PRINT_DEBUG_INFO

/* Used to track the current SPI device in use, set in _command */
extern struct adsp_spi_device device;

#ifdef PRINT_DEBUG_INFO
#define UNINIT_REG_VALUE 0x81818181u

static struct dbg_reg_info {
	uint32_t	*device_reg;	/*! Pointer to the device structure member that contains the register address */
	const char *name;			/*! Human-readable register name for tracing */
	uint32_t	last_value;		/*! The value of the register last time we read it */
} reg_info[] = {
	{ &device.spi.control,					"SPI Control",			UNINIT_REG_VALUE },
	{ &device.spi.status,					"SPI Status",			UNINIT_REG_VALUE },
	{ &device.spi.clk,						"SPI Clk",				UNINIT_REG_VALUE },
	{ &device.spi.delay,					"SPI Delay",			UNINIT_REG_VALUE },
	{ &device.spi.rx_control,				"SPI RX Control",		UNINIT_REG_VALUE },
	{ &device.spi.tx_control,				"SPI TX Control",		UNINIT_REG_VALUE },
	{ &device.spi.rx_word_count,			"SPI RX Word Count",	UNINIT_REG_VALUE },
	{ &device.spi.tx_word_count,			"SPI TX Word Count",	UNINIT_REG_VALUE },
	{ &device.spi.slave_select,				"SPI Slave Select",		UNINIT_REG_VALUE },

	{ &device.pdma_tx.descriptor_current,	"PDMA TX Desc Current",	UNINIT_REG_VALUE },
	{ &device.pdma_tx.x_count,				"PDMA TX X Count",		UNINIT_REG_VALUE },
	{ &device.pdma_tx.x_increment,			"PDMA TX X Inc",		UNINIT_REG_VALUE },
	{ &device.pdma_tx.config,				"PDMA TX Config",		UNINIT_REG_VALUE },
	{ &device.pdma_tx.address_start,		"PDMA TX Addr Start",	UNINIT_REG_VALUE },
	{ &device.pdma_tx.status,				"PDMA TX Status",		UNINIT_REG_VALUE },

	{ &device.pdma_rx.descriptor_current,	"PDMA RX Desc Current",	UNINIT_REG_VALUE },
	{ &device.pdma_rx.x_count,				"PDMA RX X Count",		UNINIT_REG_VALUE },
	{ &device.pdma_rx.x_increment,			"PDMA RX X Inc",		UNINIT_REG_VALUE },
	{ &device.pdma_rx.config,				"PDMA RX Config",		UNINIT_REG_VALUE },
	{ &device.pdma_rx.address_start,		"PDMA RX Addr Start",	UNINIT_REG_VALUE },
	{ &device.pdma_rx.status,				"PDMA RX Status",		UNINIT_REG_VALUE },

	{ &device.mdma_tx.descriptor_current,	"MDMA TX Desc Current",	UNINIT_REG_VALUE },
	{ &device.mdma_tx.x_count,				"MDMA TX X Count",		UNINIT_REG_VALUE },
	{ &device.mdma_tx.x_increment,			"MDMA TX X Inc",		UNINIT_REG_VALUE },
	{ &device.mdma_tx.config,				"MDMA TX Config",		UNINIT_REG_VALUE },
	{ &device.mdma_tx.address_start,		"MDMA TX Addr Start",	UNINIT_REG_VALUE },
	{ &device.mdma_tx.status,				"MDMA TX Status",		UNINIT_REG_VALUE },

	{ &device.mdma_rx.descriptor_current,	"MDMA RX Desc Current",	UNINIT_REG_VALUE },
	{ &device.mdma_rx.x_count,				"MDMA RX X Count",		UNINIT_REG_VALUE },
	{ &device.mdma_rx.x_increment,			"MDMA RX X Inc",		UNINIT_REG_VALUE },
	{ &device.mdma_rx.config,				"MDMA RX Config",		UNINIT_REG_VALUE },
	{ &device.mdma_rx.address_start,		"MDMA RX Addr Start",	UNINIT_REG_VALUE },
	{ &device.mdma_rx.status,				"MDMA RX Status",		UNINIT_REG_VALUE },

	{ &device.wait_reg,						"Wait",					UNINIT_REG_VALUE },
};
#endif

/**
 * Utility function useful in debugging the SPI driver. Will print
 * out significant registers if the value has changed since its
 * last invocation (or regardless, if you change print_all to true).
 *
 * @param target		Pointer to the target device to use
*/
void _debug_print_registers(struct target *target)
{
	#ifdef PRINT_DEBUG_INFO
	uint32_t tmp;
	const bool print_all = false;

	for (uint32_t i; i < ARRAY_SIZE(reg_info); i++) {
		target_read_u32(target, *reg_info[i].device_reg, &tmp);
		if (print_all || tmp != reg_info[i].last_value) {
			LOG_INFO("%s Register: 0x%08X", reg_info[i].name, tmp);
			reg_info[i].last_value = tmp;
		}
	}
	#endif
}

#ifdef PRINT_DEBUG_INFO
/**
 * Utility function to determine the register name from the address.
 *
 * @param address	The MMR address
 *
 * @returns			The human-readable name of the register
 */
static char *_dbg_reg_name(uint32_t address)
{
	for (uint32_t i; i < ARRAY_SIZE(reg_info); i++) {
		if (address == *reg_info[i].device_reg)
			return reg_info[i].name;
	}
	return "Unknown";
}
#endif

/**
 * Utility function for printing out an MDMA descriptor that is awaiting
 * being kicked off.
 *
 * @param target						Pointer to the target device to use
 * @param queue							The DMA queue to print out
 * @param i								The MDMA descriptor number to print
 * @param mdma_tx_descriptor_address	The MDMA TX Descriptor Array
 * @param mdma_tx_offset				The offset in words of the start of the TX descriptor in the descriptor array
 * @param mdma_tx_size					The size in words of the MDMA TX Descriptor
 * @param mdma_rx_descriptor_address	The MDMA RX Descriptor Array
 * @param mdma_tx_offset				The offset in words of the start of the RX descriptor in the descriptor array
 * @param mdma_rx_size					The size in words of the MDMA RX Descriptor
 *
*/
void _debug_print_mdma_descriptor(struct target *target, struct adsp_dma_queue *queue, uint32_t i,
					target_addr_t mdma_tx_descriptor_address, uint32_t mdma_tx_offset, uint32_t mdma_tx_size,
					target_addr_t mdma_rx_descriptor_address, uint32_t mdma_rx_offset, uint32_t mdma_rx_size)
{
    #ifdef PRINT_DEBUG_INFO
	uint32_t value;
	struct adsp_1d_dma_array_desc *tx_desc = (struct adsp_1d_dma_array_desc *)(&queue->mdma_tx.descriptors[mdma_tx_offset]);
	struct adsp_1d_dma_array_desc *rx_desc = (struct adsp_1d_dma_array_desc *)(&queue->mdma_rx.descriptors[mdma_rx_offset]);
	target_read_u32(target, tx_desc->address_start, &value);
	LOG_INFO("MDMA TX Descriptor %d (0x%08lx)", i, mdma_tx_descriptor_address + (mdma_tx_offset * sizeof(uint32_t)));
	LOG_INFO("  Address: 0x%08x", tx_desc->address_start);
	LOG_INFO("    Value: 0x%08x", value);
	LOG_INFO("  Config:  0x%08x", tx_desc->config);
	if (mdma_tx_size > 2)
		LOG_INFO("  X Count: %d", tx_desc->x_count);

	if (mdma_tx_size > 3)
		LOG_INFO("  X Inc:   %d", tx_desc->x_increment);

	LOG_INFO("  Triggers: %s%s",  tx_desc->config & ENUM_DMA_CFG_TRGWAIT ? "TWAIT " : "",
									tx_desc->config & BITM_DMA_CFG_TRIG ? "TRIG " : "");
	LOG_INFO("MDMA RX Descriptor %d (0x%08lx)", i, mdma_rx_descriptor_address + (mdma_rx_offset * sizeof(uint32_t)));
	LOG_INFO("  Address: %s (0x%08x)", _dbg_reg_name(rx_desc->address_start), rx_desc->address_start);
	LOG_INFO("  Config:  0x%08x", rx_desc->config);
	if (mdma_rx_size > 2)
		LOG_INFO("  X Count: %d", rx_desc->x_count);

	if (mdma_rx_size > 3)
		LOG_INFO("  X Inc:   %d", rx_desc->x_increment);

	LOG_INFO("  Triggers: %s%s",  rx_desc->config & ENUM_DMA_CFG_TRGWAIT ? "TWAIT " : "",
									rx_desc->config & BITM_DMA_CFG_TRIG ? "TRIG " : "");
    #endif
}

/**
 * Utility function for printing out a PDMA descriptor that is awaiting
 * being kicked off.
 *
 * @param target						Pointer to the target device to use
 * @param queue							The DMA queue to print out
 * @param i								The PDMA descriptor number to print
 * @param pdma_tx_descriptor_address	The PDMA TX Current Descriptor
 * @param pdma_tx_offset				The offset in words of the start of the descriptor in the descriptor array
 * @param pdma_tx_size					The size in words of the PDMA TX Descriptor
 *
*/
void _debug_print_pdma_descriptor(struct target *target, struct adsp_dma_queue *queue, uint32_t i,
					target_addr_t pdma_tx_descriptor_address, uint32_t pdma_tx_offset, uint32_t pdma_tx_size)
{
    #ifdef PRINT_DEBUG_INFO
	uint32_t j;
	struct adsp_1d_dma_array_desc *desc = (struct adsp_1d_dma_array_desc *)(&queue->pdma_tx.descriptors[pdma_tx_offset]);
	LOG_INFO("PDMA TX Descriptor %d (0x%08lx)", i, pdma_tx_descriptor_address + (pdma_tx_offset * sizeof(uint32_t)));
	LOG_INFO("  Address: 0x%08x", desc->address_start);
	for (j = 0; j < (desc->x_count < 8 ? desc->x_count : 8); j++) {
		uint8_t value;
		target_read_u8(target, desc->address_start + j, &value);
		LOG_INFO("    Data:  0x%02x", value);
	}
	LOG_INFO("  Config:  0x%08x", desc->config);
	if (pdma_tx_size > 2)
		LOG_INFO("  X Count: 0x%08x", desc->x_count);

	if (pdma_tx_size > 3)
		LOG_INFO("  X Inc:   0x%08x", desc->x_increment);

	LOG_INFO("  Triggers: %s%s",  desc->config & ENUM_DMA_CFG_TRGWAIT ? "TWAIT " : "",
									desc->config & BITM_DMA_CFG_TRIG ? "TRIG " : "");
    #endif
}

/**
 * Utility function for printing out a DMA queue (MDMA and PDMA) that is awaiting
 * being kicked off.
 *
 * @param target						Pointer to the target device to use
 * @param queue							The DMA queue to print out
 *
*/
void _debug_print_dma_queue(struct target *target,
							struct adsp_dma_queue *queue)
{
	#ifdef PRINT_DEBUG_INFO
	uint32_t i;

	if (queue->mdma_tx.num_in_queue > 0) {
		LOG_INFO("MDMA TX Start");
		LOG_INFO("  Config:  0x%08x", queue->mdma_tx.first_config);
		LOG_INFO("  Triggers: %s%s",  queue->mdma_tx.first_config & ENUM_DMA_CFG_TRGWAIT ? "TWAIT " : "",
									queue->mdma_tx.first_config & BITM_DMA_CFG_TRIG ? "TRIG " : "");
	}
	if (queue->mdma_rx.num_in_queue > 0) {
		LOG_INFO("MDMA RX Start");
		LOG_INFO("  Config:  0x%08x", queue->mdma_rx.first_config);
		LOG_INFO("  Triggers: %s%s",  queue->mdma_rx.first_config & ENUM_DMA_CFG_TRGWAIT ? "TWAIT " : "",
									queue->mdma_rx.first_config & BITM_DMA_CFG_TRIG ? "TRIG " : "");
	}
	uint32_t tx_addr_idx = 0;
	uint32_t tx_desc_size = adsp_config_to_descriptor_size(queue->mdma_tx.first_config);
	uint32_t rx_addr_idx = 0;
	uint32_t rx_desc_size = adsp_config_to_descriptor_size(queue->mdma_rx.first_config);
	for (i = 0; i < queue->mdma_tx.num_in_queue; i++) {
		struct adsp_1d_dma_array_desc *tx_desc = (struct adsp_1d_dma_array_desc *)&queue->mdma_tx.descriptors[tx_addr_idx];
		struct adsp_1d_dma_array_desc *rx_desc = (struct adsp_1d_dma_array_desc *)&queue->mdma_rx.descriptors[rx_addr_idx];
		_debug_print_mdma_descriptor(target, queue, i, queue->mdma_tx.descriptors_target->address, tx_addr_idx, tx_desc_size,
									queue->mdma_rx.descriptors_target->address, rx_addr_idx, rx_desc_size);
		tx_addr_idx += tx_desc_size;
		rx_addr_idx += rx_desc_size;
		tx_desc_size = adsp_config_to_descriptor_size(tx_desc->config);
		rx_desc_size = adsp_config_to_descriptor_size(rx_desc->config);
	}
	LOG_INFO("PDMA TX Start");
	LOG_INFO("  Config:  0x%08x", queue->pdma_tx.first_config);
	LOG_INFO("  Triggers: %s%s", queue->pdma_tx.first_config & ENUM_DMA_CFG_TRGWAIT ? "TWAIT " : "",
								queue->pdma_tx.first_config & BITM_DMA_CFG_TRIG ? "TRIG " : "");
	tx_addr_idx = 0;
	tx_desc_size = adsp_config_to_descriptor_size(queue->pdma_tx.first_config);
	for (i = 0; i < queue->pdma_tx.num_in_queue; i++) {
		struct adsp_1d_dma_array_desc *tx_desc = (struct adsp_1d_dma_array_desc *)&queue->pdma_tx.descriptors[tx_addr_idx];
		_debug_print_pdma_descriptor(target, queue, i, queue->pdma_tx.descriptors_target->address, tx_addr_idx, tx_desc_size);
		tx_addr_idx += tx_desc_size;
		tx_desc_size = adsp_config_to_descriptor_size(tx_desc->config);
	}
	#endif
}

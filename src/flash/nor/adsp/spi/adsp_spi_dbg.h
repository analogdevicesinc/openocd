/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef ADSP_SPI_DBG_H
#define ADSP_SPI_DBG_H

#include <stdint.h>
#include <stdbool.h>
#include "adsp_spi.h"
#include "adsp_spi_device.h"

/**
 * Utility function useful in debugging the SPI driver. Will print
 * out significant SPI registers if the value has changed since its
 * last invocation.
 *
 * @param target		Pointer to the target device to use
*/
void _debug_print_registers(struct target *target);

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
					target_addr_t mdma_rx_descriptor_address, uint32_t mdma_rx_offset, uint32_t mdma_rx_size);

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
					target_addr_t pdma_tx_descriptor_address, uint32_t pdma_tx_offset, uint32_t pdma_tx_size);

/**
 * Utility function for printing out a DMA queue (MDMA and PDMA) that is awaiting
 * being kicked off.
 *
 * @param target						Pointer to the target device to use
 * @param queue							The DMA queue to print out
 *
*/
void _debug_print_dma_queue(struct target *target,
							struct adsp_dma_queue *queue);


#endif /* ADSP_SPI_DBG_H */

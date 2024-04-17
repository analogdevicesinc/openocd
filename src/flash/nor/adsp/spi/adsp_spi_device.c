/* SPDX-License-Identifier: GPL-2.0-or-later */

/****************************************************************************
 *	Copyright (C) 2022-2024 Analog Devices, Inc.							*
 ***************************************************************************/

#include "adsp_spi_device.h"
#include "adsp_2183x_regs.h"
#include "adsp_sc59x_regs.h"
#include <stddef.h>

const struct adsp_spi_device adsp_spi_devices[ADSP_SPI_DEVICE_UNKNOWN] = {
	/* ADSP_SPI_DEVICE_2183X Definition */
	{
		.spi = {
			.base				= REG_2183X_SPI1_CTL,
			.size				= 40,
			.control			= REG_2183X_SPI1_CTL,
			.rx_control			= REG_2183X_SPI1_RXCTL,
			.tx_control			= REG_2183X_SPI1_TXCTL,
			.clk				= REG_2183X_SPI1_CLK,
			.delay				= REG_2183X_SPI1_DLY,
			.slave_select		= REG_2183X_SPI1_SLVSEL,
			.rx_word_count		= REG_2183X_SPI1_RWC,
			.tx_word_count		= REG_2183X_SPI1_TWC,
			.status				= REG_2183X_SPI1_STAT,
			.rx_fifo			= REG_2183X_SPI1_RFIFO,
			.tx_fifo			= REG_2183X_SPI1_TFIFO
		},
		.pdma_tx = {
			.base				= REG_2183X_DMA24_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_2183X_DMA24_ADDRSTART,
			.config				= REG_2183X_DMA24_CFG,
			.x_count			= REG_2183X_DMA24_XCNT,
			.x_increment		= REG_2183X_DMA24_XMOD,
			.status				= REG_2183X_DMA24_STAT,
			.descriptor_current = REG_2183X_DMA24_DSCPTR_CUR
		},
		.pdma_rx = {
			.base				= REG_2183X_DMA25_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_2183X_DMA25_ADDRSTART,
			.config				= REG_2183X_DMA25_CFG,
			.x_count			= REG_2183X_DMA25_XCNT,
			.x_increment		= REG_2183X_DMA25_XMOD,
			.status				= REG_2183X_DMA25_STAT,
			.descriptor_current = REG_2183X_DMA25_DSCPTR_CUR
		},
		.mdma_tx = {
			.base				= REG_2183X_DMA39_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_2183X_DMA39_ADDRSTART,
			.config				= REG_2183X_DMA39_CFG,
			.x_count			= REG_2183X_DMA39_XCNT,
			.x_increment		= REG_2183X_DMA39_XMOD,
			.status				= REG_2183X_DMA39_STAT,
			.descriptor_current = REG_2183X_DMA39_DSCPTR_CUR
		},
		.mdma_rx = {
			.base				= REG_2183X_DMA40_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_2183X_DMA40_ADDRSTART,
			.config				= REG_2183X_DMA40_CFG,
			.x_count			= REG_2183X_DMA40_XCNT,
			.x_increment		= REG_2183X_DMA40_XMOD,
			.status				= REG_2183X_DMA40_STAT,
			.descriptor_current = REG_2183X_DMA40_DSCPTR_CUR
		},
		.smpu = {
			.secure_ctl			= REG_2183X_SMPU5_SECURECTL,
			.status				= REG_2183X_SMPU5_STAT
		},
		.spu = {
			.secure_periph		= REG_2183X_SPU0_SECUREP126,
		},
		.tru = {
			.gctl				= REG_2183X_TRU0_GCTL,
			.pdma_receiver		= REG_2183X_TRU0_RCV_DMA24,
			.mdma_receiver		= REG_2183X_TRU0_RCV_DMA40,
			.pdma_producer		= ID_2183X_TRU0_PROD_DMA24,
			.mdma_producer		= ID_2183X_TRU0_PROD_DMA40
		},
		.chip_select = {
			.data_set_reg		= REG_2183X_PORTA_DATA_SET,
			.data_clear_reg		= REG_2183X_PORTA_DATA_CLR,
			.direction_set_reg	= REG_2183X_PORTA_DIR_SET,
			.pin_number			= 13
		},
		.pinmux = {
			.port_mux_reg		= REG_2183X_PORTA_MUX,
			.port_function_reg	= REG_2183X_PORTA_FER,
			.port_mux_cfg		= ( (uint32_t) ((uint32_t) 1<<20) |
									(uint32_t) ((uint32_t) 1<<22) |
									(uint32_t) ((uint32_t) 1<<24) |
									(uint32_t) ((uint32_t) 1<<28) |
									(uint32_t) ((uint32_t) 1<<30)),
			.port_function_cfg	= ( (uint32_t) ((uint32_t) 1<<10) |
									(uint32_t) ((uint32_t) 1<<11) |
									(uint32_t) ((uint32_t) 1<<12) |
									(uint32_t) ((uint32_t) 1<<14) |
									(uint32_t) ((uint32_t) 1<<15)),
		},
		.wait_reg = REG_2183X_RCU0_MSG_SET,
		.max_word_count = 0xFFFF,
		.use_dma = true,
		.use_tx_dma_chain = true,
		.spi_baud = 0x3uL
	},
	/* ADSP_SPI_DEVICE_SC59X Definition */
	{
		.spi = {
			.base				= REG_SC59X_SPI2_CTL,
			.size				= 40,
			.control			= REG_SC59X_SPI2_CTL,
			.rx_control			= REG_SC59X_SPI2_RXCTL,
			.tx_control			= REG_SC59X_SPI2_TXCTL,
			.clk				= REG_SC59X_SPI2_CLK,
			.delay				= REG_SC59X_SPI2_DLY,
			.slave_select		= REG_SC59X_SPI2_SLVSEL,
			.rx_word_count		= REG_SC59X_SPI2_RWC,
			.tx_word_count		= REG_SC59X_SPI2_TWC,
			.status				= REG_SC59X_SPI2_STAT,
			.rx_fifo			= REG_SC59X_SPI2_RFIFO,
			.tx_fifo			= REG_SC59X_SPI2_TFIFO
		},
		.pdma_tx = {
			.base				= REG_SC59X_DMA26_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA26_ADDRSTART,
			.config				= REG_SC59X_DMA26_CFG,
			.x_count			= REG_SC59X_DMA26_XCNT,
			.x_increment		= REG_SC59X_DMA26_XMOD,
			.status				= REG_SC59X_DMA26_STAT,
			.descriptor_current = REG_SC59X_DMA26_DSCPTR_CUR
		},
		.pdma_rx = {
			.base				= REG_SC59X_DMA27_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA27_ADDRSTART,
			.config				= REG_SC59X_DMA27_CFG,
			.x_count			= REG_SC59X_DMA27_XCNT,
			.x_increment		= REG_SC59X_DMA27_XMOD,
			.status				= REG_SC59X_DMA27_STAT,
			.descriptor_current = REG_SC59X_DMA27_DSCPTR_CUR
		},
		.mdma_tx = {
			.base				= REG_SC59X_DMA39_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA39_ADDRSTART,
			.config				= REG_SC59X_DMA39_CFG,
			.x_count			= REG_SC59X_DMA39_XCNT,
			.x_increment		= REG_SC59X_DMA39_XMOD,
			.status				= REG_SC59X_DMA39_STAT,
			.descriptor_current = REG_SC59X_DMA39_DSCPTR_CUR
		},
		.mdma_rx = {
			.base				= REG_SC59X_DMA40_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA40_ADDRSTART,
			.config				= REG_SC59X_DMA40_CFG,
			.x_count			= REG_SC59X_DMA40_XCNT,
			.x_increment		= REG_SC59X_DMA40_XMOD,
			.status				= REG_SC59X_DMA40_STAT,
			.descriptor_current = REG_SC59X_DMA40_DSCPTR_CUR
		},
		.smpu				= {0}, // SC59X does not require any SMPU configuration
		.spu = {
			.secure_periph		= REG_SC59X_SPU0_SECUREP134,
		},
		.tru = {
			.gctl				= REG_SC59X_TRU0_GCTL,
			.pdma_receiver		= REG_SC59X_TRU0_RCV_DMA26,
			.mdma_receiver		= REG_SC59X_TRU0_RCV_DMA40,
			.pdma_producer		= ID_SC594_TRU0_PROD_DMA26,
			.mdma_producer		= ID_SC594_TRU0_PROD_DMA40
		},
		.chip_select = {
			.data_set_reg		= REG_SC59X_PORTA_DATA_SET,
			.data_clear_reg		= REG_SC59X_PORTA_DATA_CLR,
			.direction_set_reg	= REG_SC59X_PORTA_DIR_SET,
			.pin_number			= 5
		},
		.pinmux = {
			.port_mux_reg		= REG_SC59X_PORTA_MUX,
			.port_function_reg	= REG_SC59X_PORTA_FER,
			.port_mux_cfg		= (0uL),
			.port_function_cfg	= ( (uint32_t) ((uint32_t) 1<<0) |
									(uint32_t) ((uint32_t) 1<<1) |
									(uint32_t) ((uint32_t) 1<<2) |
									(uint32_t) ((uint32_t) 1<<3) |
									(uint32_t) ((uint32_t) 1<<4)),
		},
		.wait_reg = REG_SC59X_RCU0_MSG_SET,
		.max_word_count = 0xFFFF,
		.use_dma = true,
		.use_tx_dma_chain = true,
		.spi_baud = 0x3uL
	},
	/* ADSP_SPI_DEVICE_SC59X_A55 Definition */
	{
		.spi = {
			.base				= REG_SC59X_SPI2_CTL,
			.size				= 40,
			.control			= REG_SC59X_SPI2_CTL,
			.rx_control			= REG_SC59X_SPI2_RXCTL,
			.tx_control			= REG_SC59X_SPI2_TXCTL,
			.clk				= REG_SC59X_SPI2_CLK,
			.delay				= REG_SC59X_SPI2_DLY,
			.slave_select		= REG_SC59X_SPI2_SLVSEL,
			.rx_word_count		= REG_SC59X_SPI2_RWC,
			.tx_word_count		= REG_SC59X_SPI2_TWC,
			.status				= REG_SC59X_SPI2_STAT,
			.rx_fifo			= REG_SC59X_SPI2_RFIFO,
			.tx_fifo			= REG_SC59X_SPI2_TFIFO
		},
		.pdma_tx = {
			.base				= REG_SC59X_DMA26_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA26_ADDRSTART,
			.config				= REG_SC59X_DMA26_CFG,
			.x_count			= REG_SC59X_DMA26_XCNT,
			.x_increment		= REG_SC59X_DMA26_XMOD,
			.status				= REG_SC59X_DMA26_STAT,
			.descriptor_current = REG_SC59X_DMA26_DSCPTR_CUR
		},
		.pdma_rx = {
			.base				= REG_SC59X_DMA27_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA27_ADDRSTART,
			.config				= REG_SC59X_DMA27_CFG,
			.x_count			= REG_SC59X_DMA27_XCNT,
			.x_increment		= REG_SC59X_DMA27_XMOD,
			.status				= REG_SC59X_DMA27_STAT,
			.descriptor_current = REG_SC59X_DMA27_DSCPTR_CUR
		},
		.mdma_tx = {
			.base				= REG_SC59X_DMA39_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA39_ADDRSTART,
			.config				= REG_SC59X_DMA39_CFG,
			.x_count			= REG_SC59X_DMA39_XCNT,
			.x_increment		= REG_SC59X_DMA39_XMOD,
			.status				= REG_SC59X_DMA39_STAT,
			.descriptor_current = REG_SC59X_DMA39_DSCPTR_CUR
		},
		.mdma_rx = {
			.base				= REG_SC59X_DMA40_DSCPTR_NXT,
			.size				= 16,
			.address_start		= REG_SC59X_DMA40_ADDRSTART,
			.config				= REG_SC59X_DMA40_CFG,
			.x_count			= REG_SC59X_DMA40_XCNT,
			.x_increment		= REG_SC59X_DMA40_XMOD,
			.status				= REG_SC59X_DMA40_STAT,
			.descriptor_current = REG_SC59X_DMA40_DSCPTR_CUR
		},
		.smpu				= {0}, // SC59X does not require any SMPU configuration
		.spu = {
			.secure_periph		= REG_SC59X_SPU0_SECUREP135
		},
		.tru = {
			.gctl				= REG_SC59X_TRU0_GCTL,
			.pdma_receiver		= REG_SC59X_TRU0_RCV_DMA26,
			.mdma_receiver		= REG_SC59X_TRU0_RCV_DMA40,
			.pdma_producer		= ID_SC598_TRU0_PROD_DMA26,
			.mdma_producer		= ID_SC598_TRU0_PROD_DMA40
		},
		.chip_select = {
			.data_set_reg		= REG_SC59X_PORTA_DATA_SET,
			.data_clear_reg		= REG_SC59X_PORTA_DATA_CLR,
			.direction_set_reg	= REG_SC59X_PORTA_DIR_SET,
			.pin_number			= 5
		},
		.pinmux = {
			.port_mux_reg		= REG_SC59X_PORTA_MUX,
			.port_function_reg	= REG_SC59X_PORTA_FER,
			.port_mux_cfg		= (0uL),
			.port_function_cfg	= ( (uint32_t) ((uint32_t) 1<<0) |
									(uint32_t) ((uint32_t) 1<<1) |
									(uint32_t) ((uint32_t) 1<<2) |
									(uint32_t) ((uint32_t) 1<<3) |
									(uint32_t) ((uint32_t) 1<<4)),
		},
		.wait_reg = REG_SC59X_RCU0_MSG_SET,
		.max_word_count = 0xFFFF,
		.use_dma = true,
		.use_tx_dma_chain = true,
		.spi_baud = 0x3uL
	}
};

// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef ADSP_2183X_REGS_H
#define ADSP_2183X_REGS_H

/* ============================================================================================================================
	Serial Peripheral Interface
============================================================================================================================ */

/* ============================================================================================================================
		SPI1
============================================================================================================================ */
#define REG_2183X_SPI1_CTL			(0x3102F004uL)		/* SPI1 Control Register */
#define REG_2183X_SPI1_RXCTL		(0x3102F008uL)		/* SPI1 Receive Control Register */
#define REG_2183X_SPI1_TXCTL		(0x3102F00CuL)		/* SPI1 Transmit Control Register */
#define REG_2183X_SPI1_CLK			(0x3102F010uL)		/* SPI1 Clock Rate Register */
#define REG_2183X_SPI1_DLY			(0x3102F014uL)		/* SPI1 Delay Register */
#define REG_2183X_SPI1_SLVSEL		(0x3102F018uL)		/* SPI1 Slave Select Register */
#define REG_2183X_SPI1_RWC			(0x3102F01CuL)		/* SPI1 Received Word Count Register */
#define REG_2183X_SPI1_RWCR			(0x3102F020uL)		/* SPI1 Received Word Count Reload Register */
#define REG_2183X_SPI1_TWC			(0x3102F024uL)		/* SPI1 Transmitted Word Count Register */
#define REG_2183X_SPI1_TWCR			(0x3102F028uL)		/* SPI1 Transmitted Word Count Reload Register */
#define REG_2183X_SPI1_IMSK			(0x3102F030uL)		/* SPI1 Interrupt Mask Register */
#define REG_2183X_SPI1_IMSK_CLR		(0x3102F034uL)		/* SPI1 Interrupt Mask Clear Register */
#define REG_2183X_SPI1_IMSK_SET		(0x3102F038uL)		/* SPI1 Interrupt Mask Set Register */
#define REG_2183X_SPI1_STAT			(0x3102F040uL)		/* SPI1 Status Register */
#define REG_2183X_SPI1_ILAT			(0x3102F044uL)		/* SPI1 Masked Interrupt Condition Register */
#define REG_2183X_SPI1_ILAT_CLR		(0x3102F048uL)		/* SPI1 Masked Interrupt Clear Register */
#define REG_2183X_SPI1_RFIFO		(0x3102F050uL)		/* SPI1 Receive FIFO Data Register */
#define REG_2183X_SPI1_TFIFO		(0x3102F058uL)		/* SPI1 Transmit FIFO Data Register */

/* ============================================================================================================================
	The Direct Memory Access module
============================================================================================================================ */

/* ============================================================================================================================
		DMA24
============================================================================================================================ */
#define REG_2183X_DMA24_DSCPTR_NXT	(0x3102D100uL)		/* DMA24 Pointer to Next Initial Descriptor Register */
#define REG_2183X_DMA24_ADDRSTART	(0x3102D104uL)		/* DMA24 Start Address of Current Buffer Register */
#define REG_2183X_DMA24_CFG			(0x3102D108uL)		/* DMA24 Configuration Register */
#define REG_2183X_DMA24_XCNT		(0x3102D10CuL)		/* DMA24 Inner Loop Count Start Value Register */
#define REG_2183X_DMA24_XMOD		(0x3102D110uL)		/* DMA24 Inner Loop Address Increment Register */
#define REG_2183X_DMA24_YCNT		(0x3102D114uL)		/* DMA24 Outer Loop Count Start Value (2D only) Register */
#define REG_2183X_DMA24_YMOD		(0x3102D118uL)		/* DMA24 Outer Loop Address Increment (2D only) Register */
#define REG_2183X_DMA24_DSCPTR_CUR	(0x3102D124uL)		/* DMA24 Current Descriptor Pointer Register */
#define REG_2183X_DMA24_DSCPTR_PRV	(0x3102D128uL)		/* DMA24 Previous Initial Descriptor Pointer Register */
#define REG_2183X_DMA24_ADDR_CUR	(0x3102D12CuL)		/* DMA24 Current Address Register */
#define REG_2183X_DMA24_STAT		(0x3102D130uL)		/* DMA24 Status Register */
#define REG_2183X_DMA24_XCNT_CUR	(0x3102D134uL)		/* DMA24 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_2183X_DMA24_YCNT_CUR	(0x3102D138uL)		/* DMA24 Current Row Count (2D only) Register */
#define REG_2183X_DMA24_BWLCNT		(0x3102D140uL)		/* DMA24 Bandwidth Limit Count Register */
#define REG_2183X_DMA24_BWLCNT_CUR	(0x3102D144uL)		/* DMA24 Bandwidth Limit Count Current Register */
#define REG_2183X_DMA24_BWMCNT		(0x3102D148uL)		/* DMA24 Bandwidth Monitor Count Register */
#define REG_2183X_DMA24_BWMCNT_CUR	(0x3102D14CuL)		/* DMA24 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
		DMA25
============================================================================================================================ */
#define REG_2183X_DMA25_DSCPTR_NXT	(0x3102D180uL)		/* DMA25 Pointer to Next Initial Descriptor Register */
#define REG_2183X_DMA25_ADDRSTART	(0x3102D184uL)		/* DMA25 Start Address of Current Buffer Register */
#define REG_2183X_DMA25_CFG			(0x3102D188uL)		/* DMA25 Configuration Register */
#define REG_2183X_DMA25_XCNT		(0x3102D18CuL)		/* DMA25 Inner Loop Count Start Value Register */
#define REG_2183X_DMA25_XMOD		(0x3102D190uL)		/* DMA25 Inner Loop Address Increment Register */
#define REG_2183X_DMA25_YCNT		(0x3102D194uL)		/* DMA25 Outer Loop Count Start Value (2D only) Register */
#define REG_2183X_DMA25_YMOD		(0x3102D198uL)		/* DMA25 Outer Loop Address Increment (2D only) Register */
#define REG_2183X_DMA25_DSCPTR_CUR	(0x3102D1A4uL)		/* DMA25 Current Descriptor Pointer Register */
#define REG_2183X_DMA25_DSCPTR_PRV	(0x3102D1A8uL)		/* DMA25 Previous Initial Descriptor Pointer Register */
#define REG_2183X_DMA25_ADDR_CUR	(0x3102D1ACuL)		/* DMA25 Current Address Register */
#define REG_2183X_DMA25_STAT		(0x3102D1B0uL)		/* DMA25 Status Register */
#define REG_2183X_DMA25_XCNT_CUR	(0x3102D1B4uL)		/* DMA25 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_2183X_DMA25_YCNT_CUR	(0x3102D1B8uL)		/* DMA25 Current Row Count (2D only) Register */
#define REG_2183X_DMA25_BWLCNT		(0x3102D1C0uL)		/* DMA25 Bandwidth Limit Count Register */
#define REG_2183X_DMA25_BWLCNT_CUR	(0x3102D1C4uL)		/* DMA25 Bandwidth Limit Count Current Register */
#define REG_2183X_DMA25_BWMCNT		(0x3102D1C8uL)		/* DMA25 Bandwidth Monitor Count Register */
#define REG_2183X_DMA25_BWMCNT_CUR	(0x3102D1CCuL)		/* DMA25 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
		DMA39
============================================================================================================================ */
#define REG_2183X_DMA39_DSCPTR_NXT	(0x3109A000uL)		/* DMA39 Pointer to Next Initial Descriptor Register */
#define REG_2183X_DMA39_ADDRSTART	(0x3109A004uL)		/* DMA39 Start Address of Current Buffer Register */
#define REG_2183X_DMA39_CFG			(0x3109A008uL)		/* DMA39 Configuration Register */
#define REG_2183X_DMA39_XCNT		(0x3109A00CuL)		/* DMA39 Inner Loop Count Start Value Register */
#define REG_2183X_DMA39_XMOD		(0x3109A010uL)		/* DMA39 Inner Loop Address Increment Register */
#define REG_2183X_DMA39_YCNT		(0x3109A014uL)		/* DMA39 Outer Loop Count Start Value (2D only) Register */
#define REG_2183X_DMA39_YMOD		(0x3109A018uL)		/* DMA39 Outer Loop Address Increment (2D only) Register */
#define REG_2183X_DMA39_DSCPTR_CUR	(0x3109A024uL)		/* DMA39 Current Descriptor Pointer Register */
#define REG_2183X_DMA39_DSCPTR_PRV	(0x3109A028uL)		/* DMA39 Previous Initial Descriptor Pointer Register */
#define REG_2183X_DMA39_ADDR_CUR	(0x3109A02CuL)		/* DMA39 Current Address Register */
#define REG_2183X_DMA39_STAT		(0x3109A030uL)		/* DMA39 Status Register */
#define REG_2183X_DMA39_XCNT_CUR	(0x3109A034uL)		/* DMA39 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_2183X_DMA39_YCNT_CUR	(0x3109A038uL)		/* DMA39 Current Row Count (2D only) Register */
#define REG_2183X_DMA39_BWLCNT		(0x3109A040uL)		/* DMA39 Bandwidth Limit Count Register */
#define REG_2183X_DMA39_BWLCNT_CUR	(0x3109A044uL)		/* DMA39 Bandwidth Limit Count Current Register */
#define REG_2183X_DMA39_BWMCNT		(0x3109A048uL)		/* DMA39 Bandwidth Monitor Count Register */
#define REG_2183X_DMA39_BWMCNT_CUR	(0x3109A04CuL)		/* DMA39 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
		DMA40
============================================================================================================================ */
#define REG_2183X_DMA40_DSCPTR_NXT	(0x3109A080uL)		/* DMA40 Pointer to Next Initial Descriptor Register */
#define REG_2183X_DMA40_ADDRSTART	(0x3109A084uL)		/* DMA40 Start Address of Current Buffer Register */
#define REG_2183X_DMA40_CFG			(0x3109A088uL)		/* DMA40 Configuration Register */
#define REG_2183X_DMA40_XCNT		(0x3109A08CuL)		/* DMA40 Inner Loop Count Start Value Register */
#define REG_2183X_DMA40_XMOD		(0x3109A090uL)		/* DMA40 Inner Loop Address Increment Register */
#define REG_2183X_DMA40_YCNT		(0x3109A094uL)		/* DMA40 Outer Loop Count Start Value (2D only) Register */
#define REG_2183X_DMA40_YMOD		(0x3109A098uL)		/* DMA40 Outer Loop Address Increment (2D only) Register */
#define REG_2183X_DMA40_DSCPTR_CUR	(0x3109A0A4uL)		/* DMA40 Current Descriptor Pointer Register */
#define REG_2183X_DMA40_DSCPTR_PRV	(0x3109A0A8uL)		/* DMA40 Previous Initial Descriptor Pointer Register */
#define REG_2183X_DMA40_ADDR_CUR	(0x3109A0ACuL)		/* DMA40 Current Address Register */
#define REG_2183X_DMA40_STAT		(0x3109A0B0uL)		/* DMA40 Status Register */
#define REG_2183X_DMA40_XCNT_CUR	(0x3109A0B4uL)		/* DMA40 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_2183X_DMA40_YCNT_CUR	(0x3109A0B8uL)		/* DMA40 Current Row Count (2D only) Register */
#define REG_2183X_DMA40_BWLCNT		(0x3109A0C0uL)		/* DMA40 Bandwidth Limit Count Register */
#define REG_2183X_DMA40_BWLCNT_CUR	(0x3109A0C4uL)		/* DMA40 Bandwidth Limit Count Current Register */
#define REG_2183X_DMA40_BWMCNT		(0x3109A0C8uL)		/* DMA40 Bandwidth Monitor Count Register */
#define REG_2183X_DMA40_BWMCNT_CUR	(0x3109A0CCuL)		/* DMA40 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
	The General-Purpose Input/Output Port
============================================================================================================================ */

/* ============================================================================================================================
		PORTA
============================================================================================================================ */
#define REG_2183X_PORTA_FER			(0x31004000uL)		/* PORTA Function Enable Register */
#define REG_2183X_PORTA_DATA_SET	(0x31004010uL)		/* PORTA GPIO Data Set Register */
#define REG_2183X_PORTA_DATA_CLR	(0x31004014uL)		/* PORTA GPIO Data Clear Register */
#define REG_2183X_PORTA_DIR_SET		(0x3100401CuL)		/* PORTA GPIO Direction Set Register */
#define REG_2183X_PORTA_MUX			(0x31004030uL)		/* PORTA Multiplexer Control Register */

/* ============================================================================================================================
		The System Memory Protection Unit
============================================================================================================================ */

/* ============================================================================================================================
		SMPU5
============================================================================================================================ */
#define REG_2183X_SMPU5_STAT		(0x31086004uL)		/* SMPU5 SMPU Status Register */
#define REG_2183X_SMPU5_SECURECTL	(0x31086800uL)		/* SMPU5 SMPU Control Secure Accesses Register */

/* ============================================================================================================================
		The System Protection Unit
============================================================================================================================ */

/* ============================================================================================================================
		SPU126
============================================================================================================================ */
#define REG_2183X_SPU0_SECUREP126	(0x3108BBF8uL)		/* SPU0 Secure Peripheral 126 Register for DMA39 */

/* ============================================================================================================================
		TRU
============================================================================================================================ */
#define REG_2183X_TRU0_GCTL			(0x3108A7F4uL)		/* TRU General Control Register */
#define REG_2183X_TRU0_RCV_DMA24	(0x3108A0D8uL)		/* Trigger Receiver Register for DMA24 (SPI1 TX) */
#define REG_2183X_TRU0_RCV_DMA40	(0x3108A088uL)		/* Trigger Receiver Register for DMA40 (MDMA RX) */
#define ID_2183X_TRU0_PROD_DMA24	(84)				/* Trigger Producer Value for DMA24 (SPI1 TX) */
#define ID_2183X_TRU0_PROD_DMA40	(51)				/* Trigger Producer Value for DMA40 (MDMA RX) */

/* ============================================================================================================================
		Miscellaneous
============================================================================================================================ */
#define REG_2183X_RCU0_MSG_SET		(0x3108C070uL)		/* The RCU MSG SET Register */

#endif /* ADSP_2183X_REGS_H */
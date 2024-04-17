// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef ADSP_SC59X_REGS_H
#define ADSP_SC59X_REGS_H

/* ============================================================================================================================
	Serial Peripheral Interface
============================================================================================================================ */

/* ============================================================================================================================
		SPI2
============================================================================================================================ */
#define REG_SC59X_SPI2_CTL			(0x31030004uL)		/* SPI2 Control Register */
#define REG_SC59X_SPI2_RXCTL		(0x31030008uL)		/* SPI2 Receive Control Register */
#define REG_SC59X_SPI2_TXCTL		(0x3103000CuL)		/* SPI2 Transmit Control Register */
#define REG_SC59X_SPI2_CLK			(0x31030010uL)		/* SPI2 Clock Rate Register */
#define REG_SC59X_SPI2_DLY			(0x31030014uL)		/* SPI2 Delay Register */
#define REG_SC59X_SPI2_SLVSEL		(0x31030018uL)		/* SPI2 Slave Select Register */
#define REG_SC59X_SPI2_RWC			(0x3103001CuL)		/* SPI2 Received Word Count Register */
#define REG_SC59X_SPI2_RWCR			(0x31030020uL)		/* SPI2 Received Word Count Reload Register */
#define REG_SC59X_SPI2_TWC			(0x31030024uL)		/* SPI2 Transmitted Word Count Register */
#define REG_SC59X_SPI2_TWCR			(0x31030028uL)		/* SPI2 Transmitted Word Count Reload Register */
#define REG_SC59X_SPI2_IMSK			(0x31030030uL)		/* SPI2 Interrupt Mask Register */
#define REG_SC59X_SPI2_IMSK_CLR		(0x31030034uL)		/* SPI2 Interrupt Mask Clear Register */
#define REG_SC59X_SPI2_IMSK_SET		(0x31030038uL)		/* SPI2 Interrupt Mask Set Register */
#define REG_SC59X_SPI2_STAT			(0x31030040uL)		/* SPI2 Status Register */
#define REG_SC59X_SPI2_ILAT			(0x31030044uL)		/* SPI2 Masked Interrupt Condition Register */
#define REG_SC59X_SPI2_ILAT_CLR		(0x31030048uL)		/* SPI2 Masked Interrupt Clear Register */
#define REG_SC59X_SPI2_RFIFO		(0x31030050uL)		/* SPI2 Receive FIFO Data Register */
#define REG_SC59X_SPI2_TFIFO		(0x31030058uL)		/* SPI2 Transmit FIFO Data Register */
#define REG_SC59X_SPI2_MMRDH		(0x31030060uL)		/* SPI2 Memory Mapped Read Header */
#define REG_SC59X_SPI2_MMTOP		(0x31030064uL)		/* SPI2 SPI Memory Top Address */

/* ============================================================================================================================
	The Direct Memory Access module
============================================================================================================================ */

/* ============================================================================================================================
		DMA26
============================================================================================================================ */
#define REG_SC59X_DMA26_DSCPTR_NXT	(0x3102D200uL)		/* DMA26 Pointer to Next Initial Descriptor Register */
#define REG_SC59X_DMA26_ADDRSTART	(0x3102D204uL)		/* DMA26 Start Address of Current Buffer Register */
#define REG_SC59X_DMA26_CFG			(0x3102D208uL)		/* DMA26 Configuration Register */
#define REG_SC59X_DMA26_XCNT		(0x3102D20CuL)		/* DMA26 Inner Loop Count Start Value Register */
#define REG_SC59X_DMA26_XMOD		(0x3102D210uL)		/* DMA26 Inner Loop Address Increment Register */
#define REG_SC59X_DMA26_YCNT		(0x3102D214uL)		/* DMA26 Outer Loop Count Start Value (2D only) Register */
#define REG_SC59X_DMA26_YMOD		(0x3102D218uL)		/* DMA26 Outer Loop Address Increment (2D only) Register */
#define REG_SC59X_DMA26_DSCPTR_CUR	(0x3102D224uL)		/* DMA26 Current Descriptor Pointer Register */
#define REG_SC59X_DMA26_DSCPTR_PRV	(0x3102D228uL)		/* DMA26 Previous Initial Descriptor Pointer Register */
#define REG_SC59X_DMA26_ADDR_CUR	(0x3102D22CuL)		/* DMA26 Current Address Register */
#define REG_SC59X_DMA26_STAT		(0x3102D230uL)		/* DMA26 Status Register */
#define REG_SC59X_DMA26_XCNT_CUR	(0x3102D234uL)		/* DMA26 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_SC59X_DMA26_YCNT_CUR	(0x3102D238uL)		/* DMA26 Current Row Count (2D only) Register */
#define REG_SC59X_DMA26_BWLCNT		(0x3102D240uL)		/* DMA26 Bandwidth Limit Count Register */
#define REG_SC59X_DMA26_BWLCNT_CUR	(0x3102D244uL)		/* DMA26 Bandwidth Limit Count Current Register */
#define REG_SC59X_DMA26_BWMCNT		(0x3102D248uL)		/* DMA26 Bandwidth Monitor Count Register */
#define REG_SC59X_DMA26_BWMCNT_CUR	(0x3102D24CuL)		/* DMA26 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
		DMA27
============================================================================================================================ */
#define REG_SC59X_DMA27_DSCPTR_NXT	(0x3102D280uL)		/* DMA27 Pointer to Next Initial Descriptor Register */
#define REG_SC59X_DMA27_ADDRSTART	(0x3102D284uL)		/* DMA27 Start Address of Current Buffer Register */
#define REG_SC59X_DMA27_CFG			(0x3102D288uL)		/* DMA27 Configuration Register */
#define REG_SC59X_DMA27_XCNT		(0x3102D28CuL)		/* DMA27 Inner Loop Count Start Value Register */
#define REG_SC59X_DMA27_XMOD		(0x3102D290uL)		/* DMA27 Inner Loop Address Increment Register */
#define REG_SC59X_DMA27_YCNT		(0x3102D294uL)		/* DMA27 Outer Loop Count Start Value (2D only) Register */
#define REG_SC59X_DMA27_YMOD		(0x3102D298uL)		/* DMA27 Outer Loop Address Increment (2D only) Register */
#define REG_SC59X_DMA27_DSCPTR_CUR	(0x3102D2A4uL)		/* DMA27 Current Descriptor Pointer Register */
#define REG_SC59X_DMA27_DSCPTR_PRV	(0x3102D2A8uL)		/* DMA27 Previous Initial Descriptor Pointer Register */
#define REG_SC59X_DMA27_ADDR_CUR	(0x3102D2ACuL)		/* DMA27 Current Address Register */
#define REG_SC59X_DMA27_STAT		(0x3102D2B0uL)		/* DMA27 Status Register */
#define REG_SC59X_DMA27_XCNT_CUR	(0x3102D2B4uL)		/* DMA27 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_SC59X_DMA27_YCNT_CUR	(0x3102D2B8uL)		/* DMA27 Current Row Count (2D only) Register */
#define REG_SC59X_DMA27_BWLCNT		(0x3102D2C0uL)		/* DMA27 Bandwidth Limit Count Register */
#define REG_SC59X_DMA27_BWLCNT_CUR	(0x3102D2C4uL)		/* DMA27 Bandwidth Limit Count Current Register */
#define REG_SC59X_DMA27_BWMCNT		(0x3102D2C8uL)		/* DMA27 Bandwidth Monitor Count Register */
#define REG_SC59X_DMA27_BWMCNT_CUR	(0x3102D2CCuL)		/* DMA27 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
		DMA39
============================================================================================================================ */
#define REG_SC59X_DMA39_DSCPTR_NXT	(0x3109A000uL)		/* DMA39 Pointer to Next Initial Descriptor Register */
#define REG_SC59X_DMA39_ADDRSTART	(0x3109A004uL)		/* DMA39 Start Address of Current Buffer Register */
#define REG_SC59X_DMA39_CFG			(0x3109A008uL)		/* DMA39 Configuration Register */
#define REG_SC59X_DMA39_XCNT		(0x3109A00CuL)		/* DMA39 Inner Loop Count Start Value Register */
#define REG_SC59X_DMA39_XMOD		(0x3109A010uL)		/* DMA39 Inner Loop Address Increment Register */
#define REG_SC59X_DMA39_YCNT		(0x3109A014uL)		/* DMA39 Outer Loop Count Start Value (2D only) Register */
#define REG_SC59X_DMA39_YMOD		(0x3109A018uL)		/* DMA39 Outer Loop Address Increment (2D only) Register */
#define REG_SC59X_DMA39_DSCPTR_CUR	(0x3109A024uL)		/* DMA39 Current Descriptor Pointer Register */
#define REG_SC59X_DMA39_DSCPTR_PRV	(0x3109A028uL)		/* DMA39 Previous Initial Descriptor Pointer Register */
#define REG_SC59X_DMA39_ADDR_CUR	(0x3109A02CuL)		/* DMA39 Current Address Register */
#define REG_SC59X_DMA39_STAT		(0x3109A030uL)		/* DMA39 Status Register */
#define REG_SC59X_DMA39_XCNT_CUR	(0x3109A034uL)		/* DMA39 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_SC59X_DMA39_YCNT_CUR	(0x3109A038uL)		/* DMA39 Current Row Count (2D only) Register */
#define REG_SC59X_DMA39_BWLCNT		(0x3109A040uL)		/* DMA39 Bandwidth Limit Count Register */
#define REG_SC59X_DMA39_BWLCNT_CUR	(0x3109A044uL)		/* DMA39 Bandwidth Limit Count Current Register */
#define REG_SC59X_DMA39_BWMCNT		(0x3109A048uL)		/* DMA39 Bandwidth Monitor Count Register */
#define REG_SC59X_DMA39_BWMCNT_CUR	(0x3109A04CuL)		/* DMA39 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
		DMA40
============================================================================================================================ */
#define REG_SC59X_DMA40_DSCPTR_NXT	(0x3109A080uL)		/* DMA40 Pointer to Next Initial Descriptor Register */
#define REG_SC59X_DMA40_ADDRSTART	(0x3109A084uL)		/* DMA40 Start Address of Current Buffer Register */
#define REG_SC59X_DMA40_CFG			(0x3109A088uL)		/* DMA40 Configuration Register */
#define REG_SC59X_DMA40_XCNT		(0x3109A08CuL)		/* DMA40 Inner Loop Count Start Value Register */
#define REG_SC59X_DMA40_XMOD		(0x3109A090uL)		/* DMA40 Inner Loop Address Increment Register */
#define REG_SC59X_DMA40_YCNT		(0x3109A094uL)		/* DMA40 Outer Loop Count Start Value (2D only) Register */
#define REG_SC59X_DMA40_YMOD		(0x3109A098uL)		/* DMA40 Outer Loop Address Increment (2D only) Register */
#define REG_SC59X_DMA40_DSCPTR_CUR	(0x3109A0A4uL)		/* DMA40 Current Descriptor Pointer Register */
#define REG_SC59X_DMA40_DSCPTR_PRV	(0x3109A0A8uL)		/* DMA40 Previous Initial Descriptor Pointer Register */
#define REG_SC59X_DMA40_ADDR_CUR	(0x3109A0ACuL)		/* DMA40 Current Address Register */
#define REG_SC59X_DMA40_STAT		(0x3109A0B0uL)		/* DMA40 Status Register */
#define REG_SC59X_DMA40_XCNT_CUR	(0x3109A0B4uL)		/* DMA40 Current Count (1D) or Intra-row XCNT (2D) Register */
#define REG_SC59X_DMA40_YCNT_CUR	(0x3109A0B8uL)		/* DMA40 Current Row Count (2D only) Register */
#define REG_SC59X_DMA40_BWLCNT		(0x3109A0C0uL)		/* DMA40 Bandwidth Limit Count Register */
#define REG_SC59X_DMA40_BWLCNT_CUR	(0x3109A0C4uL)		/* DMA40 Bandwidth Limit Count Current Register */
#define REG_SC59X_DMA40_BWMCNT		(0x3109A0C8uL)		/* DMA40 Bandwidth Monitor Count Register */
#define REG_SC59X_DMA40_BWMCNT_CUR	(0x3109A0CCuL)		/* DMA40 Bandwidth Monitor Count Current Register */

/* ============================================================================================================================
	The General-Purpose Input/Output Port
============================================================================================================================ */

/* ============================================================================================================================
		PORTA
============================================================================================================================ */
#define REG_SC59X_PORTA_MUX			(0x31004030uL)		/* PORTA Multiplexer Control Register */
#define REG_SC59X_PORTA_FER			(0x31004000uL)		/* PORTA Function Enable Register */
#define REG_SC59X_PORTA_DIR_SET		(0x3100401CuL)		/* PORTA GPIO Direction Set Register */
#define REG_SC59X_PORTA_DATA_SET	(0x31004010uL)		/* PORTA GPIO Data Set Register */
#define REG_SC59X_PORTA_DATA_CLR	(0x31004014uL)		/* PORTA GPIO Data Clear Register */

/* ============================================================================================================================
		The System Protection Unit
============================================================================================================================ */

/* ============================================================================================================================
		SPU0
============================================================================================================================ */
#define REG_SC59X_SPU0_SECUREP134	(0x3108BC18uL)		/* SPU Secure Peripheral 134 Register for DMA39 */
#define REG_SC59X_SPU0_SECUREP135	(0x3108BC1CuL)		/* SPU Secure Peripheral 135 Register for DMA39 */

/* ============================================================================================================================
		TRU
============================================================================================================================ */
#define REG_SC59X_TRU0_GCTL			(0x3108A7F4uL)		/* TRU General Control Register */
#define REG_SC59X_TRU0_RCV_DMA26	(0x3108A0E8uL)		/* Trigger Receiver Register for DMA26 (SPI2 TX) */
#define REG_SC59X_TRU0_RCV_DMA40	(0x3108A088uL)		/* Trigger Receiver Register for DMA40 (MDMA RX) */

#define ID_SC594_TRU0_PROD_DMA26	(88)				/* Trigger Producer Value for DMA26 (SPI2 TX) */
#define ID_SC594_TRU0_PROD_DMA40	(51)				/* Trigger Producer Value for DMA40 (MDMA RX) */

#define ID_SC598_TRU0_PROD_DMA26	(83)				/* Trigger Producer Value for DMA26 (SPI2 TX) */
#define ID_SC598_TRU0_PROD_DMA40	(46)				/* Trigger Producer Value for DMA40 (MDMA RX) */

/* ============================================================================================================================
		Miscellaneous
============================================================================================================================ */
#define REG_SC59X_RCU0_MSG_SET		(0x3108C070uL)		/* The RCU MSG SET Register */

#endif /* ADSP_SC59X_REGS_H */
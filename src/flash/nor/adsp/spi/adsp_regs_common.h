/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef ADSP_REGS_COMMON_H
#define ADSP_REGS_COMMON_H

/* -------------------------------------------------------------------------------------------------------------------------
		SPI_CTL					Pos/Masks			Description
------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_CTL_MMSE			31				/* Memory-Mapped SPI Enable */
#define BITP_SPI_CTL_MMWEM			30				/* Memory Mapped Write Error Mask */
#define BITP_SPI_CTL_SOSI			22				/* Start on MOSI */
#define BITP_SPI_CTL_MIOM			20				/* Multiple I/O Mode */
#define BITP_SPI_CTL_FMODE			18				/* Fast-Mode Enable */
#define BITP_SPI_CTL_FCWM			16				/* Flow Control Watermark */
#define BITP_SPI_CTL_FCPL			15				/* Flow Control Polarity */
#define BITP_SPI_CTL_FCCH			14				/* Flow Control Channel Selection */
#define BITP_SPI_CTL_FCEN			13				/* Flow Control Enable */
#define BITP_SPI_CTL_LSBF			12				/* Least Significant Bit First */
#define BITP_SPI_CTL_SIZE			9				/* Word Transfer Size */
#define BITP_SPI_CTL_EMISO			8				/* Enable MISO */
#define BITP_SPI_CTL_SELST			7				/* Slave Select Polarity Between Transfers */
#define BITP_SPI_CTL_ASSEL			6				/* Slave Select Pin Control */
#define BITP_SPI_CTL_CPOL			5				/* Clock Polarity */
#define BITP_SPI_CTL_CPHA			4				/* Clock Phase */
#define BITP_SPI_CTL_ODM			3				/* Open Drain Mode */
#define BITP_SPI_CTL_PSSE			2				/* Protected Slave Select Enable */
#define BITP_SPI_CTL_MSTR			1				/* Master/Slave */
#define BITP_SPI_CTL_EN				0				/* Enable */
#define BITM_SPI_CTL_MMSE			(0x80000000uL)	/* Memory-Mapped SPI Enable */
#define BITM_SPI_CTL_MMWEM			(0x40000000uL)	/* Memory Mapped Write Error Mask */
#define BITM_SPI_CTL_SOSI			(0x00400000uL)	/* Start on MOSI */
#define BITM_SPI_CTL_MIOM			(0x00300000uL)	/* Multiple I/O Mode */
#define BITM_SPI_CTL_FMODE			(0x00040000uL)	/* Fast-Mode Enable */
#define BITM_SPI_CTL_FCWM			(0x00030000uL)	/* Flow Control Watermark */
#define BITM_SPI_CTL_FCPL			(0x00008000uL)	/* Flow Control Polarity */
#define BITM_SPI_CTL_FCCH			(0x00004000uL)	/* Flow Control Channel Selection */
#define BITM_SPI_CTL_FCEN			(0x00002000uL)	/* Flow Control Enable */
#define BITM_SPI_CTL_LSBF			(0x00001000uL)	/* Least Significant Bit First */
#define BITM_SPI_CTL_SIZE			(0x00000600uL)	/* Word Transfer Size */
#define BITM_SPI_CTL_EMISO			(0x00000100uL)	/* Enable MISO */
#define BITM_SPI_CTL_SELST			(0x00000080uL)	/* Slave Select Polarity Between Transfers */
#define BITM_SPI_CTL_ASSEL			(0x00000040uL)	/* Slave Select Pin Control */
#define BITM_SPI_CTL_CPOL			(0x00000020uL)	/* Clock Polarity */
#define BITM_SPI_CTL_CPHA			(0x00000010uL)	/* Clock Phase */
#define BITM_SPI_CTL_ODM			(0x00000008uL)	/* Open Drain Mode */
#define BITM_SPI_CTL_PSSE			(0x00000004uL)	/* Protected Slave Select Enable */
#define BITM_SPI_CTL_MSTR			(0x00000002uL)	/* Master/Slave */
#define BITM_SPI_CTL_EN				(0x00000001uL)	/* Enable */
#define ENUM_SPI_CTL_MM_DIS			(0x00000000uL)	/* MMSE: Hardware automated access of memory-mapped SPI memory disabled. */
#define ENUM_SPI_CTL_MM_EN			(0x80000000uL)	/* MMSE: Hardware-automated access of memory-mapped SPI memory enabled. */
#define ENUM_SPI_CTL_WEM_UNMSK		(0x00000000uL)	/* MMWEM: Write error response returned upon write attempts to memory-mapped SPI memory */
#define ENUM_SPI_CTL_WEM_MSK		(0x40000000uL)	/* MMWEM: Write error response masked (not returned) upon write attempts to memory-mapped SPI memory */
#define ENUM_SPI_CTL_STMISO			(0x00000000uL)	/* SOSI: Start on MISO (DIOM) or start on SPI_D3 */
#define ENUM_SPI_CTL_STMOSI			(0x00400000uL)	/* SOSI: Start on MOSI */
#define ENUM_SPI_CTL_MIO_DIS		(0x00000000uL)	/* MIOM: No MIOM (disabled) */
#define ENUM_SPI_CTL_MIO_DUAL		(0x00100000uL)	/* MIOM: DIOM operation */
#define ENUM_SPI_CTL_MIO_QUAD		(0x00200000uL)	/* MIOM: QIOM operation */
#define ENUM_SPI_CTL_FAST_DIS		(0x00000000uL)	/* FMODE: Disable */
#define ENUM_SPI_CTL_FAST_EN		(0x00040000uL)	/* FMODE: Enable */
#define ENUM_SPI_CTL_FIFO0			(0x00000000uL)	/* FCWM: TFIFO empty or RFIFO full */
#define ENUM_SPI_CTL_FIFO1			(0x00010000uL)	/* FCWM: TFIFO 75% or more empty, or RFIFO 75% or more full */
#define ENUM_SPI_CTL_FIFO2			(0x00020000uL)	/* FCWM: TFIFO 50% or more empty, or RFIFO 50% or more full */
#define ENUM_SPI_CTL_FLOW_LO		(0x00000000uL)	/* FCPL: Active-low RDY */
#define ENUM_SPI_CTL_FLOW_HI		(0x00008000uL)	/* FCPL: Active-high RDY */
#define ENUM_SPI_CTL_FLOW_RX		(0x00000000uL)	/* FCCH: Flow control on RX buffer */
#define ENUM_SPI_CTL_FLOW_TX		(0x00004000uL)	/* FCCH: Flow control on TX buffer */
#define ENUM_SPI_CTL_FLOW_DIS		(0x00000000uL)	/* FCEN: Disable */
#define ENUM_SPI_CTL_FLOW_EN		(0x00002000uL)	/* FCEN: Enable */
#define ENUM_SPI_CTL_MSB_FIRST		(0x00000000uL)	/* LSBF: MSB sent/received first (big endian) */
#define ENUM_SPI_CTL_LSB_FIRST		(0x00001000uL)	/* LSBF: LSB sent/received first (little endian) */
#define ENUM_SPI_CTL_SIZE08			(0x00000000uL)	/* SIZE: 8-bit word */
#define ENUM_SPI_CTL_SIZE16			(0x00000200uL)	/* SIZE: 16-bit word */
#define ENUM_SPI_CTL_SIZE32			(0x00000400uL)	/* SIZE: 32-bit word */
#define ENUM_SPI_CTL_MISO_DIS		(0x00000000uL)	/* EMISO: Disable */
#define ENUM_SPI_CTL_MISO_EN		(0x00000100uL)	/* EMISO: Enable */
#define ENUM_SPI_CTL_DEASSRT_SSEL	(0x00000000uL)	/* SELST: Deassert slave select (high) */
#define ENUM_SPI_CTL_ASSRT_SSEL		(0x00000080uL)	/* SELST: Assert slave select (low) */
#define ENUM_SPI_CTL_SW_SSEL		(0x00000000uL)	/* ASSEL: Software slave select control */
#define ENUM_SPI_CTL_HW_SSEL		(0x00000040uL)	/* ASSEL: Hardware slave select control */
#define ENUM_SPI_CTL_SCKHI			(0x00000000uL)	/* CPOL: Active-high SPI CLK */
#define ENUM_SPI_CTL_SCKLO			(0x00000020uL)	/* CPOL: Active-low SPI CLK */
#define ENUM_SPI_CTL_SCKMID			(0x00000000uL)	/* CPHA: SPI CLK toggles from middle */
#define ENUM_SPI_CTL_SCKBEG			(0x00000010uL)	/* CPHA: SPI CLK toggles from start */
#define ENUM_SPI_CTL_ODM_DIS		(0x00000000uL)	/* ODM: Disable */
#define ENUM_SPI_CTL_ODM_EN			(0x00000008uL)	/* ODM: Enable */
#define ENUM_SPI_CTL_PSSE_DIS		(0x00000000uL)	/* PSSE: Disable */
#define ENUM_SPI_CTL_PSSE_EN		(0x00000004uL)	/* PSSE: Enable */
#define ENUM_SPI_CTL_SLAVE			(0x00000000uL)	/* MSTR: Slave */
#define ENUM_SPI_CTL_MASTER			(0x00000002uL)	/* MSTR: Master */
#define ENUM_SPI_CTL_DIS			(0x00000000uL)	/* EN: Disable SPI module */
#define ENUM_SPI_CTL_EN				(0x00000001uL)	/* EN: Enable */

/* -------------------------------------------------------------------------------------------------------------------------
		SPI_RXCTL					Pos/Masks		Description
	------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_RXCTL_RUWM			16				/* Receive FIFO Urgent Watermark */
#define BITP_SPI_RXCTL_RRWM			12				/* Receive FIFO Regular Watermark */
#define BITP_SPI_RXCTL_RDO			8				/* Receive Data Overrun */
#define BITP_SPI_RXCTL_RDR			4				/* Receive Data Request */
#define BITP_SPI_RXCTL_RWCEN		3				/* Receive Word Counter Enable */
#define BITP_SPI_RXCTL_RTI			2				/* Receive Transfer Initiate */
#define BITP_SPI_RXCTL_REN			0				/* Receive Enable */
#define BITM_SPI_RXCTL_RUWM			(0x00070000uL)	/* Receive FIFO Urgent Watermark */
#define BITM_SPI_RXCTL_RRWM			(0x00003000uL)	/* Receive FIFO Regular Watermark */
#define BITM_SPI_RXCTL_RDO			(0x00000100uL)	/* Receive Data Overrun */
#define BITM_SPI_RXCTL_RDR			(0x00000070uL)	/* Receive Data Request */
#define BITM_SPI_RXCTL_RWCEN		(0x00000008uL)	/* Receive Word Counter Enable */
#define BITM_SPI_RXCTL_RTI			(0x00000004uL)	/* Receive Transfer Initiate */
#define BITM_SPI_RXCTL_REN			(0x00000001uL)	/* Receive Enable */
#define ENUM_SPI_RXCTL_UWM_DIS		(0x00000000uL)	/* RUWM: Disabled */
#define ENUM_SPI_RXCTL_UWM_25		(0x00010000uL)	/* RUWM: 25% full RFIFO */
#define ENUM_SPI_RXCTL_UWM_50		(0x00020000uL)	/* RUWM: 50% full RFIFO */
#define ENUM_SPI_RXCTL_UWM_75		(0x00030000uL)	/* RUWM: 75% full RFIFO */
#define ENUM_SPI_RXCTL_UWM_FULL		(0x00040000uL)	/* RUWM: Full RFIFO */
#define ENUM_SPI_RXCTL_RWM_0		(0x00000000uL)	/* RRWM: Empty RFIFO */
#define ENUM_SPI_RXCTL_RWM_25		(0x00001000uL)	/* RRWM: RFIFO less than 25% full */
#define ENUM_SPI_RXCTL_RWM_50		(0x00002000uL)	/* RRWM: RFIFO less than 50% full */
#define ENUM_SPI_RXCTL_RWM_75		(0x00003000uL)	/* RRWM: RFIFO less than 75% full */
#define ENUM_SPI_RXCTL_DISCARD		(0x00000000uL)	/* RDO: Discard incoming data if SPI_RFIFO is full */
#define ENUM_SPI_RXCTL_OVERWRITE	(0x00000100uL)	/* RDO: Overwrite old data if SPI_RFIFO is full */
#define ENUM_SPI_RXCTL_RDR_DIS		(0x00000000uL)	/* RDR: Disabled */
#define ENUM_SPI_RXCTL_RDR_NE		(0x00000010uL)	/* RDR: Not empty RFIFO */
#define ENUM_SPI_RXCTL_RDR_25		(0x00000020uL)	/* RDR: 25% full RFIFO */
#define ENUM_SPI_RXCTL_RDR_50		(0x00000030uL)	/* RDR: 50% full RFIFO */
#define ENUM_SPI_RXCTL_RDR_75		(0x00000040uL)	/* RDR: 75% full RFIFO */
#define ENUM_SPI_RXCTL_RDR_FULL		(0x00000050uL)	/* RDR: Full RFIFO */
#define ENUM_SPI_RXCTL_RWC_DIS		(0x00000000uL)	/* RWCEN: Disable */
#define ENUM_SPI_RXCTL_RWC_EN		(0x00000008uL)	/* RWCEN: Enable */
#define ENUM_SPI_RXCTL_RTI_DIS		(0x00000000uL)	/* RTI: Disable */
#define ENUM_SPI_RXCTL_RTI_EN		(0x00000004uL)	/* RTI: Enable */
#define ENUM_SPI_RXCTL_RX_DIS		(0x00000000uL)	/* REN: Disable */
#define ENUM_SPI_RXCTL_RX_EN		(0x00000001uL)	/* REN: Enable */

/* -------------------------------------------------------------------------------------------------------------------------
		SPI_TXCTL					Pos/Masks		Description
	------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_TXCTL_TUWM			16				/* FIFO Urgent Watermark */
#define BITP_SPI_TXCTL_TRWM			12				/* FIFO Regular Watermark */
#define BITP_SPI_TXCTL_TDU			8				/* Transmit Data Underrun */
#define BITP_SPI_TXCTL_TDR			4				/* Transmit Data Request */
#define BITP_SPI_TXCTL_TWCEN		3				/* Transmit Word Counter Enable */
#define BITP_SPI_TXCTL_TTI			2				/* Transmit Transfer Initiate */
#define BITP_SPI_TXCTL_TEN			0				/* Transmit Enable */
#define BITM_SPI_TXCTL_TUWM			(0x00070000uL)	/* FIFO Urgent Watermark */
#define BITM_SPI_TXCTL_TRWM			(0x00003000uL)	/* FIFO Regular Watermark */
#define BITM_SPI_TXCTL_TDU			(0x00000100uL)	/* Transmit Data Underrun */
#define BITM_SPI_TXCTL_TDR			(0x00000070uL)	/* Transmit Data Request */
#define BITM_SPI_TXCTL_TWCEN		(0x00000008uL)	/* Transmit Word Counter Enable */
#define BITM_SPI_TXCTL_TTI			(0x00000004uL)	/* Transmit Transfer Initiate */
#define BITM_SPI_TXCTL_TEN			(0x00000001uL)	/* Transmit Enable */
#define ENUM_SPI_TXCTL_UWM_DIS		(0x00000000uL)	/* TUWM: Disabled */
#define ENUM_SPI_TXCTL_UWM_25		(0x00010000uL)	/* TUWM: 25% empty TFIFO */
#define ENUM_SPI_TXCTL_UWM_50		(0x00020000uL)	/* TUWM: 50% empty TFIFO */
#define ENUM_SPI_TXCTL_UWM_75		(0x00030000uL)	/* TUWM: 75% empty TFIFO */
#define ENUM_SPI_TXCTL_UWM_EMPTY	(0x00040000uL)	/* TUWM: Empty TFIFO */
#define ENUM_SPI_TXCTL_RWM_FULL		(0x00000000uL)	/* TRWM: Full TFIFO */
#define ENUM_SPI_TXCTL_RWM_25		(0x00001000uL)	/* TRWM: TFIFO less than 25% empty */
#define ENUM_SPI_TXCTL_RWM_50		(0x00002000uL)	/* TRWM: TFIFO less than 50% empty */
#define ENUM_SPI_TXCTL_RWM_75		(0x00003000uL)	/* TRWM: TFIFO less than 75% empty */
#define ENUM_SPI_TXCTL_LASTWD		(0x00000000uL)	/* TDU: Send last word when SPI_TFIFO is empty */
#define ENUM_SPI_TXCTL_ZERO			(0x00000100uL)	/* TDU: Send zeros when SPI_TFIFO is empty */
#define ENUM_SPI_TXCTL_TDR_DIS		(0x00000000uL)	/* TDR: Disabled */
#define ENUM_SPI_TXCTL_TDR_NF		(0x00000010uL)	/* TDR: Not full TFIFO */
#define ENUM_SPI_TXCTL_TDR_25		(0x00000020uL)	/* TDR: 25% empty TFIFO */
#define ENUM_SPI_TXCTL_TDR_50		(0x00000030uL)	/* TDR: 50% empty TFIFO */
#define ENUM_SPI_TXCTL_TDR_75		(0x00000040uL)	/* TDR: 75% empty TFIFO */
#define ENUM_SPI_TXCTL_TDR_EMPTY	(0x00000050uL)	/* TDR: Empty TFIFO */
#define ENUM_SPI_TXCTL_TWC_DIS		(0x00000000uL)	/* TWCEN: Disable */
#define ENUM_SPI_TXCTL_TWC_EN		(0x00000008uL)	/* TWCEN: Enable */
#define ENUM_SPI_TXCTL_TTI_DIS		(0x00000000uL)	/* TTI: Disable */
#define ENUM_SPI_TXCTL_TTI_EN		(0x00000004uL)	/* TTI: Enable */
#define ENUM_SPI_TXCTL_TX_DIS		(0x00000000uL)	/* TEN: Disable */
#define ENUM_SPI_TXCTL_TX_EN		(0x00000001uL)	/* TEN: Enable */

/* -------------------------------------------------------------------------------------------------------------------------
		SPI_SLVSEL					Pos/Masks		Description
	------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_SLVSEL_SSEL7		15				/* Slave Select 7 Output */
#define BITP_SPI_SLVSEL_SSEL6		14				/* Slave Select 6 Output */
#define BITP_SPI_SLVSEL_SSEL5		13				/* Slave Select 5 Output */
#define BITP_SPI_SLVSEL_SSEL4		12				/* Slave Select 4 Output */
#define BITP_SPI_SLVSEL_SSEL3		11				/* Slave Select 3 Output */
#define BITP_SPI_SLVSEL_SSEL2		10				/* Slave Select 2 Output */
#define BITP_SPI_SLVSEL_SSEL1		9				/* Slave Select 1 Output */
#define BITP_SPI_SLVSEL_SSE7		7				/* Slave Select 7 Enable */
#define BITP_SPI_SLVSEL_SSE6		6				/* Slave Select 6 Enable */
#define BITP_SPI_SLVSEL_SSE5		5				/* Slave Select 5 Enable */
#define BITP_SPI_SLVSEL_SSE4		4				/* Slave Select 4 Enable */
#define BITP_SPI_SLVSEL_SSE3		3				/* Slave Select 3 Enable */
#define BITP_SPI_SLVSEL_SSE2		2				/* Slave Select 2 Enable */
#define BITP_SPI_SLVSEL_SSE1		1				/* Slave Select 1 Enable */
#define BITM_SPI_SLVSEL_SSEL7		(0x00008000uL)	/* Slave Select 7 Output */
#define BITM_SPI_SLVSEL_SSEL6		(0x00004000uL)	/* Slave Select 6 Output */
#define BITM_SPI_SLVSEL_SSEL5		(0x00002000uL)	/* Slave Select 5 Output */
#define BITM_SPI_SLVSEL_SSEL4		(0x00001000uL)	/* Slave Select 4 Output */
#define BITM_SPI_SLVSEL_SSEL3		(0x00000800uL)	/* Slave Select 3 Output */
#define BITM_SPI_SLVSEL_SSEL2		(0x00000400uL)	/* Slave Select 2 Output */
#define BITM_SPI_SLVSEL_SSEL1		(0x00000200uL)	/* Slave Select 1 Output */
#define BITM_SPI_SLVSEL_SSE7		(0x00000080uL)	/* Slave Select 7 Enable */
#define BITM_SPI_SLVSEL_SSE6		(0x00000040uL)	/* Slave Select 6 Enable */
#define BITM_SPI_SLVSEL_SSE5		(0x00000020uL)	/* Slave Select 5 Enable */
#define BITM_SPI_SLVSEL_SSE4		(0x00000010uL)	/* Slave Select 4 Enable */
#define BITM_SPI_SLVSEL_SSE3		(0x00000008uL)	/* Slave Select 3 Enable */
#define BITM_SPI_SLVSEL_SSE2		(0x00000004uL)	/* Slave Select 2 Enable */
#define BITM_SPI_SLVSEL_SSE1		(0x00000002uL)	/* Slave Select 1 Enable */
#define ENUM_SPI_SLVSEL_SSEL7_LO	(0x00000000uL)	/* SSEL7: Low */
#define ENUM_SPI_SLVSEL_SSEL7_HI	(0x00008000uL)	/* SSEL7: High */
#define ENUM_SPI_SLVSEL_SSEL6_LO	(0x00000000uL)	/* SSEL6: Low */
#define ENUM_SPI_SLVSEL_SSEL6_HI	(0x00004000uL)	/* SSEL6: High */
#define ENUM_SPI_SLVSEL_SSEL5_LO	(0x00000000uL)	/* SSEL5: Low */
#define ENUM_SPI_SLVSEL_SSEL5_HI	(0x00002000uL)	/* SSEL5: High */
#define ENUM_SPI_SLVSEL_SSEL4_LO	(0x00000000uL)	/* SSEL4: Low */
#define ENUM_SPI_SLVSEL_SSEL4_HI	(0x00001000uL)	/* SSEL4: High */
#define ENUM_SPI_SLVSEL_SSEL3_LO	(0x00000000uL)	/* SSEL3: Low */
#define ENUM_SPI_SLVSEL_SSEL3_HI	(0x00000800uL)	/* SSEL3: High */
#define ENUM_SPI_SLVSEL_SSEL2_LO	(0x00000000uL)	/* SSEL2: Low */
#define ENUM_SPI_SLVSEL_SSEL2_HI	(0x00000400uL)	/* SSEL2: High */
#define ENUM_SPI_SLVSEL_SSEL1_LO	(0x00000000uL)	/* SSEL1: Low */
#define ENUM_SPI_SLVSEL_SSEL1_HI	(0x00000200uL)	/* SSEL1: High */
#define ENUM_SPI_SLVSEL_SSEL7_DIS	(0x00000000uL)	/* SSE7: Disable */
#define ENUM_SPI_SLVSEL_SSEL7_EN	(0x00000080uL)	/* SSE7: Enable */
#define ENUM_SPI_SLVSEL_SSEL6_DIS	(0x00000000uL)	/* SSE6: Disable */
#define ENUM_SPI_SLVSEL_SSEL6_EN	(0x00000040uL)	/* SSE6: Enable */
#define ENUM_SPI_SLVSEL_SSEL5_DIS	(0x00000000uL)	/* SSE5: Disable */
#define ENUM_SPI_SLVSEL_SSEL5_EN	(0x00000020uL)	/* SSE5: Enable */
#define ENUM_SPI_SLVSEL_SSEL4_DIS	(0x00000000uL)	/* SSE4: Disable */
#define ENUM_SPI_SLVSEL_SSEL4_EN	(0x00000010uL)	/* SSE4: Enable */
#define ENUM_SPI_SLVSEL_SSEL3_DIS	(0x00000000uL)	/* SSE3: Disable */
#define ENUM_SPI_SLVSEL_SSEL3_EN	(0x00000008uL)	/* SSE3: Enable */
#define ENUM_SPI_SLVSEL_SSEL2_DIS	(0x00000000uL)	/* SSE2: Disable */
#define ENUM_SPI_SLVSEL_SSEL2_EN	(0x00000004uL)	/* SSE2: Enable */
#define ENUM_SPI_SLVSEL_SSEL1_DIS	(0x00000000uL)	/* SSE1: Disable */
#define ENUM_SPI_SLVSEL_SSEL1_EN	(0x00000002uL)	/* SSE1: Enable */

/* -------------------------------------------------------------------------------------------------------------------------
		SPI_STAT					Pos/Masks		Description
	------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_STAT_MMAE			31				/* Memory Mapped Access Error */
#define BITP_SPI_STAT_MMRE			29				/* Memory Mapped Read Error */
#define BITP_SPI_STAT_MMWE			28				/* Memory Mapped Write Error */
#define BITP_SPI_STAT_TFF			23				/* SPI_TFIFO Full */
#define BITP_SPI_STAT_RFE			22				/* SPI_RFIFO Empty */
#define BITP_SPI_STAT_FCS			20				/* Flow Control Stall Indication */
#define BITP_SPI_STAT_TFS			16				/* SPI_TFIFO Status */
#define BITP_SPI_STAT_RFS			12				/* SPI_RFIFO Status */
#define BITP_SPI_STAT_TF			11				/* Transmit Finish Indication */
#define BITP_SPI_STAT_RF			10				/* Receive Finish Indication */
#define BITP_SPI_STAT_TS			9				/* Transmit Start */
#define BITP_SPI_STAT_RS			8				/* Receive Start */
#define BITP_SPI_STAT_MF			7				/* Mode Fault Indication */
#define BITP_SPI_STAT_TC			6				/* Transmit Collision Indication */
#define BITP_SPI_STAT_TUR			5				/* Transmit Underrun Indication */
#define BITP_SPI_STAT_ROR			4				/* Receive Overrun Indication */
#define BITP_SPI_STAT_TUWM			2				/* Transmit Urgent Watermark Breached */
#define BITP_SPI_STAT_RUWM			1				/* Receive Urgent Watermark Breached */
#define BITP_SPI_STAT_SPIF			0				/* SPI Finished */
#define BITM_SPI_STAT_MMAE			(0x80000000uL)	/* Memory Mapped Access Error */
#define BITM_SPI_STAT_MMRE			(0x20000000uL)	/* Memory Mapped Read Error */
#define BITM_SPI_STAT_MMWE			(0x10000000uL)	/* Memory Mapped Write Error */
#define BITM_SPI_STAT_TFF			(0x00800000uL)	/* SPI_TFIFO Full */
#define BITM_SPI_STAT_RFE			(0x00400000uL)	/* SPI_RFIFO Empty */
#define BITM_SPI_STAT_FCS			(0x00100000uL)	/* Flow Control Stall Indication */
#define BITM_SPI_STAT_TFS			(0x00070000uL)	/* SPI_TFIFO Status */
#define BITM_SPI_STAT_RFS			(0x00007000uL)	/* SPI_RFIFO Status */
#define BITM_SPI_STAT_TF			(0x00000800uL)	/* Transmit Finish Indication */
#define BITM_SPI_STAT_RF			(0x00000400uL)	/* Receive Finish Indication */
#define BITM_SPI_STAT_TS			(0x00000200uL)	/* Transmit Start */
#define BITM_SPI_STAT_RS			(0x00000100uL)	/* Receive Start */
#define BITM_SPI_STAT_MF			(0x00000080uL)	/* Mode Fault Indication */
#define BITM_SPI_STAT_TC			(0x00000040uL)	/* Transmit Collision Indication */
#define BITM_SPI_STAT_TUR			(0x00000020uL)	/* Transmit Underrun Indication */
#define BITM_SPI_STAT_ROR			(0x00000010uL)	/* Receive Overrun Indication */
#define BITM_SPI_STAT_TUWM			(0x00000004uL)	/* Transmit Urgent Watermark Breached */
#define BITM_SPI_STAT_RUWM			(0x00000002uL)	/* Receive Urgent Watermark Breached */
#define BITM_SPI_STAT_SPIF			(0x00000001uL)	/* SPI Finished */
#define ENUM_SPI_STAT_TFIFO_NF		(0x00000000uL)	/* TFF: Not full Tx FIFO */
#define ENUM_SPI_STAT_TFIFO_F		(0x00800000uL)	/* TFF: Full Tx FIFO */
#define ENUM_SPI_STAT_RFIFO_NE		(0x00000000uL)	/* RFE: Rx FIFO not empty */
#define ENUM_SPI_STAT_RFIFO_E		(0x00400000uL)	/* RFE: Rx FIFO empty */
#define ENUM_SPI_STAT_STALL			(0x00000000uL)	/* FCS: No Stall (RDY pin asserted) */
#define ENUM_SPI_STAT_NOSTALL		(0x00100000uL)	/* FCS: Stall (RDY pin deasserted) */
#define ENUM_SPI_STAT_TFIFO_FULL	(0x00000000uL)	/* TFS: Full TFIFO */
#define ENUM_SPI_STAT_TFIFO_25		(0x00010000uL)	/* TFS: 25% empty TFIFO */
#define ENUM_SPI_STAT_TFIFO_50		(0x00020000uL)	/* TFS: 50% empty TFIFO */
#define ENUM_SPI_STAT_TFIFO_75		(0x00030000uL)	/* TFS: 75% empty TFIFO */
#define ENUM_SPI_STAT_TFIFO_EMPTY	(0x00040000uL)	/* TFS: Empty TFIFO */
#define ENUM_SPI_STAT_RFIFO_EMPTY	(0x00000000uL)	/* RFS: Empty RFIFO */
#define ENUM_SPI_STAT_RFIFO_25		(0x00001000uL)	/* RFS: 25% full RFIFO */
#define ENUM_SPI_STAT_RFIFO_50		(0x00002000uL)	/* RFS: 50% full RFIFO */
#define ENUM_SPI_STAT_RFIFO_75		(0x00003000uL)	/* RFS: 75% full RFIFO */
#define ENUM_SPI_STAT_RFIFO_FULL	(0x00004000uL)	/* RFS: Full RFIFO */
#define ENUM_SPI_STAT_TF_LO			(0x00000000uL)	/* TF: No status */
#define ENUM_SPI_STAT_TF_HI			(0x00000800uL)	/* TF: Transmit finish detected */
#define ENUM_SPI_STAT_RF_LO			(0x00000000uL)	/* RF: No status */
#define ENUM_SPI_STAT_RF_HI			(0x00000400uL)	/* RF: Receive finish detected */
#define ENUM_SPI_STAT_TS_LO			(0x00000000uL)	/* TS: No status */
#define ENUM_SPI_STAT_TS_HI			(0x00000200uL)	/* TS: Transmit start detected */
#define ENUM_SPI_STAT_RS_LO			(0x00000000uL)	/* RS: No status */
#define ENUM_SPI_STAT_RS_HI			(0x00000100uL)	/* RS: Receive start detected */
#define ENUM_SPI_STAT_MF_LO			(0x00000000uL)	/* MF: No status */
#define ENUM_SPI_STAT_MF_HI			(0x00000080uL)	/* MF: Mode fault occurred */
#define ENUM_SPI_STAT_TC_LO			(0x00000000uL)	/* TC: No status */
#define ENUM_SPI_STAT_TC_HI			(0x00000040uL)	/* TC: Transmit collision occurred */
#define ENUM_SPI_STAT_TUR_LO		(0x00000000uL)	/* TUR: No status */
#define ENUM_SPI_STAT_TUR_HI		(0x00000020uL)	/* TUR: Transmit underrun occurred */
#define ENUM_SPI_STAT_ROR_LO		(0x00000000uL)	/* ROR: No status */
#define ENUM_SPI_STAT_ROR_HI		(0x00000010uL)	/* ROR: Receive overrun occurred */
#define ENUM_SPI_STAT_TUWM_LO		(0x00000000uL)	/* TUWM: Tx regular watermark reached */
#define ENUM_SPI_STAT_TUWM_HI		(0x00000004uL)	/* TUWM: Tx urgent watermark breached */
#define ENUM_SPI_STAT_RUWM_LO		(0x00000000uL)	/* RUWM: Rx regular watermark reached */
#define ENUM_SPI_STAT_RUWM_HI		(0x00000002uL)	/* RUWM: Rx urgent watermark breached */
#define ENUM_SPI_STAT_SPIF_LO		(0x00000000uL)	/* SPIF: No status */
#define ENUM_SPI_STAT_SPIF_HI		(0x00000001uL)	/* SPIF: Completed single word transfer */

/* ============================================================================================================================
		The Direct Memory Access module
============================================================================================================================ */

/* -------------------------------------------------------------------------------------------------------------------------
		DMA_CFG						Pos/Masks		Description
------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_CFG_PDRF			28				/* Peripheral Data Request Forward */
#define BITP_DMA_CFG_TWOD			26				/* Two Dimension Addressing Enable */
#define BITP_DMA_CFG_DESCIDCPY		25				/* Descriptor ID Copy Control */
#define BITP_DMA_CFG_TOVEN			24				/* Trigger Overrun Error Enable */
#define BITP_DMA_CFG_TRIG			22				/* Generate Outgoing Trigger */
#define BITP_DMA_CFG_INT			20				/* Generate Interrupt Request */
#define BITP_DMA_CFG_NDSIZE			16				/* Next Descriptor Set Size */
#define BITP_DMA_CFG_TWAIT			15				/* Wait for Trigger */
#define BITP_DMA_CFG_FLOW			12				/* Next Operation */
#define BITP_DMA_CFG_MSIZE			8				/* Memory Transfer Word Size */
#define BITP_DMA_CFG_PSIZE			4				/* Peripheral Transfer Word Size */
#define BITP_DMA_CFG_CADDR			3				/* Use Current Address */
#define BITP_DMA_CFG_SYNC			2				/* Synchronize Work Unit Transitions */
#define BITP_DMA_CFG_WNR			1				/* Write/Read Channel Direction */
#define BITP_DMA_CFG_EN				0				/* DMA Channel Enable */
#define BITM_DMA_CFG_PDRF			(0x10000000uL)	/* Peripheral Data Request Forward */
#define BITM_DMA_CFG_TWOD			(0x04000000uL)	/* Two Dimension Addressing Enable */
#define BITM_DMA_CFG_DESCIDCPY		(0x02000000uL)	/* Descriptor ID Copy Control */
#define BITM_DMA_CFG_TOVEN			(0x01000000uL)	/* Trigger Overrun Error Enable */
#define BITM_DMA_CFG_TRIG			(0x00C00000uL)	/* Generate Outgoing Trigger */
#define BITM_DMA_CFG_INT			(0x00300000uL)	/* Generate Interrupt Request */
#define BITM_DMA_CFG_NDSIZE			(0x00070000uL)	/* Next Descriptor Set Size */
#define BITM_DMA_CFG_TWAIT			(0x00008000uL)	/* Wait for Trigger */
#define BITM_DMA_CFG_FLOW			(0x00007000uL)	/* Next Operation */
#define BITM_DMA_CFG_MSIZE			(0x00000700uL)	/* Memory Transfer Word Size */
#define BITM_DMA_CFG_PSIZE			(0x00000070uL)	/* Peripheral Transfer Word Size */
#define BITM_DMA_CFG_CADDR			(0x00000008uL)	/* Use Current Address */
#define BITM_DMA_CFG_SYNC			(0x00000004uL)	/* Synchronize Work Unit Transitions */
#define BITM_DMA_CFG_WNR			(0x00000002uL)	/* Write/Read Channel Direction */
#define BITM_DMA_CFG_EN				(0x00000001uL)	/* DMA Channel Enable */
#define ENUM_DMA_CFG_PDAT_NOTFWD	(0x00000000uL)	/* PDRF: Peripheral Data Request Not Forwarded */
#define ENUM_DMA_CFG_PDAT_FWD		(0x10000000uL)	/* PDRF: Peripheral Data Request Forwarded */
#define ENUM_DMA_CFG_ADDR1D			(0x00000000uL)	/* TWOD: One-Dimensional Addressing */
#define ENUM_DMA_CFG_ADDR2D			(0x04000000uL)	/* TWOD: Two-Dimensional Addressing */
#define ENUM_DMA_CFG_NO_COPY		(0x00000000uL)	/* DESCIDCPY: Never Copy */
#define ENUM_DMA_CFG_COPY			(0x02000000uL)	/* DESCIDCPY: Copy on Work Unit Complete */
#define ENUM_DMA_CFG_TOV_DIS		(0x00000000uL)	/* TOVEN: Ignore Trigger Overrun */
#define ENUM_DMA_CFG_TOV_EN			(0x01000000uL)	/* TOVEN: Error on Trigger Overrun */
#define ENUM_DMA_CFG_NO_TRIG		(0x00000000uL)	/* TRIG: Never Assert Trigger */
#define ENUM_DMA_CFG_XCNT_TRIG		(0x00400000uL)	/* TRIG: Trigger When XCNTCUR Reaches 0 */
#define ENUM_DMA_CFG_YCNT_TRIG		(0x00800000uL)	/* TRIG: Trigger When YCNTCUR Reaches 0 */
#define ENUM_DMA_CFG_NO_INT			(0x00000000uL)	/* INT: Never Assert Interrupt */
#define ENUM_DMA_CFG_XCNT_INT		(0x00100000uL)	/* INT: Interrupt When X Count Expires */
#define ENUM_DMA_CFG_YCNT_INT		(0x00200000uL)	/* INT: Interrupt When Y Count Expires */
#define ENUM_DMA_CFG_PERIPH_INT		(0x00300000uL)	/* INT: Peripheral Interrupt request */
#define ENUM_DMA_CFG_FETCH01		(0x00000000uL)	/* NDSIZE: Fetch One Descriptor Element */
#define ENUM_DMA_CFG_FETCH02		(0x00010000uL)	/* NDSIZE: Fetch Two Descriptor Elements */
#define ENUM_DMA_CFG_FETCH03		(0x00020000uL)	/* NDSIZE: Fetch Three Descriptor Elements */
#define ENUM_DMA_CFG_FETCH04		(0x00030000uL)	/* NDSIZE: Fetch Four Descriptor Elements */
#define ENUM_DMA_CFG_FETCH05		(0x00040000uL)	/* NDSIZE: Fetch Five Descriptor Elements */
#define ENUM_DMA_CFG_FETCH06		(0x00050000uL)	/* NDSIZE: Fetch Six Descriptor Elements */
#define ENUM_DMA_CFG_FETCH07		(0x00060000uL)	/* NDSIZE: Fetch Seven Descriptor Elements */
#define ENUM_DMA_CFG_NO_TRGWAIT		(0x00000000uL)	/* TWAIT: Begin Work Unit Automatically (No Wait) */
#define ENUM_DMA_CFG_TRGWAIT		(0x00008000uL)	/* TWAIT: Wait for Trigger (Halt before Work Unit) */
#define ENUM_DMA_CFG_STOP			(0x00000000uL)	/* FLOW: STOP. */
#define ENUM_DMA_CFG_AUTO			(0x00001000uL)	/* FLOW: AUTO. */
#define ENUM_DMA_CFG_DSCLIST		(0x00004000uL)	/* FLOW: DSCL. */
#define ENUM_DMA_CFG_DSCARRAY		(0x00005000uL)	/* FLOW: DSCA. */
#define ENUM_DMA_CFG_DODLIST		(0x00006000uL)	/* FLOW: Descriptor On-Demand List. */
#define ENUM_DMA_CFG_DODARRAY		(0x00007000uL)	/* FLOW: Descriptor On Demand Array. */
#define ENUM_DMA_CFG_MSIZE04		(0x00000200uL)	/* MSIZE: 4 Bytes */
#define ENUM_DMA_CFG_MSIZE08		(0x00000300uL)	/* MSIZE: 8 Bytes */
#define ENUM_DMA_CFG_MSIZE16		(0x00000400uL)	/* MSIZE: 16 Bytes */
#define ENUM_DMA_CFG_MSIZE32		(0x00000500uL)	/* MSIZE: 32 Bytes */
#define ENUM_DMA_CFG_PSIZE04		(0x00000020uL)	/* PSIZE: 4 Bytes */
#define ENUM_DMA_CFG_PSIZE08		(0x00000030uL)	/* PSIZE: 8 Bytes */
#define ENUM_DMA_CFG_LD_STARTADDR	(0x00000000uL)	/* CADDR: Load Starting Address */
#define ENUM_DMA_CFG_LD_CURADDR		(0x00000008uL)	/* CADDR: Use Current Address */
#define ENUM_DMA_CFG_NO_SYNC		(0x00000000uL)	/* SYNC: No Synchronization */
#define ENUM_DMA_CFG_SYNC			(0x00000004uL)	/* SYNC: Synchronize Channel */
#define ENUM_DMA_CFG_READ			(0x00000000uL)	/* WNR: Transmit (Read from memory) */
#define ENUM_DMA_CFG_WRITE			(0x00000002uL)	/* WNR: Receive (Write to memory) */
#define ENUM_DMA_CFG_DIS			(0x00000000uL)	/* EN: Disable */
#define ENUM_DMA_CFG_EN				(0x00000001uL)	/* EN: Enable */

/* -------------------------------------------------------------------------------------------------------------------------
		DMA_STAT						Pos/Masks		Description
------------------------------------------------------------------------------------------------------------------------- */
#define BITM_DMA_STAT_RUN				(0x00000700uL)	/* DMA Channel Run Status */

/* -------------------------------------------------------------------------------------------------------------------------
		SMPU_SECURECTL					Pos/Masks		Description
------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SMPU_SECURECTL_LOCK		31				/*  Lock Bit */
#define BITP_SMPU_SECURECTL_WSECDIS		11				/*  Secure Write Transaction Disable */
#define BITP_SMPU_SECURECTL_WNSEN		10				/*  Non-secure Write Transaction Enable */
#define BITP_SMPU_SECURECTL_RSECDIS		9				/*  Secure Read Transaction Disable */
#define BITP_SMPU_SECURECTL_RNSEN		8				/*  Non-secure Read Transaction Enable */
#define BITP_SMPU_SECURECTL_RLOCK		3				/*  Secure Region Registers Lock Bit */
#define BITP_SMPU_SECURECTL_SINTEN		2				/*  Security Violation Interrupt Enable */
#define BITP_SMPU_SECURECTL_SBETYPE		1				/*  Security Violation Bus Error Type */
#define BITP_SMPU_SECURECTL_SBEDIS		0				/*  Security Violation Bus Error Disable */
#define BITM_SMPU_SECURECTL_LOCK		(0x80000000uL)	/* Lock Bit */
#define BITM_SMPU_SECURECTL_WSECDIS		(0x00000800uL)	/* Secure Write Transaction Disable */
#define BITM_SMPU_SECURECTL_WNSEN		(0x00000400uL)	/* Non-secure Write Transaction Enable */
#define BITM_SMPU_SECURECTL_RSECDIS		(0x00000200uL)	/* Secure Read Transaction Disable */
#define BITM_SMPU_SECURECTL_RNSEN		(0x00000100uL)	/* Non-secure Read Transaction Enable */
#define BITM_SMPU_SECURECTL_RLOCK		(0x00000008uL)	/* Secure Region Registers Lock Bit */
#define BITM_SMPU_SECURECTL_SINTEN		(0x00000004uL)	/* Security Violation Interrupt Enable */
#define BITM_SMPU_SECURECTL_SBETYPE		(0x00000002uL)	/* Security Violation Bus Error Type */
#define BITM_SMPU_SECURECTL_SBEDIS		(0x00000001uL)	/* Security Violation Bus Error Disable */
#define ENUM_SMPU_SECURECTL_WSECEN		(0x00000000uL)	/*  WSECDIS: Enable secure write transactions */
#define ENUM_SMPU_SECURECTL_WSECDIS		(0x00000800uL)	/*  WSECDIS: Disable secure write transactions */
#define ENUM_SMPU_SECURECTL_WNSDIS		(0x00000000uL)	/*  WNSEN: Disable non-secure writes */
#define ENUM_SMPU_SECURECTL_WNSEN		(0x00000400uL)	/*  WNSEN: Enable non-secure writes */
#define ENUM_SMPU_SECURECTL_RSECEN		(0x00000000uL)	/*  RSECDIS: Enable secure read transactions */
#define ENUM_SMPU_SECURECTL_RSECDIS		(0x00000200uL)	/*  RSECDIS: Disable secure read transactions */
#define ENUM_SMPU_SECURECTL_RNSDIS		(0x00000000uL)	/*  RNSEN: Disable non-secure read transactions */
#define ENUM_SMPU_SECURECTL_RNSEN		(0x00000100uL)	/*  RNSEN: Enable non-secure read transactions */
#define ENUM_SMPU_SECURECTL_RLOCKDIS	(0x00000000uL)	/*  RLOCK: Disable write-protection on secure region registers */
#define ENUM_SMPU_SECURECTL_RLOCKEN		(0x00000008uL)	/*  RLOCK: Enable write-protection on secure region registers */
#define ENUM_SMPU_SECURECTL_SINTDIS		(0x00000000uL)	/*  SINTEN: Disable security settings violation interrupt */
#define ENUM_SMPU_SECURECTL_SINTEN		(0x00000004uL)	/*  SINTEN: Enable security settings violation interrupt */
#define ENUM_SMPU_SECURECTL_DECERR		(0x00000000uL)	/*  SBETYPE: Decode error */
#define ENUM_SMPU_SECURECTL_SLVERR		(0x00000002uL)	/*  SBETYPE: Completer error */
#define ENUM_SMPU_SECURECTL_SBEEN		(0x00000000uL)	/*  SBEDIS: Enable bus error */
#define ENUM_SMPU_SECURECTL_SBEDIS		(0x00000001uL)	/*  SBEDIS: Disable bus error */

/* -------------------------------------------------------------------------------------------------------------------------
		SPU_SECUREP						Pos/Masks		Description
------------------------------------------------------------------------------------------------------------------------- */
#define BITM_SPU_SECUREP_MSEC			(0x00000002ul)	/* Requester Secure Enable */

/* -------------------------------------------------------------------------------------------------------------------------
		TRU								Pos/Masks		Description
------------------------------------------------------------------------------------------------------------------------- */
#define BITM_TRU_GCTL_EN				(0x00000001ul)	/* Trigger Routing Unit Enable */

#endif /* ADSP_REGS_COMMON_H */

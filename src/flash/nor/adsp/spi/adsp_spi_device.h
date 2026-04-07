/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022-2024 Analog Devices, Inc.                          *
 ***************************************************************************/

#ifndef ADSP_SPI_DEVICE_H
#define ADSP_SPI_DEVICE_H

#include <stdint.h>
#include <stdbool.h>

/*!
 * @enum	ADSP_SPI_DEVICE
 * @brief	Lists the available targets with SPI device definitions
 */
enum ADSP_SPI_DEVICE {
	ADSP_SPI_DEVICE_2183X,		/*!< ADSP-2183x/ADSP-SC83x family*/
	ADSP_SPI_DEVICE_SC59X,		/*!< ADSP-SC594 family */
	ADSP_SPI_DEVICE_SC59X_A55,	/*!< ADSP-SC598 family */
	ADSP_SPI_DEVICE_UNKNOWN		/*!< Unknown device*/
};

/**! @struct	adsp_spi_regs
 *	 @brief		Defines the SPI registers required in the driver
*/
struct adsp_spi_regs {
	uint32_t	base;			/*!< Base Address (lowest) of the registers, used for resetting in bulk */
	uint32_t	size;			/*!< Size in bytes of the register block, used for resetting in bulk */
	uint32_t	control;		/*!< Control Register Address */
	uint32_t	control_value;	/*!< Control Register Tracked Value */
	uint32_t	rx_control;		/*!< Receive Control Register Address */
	uint32_t	tx_control;		/*!< Transmit Control Register Address */
	uint32_t	clk;			/*!< Clock Register Address */
	uint32_t	delay;			/*!< Delay Register Address */
	uint32_t	slave_select;	/*!< Slave Select Register Address */
	uint32_t	rx_word_count;	/*!< Receive Word Count Register Address */
	uint32_t	tx_word_count;	/*!< Transmit Word Count Register Address */
	uint32_t	status;			/*!< Status Register Address */
	uint32_t	rx_fifo;		/*!< Receive FIFO Data Register Address */
	uint32_t	tx_fifo;		/*!< Transmit FIFO Data Register Address */
};

/**! @struct	adsp_dma_regs
 *	 @brief		Defines the DMA registers required in the driver
*/
struct adsp_dma_regs {
	uint32_t	base;				/*!< Base Address (lowest) of the registers, used for resetting in bulk */
	uint32_t	size;				/*!< Size in bytes of the register block, used for resetting in bulk */
	uint32_t	address_start;		/*!< Receive Control Register Address */
	uint32_t	config;				/*!< Transmit Control Register Address */
	uint32_t	x_count;			/*!< X Count Register Address */
	uint32_t	x_increment;		/*!< X Increment Register Address */
	uint32_t	status;				/*!< Status Register Address */
	uint32_t	descriptor_current;	/*!< Current Descriptor Pointer Register */
};

/**! @struct	adsp_1d_dma_array_desc
 *	 @brief		Defines the 1D DMA descriptor for descriptor-array based mode.
*/
struct adsp_1d_dma_array_desc {
	uint32_t	address_start;		/*!< Receive Control Register Address */
	uint32_t	config;				/*!< Config Register Address */
	uint32_t	x_count;			/*!< X Count Register Address */
	int32_t		x_increment;		/*!< X Increment Register Address */
} __attribute__((packed, aligned(4)));

/**! @struct	adsp_smpu_regs
 *	 @brief		Defines the SMPU registers required in the driver
*/
struct adsp_smpu_regs {
	uint32_t	secure_ctl;			/*!< Secure Control Register Address for L2 protection */
	uint32_t	status;				/*!< Status Register Address for L2 protection */
};

/**! @struct	adsp_spu_regs
 *	 @brief		Defines the SPU registers required in the driver
*/
struct adsp_spu_regs {
	uint32_t	secure_periph;		/*!< Secure Peripheral Address for MDMA */
};

/**! @struct	adsp_tru_regs
 *	 @brief		Defines the TRU registers required in the driver
*/
struct adsp_tru_regs {
	uint32_t	gctl;				/*!< Control register for TRU */
	uint32_t	mdma_receiver;		/*!< Receiver register for MDMA */
	uint32_t	pdma_receiver;		/*!< Receiver register for PDMA */
	uint32_t	mdma_producer;		/*!< Producer value from MDMA */
	uint32_t	pdma_producer;		/*!< Producer value from PDMA */
};

/**! @struct	adsp_cs_config
 *	 @brief		Defines the Chip Select registers and config required in the driver
*/
struct adsp_cs_config {
	uint32_t	data_set_reg;		/*!< Data Set for CS Pin Port Register Address */
	uint32_t	data_clear_reg;		/*!< Data Clear for CS Pin Port Register Address */
	uint32_t	direction_set_reg;	/*!< Direction Set for CS Pin Port Register Address */
	uint32_t	pin_number;			/*!< CS Pin Number within Port */
};

/**! @struct	adsp_pinmux_config
 *	 @brief		Defines the Pin Mux registers and config required in the driver
*/
struct adsp_pinmux_config {
	uint32_t	port_mux_reg;		/*!< Port Multiplexer Register Address */
	uint32_t	port_function_reg;	/*!< Port Function Enable Register Address */
	uint32_t	port_mux_cfg;		/*!< Value to write to port multiplexer register */
	uint32_t	port_function_cfg;	/*!< Value to write to port function register */
};

/**! @struct	adsp_spi_device
 *	 @brief		Defines chip specific registers and configuration used in the driver
*/
struct adsp_spi_device {
	struct adsp_spi_regs		spi;			/*!< SPI Registers associated with this device */
	struct adsp_dma_regs		pdma_tx;		/*!< DMA Registers associated with this device used for PDMA Tx Channel */
	struct adsp_dma_regs		pdma_rx;		/*!< DMA Registers associated with this device used for PDMA Rx Channel */
	struct adsp_dma_regs		mdma_tx;		/*!< DMA Registers associated with this device used for MDMA Tx Channel */
	struct adsp_dma_regs		mdma_rx;		/*!< DMA Registers associated with this device used for MDMA Rx Channel */
	struct adsp_smpu_regs		smpu;			/*!< SMPU Registers associated with this device. Zero init if SMPU config not required. */
	struct adsp_spu_regs		spu;			/*!< SPU Registers associated with this device. Zero init if SPU config not required. */
	struct adsp_tru_regs		tru;			/*!< TRU Registers associated with this device. Zero init if TRU config not required. */
	struct adsp_cs_config		chip_select;	/*!< CS Registers and configuration associated with this device */
	struct adsp_pinmux_config	pinmux;			/*!< Pin Mux Registers and configuration associated with this device */
	uint32_t					wait_reg;		/*!< MMR we can write zero to with no effect. Used for wait loops */
	uint32_t					max_word_count;	/*!< Maximum SPI word count available for this device */
	uint16_t					spi_baud;		/*!< SPI Baud rate to set in CLK register. SPI speed is calculated using this */
	bool						use_dma;		/*!< If true, DMA-driven SPI will be used; otherwise core-driven SPI is used */
	bool						use_tx_dma_chain; /*!< If true, chain DMA descriptors together for the TX channel */
};

extern const struct adsp_spi_device adsp_spi_devices[ADSP_SPI_DEVICE_UNKNOWN];

#endif /* ADSP_SPI_DEVICE_H */

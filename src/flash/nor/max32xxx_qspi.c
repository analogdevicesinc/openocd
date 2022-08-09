/***************************************************************************
 *   Copyright (C) 2016 - 2019 by Andreas Bolsch                           *
 *   andreas.bolsch@mni.thm.de                                             *
 *                                                                         *
 *   Copyright (C) 2010 by Antonio Borneo                                  *
 *   borneo.antonio@gmail.com                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/image.h>
#include "spi.h"
#include "sfdp.h"
#include "helper/binarybuffer.h"

#define SPIXFC_BASE								0x40027000
#define SPIXFC_CFG								(SPIXFC_BASE | 0x00)
#define SPIXFC_SS_POL							(SPIXFC_BASE | 0x04)
#define SPIXFC_GEN_CTRL							(SPIXFC_BASE | 0x08)
#define SPIXFC_FIFO_CTRL						(SPIXFC_BASE | 0x0C)

#define SPIXFC_CONFIG_PAGE_SIZE_POS				6
#define SPIXFC_CONFIG_PAGE_SIZE					(0x3 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_4_BYTES			(0x0 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_8_BYTES			(0x1 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_16_BYTES		(0x2 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_32_BYTES		(0x3 << SPIXFC_CONFIG_PAGE_SIZE_POS)

#define SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS		8
#define SPIXFC_FIFO_CTRL_TX_FIFO_CNT			(0x1FUL << SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS)

#define SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS		24
#define SPIXFC_FIFO_CTRL_RX_FIFO_CNT			(0x3FUL << SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS)

#define SPIXFC_FIFO_TX							0x400BC000
#define SPIXFC_FIFO_RX							0x400BC004

#define SPIXFC_FIFO_DEPTH						(16)
#define SPIXFC_HEADER_TX						0x1
#define SPIXFC_HEADER_RX						0x2
#define SPIXFC_HEADER_BIT						(0x0 << 2)
#define SPIXFC_HEADER_BYTE						(0x1 << 2)
#define SPIXFC_HEADER_PAGE						(0x2 << 2)
#define SPIXFC_HEADER_SIZE_POS					4
#define SPIXFC_HEADER_WIDTH_POS					9
#define SPIXFC_HEADER_SS_DEASS					(0x1 << 13)
#define SPIXFC_HEADER_NULL						0xF000	/* 16-bit filler magic word indicating this isn't a header */

#define SPIXF_BASE								0x40026000
#define SPIXF_CFG								(SPIXF_BASE | 0x00)
#define SPIXF_FETCH_CTRL						(SPIXF_BASE | 0x04)
#define SPIXF_MODE_CTRL							(SPIXF_BASE | 0x08)
#define SPIXF_MODE_DATA							(SPIXF_BASE | 0x0C)
#define SPIXF_SCLK_FB_CTRL						(SPIXF_BASE | 0x10)
#define SPIXF_IO_CTRL							(SPIXF_BASE | 0x1C)
#define SPIXF_MEMSECCN							(SPIXF_BASE | 0x20)
#define SPIXF_BUS_IDLE							(SPIXF_BASE | 0x24)

#define SPIXF_MEMSECCN_ENC_ENABLE 				(0x1)
#define SPIXF_MEMSECCN_AUTH_DISABLE				(0x2)

#define SPI_ICC_BASE							0x4002F000
#define SPI_ICC_CTRL							(SPI_ICC_BASE | 0x100)

#define SPI_ICC_CTRL_EN_POS						0
#define SPI_ICC_CTRL_EN							(0x1UL << SPI_ICC_CTRL_EN_POS)

#define GCR_BASE 								0x40000000
#define GCR_SCON 								(GCR_BASE | 0x00)

/* Set the number of system clocks per low/high period of the SPI clock */
#define SPI_CLOCK_PERIOD						2

/* Address boundary for writes */
#define SPI_WRITE_BOUNDARY						256

/* Set this to 1 to enable dual (1-2-2) reads if available from SFDP */
#ifndef SPI_DUAL_MODE
#define SPI_DUAL_MODE 							1
#endif

#define OPTIONS_128								0x01 /* Perform 128 bit flash writes */
#define OPTIONS_ENC								0x02 /* Encrypt the flash contents */
#define OPTIONS_AUTH							0x04 /* Authenticate the flash contents */
#define OPTIONS_COUNT							0x08 /* Add counter values to authentication */
#define OPTIONS_INTER							0x10 /* Interleave the authentication and count values*/
#define OPTIONS_RELATIVE_XOR					0x20 /* Only XOR the offset of the address when encrypting */
#define OPTIONS_KEYSIZE							0x40 /* Use a 256 bit KEY */
#define OPTIONS_QSPI							0x80 /* Use quad SPI */

#define SPIX_ALGO_STACK_SIZE					256
#define SPIX_ALGO_ENTRY_OFFSET 					0x42c

static const uint8_t write_code[] = {
#include "contrib/loaders/flash/max32xxx_qspi/max32xxx_qspi.inc"
};

struct max32xxx_qspi_flash_bank {
	bool probed;
	char devname[32];
	struct flash_device dev;
	unsigned int options;
};

FLASH_BANK_COMMAND_HANDLER(max32xxx_qspi_flash_bank_command)
{
	struct max32xxx_qspi_flash_bank *info;

	LOG_DEBUG("%s", __func__);

	if ((CMD_ARGC < 7) || (CMD_ARGC > 7)) {
		LOG_ERROR("incorrect flash bank max32xxx_qspi configuration: <flash_addr_base> <flash_addr_size> 0 0 <target> <opitons>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	info = malloc(sizeof(struct max32xxx_qspi_flash_bank));
	if (!info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = info;

	info->probed = false;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->options);
	bank->driver_priv = info;

	return ERROR_OK;
}

static int max32xxx_qspi_pre_op(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t temp32;

	/* Set the number of system clocks for the SPI clock low and high period */
	temp32 = (SPI_CLOCK_PERIOD << 8) | (SPI_CLOCK_PERIOD << 12);
	target_write_u32(target, SPIXFC_CFG, temp32);

	/* Enable the peripheral, FIFOs and SCK feedback */
	temp32 = (0x7 << 0) | (0x1 << 5) | (0x1 << 24);
	target_write_u32(target, SPIXFC_GEN_CTRL, temp32);

	return ERROR_OK;
}

static int max32xxx_qspi_post_op(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	uint32_t temp32;

	/* Disable SPIXFC */
	temp32 = 0;
	target_write_u32(target, SPIXFC_GEN_CTRL, temp32);

	/* Set the number of system clocks for the SPI clock low and high period */
	temp32 = (SPI_CLOCK_PERIOD << 8) | (SPI_CLOCK_PERIOD << 12) | (0x1 << 2);

	target_write_u32(target, SPIXF_CFG, temp32);


	/* Enter 1-2-2 mode */
	if(SPI_DUAL_MODE && (max32xxx_qspi_info->dev.dread_cmd != 0x0)) {
		LOG_DEBUG("Entering 1-2-2 read mode");

		/* Set the read command */
		temp32 = (0x1 << 10) | (0x1 << 12) | max32xxx_qspi_info->dev.dread_cmd;
		target_write_u32(target, SPIXF_FETCH_CTRL, temp32);

		/* Set mode control */
		temp32 = max32xxx_qspi_info->dev.dread_mode + max32xxx_qspi_info->dev.dread_dclk;
		target_write_u32(target, SPIXF_MODE_CTRL, temp32);
	} else {
		/* Set the read command */
		temp32 = max32xxx_qspi_info->dev.read_cmd;
		/* Enable 4 byte addresses if using read command 0x13 */
		if(max32xxx_qspi_info->dev.read_cmd == 0x13){
		    temp32 |= 0x1 << 16;
		}
		target_write_u32(target, SPIXF_FETCH_CTRL, temp32);

		/* Set mode control */
		temp32 = 0x0;
		target_write_u32(target, SPIXF_MODE_CTRL, temp32);
	}

	/* Setup the encryption options */
	if(max32xxx_qspi_info->options & OPTIONS_ENC) {
		temp32 = SPIXF_MEMSECCN_ENC_ENABLE;

		if(!(max32xxx_qspi_info->options & OPTIONS_AUTH)) {
			temp32 |= SPIXF_MEMSECCN_AUTH_DISABLE;
		}
		target_write_u32(target, SPIXF_MEMSECCN, temp32);
	} else {
		temp32 = 0;
		target_write_u32(target, SPIXF_MEMSECCN, temp32);
	}

	/* Enable feedback mode */
	temp32 = 0x1;
	target_write_u32(target, SPIXF_SCLK_FB_CTRL, temp32);

	/* Bus idle timeout */
	temp32 = 0x1;
	target_write_u32(target, SPIXF_BUS_IDLE, temp32);

	/* Enable and flush the SPI ICC */
	temp32 = 0;
	target_write_u32(target, SPI_ICC_CTRL, temp32);
	temp32 = SPI_ICC_CTRL_EN;
	target_write_u32(target, SPI_ICC_CTRL, temp32);

    /* Clear the code cache */
	target_read_u32(target, GCR_SCON, &temp32);
	temp32 |= (0x1 << 6);
	target_write_u32(target, GCR_SCON, temp32);

	return ERROR_OK;
}

static int max32xxx_qspi_write_txfifo(struct target *target, const uint8_t* data, unsigned len)
{
	uint32_t temp32;
	unsigned txFifoAvailable;
	unsigned dataIndex = 0;


	while (len - dataIndex) {

		unsigned writeLen;

		/* Calculate how many bytes we can write on this round */
		if ((len - dataIndex) > SPIXFC_FIFO_DEPTH)
			writeLen = SPIXFC_FIFO_DEPTH;
		else
			writeLen = (len - dataIndex);

		/* Wait for there to be room in the TX FIFO */
		unsigned retryCount = 10000;
		do {
			target_read_u32(target, SPIXFC_FIFO_CTRL, &temp32);
			txFifoAvailable = SPIXFC_FIFO_DEPTH - ((temp32 & SPIXFC_FIFO_CTRL_TX_FIFO_CNT)
				>> SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS);

			if(--retryCount == 0) {
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			}

		} while (txFifoAvailable < writeLen);

		while (writeLen) {
			uint16_t writeData = data[dataIndex++];

			if (dataIndex < len) {
				writeData |= (data[dataIndex++] << 8);
				writeLen -= 2;
			} else {
				writeData |= SPIXFC_HEADER_NULL;
				writeLen -= 1;
			}
			target_write_u16(target, SPIXFC_FIFO_TX, writeData);
		}
	}

	return ERROR_OK;
}

static int max32xxx_qspi_read_rxfifo(struct target *target, uint8_t* data, unsigned len)
{
	uint32_t temp32;
	unsigned rxFifoAvailable;
	unsigned dataIndex = 0;


	while (len - dataIndex) {

		unsigned readLen;

		/* Wait for there to be data in the RX FIFO */
		unsigned retryCount = 10000;
		do {
			target_read_u32(target, SPIXFC_FIFO_CTRL, &temp32);
			rxFifoAvailable = (temp32 & SPIXFC_FIFO_CTRL_RX_FIFO_CNT)
				>> SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS;

			if(--retryCount == 0) {
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			}

		} while (!rxFifoAvailable);

		/* Calculate how many bytes we can write on this round */
		if ((len - dataIndex) > rxFifoAvailable)
			readLen = rxFifoAvailable;
		else
			readLen = (len - dataIndex);

		while (readLen) {
			target_read_u8(target, SPIXFC_FIFO_RX, &data[dataIndex++]);
			readLen--;
		}
	}

	return ERROR_OK;
}

static int max32xxx_qspi_write_bytes(struct target *target, const uint8_t* data, unsigned len, bool deass)
{
	int retval;
	uint16_t header;
	unsigned chunkLen;
	unsigned dataIndex = 0;

	/* Wrap the length */
	while (len - dataIndex) {

		/* Max transaction length is 32 units */
		if ((len - dataIndex) > 32)
			chunkLen = 32;
		else
			chunkLen = (len - dataIndex);

		/* Setup the SPI header, 32 maps to 0 in the size field */
		header = (SPIXFC_HEADER_TX | SPIXFC_HEADER_BYTE);
		if (chunkLen == 32)
			header |= (0 << SPIXFC_HEADER_SIZE_POS);
		else
			header |= (chunkLen << SPIXFC_HEADER_SIZE_POS);

		/* If we de-asserting and this is the final chunk */
		if (deass && ((len - dataIndex - chunkLen) == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		/* Write the header to the TX FIFO */
		retval = max32xxx_qspi_write_txfifo(target, (uint8_t *)&header, sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		/* Write the data to the TX FIFO */
		retval = max32xxx_qspi_write_txfifo(target, &data[dataIndex], chunkLen);
		if (retval != ERROR_OK)
			return retval;
		dataIndex += chunkLen;
	}

	return ERROR_OK;
}

static int max32xxx_qspi_read_bytes(struct target *target, uint8_t* data, unsigned len, bool deass)
{
	int retval;
	uint16_t header;
	unsigned chunkLen;
	unsigned dataIndex = 0;

	/* Wrap the length */
	while (len - dataIndex) {

		/* Max transaction length is 32 units */
		if ((len - dataIndex) > 32)
			chunkLen = 32;
		else
			chunkLen = (len - dataIndex);

		/* Setup the SPI header, 32 maps to 0 in the size field */
		header = SPIXFC_HEADER_RX | SPIXFC_HEADER_BYTE;
		if (chunkLen == 32)
			header |= (0 << SPIXFC_HEADER_SIZE_POS);
		else
			header |= (chunkLen << SPIXFC_HEADER_SIZE_POS);

		/* If we de-asserting and this is the final chunk */
		if (deass && ((len - dataIndex - chunkLen) == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		/* Write the header to the TX FIFO */
		retval = max32xxx_qspi_write_txfifo(target, (uint8_t *)&header, sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		/* Read the data to the TX FIFO, convert to number of bytes */
		retval = max32xxx_qspi_read_rxfifo(target, (uint8_t *)&data[dataIndex], chunkLen);
		if (retval != ERROR_OK)
			return retval;
		dataIndex += chunkLen;
	}

	return ERROR_OK;
}

static int max32xxx_qspi_read_words(struct target *target, uint32_t* data, unsigned len, bool deass)
{
	int retval;
	uint16_t header;
	uint32_t temp32;
	unsigned chunkLen;
	unsigned dataIndex = 0;

	/* Configure the page size */
	target_read_u32(target, SPIXFC_CFG, &temp32);
	temp32 = (temp32 & ~(SPIXFC_CONFIG_PAGE_SIZE)) | SPIXFC_CONFIG_PAGE_SIZE_4_BYTES;
	target_write_u32(target, SPIXFC_CFG, temp32);

	while (len - dataIndex) {

		/* Max transaction length is 32 units */
		if ((len - dataIndex) > 32)
			chunkLen = 32;
		else
			chunkLen = (len - dataIndex);

		/* Setup the SPI header */
		header = SPIXFC_HEADER_RX | SPIXFC_HEADER_PAGE;
		if (chunkLen == 32)
			header |= (0 << SPIXFC_HEADER_SIZE_POS);
		else
			header |= (chunkLen << SPIXFC_HEADER_SIZE_POS);

		/* If we de-asserting and this is the final chunk */
		if (deass && ((len - dataIndex - chunkLen) == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		/* Write the header to the TX FIFO */
		retval = max32xxx_qspi_write_txfifo(target, (uint8_t *)&header, sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		/* Read the data to the TX FIFO, convert to number of bytes */
		retval = max32xxx_qspi_read_rxfifo(target, (uint8_t *)&data[dataIndex * 4], chunkLen * 4);
		if (retval != ERROR_OK)
			return retval;
		dataIndex += chunkLen;
	}

	return ERROR_OK;
}

static int max32xxx_qspi_poll_wip(struct target *target)
{
	uint8_t cmdData = SPIFLASH_READ_STATUS;
	uint8_t readData;
	int retval;

	do {
		retval = max32xxx_qspi_write_bytes(target, &cmdData, 1, false);
		if (retval != ERROR_OK)
			return retval;

		readData = SPIFLASH_BSY_BIT;
		retval = max32xxx_qspi_read_bytes(target, &readData, 1, true);
		if (retval != ERROR_OK)
			return retval;

		/* Prevent GDB warnings */
		keep_alive();

	} while (readData & SPIFLASH_BSY_BIT);


	/* TODO Timeout */
	return ERROR_OK;
}

static int max32xxx_qspi_set_we(struct target *target)
{
	uint8_t cmdData = SPIFLASH_WRITE_ENABLE;

	/* TODO: Could also be instruction 0x50 */
	return max32xxx_qspi_write_bytes(target, &cmdData, 1, true);
}

static int max32xxx_qspi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	int retval;
	uint8_t cmdData[5];
	uint32_t addr;

	LOG_DEBUG("%s: first = %d last = %d\n", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(max32xxx_qspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	max32xxx_qspi_pre_op(bank);

	while (first <= last) {

		/* Set the write enable */
		retval = max32xxx_qspi_set_we(target);
		if (retval != ERROR_OK)
			goto exit;

		/* Send the erase command */
		cmdData[0] = max32xxx_qspi_info->dev.erase_cmd;
		/* Address is MSB first */
		addr = first++ * max32xxx_qspi_info->dev.sectorsize;
		
		if(max32xxx_qspi_info->dev.erase_cmd == 0xdc){
		    cmdData[4] = (addr & 0x000000FF) >> 0;
		    cmdData[3] = (addr & 0x0000FF00) >> 8;
		    cmdData[2] = (addr & 0x00FF0000) >> 16;
		    cmdData[1] = (addr & 0xFF000000) >> 24;
		    retval = max32xxx_qspi_write_bytes(target, cmdData, 5, true);
		}else{
		
		    cmdData[3] = (addr & 0x0000FF) >> 0;
		    cmdData[2] = (addr & 0x00FF00) >> 8;
		    cmdData[1] = (addr & 0xFF0000) >> 16;
		    retval = max32xxx_qspi_write_bytes(target, cmdData, 4, true);
		}
		
		if (retval != ERROR_OK)
			goto exit;

		/* Poll WIP until erase is complete */
		max32xxx_qspi_poll_wip(target);
	}

exit:
	max32xxx_qspi_post_op(bank);

	return retval;
}

static int max32xxx_qspi_write_block(struct flash_bank *bank, const uint8_t *buffer,
                                uint32_t offset, uint32_t len)
{
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *source;
	struct working_area *write_algorithm;
	struct reg_param reg_params[5];
	struct mem_param mem_param[2];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	/* power of two, and multiple of word size */
	static const unsigned buf_min = 512;

	LOG_DEBUG("max32xxx_write_block bank=%p buffer=%p offset=%08" PRIx32 " len=%08" PRIx32 "",
	          bank, buffer, offset, len);

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(write_code), &write_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;

		if (buffer_size <= buf_min) {
			target_free_working_area(target, write_algorithm);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		LOG_DEBUG("retry target_alloc_working_area(%s, size=%u)",
		          target_name(target), (unsigned) buffer_size);
	}

	target_write_buffer(target, write_algorithm->address, sizeof(write_code),
	                    write_code);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, len);
	buf_set_u32(reg_params[3].value, 0, 32, offset);
	buf_set_u32(reg_params[4].value, 0, 32, source->address + source->size);

	/* mem_params for options */
	/* leave room for stack, 32-bit options, 32-bit SPI write command */
	init_mem_param(&mem_param[0], source->address + (source->size - 4  - 
		SPIX_ALGO_STACK_SIZE), 4, PARAM_OUT);
	init_mem_param(&mem_param[1], source->address + (source->size - 8  - 
		SPIX_ALGO_STACK_SIZE), 4, PARAM_OUT);
	buf_set_u32(mem_param[0].value, 0, 32, max32xxx_qspi_info->options);
	buf_set_u32(mem_param[1].value, 0, 32, max32xxx_qspi_info->dev.pprog_cmd);

	LOG_DEBUG("max32xxx_write_block source->address=%08" PRIx32 " source->size=%08" PRIx32 "", (unsigned int)source->address, (unsigned int)source->size);

	/* leave room for stack, 32-bit options, 32-bit SPI write command */
	/* Algorithm entry point is inside the code block, not at the beginning */
	retval = target_run_flash_async_algorithm(target, 
			buffer, len, 1, 
			2, mem_param,
	        5, reg_params, 
	        source->address, (source->size - 4 - 4 - SPIX_ALGO_STACK_SIZE), 
	        (write_algorithm->address + SPIX_ALGO_ENTRY_OFFSET), 0, 
	        &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("error %d executing max32xxx qspi write algorithm", retval);

	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int max32xxx_qspi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	int retval;
	uint8_t cmdData[5];
	unsigned writeLen, bufferIndex;
    
	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(max32xxx_qspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > bank->size) {
		LOG_ERROR("Write beyond end of flash.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Determine if we want to use the on-chip algorithm */
	if((max32xxx_qspi_info->options & OPTIONS_ENC) || (count > 16)) {

		if(max32xxx_qspi_info->options & OPTIONS_AUTH) {
			/* Need to erase extra length if we're writing authentication data */
			uint32_t max_sector_plain = (offset + count) / max32xxx_qspi_info->dev.sectorsize;
			uint32_t max_sector_auth = (offset + (count * 10 / 8)) / max32xxx_qspi_info->dev.sectorsize;
			if(max_sector_auth > max_sector_plain) {
				LOG_WARNING("Erasing extra flash for authentication data");
				retval = max32xxx_qspi_erase(bank, max_sector_plain, max_sector_auth);
				if(retval != ERROR_OK) {
					return retval;
				}
			}
		}

		max32xxx_qspi_pre_op(bank);

		retval = max32xxx_qspi_write_block(bank, buffer, offset, count);

		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				if(max32xxx_qspi_info->options & OPTIONS_ENC) {
					LOG_ERROR("Must use algorithm in working area for encryption");
					goto exit;

				}
				LOG_DEBUG("working area algorithm not available");
			} else {
				LOG_ERROR("Error with flash algorithm");
				goto exit;
			}
		} else {
			goto exit;
		}
	} else {
		max32xxx_qspi_pre_op(bank);
	}

	/* Send the page program command */
	cmdData[0] = max32xxx_qspi_info->dev.pprog_cmd;

	bufferIndex = 0;

	/* Get on the write boundary */
	if (offset % SPI_WRITE_BOUNDARY) {

		/* Set the write enable */
		retval = max32xxx_qspi_set_we(target);
		if (retval != ERROR_OK)
			goto exit;

		writeLen = SPI_WRITE_BOUNDARY - (offset % SPI_WRITE_BOUNDARY);

		if (writeLen > count)
			writeLen = count;

        if(max32xxx_qspi_info->dev.pprog_cmd == 0x12){
        	cmdData[4] = (offset & 0x000000FF) >> 0;
		    cmdData[3] = (offset & 0x0000FF00) >> 8;
		    cmdData[2] = (offset & 0x00FF0000) >> 16;
		    cmdData[1] = (offset & 0xFF000000) >> 24;
		    /* Write the command */
		    retval = max32xxx_qspi_write_bytes(target, cmdData, 5, false);
        }else{
            /* Address is MSB first */
            cmdData[3] = (offset & 0x0000FF) >> 0;
            cmdData[2] = (offset & 0x00FF00) >> 8;
            cmdData[1] = (offset & 0xFF0000) >> 16;
            /* Write the command */
		    retval = max32xxx_qspi_write_bytes(target, cmdData, 4, false);
        }

		if (retval != ERROR_OK)
			goto exit;

		/* Write the data */
		retval = max32xxx_qspi_write_bytes(target, &buffer[bufferIndex], writeLen, true);
		if (retval != ERROR_OK)
			goto exit;

		/* Increment the pointers */
		bufferIndex += writeLen;
		offset += writeLen;
	}

	while (count - bufferIndex) {

		/* Set the write enable */
		retval = max32xxx_qspi_set_we(target);
		if (retval != ERROR_OK)
			goto exit;

		/* Write along the boundary */
		writeLen = SPI_WRITE_BOUNDARY;

		if (writeLen > (count - bufferIndex))
			writeLen = (count - bufferIndex);

        if(max32xxx_qspi_info->dev.pprog_cmd == 0x12){
        	cmdData[4] = (offset & 0x000000FF) >> 0;
		    cmdData[3] = (offset & 0x0000FF00) >> 8;
		    cmdData[2] = (offset & 0x00FF0000) >> 16;
		    cmdData[1] = (offset & 0xFF000000) >> 24;
		    /* Write the command */
		    retval = max32xxx_qspi_write_bytes(target, cmdData, 5, false);
        }else{
            /* Address is MSB first */
            cmdData[3] = (offset & 0x0000FF) >> 0;
            cmdData[2] = (offset & 0x00FF00) >> 8;
            cmdData[1] = (offset & 0xFF0000) >> 16;
            /* Write the command */
		    retval = max32xxx_qspi_write_bytes(target, cmdData, 4, false);
        }
        
		if (retval != ERROR_OK)
			goto exit;

		/* Write the data */
		retval = max32xxx_qspi_write_bytes(target, &buffer[bufferIndex], writeLen, true);
		if (retval != ERROR_OK)
			goto exit;

		/* Increment the pointers */
		bufferIndex += writeLen;
		offset += writeLen;
	}

exit:
	max32xxx_qspi_post_op(bank);

	return retval;
}

static int read_sfdp_block(struct flash_bank *bank, uint32_t addr,
	uint32_t words, uint32_t *buffer)
{
	struct target *target = bank->target;
	uint8_t cmdData[5];

	/* Write the command */
	cmdData[0] = SPIFLASH_READ_SFDP;

	/* Address is MSB first */
	cmdData[3] = (addr & 0x0000FF) >> 0;
	cmdData[2] = (addr & 0x00FF00) >> 8;
	cmdData[1] = (addr & 0xFF0000) >> 16;

	/* 1 dummy bytes */
	cmdData[4] = 0;

	max32xxx_qspi_write_bytes(target, cmdData, 5, false);

	/* Read the response, convert words to number of bytes */
	max32xxx_qspi_read_words(target, buffer, words, true);

	return ERROR_OK;
}

static int max32xxx_qspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct flash_device temp_flash_device;
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	int retval;
	uint32_t temp32;
	uint8_t cmd;

	LOG_DEBUG("%s", __func__);

	if (max32xxx_qspi_info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		free(bank->sectors);
		bank->sectors = NULL;
		memset(&max32xxx_qspi_info->dev, 0, sizeof(max32xxx_qspi_info->dev));
		max32xxx_qspi_info->probed = false;
	}

	max32xxx_qspi_pre_op(bank);

	target_read_u32(target, SPIXFC_CFG, &temp32);
	LOG_DEBUG("SPIXFC_CFG       = 0x%08X", temp32);

	target_read_u32(target, SPIXFC_SS_POL, &temp32);
	LOG_DEBUG("SPIXFC_SS_POL    = 0x%08X", temp32);

	target_read_u32(target, SPIXFC_GEN_CTRL, &temp32);
	LOG_DEBUG("SPIXFC_GEN_CTRL  = 0x%08X", temp32);

	target_read_u32(target, SPIXFC_FIFO_CTRL, &temp32);
	LOG_DEBUG("SPIXFC_FIFO_CTRL = 0x%08X", temp32);

	/* Read the SFDP settings from the flash device */
	retval = spi_sfdp(bank, &temp_flash_device, &read_sfdp_block);
	if (retval != ERROR_OK)
		goto exit;
	LOG_INFO("max32xxx flash \'%s\' size = %" PRIu32 "kbytes",
		temp_flash_device.name, temp_flash_device.size_in_bytes / 1024);

	max32xxx_qspi_info->dev = temp_flash_device;

	/* Read the device ID */
	cmd = SPIFLASH_READ_ID;
	retval = max32xxx_qspi_write_bytes(target, &cmd, 1, false);
	if (retval != ERROR_OK)
		goto exit;

	retval = max32xxx_qspi_read_bytes(target, (uint8_t *) &(max32xxx_qspi_info->dev.device_id), 3, true);
	if (retval != ERROR_OK)
		goto exit;

	/* Set correct size value */
	bank->size = max32xxx_qspi_info->dev.size_in_bytes;

	/* TODO: Get more than 1 erase command */

	/* Create and fill sectors array */
	bank->num_sectors = max32xxx_qspi_info->dev.size_in_bytes / max32xxx_qspi_info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		retval = ERROR_FAIL;
		goto exit;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * max32xxx_qspi_info->dev.sectorsize;
		sectors[sector].size = max32xxx_qspi_info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;

	max32xxx_qspi_info->probed = true;
	
	/* Setup memory mapped mode */
	max32xxx_qspi_post_op(bank);

	target_read_u32(target, SPIXF_CFG, &temp32);
	LOG_DEBUG("SPIXF_CFG			= 0x%08X", temp32);
	target_read_u32(target, SPIXF_FETCH_CTRL, &temp32);
	LOG_DEBUG("SPIXF_FETCH_CTRL		= 0x%08X", temp32);
	target_read_u32(target, SPIXF_MODE_CTRL, &temp32);
	LOG_DEBUG("SPIXF_MODE_CTRL		= 0x%08X", temp32);
	target_read_u32(target, SPIXF_MODE_DATA, &temp32);
	LOG_DEBUG("SPIXF_MODE_DATA		= 0x%08X", temp32);
	target_read_u32(target, SPIXF_SCLK_FB_CTRL, &temp32);
	LOG_DEBUG("SPIXF_SCLK_FB_CTRL	= 0x%08X", temp32);
	target_read_u32(target, SPIXF_IO_CTRL, &temp32);
	LOG_DEBUG("SPIXF_IO_CTRL		= 0x%08X", temp32);
	target_read_u32(target, SPIXF_MEMSECCN, &temp32);
	LOG_DEBUG("SPIXF_MEMSECCN		= 0x%08X", temp32);
	target_read_u32(target, SPIXF_BUS_IDLE, &temp32);
	LOG_DEBUG("SPIXF_BUS_IDLE		= 0x%08X", temp32);

exit:

	return retval;
}

static int max32xxx_qspi_auto_probe(struct flash_bank *bank)
{
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;

	if (max32xxx_qspi_info->probed)
		return ERROR_OK;
	max32xxx_qspi_probe(bank);
	return ERROR_OK;
}

static int max32xxx_qspi_protect_check(struct flash_bank *bank)
{
	/* TODO: max32xxx_qspi_protect_check */
	return ERROR_OK;
}

static int max32xxx_qspi_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	/* TODO: max32xxx_qspi_protect */
	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_qspi_handle_reset_deassert_command)
{
	int retval;

	struct flash_bank *bank;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	max32xxx_qspi_post_op(bank);
	return retval;
}

COMMAND_HANDLER(max32xxx_qspi_handle_mass_erase_command)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info;
	int retval;
	uint8_t cmdData[1];

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	max32xxx_qspi_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(max32xxx_qspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (max32xxx_qspi_info->dev.chip_erase_cmd == 0x00) {
		LOG_ERROR("Mass erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	max32xxx_qspi_pre_op(bank);

	/* Set the write enable */
	retval = max32xxx_qspi_set_we(target);
	if (retval != ERROR_OK)
		goto exit;

	/* Send the mass erase command */
	cmdData[0] = max32xxx_qspi_info->dev.chip_erase_cmd;

	retval = max32xxx_qspi_write_bytes(target, cmdData, 1, true);
	if (retval != ERROR_OK)
		goto exit;

	/* Poll WIP until erase is complete */
	max32xxx_qspi_poll_wip(target);

exit:
	max32xxx_qspi_post_op(bank);

	return retval;
}

static int get_max32xxx_qspi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;

	if (!(max32xxx_qspi_info->probed)) {
		command_print_sameline(cmd, "\nQSPI flash bank not probed yet\n");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	command_print_sameline(cmd, "\nQSPI flash:\n");

	command_print_sameline(cmd, "  name          : \'%s\'\n", max32xxx_qspi_info->dev.name);
	command_print_sameline(cmd, "  ID            : 0x%06" PRIx32 "\n", max32xxx_qspi_info->dev.device_id);
	command_print_sameline(cmd, "  size          : 0x%08" PRIx32 " B\n", max32xxx_qspi_info->dev.size_in_bytes);
	command_print_sameline(cmd, "  page size     : 0x%08" PRIx32 " B\n", max32xxx_qspi_info->dev.pagesize);
	command_print_sameline(cmd, "  sector size   : 0x%08" PRIx32 " B\n", max32xxx_qspi_info->dev.sectorsize);
	command_print_sameline(cmd, "  read cmd      : 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.read_cmd);
	command_print_sameline(cmd, "  dread cmd     : 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.dread_cmd);
	command_print_sameline(cmd, "  dread mode    : 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.dread_mode);
	command_print_sameline(cmd, "  dread dclk    : 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.dread_dclk);
	command_print_sameline(cmd, "  qread cmd     : 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.qread_cmd);
	command_print_sameline(cmd, "  pprog cmd     : 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.pprog_cmd);
	command_print_sameline(cmd, "  erase cmd     : 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.erase_cmd);
	command_print_sameline(cmd, "  chip_erase cmd: 0x%02" PRIx32 "\n", max32xxx_qspi_info->dev.chip_erase_cmd);

	return ERROR_OK;
}

static const struct command_registration max32xxx_qspi_exec_command_handlers[] = {
	{
		.name = "reset_deassert",
		.handler = max32xxx_qspi_handle_reset_deassert_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Setup the QSPI after a reset event.",
	},
	{
		.name = "mass_erase",
		.handler = max32xxx_qspi_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration max32xxx_qspi_command_handlers[] = {
	{
		.name = "max32xxx_qspi",
		.mode = COMMAND_ANY,
		.help = "max32xxx_qspi flash command group",
		.usage = "",
		.chain = max32xxx_qspi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver max32xxx_qspi_flash = {
	.name = "max32xxx_qspi",
	.commands = max32xxx_qspi_command_handlers,
	.flash_bank_command = max32xxx_qspi_flash_bank_command,
	.erase = max32xxx_qspi_erase,
	.protect = max32xxx_qspi_protect,
	.write = max32xxx_qspi_write,
	.read = default_flash_read,
	.verify = default_flash_verify,
	.probe = max32xxx_qspi_probe,
	.auto_probe = max32xxx_qspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = max32xxx_qspi_protect_check,
	.info = get_max32xxx_qspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

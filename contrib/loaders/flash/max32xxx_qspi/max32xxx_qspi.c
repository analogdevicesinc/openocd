/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *******************************************************************************
 */

/***** Includes *****/
#ifdef ALGO_TEST
#include "mxc_device.h"
#endif

#include "tpu_regs.h"
#include "gcr_regs.h"
#include "flc_regs.h"
#include "algo_options.h"

#ifdef ALGO_TEST
#include <stdio.h>
#else
#define printf(...)
#endif

/***** Definitions *****/
#define MXC_BASE_TPU                            ((uint32_t)0x40001000UL)
#define MXC_TPU                                 ((mxc_tpu_regs_t*)MXC_BASE_TPU)
#define MXC_BASE_GCR                            ((uint32_t)0x40000000UL)
#define MXC_GCR                                 ((mxc_gcr_regs_t*)MXC_BASE_GCR)

#define ERROR_OK                                0

#define SPIFLASH_WRITE_ENABLE                   0x06
#define SPIFLASH_READ_STATUS                    0x05
#define SPIFLASH_BSY                            0
#define SPIFLASH_BSY_BIT                        (1 << SPIFLASH_BSY)

/* Address boundary for writes */
#define SPI_WRITE_BOUNDARY                      256

#define SPIXFC_BASE                             0x40027000
#define SPIXFC_CFG                              (SPIXFC_BASE | 0x00)
#define SPIXFC_SS_POL                           (SPIXFC_BASE | 0x04)
#define SPIXFC_GEN_CTRL                         (SPIXFC_BASE | 0x08)
#define SPIXFC_FIFO_CTRL                        (SPIXFC_BASE | 0x0C)

#define SPIXFC_FIFO_DEPTH                       (16)
#define SPIXFC_HEADER_TX                        0x1
#define SPIXFC_HEADER_RX                        0x2
#define SPIXFC_HEADER_BIT                       (0x0 << 2)
#define SPIXFC_HEADER_BYTE                      (0x1 << 2)
#define SPIXFC_HEADER_PAGE                      (0x2 << 2)
#define SPIXFC_HEADER_SIZE_POS                  4
#define SPIXFC_HEADER_WIDTH_POS                 9
#define SPIXFC_HEADER_SS_DEASS                  (0x1 << 13)
#define SPIXFC_HEADER_NULL                      0xF000  /* 16-bit filler magic word indicating this isn't a header */

#define SPIXFC_CONFIG_PAGE_SIZE_POS             6
#define SPIXFC_CONFIG_PAGE_SIZE                 (0x3 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_4_BYTES         (0x0 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_8_BYTES         (0x1 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_16_BYTES        (0x2 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_32_BYTES        (0x3 << SPIXFC_CONFIG_PAGE_SIZE_POS)

#define SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS        8
#define SPIXFC_FIFO_CTRL_TX_FIFO_CNT            (0x1FUL << SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS)

#define SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS        24
#define SPIXFC_FIFO_CTRL_RX_FIFO_CNT            (0x3FUL << SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS)

#define SPIXFC_FIFO_TX                          0x400BC000
#define SPIXFC_FIFO_RX                          0x400BC004

#define STACK_SIZE                              256

/******************************************************************************/
#define getbyte(temp8)                                                          \
    /* Wait for the Read FIFO to not equal the Write FIFO */                    \
    while (*read_ptr == *write_ptr);                                            \
    temp8 = **read_ptr;                                                         \
                                                                                \
    /* Increment and wrap around the read pointer */                            \
    if ((*read_ptr + 1) >= (uint8_t*)(work_end)) {                              \
        *read_ptr = (uint8_t *)(work_start + 8);                                \
    } else {                                                                    \
        (*read_ptr)++;                                                          \
    }                                                                           \
    len--;                                                                      \
    addr++;

/******************************************************************************/

void target_read_u32(uint32_t addr, uint32_t* data)
{
	volatile uint32_t* dataPtr = (uint32_t*)addr;

	*data = *dataPtr;
}

void target_read_u8(uint32_t addr, uint8_t* data)
{
	volatile uint8_t* dataPtr = (uint8_t*)addr;

	*data = *dataPtr;
}

void target_write_u16(uint32_t addr, uint16_t data)
{
	volatile uint16_t* dataPtr = (uint16_t*)addr;

	*dataPtr = data;
}

int max32xxx_qspi_write_txfifo(const uint8_t* data, unsigned len)
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
		do {
			temp32 = 0;
			target_read_u32(SPIXFC_FIFO_CTRL, &temp32);
			txFifoAvailable = SPIXFC_FIFO_DEPTH - ((temp32 & SPIXFC_FIFO_CTRL_TX_FIFO_CNT)
			                                       >> SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS);

			/* TODO: Timeout */
			/* ERROR_TARGET_RESOURCE_NOT_AVAILABLE */

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
			target_write_u16(SPIXFC_FIFO_TX, writeData);
		}
	}

	return ERROR_OK;
}

int max32xxx_qspi_read_rxfifo(uint8_t* data, unsigned len)
{
	uint32_t temp32;
	unsigned rxFifoAvailable;
	unsigned dataIndex = 0;

	while (len - dataIndex) {
		unsigned readLen;

		/* Wait for there to be data in the RX FIFO */
		do {
			target_read_u32(SPIXFC_FIFO_CTRL, &temp32);
			rxFifoAvailable = (temp32 & SPIXFC_FIFO_CTRL_RX_FIFO_CNT)
			                  >> SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS;

			/* TODO: Timeout */
			/* ERROR_TARGET_RESOURCE_NOT_AVAILABLE */

		} while (!rxFifoAvailable);

		/* Calculate how many bytes we can write on this round */
		if ((len - dataIndex) > rxFifoAvailable)
			readLen = rxFifoAvailable;
		else
			readLen = (len - dataIndex);

		while (readLen) {
			target_read_u8(SPIXFC_FIFO_RX, &data[dataIndex++]);
			readLen--;
		}
	}

	return ERROR_OK;
}

int max32xxx_qspi_write_bytes(const uint8_t* data, unsigned len, int deass)
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
		retval = max32xxx_qspi_write_txfifo((uint8_t *)&header, sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		/* Write the data to the TX FIFO */
		retval = max32xxx_qspi_write_txfifo(&data[dataIndex], chunkLen);
		if (retval != ERROR_OK)
			return retval;
		dataIndex += chunkLen;
	}

	return ERROR_OK;
}

int max32xxx_qspi_read_bytes(uint8_t* data, unsigned len, int deass)
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
		retval = max32xxx_qspi_write_txfifo((uint8_t *)&header, sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		/* Read the data to the TX FIFO, convert to number of bytes */
		retval = max32xxx_qspi_read_rxfifo((uint8_t *)&data[dataIndex], chunkLen);
		if (retval != ERROR_OK)
			return retval;
		dataIndex += chunkLen;
	}

	return ERROR_OK;
}

int max32xxx_qspi_poll_wip(void)
{
	uint8_t cmdData = SPIFLASH_READ_STATUS;
	uint8_t readData;
	int retval;

	do {
		retval = max32xxx_qspi_write_bytes(&cmdData, 1, 0);
		if (retval != ERROR_OK)
			return retval;

		readData = SPIFLASH_BSY_BIT;
		retval = max32xxx_qspi_read_bytes(&readData, 1, 1);
		if (retval != ERROR_OK)
			return retval;

	} while (readData & SPIFLASH_BSY_BIT);


	/* TODO Timeout */
	return ERROR_OK;
}

int max32xxx_qspi_set_we(void)
{
	uint8_t cmdData = SPIFLASH_WRITE_ENABLE;

	/* TODO: Could also be instruction 0x50 */
	return max32xxx_qspi_write_bytes(&cmdData, 1, 1);
}

int max32xxx_qspi_write(const uint8_t *buffer, uint32_t offset, uint32_t count, uint32_t spi_cmd)
{
	int retval;
	uint8_t cmdData[4];
	unsigned writeLen, bufferIndex;

	/* Send the page program command */
	cmdData[0] = spi_cmd & 0xFF;

	bufferIndex = 0;

	/* Get on the write boundary */
	if (offset % SPI_WRITE_BOUNDARY) {

		max32xxx_qspi_poll_wip();

		/* Set the write enable */
		retval = max32xxx_qspi_set_we();
		if (retval != ERROR_OK)
			goto exit;

		writeLen = SPI_WRITE_BOUNDARY - (offset % SPI_WRITE_BOUNDARY);

		if (writeLen > count)
			writeLen = count;

		/* Address is MSB first */
		cmdData[3] = (offset & 0x0000FF) >> 0;
		cmdData[2] = (offset & 0x00FF00) >> 8;
		cmdData[1] = (offset & 0xFF0000) >> 16;

		/* Write the command */
		retval = max32xxx_qspi_write_bytes(cmdData, 4, 0);
		if (retval != ERROR_OK)
			goto exit;

		/* Write the data */
		retval = max32xxx_qspi_write_bytes(&buffer[bufferIndex], writeLen, 1);
		if (retval != ERROR_OK)
			goto exit;

		/* Increment the pointers */
		bufferIndex += writeLen;
		offset += writeLen;
	}

	while (count - bufferIndex) {

		max32xxx_qspi_poll_wip();

		/* Set the write enable */
		retval = max32xxx_qspi_set_we();
		if (retval != ERROR_OK)
			goto exit;

		/* Write along the boundary */
		writeLen = SPI_WRITE_BOUNDARY;

		if (writeLen > (count - bufferIndex))
			writeLen = (count - bufferIndex);

		/* Address is MSB first */
		cmdData[3] = (offset & 0x0000FF) >> 0;
		cmdData[2] = (offset & 0x00FF00) >> 8;
		cmdData[1] = (offset & 0xFF0000) >> 16;

		/* Write the command */
		retval = max32xxx_qspi_write_bytes(cmdData, 4, 0);
		if (retval != ERROR_OK)
			goto exit;

		/* Write the data */
		retval = max32xxx_qspi_write_bytes(&buffer[bufferIndex], writeLen, 1);
		if (retval != ERROR_OK)
			goto exit;

		/* Increment the pointers */
		bufferIndex += writeLen;
		offset += writeLen;
	}

exit:
	return retval;
}

#ifndef ALGO_TEST
__attribute__ ((naked))
#endif
void algo_write(uint8_t *work_start, uint8_t *work_end, uint32_t len, uint32_t addr)
{

	printf(" > algo_write() starting\n");

	/* Setup the pointers */
	uint8_t * volatile *write_ptr = (uint8_t **)work_start;
	uint8_t * volatile *read_ptr = (uint8_t **)(work_start + 4);
	uint32_t *spi_write_cmd = (uint32_t *)(work_end - 32 - STACK_SIZE - 8);
	uint32_t *options = (uint32_t *)(work_end - 32 - STACK_SIZE - 4);
	uint32_t *enc_buffer = (uint32_t *)(work_end - 32 - STACK_SIZE);
	uint8_t temp8;
	uint32_t addr_save;
	int i;

	/* Adjust the work_end pointer to the end of the working area buffer */
	work_end = (uint8_t *)(work_end - 32 - STACK_SIZE - 8);

	printf(" > w:%08x r:%08x o:%08x enc:%08x s:%08x e:%08x spi:%08x\n",
	       (uint32_t)write_ptr, (uint32_t)read_ptr, (uint32_t)*options, (uint32_t)enc_buffer,
	       (uint32_t)work_start, (uint32_t)work_end, (uint32_t)*spi_write_cmd);

	/* Make sure the address is on a 128 bit boundary */
	if (addr & 0xF) {
		#ifndef ALGO_TEST
		__asm("bkpt\n");
		#else
		printf(" > algo_write returning, not on a 128 bit boundary\n");
		return;
		#endif
	}

	if (*options & OPTIONS_ENC) {
		/* Enable Memory Protection */
		MXC_GCR->scon |= MXC_F_GCR_SCON_MEMPROT_EN;

		/* Set the keysize */
		if (*options & OPTIONS_KEYSIZE)
			MXC_GCR->scon |= MXC_F_GCR_SCON_MEMPROT_KEYSZ;

		else
			MXC_GCR->scon &= ~(MXC_F_GCR_SCON_MEMPROT_KEYSZ);
	} else {
		/* Disable memory protection */
		MXC_GCR->scon &= ~MXC_F_GCR_SCON_MEMPROT_EN;
	}

	if (*options & OPTIONS_ENC) {
		/* Setup the AES */

		/* Enable CRYPTO clock */
		if ((MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC_EN) == 0)
			MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC_EN;

		/* Disable CRYPTO clock gate */
		if (MXC_GCR->perckcn0 & MXC_F_GCR_PERCKCN0_CRYPTOD)
			MXC_GCR->perckcn0 &= ~(MXC_F_GCR_PERCKCN0_CRYPTOD);

		/* Reset Crypto block and clear state */
		MXC_TPU->ctrl = MXC_F_TPU_CTRL_RST;

		/* Set the legacy bit */
		MXC_TPU->ctrl |= MXC_F_TPU_CTRL_FLAG_MODE;

		/* Byte swap the input and output */
		MXC_TPU->ctrl |= MXC_F_TPU_CTRL_BSO;
		MXC_TPU->ctrl |= MXC_F_TPU_CTRL_BSI;
	}

	while (len) {

		/* Save the current address before we read from the working area */
		addr_save = addr;

		/* Fill the buffer with the plain text data from the working area */
		for (i = 0; i < 4; i++) {
			/* Get data from the working area, pad with 0xFF */
			enc_buffer[i] = 0;
			if (len){
				getbyte(temp8);
            } else {
				temp8 = 0xFF;
            }
			enc_buffer[i] |= (temp8 << (0));
			/* Get data from the working area, pad with 0xFF */
			if (len){
				getbyte(temp8);
            } else{
				temp8 = 0xFF;
            }
			enc_buffer[i] |= (temp8 << (8));
			/* Get data from the working area, pad with 0xFF */
			if (len) {
				getbyte(temp8);
            } else {
				temp8 = 0xFF;
            }
			enc_buffer[i] |= (temp8 << (16));
			/* Get data from the working area, pad with 0xFF */
			if (len) {
				getbyte(temp8);
            } else{
				temp8 = 0xFF;
            }
			enc_buffer[i] |= (temp8 << (24));
		}

		if (*options & OPTIONS_ENC) {

			/* XOR data with the address */
			for (i = 0; i < 4; i++) {
				if (*options & OPTIONS_RELATIVE_XOR)
					enc_buffer[i] ^= ((addr_save & 0x00FFFFFF) + i * 4);

				else
					enc_buffer[i] ^= (addr_save + i * 4);
			}

			/* Encrypt the plain text */
			/* Clear interrupt flags */
			MXC_TPU->ctrl |= MXC_F_TPU_CTRL_CPH_DONE;

			MXC_TPU->cipher_ctrl = ((0x0 << MXC_F_TPU_CIPHER_CTRL_MODE_POS) |
			                        (0x0 << MXC_F_TPU_CIPHER_CTRL_ENC_POS));

			if (*options & OPTIONS_KEYSIZE) {
				/* ECB, AES-256, encrypt */
				MXC_TPU->cipher_ctrl |= (0x3 << MXC_F_TPU_CIPHER_CTRL_CIPHER_POS);
			} else {
				/* ECB, AES-128, encrypt */
				MXC_TPU->cipher_ctrl |= (0x1 << MXC_F_TPU_CIPHER_CTRL_CIPHER_POS);
			}

			/* Set the key source */
			MXC_TPU->cipher_ctrl = ((MXC_TPU->cipher_ctrl & ~MXC_F_TPU_CIPHER_CTRL_SRC) |
			                        (0x3 << MXC_F_TPU_CIPHER_CTRL_SRC_POS));

			/* Copy data to start the operation */
			MXC_TPU->din[0] = enc_buffer[0];
			MXC_TPU->din[1] = enc_buffer[1];
			MXC_TPU->din[2] = enc_buffer[2];
			MXC_TPU->din[3] = enc_buffer[3];

			/* Wait until operation is complete */
			while (!(MXC_TPU->ctrl & MXC_F_TPU_CTRL_CPH_DONE)) {}

			/* Copy the data out */
			enc_buffer[0] = MXC_TPU->dout[0];
			enc_buffer[1] = MXC_TPU->dout[1];
			enc_buffer[2] = MXC_TPU->dout[2];
			enc_buffer[3] = MXC_TPU->dout[3];
		}

		if (max32xxx_qspi_write((const uint8_t*)enc_buffer, addr_save, 16, *spi_write_cmd) != ERROR_OK) {
			#ifndef ALGO_TEST
			__asm("bkpt\n");
			#else
			printf(" > algo_write error\n");
			return;
			#endif
		}

		/* Restart the command if this contains the last byte on the boundary */
	}

	#ifndef ALGO_TEST
	__asm("bkpt\n");
	#else
	printf(" > algo_write returning\n");
	return;
	#endif
}

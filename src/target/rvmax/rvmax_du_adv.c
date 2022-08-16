/***************************************************************************
 *   Copyright (C) 2013-2014 by Franck Jullien                             *
 *   elec4fun@gmail.com                                                    *
 *                                                                         *
 *   Inspired from adv_jtag_bridge which is:                               *
 *   Copyright (C) 2008-2010 Nathan Yawn                                   *
 *   nyawn@opencores.net                                                   *
 *                                                                         *
 *   And the Mohor interface version of this file which is:                *
 *   Copyright (C) 2011 by Julius Baxter                                   *
 *   julius@opencores.org                                                  *
 *                                                                         *
 *   Maxim PULP-inspired core version                                      *
 *   Copyright (C) 2019 by Maxim Integrated                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
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

#include "rvmax_tap.h"
#include "rvmax.h"
#include "rvmax_du.h"

#include <target/target.h>
#include <jtag/jtag.h>

#define NO_OPTION           0

/* This an option to the adv debug unit.
 * If this is defined, status bits will be skipped on burst
 * reads and writes to improve download speeds.
 * This option must match the RTL configured option.
 */
#define ADBG_USE_HISPEED    1

/* Definitions for the top-level debug unit.  This really just consists
 * of a single register, used to select the active debug module ("chain").
 */
#define DBG_MODULE_SELECT_REG_SIZE  2
#define DBG_MAX_MODULES             4

#define DC_NONE                     -1
#define DC_WISHBONE                 0
#define DC_CPU0                     1
#define DC_CPU1                     2
#define DC_JSP                      3

/* CPU control register bits mask */
#define DBG_CPU_CR_STALL    0x010000
#define DBG_CPU_CR_STEP     0x000001
#define DBG_CPU_CR_RESET    0x010000

/* Polynomial for the CRC calculation
 * Yes, it's backwards.  Yes, this is on purpose.
 * The hardware is designed this way to save on logic and routing,
 * and it's really all the same to us here.
 */
#define ADBG_CRC_POLY      0xedb88320

/* These are for the internal registers in the Wishbone module
 * The first is the length of the index register,
 * the indexes of the various registers are defined after that.
 */
#define DBG_WB_REG_SEL_LEN      1
#define DBG_WB_REG_ERROR        0

/* Opcode definitions for the Wishbone module. */
#define DBG_WB_OPCODE_LEN       4
#define DBG_WB_CMD_NOP          0x0
#define DBG_WB_CMD_BWRITE8      0x1
#define DBG_WB_CMD_BWRITE16     0x2
#define DBG_WB_CMD_BWRITE32     0x3
#define DBG_WB_CMD_BREAD8       0x5
#define DBG_WB_CMD_BREAD16      0x6
#define DBG_WB_CMD_BREAD32      0x7
#define DBG_WB_CMD_IREG_WR      0x9
#define DBG_WB_CMD_IREG_SEL     0xd

/* Internal register definitions for the CPU0 module. */
#define DBG_CPU0_REG_SEL_LEN    1
#define DBG_CPU0_REG_STATUS     0

#define DBG_BREAK_ENABLE                0x00000008

#define DEBUGGER_OFFSET                 0xE0000000
#define DEBUGGER_BREAK_OFFSET           0xE0000008
#define DEBUGGER_AUTH_OFFSET            0xE0000030
#define DEBUGGER_BREAKPOINT_OFFSET      0xE0000040

/* Opcode definitions for the first CPU module. */
#define DBG_CPU0_OPCODE_LEN     4
#define DBG_CPU0_CMD_NOP        0x0
#define DBG_CPU0_CMD_BWRITE32   0x3
#define DBG_CPU0_CMD_BREAD32    0x7
#define DBG_CPU0_CMD_IREG_WR    0x9
#define DBG_CPU0_CMD_IREG_SEL   0xd

/* Internal register definitions for the CPU1 module. */
#define DBG_CPU1_REG_SEL_LEN    1
#define DBG_CPU1_REG_STATUS     0

/* Opcode definitions for the second CPU module. */
#define DBG_CPU1_OPCODE_LEN     4
#define DBG_CPU1_CMD_NOP        0x0
#define DBG_CPU1_CMD_BWRITE32   0x3
#define DBG_CPU1_CMD_BREAD32    0x7
#define DBG_CPU1_CMD_IREG_WR    0x9
#define DBG_CPU1_CMD_IREG_SEL   0xd

#define MAX_READ_STATUS_WAIT    10
#define MAX_READ_BUSY_RETRY     6
#define MAX_READ_CRC_RETRY      2
#define MAX_WRITE_CRC_RETRY     6
#define BURST_READ_READY        1
#define MAX_BUS_ERRORS          2

/*#define MAX_BURST_SIZE      (1) */
#define MAX_BURST_SIZE          (4*1024)

#define STATUS_BYTES            4
#define CRC_LEN                 4

static int rvmax_auth_data_init;
uint32_t rvmax_auth_data[AUTH_LEN];
static struct rvmax_du rvmax_du_adv;

static int print_crc;
static int break_enabled;

static int adbg_wb_burst_write(struct rvmax_jtag *jtag_info,
	const uint8_t *data,
	int size,
	int count,
	unsigned long start_address);
static int adbg_wb_burst_read(struct rvmax_jtag *jtag_info, int size,
	int count, uint32_t start_address, uint8_t *data);

static uint32_t adbg_compute_crc(uint32_t crc, uint32_t data_in,
	int length_bits)
{
/*LOG_DEBUG("+"); */

	for (int i = 0; i < length_bits; i++) {
		uint32_t d, c;
		d = ((data_in >> i) & 0x1) ? 0xffffffff : 0;
		c = (crc & 0x1) ? 0xffffffff : 0;
		crc = crc >> 1;
		crc = crc ^ ((d ^ c) & ADBG_CRC_POLY);
	}

	if (print_crc)
		LOG_DEBUG("In[%d]: %08" PRIx32 " CRC: 0x%08" PRIx32, length_bits, data_in, crc);

	return crc;
}

static int find_status_bit(void *_buf, int len)
{
	int i = 0;
	int count = 0;
	int ret = -1;
	uint8_t *buf = _buf;
/*LOG_DEBUG("+"); */

	while (!(buf[i] & (1 << count++)) && (i < len)) {
		if (count == 8) {
			count = 0;
			i++;
		}
	}

	if (i < len)
		ret = (i * 8) + count;

	return ret;
}

static int rvmax_adv_jtag_init(struct rvmax_jtag *jtag_info)
{
	struct rvmax_tap_ip *tap_ip = jtag_info->tap_ip;
	int i;

	int retval = tap_ip->init(jtag_info);
/*LOG_DEBUG("+"); */
	if (retval != ERROR_OK) {
		LOG_ERROR("TAP initialization failed");
		return retval;
	}

	/* TAP is now configured to communicate with debug interface */
	jtag_info->rvmax_jtag_inited = 1;

	/* TAP reset - not sure what state debug module chain is in now */
	jtag_info->rvmax_jtag_module_selected = DC_NONE;

	jtag_info->current_reg_idx = malloc(DBG_MAX_MODULES * sizeof(uint8_t));
	memset(jtag_info->current_reg_idx, 0, DBG_MAX_MODULES * sizeof(uint8_t));

	if (rvmax_du_adv.options & ADBG_USE_HISPEED)
		LOG_DEBUG("adv debug unit is configured with option ADBG_USE_HISPEED");

	if (!rvmax_auth_data_init) {
		LOG_DEBUG("Sending auth info");
		jtag_info->rvmax_jtag_module_selected = DC_CPU0;

		for (i = 0; i < AUTH_LEN; i++) {
			retval = adbg_wb_burst_write(jtag_info,
					(const uint8_t *)&rvmax_auth_data[i],
					4,
					1,
					DEBUGGER_AUTH_OFFSET);

			if (retval != ERROR_OK) {
				LOG_ERROR("Auth initialization failed");
				jtag_info->rvmax_jtag_module_selected = DC_NONE;
				return retval;
			}
		}

		/*rvmax_auth_data_init = 1; */
		LOG_DEBUG("Auth data sent");
	} else
		LOG_DEBUG(" auth info already sent");
	/*LOG_DEBUG("Init done"); */

	return ERROR_OK;

}

/* Selects one of the modules in the debug unit
 * (e.g. wishbone unit, CPU0, etc.)
 */
static int adbg_select_module(struct rvmax_jtag *jtag_info, int chain)
{
	if (jtag_info->rvmax_jtag_module_selected == chain)
		return ERROR_OK;
	/* MSB of the data out must be set to 1, indicating a module
	 * select command
	 */
	uint8_t data = chain | (1 << DBG_MODULE_SELECT_REG_SIZE);
	struct scan_field field;

	field.num_bits = (DBG_MODULE_SELECT_REG_SIZE + 1);
	field.out_value = &data;
	field.in_value = NULL;
	jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();

	if (retval != ERROR_OK)
		return retval;

	jtag_info->rvmax_jtag_module_selected = chain;

	return ERROR_OK;
}

/* Write control register (internal to the debug unit) */
static int adbg_ctrl_write(struct rvmax_jtag *jtag_info, uint8_t regidx,
	uint32_t *cmd_data, int length_bits)
{

	uint32_t value;
	int retval = 0;

	LOG_DEBUG("ctrlW: %08x", cmd_data[0]);

	value = cmd_data[0];
	retval = adbg_wb_burst_write(jtag_info, (const uint8_t *)&value, 4, 1, DEBUGGER_OFFSET);
	return retval;
}
static int adbg_ctrl_read(struct rvmax_jtag *jtag_info, uint32_t regidx,
	uint32_t *data, int length_bits)
{
	int retval;
	/*LOG_DEBUG("+"); */
	retval = adbg_wb_burst_read(jtag_info, 4, 1, DEBUGGER_OFFSET, (uint8_t *)data);
	return retval;
}
/* sends out a burst command to the selected module in the debug unit (MSB to LSB):
 * 1-bit module command
 * 4-bit opcode
 * 32-bit address
 * 16-bit length (of the burst, in words)
 */
static int adbg_burst_command(struct rvmax_jtag *jtag_info, uint32_t opcode,
	uint32_t address, uint16_t length_words)
{
	uint32_t data[2];

/*LOG_DEBUG("+"); */
	/* Set up the data */
	data[0] = length_words | (address << 16);
	/* MSB must be 0 to access modules */
	data[1] = ((address >> 16) | ((opcode & 0xf) << 16)) & ~(0x1 << 20);
	struct scan_field field;

	field.num_bits = 53;
	field.out_value = (uint8_t *)&data[0];
	field.in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

	return jtag_execute_queue();
}

static int adbg_wb_burst_read(struct rvmax_jtag *jtag_info, int size,
	int count, uint32_t start_address, uint8_t *data)
{
	int retry_full_crc = 0;
	int retry_full_busy = 0;
	int retval;
	uint8_t opcode;

	if ((start_address & DEBUGGER_OFFSET) != DEBUGGER_OFFSET)
		LOG_DEBUG(
			"Doing burst read (%d), word size %d, word count %d, start address 0x%08" PRIx32,
			jtag_info->rvmax_jtag_module_selected,
			size,
			count,
			start_address);
	else {
		/*LOG_DEBUG("+%d,%d",size,jtag_info->rvmax_jtag_module_selected); */
	}
	/* Select the appropriate opcode */
	switch (jtag_info->rvmax_jtag_module_selected) {
		case DC_WISHBONE:
/*LOG_DEBUG("DC_WISHBONE"); */
			if (size == 1)
				opcode = DBG_WB_CMD_BREAD8;
			else if (size == 2)
				opcode = DBG_WB_CMD_BREAD16;
			else if (size == 4)
				opcode = DBG_WB_CMD_BREAD32;
			else {
				LOG_WARNING("Tried burst read with invalid word size (%d),"
					"defaulting to 4-byte words", size);
				opcode = DBG_WB_CMD_BREAD32;
			}
			break;
		case DC_CPU0:
/*LOG_DEBUG("DC_CPU0"); */
			if (size == 4)
				opcode = DBG_CPU0_CMD_BREAD32;
			else {
				LOG_WARNING("Tried burst read with invalid word size (%d),"
					"defaulting to 4-byte words", size);
				opcode = DBG_CPU0_CMD_BREAD32;
			}
			break;
		case DC_CPU1:
/*LOG_DEBUG("DC_CPU1"); */
			if (size == 4)
				opcode = DBG_CPU1_CMD_BREAD32;
			else {
				LOG_WARNING("Tried burst read with invalid word size (%d),"
					"defaulting to 4-byte words", size);
				opcode = DBG_CPU0_CMD_BREAD32;
			}
			break;
		default:
			LOG_ERROR("Illegal debug chain selected (%i) while doing burst read",
				jtag_info->rvmax_jtag_module_selected);
			return ERROR_FAIL;
	}

	int total_size_bytes = count * size;
	struct scan_field field[3];
	uint8_t *in_buffer = malloc(total_size_bytes + CRC_LEN + STATUS_BYTES);

retry_read_full:

	/* Send the BURST READ command, returns TAP to idle state */
	retval = adbg_burst_command(jtag_info, opcode, start_address, count);
	if (retval != ERROR_OK)
		goto out;

	field[0].num_bits = (total_size_bytes + CRC_LEN + STATUS_BYTES) * 8;

	if ((start_address & DEBUGGER_OFFSET) != DEBUGGER_OFFSET)
		LOG_DEBUG("num_bits: %d", field[0].num_bits);

	field[0].out_value = NULL;
	field[0].in_value = in_buffer;

	field[1].num_bits = CRC_LEN*8;
	field[1].out_value = NULL;
	field[1].in_value = in_buffer+8;

	field[2].num_bits = (total_size_bytes + CRC_LEN + STATUS_BYTES) * 8;
	field[2].out_value = NULL;
	field[2].in_value = in_buffer+8;
	jtag_add_dr_scan(jtag_info->tap, 1, &field[0], TAP_IDLE);

	retval = jtag_execute_queue();

	if (retval != ERROR_OK)
		goto out;

	/* Look for the start bit in the first (STATUS_BYTES * 8) bits */
	int shift = find_status_bit(in_buffer, STATUS_BYTES);

	/* We expect the status bit to be in the first byte */
	if (shift < 0) {
		if (retry_full_busy++ < MAX_READ_BUSY_RETRY) {
			LOG_WARNING("Burst read timed out");
			goto retry_read_full;
		} else {
			LOG_ERROR("Burst read failed");
			retval = ERROR_FAIL;
			goto out;
		}
	}

	buffer_shr(in_buffer, total_size_bytes + CRC_LEN + STATUS_BYTES, shift);

	uint32_t crc_read;
	memcpy(data, in_buffer, total_size_bytes);
	memcpy(&crc_read, &in_buffer[total_size_bytes], CRC_LEN);

	uint32_t crc_calc = 0xffffffff;
	for (int i = 0; i < total_size_bytes; i++)
		crc_calc = adbg_compute_crc(crc_calc, data[i], 8);

	if (crc_calc != crc_read) {
		LOG_WARNING("CRC ERROR! Computed 0x%08" PRIx32 ", read CRC 0x%08" PRIx32,
			crc_calc,
			crc_read);
		if (retry_full_crc++ < MAX_READ_CRC_RETRY)
			goto retry_read_full;
		else {
			LOG_ERROR("Burst read failed");
			retval = ERROR_FAIL;
			goto out;
		}
	} else {
		/*LOG_DEBUG("CRC OK!"); */
	}

	/* Now, read the error register, and retry/recompute as necessary */
	if (jtag_info->rvmax_jtag_module_selected == DC_WISHBONE &&
		!(rvmax_du_adv.options & ADBG_USE_HISPEED)) {

		uint32_t err_data[2] = {0, 0};
		uint32_t addr;
		int bus_error_retries = 0;

		/* First, just get 1 bit...read address only if necessary */
		retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 1);

		if (retval != ERROR_OK)
			goto out;

		/* Then we have a problem */
		if (err_data[0] & 0x1) {

			retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 33);
			if (retval != ERROR_OK)
				goto out;

			addr = (err_data[0] >> 1) | (err_data[1] << 31);
			LOG_WARNING(
				"WB bus error during burst read, address 0x%08" PRIx32 ", retrying!",
				addr);

			bus_error_retries++;

			if (bus_error_retries > MAX_BUS_ERRORS) {
				LOG_ERROR("Max WB bus errors reached during burst read");
				retval = ERROR_FAIL;
				goto out;
			}

			/* Don't call retry_do(), a JTAG reset won't help a WB bus error
			 * Write 1 bit, to reset the error register*/
			err_data[0] = 1;
			retval = adbg_ctrl_write(jtag_info, DBG_WB_REG_ERROR, err_data, 1);

			if (retval != ERROR_OK)
				goto out;

			goto retry_read_full;
		}
	}

out:
	free(in_buffer);
	if ((start_address & DEBUGGER_OFFSET) != DEBUGGER_OFFSET)
		LOG_DEBUG("\nData32: 0x%08x", data[0]|(data[1]<<8)|(data[2]<<16)|(data[3]<<24));
	return retval;
}

/* Set up and execute a burst write to a contiguous set of addresses */
static int adbg_wb_burst_write(struct rvmax_jtag *jtag_info,
	const uint8_t *data,
	int size,
	int count,
	unsigned long start_address)
{
	int retry_full_crc = 0;
	int retval;
	uint8_t opcode;

	uint32_t data32;
	memcpy(&data32, data, sizeof(uint32_t));
	/* LOG_DEBUG("Doing burst write, word size %d, word count %d," "start address 0x%08lx, data
	 *%08x", size, count, start_address,(*((uint32_t*)data))&((1<<size*8)-1));
	 * LOG_DEBUG("%08x",*((uint32_t*)data));
	 * Select the appropriate opcode*/
	switch (jtag_info->rvmax_jtag_module_selected) {
		case DC_WISHBONE:
			if (size == 1)
				opcode = DBG_WB_CMD_BWRITE8;
			else if (size == 2)
				opcode = DBG_WB_CMD_BWRITE16;
			else if (size == 4)
				opcode = DBG_WB_CMD_BWRITE32;
			else {
				/*LOG_DEBUG("Tried WB burst write with invalid word size (%d),"
				 *  "defaulting to 4-byte words", size); */
				opcode = DBG_WB_CMD_BWRITE32;
			}
			break;
		case DC_CPU0:
			if (size == 4)
				opcode = DBG_CPU0_CMD_BWRITE32;
			else {
				/*LOG_DEBUG("Tried CPU0 burst write with invalid word size (%d),"
				 *  "defaulting to 4-byte words", size); */
				opcode = DBG_CPU0_CMD_BWRITE32;
			}
			break;
		case DC_CPU1:
			if (size == 4)
				opcode = DBG_CPU1_CMD_BWRITE32;
			else {
				/*LOG_DEBUG("Tried CPU1 burst write with invalid word size (%d),"
				 *  "defaulting to 4-byte words", size); */
				opcode = DBG_CPU0_CMD_BWRITE32;
			}
			break;
		default:
			LOG_ERROR("Illegal debug chain selected (%i) while doing burst write",
				jtag_info->rvmax_jtag_module_selected);
			return ERROR_FAIL;
	}

retry_full_write:

	/* Send the BURST WRITE command, returns TAP to idle state */
	retval = adbg_burst_command(jtag_info, opcode, start_address, count);
	if (retval != ERROR_OK)
		return retval;

	struct scan_field field[3];

	/* Write a start bit so it knows when to start counting */
	uint8_t value = 1;
	field[0].num_bits = 1;
	field[0].out_value = &value;
	field[0].in_value = NULL;

	uint32_t crc_calc = 0xffffffff;
	print_crc = 1;
	for (int i = 0; i < (count * size); i++)
		crc_calc = adbg_compute_crc(crc_calc, data[i], 8);
	print_crc = 0;

	field[1].num_bits = count * size * 8;
	field[1].out_value = data;
	field[1].in_value = NULL;

	field[2].num_bits = CRC_LEN * 8;
	field[2].out_value = (uint8_t *)&crc_calc;
	field[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, field, TAP_DRSHIFT);

	/* Read the 'CRC match' bit, and go to idle */
	field[0].num_bits = 1;
	field[0].out_value = NULL;
	field[0].in_value = &value;
	jtag_add_dr_scan(jtag_info->tap, 1, field, TAP_IDLE);

	retval = jtag_execute_queue();

	if (retval != ERROR_OK)
		return retval;

	if (!value) {
		LOG_WARNING(
			"CRC ERROR! match bit after write is %" PRIi8 " (computed CRC 0x%08" PRIx32 ")",
			value,
			crc_calc);
		if (retry_full_crc++ < MAX_WRITE_CRC_RETRY)
			goto retry_full_write;
		else {
			LOG_ERROR(
				"CRC ERROR!!!!! match bit after write is %" PRIi8 " (computed CRC 0x%08" PRIx32 ")",
				value,
				crc_calc);
			/*FIXME   return ERROR_FAIL; */
		}
	} else {
		/*LOG_DEBUG("CRC OK!\n"); */
	}

	/* Now, read the error register, and retry/recompute as necessary */
	if (jtag_info->rvmax_jtag_module_selected == DC_WISHBONE &&
		!(rvmax_du_adv.options & ADBG_USE_HISPEED)) {
		uint32_t addr;
		int bus_error_retries = 0;
		uint32_t err_data[2] = {0, 0};

		/* First, just get 1 bit...read address only if necessary */
		retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 1);

		if (retval != ERROR_OK) {
			LOG_DEBUG("e1");
			return retval;
		}

		/* Then we have a problem */
		if (err_data[0] & 0x1) {

			retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 33);

			if (retval != ERROR_OK) {
				LOG_DEBUG("e2");
				return retval;
			}

			addr = (err_data[0] >> 1) | (err_data[1] << 31);
			LOG_WARNING(
				"WB bus error during burst write, address 0x%08" PRIx32 ", retrying!",
				addr);

			bus_error_retries++;

			if (bus_error_retries > MAX_BUS_ERRORS) {
				LOG_ERROR("Max WB bus errors reached during burst read");
				retval = ERROR_FAIL;
				return retval;
			}

			/* Don't call retry_do(), a JTAG reset won't help a WB bus error
			 * Write 1 bit, to reset the error register*/
			err_data[0] = 1;
			retval = adbg_ctrl_write(jtag_info, DBG_WB_REG_ERROR, err_data, 1);

			if (retval != ERROR_OK)
				return retval;

			goto retry_full_write;
		}
	}
	return ERROR_OK;
}

/* Currently hard set in functions to 32-bits */
static int rvmax_adv_jtag_read_cpu(struct rvmax_jtag *jtag_info,
	uint32_t addr, int count, uint32_t *value)
{
	int retval;
	uint32_t temp;

	/* distinguish riscv core registers from APB/AHB based registers */
	if ((addr & 0xF0000000) == 0x00000000)
		addr |= DEBUGGER_OFFSET;

	LOG_DEBUG("+ %d %08x", count, addr);

	if (!jtag_info->rvmax_jtag_inited) {
		retval = rvmax_adv_jtag_init(jtag_info);

		if (retval != ERROR_OK)
			return retval;
	}

	retval = adbg_select_module(jtag_info, DC_CPU0);

	if (retval != ERROR_OK)
		return retval;

	temp = adbg_wb_burst_read(jtag_info, 4, count, addr, (uint8_t *)value);
	LOG_DEBUG("%08x", *value);
	return temp;
}

static int rvmax_adv_jtag_write_cpu(struct rvmax_jtag *jtag_info,
	uint32_t addr, int count, const uint32_t *value)
{
	int retval;
	volatile uint32_t temp[8];
	uint32_t cpu_addr = addr;

	/* distinguish riscv core registers from APB/AHB based registers */
	if ((addr & 0xF0000000) == 0x00000000)
		cpu_addr = addr | DEBUGGER_OFFSET;

	LOG_DEBUG("+ %08x %08x %08x", cpu_addr, count, *value);

	if (!jtag_info->rvmax_jtag_inited) {
		retval = rvmax_adv_jtag_init(jtag_info);

		if (retval != ERROR_OK)
			return retval;
	}

	retval = adbg_select_module(jtag_info, DC_CPU0);

	if (retval != ERROR_OK)
		return retval;

	adbg_wb_burst_read(jtag_info, 4, count, cpu_addr, (uint8_t *)temp);
	/*LOG_DEBUG("%08x %08x %08x", temp[0],cpu_addr,*value); */
	retval = adbg_wb_burst_write(jtag_info, (uint8_t *)value, 4, count, cpu_addr);
	adbg_wb_burst_read(jtag_info, 4, count, cpu_addr, (uint8_t *)temp);
	/*LOG_DEBUG("%08x %08x", temp[0],cpu_addr); */
	return retval;
}

static int rvmax_adv_cpu_stall(struct rvmax_jtag *jtag_info, int action)
{
	uint32_t value;
	int retval = 0;
	/*LOG_DEBUG("+"); */
	if (!jtag_info->rvmax_jtag_inited) {
		retval = rvmax_adv_jtag_init(jtag_info);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = adbg_select_module(jtag_info, DC_CPU0);

	if (retval != ERROR_OK)
		return retval;

	uint32_t cpu_cr = 0;
	uint32_t rcpu_cr = 2;
	retval = adbg_ctrl_read(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);

	if (retval != ERROR_OK)
		return retval;

	if (action == CPU_STALL)
		cpu_cr = DBG_CPU_CR_STALL;
	else if (action == CPU_STEP)
		cpu_cr = DBG_CPU_CR_STEP;
	else if (action == CPU_UNSTALL) {
		adbg_wb_burst_read(jtag_info, 4, 1, DEBUGGER_BREAK_OFFSET, (uint8_t *)&value);

		if (value & DBG_BREAK_ENABLE)
			break_enabled = 1;
		else
			break_enabled = 0;

		if (!break_enabled) {
			value = DBG_BREAK_ENABLE;
			adbg_wb_burst_write(jtag_info,
				(const uint8_t *)&value,
				4,
				1,
				DEBUGGER_BREAK_OFFSET);
			adbg_wb_burst_read(jtag_info, 4, 1, DEBUGGER_BREAK_OFFSET,
				(uint8_t *)&value);
		}

		cpu_cr = 0;
	}

	retval = adbg_select_module(jtag_info, DC_CPU0);

	if (retval != ERROR_OK)
		return retval;

	retval = adbg_ctrl_write(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);
	rcpu_cr = 0;
	adbg_ctrl_read(jtag_info, DBG_CPU0_REG_STATUS, &rcpu_cr, 2);
	LOG_DEBUG("+d %x", rcpu_cr);


	if (rcpu_cr != cpu_cr) {
		rcpu_cr = 0;
		adbg_wb_burst_read(jtag_info, 4, 1, DEBUGGER_OFFSET, (uint8_t *)&rcpu_cr);
		LOG_DEBUG("+e %x", rcpu_cr);
		rcpu_cr = 0;
		adbg_wb_burst_read(jtag_info, 4, 1, DEBUGGER_OFFSET+4, (uint8_t *)&rcpu_cr);
		LOG_DEBUG("+f %x", rcpu_cr);
		rcpu_cr = 0;
		adbg_wb_burst_read(jtag_info, 4, 1, DEBUGGER_OFFSET+8, (uint8_t *)&rcpu_cr);
		LOG_DEBUG("+g %x", rcpu_cr);
		rcpu_cr = 0;
		adbg_wb_burst_read(jtag_info, 4, 1, DEBUGGER_OFFSET+0x0c, (uint8_t *)&rcpu_cr);
		LOG_DEBUG("+h %x", rcpu_cr);
	}
	return retval;
}

static int rvmax_adv_is_cpu_running(struct rvmax_jtag *jtag_info, int *running)
{
	int retval;
/*LOG_DEBUG("+"); */
	if (!jtag_info->rvmax_jtag_inited) {
		retval = rvmax_adv_jtag_init(jtag_info);

		if (retval != ERROR_OK)
			return retval;
	}

	int current = jtag_info->rvmax_jtag_module_selected;

	retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	uint32_t cpu_cr = 0;
	retval = adbg_ctrl_read(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);

	if (retval != ERROR_OK)
		return retval;

	if (cpu_cr & DBG_CPU_CR_STALL)
		*running = 0;
	else
		*running = 1;

	if (current != DC_NONE) {
		retval = adbg_select_module(jtag_info, current);

		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int rvmax_adv_cpu_reset(struct rvmax_jtag *jtag_info, int action)
{
	int retval = ERROR_OK;

	if (action == CPU_RESET)
		retval = rvmax_adv_jtag_init(jtag_info);

	if (retval != ERROR_OK)
		return retval;

	retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	uint32_t cpu_cr = 0;
	retval = adbg_ctrl_read(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);

	if (retval != ERROR_OK)
		return retval;

	if (action == CPU_RESET)
		cpu_cr |= DBG_CPU_CR_RESET;
	else
		cpu_cr &= ~DBG_CPU_CR_RESET;

	retval = adbg_select_module(jtag_info, DC_CPU0);

	if (retval != ERROR_OK)
		return retval;

	return adbg_ctrl_write(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);
}

static int rvmax_adv_jtag_read_memory(struct rvmax_jtag *jtag_info,
	uint32_t addr, uint32_t size, int count, uint8_t *buffer)
{
	/*LOG_DEBUG("Reading WB%" PRId32 " at 0x%08" PRIx32, size * 8, addr); */

	int retval;
	if (!jtag_info->rvmax_jtag_inited) {
		retval = rvmax_adv_jtag_init(jtag_info);

		if (retval != ERROR_OK)
			return retval;
	}

	retval = adbg_select_module(jtag_info, DC_WISHBONE);

	if (retval != ERROR_OK)
		return retval;

	int block_count_left = count;
	uint32_t block_count_address = addr;
	uint8_t *block_count_buffer = buffer;

	while (block_count_left) {

		int blocks_this_round = (block_count_left > MAX_BURST_SIZE) ?
			MAX_BURST_SIZE : block_count_left;

		retval = adbg_wb_burst_read(jtag_info, size, blocks_this_round,
				block_count_address, block_count_buffer);

		if (retval != ERROR_OK)
			return retval;

		block_count_left -= blocks_this_round;
		block_count_address += size * MAX_BURST_SIZE;
		block_count_buffer += size * MAX_BURST_SIZE;
	}

	/* The adv_debug_if always return words and half words in
	 * little-endian order no matter what the target endian is.
	 * So if the target endian is big, change the order.
	 */

	struct target *target = jtag_info->target;
	if ((target->endianness == TARGET_BIG_ENDIAN) && (size != 1)) {
		switch (size) {
			case 4:
				buf_bswap32(buffer, buffer, size * count);
				break;
			case 2:
				buf_bswap16(buffer, buffer, size * count);
				break;
		}
	}

	retval = adbg_select_module(jtag_info, DC_CPU0);
	return ERROR_OK;
}

static int rvmax_adv_jtag_write_memory(struct rvmax_jtag *jtag_info,
	uint32_t addr, uint32_t size, int count, const uint8_t *buffer)
{
	LOG_DEBUG("Writing WB%" PRId32 " at 0x%08" PRIx32, size * 8, addr);

	int retval;
	if (!jtag_info->rvmax_jtag_inited) {
		retval = rvmax_adv_jtag_init(jtag_info);

		if (retval != ERROR_OK) {
			LOG_DEBUG("NOT ERROR_OK1");
			return retval;
		}
	}

	retval = adbg_select_module(jtag_info, DC_WISHBONE);

	if (retval != ERROR_OK) {
		LOG_DEBUG("NOT ERROR_OK2");
		return retval;
	}

	/* The adv_debug_if wants words and half words in little-endian
	 * order no matter what the target endian is. So if the target
	 * endian is big, change the order.
	 */

	void *t = NULL;
	struct target *target = jtag_info->target;
	if ((target->endianness == TARGET_BIG_ENDIAN) && (size != 1)) {
		t = malloc(count * size * sizeof(uint8_t));
		if (t == NULL) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}

		switch (size) {
			case 4:
				buf_bswap32(t, buffer, size * count);
				break;
			case 2:
				buf_bswap16(t, buffer, size * count);
				break;
		}
		buffer = t;
	}

	int block_count_left = count;
	uint32_t block_count_address = addr;
	uint8_t *block_count_buffer = (uint8_t *)buffer;
	while (block_count_left) {

		int blocks_this_round = (block_count_left > MAX_BURST_SIZE) ?
			MAX_BURST_SIZE : block_count_left;

		retval = adbg_wb_burst_write(jtag_info, block_count_buffer,
				size, blocks_this_round,
				block_count_address);

		if (retval != ERROR_OK) {
			if (t != NULL)
				free(t);
			LOG_DEBUG("NOT ERROR_OK3");
			LOG_DEBUG("Writing WB%" PRId32 " at 0x%08" PRIx32,
				blocks_this_round,
				block_count_address);
			return retval;
		}

		block_count_left -= blocks_this_round;
		block_count_address += size * MAX_BURST_SIZE;
		block_count_buffer += size * MAX_BURST_SIZE;
	}

	if (t != NULL)
		free(t);

	LOG_DEBUG("ERROR_OK");
	return ERROR_OK;
}

static struct rvmax_du rvmax_du_adv = {
	.name                      = "adv",
	.options                   = ADBG_USE_HISPEED,
	.rvmax_jtag_init           = rvmax_adv_jtag_init,

	.rvmax_is_cpu_running      = rvmax_adv_is_cpu_running,
	.rvmax_cpu_stall           = rvmax_adv_cpu_stall,
	.rvmax_cpu_reset           = rvmax_adv_cpu_reset,

	.rvmax_jtag_read_cpu       = rvmax_adv_jtag_read_cpu,
	.rvmax_jtag_write_cpu      = rvmax_adv_jtag_write_cpu,

	.rvmax_jtag_read_memory    = rvmax_adv_jtag_read_memory,
	.rvmax_jtag_write_memory   = rvmax_adv_jtag_write_memory
};

int rvmax_du_adv_register(void)
{
/*LOG_DEBUG("+"); */
	list_add_tail(&rvmax_du_adv.list, &rm_du_list);
	return 0;
}

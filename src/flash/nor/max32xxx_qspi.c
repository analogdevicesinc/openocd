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

/* STM QuadSPI (QSPI) and OctoSPI (OCTOSPI) controller are SPI bus controllers
 * specifically designed for SPI memories.
 * Two working modes are available:
 * - indirect mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus.
 * - memory mapped mode: the SPI is under QSPI/OCTOSPI control. Memory content
 *   is directly accessible in CPU memory space. CPU can read and execute from
 *   memory (but not write to) */

/* ATTENTION:
 * To have flash mapped in CPU memory space, the QSPI/OCTOSPI controller
 * has to be in "memory mapped mode". This requires following constraints:
 * 1) The command "reset init" has to initialize QSPI/OCTOSPI controller and put
 *    it in memory mapped mode;
 * 2) every command in this file has to return to prompt in memory mapped mode. */

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

struct max32xxx_qspi_flash_bank {
	bool probed;
	char devname[32];
	bool octo;
	struct flash_device dev;
	uint32_t io_base;
	uint32_t saved_cr;	/* in particular FSEL, DFM bit mask in QUADSPI_CR *AND* OCTOSPI_CR */
	uint32_t saved_ccr; /* different meaning for QUADSPI and OCTOSPI */
	uint32_t saved_tcr;	/* only for OCTOSPI */
	uint32_t saved_ir;	/* only for OCTOSPI */
	unsigned int sfdp_dummy1;	/* number of dummy bytes for SFDP read for flash1 and octo */
	unsigned int sfdp_dummy2;	/* number of dummy bytes for SFDP read for flash2 */
};


FLASH_BANK_COMMAND_HANDLER(max32xxx_qspi_flash_bank_command)
{
	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_qspi_handle_mass_erase_command)
{
	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_qspi_handle_set)
{
	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_qspi_handle_cmd)
{
	return ERROR_OK;
}

static int max32xxx_qspi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	return ERROR_OK;
}

static int max32xxx_qspi_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	return ERROR_OK;
}

/* Check whether flash is blank */
static int max32xxx_qspi_blank_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int max32xxx_qspi_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	return ERROR_OK;
}

static int max32xxx_qspi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	return ERROR_OK;
}

static int max32xxx_qspi_verify(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	return ERROR_OK;
}

static int max32xxx_qspi_probe(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int max32xxx_qspi_auto_probe(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int max32xxx_qspi_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int get_max32xxx_qspi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	return ERROR_OK;
}

static const struct command_registration max32xxx_qspi_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = max32xxx_qspi_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	{
		.name = "set",
		.handler = max32xxx_qspi_handle_set,
		.mode = COMMAND_EXEC,
		.usage = "bank_id name chip_size page_size read_cmd qread_cmd pprg_cmd "
			"[ mass_erase_cmd ] [ sector_size sector_erase_cmd ]",
		.help = "Set params of single flash chip",
	},
	{
		.name = "cmd",
		.handler = max32xxx_qspi_handle_cmd,
		.mode = COMMAND_EXEC,
		.usage = "bank_id num_resp cmd_byte ...",
		.help = "Send low-level command cmd_byte and following bytes or read num_resp.",
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
	.read = max32xxx_qspi_read,
	.verify = max32xxx_qspi_verify,
	.probe = max32xxx_qspi_probe,
	.auto_probe = max32xxx_qspi_auto_probe,
	.erase_check = max32xxx_qspi_blank_check,
	.protect_check = max32xxx_qspi_protect_check,
	.info = get_max32xxx_qspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2019, Ampere Computing LLC                              *
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

#ifndef OPENOCD_TARGET_ARM_ADI_V5_H
#define OPENOCD_TARGET_ARM_ADI_V5_H

/**
 * @file
 * This defines formats and data structures used to talk to ADIv5 entities.
 * Those include a DAP, different types of Debug Port (DP), and memory mapped
 * resources accessed through a MEM-AP.
 */

#include <helper/list.h>


/* MEM-AP register addresses */
#define MEM_AP_REG_CSW		0x00
#define MEM_AP_REG_TAR		0x04
#define MEM_AP_REG_TAR_UPPER	0x08		/* RW: Large Physical Address Extension */
#define MEM_AP_REG_DRW		0x0C		/* RW: Data Read/Write register */
#define MEM_AP_REG_BD0		0x10		/* RW: Banked Data register 0-3 */
#define MEM_AP_REG_BD1		0x14
#define MEM_AP_REG_BD2		0x18
#define MEM_AP_REG_BD3		0x1C
#define MEM_AP_REG_MBT		0x20		/* --: Memory Barrier Transfer register */
#define MEM_AP_REG_BASE_UPPER	0xF0		/* RO: Debug Base Address (LA) register */
#define MEM_AP_REG_CFG		0xF4		/* RO: Configuration register */
#define MEM_AP_REG_BASE		0xF8		/* RO: Debug Base Address register */
/* Generic AP register address */
#define AP_REG_IDR			0xFC		/* RO: Identification Register */

#endif /* OPENOCD_TARGET_ARM_ADI_V5_H */

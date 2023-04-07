/* SPDX-License-Identifier: GPL-2.0-or-later */
/***************************************************************************
 *   Copyright (C) 2013 Franck Jullien                                     *
 *   elec4fun@gmail.com                                                    *
 *                                                                         *
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

#ifndef OPENOCD_TARGET_OPENRISC_RVMAX_DU_H
#define OPENOCD_TARGET_OPENRISC_RVMAX_DU_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define CPU_STALL	0
#define CPU_UNSTALL	1
#define CPU_STEP	2

#define CPU_RESET	0
#define CPU_NOT_RESET	1

#define DBG_BREAK_ENABLE       0x00000008

#define DBG_MAX_HWBREAKPOINTS    8
#define DBG_HWBREAKPOINT_AVAIL   1
#define DBG_HWBREAKPOINT_ENABLED 2

#define DEBUGGER_OFFSET                 0xE0000000
#define DEBUGGER_BREAK_OFFSET           0xE0000008
#define DEBUGGER_AUTH_OFFSET            0xE0000030
#define DEBUGGER_HWBREAKPOINT_CONTROL_OFFSET   0xE0000040
#define DEBUGGER_HWBREAKPOINT_DATA_OFFSET      0xE0000044

int rvmax_du_adv_register(void);

/* Linear list over all available rvmax debug unit */
extern struct list_head rm_du_list;

struct rvmax_du {
	const char *name;
	struct list_head list;
	int options;

	int (*rvmax_jtag_init)(struct rvmax_jtag *jtag_info);

	int (*rvmax_is_cpu_running)(struct rvmax_jtag *jtag_info, int *running);

	int (*rvmax_cpu_stall)(struct rvmax_jtag *jtag_info, int action);

	int (*rvmax_cpu_reset)(struct rvmax_jtag *jtag_info, int action);

	int (*rvmax_jtag_read_cpu)(struct rvmax_jtag *jtag_info,
				  uint32_t addr, int count, uint32_t *value);

	int (*rvmax_jtag_write_cpu)(struct rvmax_jtag *jtag_info,
				   uint32_t addr, int count, const uint32_t *value);

	int (*rvmax_jtag_read_memory)(struct rvmax_jtag *jtag_info, uint32_t addr, uint32_t size,
					int count, uint8_t *buffer);

	int (*rvmax_jtag_write_memory)(struct rvmax_jtag *jtag_info, uint32_t addr, uint32_t size,
					 int count, const uint8_t *buffer);
};

static inline struct rvmax_du *rvmax_jtag_to_du(struct rvmax_jtag *jtag_info)
{
	return (struct rvmax_du *)jtag_info->du_core;
}

static inline struct rvmax_du *rvmax_to_du(struct rvmax_common *rvmax)
{
	struct rvmax_jtag *jtag = &rvmax->jtag;
	return (struct rvmax_du *)jtag->du_core;
}
#endif /* OPENOCD_TARGET_OPENRISC_RVMAX_DU_H */

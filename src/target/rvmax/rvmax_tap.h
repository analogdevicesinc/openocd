/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2012 by Franck Jullien                                  *
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
 *   Copyright (C) 2016 by Maxim Integrated                                *
 *   Kevin Gillespie <kevin.gillespie@maximintegrated.com                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_OPENRISC_RVMAX_TAP_H
#define OPENOCD_TARGET_OPENRISC_RVMAX_TAP_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/list.h>
#include "rvmax.h"

int rvmax_tap_vjtag_register(void);
int rvmax_tap_xilinx_bscan_register(void);
int rvmax_tap_mohor_register(void);

/* Linear list over all available rvmax taps */
extern struct list_head rm_tap_list;

struct rvmax_tap_ip {
	struct list_head list;
	int (*init)(struct rvmax_jtag *jtag_info);
	const char *name;
};

#endif /* OPENOCD_TARGET_OPENRISC_RVMAX_TAP_H */

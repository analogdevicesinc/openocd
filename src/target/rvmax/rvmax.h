/***************************************************************************
 *   Copyright (C) 2011 by Julius Baxter                                   *
 *   julius@opencores.org                                                  *
 *                                                                         *
 *   Copyright (C) 2013 by Marek Czerski                                   *
 *   ma.czerski@gmail.com                                                  *
 *                                                                         *
 *   Copyright (C) 2013 by Franck Jullien                                  *
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

#ifndef OPENOCD_TARGET_OPENRISC_RVMAX_H
#define OPENOCD_TARGET_OPENRISC_RVMAX_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/target.h>
#include "opcodes.h"

/* SPR groups start address */
#define GROUP0		(0  << 11)
#define GROUP1		(1  << 11)
#define GROUP2		(2  << 11)
#define GROUP3		(3  << 11)
#define GROUP4		(4  << 11)
#define GROUP5		(5  << 11)
#define GROUP6		(6  << 11)
#define GROUP7		(7  << 11)
#define GROUP8		(8  << 11)
#define GROUP9		(9  << 11)
#define GROUP10		(10 << 11)

/* RVMAX registers */
enum rvmax_reg_nums {
	RVMAX_REG_R0 = 0,
	RVMAX_REG_R1,
	RVMAX_REG_R2,
	RVMAX_REG_R3,
	RVMAX_REG_R4,
	RVMAX_REG_R5,
	RVMAX_REG_R6,
	RVMAX_REG_R7,
	RVMAX_REG_R8,
	RVMAX_REG_R9,
	RVMAX_REG_R10,
	RVMAX_REG_R11,
	RVMAX_REG_R12,
	RVMAX_REG_R13,
	RVMAX_REG_R14,
	RVMAX_REG_R15,
	RVMAX_REG_R16,
	RVMAX_REG_R17,
	RVMAX_REG_R18,
	RVMAX_REG_R19,
	RVMAX_REG_R20,
	RVMAX_REG_R21,
	RVMAX_REG_R22,
	RVMAX_REG_R23,
	RVMAX_REG_R24,
	RVMAX_REG_R25,
	RVMAX_REG_R26,
	RVMAX_REG_R27,
	RVMAX_REG_R28,
	RVMAX_REG_R29,
	RVMAX_REG_R30,
	RVMAX_REG_R31,
	RVMAX_REG_NPC,
	RVMAXNUMCOREREGS
};

struct rvmax_jtag {
	struct jtag_tap *tap;
	int rvmax_jtag_inited;
	int rvmax_jtag_module_selected;
	uint8_t *current_reg_idx;
	struct rvmax_tap_ip *tap_ip;
	struct rvmax_du *du_core;
	struct target *target;
};

struct rvmax_common {
	struct rvmax_jtag jtag;
	struct reg_cache *core_cache;
	uint32_t core_regs[RVMAXNUMCOREREGS];
	int nb_regs;
	struct rvmax_core_reg *arch_info;
};

static inline struct rvmax_common *
target_to_rvmax(struct target *target)
{
	return (struct rvmax_common *)target->arch_info;
}

struct rvmax_core_reg {
	const char *name;
	uint32_t list_num;   /* Index in register cache */
	uint32_t spr_num;    /* Number in architecture's SPR space */
	struct target *target;
	struct rvmax_common *rvmax_common;
	const char *feature; /* feature name in XML tdesc file */
	const char *group;   /* register group in XML tdesc file */
};

struct rvmax_core_reg_init {
	const char *name;
	uint32_t spr_num;    /* Number in architecture's SPR space */
	const char *feature; /* feature name in XML tdesc file */
	const char *group;   /* register group in XML tdesc file */
};

#define AUTH_LEN 8

/* ORBIS32 Trap instruction */
/* ebreak */
#define RVMAX_TRAP_INSTR  0x00100073
/* c.ebreak */
#define RVMAX_TRAP_INSTRC  (uint16_t)0x9002

#define RVMAX_RAM_START   0x20000000
#define NO_SINGLE_STEP		0
#define SINGLE_STEP		1

/* RVMAX Instruction cache registers needed for invalidating instruction
 * memory during adding and removing breakpoints.
 */
#define RVMAX_ICBIR_CPU_REG_ADD ((4 << 11) + 2)             /* IC Block Invalidate Register 0x2002 */
#define RVMAX_PC_RESET_ADDR 0x00000080
#endif /* OPENOCD_TARGET_OPENRISC_RVMAX_H */

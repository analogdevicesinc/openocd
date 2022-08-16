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

#include <jtag/jtag.h>
#include <target/register.h>
#include <target/target.h>
#include "target/algorithm.h"
#include <target/breakpoints.h>
#include <target/target_type.h>
#include <helper/time_support.h>
#include <helper/fileio.h>
#include "rvmax_tap.h"
#include "rvmax.h"
#include "rvmax_du.h"
#include "gdb_regs.h"

#define RVMAX_VERSION_MAX 0
#define RVMAX_VERSION_MIN 10

#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

LIST_HEAD(rm_tap_list);
LIST_HEAD(rm_du_list);
extern uint32_t rvmax_auth_data[];
char rvmax_driver_info[256] = "";
static int rvmax_remove_breakpoint(struct target *target,
					struct breakpoint *breakpoint);

static int rvmax_read_core_reg(struct target *target, int num);
static int rvmax_write_core_reg(struct target *target, int num);
static int debugger_breakpoint_count = -1;
static struct rvmax_core_reg *rvmax_core_reg_list_arch_info;

/* core register set based on RISC-V spec */
static const struct rvmax_core_reg_init rvmax_init_reg_list[] = {
	{"zero"     , GROUP0 + 1024, "org.gnu.gdb.riscv.cpu", NULL},
	{ "ra"      , GROUP0 + 1025, "org.gnu.gdb.riscv.cpu", NULL},
	{ "sp"      , GROUP0 + 1026, "org.gnu.gdb.riscv.cpu", NULL},
	{ "gp"      , GROUP0 + 1027, "org.gnu.gdb.riscv.cpu", NULL},
	{ "tp"      , GROUP0 + 1028, "org.gnu.gdb.riscv.cpu", NULL},
	{ "t0"      , GROUP0 + 1029, "org.gnu.gdb.riscv.cpu", NULL},
	{ "t1"      , GROUP0 + 1030, "org.gnu.gdb.riscv.cpu", NULL},
	{ "t2"      , GROUP0 + 1031, "org.gnu.gdb.riscv.cpu", NULL},
	{ "fp"      , GROUP0 + 1032, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s1"      , GROUP0 + 1033, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a0"      , GROUP0 + 1034, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a1"      , GROUP0 + 1035, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a2"      , GROUP0 + 1036, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a3"      , GROUP0 + 1037, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a4"      , GROUP0 + 1038, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a5"      , GROUP0 + 1039, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a6"      , GROUP0 + 1040, "org.gnu.gdb.riscv.cpu", NULL},
	{ "a7"      , GROUP0 + 1041, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s2"      , GROUP0 + 1042, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s3"      , GROUP0 + 1043, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s4"      , GROUP0 + 1044, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s5"      , GROUP0 + 1045, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s6"      , GROUP0 + 1046, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s7"      , GROUP0 + 1047, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s8"      , GROUP0 + 1048, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s9"      , GROUP0 + 1049, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s10"     , GROUP0 + 1050, "org.gnu.gdb.riscv.cpu", NULL},
	{ "s11"     , GROUP0 + 1051, "org.gnu.gdb.riscv.cpu", NULL},
	{ "t3"      , GROUP0 + 1052, "org.gnu.gdb.riscv.cpu", NULL},
	{ "t4"      , GROUP0 + 1053, "org.gnu.gdb.riscv.cpu", NULL},
	{ "t5"      , GROUP0 + 1054, "org.gnu.gdb.riscv.cpu", NULL},
	{ "t6"      , GROUP0 + 1055, "org.gnu.gdb.riscv.cpu", NULL},
	{ "pc"      , GROUP0 + 0x2000, "org.gnu.gdb.riscv.cpu", NULL},
};

static int rvmax_add_reg(struct target *target, struct rvmax_core_reg *new_reg)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	int reg_list_size = rvmax->nb_regs * sizeof(struct rvmax_core_reg);

	rvmax_core_reg_list_arch_info = realloc(rvmax_core_reg_list_arch_info,
				reg_list_size + sizeof(struct rvmax_core_reg));

	memcpy(&rvmax_core_reg_list_arch_info[rvmax->nb_regs], new_reg,
		sizeof(struct rvmax_core_reg));

	rvmax_core_reg_list_arch_info[rvmax->nb_regs].list_num = rvmax->nb_regs;

	rvmax->nb_regs++;

	return ERROR_OK;
}

static int rvmax_create_reg_list(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);

	LOG_DEBUG("-");

	rvmax_core_reg_list_arch_info = malloc(ARRAY_SIZE(rvmax_init_reg_list) *
							 sizeof(struct rvmax_core_reg));

	for (int i = 0; i < (int)ARRAY_SIZE(rvmax_init_reg_list); i++) {
		rvmax_core_reg_list_arch_info[i].name = rvmax_init_reg_list[i].name;
		rvmax_core_reg_list_arch_info[i].spr_num = rvmax_init_reg_list[i].spr_num;
		rvmax_core_reg_list_arch_info[i].group = rvmax_init_reg_list[i].group;
		rvmax_core_reg_list_arch_info[i].feature = rvmax_init_reg_list[i].feature;
		rvmax_core_reg_list_arch_info[i].list_num = i;
		rvmax_core_reg_list_arch_info[i].target = NULL;
		rvmax_core_reg_list_arch_info[i].rvmax_common = NULL;
	}

	rvmax->nb_regs = ARRAY_SIZE(rvmax_init_reg_list);

	return ERROR_OK;
}

static int rvmax_jtag_read_regs(struct rvmax_common *rvmax, uint32_t *regs)
{
	struct rvmax_du *du_core = rvmax_jtag_to_du(&rvmax->jtag);
	/*int retval = ERROR_OK; */
	uint32_t reg_value;
	int i;

	LOG_DEBUG("-");

	if (debugger_breakpoint_count < 0) {
		for (i = 0; i < DBG_MAX_HWBREAKPOINTS; i++) {
			/* retval =  */
			du_core->rvmax_jtag_read_cpu(&rvmax->jtag, DEBUGGER_HWBREAKPOINT_CONTROL_OFFSET+i*8, 1, &reg_value);
			/*LOG_DEBUG("testing debug reg %d: %x",i,reg_value); */

			if (!(reg_value & DBG_HWBREAKPOINT_AVAIL))
				break;

			reg_value &= ~DBG_HWBREAKPOINT_ENABLED;
			du_core->rvmax_jtag_write_cpu(&rvmax->jtag, DEBUGGER_HWBREAKPOINT_CONTROL_OFFSET+i*8, 1, &reg_value);
			/*reg_value = 0xBA; */
			reg_value = 0x0;
			du_core->rvmax_jtag_write_cpu(&rvmax->jtag, DEBUGGER_HWBREAKPOINT_DATA_OFFSET+i*8, 1, &reg_value);
/*LOG_DEBUG("clearing debug reg %d: %x",i,reg_value); */
		}

		debugger_breakpoint_count = i;
	}


	return du_core->rvmax_jtag_read_cpu(&rvmax->jtag,
			rvmax->arch_info[RVMAX_REG_R0].spr_num, RVMAX_REG_R31 + 1,
			regs + RVMAX_REG_R0);
}

static int rvmax_jtag_write_regs(struct rvmax_common *rvmax, uint32_t *regs)
{
	struct rvmax_du *du_core = rvmax_jtag_to_du(&rvmax->jtag);

	LOG_DEBUG("-");

	return du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
			rvmax->arch_info[RVMAX_REG_R0].spr_num, RVMAX_REG_R31 + 1,
			&regs[RVMAX_REG_R0]);
}

static int rvmax_save_context(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int regs_read = 0;
	int retval;

	for (int i = 0; i < RVMAXNUMCOREREGS; i++) {
		if (!rvmax->core_cache->reg_list[i].valid) {
			if (i == RVMAX_REG_NPC) {
				retval = du_core->rvmax_jtag_read_cpu(&rvmax->jtag,
						rvmax->arch_info[i].spr_num, 1,
						&rvmax->core_regs[i]);

				if (retval != ERROR_OK)
					return retval;
			} else if (!regs_read) {
				/* read gpr registers at once (but only one time in this loop) */
				retval = rvmax_jtag_read_regs(rvmax, rvmax->core_regs);

				if (retval != ERROR_OK)
					return retval;
				/* prevent next reads in this loop */
				regs_read = 1;
			}
			/* We've just updated the core_reg[i], now update
				 the core cache */
			rvmax_read_core_reg(target, i);
		}
	}

	return ERROR_OK;
}

static int rvmax_restore_context(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int reg_write = 0;
	int retval;

	LOG_DEBUG("-");

	for (int i = 0; i < RVMAXNUMCOREREGS; i++) {
		if (rvmax->core_cache->reg_list[i].dirty) {
			rvmax_write_core_reg(target, i);

			if (i == RVMAX_REG_NPC) {
				retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
						rvmax->arch_info[i].spr_num, 1,
						&rvmax->core_regs[i]);

				if (retval != ERROR_OK) {
					LOG_ERROR("Error while restoring context");
					return retval;
				}
			} else
				reg_write = 1;
		}
	}

	if (reg_write) {
		/* read gpr registers at once (but only one time in this loop) */
		retval = rvmax_jtag_write_regs(rvmax, rvmax->core_regs);

		if (retval != ERROR_OK) {
			LOG_ERROR("Error while restoring context");
			return retval;
		}
	}

	return ERROR_OK;
}

static int rvmax_read_core_reg(struct target *target, int num)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	uint32_t reg_value;

	LOG_DEBUG("- %d", num);

	if ((num < 0) || (num >= rvmax->nb_regs))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if ((num >= 0) && (num < RVMAXNUMCOREREGS)) {
		reg_value = rvmax->core_regs[num];
		buf_set_u32(rvmax->core_cache->reg_list[num].value, 0, 32, reg_value);
		/*LOG_DEBUG("Read core reg %i value 0x%08" PRIx32, num , reg_value); */
		rvmax->core_cache->reg_list[num].valid = 1;
		rvmax->core_cache->reg_list[num].dirty = 0;
	} else {
		/* This is an spr, always read value from HW */
		int retval = du_core->rvmax_jtag_read_cpu(&rvmax->jtag,
							 rvmax->arch_info[num].spr_num, 1, &reg_value);

		if (retval != ERROR_OK) {
			LOG_ERROR("Error while reading spr 0x%08" PRIx32, rvmax->arch_info[num].spr_num);
			return retval;
		}

		buf_set_u32(rvmax->core_cache->reg_list[num].value, 0, 32, reg_value);
		LOG_DEBUG("Read spr reg %i value 0x%08" PRIx32, num , reg_value);
	}

	return ERROR_OK;
}

static int rvmax_write_core_reg(struct target *target, int num)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);

	LOG_DEBUG("-");

	if ((num < 0) || (num >= RVMAXNUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t reg_value = buf_get_u32(rvmax->core_cache->reg_list[num].value, 0, 32);
	rvmax->core_regs[num] = reg_value;
	LOG_DEBUG("Write core reg %i value 0x%08" PRIx32, num , reg_value);
	rvmax->core_cache->reg_list[num].valid = 1;
	rvmax->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

static int rvmax_get_core_reg(struct reg *reg)
{
	struct rvmax_core_reg *rvmax_reg = reg->arch_info;
	struct target *target = rvmax_reg->target;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	return rvmax_read_core_reg(target, rvmax_reg->list_num);
}

static int rvmax_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct rvmax_core_reg *rvmax_reg = reg->arch_info;
	struct target *target = rvmax_reg->target;
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	uint32_t value = buf_get_u32(buf, 0, 32);

	LOG_DEBUG("- %02x, %02x, %02x, %02x", buf[0], buf[1], buf[2], buf[3]);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (rvmax_reg->list_num < RVMAXNUMCOREREGS) {
		int retval;
		buf_set_u32(reg->value, 0, 32, value);
		reg->dirty = 1;
		reg->valid = 1;

		retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
								rvmax_reg->spr_num, 1, &value);

		if (retval != ERROR_OK) {
			LOG_ERROR("Error while writing spr 0x%08" PRIx32, rvmax_reg->spr_num);
			return retval;
		}
	} else {
		/* This is an spr, write it to the HW */
		int retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
								rvmax_reg->spr_num, 1, &value);

		if (retval != ERROR_OK) {
			LOG_ERROR("Error while writing spr 0x%08" PRIx32, rvmax_reg->spr_num);
			return retval;
		}
	}

	return ERROR_OK;
}

static const struct reg_arch_type rvmax_reg_type = {
	.get = rvmax_get_core_reg,
	.set = rvmax_set_core_reg,
};

static struct reg_cache *rvmax_build_reg_cache(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(rvmax->nb_regs, sizeof(struct reg));
	struct rvmax_core_reg *arch_info =
		malloc((rvmax->nb_regs) * sizeof(struct rvmax_core_reg));
	struct reg_feature *feature;

	LOG_DEBUG("-");

	/* Build the process context cache */
	cache->name = "RVMax registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = rvmax->nb_regs;
	(*cache_p) = cache;
	rvmax->core_cache = cache;
	rvmax->arch_info = arch_info;

	for (int i = 0; i < rvmax->nb_regs; i++) {
		arch_info[i] = rvmax_core_reg_list_arch_info[i];
		arch_info[i].target = target;
		arch_info[i].rvmax_common = rvmax;
		reg_list[i].name = rvmax_core_reg_list_arch_info[i].name;

		feature = malloc(sizeof(struct reg_feature));
		feature->name = rvmax_core_reg_list_arch_info[i].feature;
		reg_list[i].feature = feature;

		reg_list[i].group = rvmax_core_reg_list_arch_info[i].group;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &rvmax_reg_type;
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].number = i;
		reg_list[i].exist = true;
	}

	return cache;
}

static int rvmax_debug_entry(struct target *target)
{
	LOG_DEBUG("-");

	int retval = rvmax_save_context(target);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling rvmax_save_context");
		return retval;
	}

	struct rvmax_common *rvmax = target_to_rvmax(target);
	uint32_t addr = rvmax->core_regs[RVMAX_REG_NPC];

	if (addr >= RVMAX_RAM_START) {
		if (breakpoint_find(target, addr)) {
			/* Halted on a breakpoint, step back to permit executing the instruction there */
			retval = rvmax_set_core_reg(&rvmax->core_cache->reg_list[RVMAX_REG_NPC],
							 (uint8_t *)&addr);
		}
	}

	return retval;
}

static int rvmax_halt(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
LOG_DEBUG("-");
	LOG_DEBUG("target->state: %s",
			target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("Target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("Target was in unknown state when halt was requested");

	if (target->state == TARGET_RESET) {
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) &&
				jtag_get_srst()) {
			LOG_ERROR("Can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		} else {
			target->debug_reason = DBG_REASON_DBGRQ;
			return ERROR_OK;
		}
	}

	int retval = du_core->rvmax_cpu_stall(&rvmax->jtag, CPU_STALL);

	if (retval != ERROR_OK) {
		LOG_ERROR("Impossible to stall the CPU");
		return retval;
	}

	/* Registers are now invalid */
	retval = rvmax_restore_context(target);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling rvmax_restore_context");
		return retval;
	}

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int rvmax_is_cpu_running(struct target *target, int *running)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int retval;
	int tries = 0;
	const int RETRIES_MAX = 5;

	/*LOG_DEBUG("-"); */
	/* Have a retry loop to determine of the CPU is running.
		 If target has been hard reset for any reason, it might take a couple
		 of goes before it's ready again.
	*/
	while (tries < RETRIES_MAX) {
		tries++;
		retval = du_core->rvmax_is_cpu_running(&rvmax->jtag, running);

		if (retval != ERROR_OK) {
			LOG_WARNING("Debug IF CPU control reg read failure.");
			/* Try once to restart the JTAG infrastructure -
				 quite possibly the board has just been reset. */
			LOG_WARNING("Resetting JTAG TAP state and reconnectiong to debug IF.");
			du_core->rvmax_jtag_init(&rvmax->jtag);

			LOG_WARNING("...attempt %d of %d", tries, RETRIES_MAX);

			alive_sleep(2);

			continue;
		} else
			return ERROR_OK;
	}

	LOG_ERROR("Could not re-establish communication with target");
	return retval;
}

static int rvmax_poll(struct target *target)
{
	int retval;
	int running;

	retval = rvmax_is_cpu_running(target, &running);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling rvmax_is_cpu_running");
		return retval;
	}

	/* check for processor halted */
	if (!running) {
		/* It's actually stalled, so update our software's state */
		if ((target->state == TARGET_RUNNING) ||
				(target->state == TARGET_RESET)) {

			target->state = TARGET_HALTED;

			retval = rvmax_debug_entry(target);

			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling rvmax_debug_entry");
				return retval;
			}

			target_call_event_callbacks(target,
								TARGET_EVENT_HALTED);
		} else if (target->state == TARGET_DEBUG_RUNNING) {
			target->state = TARGET_HALTED;

			retval = rvmax_debug_entry(target);

			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling rvmax_debug_entry");
				return retval;
			}

			target_call_event_callbacks(target,
								TARGET_EVENT_DEBUG_HALTED);
		}
	} else { /* ... target is running */

		/* If target was supposed to be stalled, stall it again */
		if  (target->state == TARGET_HALTED) {

			target->state = TARGET_RUNNING;

			retval = rvmax_halt(target);

			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling rvmax_halt");
				return retval;
			}

			retval = rvmax_debug_entry(target);

			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling rvmax_debug_entry");
				return retval;
			}

			target_call_event_callbacks(target,
								TARGET_EVENT_DEBUG_HALTED);
		}

		target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

static int rvmax_assert_reset(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	uint32_t addr = RVMAX_PC_RESET_ADDR;

	LOG_DEBUG("-");

	int retval = du_core->rvmax_cpu_reset(&rvmax->jtag, CPU_RESET);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while asserting RESET");
		return retval;
	}

	retval = rvmax_set_core_reg(&rvmax->core_cache->reg_list[RVMAX_REG_NPC],
						 (uint8_t *)&addr);

	return ERROR_OK;
}

static int rvmax_deassert_reset(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);

	LOG_DEBUG("-");

	int retval = du_core->rvmax_cpu_reset(&rvmax->jtag, CPU_NOT_RESET);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while desasserting RESET");
		return retval;
	}

	return ERROR_OK;
}

static int rvmax_soft_reset_halt(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);

	LOG_DEBUG("-");

	int retval = du_core->rvmax_cpu_stall(&rvmax->jtag, CPU_STALL);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while stalling the CPU");
		return retval;
	}

	retval = rvmax_assert_reset(target);

	if (retval != ERROR_OK)
		return retval;

	retval = rvmax_deassert_reset(target);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int rvmax_resume(struct target *target, int current,
					 target_addr_t address, int handle_breakpoints,
					 int debug_execution)
{
	/* LOG_DEBUG("- %08llx %08x %08x",address,current, debug_execution); */

	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int retval = ERROR_OK;

	target->debug_reason = DBG_REASON_NOTHALTED;
	retval = du_core->rvmax_cpu_stall(&rvmax->jtag, CPU_UNSTALL);

	if (retval != ERROR_OK)
			LOG_ERROR("Error while unstalling the CPU");

	/* Registers are now invalid */
	register_cache_invalidate(rvmax->core_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("Target resumed");
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("Target debug resumed");
	}

	return retval;
}

static int rvmax_step(struct target *target, int current,
				 target_addr_t address, int handle_breakpoints)
{
	/* LOG_DEBUG("- %08llx %08x",address,current); */

	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int retval = ERROR_OK;

	target->debug_reason = DBG_REASON_SINGLESTEP;
	retval = du_core->rvmax_cpu_stall(&rvmax->jtag, CPU_STEP);

	if (retval != ERROR_OK)
		LOG_ERROR("Error while stepping the CPU");

	target->state = TARGET_HALTED;

	retval = rvmax_restore_context(target);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling rvmax_restore_context");
		return retval;
	}

	/* Registers are now invalid */
	register_cache_invalidate(rvmax->core_cache);
	return retval;
}

static int rvmax_add_breakpoint(struct target *target,
						 struct breakpoint *breakpoint)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	uint8_t data;
	int retval;
	uint32_t reg_value;
	int i;

	LOG_DEBUG("Adding breakpoint: addr 0x%08" TARGET_PRIxADDR ", len %d, type %d, set: %d, id: %" PRId32,
			breakpoint->address, breakpoint->length, breakpoint->type,
			breakpoint->set, breakpoint->unique_id);

	/* Only support SW breakpoints for now. */
	/*  if (breakpoint->type == BKPT_HARD)  */
	if (breakpoint->address < RVMAX_RAM_START) {
		LOG_DEBUG("BKPT_HARD");
		if (debugger_breakpoint_count > 0) {
			for (i = 0; i < debugger_breakpoint_count; i++) {
				retval = du_core->rvmax_jtag_read_cpu(&rvmax->jtag,
					DEBUGGER_HWBREAKPOINT_CONTROL_OFFSET+i*8, 1, &reg_value);
				LOG_DEBUG("testing debug reg %d: %x", i, reg_value);
				if (retval != ERROR_OK)
					return retval;

				if (reg_value & DBG_HWBREAKPOINT_ENABLED) /* breakpoint in use */
					continue;

				/* TODO: add check for duplicate breakpoints... investigate if openocd/gdb do this for me */
				reg_value = breakpoint->address;
				retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
					DEBUGGER_HWBREAKPOINT_DATA_OFFSET+i*8, 1, &reg_value);

				if (retval != ERROR_OK)
					return retval;

				reg_value |= DBG_HWBREAKPOINT_ENABLED;
				retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
					DEBUGGER_HWBREAKPOINT_CONTROL_OFFSET+i*8, 1, &reg_value);
				breakpoint->type = BKPT_HARD;
				breakpoint->set = true;
				return retval;
			}
		}
	}

	if (breakpoint->address < RVMAX_RAM_START) {
		LOG_ERROR("SW breakpoints are not supported in non-volatile memory");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

LOG_DEBUG("BKPT_SOFT");
	/* Read and save the instruction */
	retval = du_core->rvmax_jtag_read_memory(&rvmax->jtag,
					 breakpoint->address,
					 breakpoint->length,
					 1,
					 &data);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while reading the instruction at 0x%08" TARGET_PRIxADDR,
				 breakpoint->address);
		return retval;
	}

	if (breakpoint->orig_instr != NULL)
		free(breakpoint->orig_instr);

	breakpoint->orig_instr = malloc(breakpoint->length);
	memcpy(breakpoint->orig_instr, &data, breakpoint->length);

	/* Sub in the RVMAX trap instruction */
	uint8_t rvmax_trap_insn[4];

	/* Compact instructions can be 2 or 4 bytes in size (2 for compact instruction).  */
	/* Use breakpoint->length for size */
	if (breakpoint->length == 4)
		target_buffer_set_u32(target, rvmax_trap_insn, RVMAX_TRAP_INSTR);
	else if (breakpoint->length == 2)
		target_buffer_set_u16(target, rvmax_trap_insn, RVMAX_TRAP_INSTRC);

	retval = du_core->rvmax_jtag_write_memory(&rvmax->jtag,
						breakpoint->address,
						breakpoint->length,
						1,
						rvmax_trap_insn);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while writing RVMAX_TRAP_INSTR at 0x%08" TARGET_PRIxADDR,
				 breakpoint->address);
		return retval;
	}

	/* invalidate instruction cache */
	uint32_t addr = breakpoint->address;
	retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag, RVMAX_ICBIR_CPU_REG_ADD, 1, &addr);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while invalidating the ICACHE");
		return retval;
	}

	breakpoint->set = true;
	return ERROR_OK;
}

static int rvmax_remove_breakpoint(struct target *target,
					struct breakpoint *breakpoint)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int retval;
	uint32_t reg_value;
	int i;

	LOG_DEBUG("Removing breakpoint: addr 0x%08" TARGET_PRIxADDR ", len %d, type %d, set: %d, id: %" PRId32,
			breakpoint->address, breakpoint->length, breakpoint->type,
			breakpoint->set, breakpoint->unique_id);

	/* Only support SW breakpoints for now. */
	if (breakpoint->type == BKPT_HARD) {
		LOG_DEBUG("R BKPT_HARD");
		for (i = 0; i < debugger_breakpoint_count; i++) {
			retval = du_core->rvmax_jtag_read_cpu(&rvmax->jtag,
				DEBUGGER_HWBREAKPOINT_CONTROL_OFFSET+i*8, 1, &reg_value);
			/*LOG_DEBUG("testing debug reg %d: %x",i,reg_value); */
			if (retval != ERROR_OK)
				return retval;

			if (reg_value & DBG_HWBREAKPOINT_ENABLED) {/* breakpoint in use */
				retval = du_core->rvmax_jtag_read_cpu(&rvmax->jtag,
					DEBUGGER_HWBREAKPOINT_DATA_OFFSET+i*8, 1, &reg_value);

				if (retval != ERROR_OK)
					return retval;

				if (reg_value == breakpoint->address) {
					reg_value &= ~DBG_HWBREAKPOINT_ENABLED;
					retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
						DEBUGGER_HWBREAKPOINT_CONTROL_OFFSET+i*8, 1, &reg_value);

					if (retval != ERROR_OK)
						return retval;

					LOG_DEBUG("Removed hwbreakpoint at %08" TARGET_PRIxADDR, breakpoint->address);
					breakpoint->set = false;
					return ERROR_OK;
				}
			}
		}
	}

	if (breakpoint->address < RVMAX_RAM_START) {
		LOG_ERROR("SW breakpoints are not supported in non-volatile memory");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
LOG_DEBUG("R BKPT_SOFT");

	/* Replace the removed instruction */
	retval = du_core->rvmax_jtag_write_memory(&rvmax->jtag,
						breakpoint->address,
						breakpoint->length,
						1,
						breakpoint->orig_instr);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while writing back the instruction at 0x%08" TARGET_PRIxADDR,
				 breakpoint->address);
		return retval;
	}

	/* invalidate instruction cache */
	uint32_t addr = breakpoint->address;
	retval = du_core->rvmax_jtag_write_cpu(&rvmax->jtag,
			RVMAX_ICBIR_CPU_REG_ADD, 1, &addr);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while invalidating the ICACHE");
		return retval;
	}

	breakpoint->set = false;
	return ERROR_OK;
}

static int rvmax_add_watchpoint(struct target *target,
						 struct watchpoint *watchpoint)
{
	LOG_ERROR("%s: implement me", __func__);
	return ERROR_OK;
}

static int rvmax_remove_watchpoint(struct target *target,
					struct watchpoint *watchpoint)
{
	LOG_ERROR("%s: implement me", __func__);
	return ERROR_OK;
}

static int rvmax_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int halted = 1;
	enum target_state saved_state;

	LOG_DEBUG("Read memory at 0x%08" TARGET_PRIxADDR ", size: %" PRIu32 ", count: 0x%08" PRIx32, address, size, count);

	if (target->state != TARGET_HALTED) {
		/* LOG_ERROR("Target not halted"); */
		saved_state = target->state;
		rvmax_halt(target);
		halted = 0;
	}

	/* Sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !buffer) {
		LOG_ERROR("Bad arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u))) {
		LOG_ERROR("Can't handle unaligned memory access");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	du_core->rvmax_jtag_read_memory(&rvmax->jtag, address, size, count, buffer);

	if (!halted) {
		du_core->rvmax_cpu_stall(&rvmax->jtag, CPU_UNSTALL);
		target->state = saved_state;
	}

	return ERROR_OK;
}

static int rvmax_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int halted = 1;
	enum target_state saved_state;

	LOG_DEBUG("Write memory at 0x%08" TARGET_PRIxADDR ", size: %" PRIu32 ", count: 0x%08" PRIx32, address, size, count);

	if (target->state != TARGET_HALTED) {
		/* LOG_WARNING("Target not halted"); */
		saved_state = target->state;
		rvmax_halt(target);
		halted = 0;
	}

	/* Sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !buffer) {
		LOG_ERROR("Bad arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u))) {
		LOG_ERROR("Can't handle unaligned memory access");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	du_core->rvmax_jtag_write_memory(&rvmax->jtag, address, size, count, buffer);

	if (!halted) {
		du_core->rvmax_cpu_stall(&rvmax->jtag, CPU_UNSTALL);
		target->state = saved_state;
	}

	return ERROR_OK;
}

static int rvmax_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_jtag *jtag = &rvmax->jtag;
	struct rvmax_tap_ip *rvmax_tap;
	struct rvmax_du *rvmax_du;

	LOG_DEBUG("-");

	list_for_each_entry(rvmax_du, &rm_du_list, list) {
		if (rvmax_du->name) {
			if (!strcmp("adv", rvmax_du->name)) {
				jtag->du_core = rvmax_du;
				LOG_INFO("%s debug unit selected", rvmax_du->name);
			}
		}
	}

	if (rvmax_du == NULL) {
		LOG_ERROR("No debug unit selected");
		return ERROR_FAIL;
	}

	list_for_each_entry(rvmax_tap, &rm_tap_list, list) {
		if (rvmax_tap->name) {
			if (!strcmp(rvmax_tap->name, "mohor")) {
				jtag->tap_ip = rvmax_tap;
				LOG_INFO("%s tap selected", rvmax_tap->name);
			}
		}
	}

	if (jtag->tap_ip == NULL) {
		LOG_ERROR("No tap selected");
		return ERROR_FAIL;
	}

	rvmax->jtag.tap = target->tap;
	rvmax->jtag.rvmax_jtag_inited = 0;
	rvmax->jtag.rvmax_jtag_module_selected = 0;
	rvmax->jtag.target = target;

	rvmax_build_reg_cache(target);

	return ERROR_OK;
}

static int rvmax_target_create(struct target *target, Jim_Interp *interp)
{
	LOG_DEBUG("-");

	if (target->tap == NULL)
		return ERROR_FAIL;

	struct rvmax_common *rvmax = calloc(1, sizeof(struct rvmax_common));

	target->arch_info = rvmax;

	rvmax_create_reg_list(target);

	rvmax_tap_mohor_register();

	rvmax_du_adv_register();

	return ERROR_OK;
}

static int rvmax_examine(struct target *target)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);

	LOG_DEBUG("-");

	if (!target_was_examined(target)) {
		target_set_examined(target);

		int running;

		int retval = du_core->rvmax_is_cpu_running(&rvmax->jtag, &running);

		if (retval != ERROR_OK) {
			LOG_ERROR("Couldn't read the CPU state");
			return retval;
		} else {
			if (running) {
				target->state = TARGET_RUNNING;
			} else {
				LOG_DEBUG("Target is halted");

				/* This is the first time we examine the target,
				 * it is stalled and we don't know why. Let's
				 * assume this is because of a debug reason.
				 */
				if (target->state == TARGET_UNKNOWN)
					target->debug_reason = DBG_REASON_DBGRQ;

				target->state = TARGET_HALTED;
			}
		}
	}

	return ERROR_OK;
}

static int rvmax_arch_state(struct target *target)
{
	LOG_DEBUG("-");
	return ERROR_OK;
}

/* Algorithm must end with a software breakpoint instruction. */
static int rvmax_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_params,
		struct reg_param *reg_params, target_addr_t entry_point,
		target_addr_t exit_point, int timeout_ms, void *arch_info)
{
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	uint64_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN)
			continue;

		LOG_DEBUG("save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		if (!r) {
			LOG_ERROR("Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("Register %s is %d bits instead of %d bits.",
					reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK)
			return ERROR_FAIL;
		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);
		if (r->type->set(r, reg_params[i].value) != ERROR_OK)
			return ERROR_FAIL;
	}

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;
		retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}


	/* Disable Interrupts before attempting to run the algorithm. */
	uint64_t current_mstatus;
	uint8_t mstatus_bytes[8];

	LOG_DEBUG("Disabling Interrupts");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", 1);
	if (!reg_mstatus) {
		LOG_ERROR("Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	reg_mstatus->type->get(reg_mstatus);
	current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	uint64_t ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
	buf_set_u64(mstatus_bytes, 0, reg_mstatus->size, set_field(current_mstatus,
				ie_mask, 0));

	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Run algorithm */
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (rvmax_resume(target, 0, entry_point, 0, 0) != ERROR_OK)
		return ERROR_FAIL;

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %d ms.", timeout_ms);
			LOG_ERROR("  now   = 0x%08x", (uint32_t) now);
			LOG_ERROR("  start = 0x%08x", (uint32_t) start);
			rvmax_halt(target);
			rvmax_poll(target);
			return ERROR_TARGET_TIMEOUT;
		}

		int result = rvmax_poll(target);
		if (result != ERROR_OK)
			return result;
	}

	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	/* Read memory values to mem_params[] */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* Restore Interrupts */
	LOG_DEBUG("Restoring Interrupts");
	buf_set_u64(mstatus_bytes, 0, reg_mstatus->size, current_mstatus);
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Restore registers */
	uint8_t buf[8];
	buf_set_u64(buf, 0, reg_pc->size, saved_pc);
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		buf_set_u64(buf, 0, r->size, saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* TODO: target_start_algorithm target_wait_algorithm */
uint64_t saved_regs[32];
uint64_t saved_pc;
uint64_t current_mstatus;
uint8_t mstatus_bytes[8];
struct reg *reg_mstatus;
struct reg *reg_pc;

static int rvmax_start_algorithm(struct target *target, int num_mem_params,
			struct mem_param *mem_params, int num_reg_params,
			struct reg_param *reg_params, target_addr_t entry_point,
			target_addr_t exit_point, void *arch_info)
{
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;

	/* Save the PC */
	saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN)
			continue;

		LOG_DEBUG("save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		if (!r) {
			LOG_ERROR("Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("Register %s is %d bits instead of %d bits.",
					reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK)
			return ERROR_FAIL;
		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);
		if (r->type->set(r, reg_params[i].value) != ERROR_OK)
			return ERROR_FAIL;
	}

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;
		retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Disable Interrupts before attempting to run the algorithm. */
	LOG_DEBUG("Disabling Interrupts");
	reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", 1);
	if (!reg_mstatus) {
		LOG_ERROR("Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	reg_mstatus->type->get(reg_mstatus);
	current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	uint64_t ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
	buf_set_u64(mstatus_bytes, 0, reg_mstatus->size, set_field(current_mstatus,
				ie_mask, 0));

	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Run algorithm */
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (target_resume(target, 0, entry_point, 1, 1) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int rvmax_wait_algorithm(struct target *target, int num_mem_params,
			struct mem_param *mem_params, int num_reg_params,
			struct reg_param *reg_param, target_addr_t exit_point,
			int timeout_ms, void *arch_info)
{
	int64_t start = timeval_ms();
	int retval;

	while (target->state != TARGET_HALTED) {
		LOG_DEBUG("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %d ms.", timeout_ms);
			LOG_ERROR("  now   = 0x%08x", (uint32_t) now);
			LOG_ERROR("  start = 0x%08x", (uint32_t) start);
			rvmax_halt(target);
			rvmax_poll(target);
			return ERROR_TARGET_TIMEOUT;
		}

		int result = rvmax_poll(target);
		if (result != ERROR_OK)
			return result;
	}

	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	/* Read memory values to mem_params[] */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	#if 0
	/* Restore Interrupts */
	LOG_DEBUG("Restoring Interrupts");
	buf_set_u64(mstatus_bytes, 0, reg_mstatus->size, current_mstatus);
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Restore registers */
	uint8_t buf[8];
	buf_set_u64(buf, 0, reg_pc->size, saved_pc);
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		buf_set_u64(buf, 0, r->size, saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK)
			return ERROR_FAIL;
	}
	#endif

	return ERROR_OK;
}

static int rvmax_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
				int *reg_list_size, enum target_register_class reg_class)
{
	struct rvmax_common *rvmax = target_to_rvmax(target);
	LOG_DEBUG("-");

	if (reg_class == REG_CLASS_GENERAL) {
		/* We will have this called whenever GDB connects. */
		int retval = rvmax_save_context(target);

		if (retval != ERROR_OK) {
			LOG_ERROR("Error while calling rvmax_save_context");
			return retval;
		}

		*reg_list_size = RVMAXNUMCOREREGS;
		/* this is free()'d back in gdb_server.c's gdb_get_register_packet() */
		*reg_list = malloc((*reg_list_size) * sizeof(struct reg *));

		for (int i = 0; i < RVMAXNUMCOREREGS; i++)
			(*reg_list)[i] = &rvmax->core_cache->reg_list[i];
	} else {
		*reg_list_size = rvmax->nb_regs;
		*reg_list = malloc((*reg_list_size) * sizeof(struct reg *));

		for (int i = 0; i < rvmax->nb_regs; i++)
			(*reg_list)[i] = &rvmax->core_cache->reg_list[i];
	}

	return ERROR_OK;
}

int rvmax_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info)
{
	LOG_DEBUG("-");
	return ERROR_FAIL;
}

static int rvmax_checksum_memory(struct target *target, target_addr_t address,
		uint32_t count, uint32_t *checksum) {

	LOG_DEBUG("-");
	return ERROR_FAIL;
}

static int rvmax_profiling(struct target *target, uint32_t *samples,
		uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds)
{
	struct timeval timeout, now;
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_du *du_core = rvmax_to_du(rvmax);
	int retval = ERROR_OK;

	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, seconds, 0);

	LOG_INFO("Starting rvmax profiling. Sampling npc as fast as we can...");

	/* Make sure the target is running */
	target_poll(target);

	if (target->state == TARGET_HALTED)
		retval = target_resume(target, 1, 0, 0, 0);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while resuming target");
		return retval;
	}

	uint32_t sample_count = 0;

	for (;;) {
		uint32_t reg_value;
		retval = du_core->rvmax_jtag_read_cpu(&rvmax->jtag, GROUP0 + RVMAX_REG_NPC, 1, &reg_value);
		LOG_DEBUG("PC CPU: %08x", reg_value);

		if (retval != ERROR_OK) {
			LOG_ERROR("Error while reading PC");
			return retval;
		}

		samples[sample_count++] = reg_value;

		gettimeofday(&now, NULL);

		if ((sample_count >= max_num_samples) || timeval_compare(&now, &timeout) > 0) {
			LOG_INFO("Profiling completed. %" PRIu32 " samples.", sample_count);
			break;
		}
	}

	*num_samples = sample_count;
	return retval;
}

COMMAND_HANDLER(rvmax_tap_select_command_handler)
{
	struct target *target = get_current_target(CMD_CTX);
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_jtag *jtag = &rvmax->jtag;
	struct rvmax_tap_ip *rvmax_tap;

	LOG_DEBUG("-");
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(rvmax_tap, &rm_tap_list, list) {
		if (rvmax_tap->name) {
			if (!strcmp(CMD_ARGV[0], rvmax_tap->name)) {
				jtag->tap_ip = rvmax_tap;
				LOG_INFO("%s tap selected", rvmax_tap->name);
				return ERROR_OK;
			}
		}
	}

	LOG_ERROR("%s (wanted %s) unknown, no tap selected", CMD_ARGV[0], rvmax_tap->name);
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(rvmax_tap_list_command_handler)
{
	struct rvmax_tap_ip *rvmax_tap;

	LOG_DEBUG("-");
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(rvmax_tap, &rm_tap_list, list) {
		if (rvmax_tap->name)
			command_print(CMD, "%s", rvmax_tap->name);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(rvmax_du_select_command_handler)
{
	struct target *target = get_current_target(CMD_CTX);
	struct rvmax_common *rvmax = target_to_rvmax(target);
	struct rvmax_jtag *jtag = &rvmax->jtag;
	struct rvmax_du *rvmax_du;

	LOG_DEBUG("-");
	if (CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(rvmax_du, &rm_du_list, list) {
		if (rvmax_du->name) {
			if (!strcmp(CMD_ARGV[0], rvmax_du->name)) {
				jtag->du_core = rvmax_du;
				LOG_INFO("%s debug unit selected", rvmax_du->name);

				if (CMD_ARGC == 2) {
					int options;
					COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], options);
					rvmax_du->options = options;
					LOG_INFO("Option %x is passed to %s debug unit"
						 , options, rvmax_du->name);
				}

				return ERROR_OK;
			}
		}
	}

	LOG_ERROR("%s unknown, no debug unit selected", CMD_ARGV[0]);
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(rvmax_du_list_command_handler)
{
	struct rvmax_du *rvmax_du;

	LOG_DEBUG("-");
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(rvmax_du, &rm_du_list, list) {
		if (rvmax_du->name)
			command_print(CMD, "%s", rvmax_du->name);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(rvmax_addreg_command_handler)
{
	struct target *target = get_current_target(CMD_CTX);
	struct rvmax_core_reg new_reg;

	LOG_DEBUG("-");
	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	new_reg.target = NULL;
	new_reg.rvmax_common = NULL;

	uint32_t addr;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], addr);

	new_reg.name = strdup(CMD_ARGV[0]);
	new_reg.spr_num = addr;
	new_reg.feature = strdup(CMD_ARGV[2]);
	new_reg.group = strdup(CMD_ARGV[3]);

	rvmax_add_reg(target, &new_reg);

	LOG_DEBUG("Add reg \"%s\" @ 0x%08" PRIx32 ", group \"%s\", feature \"%s\"",
			new_reg.name, addr, new_reg.group, new_reg.feature);

	return ERROR_OK;
}

COMMAND_HANDLER(rvmax_auth_command_handler)
{

	LOG_DEBUG("-");
	if (CMD_ARGC != AUTH_LEN)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int i;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], rvmax_auth_data[0]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], rvmax_auth_data[1]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], rvmax_auth_data[2]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], rvmax_auth_data[3]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[4], rvmax_auth_data[4]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[5], rvmax_auth_data[5]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], rvmax_auth_data[6]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], rvmax_auth_data[7]);

	LOG_INFO("AUTH_DATA");

	for (i = 0; i < AUTH_LEN; i++)
		LOG_INFO("0x%08x ", rvmax_auth_data[i]);

	return ERROR_OK;
}
COMMAND_HANDLER(rvmax_version_command_handler)
{
	unsigned int i;
	if (CMD_ARGC >= 1) {
		strcat(rvmax_driver_info, ".");
		for (i = 0; i < CMD_ARGC; i++) {
			strcat(rvmax_driver_info, " ");
			strcat(rvmax_driver_info, CMD_ARGV[i]);
		}
		strcat(rvmax_driver_info, ".");
	}
	LOG_INFO("Maxim Integrated, RVMax driver version: %d.%03d%s",
		RVMAX_VERSION_MAX, RVMAX_VERSION_MIN, rvmax_driver_info);

	return ERROR_OK;
}

static const struct command_registration rvmax_hw_ip_command_handlers[] = {
	{
		"tap_select",
		.handler = rvmax_tap_select_command_handler,
		.mode = COMMAND_ANY,
		.usage = "tap_select name",
		.help = "Select the TAP core to use",
	},
	{
		"tap_list",
		.handler = rvmax_tap_list_command_handler,
		.mode = COMMAND_ANY,
		.usage = "tap_list",
		.help = "Display available TAP core",
	},
	{
		"du_select",
		.handler = rvmax_du_select_command_handler,
		.mode = COMMAND_ANY,
		.usage = "du_select name",
		.help = "Select the Debug Unit core to use",
	},
	{
		"du_list",
		.handler = rvmax_du_list_command_handler,
		.mode = COMMAND_ANY,
		.usage = "select_tap name",
		.help = "Display available Debug Unit core",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration rvmax_reg_command_handlers[] = {
	{
		"addreg",
		.handler = rvmax_addreg_command_handler,
		.mode = COMMAND_ANY,
		.usage = "addreg name addr feature group",
		.help = "Add a register to the register list",
	},
	{
		"auth",
		.handler = rvmax_auth_command_handler,
		.mode = COMMAND_ANY,
		.usage = "auth authdata 0..7",
		.help = "Add debug authorization",
	},
	{
		"rvmaxver",
		.handler = rvmax_version_command_handler,
		.mode = COMMAND_ANY,
		.usage = "rvmaxver",
		.help = "Display driver version",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration rvmax_command_handlers[] = {
	{
		.chain = rvmax_reg_command_handlers,
	},
	{
		.chain = rvmax_hw_ip_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


struct target_type rvmax_target = {
	.name = "rvmax",

	.poll = rvmax_poll,
	.arch_state = rvmax_arch_state,

	.run_algorithm = rvmax_run_algorithm,
	.start_algorithm = rvmax_start_algorithm,
	.wait_algorithm = rvmax_wait_algorithm,

	.target_request_data = NULL,

	.halt = rvmax_halt,
	.resume = rvmax_resume,
	.step = rvmax_step,

	.assert_reset = rvmax_assert_reset,
	.deassert_reset = rvmax_deassert_reset,
	.soft_reset_halt = rvmax_soft_reset_halt,

	.get_gdb_reg_list = rvmax_get_gdb_reg_list,

	.read_memory = rvmax_read_memory,
	.write_memory = rvmax_write_memory,
	.checksum_memory = rvmax_checksum_memory,

	.commands = rvmax_command_handlers,
	.add_breakpoint = rvmax_add_breakpoint,
	.remove_breakpoint = rvmax_remove_breakpoint,
	.add_watchpoint = rvmax_add_watchpoint,
	.remove_watchpoint = rvmax_remove_watchpoint,

	.target_create = rvmax_target_create,
	.init_target = rvmax_init_target,
	.examine = rvmax_examine,

	.get_gdb_fileio_info = rvmax_get_gdb_fileio_info,

	.profiling = rvmax_profiling,
};

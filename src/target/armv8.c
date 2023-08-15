// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2015 by David Ung                                       *
 *                                                                         *
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   <ilg@livius.net>                                                      *
 *                                                                         *
 *   Portions Copyright (C) 2021-2023 Analog Devices, Inc.                 *
 ***************************************************************************/


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/replacements.h>

#include "armv8.h"
#include "arm_disassembler.h"

#include "register.h"
#include <helper/binarybuffer.h>
#include <helper/command.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "armv8_opcodes.h"
#include "target.h"
#include "target_type.h"
#include "semihosting_common.h"

static const char * const armv8_state_strings[] = {
	"AArch32", "Thumb", "Jazelle", "ThumbEE", "AArch64",
};

static const struct {
	const char *name;
	unsigned psr;
} armv8_mode_data[] = {
	{
		.name = "USR",
		.psr = ARM_MODE_USR,
	},
	{
		.name = "FIQ",
		.psr = ARM_MODE_FIQ,
	},
	{
		.name = "IRQ",
		.psr = ARM_MODE_IRQ,
	},
	{
		.name = "SVC",
		.psr = ARM_MODE_SVC,
	},
	{
		.name = "MON",
		.psr = ARM_MODE_MON,
	},
	{
		.name = "ABT",
		.psr = ARM_MODE_ABT,
	},
	{
		.name = "HYP",
		.psr = ARM_MODE_HYP,
	},
	{
		.name = "UND",
		.psr = ARM_MODE_UND,
	},
	{
		.name = "SYS",
		.psr = ARM_MODE_SYS,
	},
	{
		.name = "EL0T",
		.psr = ARMV8_64_EL0T,
	},
	{
		.name = "EL1T",
		.psr = ARMV8_64_EL1T,
	},
	{
		.name = "EL1H",
		.psr = ARMV8_64_EL1H,
	},
	{
		.name = "EL2T",
		.psr = ARMV8_64_EL2T,
	},
	{
		.name = "EL2H",
		.psr = ARMV8_64_EL2H,
	},
	{
		.name = "EL3T",
		.psr = ARMV8_64_EL3T,
	},
	{
		.name = "EL3H",
		.psr = ARMV8_64_EL3H,
	},
};

/** Map PSR mode bits to the name of an ARM processor operating mode. */
const char *armv8_mode_name(unsigned psr_mode)
{
	for (unsigned i = 0; i < ARRAY_SIZE(armv8_mode_data); i++) {
		if (armv8_mode_data[i].psr == psr_mode)
			return armv8_mode_data[i].name;
	}
	LOG_ERROR("unrecognized psr mode: %#02x", psr_mode);
	return "UNRECOGNIZED";
}

static int instr_read_data_r0_64(struct arm_dpm *dpm, uint32_t opcode, uint64_t *data_64, uint32_t expected_el)
{
	if (armv8_curel_from_core_mode(dpm->arm->core_mode) < expected_el) {
		return ERROR_TARGET_EXCEPTION_LEVEL;
	}

	return dpm->instr_read_data_r0_64(dpm, opcode, data_64);
}

static int instr_read_data_r0(struct arm_dpm *dpm, uint32_t opcode, uint64_t *data_64, uint32_t expected_el)
{
	if (armv8_curel_from_core_mode(dpm->arm->core_mode) < expected_el) {
		return ERROR_TARGET_EXCEPTION_LEVEL;
	}

	uint32_t data;
	int retval = dpm->instr_read_data_r0(dpm, opcode, &data);
	*data_64 = data;
	return retval;
}

static int instr_read_data_r0_32(struct arm_dpm *dpm, uint32_t opcode, uint32_t *data, uint32_t expected_el)
{
	if (armv8_curel_from_core_mode(dpm->arm->core_mode) < expected_el) {
		return ERROR_TARGET_EXCEPTION_LEVEL;
	}

	return dpm->instr_read_data_r0(dpm, opcode, data);
}

static int instr_read_data_dcc(struct arm_dpm *dpm, uint32_t opcode, uint32_t *data, uint32_t expected_el)
{
	if (armv8_curel_from_core_mode(dpm->arm->core_mode) < expected_el) {
		return ERROR_TARGET_EXCEPTION_LEVEL;
	}

	return dpm->instr_read_data_dcc(dpm, opcode, data);
}

static int armv8_read_reg(struct armv8_common *armv8, int regnum, uint64_t *regval)
{
	struct arm_dpm *dpm = &armv8->dpm;
	int retval;
	uint32_t value;
	uint64_t value_64;

	switch (regnum) {
	case 0 ... 30:
		retval = dpm->instr_read_data_dcc_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBG_DBGDTR_EL0, regnum), &value_64);
		break;
	case ARMV8_SP:
		retval = dpm->instr_read_data_r0_64(dpm,
				ARMV8_MOVFSP_64(0), &value_64);
		break;
	case ARMV8_PC:
		retval = dpm->instr_read_data_r0_64(dpm,
				ARMV8_MRS_DLR(0), &value_64);
		break;
	case ARMV8_XPSR:
		retval = dpm->instr_read_data_r0(dpm,
				ARMV8_MRS_DSPSR(0), &value);
		value_64 = value;
		break;
	case ARMV8_FPSR:
		retval = dpm->instr_read_data_r0(dpm,
				ARMV8_MRS_FPSR(0), &value);
		value_64 = value;
		break;
	case ARMV8_FPCR:
		retval = dpm->instr_read_data_r0(dpm,
				ARMV8_MRS_FPCR(0), &value);
		value_64 = value;
		break;
	case ARMV8_AMAIR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AMAIR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_AMAIR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AMAIR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_AMAIR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AMAIR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_CCSIDR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CCSIDR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_CLIDR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CLIDR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_CPUCFR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CPUCFR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_CPUPWRCTLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CPUPWRCTLR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGAUTHSTATUS_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGAUTHSTATUS_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGCLAIMCLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGCLAIMCLR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGCLAIMSET_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGCLAIMSET_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGDTRRX_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGDTRRX_EL0, 0), &value_64, 0);
		break;
	case ARMV8_DBGDTRTX_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGDTRTX_EL0, 0), &value_64, 0);
		break;
	case ARMV8_DBGDTR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGDTR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_DBGPRCR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGPRCR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGVCR32_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGVCR32_EL2, 0), &value_64, 2);
		break;
	case ARMV8_DBGWVR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWVR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGWVR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWVR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGWVR2_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWVR2_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGWVR3_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWVR3_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGWCR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWCR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGWCR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWCR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGWCR2_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWCR2_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DBGWCR3_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DBGWCR3_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DCZID_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DCZID_EL0, 0), &value_64, 0);
		break;
	case ARMV8_ELR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ELR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ELR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ELR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ELR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ELR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_CTR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CTR, 0), &value_64, 0);
		break;
	case ARMV8_ESR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ESR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ESR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ESR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ESR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ESR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_ERRIDR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERRIDR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ERRSELR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERRSELR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ERXADDR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERXADDR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ERXCTLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERXCTLR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ERXFR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERXFR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ERXMISC0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERXMISC0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ERXMISC1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERXMISC1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ERXSTATUS_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ERXSTATUS_EL1, 0), &value_64, 1);
		break;
	case ARMV8_HACR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_HACR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_HSTR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_HSTR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_PAR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PAR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_REVIDR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_REVIDR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_SPSR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SPSR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_SPSR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SPSR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_SPSR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SPSR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_FAR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_FAR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_FAR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_FAR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_FAR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_FAR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_SCTLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SCTLR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_SCTLR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SCTLR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_SCTLR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SCTLR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_TTBR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_TTBR0_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR0_EL2, 0), &value_64, 2);
		break;
	case ARMV8_TTBR0_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR0_EL3, 0), &value_64, 3);
		break;
	case ARMV8_VBAR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VBAR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_VBAR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VBAR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_VBAR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VBAR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_VMPIDR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VMPIDR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_VPIDR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VPIDR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ACTLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ACTLR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ACTLR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ACTLR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ACTLR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ACTLR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_AFSR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AFSR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_AFSR0_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AFSR0_EL2, 0), &value_64, 2);
		break;
	case ARMV8_AFSR0_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AFSR0_EL3, 0), &value_64, 3);
		break;
	case ARMV8_AFSR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AFSR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_AFSR1_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AFSR1_EL2, 0), &value_64, 2);
		break;
	case ARMV8_AFSR1_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_AFSR1_EL3, 0), &value_64, 3);
		break;
	case ARMV8_CONTEXTIDR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CONTEXTIDR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_CONTEXTIDR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CONTEXTIDR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTFRQ_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTFRQ_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTPCT_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTPCT_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTVCT_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTVCT_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTP_TVAL_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTP_TVAL_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTP_CTL_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTP_CTL_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTP_CVAL_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTP_CVAL_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTV_TVAL_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTV_TVAL_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTV_CTL_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTV_CTL_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTV_CVAL_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTV_CVAL_EL0, 0), &value_64, 0);
		break;
	case ARMV8_CNTKCTL_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTKCTL_EL1, 0), &value_64, 1);
		break;
	case ARMV8_CNTPS_TVAL_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTPS_TVAL_EL1, 0), &value_64, 3);
		break;
	case ARMV8_CNTPS_CTL_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTPS_CTL_EL1, 0), &value_64, 3);
		break;
	case ARMV8_CNTPS_CVAL_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTPS_CVAL_EL1, 0), &value_64, 3);
		break;
	case ARMV8_CNTVOFF_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTVOFF_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTHCTL_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTHCTL_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTHP_TVAL_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTHP_TVAL_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTHP_CTL_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTHP_CTL_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTHP_CVAL_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTHP_CVAL_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTHV_TVAL_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTHV_TVAL_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTHV_CTL_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTHV_CTL_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CNTHV_CVAL_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CNTHV_CVAL_EL2, 0), &value_64, 2);
		break;

	case ARMV8_CPACR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CPACR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_CPTR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CPTR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_CPTR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CPTR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_CSSELR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_CSSELR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_DACR32_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DACR32_EL2, 0), &value_64, 2);
		break;
	case ARMV8_DISR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_DISR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_HCR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_HCR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_HPFAR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_HPFAR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_IFSR32_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_IFSR32_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ISR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ISR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_MAIR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MAIR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_MAIR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MAIR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_MAIR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MAIR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_SCR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SCR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_TCR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TCR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_TCR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TCR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_TCR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TCR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_TTBR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_TTBR1_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR1_EL2, 0), &value_64, 2);
		break;
	case ARMV8_VTCR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VTCR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_VTTBR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VTTBR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_RMR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_RMR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_RVBAR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_RVBAR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_SDER32_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_SDER32_EL3, 0), &value_64, 3);
		break;
	case ARMV8_TPIDRRO_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TPIDRRO_EL0, 0), &value_64, 0);
		break;
	case ARMV8_TPIDR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TPIDR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_TPIDR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TPIDR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_TPIDR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TPIDR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_TPIDR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TPIDR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_VDISR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VDISR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_VSESR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_VSESR_EL2, 0), &value_64, 2);
		break;

	case ARMV8_ICC_AP0R0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_AP0R0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_AP1R0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_AP1R0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_BPR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_BPR0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_BPR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_BPR1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_CTLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_CTLR_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_HPPIR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_HPPIR0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_HPPIR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_HPPIR1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_IAR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_IAR0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_IAR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_IAR1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_IGRPEN0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_IGRPEN0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_IGRPEN1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_IGRPEN1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_PMR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_PMR_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_RPR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_RPR_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICC_SRE_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_SRE_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ICV_AP0R0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_AP0R0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_AP1R0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_AP1R0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_BPR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_BPR0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_BPR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_BPR1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_CTLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_CTLR_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_HPPIR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_HPPIR0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_HPPIR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_HPPIR1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_IAR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_IAR0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_IAR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_IAR1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_IGRPEN0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_IGRPEN0_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_IGRPEN1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_IGRPEN1_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_PMR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_PMR_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ICV_RPR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICV_RPR_EL1, 0), &value_64, 3);
		break;
	case ARMV8_ID_AA64AFR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64AFR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64AFR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64AFR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64DFR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64DFR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64DFR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64DFR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64ISAR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64ISAR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64ISAR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64ISAR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64MMFR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64MMFR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64MMFR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64MMFR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64MMFR2_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64MMFR2_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64PFR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64PFR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AA64PFR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AA64PFR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_AFR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_AFR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_ISAR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_ISAR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_ISAR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_ISAR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_ISAR2_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_ISAR2_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_ISAR3_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_ISAR3_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_ISAR4_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_ISAR4_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_ISAR5_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_ISAR5_EL1, 0), &value_64, 1);
		break;

	case ARMV8_LORID_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_LORID_EL1, 0), &value_64, 1);
		break;

	case ARMV8_ID_MMFR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_MMFR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_MMFR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_MMFR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_MMFR2_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_MMFR2_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_MMFR3_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_MMFR3_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_MMFR4_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_MMFR4_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_PFR0_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_PFR0_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ID_PFR1_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ID_PFR1_EL1, 0), &value_64, 1);
		break;
	case ARMV8_ICH_AP0R0_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_AP0R0_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_AP1R0_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_AP1R0_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_EISR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_EISR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_ELRSR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_ELRSR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_HCR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_HCR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_LR0_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_LR0_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_LR1_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_LR1_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_LR2_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_LR2_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_LR3_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_LR3_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_MISR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_MISR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_VMCR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_VMCR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICH_VTR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICH_VTR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_ICC_SRE_EL2:
		// read does not work
		value_64 = 0x0;
		retval = ERROR_OK;
		break;
	case ARMV8_ICC_CTLR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_CTLR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_ICC_IGRPEN1_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_IGRPEN1_EL3, 0), &value_64, 3);
		break;
	case ARMV8_ICC_SRE_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_ICC_SRE_EL3, 0), &value_64, 3);
		break;
	case ARMV8_MDCCINT_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MDCCINT_EL1, 0), &value_64, 1);
		break;
	case ARMV8_MDCCSR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MDCCSR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_MDSCR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MDSCR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_MDCR_EL2:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MDCR_EL2, 0), &value_64, 2);
		break;
	case ARMV8_MDCR_EL3:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_MDCR_EL3, 0), &value_64, 3);
		break;
	case ARMV8_OSDLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_OSDLR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_OSDTRRX_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_OSDTRRX_EL1, 0), &value_64, 1);
		break;
	case ARMV8_OSDTRTX_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_OSDTRTX_EL1, 0), &value_64, 1);
		break;
	case ARMV8_OSECCR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_OSECCR_EL1, 0), &value_64, 1);
		break;

	case ARMV8_OSLSR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_OSLSR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_PMCCFILTR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMCCFILTR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMCCNTR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMCCNTR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMCEID0_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMCEID0_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMCEID1_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMCEID1_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMCNTENCLR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMCNTENCLR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMCNTENSET_EL0:
		retval = instr_read_data_r0(dpm,
				ARMV8_MRS(SYSTEM_PMCNTENSET_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMCR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMCR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVCNTR0_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVCNTR0_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVCNTR1_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVCNTR1_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVCNTR2_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVCNTR2_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVCNTR3_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVCNTR3_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVCNTR4_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVCNTR4_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVCNTR5_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVCNTR5_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVTYPER0_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVTYPER0_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVTYPER1_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVTYPER1_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVTYPER2_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVTYPER2_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVTYPER3_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVTYPER3_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVTYPER4_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVTYPER4_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMEVTYPER5_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMEVTYPER5_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMINTENCLR_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMINTENCLR_EL1, 0), &value_64, 1);
		break;
	case ARMV8_PMINTENSET_EL1:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMINTENSET_EL1, 0), &value_64, 1);
		break;
	case ARMV8_PMOVSCLR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMOVSCLR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMOVSSET_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMOVSSET_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMSELR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMSELR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMUSERENR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMUSERENR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMXEVCNTR_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMXEVCNTR_EL0, 0), &value_64, 0);
		break;
	case ARMV8_PMXEVTYPER_EL0:
		retval = instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_PMXEVTYPER_EL0, 0), &value_64, 0);
		break;

	// write only, skip
	case ARMV8_ICC_ASGI1R_EL1:
	case ARMV8_ICC_DIR_EL1:
	case ARMV8_ICC_EOIR0_EL1:
	case ARMV8_ICC_EOIR1_EL1:
	case ARMV8_ICC_SGI0R_EL1:
	case ARMV8_ICC_SGI1R_EL1:
	case ARMV8_ICV_DIR_EL1:
	case ARMV8_ICV_EOIR0_EL1:
	case ARMV8_ICV_EOIR1_EL1:
		value_64 = 0x0;
		retval = ERROR_OK;
		break;
	default:
		retval = ERROR_FAIL;
		break;
	}

	if (retval == ERROR_OK && regval != NULL) {
		*regval = value_64;
	}
	else if (retval == ERROR_TARGET_EXCEPTION_LEVEL && regval != NULL) {
		*regval = 0xDEADBEEF;
	}
	else
		retval = ERROR_FAIL;

	return retval;
}

static int armv8_read_reg_simdfp_aarch64(struct armv8_common *armv8, int regnum, uint64_t *lvalue, uint64_t *hvalue)
{
	int retval = ERROR_FAIL;
	struct arm_dpm *dpm = &armv8->dpm;

	switch (regnum) {
	case ARMV8_V0 ... ARMV8_V31:
		retval = dpm->instr_read_data_r0_64(dpm,
				ARMV8_MOV_GPR_VFP(0, (regnum - ARMV8_V0), 1), hvalue);
		if (retval != ERROR_OK)
			return retval;
		retval = dpm->instr_read_data_r0_64(dpm,
				ARMV8_MOV_GPR_VFP(0, (regnum - ARMV8_V0), 0), lvalue);
		break;

	default:
		retval = ERROR_FAIL;
		break;
	}

	return retval;
}

static int armv8_write_reg(struct armv8_common *armv8, int regnum, uint64_t value_64)
{
	struct arm_dpm *dpm = &armv8->dpm;
	int retval;
	uint32_t value;

	switch (regnum) {
	case 0 ... 30:
		retval = dpm->instr_write_data_dcc_64(dpm,
			ARMV8_MRS(SYSTEM_DBG_DBGDTR_EL0, regnum),
			value_64);
		break;
	case ARMV8_SP:
		retval = dpm->instr_write_data_r0_64(dpm,
			ARMV8_MOVTSP_64(0),
			value_64);
		break;
	case ARMV8_PC:
		retval = dpm->instr_write_data_r0_64(dpm,
			ARMV8_MSR_DLR(0),
			value_64);
		break;
	case ARMV8_XPSR:
		value = value_64;
		retval = dpm->instr_write_data_r0(dpm,
			ARMV8_MSR_DSPSR(0),
			value);
		break;
	case ARMV8_FPSR:
		value = value_64;
		retval = dpm->instr_write_data_r0(dpm,
			ARMV8_MSR_FPSR(0),
			value);
		break;
	case ARMV8_FPCR:
		value = value_64;
		retval = dpm->instr_write_data_r0(dpm,
			ARMV8_MSR_FPCR(0),
			value);
		break;
	/* registers clobbered by taking exception in debug state */
	case ARMV8_AMAIR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AMAIR_EL1, 0), value_64);
		break;
	case ARMV8_AMAIR_EL2:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AMAIR_EL2, 0), value_64);
		break;
	case ARMV8_AMAIR_EL3:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AMAIR_EL3, 0), value_64);
		break;
	case ARMV8_CCSIDR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CCSIDR_EL1, 0), value_64);
		break;
	case ARMV8_CLIDR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CLIDR_EL1, 0), value_64);
		break;
	case ARMV8_CPUCFR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CPUCFR_EL1, 0), value_64);
		break;
	case ARMV8_CPUPWRCTLR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CPUPWRCTLR_EL1, 0), value_64);
		break;
	case ARMV8_DBGAUTHSTATUS_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGAUTHSTATUS_EL1, 0), value_64);
		break;
	case ARMV8_DBGCLAIMCLR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGCLAIMCLR_EL1, 0), value_64);
		break;
	case ARMV8_DBGCLAIMSET_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGCLAIMSET_EL1, 0), value_64);
		break;
	case ARMV8_DBGDTRRX_EL0:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGDTRRX_EL0, 0), value_64);
		break;
	case ARMV8_DBGDTRTX_EL0:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGDTRTX_EL0, 0), value_64);
		break;
	case ARMV8_DBGDTR_EL0:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGDTR_EL0, 0), value_64);
		break;
	case ARMV8_DBGPRCR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGPRCR_EL1, 0), value_64);
		break;
	case ARMV8_DBGVCR32_EL2:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGVCR32_EL2, 0), value_64);
		break;
	case ARMV8_DBGWVR0_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWVR0_EL1, 0), value_64);
		break;
	case ARMV8_DBGWVR1_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWVR1_EL1, 0), value_64);
		break;
	case ARMV8_DBGWVR2_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWVR2_EL1, 0), value_64);
		break;
	case ARMV8_DBGWVR3_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWVR3_EL1, 0), value_64);
		break;
	case ARMV8_DBGWCR0_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWCR0_EL1, 0), value_64);
		break;
	case ARMV8_DBGWCR1_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWCR1_EL1, 0), value_64);
		break;
	case ARMV8_DBGWCR2_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWCR2_EL1, 0), value_64);
		break;
	case ARMV8_DBGWCR3_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBGWCR3_EL1, 0), value_64);
		break;
	case ARMV8_DCZID_EL0:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DCZID_EL0, 0), value_64);
		break;
	case ARMV8_ELR_EL1:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ELR_EL1, 0), value_64);
		break;
	case ARMV8_ELR_EL2:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ELR_EL2, 0), value_64);
		break;
	case ARMV8_ELR_EL3:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ELR_EL3, 0), value_64);
		break;
	case ARMV8_CTR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CTR, 0), value);
		break;
	case ARMV8_ESR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ESR_EL1, 0), value);
		break;
	case ARMV8_ESR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ESR_EL2, 0), value);
		break;
	case ARMV8_ESR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ESR_EL3, 0), value);
		break;
	case ARMV8_ERRIDR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERRIDR_EL1, 0), value);
		break;
	case ARMV8_ERRSELR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERRSELR_EL1, 0), value);
		break;
	case ARMV8_ERXADDR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERXADDR_EL1, 0), value);
		break;
	case ARMV8_ERXCTLR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERXCTLR_EL1, 0), value);
		break;
	case ARMV8_ERXFR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERXFR_EL1, 0), value);
		break;
	case ARMV8_ERXMISC0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERXMISC0_EL1, 0), value);
		break;
	case ARMV8_ERXMISC1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERXMISC1_EL1, 0), value);
		break;
	case ARMV8_ERXSTATUS_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ERXSTATUS_EL1, 0), value);
		break;
	case ARMV8_HACR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_HACR_EL2, 0), value);
		break;
	case ARMV8_HSTR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_HSTR_EL2, 0), value);
		break;
	case ARMV8_PAR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PAR_EL1, 0), value);
		break;
	case ARMV8_SPSR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SPSR_EL1, 0), value);
		break;
	case ARMV8_SPSR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SPSR_EL2, 0), value);
		break;
	case ARMV8_SPSR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SPSR_EL3, 0), value);
		break;
	case ARMV8_FAR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_FAR_EL1, 0), value);
		break;
	case ARMV8_FAR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_FAR_EL2, 0), value);
		break;
	case ARMV8_FAR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_FAR_EL3, 0), value);
		break;
	case ARMV8_SCTLR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SCTLR_EL1, 0), value);
		break;
	case ARMV8_SCTLR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SCTLR_EL2, 0), value);
		break;
	case ARMV8_SCTLR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SCTLR_EL3, 0), value);
		break;
	case ARMV8_TTBR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TTBR0_EL1, 0), value);
		break;
	case ARMV8_TTBR0_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TTBR0_EL2, 0), value);
		break;
	case ARMV8_TTBR0_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TTBR0_EL3, 0), value);
		break;
	case ARMV8_VBAR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VBAR_EL1, 0), value);
		break;
	case ARMV8_VBAR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VBAR_EL2, 0), value);
		break;
	case ARMV8_VBAR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VBAR_EL3, 0), value);
		break;
	case ARMV8_VMPIDR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VMPIDR_EL2, 0), value);
		break;
	case ARMV8_VPIDR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VPIDR_EL2, 0), value);
		break;
	case ARMV8_ACTLR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ACTLR_EL1, 0), value);
		break;
	case ARMV8_ACTLR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ACTLR_EL2, 0), value);
		break;
	case ARMV8_ACTLR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ACTLR_EL3, 0), value);
		break;
	case ARMV8_AFSR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AFSR0_EL1, 0), value);
		break;
	case ARMV8_AFSR0_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AFSR0_EL2, 0), value);
		break;
	case ARMV8_AFSR0_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AFSR0_EL3, 0), value);
		break;
	case ARMV8_AFSR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AFSR1_EL1, 0), value);
		break;
	case ARMV8_AFSR1_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AFSR1_EL2, 0), value);
		break;
	case ARMV8_AFSR1_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_AFSR1_EL3, 0), value);
		break;
	case ARMV8_CONTEXTIDR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CONTEXTIDR_EL1, 0), value);
		break;
	case ARMV8_CONTEXTIDR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CONTEXTIDR_EL2, 0), value);
		break;
	case ARMV8_CNTFRQ_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTFRQ_EL0, 0), value);
		break;
	case ARMV8_CNTPCT_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTPCT_EL0, 0), value);
		break;
	case ARMV8_CNTVCT_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTVCT_EL0, 0), value);
		break;
	case ARMV8_CNTP_TVAL_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTP_TVAL_EL0, 0), value);
		break;
	case ARMV8_CNTP_CTL_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTP_CTL_EL0, 0), value);
		break;
	case ARMV8_CNTP_CVAL_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTP_CVAL_EL0, 0), value);
		break;
	case ARMV8_CNTV_TVAL_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTV_TVAL_EL0, 0), value);
		break;
	case ARMV8_CNTV_CTL_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTV_CTL_EL0, 0), value);
		break;
	case ARMV8_CNTV_CVAL_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTV_CVAL_EL0, 0), value);
		break;
	case ARMV8_CNTKCTL_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTKCTL_EL1, 0), value);
		break;
	case ARMV8_CNTPS_TVAL_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTPS_TVAL_EL1, 0), value);
		break;
	case ARMV8_CNTPS_CTL_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTPS_CTL_EL1, 0), value);
		break;
	case ARMV8_CNTPS_CVAL_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTPS_CVAL_EL1, 0), value);
		break;
	case ARMV8_CNTVOFF_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTVOFF_EL2, 0), value);
		break;
	case ARMV8_CNTHCTL_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTHCTL_EL2, 0), value);
		break;
	case ARMV8_CNTHP_TVAL_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTHP_TVAL_EL2, 0), value);
		break;
	case ARMV8_CNTHP_CTL_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTHP_CTL_EL2, 0), value);
		break;
	case ARMV8_CNTHP_CVAL_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTHP_CVAL_EL2, 0), value);
		break;
	case ARMV8_CNTHV_TVAL_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTHV_TVAL_EL2, 0), value);
		break;
	case ARMV8_CNTHV_CTL_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTHV_CTL_EL2, 0), value);
		break;
	case ARMV8_CNTHV_CVAL_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CNTHV_CVAL_EL2, 0), value);
		break;

	case ARMV8_CPACR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CPACR_EL1, 0), value);
		break;
	case ARMV8_CPTR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CPTR_EL2, 0), value);
		break;
	case ARMV8_CPTR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CPTR_EL3, 0), value);
		break;
	case ARMV8_CSSELR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_CSSELR_EL1, 0), value);
		break;
	case ARMV8_DACR32_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DACR32_EL2, 0), value);
		break;
	case ARMV8_DISR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DISR_EL1, 0), value);
		break;
	case ARMV8_HCR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_HCR_EL2, 0), value);
		break;
	case ARMV8_HPFAR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_HPFAR_EL2, 0), value);
		break;
	case ARMV8_IFSR32_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_IFSR32_EL2, 0), value);
		break;
	case ARMV8_ISR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ISR_EL1, 0), value);
		break;
	case ARMV8_MAIR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MAIR_EL1, 0), value);
		break;
	case ARMV8_MAIR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MAIR_EL2, 0), value);
		break;
	case ARMV8_MAIR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MAIR_EL3, 0), value);
		break;
	case ARMV8_SCR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SCR_EL3, 0), value);
		break;
	case ARMV8_TCR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TCR_EL1, 0), value);
		break;
	case ARMV8_TCR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TCR_EL2, 0), value);
		break;
	case ARMV8_TCR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TCR_EL3, 0), value);
		break;
	case ARMV8_TTBR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TTBR1_EL1, 0), value);
		break;
	case ARMV8_TTBR1_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TTBR1_EL2, 0), value);
		break;
	case ARMV8_VTCR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VTCR_EL2, 0), value);
		break;
	case ARMV8_VTTBR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VTTBR_EL2, 0), value);
		break;
	case ARMV8_RMR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_RMR_EL3, 0), value);
		break;
	case ARMV8_RVBAR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_RVBAR_EL3, 0), value);
		break;
	case ARMV8_SDER32_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_SDER32_EL3, 0), value);
		break;
	case ARMV8_TPIDRRO_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TPIDRRO_EL0, 0), value);
		break;
	case ARMV8_TPIDR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TPIDR_EL0, 0), value);
		break;
	case ARMV8_TPIDR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TPIDR_EL1, 0), value);
		break;
	case ARMV8_TPIDR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TPIDR_EL2, 0), value);
		break;
	case ARMV8_TPIDR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_TPIDR_EL3, 0), value);
		break;
	case ARMV8_VDISR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VDISR_EL2, 0), value);
		break;
	case ARMV8_VSESR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_VSESR_EL2, 0), value);
		break;

	case ARMV8_ICC_AP0R0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_AP0R0_EL1, 0), value);
		break;
	case ARMV8_ICC_AP1R0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_AP1R0_EL1, 0), value);
		break;
	case ARMV8_ICC_ASGI1R_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_ASGI1R_EL1, 0), value);
		break;
	case ARMV8_ICC_BPR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_BPR0_EL1, 0), value);
		break;
	case ARMV8_ICC_BPR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_BPR1_EL1, 0), value);
		break;
	case ARMV8_ICC_CTLR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_CTLR_EL1, 0), value);
		break;
	case ARMV8_ICC_DIR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_DIR_EL1, 0), value);
		break;
	case ARMV8_ICC_EOIR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_EOIR0_EL1, 0), value);
		break;
	case ARMV8_ICC_EOIR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_EOIR1_EL1, 0), value);
		break;
	case ARMV8_ICC_HPPIR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_HPPIR0_EL1, 0), value);
		break;
	case ARMV8_ICC_HPPIR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_HPPIR1_EL1, 0), value);
		break;
	case ARMV8_ICC_IAR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_IAR0_EL1, 0), value);
		break;
	case ARMV8_ICC_IAR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_IAR1_EL1, 0), value);
		break;
	case ARMV8_ICC_IGRPEN0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_IGRPEN0_EL1, 0), value);
		break;
	case ARMV8_ICC_IGRPEN1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_IGRPEN1_EL1, 0), value);
		break;
	case ARMV8_ICC_PMR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_PMR_EL1, 0), value);
		break;
	case ARMV8_ICC_RPR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_RPR_EL1, 0), value);
		break;
	case ARMV8_ICC_SGI0R_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_SGI0R_EL1, 0), value);
		break;
	case ARMV8_ICC_SGI1R_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_SGI1R_EL1, 0), value);
		break;
	case ARMV8_ICC_SRE_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_SRE_EL1, 0), value);
		break;
	case ARMV8_ICV_AP0R0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_AP0R0_EL1, 0), value);
		break;
	case ARMV8_ICV_AP1R0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_AP1R0_EL1, 0), value);
		break;
	case ARMV8_ICV_BPR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_BPR0_EL1, 0), value);
		break;
	case ARMV8_ICV_BPR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_BPR1_EL1, 0), value);
		break;
	case ARMV8_ICV_CTLR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_CTLR_EL1, 0), value);
		break;
	case ARMV8_ICV_DIR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_DIR_EL1, 0), value);
		break;
	case ARMV8_ICV_EOIR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_EOIR0_EL1, 0), value);
		break;
	case ARMV8_ICV_EOIR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_EOIR1_EL1, 0), value);
		break;
	case ARMV8_ICV_HPPIR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_HPPIR0_EL1, 0), value);
		break;
	case ARMV8_ICV_HPPIR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_HPPIR1_EL1, 0), value);
		break;
	case ARMV8_ICV_IAR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_IAR0_EL1, 0), value);
		break;
	case ARMV8_ICV_IAR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_IAR1_EL1, 0), value);
		break;
	case ARMV8_ICV_IGRPEN0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_IGRPEN0_EL1, 0), value);
		break;
	case ARMV8_ICV_IGRPEN1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_IGRPEN1_EL1, 0), value);
		break;
	case ARMV8_ICV_PMR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_PMR_EL1, 0), value);
		break;
	case ARMV8_ID_AA64AFR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64AFR0_EL1, 0), value);
		break;
	case ARMV8_ID_AA64AFR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64AFR1_EL1, 0), value);
		break;
	case ARMV8_ID_AA64DFR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64DFR0_EL1, 0), value);
		break;
	case ARMV8_ID_AA64DFR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64DFR1_EL1, 0), value);
		break;
	case ARMV8_ID_AA64ISAR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64ISAR0_EL1, 0), value);
		break;
	case ARMV8_ID_AA64ISAR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64ISAR1_EL1, 0), value);
		break;
	case ARMV8_ID_AA64MMFR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64MMFR0_EL1, 0), value);
		break;
	case ARMV8_ID_AA64MMFR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64MMFR1_EL1, 0), value);
		break;
	case ARMV8_ID_AA64MMFR2_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64MMFR2_EL1, 0), value);
		break;
	case ARMV8_ID_AA64PFR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64PFR0_EL1, 0), value);
		break;
	case ARMV8_ID_AA64PFR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AA64PFR1_EL1, 0), value);
		break;
	case ARMV8_ID_AFR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_AFR0_EL1, 0), value);
		break;
	case ARMV8_ID_ISAR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_ISAR0_EL1, 0), value);
		break;
	case ARMV8_ID_ISAR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_ISAR1_EL1, 0), value);
		break;
	case ARMV8_ID_ISAR2_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_ISAR2_EL1, 0), value);
		break;
	case ARMV8_ID_ISAR3_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_ISAR3_EL1, 0), value);
		break;
	case ARMV8_ID_ISAR4_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_ISAR4_EL1, 0), value);
		break;
	case ARMV8_ID_ISAR5_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_ISAR5_EL1, 0), value);
		break;

	case ARMV8_LORID_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_LORID_EL1, 0), value);
		break;

	case ARMV8_ID_MMFR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_MMFR0_EL1, 0), value);
		break;
	case ARMV8_ID_MMFR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_MMFR1_EL1, 0), value);
		break;
	case ARMV8_ID_MMFR2_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_MMFR2_EL1, 0), value);
		break;
	case ARMV8_ID_MMFR3_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_MMFR3_EL1, 0), value);
		break;
	case ARMV8_ID_MMFR4_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_MMFR4_EL1, 0), value);
		break;
	case ARMV8_ID_PFR0_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_PFR0_EL1, 0), value);
		break;
	case ARMV8_ID_PFR1_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ID_PFR1_EL1, 0), value);
		break;
	case ARMV8_ICV_RPR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICV_RPR_EL1, 0), value);
		break;
	case ARMV8_ICH_AP0R0_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_AP0R0_EL2, 0), value);
		break;
	case ARMV8_ICH_AP1R0_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_AP1R0_EL2, 0), value);
		break;
	case ARMV8_ICH_EISR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_EISR_EL2, 0), value);
		break;
	case ARMV8_ICH_ELRSR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_ELRSR_EL2, 0), value);
		break;
	case ARMV8_ICH_HCR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_HCR_EL2, 0), value);
		break;
	case ARMV8_ICH_LR0_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_LR0_EL2, 0), value);
		break;
	case ARMV8_ICH_LR1_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_LR1_EL2, 0), value);
		break;
	case ARMV8_ICH_LR2_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_LR2_EL2, 0), value);
		break;
	case ARMV8_ICH_LR3_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_LR3_EL2, 0), value);
		break;
	case ARMV8_ICH_MISR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_MISR_EL2, 0), value);
		break;
	case ARMV8_ICH_VMCR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_VMCR_EL2, 0), value);
		break;
	case ARMV8_ICH_VTR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICH_VTR_EL2, 0), value);
		break;
	case ARMV8_ICC_SRE_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_SRE_EL2, 0), value);
		break;
	case ARMV8_ICC_CTLR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_CTLR_EL3, 0), value);
		break;
	case ARMV8_ICC_IGRPEN1_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_IGRPEN1_EL3, 0), value);
		break;
	case ARMV8_ICC_SRE_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_ICC_SRE_EL3, 0), value);
		break;
	case ARMV8_MDCCINT_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MDCCINT_EL1, 0), value);
		break;
	case ARMV8_MDCCSR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MDCCSR_EL0, 0), value);
		break;
	case ARMV8_MDSCR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MDSCR_EL1, 0), value);
		break;
	case ARMV8_MDCR_EL2:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MDCR_EL2, 0), value);
		break;
	case ARMV8_MDCR_EL3:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_MDCR_EL3, 0), value);
		break;
	case ARMV8_OSDLR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_OSDLR_EL1, 0), value);
		break;
	case ARMV8_OSDTRRX_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_OSDTRRX_EL1, 0), value);
		break;
	case ARMV8_OSDTRTX_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_OSDTRTX_EL1, 0), value);
		break;
	case ARMV8_OSECCR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_OSECCR_EL1, 0), value);
		break;

	case ARMV8_OSLSR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_OSLSR_EL1, 0), value);
		break;
	case ARMV8_PMCCFILTR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMCCFILTR_EL0, 0), value);
		break;
	case ARMV8_PMCCNTR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMCCNTR_EL0, 0), value);
		break;
	case ARMV8_PMCEID0_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMCEID0_EL0, 0), value);
		break;
	case ARMV8_PMCEID1_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMCEID1_EL0, 0), value);
		break;
	case ARMV8_PMCNTENCLR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMCNTENCLR_EL0, 0), value);
		break;
	case ARMV8_PMCNTENSET_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMCNTENSET_EL0, 0), value);
		break;
	case ARMV8_PMCR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMCR_EL0, 0), value);
		break;
	case ARMV8_PMEVCNTR0_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVCNTR0_EL0, 0), value);
		break;
	case ARMV8_PMEVCNTR1_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVCNTR1_EL0, 0), value);
		break;
	case ARMV8_PMEVCNTR2_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVCNTR2_EL0, 0), value);
		break;
	case ARMV8_PMEVCNTR3_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVCNTR3_EL0, 0), value);
		break;
	case ARMV8_PMEVCNTR4_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVCNTR4_EL0, 0), value);
		break;
	case ARMV8_PMEVCNTR5_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVCNTR5_EL0, 0), value);
		break;
	case ARMV8_PMEVTYPER0_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVTYPER0_EL0, 0), value);
		break;
	case ARMV8_PMEVTYPER1_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVTYPER1_EL0, 0), value);
		break;
	case ARMV8_PMEVTYPER2_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVTYPER2_EL0, 0), value);
		break;
	case ARMV8_PMEVTYPER3_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVTYPER3_EL0, 0), value);
		break;
	case ARMV8_PMEVTYPER4_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVTYPER4_EL0, 0), value);
		break;
	case ARMV8_PMEVTYPER5_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMEVTYPER5_EL0, 0), value);
		break;
	case ARMV8_PMINTENCLR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMINTENCLR_EL1, 0), value);
		break;
	case ARMV8_PMINTENSET_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMINTENSET_EL1, 0), value);
		break;
	case ARMV8_PMOVSCLR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMOVSCLR_EL0, 0), value);
		break;
	case ARMV8_PMOVSSET_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMOVSSET_EL0, 0), value);
		break;
	case ARMV8_PMSELR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMSELR_EL0, 0), value);
		break;
	case ARMV8_PMUSERENR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMUSERENR_EL0, 0), value);
		break;
	case ARMV8_PMXEVCNTR_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMXEVCNTR_EL0, 0), value);
		break;
	case ARMV8_PMXEVTYPER_EL0:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_PMXEVTYPER_EL0, 0), value);
		break;
	case ARMV8_REVIDR_EL1:
		value = value_64;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_GP(SYSTEM_REVIDR_EL1, 0), value);
		break;
	default:
		retval = ERROR_FAIL;
		break;
	}

	return retval;
}

static int armv8_write_reg_simdfp_aarch64(struct armv8_common *armv8, int regnum, uint64_t lvalue, uint64_t hvalue)
{
	int retval = ERROR_FAIL;
	struct arm_dpm *dpm = &armv8->dpm;

	switch (regnum) {
	case ARMV8_V0 ... ARMV8_V31:
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MOV_VFP_GPR((regnum - ARMV8_V0), 0, 1), hvalue);
		if (retval != ERROR_OK)
			return retval;
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MOV_VFP_GPR((regnum - ARMV8_V0), 0, 0), lvalue);
		break;

	default:
		retval = ERROR_FAIL;
		break;
	}

	return retval;
}

static int armv8_read_reg32(struct armv8_common *armv8, int regnum, uint64_t *regval)
{
	struct arm_dpm *dpm = &armv8->dpm;
	uint32_t value = 0;
	int retval;

	switch (regnum) {
	case ARMV8_R0 ... ARMV8_R14:
		/* return via DCC:  "MCR p14, 0, Rnum, c0, c5, 0" */
		retval = dpm->instr_read_data_dcc(dpm,
			ARMV4_5_MCR(14, 0, regnum, 0, 5, 0),
			&value);
		break;
	case ARMV8_SP:
		retval = dpm->instr_read_data_dcc(dpm,
			ARMV4_5_MCR(14, 0, 13, 0, 5, 0),
			&value);
		break;
	case ARMV8_PC:
		retval = dpm->instr_read_data_r0(dpm,
			ARMV8_MRC_DLR(0),
			&value);
		break;
	case ARMV8_XPSR:
		retval = dpm->instr_read_data_r0(dpm,
			ARMV8_MRC_DSPSR(0),
			&value);
		break;
	case ARMV8_ELR_EL1: /* mapped to LR_svc */
		retval = instr_read_data_dcc(dpm,
				ARMV4_5_MCR(14, 0, 14, 0, 5, 0),
				&value, 1);
		break;
	case ARMV8_ELR_EL2: /* mapped to ELR_hyp */
		retval = instr_read_data_r0_32(dpm,
				ARMV8_MRS_T1(0, 14, 0, 1),
				&value, 2);
		break;
	case ARMV8_ELR_EL3: /* mapped to LR_mon */
		retval = instr_read_data_dcc(dpm,
				ARMV4_5_MCR(14, 0, 14, 0, 5, 0),
				&value, 3);
		break;
	case ARMV8_ESR_EL1: /* mapped to DFSR */
		retval = instr_read_data_r0_32(dpm,
				ARMV4_5_MRC(15, 0, 0, 5, 0, 0),
				&value, 1);
		break;
	case ARMV8_ESR_EL2: /* mapped to HSR */
		retval = instr_read_data_r0_32(dpm,
				ARMV4_5_MRC(15, 4, 0, 5, 2, 0),
				&value, 2);
		break;
	case ARMV8_ESR_EL3: /* FIXME: no equivalent in aarch32? */
		retval = ERROR_FAIL;
		break;
	case ARMV8_SPSR_EL1: /* mapped to SPSR_svc */
		retval = instr_read_data_r0_32(dpm,
				ARMV8_MRS_XPSR_T1(1, 0),
				&value, 1);
		break;
	case ARMV8_SPSR_EL2: /* mapped to SPSR_hyp */
		retval = instr_read_data_r0_32(dpm,
				ARMV8_MRS_XPSR_T1(1, 0),
				&value, 2);
		break;
	case ARMV8_SPSR_EL3: /* mapped to SPSR_mon */
		retval = instr_read_data_r0_32(dpm,
				ARMV8_MRS_XPSR_T1(1, 0),
				&value, 3);
		break;
	case ARMV8_FPSR:
		/* "VMRS r0, FPSCR"; then return via DCC */
		retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_VMRS(0), &value);
		break;
	default:
		retval = ERROR_FAIL;
		break;
	}

	if (retval == ERROR_OK && regval != NULL) {
		*regval = value;
	}
	else if (retval == ERROR_TARGET_EXCEPTION_LEVEL && regval != NULL) {
		*regval = 0xDEADBEEF;
	}
	else
		retval = ERROR_FAIL;

	return retval;
}

static int armv8_read_reg_simdfp_aarch32(struct armv8_common *armv8, int regnum, uint64_t *lvalue, uint64_t *hvalue)
{
	int retval = ERROR_FAIL;
	struct arm_dpm *dpm = &armv8->dpm;
	struct reg *reg_r1 = dpm->arm->core_cache->reg_list + ARMV8_R1;
	uint32_t value_r0 = 0, value_r1 = 0;
	unsigned num = (regnum - ARMV8_V0) << 1;

	switch (regnum) {
	case ARMV8_V0 ... ARMV8_V15:
		/* we are going to write R1, mark it dirty */
		reg_r1->dirty = true;
		/* move from double word register to r0:r1: "vmov r0, r1, vm"
		 * then read r0 via dcc
		 */
		retval = dpm->instr_read_data_r0(dpm,
				ARMV4_5_VMOV(1, 1, 0, (num >> 4), (num & 0xf)),
				&value_r0);
		if (retval != ERROR_OK)
			return retval;
		/* read r1 via dcc */
		retval = dpm->instr_read_data_dcc(dpm,
				ARMV4_5_MCR(14, 0, 1, 0, 5, 0),
				&value_r1);
		if (retval != ERROR_OK)
			return retval;
		*lvalue = value_r1;
		*lvalue = ((*lvalue) << 32) | value_r0;

		num++;
		/* repeat above steps for high 64 bits of V register */
		retval = dpm->instr_read_data_r0(dpm,
				ARMV4_5_VMOV(1, 1, 0, (num >> 4), (num & 0xf)),
				&value_r0);
		if (retval != ERROR_OK)
			return retval;
		retval = dpm->instr_read_data_dcc(dpm,
				ARMV4_5_MCR(14, 0, 1, 0, 5, 0),
				&value_r1);
		if (retval != ERROR_OK)
			return retval;
		*hvalue = value_r1;
		*hvalue = ((*hvalue) << 32) | value_r0;
		break;
	default:
		retval = ERROR_FAIL;
		break;
	}

	return retval;
}

static int armv8_write_reg32(struct armv8_common *armv8, int regnum, uint64_t value)
{
	struct arm_dpm *dpm = &armv8->dpm;
	int retval;

	switch (regnum) {
	case ARMV8_R0 ... ARMV8_R14:
		/* load register from DCC:  "MRC p14, 0, Rnum, c0, c5, 0" */
		retval = dpm->instr_write_data_dcc(dpm,
				ARMV4_5_MRC(14, 0, regnum, 0, 5, 0), value);
		break;
	case ARMV8_SP:
		retval = dpm->instr_write_data_dcc(dpm,
				ARMV4_5_MRC(14, 0, 13, 0, 5, 0), value);
			break;
	case ARMV8_PC:/* PC
		 * read r0 from DCC; then "MOV pc, r0" */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MCR_DLR(0), value);
		break;
	case ARMV8_XPSR: /* CPSR */
		/* read r0 from DCC, then "MCR r0, DSPSR" */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MCR_DSPSR(0), value);
		break;
	case ARMV8_ELR_EL1: /* mapped to LR_svc */
		retval = dpm->instr_write_data_dcc(dpm,
				ARMV4_5_MRC(14, 0, 14, 0, 5, 0),
				value);
		break;
	case ARMV8_ELR_EL2: /* mapped to ELR_hyp */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MSR_GP_T1(0, 14, 0, 1),
				value);
		break;
	case ARMV8_ELR_EL3: /* mapped to LR_mon */
		retval = dpm->instr_write_data_dcc(dpm,
				ARMV4_5_MRC(14, 0, 14, 0, 5, 0),
				value);
		break;
	case ARMV8_ESR_EL1: /* mapped to DFSR */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 5, 0, 0),
				value);
		break;
	case ARMV8_ESR_EL2: /* mapped to HSR */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 4, 0, 5, 2, 0),
				value);
		break;
	case ARMV8_ESR_EL3: /* FIXME: no equivalent in aarch32? */
		retval = ERROR_FAIL;
		break;
	case ARMV8_SPSR_EL1: /* mapped to SPSR_svc */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MSR_GP_XPSR_T1(1, 0, 15),
				value);
		break;
	case ARMV8_SPSR_EL2: /* mapped to SPSR_hyp */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MSR_GP_XPSR_T1(1, 0, 15),
				value);
		break;
	case ARMV8_SPSR_EL3: /* mapped to SPSR_mon */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MSR_GP_XPSR_T1(1, 0, 15),
				value);
		break;
	case ARMV8_FPSR:
		/* move to r0 from DCC, then "VMSR FPSCR, r0" */
		retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_VMSR(0), value);
		break;
	default:
		retval = ERROR_FAIL;
		break;
	}

	return retval;

}

static int armv8_write_reg_simdfp_aarch32(struct armv8_common *armv8, int regnum, uint64_t lvalue, uint64_t hvalue)
{
	int retval = ERROR_FAIL;
	struct arm_dpm *dpm = &armv8->dpm;
	struct reg *reg_r1 = dpm->arm->core_cache->reg_list + ARMV8_R1;
	uint32_t value_r0 = 0, value_r1 = 0;
	unsigned num = (regnum - ARMV8_V0) << 1;

	switch (regnum) {
	case ARMV8_V0 ... ARMV8_V15:
		/* we are going to write R1, mark it dirty */
		reg_r1->dirty = true;
		value_r1 = lvalue >> 32;
		value_r0 = lvalue & 0xFFFFFFFF;
		/* write value_r1 to r1 via dcc */
		retval = dpm->instr_write_data_dcc(dpm,
			ARMV4_5_MRC(14, 0, 1, 0, 5, 0),
			value_r1);
		if (retval != ERROR_OK)
			return retval;
		/* write value_r0 to r0 via dcc then,
		 * move to double word register from r0:r1: "vmov vm, r0, r1"
		 */
		retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_VMOV(0, 1, 0, (num >> 4), (num & 0xf)),
			value_r0);
		if (retval != ERROR_OK)
			return retval;

		num++;
		/* repeat above steps for high 64 bits of V register */
		value_r1 = hvalue >> 32;
		value_r0 = hvalue & 0xFFFFFFFF;
		retval = dpm->instr_write_data_dcc(dpm,
			ARMV4_5_MRC(14, 0, 1, 0, 5, 0),
			value_r1);
		if (retval != ERROR_OK)
			return retval;
		retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_VMOV(0, 1, 0, (num >> 4), (num & 0xf)),
			value_r0);
		break;
	default:
		retval = ERROR_FAIL;
		break;
	}

	return retval;
}

void armv8_select_reg_access(struct armv8_common *armv8, bool is_aarch64)
{
	if (is_aarch64) {
		armv8->read_reg_u64 = armv8_read_reg;
		armv8->write_reg_u64 = armv8_write_reg;
		armv8->read_reg_u128 = armv8_read_reg_simdfp_aarch64;
		armv8->write_reg_u128 = armv8_write_reg_simdfp_aarch64;

	} else {
		armv8->read_reg_u64 = armv8_read_reg32;
		armv8->write_reg_u64 = armv8_write_reg32;
		armv8->read_reg_u128 = armv8_read_reg_simdfp_aarch32;
		armv8->write_reg_u128 = armv8_write_reg_simdfp_aarch32;
	}
}

/*  retrieve core id cluster id  */
int armv8_read_mpidr(struct armv8_common *armv8)
{
	int retval = ERROR_FAIL;
	struct arm *arm = &armv8->arm;
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t mpidr;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* check if we're in an unprivileged mode */
	if (armv8_curel_from_core_mode(arm->core_mode) < SYSTEM_CUREL_EL1) {
		retval = armv8_dpm_modeswitch(dpm, ARMV8_64_EL1H);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = dpm->instr_read_data_r0(dpm, armv8_opcode(armv8, READ_REG_MPIDR), &mpidr);
	if (retval != ERROR_OK)
		goto done;
	if (mpidr & 1U<<31) {
		armv8->multi_processor_system = (mpidr >> 30) & 1;
		armv8->cluster_id = (mpidr >> 8) & 0xf;
		armv8->cpu_id = mpidr & 0x3;
		LOG_INFO("%s cluster %x core %x %s", target_name(armv8->arm.target),
			armv8->cluster_id,
			armv8->cpu_id,
			armv8->multi_processor_system == 0 ? "multi core" : "single core");
	} else
		LOG_ERROR("mpidr not in multiprocessor format");

done:
	armv8_dpm_modeswitch(dpm, ARM_MODE_ANY);
	dpm->finish(dpm);
	return retval;
}

/**
 * Configures host-side ARM records to reflect the specified CPSR.
 * Later, code can use arm_reg_current() to map register numbers
 * according to how they are exposed by this mode.
 */
void armv8_set_cpsr(struct arm *arm, uint32_t cpsr)
{
	uint32_t mode = cpsr & 0x1F;

	/* NOTE:  this may be called very early, before the register
	 * cache is set up.  We can't defend against many errors, in
	 * particular against CPSRs that aren't valid *here* ...
	 */
	if (arm->cpsr) {
		buf_set_u32(arm->cpsr->value, 0, 32, cpsr);
		arm->cpsr->valid = true;
		arm->cpsr->dirty = false;
	}

	/* Older ARMs won't have the J bit */
	enum arm_state state = 0xFF;

	if ((cpsr & 0x10) != 0) {
		/* Aarch32 state */
		if (cpsr & (1 << 5)) {	/* T */
			if (cpsr & (1 << 24)) { /* J */
				LOG_WARNING("ThumbEE -- incomplete support");
				state = ARM_STATE_THUMB_EE;
			} else
				state = ARM_STATE_THUMB;
		} else {
			if (cpsr & (1 << 24)) { /* J */
				LOG_ERROR("Jazelle state handling is BROKEN!");
				state = ARM_STATE_JAZELLE;
			} else
				state = ARM_STATE_ARM;
		}
	} else {
		/* Aarch64 state */
		state = ARM_STATE_AARCH64;
	}

	arm->core_state = state;
	arm->core_mode = mode;

	LOG_DEBUG("set CPSR %#8.8x: %s mode, %s state", (unsigned) cpsr,
		armv8_mode_name(arm->core_mode),
		armv8_state_strings[arm->core_state]);
}

static void armv8_show_fault_registers32(struct armv8_common *armv8)
{
	uint32_t dfsr, ifsr, dfar, ifar;
	struct arm_dpm *dpm = armv8->arm.dpm;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return;

	/* ARMV4_5_MRC(cpnum, op1, r0, crn, crm, op2) */

	/* c5/c0 - {data, instruction} fault status registers */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 5, 0, 0),
			&dfsr);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 5, 0, 1),
			&ifsr);
	if (retval != ERROR_OK)
		goto done;

	/* c6/c0 - {data, instruction} fault address registers */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 6, 0, 0),
			&dfar);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 6, 0, 2),
			&ifar);
	if (retval != ERROR_OK)
		goto done;

	LOG_USER("Data fault registers        DFSR: %8.8" PRIx32
		", DFAR: %8.8" PRIx32, dfsr, dfar);
	LOG_USER("Instruction fault registers IFSR: %8.8" PRIx32
		", IFAR: %8.8" PRIx32, ifsr, ifar);

done:
	/* (void) */ dpm->finish(dpm);
}

static __attribute__((unused)) void armv8_show_fault_registers(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);

	if (armv8->arm.core_state != ARM_STATE_AARCH64)
		armv8_show_fault_registers32(armv8);
}

static uint8_t armv8_pa_size(uint32_t ps)
{
	uint8_t ret = 0;
	switch (ps) {
		case 0:
			ret = 32;
			break;
		case 1:
			ret = 36;
			break;
		case 2:
			ret = 40;
			break;
		case 3:
			ret = 42;
			break;
		case 4:
			ret = 44;
			break;
		case 5:
			ret = 48;
			break;
		default:
			LOG_INFO("Unknown physical address size");
			break;
	}
	return ret;
}

static __attribute__((unused)) int armv8_read_ttbcr32(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t ttbcr, ttbcr_n;
	int retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/*  MRC p15,0,<Rt>,c2,c0,2 ; Read CP15 Translation Table Base Control Register*/
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 2, 0, 2),
			&ttbcr);
	if (retval != ERROR_OK)
		goto done;

	LOG_DEBUG("ttbcr %" PRIx32, ttbcr);

	ttbcr_n = ttbcr & 0x7;
	armv8->armv8_mmu.ttbcr = ttbcr;

	/*
	 * ARM Architecture Reference Manual (ARMv7-A and ARMv7-R edition),
	 * document # ARM DDI 0406C
	 */
	armv8->armv8_mmu.ttbr_range[0]  = 0xffffffff >> ttbcr_n;
	armv8->armv8_mmu.ttbr_range[1] = 0xffffffff;
	armv8->armv8_mmu.ttbr_mask[0] = 0xffffffff << (14 - ttbcr_n);
	armv8->armv8_mmu.ttbr_mask[1] = 0xffffffff << 14;

	LOG_DEBUG("ttbr1 %s, ttbr0_mask %" PRIx32 " ttbr1_mask %" PRIx32,
		  (ttbcr_n != 0) ? "used" : "not used",
		  armv8->armv8_mmu.ttbr_mask[0],
		  armv8->armv8_mmu.ttbr_mask[1]);

done:
	dpm->finish(dpm);
	return retval;
}

static __attribute__((unused)) int armv8_read_ttbcr(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	struct arm *arm = &armv8->arm;
	uint32_t ttbcr;
	uint64_t ttbcr_64;

	int retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* clear ttrr1_used and ttbr0_mask */
	memset(&armv8->armv8_mmu.ttbr1_used, 0, sizeof(armv8->armv8_mmu.ttbr1_used));
	memset(&armv8->armv8_mmu.ttbr0_mask, 0, sizeof(armv8->armv8_mmu.ttbr0_mask));

	switch (armv8_curel_from_core_mode(arm->core_mode)) {
	case SYSTEM_CUREL_EL3:
		retval = dpm->instr_read_data_r0(dpm,
				ARMV8_MRS(SYSTEM_TCR_EL3, 0),
				&ttbcr);
		retval += dpm->instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR0_EL3, 0),
				&armv8->ttbr_base);
		if (retval != ERROR_OK)
			goto done;
		armv8->va_size = 64 - (ttbcr & 0x3F);
		armv8->pa_size = armv8_pa_size((ttbcr >> 16) & 7);
		armv8->page_size = (ttbcr >> 14) & 3;
		break;
	case SYSTEM_CUREL_EL2:
		retval = dpm->instr_read_data_r0(dpm,
				ARMV8_MRS(SYSTEM_TCR_EL2, 0),
				&ttbcr);
		retval += dpm->instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR0_EL2, 0),
				&armv8->ttbr_base);
		if (retval != ERROR_OK)
			goto done;
		armv8->va_size = 64 - (ttbcr & 0x3F);
		armv8->pa_size = armv8_pa_size((ttbcr >> 16) & 7);
		armv8->page_size = (ttbcr >> 14) & 3;
		break;
	case SYSTEM_CUREL_EL0:
		armv8_dpm_modeswitch(dpm, ARMV8_64_EL1H);
		/* fall through */
	case SYSTEM_CUREL_EL1:
		retval = dpm->instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TCR_EL1, 0),
				&ttbcr_64);
		armv8->va_size = 64 - (ttbcr_64 & 0x3F);
		armv8->pa_size = armv8_pa_size((ttbcr_64 >> 32) & 7);
		armv8->page_size = (ttbcr_64 >> 14) & 3;
		armv8->armv8_mmu.ttbr1_used = (((ttbcr_64 >> 16) & 0x3F) != 0) ? 1 : 0;
		armv8->armv8_mmu.ttbr0_mask  = 0x0000FFFFFFFFFFFF;
		retval += dpm->instr_read_data_r0_64(dpm,
				ARMV8_MRS(SYSTEM_TTBR0_EL1 | (armv8->armv8_mmu.ttbr1_used), 0),
				&armv8->ttbr_base);
		if (retval != ERROR_OK)
			goto done;
		break;
	default:
		LOG_ERROR("unknown core state");
		retval = ERROR_FAIL;
		break;
	}
	if (retval != ERROR_OK)
		goto done;

	if (armv8->armv8_mmu.ttbr1_used == 1)
		LOG_INFO("TTBR0 access above %" PRIx64, (uint64_t)(armv8->armv8_mmu.ttbr0_mask));

done:
	armv8_dpm_modeswitch(dpm, ARM_MODE_ANY);
	dpm->finish(dpm);
	return retval;
}

/*  method adapted to cortex A : reused arm v4 v5 method*/
int armv8_mmu_translate_va(struct target *target,  target_addr_t va, target_addr_t *val)
{
	return ERROR_OK;
}

/*  V8 method VA TO PA  */
int armv8_mmu_translate_va_pa(struct target *target, target_addr_t va,
	target_addr_t *val, int meminfo)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = &armv8->dpm;
	enum arm_mode target_mode = ARM_MODE_ANY;
	uint32_t retval;
	uint32_t instr = 0;
	uint64_t par;

	static const char * const shared_name[] = {
			"Non-", "UNDEFINED ", "Outer ", "Inner "
	};

	static const char * const secure_name[] = {
			"Secure", "Not Secure"
	};

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target %s not halted", target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	switch (armv8_curel_from_core_mode(arm->core_mode)) {
	case SYSTEM_CUREL_EL0:
		instr = ARMV8_SYS(SYSTEM_ATS12E0R, 0);
		/* can only execute instruction at EL2 */
		target_mode = ARMV8_64_EL2H;
		break;
	case SYSTEM_CUREL_EL1:
		instr = ARMV8_SYS(SYSTEM_ATS12E1R, 0);
		/* can only execute instruction at EL2 */
		target_mode = ARMV8_64_EL2H;
		break;
	case SYSTEM_CUREL_EL2:
		instr = ARMV8_SYS(SYSTEM_ATS1E2R, 0);
		break;
	case SYSTEM_CUREL_EL3:
		instr = ARMV8_SYS(SYSTEM_ATS1E3R, 0);
		break;

	default:
		break;
	};

	if (target_mode != ARM_MODE_ANY)
		armv8_dpm_modeswitch(dpm, target_mode);

	/* write VA to R0 and execute translation instruction */
	retval = dpm->instr_write_data_r0_64(dpm, instr, (uint64_t)va);
	/* read result from PAR_EL1 */
	if (retval == ERROR_OK)
		retval = dpm->instr_read_data_r0_64(dpm, ARMV8_MRS(SYSTEM_PAR_EL1, 0), &par);

	/* switch back to saved PE mode */
	if (target_mode != ARM_MODE_ANY)
		armv8_dpm_modeswitch(dpm, ARM_MODE_ANY);

	dpm->finish(dpm);

	if (retval != ERROR_OK)
		return retval;

	if (par & 1) {
		LOG_ERROR("Address translation failed at stage %i, FST=%x, PTW=%i",
				((int)(par >> 9) & 1)+1, (int)(par >> 1) & 0x3f, (int)(par >> 8) & 1);

		*val = 0;
		retval = ERROR_FAIL;
	} else {
		*val = (par & 0xFFFFFFFFF000UL) | (va & 0xFFF);
		if (meminfo) {
			int SH = (par >> 7) & 3;
			int NS = (par >> 9) & 1;
			int ATTR = (par >> 56) & 0xFF;

			char *memtype = (ATTR & 0xF0) == 0 ? "Device Memory" : "Normal Memory";

			LOG_USER("%sshareable, %s",
					shared_name[SH], secure_name[NS]);
			LOG_USER("%s", memtype);
		}
	}

	return retval;
}

COMMAND_HANDLER(armv8_handle_exception_catch_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv8_common *armv8 = target_to_armv8(target);
	uint32_t edeccr = 0;
	unsigned int argp = 0;
	int retval;

	static const struct jim_nvp nvp_ecatch_modes[] = {
		{ .name = "off",       .value = 0 },
		{ .name = "nsec_el1",  .value = (1 << 5) },
		{ .name = "nsec_el2",  .value = (2 << 5) },
		{ .name = "nsec_el12", .value = (3 << 5) },
		{ .name = "sec_el1",   .value = (1 << 1) },
		{ .name = "sec_el3",   .value = (4 << 1) },
		{ .name = "sec_el13",  .value = (5 << 1) },
		{ .name = NULL, .value = -1 },
	};
	const struct jim_nvp *n;

	if (CMD_ARGC == 0) {
		const char *sec = NULL, *nsec = NULL;

		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
					armv8->debug_base + CPUV8_DBG_ECCR, &edeccr);
		if (retval != ERROR_OK)
			return retval;

		n = jim_nvp_value2name_simple(nvp_ecatch_modes, edeccr & 0x0f);
		if (n->name)
			sec = n->name;

		n = jim_nvp_value2name_simple(nvp_ecatch_modes, edeccr & 0xf0);
		if (n->name)
			nsec = n->name;

		if (!sec || !nsec) {
			LOG_WARNING("Exception Catch: unknown exception catch configuration: EDECCR = %02" PRIx32, edeccr & 0xff);
			return ERROR_FAIL;
		}

		command_print(CMD, "Exception Catch: Secure: %s, Non-Secure: %s", sec, nsec);
		return ERROR_OK;
	}

	while (argp < CMD_ARGC) {
		n = jim_nvp_name2value_simple(nvp_ecatch_modes, CMD_ARGV[argp]);
		if (!n->name) {
			LOG_ERROR("Unknown option: %s", CMD_ARGV[argp]);
			return ERROR_FAIL;
		}

		LOG_DEBUG("found: %s", n->name);

		edeccr |= n->value;
		argp++;
	}

	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_ECCR, edeccr);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int armv8_handle_cache_info_command(struct command_invocation *cmd,
	struct armv8_cache_common *armv8_cache)
{
	if (armv8_cache->info == -1) {
		command_print(cmd, "cache not yet identified");
		return ERROR_OK;
	}

	if (armv8_cache->display_cache_info)
		armv8_cache->display_cache_info(cmd, armv8_cache);
	return ERROR_OK;
}

static int armv8_setup_semihosting(struct target *target, int enable)
{
	return ERROR_OK;
}

int armv8_init_arch_info(struct target *target, struct armv8_common *armv8)
{
	struct arm *arm = &armv8->arm;
	arm->arch_info = armv8;
	target->arch_info = &armv8->arm;
	arm->setup_semihosting = armv8_setup_semihosting;
	/*  target is useful in all function arm v4 5 compatible */
	armv8->arm.target = target;
	armv8->arm.common_magic = ARM_COMMON_MAGIC;
	armv8->common_magic = ARMV8_COMMON_MAGIC;

	armv8->armv8_mmu.armv8_cache.l2_cache = NULL;
	armv8->armv8_mmu.armv8_cache.info = -1;
	armv8->armv8_mmu.armv8_cache.flush_all_data_cache = NULL;
	armv8->armv8_mmu.armv8_cache.display_cache_info = NULL;
	return ERROR_OK;
}

static int armv8_aarch64_state(struct target *target)
{
	struct arm *arm = target_to_arm(target);

	if (arm->common_magic != ARM_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-ARM target");
		return ERROR_FAIL;
	}

	LOG_USER("%s halted in %s state due to %s, current mode: %s\n"
		"pc: 0x%" PRIx64 "%s",
		target_name(target),
		armv8_state_strings[arm->core_state],
		debug_reason_name(target),
		armv8_mode_name(arm->core_mode),
		buf_get_u64(arm->pc->value, 0, 64),
		(target->semihosting && target->semihosting->is_active) ? ", semihosting" : "");

	return ERROR_OK;
}

int armv8_arch_state(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;

	if (armv8->common_magic != ARMV8_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-Armv8 target");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (arm->core_state == ARM_STATE_AARCH64)
		armv8_aarch64_state(target);
	else
		arm_arch_state(target);

	if (arm->core_mode == ARM_MODE_ABT)
		armv8_show_fault_registers(target);

	if (target->debug_reason == DBG_REASON_WATCHPOINT)
		LOG_USER("Watchpoint triggered at " TARGET_ADDR_FMT, armv8->dpm.wp_addr);

	return ERROR_OK;
}

static struct reg_data_type aarch64_vector_base_types[] = {
	{REG_TYPE_IEEE_DOUBLE, "ieee_double", 0, {NULL} },
	{REG_TYPE_UINT64, "uint64", 0, {NULL} },
	{REG_TYPE_INT64, "int64", 0, {NULL} },
	{REG_TYPE_IEEE_SINGLE, "ieee_single", 0, {NULL} },
	{REG_TYPE_UINT32, "uint32", 0, {NULL} },
	{REG_TYPE_INT32, "int32", 0, {NULL} },
	{REG_TYPE_UINT16, "uint16", 0, {NULL} },
	{REG_TYPE_INT16, "int16", 0, {NULL} },
	{REG_TYPE_UINT8, "uint8", 0, {NULL} },
	{REG_TYPE_INT8, "int8", 0, {NULL} },
	{REG_TYPE_UINT128, "uint128", 0, {NULL} },
	{REG_TYPE_INT128, "int128", 0, {NULL} }
};

static struct reg_data_type_vector aarch64_vector_types[] = {
	{aarch64_vector_base_types + 0, 2},
	{aarch64_vector_base_types + 1, 2},
	{aarch64_vector_base_types + 2, 2},
	{aarch64_vector_base_types + 3, 4},
	{aarch64_vector_base_types + 4, 4},
	{aarch64_vector_base_types + 5, 4},
	{aarch64_vector_base_types + 6, 8},
	{aarch64_vector_base_types + 7, 8},
	{aarch64_vector_base_types + 8, 16},
	{aarch64_vector_base_types + 9, 16},
	{aarch64_vector_base_types + 10, 01},
	{aarch64_vector_base_types + 11, 01},
};

static struct reg_data_type aarch64_fpu_vector[] = {
	{REG_TYPE_ARCH_DEFINED, "v2d",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 0} },
	{REG_TYPE_ARCH_DEFINED, "v2u",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 1} },
	{REG_TYPE_ARCH_DEFINED, "v2i",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 2} },
	{REG_TYPE_ARCH_DEFINED, "v4f",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 3} },
	{REG_TYPE_ARCH_DEFINED, "v4u",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 4} },
	{REG_TYPE_ARCH_DEFINED, "v4i",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 5} },
	{REG_TYPE_ARCH_DEFINED, "v8u",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 6} },
	{REG_TYPE_ARCH_DEFINED, "v8i",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 7} },
	{REG_TYPE_ARCH_DEFINED, "v16u", REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 8} },
	{REG_TYPE_ARCH_DEFINED, "v16i", REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 9} },
	{REG_TYPE_ARCH_DEFINED, "v1u",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 10} },
	{REG_TYPE_ARCH_DEFINED, "v1i",  REG_TYPE_CLASS_VECTOR, {aarch64_vector_types + 11} },
};

static struct reg_data_type_union_field aarch64_union_fields_vnd[] = {
	{"f", aarch64_fpu_vector + 0, aarch64_union_fields_vnd + 1},
	{"u", aarch64_fpu_vector + 1, aarch64_union_fields_vnd + 2},
	{"s", aarch64_fpu_vector + 2, NULL},
};

static struct reg_data_type_union_field aarch64_union_fields_vns[] = {
	{"f", aarch64_fpu_vector + 3, aarch64_union_fields_vns + 1},
	{"u", aarch64_fpu_vector + 4, aarch64_union_fields_vns + 2},
	{"s", aarch64_fpu_vector + 5, NULL},
};

static struct reg_data_type_union_field aarch64_union_fields_vnh[] = {
	{"u", aarch64_fpu_vector + 6, aarch64_union_fields_vnh + 1},
	{"s", aarch64_fpu_vector + 7, NULL},
};

static struct reg_data_type_union_field aarch64_union_fields_vnb[] = {
	{"u", aarch64_fpu_vector + 8, aarch64_union_fields_vnb + 1},
	{"s", aarch64_fpu_vector + 9, NULL},
};

static struct reg_data_type_union_field aarch64_union_fields_vnq[] = {
	{"u", aarch64_fpu_vector + 10, aarch64_union_fields_vnq + 1},
	{"s", aarch64_fpu_vector + 11, NULL},
};

static struct reg_data_type_union aarch64_union_types[] = {
	{aarch64_union_fields_vnd},
	{aarch64_union_fields_vns},
	{aarch64_union_fields_vnh},
	{aarch64_union_fields_vnb},
	{aarch64_union_fields_vnq},
};

static struct reg_data_type aarch64_fpu_union[] = {
	{REG_TYPE_ARCH_DEFINED, "vnd", REG_TYPE_CLASS_UNION, {.reg_type_union = aarch64_union_types + 0} },
	{REG_TYPE_ARCH_DEFINED, "vns", REG_TYPE_CLASS_UNION, {.reg_type_union = aarch64_union_types + 1} },
	{REG_TYPE_ARCH_DEFINED, "vnh", REG_TYPE_CLASS_UNION, {.reg_type_union = aarch64_union_types + 2} },
	{REG_TYPE_ARCH_DEFINED, "vnb", REG_TYPE_CLASS_UNION, {.reg_type_union = aarch64_union_types + 3} },
	{REG_TYPE_ARCH_DEFINED, "vnq", REG_TYPE_CLASS_UNION, {.reg_type_union = aarch64_union_types + 4} },
};

static struct reg_data_type_union_field aarch64v_union_fields[] = {
	{"d", aarch64_fpu_union + 0, aarch64v_union_fields + 1},
	{"s", aarch64_fpu_union + 1, aarch64v_union_fields + 2},
	{"h", aarch64_fpu_union + 2, aarch64v_union_fields + 3},
	{"b", aarch64_fpu_union + 3, aarch64v_union_fields + 4},
	{"q", aarch64_fpu_union + 4, NULL},
};

static struct reg_data_type_union aarch64v_union[] = {
	{aarch64v_union_fields}
};

static struct reg_data_type aarch64v[] = {
	{REG_TYPE_ARCH_DEFINED, "aarch64v", REG_TYPE_CLASS_UNION,
		{.reg_type_union = aarch64v_union} },
};

static struct reg_data_type_bitfield aarch64_cpsr_bits[] = {
	{  0,  0, REG_TYPE_UINT8 },
	{  2,  3, REG_TYPE_UINT8 },
	{  4,  4, REG_TYPE_UINT8 },
	{  6,  6, REG_TYPE_BOOL },
	{  7,  7, REG_TYPE_BOOL },
	{  8,  8, REG_TYPE_BOOL },
	{  9,  9, REG_TYPE_BOOL },
	{ 20, 20, REG_TYPE_BOOL },
	{ 21, 21, REG_TYPE_BOOL },
	{ 28, 28, REG_TYPE_BOOL },
	{ 29, 29, REG_TYPE_BOOL },
	{ 30, 30, REG_TYPE_BOOL },
	{ 31, 31, REG_TYPE_BOOL },
};

static struct reg_data_type_flags_field aarch64_cpsr_fields[] = {
	{ "SP",  aarch64_cpsr_bits + 0,  aarch64_cpsr_fields + 1 },
	{ "EL",  aarch64_cpsr_bits + 1,  aarch64_cpsr_fields + 2 },
	{ "nRW", aarch64_cpsr_bits + 2,  aarch64_cpsr_fields + 3 },
	{ "F",   aarch64_cpsr_bits + 3,  aarch64_cpsr_fields + 4 },
	{ "I",   aarch64_cpsr_bits + 4,  aarch64_cpsr_fields + 5 },
	{ "A",   aarch64_cpsr_bits + 5,  aarch64_cpsr_fields + 6 },
	{ "D",   aarch64_cpsr_bits + 6,  aarch64_cpsr_fields + 7 },
	{ "IL",  aarch64_cpsr_bits + 7,  aarch64_cpsr_fields + 8 },
	{ "SS",  aarch64_cpsr_bits + 8,  aarch64_cpsr_fields + 9 },
	{ "V",   aarch64_cpsr_bits + 9,  aarch64_cpsr_fields + 10 },
	{ "C",   aarch64_cpsr_bits + 10, aarch64_cpsr_fields + 11 },
	{ "Z",   aarch64_cpsr_bits + 11, aarch64_cpsr_fields + 12 },
	{ "N",   aarch64_cpsr_bits + 12, NULL }
};

static struct reg_data_type_flags aarch64_cpsr_flags[] = {
	{ 4, aarch64_cpsr_fields }
};

static struct reg_data_type aarch64_flags_cpsr[] = {
	{REG_TYPE_ARCH_DEFINED, "cpsr_flags", REG_TYPE_CLASS_FLAGS,
		{.reg_type_flags = aarch64_cpsr_flags} },
};

static const struct {
	unsigned id;
	const char *name;
	unsigned bits;
	enum arm_mode mode;
	enum reg_type type;
	const char *group;
	const char *feature;
	struct reg_data_type *data_type;
} armv8_regs[] = {
	{ ARMV8_R0,  "x0",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R1,  "x1",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R2,  "x2",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R3,  "x3",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R4,  "x4",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R5,  "x5",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R6,  "x6",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R7,  "x7",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R8,  "x8",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R9,  "x9",  64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R10, "x10", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R11, "x11", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R12, "x12", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R13, "x13", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R14, "x14", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R15, "x15", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R16, "x16", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R17, "x17", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R18, "x18", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R19, "x19", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R20, "x20", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R21, "x21", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R22, "x22", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R23, "x23", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R24, "x24", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R25, "x25", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R26, "x26", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R27, "x27", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R28, "x28", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R29, "x29", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_R30, "x30", 64, ARM_MODE_ANY, REG_TYPE_UINT64, "general", "org.gnu.gdb.aarch64.core", NULL},

	{ ARMV8_SP, "sp", 64, ARM_MODE_ANY, REG_TYPE_DATA_PTR, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_PC, "pc", 64, ARM_MODE_ANY, REG_TYPE_CODE_PTR, "general", "org.gnu.gdb.aarch64.core", NULL},
	{ ARMV8_XPSR, "cpsr", 32, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED,
		"general", "org.gnu.gdb.aarch64.core", aarch64_flags_cpsr},
	{ ARMV8_V0,  "v0",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V1,  "v1",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V2,  "v2",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V3,  "v3",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V4,  "v4",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V5,  "v5",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V6,  "v6",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V7,  "v7",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V8,  "v8",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V9,  "v9",  128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V10, "v10", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V11, "v11", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V12, "v12", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V13, "v13", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V14, "v14", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V15, "v15", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V16, "v16", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V17, "v17", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V18, "v18", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V19, "v19", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V20, "v20", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V21, "v21", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V22, "v22", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V23, "v23", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V24, "v24", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V25, "v25", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V26, "v26", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V27, "v27", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V28, "v28", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V29, "v29", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V30, "v30", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_V31, "v31", 128, ARM_MODE_ANY, REG_TYPE_ARCH_DEFINED, "simdfp", "org.gnu.gdb.aarch64.fpu", aarch64v},
	{ ARMV8_FPSR, "fpsr", 32, ARM_MODE_ANY, REG_TYPE_UINT32, "simdfp", "org.gnu.gdb.aarch64.fpu", NULL},
	{ ARMV8_FPCR, "fpcr", 32, ARM_MODE_ANY, REG_TYPE_UINT32, "simdfp", "org.gnu.gdb.aarch64.fpu", NULL},

	{ ARMV8_ELR_EL1, "ELR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_CODE_PTR, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_SPSR_EL1, "SPSR_EL1", 32, ARMV8_64_EL1H, REG_TYPE_UINT32, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_ELR_EL2, "ELR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_CODE_PTR, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_SPSR_EL2, "SPSR_EL2", 32, ARMV8_64_EL2H, REG_TYPE_UINT32, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_ELR_EL3, "ELR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_CODE_PTR, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_SPSR_EL3, "SPSR_EL3", 32, ARMV8_64_EL3H, REG_TYPE_UINT32, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGAUTHSTATUS_EL1, "DBGAUTHSTATUS_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGCLAIMCLR_EL1, "DBGCLAIMCLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGCLAIMSET_EL1, "DBGCLAIMSET_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGDTRRX_EL0, "DBGDTRRX_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGDTRTX_EL0, "DBGDTRTX_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGDTR_EL0, "DBGDTR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGPRCR_EL1, "DBGPRCR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGVCR32_EL2, "DBGVCR32_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWVR0_EL1, "DBGWVR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWVR1_EL1, "DBGWVR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWVR2_EL1, "DBGWVR2_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWVR3_EL1, "DBGWVR3_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWCR0_EL1, "DBGWCR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWCR1_EL1, "DBGWCR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWCR2_EL1, "DBGWCR2_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_DBGWCR3_EL1, "DBGWCR3_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},

	{ ARMV8_OSDLR_EL1, "OSDLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_OSDTRRX_EL1, "OSDTRRX_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_OSDTRTX_EL1, "OSDTRTX_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_OSECCR_EL1, "OSECCR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_OSLSR_EL1, "OSLSR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_MDCCINT_EL1, "MDCCINT_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_MDCCSR_EL0, "MDCCSR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},
	{ ARMV8_MDSCR_EL1, "MDSCR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Debug", "net.sourceforge.openocd.debug", NULL},

	{ ARMV8_TPIDR_EL0, "TPIDR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_TPIDRRO_EL0, "TPIDRRO_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_ESR_EL1, "ESR_EL1", 32, ARMV8_64_EL1H, REG_TYPE_UINT32, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_FAR_EL1, "FAR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_VBAR_EL1, "VBAR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_SCTLR_EL1, "SCTLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_CPACR_EL1, "CPACR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_ACTLR_EL1, "ACTLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_AFSR0_EL1, "AFSR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_AFSR1_EL1, "AFSR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_CONTEXTIDR_EL1, "CONTEXTIDR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_DISR_EL1, "DISR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_ISR_EL1, "ISR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_TPIDR_EL1, "TPIDR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_ESR_EL2, "ESR_EL2", 32, ARMV8_64_EL2H, REG_TYPE_UINT32, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_FAR_EL2, "FAR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_VBAR_EL2, "VBAR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_SCTLR_EL2, "SCTLR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_ACTLR_EL2, "ACTLR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_AFSR0_EL2, "AFSR0_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_AFSR1_EL2, "AFSR1_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_CONTEXTIDR_EL2, "CONTEXTIDR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_CPTR_EL2, "CPTR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_HCR_EL2, "HCR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_HPFAR_EL2, "HPFAR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_IFSR32_EL2, "IFSR32_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_TPIDR_EL2, "TPIDR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_VDISR_EL2, "VDISR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_VSESR_EL2, "VSESR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_ESR_EL3, "ESR_EL3", 32, ARMV8_64_EL3H, REG_TYPE_UINT32, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_FAR_EL3, "FAR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_VBAR_EL3, "VBAR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_SCTLR_EL3, "SCTLR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_ACTLR_EL3, "ACTLR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_AFSR0_EL3, "AFSR0_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_AFSR1_EL3, "AFSR1_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_CPTR_EL3, "CPTR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_SCR_EL3, "SCR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_RMR_EL3, "RMR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_RVBAR_EL3, "RVBAR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_SDER32_EL3, "SDER32_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},
	{ ARMV8_TPIDR_EL3, "TPIDR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "SystemControlAndConfig", "net.sourceforge.openocd.sysconfig", NULL},

	{ ARMV8_AMAIR_EL1, "AMAIR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TTBR0_EL1, "TTBR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TTBR1_EL1, "TTBR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_MAIR_EL1, "MAIR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_PAR_EL1, "PAR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TCR_EL1, "TCR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_AMAIR_EL2, "AMAIR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_DACR32_EL2, "DACR32_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_HACR_EL2, "HACR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_HSTR_EL2, "HSTR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TTBR0_EL2, "TTBR0_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TTBR1_EL2, "TTBR1_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_MAIR_EL2, "MAIR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TCR_EL2, "TCR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_VTCR_EL2, "VTCR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_VTTBR_EL2, "VTTBR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_AMAIR_EL3, "AMAIR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TTBR0_EL3, "TTBR0_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_MAIR_EL3, "MAIR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},
	{ ARMV8_TCR_EL3, "TCR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "MemoryManagement", "net.sourceforge.openocd.memory", NULL},

	{ ARMV8_ICC_AP0R0_EL1, "ICC_AP0R0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_AP1R0_EL1, "ICC_AP1R0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_ASGI1R_EL1, "ICC_ASGI1R_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_BPR0_EL1, "ICC_BPR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_BPR1_EL1, "ICC_BPR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_CTLR_EL1, "ICC_CTLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_DIR_EL1, "ICC_DIR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_EOIR0_EL1, "ICC_EOIR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_EOIR1_EL1, "ICC_EOIR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_HPPIR0_EL1, "ICC_HPPIR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_HPPIR1_EL1, "ICC_HPPIR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_IAR0_EL1, "ICC_IAR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_IAR1_EL1, "ICC_IAR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_IGRPEN0_EL1, "ICC_IGRPEN0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_IGRPEN1_EL1, "ICC_IGRPEN1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_PMR_EL1, "ICC_PMR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_RPR_EL1, "ICC_RPR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_SGI0R_EL1, "ICC_SGI0R_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_SGI1R_EL1, "ICC_SGI1R_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_SRE_EL1, "ICC_SRE_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_AP0R0_EL1, "ICV_AP0R0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_AP1R0_EL1, "ICV_AP1R0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_BPR0_EL1, "ICV_BPR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_BPR1_EL1, "ICV_BPR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_CTLR_EL1, "ICV_CTLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_DIR_EL1, "ICV_DIR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_EOIR0_EL1, "ICV_EOIR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_EOIR1_EL1, "ICV_EOIR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_HPPIR0_EL1, "ICV_HPPIR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_HPPIR1_EL1, "ICV_HPPIR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_IAR0_EL1, "ICV_IAR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_IAR1_EL1, "ICV_IAR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_IGRPEN0_EL1, "ICV_IGRPEN0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_IGRPEN1_EL1, "ICV_IGRPEN1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_PMR_EL1, "ICV_PMR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICV_RPR_EL1, "ICV_RPR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},

	{ ARMV8_ICH_AP0R0_EL2, "ICH_AP0R0_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_AP1R0_EL2, "ICH_AP1R0_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_EISR_EL2, "ICH_EISR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_ELRSR_EL2, "ICH_ELRSR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_HCR_EL2, "ICH_HCR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_LR0_EL2, "ICH_LR0_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_LR1_EL2, "ICH_LR1_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_LR2_EL2, "ICH_LR2_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_LR3_EL2, "ICH_LR3_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_MISR_EL2, "ICH_MISR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_VMCR_EL2, "ICH_VMCR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICH_VTR_EL2, "ICH_VTR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_SRE_EL2, "ICC_SRE_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_CTLR_EL3, "ICC_CTLR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_IGRPEN1_EL3, "ICC_IGRPEN1_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},
	{ ARMV8_ICC_SRE_EL3, "ICC_SRE_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "gic", "net.sourceforge.openocd.gic", NULL},

	{ ARMV8_CNTFRQ_EL0, "CNTFRQ_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTPCT_EL0, "CNTPCT_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTVCT_EL0, "CNTVCT_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTP_TVAL_EL0, "CNTP_TVAL_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTP_CTL_EL0, "CNTP_CTL_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTP_CVAL_EL0, "CNTP_CVAL_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTV_TVAL_EL0, "CNTV_TVAL_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTV_CTL_EL0, "CNTV_CTL_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTV_CVAL_EL0, "CNTV_CVAL_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTKCTL_EL1, "CNTKCTL_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTPS_TVAL_EL1, "CNTPS_TVAL_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTPS_CTL_EL1, "CNTPS_CTL_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTPS_CVAL_EL1, "CNTPS_CVAL_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTVOFF_EL2, "CNTVOFF_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTHCTL_EL2, "CNTHCTL_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTHP_TVAL_EL2, "CNTHP_TVAL_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTHP_CTL_EL2, "CNTHP_CTL_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTHP_CVAL_EL2, "CNTHP_CVAL_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTHV_TVAL_EL2, "CNTHV_TVAL_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTHV_CTL_EL2, "CNTHV_CTL_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},
	{ ARMV8_CNTHV_CVAL_EL2, "CNTHV_CVAL_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "SystemTimer", "net.sourceforge.openocd.timer", NULL},

	{ ARMV8_CTR_EL0, "CTR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "CacheControlAndConfig", "net.sourceforge.openocd.cacheconfig", NULL},
	{ ARMV8_CCSIDR_EL1, "CCSIDR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "CacheControlAndConfig", "net.sourceforge.openocd.cacheconfig", NULL},
	{ ARMV8_CLIDR_EL1, "CLIDR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "CacheControlAndConfig", "net.sourceforge.openocd.cacheconfig", NULL},
	{ ARMV8_CSSELR_EL1, "CSSELR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "CacheControlAndConfig", "net.sourceforge.openocd.cacheconfig", NULL},
	{ ARMV8_CPUCFR_EL1, "CPUCFR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "CacheControlAndConfig", "net.sourceforge.openocd.cacheconfig", NULL},
	{ ARMV8_CPUPWRCTLR_EL1, "CPUPWRCTLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "CacheControlAndConfig", "net.sourceforge.openocd.cacheconfig", NULL},

	{ ARMV8_PMCCFILTR_EL0, "PMCCFILTR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMCCNTR_EL0, "PMCCNTR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMCEID0_EL0, "PMCEID0_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMCEID1_EL0, "PMCEID1_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMCNTENCLR_EL0, "PMCNTENCLR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMCNTENSET_EL0, "PMCNTENSET_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMCR_EL0, "PMCR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVCNTR0_EL0, "PMEVCNTR0_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVCNTR1_EL0, "PMEVCNTR1_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVCNTR2_EL0, "PMEVCNTR2_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVCNTR3_EL0, "PMEVCNTR3_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVCNTR4_EL0, "PMEVCNTR4_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVCNTR5_EL0, "PMEVCNTR5_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVTYPER0_EL0, "PMEVTYPER0_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVTYPER1_EL0, "PMEVTYPER1_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVTYPER2_EL0, "PMEVTYPER2_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVTYPER3_EL0, "PMEVTYPER3_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVTYPER4_EL0, "PMEVTYPER4_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMEVTYPER5_EL0, "PMEVTYPER5_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMINTENCLR_EL1, "PMINTENCLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMINTENSET_EL1, "PMINTENSET_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMOVSCLR_EL0, "PMOVSCLR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMOVSSET_EL0, "PMOVSSET_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMSELR_EL0, "PMSELR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMUSERENR_EL0, "PMUSERENR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMXEVCNTR_EL0, "PMXEVCNTR_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},
	{ ARMV8_PMXEVTYPER_EL0, "PMXEVTYPER_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "PerformanceMonitor", "net.sourceforge.openocd.performmon", NULL},

	{ ARMV8_ID_AA64AFR0_EL1, "ID_AA64AFR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64AFR1_EL1, "ID_AA64AFR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64DFR0_EL1, "ID_AA64DFR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64DFR1_EL1, "ID_AA64DFR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64ISAR0_EL1, "ID_AA64ISAR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64ISAR1_EL1, "ID_AA64ISAR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64MMFR0_EL1, "ID_AA64MMFR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64MMFR1_EL1, "ID_AA64MMFR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64MMFR2_EL1, "ID_AA64MMFR2_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64PFR0_EL1, "ID_AA64PFR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AA64PFR1_EL1, "ID_AA64PFR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_AFR0_EL1, "ID_AFR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_ISAR0_EL1, "ID_ISAR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_ISAR1_EL1, "ID_ISAR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_ISAR2_EL1, "ID_ISAR2_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_ISAR3_EL1, "ID_ISAR3_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_ISAR4_EL1, "ID_ISAR4_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_ISAR5_EL1, "ID_ISAR5_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_MMFR0_EL1, "ID_MMFR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_MMFR1_EL1, "ID_MMFR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_MMFR2_EL1, "ID_MMFR2_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_MMFR3_EL1, "ID_MMFR3_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_MMFR4_EL1, "ID_MMFR4_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_PFR0_EL1, "ID_PFR0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_ID_PFR1_EL1, "ID_PFR1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_DCZID_EL0, "DCZID_EL0", 64, ARMV8_64_EL0T, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_REVIDR_EL1, "REVIDR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_VMPIDR_EL2, "VMPIDR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},
	{ ARMV8_VPIDR_EL2, "VPIDR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "ID", "net.sourceforge.openocd.id", NULL},

	{ ARMV8_LORID_EL1, "LORID_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "LORegion", "net.sourceforge.openocd.lor", NULL},

	{ ARMV8_MDCR_EL2, "MDCR_EL2", 64, ARMV8_64_EL2H, REG_TYPE_UINT64, "Virtualization Extensions", "net.sourceforge.openocd.virtext", NULL},
	{ ARMV8_MDCR_EL3, "MDCR_EL3", 64, ARMV8_64_EL3H, REG_TYPE_UINT64, "Virtualization Extensions", "net.sourceforge.openocd.virtext", NULL},

	{ ARMV8_ERRIDR_EL1, "ERRID_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},
	{ ARMV8_ERRSELR_EL1, "ERRSELR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},
	{ ARMV8_ERXADDR_EL1, "ERXADDR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},
	{ ARMV8_ERXCTLR_EL1, "ERXCTLR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},
	{ ARMV8_ERXFR_EL1, "ERXFR_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},
	{ ARMV8_ERXMISC0_EL1, "ERXMISC0_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},
	{ ARMV8_ERXMISC1_EL1, "ERXMISC1_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},
	{ ARMV8_ERXSTATUS_EL1, "ERXSTATUS_EL1", 64, ARMV8_64_EL1H, REG_TYPE_UINT64, "Error System Registers", "net.sourceforge.openocd.esr", NULL},

};

static const struct {
	unsigned id;
	unsigned mapping;
	const char *name;
	unsigned bits;
	enum arm_mode mode;
	enum reg_type type;
	const char *group;
	const char *feature;
} armv8_regs32[] = {
	{ ARMV8_R0, 0,  "r0",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R1, 0,  "r1",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R2, 0,  "r2",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R3, 0,  "r3",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R4, 0,  "r4",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R5, 0,  "r5",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R6, 0,  "r6",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R7, 0,  "r7",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R8, 0,  "r8",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R9, 0,  "r9",  32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R10, 0, "r10", 32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R11, 0, "r11", 32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R12, 0, "r12", 32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R13, 0, "sp", 32, ARM_MODE_ANY, REG_TYPE_DATA_PTR, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_R14, 0, "lr",  32, ARM_MODE_ANY, REG_TYPE_CODE_PTR, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_PC, 0, "pc",   32, ARM_MODE_ANY, REG_TYPE_CODE_PTR, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_XPSR, 0, "cpsr", 32, ARM_MODE_ANY, REG_TYPE_UINT32, "general", "org.gnu.gdb.arm.core" },
	{ ARMV8_V0, 0, "d0",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V0, 8, "d1",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V1, 0, "d2",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V1, 8, "d3",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V2, 0, "d4",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V2, 8, "d5",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V3, 0, "d6",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V3, 8, "d7",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V4, 0, "d8",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V4, 8, "d9",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V5, 0, "d10", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V5, 8, "d11", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V6, 0, "d12", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V6, 8, "d13", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V7, 0, "d14", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V7, 8, "d15", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V8, 0, "d16", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V8, 8, "d17", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V9, 0, "d18", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V9, 8, "d19", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V10, 0, "d20", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V10, 8, "d21", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V11, 0, "d22", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V11, 8, "d23", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V12, 0, "d24", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V12, 8, "d25", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V13, 0, "d26", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V13, 8, "d27", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V14, 0, "d28", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V14, 8, "d29", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V15, 0, "d30", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_V15, 8, "d31", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARMV8_FPSR, 0, "fpscr", 32, ARM_MODE_ANY, REG_TYPE_UINT32, "float", "org.gnu.gdb.arm.vfp"},
};

#define ARMV8_NUM_REGS ARRAY_SIZE(armv8_regs)
#define ARMV8_NUM_REGS32 ARRAY_SIZE(armv8_regs32)

static int armv8_get_core_reg(struct reg *reg)
{
	struct arm_reg *armv8_reg = reg->arch_info;
	struct target *target = armv8_reg->target;
	struct arm *arm = target_to_arm(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	return arm->read_core_reg(target, reg, armv8_reg->num, arm->core_mode);
}

static int armv8_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct arm_reg *armv8_reg = reg->arch_info;
	struct target *target = armv8_reg->target;
	struct arm *arm = target_to_arm(target);
	uint64_t value = buf_get_u64(buf, 0, reg->size);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (reg->size <= 64) {
		if (reg == arm->cpsr)
			armv8_set_cpsr(arm, (uint32_t)value);
		else {
			buf_set_u64(reg->value, 0, reg->size, value);
			reg->valid = true;
		}
	} else if (reg->size <= 128) {
		uint64_t hvalue = buf_get_u64(buf + 8, 0, reg->size - 64);

		buf_set_u64(reg->value, 0, 64, value);
		buf_set_u64(reg->value + 8, 0, reg->size - 64, hvalue);
		reg->valid = true;
	}

	reg->dirty = true;

	return ERROR_OK;
}

static const struct reg_arch_type armv8_reg_type = {
	.get = armv8_get_core_reg,
	.set = armv8_set_core_reg,
};

static int armv8_get_core_reg32(struct reg *reg)
{
	struct arm_reg *armv8_reg = reg->arch_info;
	struct target *target = armv8_reg->target;
	struct arm *arm = target_to_arm(target);
	struct reg_cache *cache = arm->core_cache;
	struct reg *reg64;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* get the corresponding Aarch64 register */
	reg64 = cache->reg_list + armv8_reg->num;
	if (reg64->valid) {
		reg->valid = true;
		return ERROR_OK;
	}

	retval = arm->read_core_reg(target, reg64, armv8_reg->num, arm->core_mode);
	if (retval == ERROR_OK)
		reg->valid = reg64->valid;

	return retval;
}

static int armv8_set_core_reg32(struct reg *reg, uint8_t *buf)
{
	struct arm_reg *armv8_reg = reg->arch_info;
	struct target *target = armv8_reg->target;
	struct arm *arm = target_to_arm(target);
	struct reg_cache *cache = arm->core_cache;
	struct reg *reg64 = cache->reg_list + armv8_reg->num;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (reg64 == arm->cpsr) {
		armv8_set_cpsr(arm, value);
	} else {
		if (reg->size <= 32)
			buf_set_u32(reg->value, 0, 32, value);
		else if (reg->size <= 64) {
			uint64_t value64 = buf_get_u64(buf, 0, 64);
			buf_set_u64(reg->value, 0, 64, value64);
		}
		reg->valid = true;
		reg64->valid = true;
	}

	reg64->dirty = true;

	return ERROR_OK;
}

static const struct reg_arch_type armv8_reg32_type = {
	.get = armv8_get_core_reg32,
	.set = armv8_set_core_reg32,
};

/** Builds cache of architecturally defined registers.  */
struct reg_cache *armv8_build_reg_cache(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	int num_regs = ARMV8_NUM_REGS;
	int num_regs32 = ARMV8_NUM_REGS32;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg_cache *cache32 = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct reg *reg_list32 = calloc(num_regs32, sizeof(struct reg));
	struct arm_reg *arch_info = calloc(num_regs, sizeof(struct arm_reg));
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "Aarch64 registers";
	cache->next = cache32;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = armv8_regs[i].id;
		arch_info[i].mode = armv8_regs[i].mode;
		arch_info[i].target = target;
		arch_info[i].arm = arm;

		reg_list[i].name = armv8_regs[i].name;
		reg_list[i].size = armv8_regs[i].bits;
		reg_list[i].value = &arch_info[i].value[0];
		reg_list[i].type = &armv8_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].group = armv8_regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = armv8_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type) {
			if (!armv8_regs[i].data_type)
				reg_list[i].reg_data_type->type = armv8_regs[i].type;
			else
				*reg_list[i].reg_data_type = *armv8_regs[i].data_type;
		} else
			LOG_ERROR("unable to allocate reg type list");
	}

	arm->cpsr = reg_list + ARMV8_XPSR;
	arm->pc = reg_list + ARMV8_PC;
	arm->core_cache = cache;

	/* shadow cache for ARM mode registers */
	cache32->name = "Aarch32 registers";
	cache32->next = NULL;
	cache32->reg_list = reg_list32;
	cache32->num_regs = num_regs32;

	for (i = 0; i < num_regs32; i++) {
		reg_list32[i].name = armv8_regs32[i].name;
		reg_list32[i].size = armv8_regs32[i].bits;
		reg_list32[i].value = &arch_info[armv8_regs32[i].id].value[armv8_regs32[i].mapping];
		reg_list32[i].type = &armv8_reg32_type;
		reg_list32[i].arch_info = &arch_info[armv8_regs32[i].id];
		reg_list32[i].group = armv8_regs32[i].group;
		reg_list32[i].number = i;
		reg_list32[i].exist = true;
		reg_list32[i].caller_save = true;

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = armv8_regs32[i].feature;
			reg_list32[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");

		reg_list32[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list32[i].reg_data_type)
			reg_list32[i].reg_data_type->type = armv8_regs32[i].type;
		else
			LOG_ERROR("unable to allocate reg type list");
	}

	(*cache_p) = cache;
	return cache;
}

struct reg *armv8_reg_current(struct arm *arm, unsigned regnum)
{
	struct reg *r;

	if (regnum > (ARMV8_LAST_REG - 1))
		return NULL;

	r = arm->core_cache->reg_list + regnum;
	return r;
}

static void armv8_free_cache(struct reg_cache *cache, bool regs32)
{
	struct reg *reg;
	unsigned int i;

	if (!cache)
		return;

	for (i = 0; i < cache->num_regs; i++) {
		reg = &cache->reg_list[i];

		free(reg->feature);
		free(reg->reg_data_type);
	}

	if (!regs32)
		free(cache->reg_list[0].arch_info);
	free(cache->reg_list);
	free(cache);
}

void armv8_free_reg_cache(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	struct reg_cache *cache = NULL, *cache32 = NULL;

	cache = arm->core_cache;
	if (cache)
		cache32 = cache->next;
	armv8_free_cache(cache32, true);
	armv8_free_cache(cache, false);
	arm->core_cache = NULL;
}

const struct command_registration armv8_command_handlers[] = {
	{
		.name = "catch_exc",
		.handler = armv8_handle_exception_catch_command,
		.mode = COMMAND_EXEC,
		.help = "configure exception catch",
		.usage = "[(nsec_el1,nsec_el2,sec_el1,sec_el3)+,off]",
	},
	COMMAND_REGISTRATION_DONE
};

const char *armv8_get_gdb_arch(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	return arm->core_state == ARM_STATE_AARCH64 ? "aarch64" : "arm";
}

int armv8_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	struct arm *arm = target_to_arm(target);
	int i;

	if (arm->core_state == ARM_STATE_AARCH64) {

		LOG_DEBUG("Creating Aarch64 register list for target %s", target_name(target));

		switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = ARMV8_V0;
			*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

			for (i = 0; i < *reg_list_size; i++)
					(*reg_list)[i] = armv8_reg_current(arm, i);
			return ERROR_OK;

		case REG_CLASS_ALL:
			*reg_list_size = ARMV8_LAST_REG;
			*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

			for (i = 0; i < *reg_list_size; i++)
					(*reg_list)[i] = armv8_reg_current(arm, i);

			return ERROR_OK;

		default:
			LOG_ERROR("not a valid register class type in query.");
			return ERROR_FAIL;
		}
	} else {
		struct reg_cache *cache32 = arm->core_cache->next;

		LOG_DEBUG("Creating Aarch32 register list for target %s", target_name(target));

		switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = ARMV8_R14 + 3;
			*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

			for (i = 0; i < *reg_list_size; i++)
				(*reg_list)[i] = cache32->reg_list + i;

			return ERROR_OK;
		case REG_CLASS_ALL:
			*reg_list_size = cache32->num_regs;
			*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

			for (i = 0; i < *reg_list_size; i++)
				(*reg_list)[i] = cache32->reg_list + i;

			return ERROR_OK;
		default:
			LOG_ERROR("not a valid register class type in query.");
			return ERROR_FAIL;
		}
	}
}

int armv8_set_dbgreg_bits(struct armv8_common *armv8, unsigned int reg, unsigned long mask, unsigned long value)
{
	uint32_t tmp;

	/* Read register */
	int retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + reg, &tmp);
	if (retval != ERROR_OK)
		return retval;

	/* clear bitfield */
	tmp &= ~mask;
	/* put new value */
	tmp |= value & mask;

	/* write new value */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + reg, tmp);
	return retval;
}

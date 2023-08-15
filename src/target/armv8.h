/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2015 by David Ung                                       *
 *                                                                         *
 *   Portions Copyright (C) 2023 Analog Devices, Inc.                      *
 *                                                                         *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARMV8_H
#define OPENOCD_TARGET_ARMV8_H

#include "arm_adi_v5.h"
#include "arm.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"
#include "armv8_dpm.h"
#include "arm_cti.h"

enum {
	ARMV8_R0 = 0,
	ARMV8_R1,
	ARMV8_R2,
	ARMV8_R3,
	ARMV8_R4,
	ARMV8_R5,
	ARMV8_R6,
	ARMV8_R7,
	ARMV8_R8,
	ARMV8_R9,
	ARMV8_R10,
	ARMV8_R11,
	ARMV8_R12,
	ARMV8_R13,
	ARMV8_R14,
	ARMV8_R15,
	ARMV8_R16,
	ARMV8_R17,
	ARMV8_R18,
	ARMV8_R19,
	ARMV8_R20,
	ARMV8_R21,
	ARMV8_R22,
	ARMV8_R23,
	ARMV8_R24,
	ARMV8_R25,
	ARMV8_R26,
	ARMV8_R27,
	ARMV8_R28,
	ARMV8_R29,
	ARMV8_R30,

	ARMV8_SP = 31,
	ARMV8_PC = 32,
	ARMV8_XPSR = 33,

	ARMV8_V0 = 34,
	ARMV8_V1,
	ARMV8_V2,
	ARMV8_V3,
	ARMV8_V4,
	ARMV8_V5,
	ARMV8_V6,
	ARMV8_V7,
	ARMV8_V8,
	ARMV8_V9,
	ARMV8_V10,
	ARMV8_V11,
	ARMV8_V12,
	ARMV8_V13,
	ARMV8_V14,
	ARMV8_V15,
	ARMV8_V16,
	ARMV8_V17,
	ARMV8_V18,
	ARMV8_V19,
	ARMV8_V20,
	ARMV8_V21,
	ARMV8_V22,
	ARMV8_V23,
	ARMV8_V24,
	ARMV8_V25,
	ARMV8_V26,
	ARMV8_V27,
	ARMV8_V28,
	ARMV8_V29,
	ARMV8_V30,
	ARMV8_V31,
	ARMV8_FPSR,
	ARMV8_FPCR,

	ARMV8_ELR_EL1 = 68,
	ARMV8_SPSR_EL1,
	ARMV8_ELR_EL2,
	ARMV8_SPSR_EL2,
	ARMV8_ELR_EL3,
	ARMV8_SPSR_EL3,
	ARMV8_DBGAUTHSTATUS_EL1,
	ARMV8_DBGCLAIMCLR_EL1,
	ARMV8_DBGCLAIMSET_EL1,
	ARMV8_DBGDTRRX_EL0,
	ARMV8_DBGDTRTX_EL0,
	ARMV8_DBGDTR_EL0,
	ARMV8_DBGPRCR_EL1,
	ARMV8_DBGVCR32_EL2,
	ARMV8_DBGWVR0_EL1,
	ARMV8_DBGWVR1_EL1,
	ARMV8_DBGWVR2_EL1,
	ARMV8_DBGWVR3_EL1,
	ARMV8_DBGWCR0_EL1,
	ARMV8_DBGWCR1_EL1,
	ARMV8_DBGWCR2_EL1,
	ARMV8_DBGWCR3_EL1,
	ARMV8_OSDLR_EL1,
	ARMV8_OSDTRRX_EL1,
	ARMV8_OSDTRTX_EL1,
	ARMV8_OSECCR_EL1,
	ARMV8_OSLSR_EL1,
	ARMV8_MDCCINT_EL1,
	ARMV8_MDCCSR_EL0,
	ARMV8_MDSCR_EL1,
	
	ARMV8_TPIDR_EL0,
	ARMV8_TPIDRRO_EL0,
	ARMV8_ESR_EL1,
	ARMV8_FAR_EL1,
	ARMV8_VBAR_EL1,
	ARMV8_SCTLR_EL1,
	ARMV8_CPACR_EL1,
	ARMV8_ACTLR_EL1,
	ARMV8_AFSR0_EL1,
	ARMV8_AFSR1_EL1,
	ARMV8_CONTEXTIDR_EL1,
	ARMV8_DISR_EL1,
	ARMV8_ISR_EL1,
	ARMV8_TPIDR_EL1,

	ARMV8_ESR_EL2,
	ARMV8_FAR_EL2,
	ARMV8_VBAR_EL2,
	ARMV8_SCTLR_EL2,
	ARMV8_ACTLR_EL2,
	ARMV8_AFSR0_EL2,
	ARMV8_AFSR1_EL2,
	ARMV8_CONTEXTIDR_EL2,
	ARMV8_CPTR_EL2,
	ARMV8_HCR_EL2,
	ARMV8_HPFAR_EL2,
	ARMV8_IFSR32_EL2,
	ARMV8_TPIDR_EL2,
	ARMV8_VDISR_EL2,
	ARMV8_VSESR_EL2,

	ARMV8_ESR_EL3,
	ARMV8_FAR_EL3,
	ARMV8_VBAR_EL3,
	ARMV8_SCTLR_EL3,
	ARMV8_ACTLR_EL3,
	ARMV8_AFSR0_EL3,
	ARMV8_AFSR1_EL3,
	ARMV8_CPTR_EL3,
	ARMV8_SCR_EL3,
	ARMV8_RMR_EL3,
	ARMV8_RVBAR_EL3,
	ARMV8_SDER32_EL3,
	ARMV8_TPIDR_EL3,

	ARMV8_AMAIR_EL1,
	ARMV8_TTBR0_EL1,
	ARMV8_TTBR1_EL1,
	ARMV8_MAIR_EL1,
	ARMV8_PAR_EL1,
	ARMV8_TCR_EL1,
	ARMV8_AMAIR_EL2,
	ARMV8_DACR32_EL2,
	ARMV8_HACR_EL2,
	ARMV8_HSTR_EL2,
	ARMV8_TTBR0_EL2,
	ARMV8_TTBR1_EL2,
	ARMV8_MAIR_EL2,
	ARMV8_TCR_EL2,
	ARMV8_VTCR_EL2,
	ARMV8_VTTBR_EL2,
	ARMV8_AMAIR_EL3,
	ARMV8_TTBR0_EL3,
	ARMV8_MAIR_EL3,
	ARMV8_TCR_EL3,

	ARMV8_ICC_AP0R0_EL1,
	ARMV8_ICC_AP1R0_EL1,
	ARMV8_ICC_ASGI1R_EL1,
	ARMV8_ICC_BPR0_EL1,
	ARMV8_ICC_BPR1_EL1,
	ARMV8_ICC_CTLR_EL1,
	ARMV8_ICC_DIR_EL1,
	ARMV8_ICC_EOIR0_EL1,
	ARMV8_ICC_EOIR1_EL1,
	ARMV8_ICC_HPPIR0_EL1,
	ARMV8_ICC_HPPIR1_EL1,
	ARMV8_ICC_IAR0_EL1,
	ARMV8_ICC_IAR1_EL1,
	ARMV8_ICC_IGRPEN0_EL1,
	ARMV8_ICC_IGRPEN1_EL1,
	ARMV8_ICC_PMR_EL1,
	ARMV8_ICC_RPR_EL1,
	ARMV8_ICC_SGI0R_EL1,
	ARMV8_ICC_SGI1R_EL1,
	ARMV8_ICC_SRE_EL1,
	ARMV8_ICV_AP0R0_EL1,
	ARMV8_ICV_AP1R0_EL1,
	ARMV8_ICV_BPR0_EL1,
	ARMV8_ICV_BPR1_EL1,
	ARMV8_ICV_CTLR_EL1,
	ARMV8_ICV_DIR_EL1,
	ARMV8_ICV_EOIR0_EL1,
	ARMV8_ICV_EOIR1_EL1,
	ARMV8_ICV_HPPIR0_EL1,
	ARMV8_ICV_HPPIR1_EL1,
	ARMV8_ICV_IAR0_EL1,
	ARMV8_ICV_IAR1_EL1,
	ARMV8_ICV_IGRPEN0_EL1,
	ARMV8_ICV_IGRPEN1_EL1,
	ARMV8_ICV_PMR_EL1,
	ARMV8_ICV_RPR_EL1,

	ARMV8_ICH_AP0R0_EL2,
	ARMV8_ICH_AP1R0_EL2,
	ARMV8_ICH_EISR_EL2,
	ARMV8_ICH_ELRSR_EL2,
	ARMV8_ICH_HCR_EL2,
	ARMV8_ICH_LR0_EL2,
	ARMV8_ICH_LR1_EL2,
	ARMV8_ICH_LR2_EL2,
	ARMV8_ICH_LR3_EL2,
	ARMV8_ICH_MISR_EL2,
	ARMV8_ICH_VMCR_EL2,
	ARMV8_ICH_VTR_EL2,
	ARMV8_ICC_SRE_EL2,

	ARMV8_ICC_CTLR_EL3,
	ARMV8_ICC_IGRPEN1_EL3,
	ARMV8_ICC_SRE_EL3,

	ARMV8_CNTFRQ_EL0,
	ARMV8_CNTPCT_EL0,
	ARMV8_CNTVCT_EL0,
	ARMV8_CNTP_TVAL_EL0,
	ARMV8_CNTP_CTL_EL0,
	ARMV8_CNTP_CVAL_EL0,
	ARMV8_CNTV_TVAL_EL0,
	ARMV8_CNTV_CTL_EL0,
	ARMV8_CNTV_CVAL_EL0,

	ARMV8_CNTKCTL_EL1,
	ARMV8_CNTPS_TVAL_EL1,
	ARMV8_CNTPS_CTL_EL1,
	ARMV8_CNTPS_CVAL_EL1,

	ARMV8_CNTVOFF_EL2,
	ARMV8_CNTHCTL_EL2,
	ARMV8_CNTHP_TVAL_EL2,
	ARMV8_CNTHP_CTL_EL2,
	ARMV8_CNTHP_CVAL_EL2,
	ARMV8_CNTHV_TVAL_EL2,
	ARMV8_CNTHV_CTL_EL2,
	ARMV8_CNTHV_CVAL_EL2,

	ARMV8_CTR_EL0,
	ARMV8_CCSIDR_EL1,
	ARMV8_CLIDR_EL1,
	ARMV8_CSSELR_EL1,
	ARMV8_CPUCFR_EL1,
	ARMV8_CPUPWRCTLR_EL1,

	ARMV8_PMCCFILTR_EL0,
	ARMV8_PMCCNTR_EL0,
	ARMV8_PMCEID0_EL0,
	ARMV8_PMCEID1_EL0,
	ARMV8_PMCNTENCLR_EL0,
	ARMV8_PMCNTENSET_EL0,
	ARMV8_PMCR_EL0,
	ARMV8_PMEVCNTR0_EL0,
	ARMV8_PMEVCNTR1_EL0,
	ARMV8_PMEVCNTR2_EL0,
	ARMV8_PMEVCNTR3_EL0,
	ARMV8_PMEVCNTR4_EL0,
	ARMV8_PMEVCNTR5_EL0,
	ARMV8_PMEVTYPER0_EL0,
	ARMV8_PMEVTYPER1_EL0,
	ARMV8_PMEVTYPER2_EL0,
	ARMV8_PMEVTYPER3_EL0,
	ARMV8_PMEVTYPER4_EL0,
	ARMV8_PMEVTYPER5_EL0,
	ARMV8_PMINTENCLR_EL1,
	ARMV8_PMINTENSET_EL1,
	ARMV8_PMOVSCLR_EL0,
	ARMV8_PMOVSSET_EL0,
	ARMV8_PMSELR_EL0,
	ARMV8_PMUSERENR_EL0,
	ARMV8_PMXEVCNTR_EL0,
	ARMV8_PMXEVTYPER_EL0,

	ARMV8_ID_AA64AFR0_EL1,
	ARMV8_ID_AA64AFR1_EL1,
	ARMV8_ID_AA64DFR0_EL1,
	ARMV8_ID_AA64DFR1_EL1,
	ARMV8_ID_AA64ISAR0_EL1,
	ARMV8_ID_AA64ISAR1_EL1,
	ARMV8_ID_AA64MMFR0_EL1,
	ARMV8_ID_AA64MMFR1_EL1,
	ARMV8_ID_AA64MMFR2_EL1,
	ARMV8_ID_AA64PFR0_EL1,
	ARMV8_ID_AA64PFR1_EL1,
	ARMV8_ID_AFR0_EL1,
	ARMV8_ID_ISAR0_EL1,
	ARMV8_ID_ISAR1_EL1,
	ARMV8_ID_ISAR2_EL1,
	ARMV8_ID_ISAR3_EL1,
	ARMV8_ID_ISAR4_EL1,
	ARMV8_ID_ISAR5_EL1,
	ARMV8_ID_MMFR0_EL1,
	ARMV8_ID_MMFR1_EL1,
	ARMV8_ID_MMFR2_EL1,
	ARMV8_ID_MMFR3_EL1,
	ARMV8_ID_MMFR4_EL1,
	ARMV8_ID_PFR0_EL1,
	ARMV8_ID_PFR1_EL1,
	ARMV8_DCZID_EL0,
	ARMV8_REVIDR_EL1,
	ARMV8_VMPIDR_EL2,
	ARMV8_VPIDR_EL2,

	ARMV8_LORID_EL1,

	ARMV8_MDCR_EL2,
	ARMV8_MDCR_EL3,

	ARMV8_ERRIDR_EL1,
	ARMV8_ERRSELR_EL1,
	ARMV8_ERXADDR_EL1,
	ARMV8_ERXCTLR_EL1,
	ARMV8_ERXFR_EL1,
	ARMV8_ERXMISC0_EL1,
	ARMV8_ERXMISC1_EL1,
	ARMV8_ERXSTATUS_EL1,
	
	ARMV8_LAST_REG,
};

enum run_control_op {
	ARMV8_RUNCONTROL_UNKNOWN = 0,
	ARMV8_RUNCONTROL_RESUME = 1,
	ARMV8_RUNCONTROL_HALT = 2,
	ARMV8_RUNCONTROL_STEP = 3,
};

#define ARMV8_COMMON_MAGIC 0x0A450AAAU

/* VA to PA translation operations opc2 values*/
#define V2PCWPR  0
#define V2PCWPW  1
#define V2PCWUR  2
#define V2PCWUW  3
#define V2POWPR  4
#define V2POWPW  5
#define V2POWUR  6
#define V2POWUW  7
/*   L210/L220 cache controller support */
struct armv8_l2x_cache {
	uint32_t base;
	uint32_t way;
};

struct armv8_cachesize {
	uint32_t level_num;
	/*  cache dimensioning */
	uint32_t linelen;
	uint32_t associativity;
	uint32_t nsets;
	uint32_t cachesize;
	/* info for set way operation on cache */
	uint32_t index;
	uint32_t index_shift;
	uint32_t way;
	uint32_t way_shift;
};

/* information about one architecture cache at any level */
struct armv8_arch_cache {
	int ctype;				/* cache type, CLIDR encoding */
	struct armv8_cachesize d_u_size;	/* data cache */
	struct armv8_cachesize i_size;		/* instruction cache */
};

struct armv8_cache_common {
	int info;
	int loc;
	uint32_t iminline;
	uint32_t dminline;
	struct armv8_arch_cache arch[6];	/* cache info, L1 - L7 */
	int i_cache_enabled;
	int d_u_cache_enabled;

	/* l2 external unified cache if some */
	void *l2_cache;
	int (*flush_all_data_cache)(struct target *target);
	int (*display_cache_info)(struct command_invocation *cmd,
			struct armv8_cache_common *armv8_cache);
};

struct armv8_mmu_common {
	/* following field mmu working way */
	int32_t ttbr1_used; /*  -1 not initialized, 0 no ttbr1 1 ttbr1 used and  */
	uint64_t ttbr0_mask;/*  masked to be used  */

	uint32_t ttbcr;     /* cache for ttbcr register */
	uint32_t ttbr_mask[2];
	uint32_t ttbr_range[2];

	int (*read_physical_memory)(struct target *target, target_addr_t address,
			uint32_t size, uint32_t count, uint8_t *buffer);
	struct armv8_cache_common armv8_cache;
	uint32_t mmu_enabled;
};

struct armv8_common {
	unsigned int common_magic;

	struct arm arm;
	struct reg_cache *core_cache;

	/* Core Debug Unit */
	struct arm_dpm dpm;
	target_addr_t debug_base;
	struct adiv5_ap *debug_ap;

	const uint32_t *opcodes;

	/* mdir */
	uint8_t multi_processor_system;
	uint8_t cluster_id;
	uint8_t cpu_id;

	/* armv8 aarch64 need below information for page translation */
	uint8_t va_size;
	uint8_t pa_size;
	uint32_t page_size;
	uint64_t ttbr_base;

	struct armv8_mmu_common armv8_mmu;

	struct arm_cti *cti;
	struct arm_cti *sys_cti;

	/* last run-control command issued to this target (resume, halt, step) */
	enum run_control_op last_run_control_op;

	/* Direct processor core register read and writes */
	int (*read_reg_u64)(struct armv8_common *armv8, int num, uint64_t *value);
	int (*write_reg_u64)(struct armv8_common *armv8, int num, uint64_t value);

	/* SIMD/FPU registers read/write interface */
	int (*read_reg_u128)(struct armv8_common *armv8, int num,
			uint64_t *lvalue, uint64_t *hvalue);
	int (*write_reg_u128)(struct armv8_common *armv8, int num,
			uint64_t lvalue, uint64_t hvalue);

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv8_common *
target_to_armv8(struct target *target)
{
	return container_of(target->arch_info, struct armv8_common, arm);
}

static inline bool is_armv8(struct armv8_common *armv8)
{
	return armv8->common_magic == ARMV8_COMMON_MAGIC;
}

/* register offsets from armv8.debug_base */
#define CPUV8_DBG_MAINID0		0xD00
#define CPUV8_DBG_CPUFEATURE0	0xD20
#define CPUV8_DBG_DBGFEATURE0	0xD28
#define CPUV8_DBG_MEMFEATURE0	0xD38

#define CPUV8_DBG_LOCKACCESS 0xFB0
#define CPUV8_DBG_LOCKSTATUS 0xFB4

#define CPUV8_DBG_EDESR		0x20
#define CPUV8_DBG_EDECR		0x24
#define CPUV8_DBG_EDWAR0	0x30
#define CPUV8_DBG_EDWAR1	0x34
#define CPUV8_DBG_DSCR		0x088
#define CPUV8_DBG_DRCR		0x090
#define CPUV8_DBG_ECCR		0x098
#define CPUV8_DBG_PRCR		0x310
#define CPUV8_DBG_PRSR		0x314

#define CPUV8_DBG_DTRRX		0x080
#define CPUV8_DBG_ITR		0x084
#define CPUV8_DBG_SCR		0x088
#define CPUV8_DBG_DTRTX		0x08c

#define CPUV8_DBG_BVR_BASE	0x400
#define CPUV8_DBG_BCR_BASE	0x408
#define CPUV8_DBG_WVR_BASE	0x800
#define CPUV8_DBG_WCR_BASE	0x808
#define CPUV8_DBG_VCR		0x01C

#define CPUV8_DBG_OSLAR		0x300
#define CPUV8_DBG_OSLSR		0x304

#define CPUV8_DBG_AUTHSTATUS	0xFB8

#define PAGE_SIZE_4KB				0x1000
#define PAGE_SIZE_4KB_LEVEL0_BITS	39
#define PAGE_SIZE_4KB_LEVEL1_BITS	30
#define PAGE_SIZE_4KB_LEVEL2_BITS	21
#define PAGE_SIZE_4KB_LEVEL3_BITS	12

#define PAGE_SIZE_4KB_LEVEL0_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL0_BITS)
#define PAGE_SIZE_4KB_LEVEL1_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL1_BITS)
#define PAGE_SIZE_4KB_LEVEL2_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL2_BITS)
#define PAGE_SIZE_4KB_LEVEL3_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL3_BITS)

#define PAGE_SIZE_4KB_TRBBASE_MASK	0xFFFFFFFFF000

int armv8_arch_state(struct target *target);
int armv8_read_mpidr(struct armv8_common *armv8);
int armv8_identify_cache(struct armv8_common *armv8);
int armv8_init_arch_info(struct target *target, struct armv8_common *armv8);
int armv8_mmu_translate_va_pa(struct target *target, target_addr_t va,
		target_addr_t *val, int meminfo);
int armv8_mmu_translate_va(struct target *target,  target_addr_t va, target_addr_t *val);

int armv8_handle_cache_info_command(struct command_invocation *cmd,
		struct armv8_cache_common *armv8_cache);

void armv8_set_cpsr(struct arm *arm, uint32_t cpsr);

static inline unsigned int armv8_curel_from_core_mode(enum arm_mode core_mode)
{
	switch (core_mode) {
	/* Aarch32 modes */
	case ARM_MODE_USR:
		return 0;
	case ARM_MODE_SVC:
	case ARM_MODE_ABT: /* FIXME: EL3? */
	case ARM_MODE_IRQ: /* FIXME: EL3? */
	case ARM_MODE_FIQ: /* FIXME: EL3? */
	case ARM_MODE_UND: /* FIXME: EL3? */
	case ARM_MODE_SYS: /* FIXME: EL3? */
		return 1;
	/* case ARM_MODE_HYP:
	 *     return 2;
	 */
	case ARM_MODE_MON:
		return 3;
	/* all Aarch64 modes */
	default:
		return (core_mode >> 2) & 3;
	}
}

const char *armv8_mode_name(unsigned psr_mode);
void armv8_select_reg_access(struct armv8_common *armv8, bool is_aarch64);
int armv8_set_dbgreg_bits(struct armv8_common *armv8, unsigned int reg, unsigned long mask, unsigned long value);

extern void armv8_free_reg_cache(struct target *target);

extern const struct command_registration armv8_command_handlers[];

#endif /* OPENOCD_TARGET_ARMV8_H */

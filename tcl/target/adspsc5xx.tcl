# SPDX-License-Identifier: GPL-2.0-or-later
# Common routines for Analog Devices ADSP-SC5xx
#
# Copyright (C) 2015-2025 Analog Devices, Inc.

# Unlock the processor
proc adjc_unlock {adjc} {
	set userkey0 "_USERKEY0_Proc$::PROCNUM"
	set userkey1 "_USERKEY1_Proc$::PROCNUM"
	set userkey2 "_USERKEY2_Proc$::PROCNUM"
	set userkey3 "_USERKEY3_Proc$::PROCNUM"

	irscan $adjc 0xa -endstate RUN/IDLE
	drscan $adjc 32 [set ::$userkey0] 32 [set ::$userkey1] 32 [set ::$userkey2] 32 [set ::$userkey3] -endstate RUN/IDLE
}

# ADJC is the TAP name for the ADI JTAG Controller
proc adjc_enable_dap {adjc} {
	# Unlock the processor before enabling DAP
	adjc_unlock $adjc

	irscan $adjc 0x5 -endstate IRPAUSE
	drscan $adjc 8 0x4 -endstate RUN/IDLE
	runtest 2
}

# Show RCU_MSG bitfields
proc show_rcu_msg { name val } {
	show_mmr_bitfield  7  0 $val BOOTERROR   { _NUMBER_ }
	show_mmr_bitfield  8  8 $val C0IDLE      { "" "Core 0 is IDLE" }
	show_mmr_bitfield  9  9 $val C1IDLE      { "" "Core 1 is IDLE" }
	show_mmr_bitfield 10 10 $val C2IDLE      { "" "Core 2 is IDLE" }
	show_mmr_bitfield 12 12 $val C0TASK      { "" "Core 0 has finished a task" }
	show_mmr_bitfield 13 13 $val C1TASK      { "" "Core 1 has finished a task" }
	show_mmr_bitfield 14 14 $val C2TASK      { "" "Core 2 has finished a task" }
	show_mmr_bitfield 16 16 $val C0L1INIT    { "" "Core 0 L1 initialized" }
	show_mmr_bitfield 17 17 $val C1L1INIT    { "" "Core 1 L1 initialized" }
	show_mmr_bitfield 18 18 $val C2L1INIT    { "" "Core 2 L1 initialized" }
	show_mmr_bitfield 22 22 $val L2INIT      { "" "L2 initialized" }
	show_mmr_bitfield 24 24 $val HALTONAPP   { "" "Halt on applicaton call" }
	show_mmr_bitfield 25 25 $val HALTONINIT  { "" "Halt on initcode call" }
	show_mmr_bitfield 26 26 $val HALTONCALL  { "" "Halt on callback call" }
	show_mmr_bitfield 27 27 $val HALTONERR   { "" "Halt on error call" }
	show_mmr_bitfield 28 28 $val CALLAPP     { "" "Call application flag" }
	show_mmr_bitfield 29 29 $val CALLINIT    { "" "Call initcode flag" }
	show_mmr_bitfield 30 30 $val CALLBACK    { "" "Call callback flag" }
	show_mmr_bitfield 31 31 $val CALLERR     { "" "Call error flag" }
}

# Show RCU_STAT bitfields
proc show_rcu_stat { name val } {
	show_mmr_bitfield  0  0 $val HWRST       { "" HWRST }
	show_mmr_bitfield  2  2 $val SSRST       { "" SSRST }
	show_mmr_bitfield  3  3 $val SWRST       { "" SWRST }
	show_mmr_bitfield  5  5 $val RSTOUT      { "" RSTOUT }
	show_mmr_bitfield 11  8 $val BMODE       { _NUMBER_ }
	show_mmr_bitfield 12 12 $val TESTMODE    { "" TESTMODE }
	show_mmr_bitfield 13 13 $val STESTMODE   { "" STESTMODE }
	show_mmr_bitfield 14 14 $val OTPLOCK     { "" OTPLOCK }
	show_mmr_bitfield 15 15 $val STESTROUTINE { "" STESTROUTINE }
	show_mmr_bitfield 16 16 $val ADDRERR     { "" ADDRERR }
	show_mmr_bitfield 17 17 $val LWERR       { "" LWERR }
	show_mmr_bitfield 18 18 $val RSTOUTERR   { "" RSTOUTERR }
}

# Show RCU_CRCTL bitfields
proc show_rcu_crctl { name val } {
	show_mmr_bitfield  0  0 $val CR0         { "" CR0 }
	show_mmr_bitfield  1  1 $val CR1         { "" CR1 }
	show_mmr_bitfield  2  2 $val CR2         { "" CR2 }
	show_mmr_bitfield  3  3 $val CR3         { "" CR3 }
}

# Show RCU_CRCTL bitfields
proc show_rcu_ctl { name val } {
}

# Show RCU_CRSTAT bitfields
proc show_rcu_crstat { name val } {
	show_mmr_bitfield  0  0 $val CR0         { "" CR0 }
	show_mmr_bitfield  1  1 $val CR1         { "" CR1 }
	show_mmr_bitfield  2  2 $val CR2         { "" CR2 }
	show_mmr_bitfield  3  3 $val CR3         { "" CR3 }
}

# Reset some debug registers in SHARC XI core debug component
proc reset_sharcxi_debug_regs { memap_name base_addr } {
	# Currently we only clear EMUCTL

	set csdbg_emuctl		[expr {$base_addr + 0x10}]
	set csdbg_extdata	[expr {$base_addr + 0x18}]

	$memap_name write_memory $csdbg_extdata 32 0
	$memap_name write_memory $csdbg_emuctl 32 0
}

# Configure CTIs so Cortex-A5 halt event will halt system peripherals, like watchdog timer
proc adspsc5xx_configure_cti {a5_cti0_base_addr} {
	set cti0_cticontrol		$a5_cti0_base_addr
	set cti0_ctiinen0		[expr {$a5_cti0_base_addr + 0x20}]
	set cti0_ctiinen1		[expr {$a5_cti0_base_addr + 0x24}]
	set cti0_ctiouten7		[expr {$a5_cti0_base_addr + 0xbc}]
	set cti0_lar			[expr {$a5_cti0_base_addr + 0xfb0}]

	set cti3_cticontrol		0x3110d000
	set cti3_ctiouten1		[expr {$cti3_cticontrol + 0xa4}]
	set cti3_ctiouten7		[expr {$cti3_cticontrol + 0xbc}]
	set cti3_lar			[expr {$cti3_cticontrol + 0xfb0}]

	# Unlock CTI0 (Cortex A5 CTI)
	# *pREG_CTI0_LAR = 0xC5ACCE55
	mww phys $cti0_lar 0xC5ACCE55

	# Unlock CTI3 (System CTI)
	# *pREG_CTI3_LAR = 0xC5ACCE55
	mww phys $cti3_lar 0xC5ACCE55

	# Enable CTI0 (Cortex A5 CTI)
	# *pREG_CTI0_CTICONTROL = 1
	mww phys $cti0_cticontrol 0x1

	# Enable CTI3 (System CTI)
	# *pREG_CTI3_CTICONTROL = 1
	mww phys $cti3_cticontrol 0x1

	# Connect DBGTRIGGER of Cortex A5 to channel 0
	# *pREG_CTI0_CTIINEN0 = 1
	mww phys $cti0_ctiinen0 0x1

	# Connect peripheral halt to channel 0
	# *pREG_CTI3_CTIOUTEN1 = 1
	mww phys $cti3_ctiouten1 0x1

	# Connect Cortex A5 DBGRESTART to channel 1
	# *pREG_CTI0_CTIOUTEN7 = 2
	mww phys $cti0_ctiouten7 0x2

	# Connect peripheral DBGRESTART to channel 1
	# *pREG_CTI3_CTIOUTEN7 = 2
	mww phys $cti3_ctiouten7 0x2
}

proc setup_cores_for_reset { chip_name } {
	reset_sharcxi_debug_regs $chip_name.apb 0x80001000
	reset_sharcxi_debug_regs $chip_name.apb 0x80005000
}

# system reset
proc adspsc5xx_system_reset { chip_name } {
	if {[string match *58* $chip_name]} {
		set rcu_ctl     0x3108b000
		set rcu_stat    [expr {$rcu_ctl + 0x4}]
		set rcu_bcode   [expr {$rcu_ctl + 0x1c}]
		set rcu_msg     [expr {$rcu_ctl + 0x60}]
		set rcu_msg_clr [expr {$rcu_ctl + 0x68}]
	} else {
		set rcu_ctl     0x3108c000
		set rcu_stat    [expr {$rcu_ctl + 0x4}]
		set rcu_bcode   [expr {$rcu_ctl + 0x28}]
		set rcu_msg     [expr {$rcu_ctl + 0x6c}]
		set rcu_msg_clr [expr {$rcu_ctl + 0x74}]
	}

	set cti3_cticontrol  0x3110d000
	set cti3_ctiapppulse [expr {$cti3_cticontrol + 0x1c}]
	set cti3_ctiouten2   [expr {$cti3_cticontrol + 0xa8}]
	set cti3_lar         [expr {$cti3_cticontrol + 0xfb0}]

	# define reset mask for a55 usage
	# Other 5xx devices included "setup_cores_for_reset" routine
	if {[string match *a55* $chip_name] || [string match *598* $chip_name]} {
		set a55_reset_msk 0x310AD010
		set ap_bus_access $chip_name.axi
	} else {
		set ap_bus_access $chip_name.ahb
		# Read BMODE from RCU0_STAT
		set data [memread32_phys $rcu_stat]
		set bootmode [expr {($data >> 8) & 0xf}]
		echo "Boot Mode $bootmode"

		# Use CTI to do system reset
		# Unlock CTI3 (System CTI)
		# *pREG_CTI3_LAR = 0xC5ACCE55
		mww phys $cti3_lar 0xC5ACCE55

		# Clear SHARC-XI debug registers for both SHARC-XI cores
		# If EMUCTL.EMUENA bit is set, the SHARC-XI core cannot be reset
		setup_cores_for_reset $chip_name
	}

	echo "start system reset..."

	# Clear REG_RCU0_MSG
	# *pREG_RCU0_MSG = 0x0
	mww phys $rcu_msg 0

	# Deassert RSTOUT in REG_RCU0_CTL
	# *pREG_RCU0_CTL |= 0x4
	pmmw $rcu_ctl 0x4 0

	# Clear REG_RCU0_STAT
	# *pREG_RCU0_STAT = 0x7000d
	mww phys $rcu_stat 0x7000d

	# Set HALT (bit 2)
	# *pREG_RCU0_BCODE = 0x4
	mww phys $rcu_bcode 0x4

	# For 58x devices system reset is handled strictly via RCU reg
	if {[string match *58* $chip_name]} {
		# Use RCU_CTL to do system reset
		# *pREG_RCU0_CTL |= 0x00000001;
		$ap_bus_access mww $rcu_ctl 0x1
	} else {
		# Enable CTI3 (System CTI)
		# *pREG_CTI3_CTICONTROL = 1
		mww phys $cti3_cticontrol 0x1

		# Connect CTITRIGOUT[2] of CTI3 to channel 2
		# *pREG_CTI3_CTIOUTEN2 = 4
		mww phys $cti3_ctiouten2 0x4

		if { [info exists a55_reset_msk] } {
			# Set RESET_CTLRSTMSK for warm reset(debug stays alive)
			mem_ap_write_reg $a55_reset_msk 0xB6 $ap_bus_access

			mww phys $cti3_ctiapppulse 0x4
		} else {
			# Send a signal to channel 2
			# *pREG_CTI3_CTIAPPPULSE = 4
			$ap_bus_access mww $cti3_ctiapppulse 0x4
		}
	}

	# Wait till Core 0 is idle
	# while((*pREG_RCU0_MSG & BITM_RCU_MSG_C0IDLE) == 0);
	set data 0
	set retry 0
	while { [expr {$data & 0x100}] == 0 } {
		set data [mem_ap_read_reg $rcu_msg $ap_bus_access]
		set retry [expr {$retry + 1}]
		if { $retry > 20 } break;
	}
	if { $retry > 20 } {
		set msg [format 0x%08x $data]
		echo "BCODE.HALT failed (REG_RCU0_MSG $msg)"
	}

	# update target state
	poll

	# Halt the core
	halt

	# separating end of system reset process
	# due to different memory accesses for adspsc59x vs adspsc59x_a55 devices
	if {[string match *a55* $chip_name] || [string match *598* $chip_name]} {
		# Clear C0IDLE from REG_RCU0_MSG
		# *pREG_RCU0_MSG_CLR = 0x100
		mem_ap_write_reg $rcu_msg_clr 0x100 $ap_bus_access

		# Now BOOT is done
		echo "system reset done"

		# clear REG_RCU0_MSG
		mem_ap_write_reg $rcu_msg 0 $ap_bus_access

		# clear REG_RCU0_BCODE
		mem_ap_write_reg $rcu_bcode 0 $ap_bus_access

		# Disable CTI3 (System CTI)
		mem_ap_write_reg $cti3_cticontrol 0 $ap_bus_access
	} else {
		# Clear C0IDLE from REG_RCU0_MSG
		# *pREG_RCU0_MSG_CLR = 0x100
		mww phys $rcu_msg_clr 0x100

		# Now BOOT is done
		echo "system reset done"

		# clear REG_RCU0_MSG
		mww phys $rcu_msg 0

		# clear REG_RCU0_BCODE
		mww phys $rcu_bcode 0

		# Disable CTI3 (System CTI)
		mww phys $cti3_cticontrol 0
	}
}

# core reset
proc adspsc59x_a55_core_reset {chip_name} {
	set rcu_ctl       0x3108c000
	set rcu_stat      [expr {$rcu_ctl + 0x4}]
	set rcu_crctl     [expr {$rcu_ctl + 0x8}]
	set rcu_crstat    [expr {$rcu_ctl + 0xC}]
	set rcu_bcode     [expr {$rcu_ctl + 0x28}]
	set rcu_svect0    [expr {$rcu_ctl + 0x2C}]
	set rcu_svect1    [expr {$rcu_ctl + 0x30}]
	set rcu_svect2    [expr {$rcu_ctl + 0x34}]
	set rcu_msg       [expr {$rcu_ctl + 0x6c}]
	set rcu_msg_clr   [expr {$rcu_ctl + 0x74}]

	# Read BMODE from RCU0_STAT
	set data [mem_ap_read_reg $rcu_stat $chip_name.axi]
	set bootmode [expr {($data >> 8) & 0xf}]

	# Clear RCU regs
	mem_ap_write_reg $rcu_ctl 0 $chip_name.axi
	mem_ap_write_reg $rcu_crctl 0 $chip_name.axi
	mem_ap_write_reg $rcu_crstat 0xFFFFFFF9 $chip_name.axi

	# Set HALT (bit 2)
	# *pREG_RCU0_BCODE = 0x4
	mem_ap_write_reg $rcu_bcode 0x4 $chip_name.axi

	# Set ARM starting address
	# *pREG_RCU0_SVECT0 = 0x40
	mem_ap_write_reg $rcu_svect0 0x40 $chip_name.axi

	# Set SHARC cores to entry points
	# *pREG_RCU0_SVECT1 = 0x00500004
	# *pREG_RCU0_SVECT2 = 0x00500004
	mem_ap_write_reg $rcu_svect1 0x00500004 $chip_name.axi
	mem_ap_write_reg $rcu_svect2 0x00500004 $chip_name.axi

	# clear CRn bit in CRSTAT register( bit is W1C )
	# *pREG_RCU0_CRSTAT = 0x1
	mem_ap_write_reg $rcu_crstat 0x1 $chip_name.axi

	# clear CRCTL
	# *pREG_RCU0_CRCTL = 0x0
	mem_ap_write_reg $rcu_crctl 0 $chip_name.axi

	# set bit 0 in CRCTL
	# *pREG_RCU0_CRCTL |= 0x1
	mem_ap_write_reg $rcu_crctl 0x1 $chip_name.axi

	# Wait till Core 0 is in reset
	set data 0
	set retry 0
	while { [expr {$data & 0x1}] == 0 } {
		set data [mem_ap_read_reg $rcu_crstat $chip_name.axi]
		set retry [expr {$retry + 1}]
		if { $retry > 20 } break;
	}
	if { $retry > 20 } {
		set msg [format 0x%08x $data]
		echo "Core was not put in reset"
	}

	# small delay to give reset time to take affect
	sleep 100

	# clear bit 0 in CRCTL
	set data [mem_ap_read_reg $rcu_crctl $chip_name.axi]
	set rcu_crctl_data  [expr {$data & 0xFFFFFFFE}]
	mem_ap_write_reg $rcu_crctl $rcu_crctl_data $chip_name.axi

	echo "core reset complete"
}

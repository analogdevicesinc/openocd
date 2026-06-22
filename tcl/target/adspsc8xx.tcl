# Common routines for Analog Devices ADSP-SC8xx
#
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2022-2025 Analog Devices, Inc.

# ADJC is the TAP name for the ADI JTAG Controller
proc adjc_enable_dap {adjc} {
	# Unlock the processor before enabling DAP
	adjc_unlock256 $adjc

	irscan $adjc 0x5 -endstate IRPAUSE
	drscan $adjc 8 0x4 -endstate RUN/IDLE

	runtest 2000
}

# Unlock the processor
proc adjc_unlock256 {adjc} {
	set userkey0 "_USERKEY0_Proc$::PROCNUM"
	set userkey1 "_USERKEY1_Proc$::PROCNUM"
	set userkey2 "_USERKEY2_Proc$::PROCNUM"
	set userkey3 "_USERKEY3_Proc$::PROCNUM"
	set userkey4 "_USERKEY4_Proc$::PROCNUM"
	set userkey5 "_USERKEY5_Proc$::PROCNUM"
	set userkey6 "_USERKEY6_Proc$::PROCNUM"
	set userkey7 "_USERKEY7_Proc$::PROCNUM"

	irscan $adjc 0xa -endstate RUN/IDLE
	drscan $adjc 32 [set ::$userkey0] 32 [set ::$userkey1] 32 [set ::$userkey2] 32 [set ::$userkey3] 32 [set ::$userkey4] 32 [set ::$userkey5] 32 [set ::$userkey6] 32 [set ::$userkey7] -endstate RUN/IDLE
}

# Configure CTIs so SHARC FX halt event will halt system peripherals, like watchdog timer
proc adspsc8xx_configure_fx_cti {sharcfx_cti0_base_addr sharcfx_ocd_base_addr} {
	set cti0_cticontrol		$sharcfx_cti0_base_addr
	set cti0_ctiinen1		[expr {$sharcfx_cti0_base_addr + 0x24}]
	set cti0_lar			[expr {$sharcfx_cti0_base_addr + 0xfb0}]

	set cti3_cticontrol		0x3110d000
	set cti3_ctiouten1		[expr {$cti3_cticontrol + 0xa4}]
	set cti3_ctiouten7		[expr {$cti3_cticontrol + 0xbc}]
	set cti3_lar			[expr {$cti3_cticontrol + 0xfb0}]

	set ocd_dcrset			[expr {$sharcfx_ocd_base_addr + 0x0c}]

	# Enable XOCDMode + BreakIn
	mww $ocd_dcrset 0x00410000

	# Unlock CTI0 (SHARCFX CTI)
	# *pREG_CTI0_LAR = 0xC5ACCE55
	mww $cti0_lar 0xC5ACCE55

	# Enable CTI0 (SHARCFX CTI)
	# *pREG_CTI0_CTICONTROL = 1
	mww $cti0_cticontrol 0x1

	# Connect DBGTRIGGER of SHARCFX to channel 0
	# *pREG_CTI0_CTIINEN1 = 1
	mww $cti0_ctiinen1 0x1

	# Enable CTI3 (System CTI)
	# *pREG_CTI3_CTICONTROL = 1
	mww $cti3_cticontrol 0x1

	# Connect peripheral halt to channel 0
	# *pREG_CTI3_CTIOUTEN1 = 1
	mww $cti3_ctiouten1 0x1

	# Connect peripheral restart to channel 1
	# *pREG_CTI3_CTIOUTEN7 = 2
	mww $cti3_ctiouten7 0x2
}

# Configure CTIs so Cortex-A55 halt event will halt system peripherals, like watchdog timer
proc adspsc8xx_configure_a55_cti {a55_cti_base_addr} {
	set cti0_cticontrol		$a55_cti_base_addr
	set cti0_ctiinen0		[expr {$a55_cti_base_addr + 0x20}]
	set cti0_ctiouten7		[expr {$a55_cti_base_addr + 0xbc}]
	set cti0_lar			[expr {$a55_cti_base_addr + 0xfb0}]

	set cti3_cticontrol		0x3110d000
	set cti3_ctiouten1		[expr {$cti3_cticontrol + 0xa4}]
	set cti3_ctiouten7		[expr {$cti3_cticontrol + 0xbc}]
	set cti3_lar			[expr {$cti3_cticontrol + 0xfb0}]

	# Unlock CTI0 (Cortex A55 CTI)
	# *pREG_CTI0_LAR = 0xC5ACCE55
	mww phys $cti0_lar 0xC5ACCE55

	# Unlock CTI3 (System CTI)
	# *pREG_CTI3_LAR = 0xC5ACCE55
	mww phys $cti3_lar 0xC5ACCE55

	# Enable CTI0 (Cortex A55 CTI)
	# *pREG_CTI0_CTICONTROL = 1
	mww phys $cti0_cticontrol 0x1

	# Enable CTI3 (System CTI)
	# *pREG_CTI3_CTICONTROL = 1
	mww phys $cti3_cticontrol 0x1

	# Connect DBGTRIGGER of Cortex A55 to channel 0
	# *pREG_CTI0_CTIINEN0 = 1
	mww phys $cti0_ctiinen0 0x1

	# Connect peripheral halt to channel 0
	# *pREG_CTI3_CTIOUTEN1 = 1
	mww phys $cti3_ctiouten1 0x1

	# Connect Cortex A55 DBGRESTART to channel 1
	# *pREG_CTI0_CTIOUTEN7 = 2
	mww phys $cti0_ctiouten7 0x2

	# Connect peripheral DBGRESTART to channel 1
	# *pREG_CTI3_CTIOUTEN7 = 2
	mww phys $cti3_ctiouten7 0x2
}

# core reset
proc adspsc84x_a55_core_reset {chip_name target_name} {
	set rcu_ctl       0x3108c000
	set rcu_stat      [expr {$rcu_ctl + 0x4}]
	set rcu_crctl     [expr {$rcu_ctl + 0x8}]
	set rcu_crstat    [expr {$rcu_ctl + 0xC}]
	set rcu_bcode     [expr {$rcu_ctl + 0x28}]
	set rcu_svect0    [expr {$rcu_ctl + 0x2c}]
	set rcu_svect2    [expr {$rcu_ctl + 0x34}]
	set resetBit 0x1

	# Read BMODE from RCU0_STAT
	set data [mem_ap_read_reg $rcu_stat $chip_name.axi]
	set bootmode [expr {($data >> 8) & 0xf}]
	echo "Boot Mode $bootmode"

	# Clear RCU regs
	mem_ap_write_reg $rcu_ctl 0 $chip_name.axi
	mem_ap_write_reg $rcu_crctl 0 $chip_name.axi

	# All asserted aside from SHARC-FX
	mem_ap_write_reg $rcu_crstat 0xFFFFFFD $chip_name.axi

	# Read BCODE and OR HALT (bit 2)
	# *pREG_RCU0_BCODE |= 0x1004
	set data [mem_ap_read_reg $rcu_bcode $chip_name.axi]
	set bcode [expr {($data >> 8) & 0xf} ]
	set bcode [expr {$bcode | 0x4} ]
	mem_ap_write_reg $rcu_bcode $bcode $chip_name.axi

	if { [string match "*a55_Core0" $target_name] } {
		# Set core to entry point
		mem_ap_write_reg $rcu_svect0 0x7D1 $chip_name.axi
		set resetBit 0x1
	} elseif { [string match "*a55_Core2" $target_name] } {
		# Set core to entry point
		mem_ap_write_reg $rcu_svect2 0x7D1 $chip_name.axi
		set resetBit 0x4
	}

	# clear CRn bit in CRSTAT register( bit is W1C )
	# *pREG_RCU0_CRSTAT = 0x1 (A55 Core 0) or 0x4 (A55 Core 2)
	mem_ap_write_reg $rcu_crstat $resetBit $chip_name.axi

	# turn polling off
	poll off

	# set bit 0 or bit 2 in CRCTL to assert core 0 (cortex a55)
	# or core 2 (cortex a55) reset signal, respectively
	# *pREG_RCU0_CRCTL = 0x1 or 0x4
	mem_ap_write_reg $rcu_crctl $resetBit $chip_name.axi

	# give time for reset to take affect
	sleep 1000

	# Wait till Core 0 is in reset
	set data 0
	set retry 0
	while { [expr {$data & $resetBit}] == 0 } {
		set data [mem_ap_read_reg $rcu_crstat $chip_name.axi]
		set retry [expr {$retry + 1}]
		if { $retry > 20 } break;
	}
	if { $retry > 20 } {
		set msg [format 0x%08x $data]
		echo "Core was not put in reset"
	}

	# small delay to give reset time to clear
	sleep 300

	# clear resetBit in CRCTL
	set data [mem_ap_read_reg $rcu_crctl $chip_name.axi]
	set mask [expr {0xFFFFFFFF & ~$resetBit}]
	set rcu_crctl_data  [expr {$data & $mask}]
	mem_ap_write_reg $rcu_crctl $rcu_crctl_data $chip_name.axi
	mem_ap_write_reg $rcu_crstat $resetBit $chip_name.axi

	poll on

	echo "Cortex-a55 core reset complete..."
}

proc adspsc83x_m33_core_reset {chip_name} {
   set rcu_ctl       0x3108c000
   set rcu_stat      [expr {$rcu_ctl + 0x4}]
   set rcu_crctl     [expr {$rcu_ctl + 0x8}]
   set rcu_crstat    [expr {$rcu_ctl + 0xC}]
   set rcu_bcode     [expr {$rcu_ctl + 0x28}]
   set rcu_svect2    [expr {$rcu_ctl + 0x34}]

   # Read BMODE from RCU0_STAT
   set data [mem_ap_read_reg $rcu_stat $chip_name.axi]
   set bootmode [expr {($data >> 8) & 0xf}]
   echo "Boot Mode $bootmode"

   # Set HALT (bit 2)
   # *pREG_RCU0_BCODE = 0x4
   mww $rcu_bcode 0x4

   # Set core to entry points
   # *pREG_RCU0_SVECT2 = 0x701
   mww $rcu_svect2 0x701

   # clear CRn bit in CRSTAT register( bit is W1C )
   # *pREG_RCU0_CRSTAT = 0x4
   mww $rcu_crstat 0x4

   # set bit 2 in CRCTL to assert core 2 (cortex m33) reset signal
   # *pREG_RCU0_CRCTL = 0x4
   mem_ap_write_reg $rcu_crctl 0x4 $chip_name.axi

   # give time for reset to take affect
   sleep 1000

   # Wait till Core 2 is in reset
   set data 0
   set retry 0
   while { [expr {$data & 0x4}] == 0 } {
      set data [mem_ap_read_reg $rcu_crstat $chip_name.axi]
      set retry [expr {$retry + 1}]
      if { $retry > 20 } break;
   }
   if { $retry > 20 } {
      set msg [format 0x%08x $data]
      echo "Core was not put in reset"
   }

   # clear CRCTL to deassert core 2 reset signal
   mem_ap_write_reg $rcu_crctl 0 $chip_name.axi

   # clear CRSTAT bit 1( bit is W1C )
   mem_ap_write_reg $rcu_crstat 0x4 $chip_name.axi

   # small delay to give reset time to clear
   sleep 300

   echo "Cortex-m33 core reset complete..."
}


# SHARC-FX core reset
proc adspsc8xx_sharcfx_core_reset { chip_name } {
	set rcu_ctl       0x3108c000
	set rcu_stat      [expr {$rcu_ctl + 0x4}]
	set rcu_crctl     [expr {$rcu_ctl + 0x8}]
	set rcu_crstat    [expr {$rcu_ctl + 0xC}]
	set rcu_bcode     [expr {$rcu_ctl + 0x28}]
	set rcu_svect1    [expr {$rcu_ctl + 0x30}]

	# Read BMODE from RCU0_STAT
	set data [mem_ap_read_reg $rcu_stat $chip_name.axi]
	set bootmode [expr {($data >> 8) & 0xf}]
	echo "Boot Mode $bootmode"

	# Set HALT (bit 2)
	# *pREG_RCU0_BCODE = 0x4
	mww $rcu_bcode 0x4

	# Set core to entry points
	# *pREG_RCU0_SVECT1 = 0x202000E4
	mww $rcu_svect1 0x202000E4

	# clear CRn bit in CRSTAT register( bit is W1C )
	# *pREG_RCU0_CRSTAT = 0x2
	mww $rcu_crstat 0x2

	# turn polling off
	poll off

	# set bit 1 in CRCTL to assert core 1 reset signal
	# *pREG_RCU0_CRCTL = 0x2
	mem_ap_write_reg $rcu_crctl 0x2 $chip_name.axi

	# give time for reset to take affect
	sleep 1000

	# Wait till Core 1 is in reset
	set data 0
	set retry 0
	while { [expr {$data & 0x2}] == 0 } {
		set data [mem_ap_read_reg $rcu_crstat $chip_name.axi]
		set retry [expr {$retry + 1}]
		if { $retry > 20 } break;
	}
	if { $retry > 20 } {
		set msg [format 0x%08x $data]
		echo "Core was not put in reset"
	}

	# clear CRCTL to deassert core 1 reset signal
	mem_ap_write_reg $rcu_crctl 0 $chip_name.axi

	# clear CRSTAT bit 1( bit is W1C )
	mem_ap_write_reg $rcu_crstat 0x2 $chip_name.axi

	# small delay to give reset time to clear
	sleep 300

	poll on

	echo "SHARC FX core reset complete..."
}

# SHARC-FX system reset
proc adspsc8xx_system_reset { chip_name system_reset_complete_var} {
	upvar #0 $system_reset_complete_var system_reset_complete
	set rcu_ctl     0x3108c000
	set rcu_stat    [expr {$rcu_ctl + 0x4}]
	set rcu_bcode   [expr {$rcu_ctl + 0x28}]
	set rcu_msg     [expr {$rcu_ctl + 0x6c}]

	set cti3_cticontrol  0x3110d000
	set cti3_ctiapppulse [expr {$cti3_cticontrol + 0x1c}]
	set cti3_ctiouten2   [expr {$cti3_cticontrol + 0xa8}]
	set cti3_lar         [expr {$cti3_cticontrol + 0xfb0}]

	#Set BITM_RCU_MSG_C1IDLE or BITM_RCU_MSG_C0IDLE for
	# adspsc83x or adspsc84x, respectively
	# define reset mask for a55 usage
	# Set idleBit
	switch -glob $chip_name {
		*83x* - *2184x* { set idleBit 0x200 }
		*sc84x*         { set idleBit 0x100 }
	}

	# Set a55_reset_msk_reg
	switch -glob $chip_name {
		*2184x* - *sc84x* { set a55_reset_msk_reg 0x31148010 }
	}

	echo "start system reset..."

	poll off

	# Clear REG_RCU0_MSG
	# *pREG_RCU0_MSG = 0x0
	mww $rcu_msg 0

	# Deassert RSTOUT in REG_RCU0_CTL
	# *pREG_RCU0_CTL |= 0x4
	set value 0
	set value [mem_ap_read_reg $rcu_ctl $chip_name.axi]
	set value [expr {$value | 0x4}]
	mww $rcu_ctl $value

	# Clear REG_RCU0_STAT
	mww $rcu_stat 0x7000d

	# Set HALT (bit 2)
	mww $rcu_bcode 0x4

	# Enable CTI3 (System CTI)
	mww $cti3_cticontrol 0x1

	# Connect CTITRIGOUT[2] of CTI3 to channel 2
	mww $cti3_ctiouten2 0x4

	# Set RESET_CTLRSTMSK for warm reset(debug stays alive)
	if { [info exists a55_reset_msk_reg] } {
		mem_ap_write_reg $a55_reset_msk_reg 0x2B6 $chip_name.axi
	}

	# this allows system reset to work correctly when happening from the FX core
	sleep 300
	mww $cti3_ctiapppulse 0x4

	# Wait till booting core is idle
	# while((*pREG_RCU0_MSG & BITM_RCU_MSG_C1IDLE) == 0);
	set data 0
	set retry 0
	while { [expr {$data & $idleBit}] == 0 } {
		set data [mem_ap_read_reg $rcu_msg $chip_name.axi]
		# add a delay before reading again to give reset time to take effect
		sleep 50
		set retry [expr {$retry + 1}]
		if { $retry > 40 } break;
	}
	if { $retry > 40 } {
		set msg [format 0x%08x $data]
		echo "BCODE.HALT failed (REG_RCU0_MSG $msg)"
	}

	# Now BOOT is done
	echo "system reset complete"

	# clear REG_RCU0_MSG
	mem_ap_write_reg $rcu_msg 0x0 $chip_name.axi

	# clear REG_RCU0_BCODE
	mem_ap_write_reg $rcu_bcode 0x0 $chip_name.axi

	# Disable CTI3 (System CTI)
	mem_ap_write_reg $cti3_cticontrol 0x0 $chip_name.axi

	poll on

	# With a heterogeneous connection, only do this once
	# There is no need to do system reset again through
	# the secondary core(s)
	# Cortex-m33 for adspsc83x
	# SHARC-FX and Cortex-A55 for adspsc84x
	set system_reset_complete 1
}
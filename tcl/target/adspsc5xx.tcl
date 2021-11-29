# Common routines for Analog Devices ADSP-SC5xx
#
# Copyright (c) 2015-2021 Analog Devices, Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.

# Unlock the processor
proc adjc_unlock {adjc} {
   global _USERKEY0
   global _USERKEY1
   global _USERKEY2
   global _USERKEY3

   irscan $adjc 0xa -endstate IRPAUSE
   drscan $adjc 32 $_USERKEY0 32 $_USERKEY1 32 $_USERKEY2 32 $_USERKEY3 -endstate RUN/IDLE
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
   log_debug "$name  [format  0x%08x $val]"
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
   log_debug "$name  [format  0x%08x $val]"
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
   log_debug "$name  [format  0x%08x $val]"
   show_mmr_bitfield  0  0 $val CR0         { "" CR0 }
   show_mmr_bitfield  1  1 $val CR1         { "" CR1 }
   show_mmr_bitfield  2  2 $val CR2         { "" CR2 }
   show_mmr_bitfield  3  3 $val CR3         { "" CR3 }
}

# Show RCU_CRCTL bitfields
proc show_rcu_ctl { name val } {
   log_debug "$name  [format  0x%08x $val]"
}

# Show RCU_CRSTAT bitfields
proc show_rcu_crstat { name val } {
   log_debug "$name  [format  0x%08x $val]"
   show_mmr_bitfield  0  0 $val CR0         { "" CR0 }
   show_mmr_bitfield  1  1 $val CR1         { "" CR1 }
   show_mmr_bitfield  2  2 $val CR2         { "" CR2 }
   show_mmr_bitfield  3  3 $val CR3         { "" CR3 }
}

# Reset some debug registers in SHARC XI core debug component
proc reset_sharcxi_debug_regs { dapname base_addr } {
   # Currently we only clear EMUCTL

   set csdbg_emuctl		[expr {$base_addr + 0x10}]
   set csdbg_extdata	[expr {$base_addr + 0x18}]

   $dapname writemem 1 $csdbg_extdata 0
   $dapname writemem 1 $csdbg_emuctl 0
}

# Configure CTIs so Cortex-A5 halt event will halt system peripherals, like watchdog timer
proc adspsc5xx_configure_cti {} {
   # Unlock CTI0 (Cortex A5 CTI)
   # *pREG_CTI0_LAR = 0xC5ACCE55
   mww phys 0x31128fb0 0xC5ACCE55

   # Unlock CTI3 (System CTI)
   # *pREG_CTI3_LAR = 0xC5ACCE55
   mww phys 0x3110dfb0 0xC5ACCE55

   # Enable CTI0 (Cortex A5 CTI)
   # *pREG_CTI0_CTICONTROL = 1
   mww phys 0x31128000 0x1

   # Enable CTI3 (System CTI)
   # *pREG_CTI3_CTICONTROL = 1
   mww phys 0x3110d000 0x1

   # Connect DBGTRIGGER of Cortex A5 to channel 0
   # *pREG_CTI0_CTIINEN0 = 1
   mww phys 0x31128020 0x1

   # Connect peripheral halt to channel 0
   # *pREG_CTI3_CTIOUTEN1 = 1
   mww phys 0x3110d0a4 0x1

   # Connect Cortex A5 DBGRESTART to channel 1
   # *pREG_CTI0_CTIOUTEN7 = 2
   mww phys 0x311280bc 0x2

   # Connect peripheral DBGRESTART to channel 1
   # *pREG_CTI3_CTIOUTEN7 = 2
   mww phys 0x3110d0bc 0x2
}

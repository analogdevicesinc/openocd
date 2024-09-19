# Common routines for Analog Devices ADSP-SC8xx
#
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2022-2024 Analog Devices, Inc.

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
   global _USERKEY0
   global _USERKEY1
   global _USERKEY2
   global _USERKEY3
   global _USERKEY4
   global _USERKEY5
   global _USERKEY6
   global _USERKEY7

   irscan $adjc 0xa -endstate RUN/IDLE
   drscan $adjc 32 $_USERKEY0 32 $_USERKEY1 32 $_USERKEY2 32 $_USERKEY3 32 $_USERKEY4 32 $_USERKEY5 32 $_USERKEY6 32 $_USERKEY7 -endstate RUN/IDLE
}

# Configure CTIs so SHARC FX halt event will halt system peripherals, like watchdog timer
proc adspsc83x_configure_cti {} {
   # Unlock CTI0 (SHARCFX CTI)
   # *pREG_CTI0_LAR = 0xC5ACCE55
   mww 0x31113fb0 0xC5ACCE55

   # Unlock CTI3 (System CTI)
   # *pREG_CTI3_LAR = 0xC5ACCE55
   mww 0x3110dfb0 0xC5ACCE55

   # Enable CTI0 (SHARCFX CTI)
   # *pREG_CTI0_CTICONTROL = 1
   mww 0x31113000 0x1

   # Connect DBGTRIGGER of SHARCFX to channel 0
   # *pREG_CTI0_CTIINEN1 = 1
   mww 0x31113024 0x1

   # Enable CTI3 (System CTI)
   # *pREG_CTI3_CTICONTROL = 1
   mww 0x3110d000 0x1

   # Connect peripheral halt to channel 0
   # *pREG_CTI3_CTIOUTEN1 = 1
   mww 0x3110d0a4 0x1

   # Connect peripheral restart to channel 1
   # *pREG_CTI3_CTIOUTEN7 = 2
   mww 0x3110d0bc 0x2
}

# Common routines used by ADI ADSP-SC58x and ADSP-SC57x boards and ADSP-SC59x SOM
#
# Copyright (c) 2015-2022 Analog Devices, Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.

proc smpu_config { smpu } {
   # Use SMPU instances to disable accesses to system memory that may not be
   # populated or needs to be initialized before being accessed. This will
   # avoid the possibility of an infinite stall in the system due to a
   # speculative access to a disabled or uninitialized memory. This is also
   # part of the workaround for silicon anomaly 20000018.

   if { $smpu == 0 } {
      set smpu_baseaddr 0x31007000
   } elseif { $smpu == 8 } {
      set smpu_baseaddr 0x31099000
   } elseif { $smpu == 9 } {
      set smpu_baseaddr 0x310a0000
   } elseif { $smpu == 10 } {
      set smpu_baseaddr 0x310a1000
   } else {
      log_error "unknown SMPU number"
      shutdown error
   }

   set smpu_ctl	        [expr {$smpu_baseaddr + 0x0}]
   set smpu_securectl   [expr {$smpu_baseaddr + 0x800}]

   # SMC - SMPU instance 0
   # *pREG_SMPU0_CTL |= ENUM_SMPU_CTL_RSDIS;
   pmmw $smpu_ctl 0x1 0

   # *pREG_SMPU0_SECURECTL = 0xf01;
   mww phys $smpu_securectl 0xf01
}

proc canfd_config { canfd_base } {
   global _CHIPNAME

   set canfd_cfg $canfd_base
   set canfd_rx_mb_gmsk    [expr {$canfd_cfg + 0x10}]
   set canfd_rx_14_msk     [expr {$canfd_cfg + 0x14}]
   set canfd_rx_15_msk     [expr {$canfd_cfg + 0x18}]
   set canfd_rx_fifo_gmsk  [expr {$canfd_cfg + 0x48}]
   set canfd_ctl2          [expr {$canfd_cfg + 0x34}]
   set canfd_ram           [expr {$canfd_cfg + 0x80}]
   set canfd_rx_imsk0      [expr {$canfd_cfg + 0x880}]
   set canfd_fd_ctl        [expr {$canfd_cfg + 0xc00}]
   set canfd_ram_rxfir     [expr {$canfd_cfg + 0xa80}]
   set canfd_ram_norm_mode [expr {$canfd_cfg + 0xab0}]
   set canfd_ram_fd_mode   [expr {$canfd_cfg + 0xf58}]

   if { $_CHIPNAME == "adspsc598" } {
      # use axi-ap
      set ap_num  2
   }
   if { $_CHIPNAME == "adspsc594" } {
      # use ahb-ap
      set ap_num 0
   }

   # /* Set the Freeze and Halt bit to enter freeze mode. */
   # pCANFDRegs->CFG |= BITM_CANFD_CFG_FRZ;
   # pCANFDRegs->CFG |= BITM_CANFD_CFG_HALT;
   pmmw $canfd_cfg 0x40000000 0x0
   pmmw $canfd_cfg 0x10000000 0x0

   # /* Wait for the Freeze acknowledgment. */
   # while((pCANFDRegs->CFG & BITM_CANFD_CFG_FRZACK) == 0u) { }
   set data [memread32_phys $canfd_cfg]
   while { ![expr {$data & 0x1000000}] } {
      set data [memread32_phys $canfd_cfg]
   }

   # Enable unrestricted write access to CANFD memory in Freeze mode
   # pCANFDRegs->CTL2 |= BITM_CANFD_CTL2_WRMFRZEN;
   pmmw $canfd_ctl2 0x10000000 0x0

   # /* Initialize the RAM area occupied by message buffers. */
   # for (i=0u; i<(RAM_SIZE/sizeof(uint32_t)); i++)
   # ram[i] = 0x0u;
   for {set i 0} {$i < 256} {incr i} {
      $_CHIPNAME.dap writemem $ap_num [expr {$canfd_ram + ($i * 4)}] 0x0
   }

   # /* Initialize the RAM area occupied by some of the CAN registers. */
   # pCANFDRegs->RX_MB_GMSK = 0xFFFFFFFFu;
   # pCANFDRegs->RX_14_MSK = 0xFFFFFFFFu;
   # pCANFDRegs->RX_15_MSK = 0xFFFFFFFFu;
   # pCANFDRegs->RX_FIFO_GMSK = 0xFFFFFFFFu;
   mww phys $canfd_rx_mb_gmsk   0xffffffff
   mww phys $canfd_rx_14_msk    0xffffffff
   mww phys $canfd_rx_15_msk    0xffffffff
   mww phys $canfd_rx_fifo_gmsk 0xffffffff

   # /* Initialize the IMSK registers which occupy RAM. */
   # for (i=0u;
   # i<(sizeof(pCANFDRegs->RX_IMSK)/sizeof(pCANFDRegs->RX_IMSK[0]));
   # i++)
   # pCANFDRegs->RX_IMSK[i] = 0u;
   for {set i 0} {$i < 64} {incr i} {
      $_CHIPNAME.dap writemem $ap_num [expr {$canfd_rx_imsk0 + ($i * 4)}] 0x0
   }

   # Initialize additional memory used for internal Serial Tx and Rx MB buffer structures for FD mode
   # pMbRamAddress = (uint32_t*)((uint8_t *)pMbAddressBase + CAN_SMB_FD_ADDR_OFFSET);
   # pMbRamLastAddress = (uint32_t*)((uint8_t *)pMbAddressBase + CAN_SMB_FD_ADDR_OFFSET + CAN_SMB_FD_SIZE);
   # while((uintptr_t)pMbRamAddress < (uintptr_t)pMbRamLastAddress)
   # {
   # *pMbRamAddress++ = 0U;
   # }
   for {set i 0} {$i < (168/4)} {incr i} {
      $_CHIPNAME.dap writemem $ap_num [expr {$canfd_ram_fd_mode + ($i * 4)}] 0x0
   }


   # Initialize additional memory used for internal Serial Tx and Rx MB buffer structures for Normal CAN mode
   # pMbRamAddress = (uint32_t*)((uint8_t *)pMbAddressBase + CAN_SMB_ADDR_OFFSET);
   # pMbRamLastAddress = (uint32_t*)((uint8_t *)pMbAddressBase + CAN_SMB_ADDR_OFFSET + CAN_SMB_SIZE);
   # while((uintptr_t)pMbRamAddress < (uintptr_t)pMbRamLastAddress)
   # {
   # *pMbRamAddress++ = 0U;
   # }
   for {set i 0} {$i < (48/4)} {incr i} {
      $_CHIPNAME.dap writemem $ap_num [expr {$canfd_ram_norm_mode + ($i * 4)}] 0x0
   }

   # Initialize additional memory used for internal RAM space used for storing RXFIR contents and some reserved space */
   # pMbRamAddress = (uint32_t*)((uint8_t *)pMbAddressBase + CAN_RXFIR_ADDR_OFFSET);
   # pMbRamLastAddress = (uint32_t*)((uint8_t *)pMbAddressBase + CAN_RXFIR_ADDR_OFFSET + CAN_RXFIR_SIZE);
   # while((uintptr_t)pMbRamAddress < (uintptr_t)pMbRamLastAddress)
   # {
   #  *pMbRamAddress++ = 0U;
   # }
   for {set i 0} {$i < (32/4)} {incr i} {
      $_CHIPNAME.dap writemem $ap_num [expr {$canfd_ram_rxfir + ($i * 4)}] 0x0
   }

    # Disable the unrestricted write access to CANFD memory in Freeze mode */
    # pCANFDRegs->CTL2 &= ~BITM_CANFD_CTL2_WRMFRZEN;
    pmmw $canfd_ctl2 0x0 0x10000000

   # Reset the CANFD registers through module soft reset
   # pCANFDRegs->CFG |= BITM_CANFD_CFG_SOFTRST;
   # while (pCANFDRegs->CFG  & BITM_CANFD_CFG_SOFTRST) {}
   pmmw $canfd_cfg 0x02000000 0x0
   set data [memread32_phys $canfd_cfg]
   while { [expr {$data & 0x02000000}] } {
      set data [memread32_phys $canfd_cfg]
   }

   # Zero initialize the registers
   # pCANFDRegs->CFG &= ~BITM_CANFD_CFG_MAXMB;
   # pCANFDRegs->FD_CTL = 0x0U;
   pmmw $canfd_cfg 0x0 0x0000007F
   mww phys $canfd_fd_ctl 0

   # /* Clear the Freeze and Halt bit to exit the freeze mode. */
   # pCANFDRegs->CFG &= ~(BITM_CANFD_CFG_FRZ|BITM_CANFD_CFG_HALT);
   pmmw $canfd_cfg 0x0 0x50000000

   # /* Wait for the Freeze acknowledgment to clear. */
   # while(pCANFDRegs->CFG & BITM_CANFD_CFG_FRZACK) { }
   set data [memread32_phys $canfd_cfg]
   while { [expr {$data & 0x1000000}] } {
      set data [memread32_phys $canfd_cfg]
   }
}

proc adspsc5xx_init_ddr3 { dmc } {
   global CHIPNAME

   if { $dmc == 0 } {
      set dmc_baseaddr 0x31070000
      set dummy_addr 0x80000000
   } else {
      set dmc_baseaddr 0x31073000
      set dummy_addr 0xc0000000
   }

   set dmc_ctl	        [expr {$dmc_baseaddr + 0x4}]
   set dmc_stat	        [expr {$dmc_baseaddr + 0x8}]
   set dmc_cfg	        [expr {$dmc_baseaddr + 0x40}]
   set dmc_tr0	        [expr {$dmc_baseaddr + 0x44}]
   set dmc_tr1	        [expr {$dmc_baseaddr + 0x48}]
   set dmc_tr2	        [expr {$dmc_baseaddr + 0x4c}]
   set dmc_mr	          [expr {$dmc_baseaddr + 0x60}]
   set dmc_mr1	        [expr {$dmc_baseaddr + 0x64}]
   set dmc_mr2	        [expr {$dmc_baseaddr + 0x68}]
   set dmc_dllctl	      [expr {$dmc_baseaddr + 0x80}]
   set dmc_cphy_ctl	    [expr {$dmc_baseaddr + 0x1c0}]
   set dmc_phy_ctl0	    [expr {$dmc_baseaddr + 0x1000}]
   set dmc_phy_ctl1	    [expr {$dmc_baseaddr + 0x1004}]
   set dmc_phy_ctl2	    [expr {$dmc_baseaddr + 0x1008}]
   set dmc_phy_ctl3	    [expr {$dmc_baseaddr + 0x100c}]
   set dmc_phy_ctl4	    [expr {$dmc_baseaddr + 0x1010}]
   set dmc_cal_padctl0  [expr {$dmc_baseaddr + 0x1034}]
   set dmc_cal_padctl2  [expr {$dmc_baseaddr + 0x103c}]

   # Configure SMPU (silicon anomaly 20000018)
   if { $CHIPNAME == "adspsc589" } {
      smpu_config 0
      smpu_config 8
   } elseif { $CHIPNAME == "adspsc573" } {
      smpu_config 0
   }

   # Set the RESETDLL bit (bit 11 of the DMC_PHY_CTL0 register) before CGU Initialization.
   # *pREG_DMC0_PHY_CTL0 |= BITM_DMC_PHY_CTL0_RESETDLL;
   pmmw $dmc_phy_ctl0 0x800 0

   # Set CGU clock select register to CLKO2/4 (ARM core)
   mww phys 0x3108d010 4

   # Reset processor to default power settings
   # Clear DISABLE and set EXIT_ACTIVE in CGU0_PLLCTL
   mww phys 0x3108d004 0x2
   # Set DF = 0 MSEL = 0x12 in CGU0_CTL
   mww phys 0x3108d000 0x1200
   # Set SYSSEL = 2 S0SEL = 2 S1SEL = 2 CSEL = 1 DSEL = 1 in CGU0_DIV
   mww phys 0x3108d00c 0x44014241

   # Read CGU0_STAT to make sure it's in FULL ON mode
   #mdw phys 0x3108d008

   # Clear the PHY_CTL0 after CGU Initialization
   mww phys $dmc_phy_ctl0 0

   # Wait for DLL lock - 9000 DCLK cycles
   # 1ms should be enough
   after 1

   # For DDR3 mode, set bit 1 and configure bits [5:2] of the DMC_CPHY_CTL
   # registers with WL(Write Latency) where WL is defined as CWL(CAS Write
   # Latency) + AL(Additive Latency) in DCLK cycles. Here WL = 6 as CWL
   # is 6 and AL is 0.
   #
   # When DMC_CPHY_CTL registers are written on the ADSP-SC57x/SC58x ARM
   # core 0 a false data abort exception is triggered that can only be
   # avoided by ensuring that asynchronous aborts are disabled by the A
   # bit in the Program State Registers (CSPR) being set. Versions of the
   # CCES uCOSII addin prior to 2.8.0 require an update in OSTaskStkInit
   # to avoid this problem.
   #
   # Writes to DMC_CPHY_CTL registers on the SHARC cores will also trigger an
   # INTR_SYS_C1_DATA_WRITE_ERR SEC error event. This can be cleared by
   # the SHARC before the ARM core enables asynchronous aborts, should it be
   # necessary to have asynchronous aborts enabled for the ARM.
   #
   # For additional information on this behavior refer to anomaly 20000091.

   # *pREG_DMC0_CPHY_CTL = 0x0000001A
   # Because of a processor anomaly, writing REG_DMC_CPHY_CTL through memory ap
   # will cause a DP error. So we have to write it through the core, i.e. without
   # the 'phys' flag. But writing it through the core will still cause a data
   # abort, although the write does go through fine. Thus we need to add the
   # new flag "ignore-data-abort" to ignore this data abort. Otherwise, the
   # commands after this command will not be run.
   mww ignore-data-abort $dmc_cphy_ctl 0x1a

   # *pREG_DMC0_PHY_CTL0 |= 0x0000000F;
   pmmw $dmc_phy_ctl0 0xf 0
   # *pREG_DMC0_PHY_CTL2 |= 0xFC000000;
   pmmw $dmc_phy_ctl2 0xfc000000 0
   # *pREG_DMC0_PHY_CTL3 |= 0x0A0000C0;
   pmmw $dmc_phy_ctl3 0xa0000c0 0

   # *pREG_DMC0_PHY_CTL1 = 0x00000000;
   mww phys $dmc_phy_ctl1 0

   # *pREG_DMC0_PHY_CTL4 = 0x00000000;
   mww phys $dmc_phy_ctl4 0

   # Program the PAD RTT and driver impedance values required here
   # *pREG_DMC0_CAL_PADCTL0 = 0xE0000000;
   mww phys $dmc_cal_padctl0 0xe0000000
   if { $CHIPNAME == "adspsc589" } {
      # *pREG_DMC0_CAL_PADCTL2 = 0x0078283C;
      mww phys $dmc_cal_padctl2 0x0078283c
   } elseif { $CHIPNAME == "adspsc573" } {
      # *pREG_DMC0_CAL_PADCTL2 = 0x00783C3C;
      mww phys $dmc_cal_padctl2 0x00783c3c
   }

   # Start calibration
   # *pREG_DMC0_CAL_PADCTL0 |= 0x10000000;
   pmmw $dmc_cal_padctl0 0x10000000 0

   # Wait for PAD calibration to complete - 300 DCLK cycle.
   # 1ms should be enough
   after 1

   # *pREG_DMC0_CFG = 0x00000522;
   mww phys $dmc_cfg 0x00000522
   # *pREG_DMC0_TR0 = 0x41711646;
   mww phys $dmc_tr0 0x41711646
   # *pREG_DMC0_TR1 = 0x40480db6;
   mww phys $dmc_tr1 0x40480db6
   # *pREG_DMC0_TR2 = 0x00347417;
   mww phys $dmc_tr2 0x00347417
   # *pREG_DMC0_MR = 0x00000730;
   mww phys $dmc_mr 0x00000730
   # *pREG_DMC0_MR1 = 0x00000006;
   mww phys $dmc_mr1 0x00000006
   # *pREG_DMC0_MR2 = 0x00000008;
   mww phys $dmc_mr2 0x00000008
   # *pREG_DMC0_CTL = 0x00000405;
   mww phys $dmc_ctl 0x00000405

   # Wait till INITDONE is set
   # while((*pREG_DMC0_STAT&BITM_DMC_STAT_INITDONE)==0);
   set data 0
   while { [expr {$data & 4}] == 0 } {
      set data [memread32_phys $dmc_stat]
   }

   # *pREG_DMC0_DLLCTL = 0x00000948;
   mww phys $dmc_dllctl 0x00000948

   # Workaround for silicon anomaly 20000037
   # Dummy read
   set data [memread32 $dummy_addr]
   # *pREG_DMC0_PHY_CTL0|=0x1000;
   # *pREG_DMC0_PHY_CTL0&=~0x1000;
   set data [memread32_phys $dmc_phy_ctl0]
   mww phys $dmc_phy_ctl0 [expr {$data | 0x1000}]
   mww phys $dmc_phy_ctl0 [expr {$data & ~0x1000}]
}

proc adspsc5xx_init_ddr2 { } {
   set dmc_baseaddr 0x31070000
   set dummy_addr 0x80000000

   set dmc_ctl			[expr {$dmc_baseaddr + 0x4}]
   set dmc_stat			[expr {$dmc_baseaddr + 0x8}]
   set dmc_cfg			[expr {$dmc_baseaddr + 0x40}]
   set dmc_tr0			[expr {$dmc_baseaddr + 0x44}]
   set dmc_tr1			[expr {$dmc_baseaddr + 0x48}]
   set dmc_tr2			[expr {$dmc_baseaddr + 0x4c}]
   set dmc_mr			[expr {$dmc_baseaddr + 0x60}]
   set dmc_emr1			[expr {$dmc_baseaddr + 0x64}]
   set dmc_emr2			[expr {$dmc_baseaddr + 0x68}]
   set dmc_dllctl		[expr {$dmc_baseaddr + 0x80}]
   set dmc_phy_ctl0		[expr {$dmc_baseaddr + 0x1000}]
   set dmc_phy_ctl1		[expr {$dmc_baseaddr + 0x1004}]
   set dmc_phy_ctl2		[expr {$dmc_baseaddr + 0x1008}]
   set dmc_phy_ctl3		[expr {$dmc_baseaddr + 0x100c}]
   set dmc_phy_ctl4		[expr {$dmc_baseaddr + 0x1010}]
   set dmc_cal_padctl0	[expr {$dmc_baseaddr + 0x1034}]
   set dmc_cal_padctl2	[expr {$dmc_baseaddr + 0x103c}]

   # Configure SMPU (silicon anomaly 20000018)
   smpu_config 0
   smpu_config 8
   smpu_config 10

   # Set the RESETDLL bit (bit 11 of the DMC_PHY_CTL0 register) before CGU Initialization.
   # *pREG_DMC0_PHY_CTL0 |= BITM_DMC_PHY_CTL0_RESETDLL;
   pmmw $dmc_phy_ctl0 0x800 0

   # Set CGU clock select register to CLKO2/4 (ARM core)
   mww phys 0x3108d010 4

   # Reset processor to default power settings
   # Clear DISABLE and set EXIT_ACTIVE in CGU0_PLLCTL
   mww phys 0x3108d004 0x2
   # Set DF = 0 MSEL = 0x10 in CGU0_CTL
   mww phys 0x3108d000 0x1000
   # Set SYSSEL = 2 S0SEL = 2 S1SEL = 2 CSEL = 1 DSEL = 1 in CGU0_DIV
   mww phys 0x3108d00c 0x44014241

   # Read CGU0_STAT to make sure it's in FULL ON mode
   #mdw phys 0x3108d008

   # Clear the PHY_CTL0 after CGU Initialization
   mww phys $dmc_phy_ctl0 0

   # Wait for DLL lock - 9000 DCLK cycles
   # 1ms should be enough
   after 1

   # *pREG_DMC0_PHY_CTL0 |= 0x0000000F;
   pmmw $dmc_phy_ctl0 0xf 0
   # *pREG_DMC0_PHY_CTL2 |= 0xFC000000;
   pmmw $dmc_phy_ctl2 0xfc000000 0
   # *pREG_DMC0_PHY_CTL3 |= 0x0A0000C0;
   pmmw $dmc_phy_ctl3 0xa0000c0 0

   # *pREG_DMC0_PHY_CTL1 = 0x00000000;
   mww phys $dmc_phy_ctl1 0

   # *pREG_DMC0_PHY_CTL4 = 0x00000001;
   mww phys $dmc_phy_ctl4 1

   # Program the PAD RTT and driver impedance values required here
   # *pREG_DMC0_CAL_PADCTL0 = 0xE0000000;
   mww phys $dmc_cal_padctl0 0xe0000000
   # *pREG_DMC0_CAL_PADCTL2 = 0x0078283C;
   mww phys $dmc_cal_padctl2 0x0078283c

   # Start calibration
   # *pREG_DMC0_CAL_PADCTL0 |= 0x10000000;
   pmmw $dmc_cal_padctl0 0x10000000 0

   # Wait for PAD calibration to complete - 300 DCLK cycle.
   # 1ms should be enough
   after 1

   # *pREG_DMC0_CFG = 0x00000522;
   mww phys $dmc_cfg 0x00000522
   # *pREG_DMC0_TR0 = 0x21610535;
   mww phys $dmc_tr0 0x21610535
   # *pREG_DMC0_TR1 = 0x404e0c30;
   mww phys $dmc_tr1 0x404e0c30
   # *pREG_DMC0_TR2 = 0x00326312;
   mww phys $dmc_tr2 0x00326312
   # *pREG_DMC0_MR = 0x00000a52;
   mww phys $dmc_mr 0x00000a52
   # *pREG_DMC0_EMR1 = 0x00000004;
   mww phys $dmc_emr1 0x00000004
   # *pREG_DMC0_EMR2 = 0x00000000;
   mww phys $dmc_emr2 0x00000000
   # *pREG_DMC0_CTL = 0x00000404;
   mww phys $dmc_ctl 0x00000404

   # Wait till INITDONE is set
   # while((*pREG_DMC0_STAT&BITM_DMC_STAT_INITDONE)==0);
   set data 0
   while { [expr {$data & 4}] == 0 } {
      set data [memread32_phys $dmc_stat]
   }

   # *pREG_DMC0_DLLCTL = 0x00000948;
   mww phys $dmc_dllctl 0x00000948

   # Workaround for silicon anomaly 20000037
   # Dummy read
   set data [memread32 $dummy_addr]
   # *pREG_DMC0_PHY_CTL0|=0x1000;
   # *pREG_DMC0_PHY_CTL0&=~0x1000;
   set data [memread32_phys $dmc_phy_ctl0]
   mww phys $dmc_phy_ctl0 [expr {$data | 0x1000}]
   mww phys $dmc_phy_ctl0 [expr {$data & ~0x1000}]
}

proc adspsc59x_init_ddr3 { dmc } {
   global _CHIPNAME

   set dmc_baseaddr       0x31070000
   set dmc_ctl            [expr {$dmc_baseaddr + 0x4}]
   set dmc_stat           [expr {$dmc_baseaddr + 0x8}]
   set dmc_cfg            [expr {$dmc_baseaddr + 0x40}]
   set dmc_tr0            [expr {$dmc_baseaddr + 0x44}]
   set dmc_tr1            [expr {$dmc_baseaddr + 0x48}]
   set dmc_tr2            [expr {$dmc_baseaddr + 0x4c}]
   set dmc_mr             [expr {$dmc_baseaddr + 0x60}]
   set dmc_mr1            [expr {$dmc_baseaddr + 0x64}]
   set dmc_mr2            [expr {$dmc_baseaddr + 0x68}]
   set dmc_emr3           [expr {$dmc_baseaddr + 0x6C}]
   set dmc_dllctl         [expr {$dmc_baseaddr + 0x80}]
   set dmc_ddr_lane0_ctl0 [expr {$dmc_baseaddr + 0x1000}]
   set dmc_ddr_lane0_ctl1 [expr {$dmc_baseaddr + 0x1004}]
   set dmc_ddr_lane1_ctl0 [expr {$dmc_baseaddr + 0x100C}]
   set dmc_ddr_lane1_ctl1 [expr {$dmc_baseaddr + 0x1010}]
   set dmc_ddr_root_ctl   [expr {$dmc_baseaddr + 0x1018}]
   set dmc_ddr_zq_ctl0    [expr {$dmc_baseaddr + 0x1034}]
   set dmc_ddr_zq_ctl1    [expr {$dmc_baseaddr + 0x1038}]
   set dmc_ddr_zq_ctl2    [expr {$dmc_baseaddr + 0x103C}]
   set dmc_ddr_ca_ctl     [expr {$dmc_baseaddr + 0x1068}]
   set dmc_ddr_scratch2   [expr {$dmc_baseaddr + 0x1074}]
   set dmc_ddr_scratch3   [expr {$dmc_baseaddr + 0x1078}]
   set dmc_ddr_scratch4   [expr {$dmc_baseaddr + 0x107c}]
   set dmc_ddr_scratch5   [expr {$dmc_baseaddr + 0x1080}]
   set dmc_ddr_scratch6   [expr {$dmc_baseaddr + 0x1084}]
   set dmc_ddr_scratch7   [expr {$dmc_baseaddr + 0x1088}]

   set cgu0_ctl           0x3108d000
   set cgu0_pllctl        [expr {$cgu0_ctl + 0x4}]
   set cgu0_stat          [expr {$cgu0_ctl + 0x8}]
   set cgu0_div           [expr {$cgu0_ctl + 0xC}]
   set cgu0_clkoutsel     [expr {$cgu0_ctl + 0x10}]
   set cgu0_divex         [expr {$cgu0_ctl + 0x40}]

   set cgu1_ctl           0x3108e000
   set cgu1_pllctl        [expr {$cgu1_ctl + 0x4}]
   set cgu1_stat          [expr {$cgu1_ctl + 0x8}]
   set cgu1_div           [expr {$cgu1_ctl + 0xC}]
   set cgu1_clkoutsel     [expr {$cgu1_ctl + 0x10}]
   set cgu1_divex         [expr {$cgu1_ctl + 0x40}]

   set cdu_cfg0     0x3108f000
   set cdu_cfg1     [expr {$cdu_cfg0 + 0x4}]
   set cdu_cfg2     [expr {$cdu_cfg0 + 0x8}]
   set cdu_cfg3     [expr {$cdu_cfg0 + 0xc}]
   set cdu_cfg4     [expr {$cdu_cfg0 + 0x10}]
   set cdu_cfg5     [expr {$cdu_cfg0 + 0x14}]
   set cdu_cfg6     [expr {$cdu_cfg0 + 0x18}]
   set cdu_cfg7     [expr {$cdu_cfg0 + 0x1C}]
   set cdu_cfg8     [expr {$cdu_cfg0 + 0x20}]
   set cdu_cfg9     [expr {$cdu_cfg0 + 0x24}]
   set cdu_cfg10    [expr {$cdu_cfg0 + 0x28}]
   set cdu_cfg11    [expr {$cdu_cfg0 + 0x2C}]
   set cdu_cfg12    [expr {$cdu_cfg0 + 0x30}]
   set cdu_cfg13    [expr {$cdu_cfg0 + 0x34}]
   set cdu_cfg14    [expr {$cdu_cfg0 + 0x38}]
   set cdu_stat     [expr {$cdu_cfg0 + 0x40}]
   set cdu_clkinsel [expr {$cdu_cfg0 + 0x44}]

   set miscreg_pll2_control   0x310A902C

   # Reset DMC Lane by setting the DMC_DDR_LANE0_CTL0.CB_RSTDLL
   # and DMC_DDR_LANE1_CTL0.CB_RSTDLL bits
   # *pREG_DMC0_DDR_LANE0_CTL0 |= BITM_DMC_DDR_LANE0_CTL0_CB_RSTDLL;
   # *pREG_DMC0_DDR_LANE1_CTL0 |= BITM_DMC_DDR_LANE1_CTL0_CB_RSTDLL;
   pmmw $dmc_ddr_lane0_ctl0 0x100 0x0
   pmmw $dmc_ddr_lane1_ctl0 0x100 0x0

   # Wait for DLL lock - 9000 DCLK cycles
   # 1ms should be enough
   after 1

   ###### Call to adi_configDCLK_1()
   # MSEL update
   # Clear BYPASSB, DSEL_DIV_CHG and MSELDF_CHG
   # *pREG_MISCREG_PLL2_CONTROL &= ~(BITM_MISCREG_PLL2_CONTROL_BYPASSB | BITM_MISCREG_PLL2_CONTROL_MSELDF_CHG | BITM_MISCREG_PLL2_CONTROL_DSEL_DIV_CHG) ;
   # Cclkdelay(20);
   pmmw $miscreg_pll2_control 0x0 0x7
   after 1

   # update MSEL ("Msel" == 63)
   # *pREG_MISCREG_PLL2_CONTROL &= ~BITM_MISCREG_PLL2_CONTROL_MSEL;
   # *pREG_MISCREG_PLL2_CONTROL |= ((Msel << BITP_MISCREG_PLL2_CONTROL_MSEL)& BITM_MISCREG_PLL2_CONTROL_MSEL);
   pmmw $miscreg_pll2_control 0x0 0x000007F0
   pmmw $miscreg_pll2_control 0x000003F0 0x0

   # set MSELDF_CHG
   # *pREG_MISCREG_PLL2_CONTROL |= BITM_MISCREG_PLL2_CONTROL_MSELDF_CHG;
   pmmw $miscreg_pll2_control 0x00000002 0x0
   # Cclkdelay(768);
   # Cclkdelay(92384);
   after 1

   # Set BYPASSB control bit
   # *pREG_MISCREG_PLL2_CONTROL |= BITM_MISCREG_PLL2_CONTROL_BYPASSB;
   pmmw $miscreg_pll2_control 0x00000001 0x0

   # DCLK Divider update
   # Clear BYPASSB, DSEL_DIV_CHG and MSELDF_CHG
   # *pREG_MISCREG_PLL2_CONTROL &= ~(BITM_MISCREG_PLL2_CONTROL_BYPASSB | BITM_MISCREG_PLL2_CONTROL_MSELDF_CHG | BITM_MISCREG_PLL2_CONTROL_DSEL_DIV_CHG) ;
   pmmw $miscreg_pll2_control 0x0 0x7

   # update DSEL ("Dsel" == 1)
   # *pREG_MISCREG_PLL2_CONTROL &= ~BITM_MISCREG_PLL2_CONTROL_DSEL;
   # *pREG_MISCREG_PLL2_CONTROL |= ((Dsel<< BITP_MISCREG_PLL2_CONTROL_DSEL)& BITM_MISCREG_PLL2_CONTROL_DSEL);
   pmmw $miscreg_pll2_control 0x0 0x0001F000
   pmmw $miscreg_pll2_control 0x00001000 0x0

   # set DSEL_DIV_CHG
   # *pREG_MISCREG_PLL2_CONTROL |= BITM_MISCREG_PLL2_CONTROL_DSEL_DIV_CHG;
   # Cclkdelay(768);
   pmmw $miscreg_pll2_control 0x00000004 0x0
   after 1

   # Set BYPASSB control bit
   # *pREG_MISCREG_PLL2_CONTROL |= BITM_MISCREG_PLL2_CONTROL_BYPASSB;
   pmmw $miscreg_pll2_control 0x00000001 0x0

   # Set the DDRCLK_FROM_3RDPLL [11] bit to connect the 3rd PLL DCLK to DMC
   # *pREG_MISCREG_PLL2_CONTROL |= BITM_MISCREG_PLL2_CONTROL_DDRCLK_FROM_3RDPLL;
   pmmw $miscreg_pll2_control 0x00000800 0x0
   ###### End of call to adi_configDCLK_1()

   # Configure the CDU (adi_pwr_CDUInit)
   set cdu_cfg_in0_en 0x1
   set cdu_cfg_in1_en 0x3
   set cdu_cfg_in2_en 0x7

   mww phys $cdu_cfg0 $cdu_cfg_in0_en
   mww phys $cdu_cfg1 $cdu_cfg_in0_en

   if { $_CHIPNAME == "adspsc598" } {
      mww phys $cdu_cfg2 $cdu_cfg_in2_en
      mww phys $cdu_cfg3 $cdu_cfg_in0_en
   } else {
      mww phys $cdu_cfg2 $cdu_cfg_in0_en
      mww phys $cdu_cfg3 $cdu_cfg_in1_en
   }
   mww phys $cdu_cfg4 $cdu_cfg_in1_en
   mww phys $cdu_cfg5 $cdu_cfg_in0_en
   mww phys $cdu_cfg6 $cdu_cfg_in0_en
   mww phys $cdu_cfg7 $cdu_cfg_in0_en
   mww phys $cdu_cfg8 $cdu_cfg_in1_en
   mww phys $cdu_cfg9 $cdu_cfg_in0_en
   mww phys $cdu_cfg10 $cdu_cfg_in0_en
   mww phys $cdu_cfg12 $cdu_cfg_in0_en
   if { $_CHIPNAME == "adspsc598" } {
      mww phys $cdu_cfg13 $cdu_cfg_in1_en
      mww phys $cdu_cfg14 $cdu_cfg_in1_en
   }

   # CGU0 Configuration
   # If PLL is disabled, then enable it
   # if(!(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLEN))
   #  {pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLEN;}
   set data [memread32_phys $cgu0_stat]
   if { ![expr {$data & 0x1}] } {
      pmmw $cgu0_pllctl 0x8 0x0
   }

   # If PLL is bypassed, then switch power mode from Active to Full on
   # if(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLBP)
   # {pDevice->pCguRegs->CGU_PLLCTL = BITM_CGU_PLLCTL_PLLBPCL;}
   set data [memread32_phys $cgu0_stat]
   if { [expr {$data & 0x2}] } {
      mww phys $cgu0_pllctl 0x2
   }

   # Wait for alignment to be done
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN){}
   set data [memread32_phys $cgu0_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu0_stat]
   }


   # Set CGU0_DIV
   # pADI_CGU_Param_List.cgu0_settings.clocksettings.div_CSEL        = 2;
   # pADI_CGU_Param_List.cgu0_settings.clocksettings.div_S0SEL       = 4;
   # pADI_CGU_Param_List.cgu0_settings.clocksettings.div_SYSSEL      = 4;
   # pADI_CGU_Param_List.cgu0_settings.clocksettings.div_S1SEL       = 2;
   # pADI_CGU_Param_List.cgu0_settings.clocksettings.div_DSEL        = 3; // 2 for SC594
   # pADI_CGU_Param_List.cgu0_settings.clocksettings.div_OSEL        = 8;
   if { $_CHIPNAME == "adspsc598" } {
      mww phys $cgu0_div 0x2034482
   } else {
      mww phys $cgu0_div 0x2024482
   }

   # adi_pwr_WriteDIVCTLLocal()
   # Put PLL in to Bypass Mode - call adi_pwr_ConfigurePLLControlReg()
   # Enable PLL
   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLEN
   pmmw $cgu0_pllctl 0x8 0
   # Wait till PLL is enabled
   # while(!(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLEN)) {};
   set data [memread32_phys $cgu0_stat]
   while { ![expr {$data & 0x1}] } {
      set data [memread32_phys $cgu0_stat]
   }

   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLBPST
   pmmw $cgu0_pllctl 0x1 0
   # Wait till PLL is bypassed
   # while(!(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLBP)) {};
   set data [memread32_phys $cgu0_stat]
   while { ![expr {$data & 0x2}] } {
      set data [memread32_phys $cgu0_stat]
   }

   # Wait until clocks are aligned
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu0_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu0_stat]
   }

   # Program the CTL register
   # pDevice->pCguRegs->CGU_CTL =  dNewCguCtl;
   mww phys $cgu0_ctl 0x25000

   # Take PLL out of Bypass Mode
   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLEN;
    pmmw $cgu0_pllctl 0x8 0

   # Wait till PLL is enabled
   # while((pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLEN) != BITM_CGU_STAT_PLLEN)
   set data [memread32_phys $cgu0_stat]
   while { ![expr {$data & 0x1}] } {
      set data [memread32_phys $cgu0_stat]
   }

   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLBPCL;
   pmmw $cgu0_pllctl 0x2 0

   # Wait until clocks are aligned
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu0_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu0_stat]
   }

   # Wait for No-Bypass to reflect in the status
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLBP) {}
   set data [memread32_phys $cgu0_stat]
   while { [expr {$data & 0x2}] } {
      set data [memread32_phys $cgu0_stat]
   }

   # adi_pwr_WriteDIVCTLLocal()
   if { $_CHIPNAME == "adspsc598" } {
      # Wait until the S0SELEXEN or S1SELEXEN enable bit is actually set
      # while(!(pDevice->pCguRegs->CGU_CTL & (BITM_CGU_CTL_S1SELEXEN|BITM_CGU_CTL_S0SELEXEN))) {}
      set data [memread32_phys $cgu0_ctl]
      while { ![expr {$data & 0x30000}] } {
         set data [memread32_phys $cgu0_ctl]
      }
   } else {
      # Wait until the S1SELEXEN enable bit is actually set
      # while(!(pDevice->pCguRegs->CGU_CTL & BITM_CGU_CTL_S1SELEXEN)) {}
      set data [memread32_phys $cgu0_ctl]
      while { ![expr {$data & 0x20000}] } {
         set data [memread32_phys $cgu0_ctl]
      }
   }

   # Update the new Divider values for S1SELEX via DIVEX
   # pADI_CGU_Param_List.cgu0_settings.clocksettings.divex_S1SELEX   = 6;
   # pDevice->pCguRegs->CGU_DIVEX = dNewCguSCLKExDiv;
   mww phys $cgu0_divex 0x60030

   # Wait until clocks are aligned
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu0_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu0_stat]
   }


   # Update the new Divider values with UPDT bit
   # pDevice->pCguRegs->CGU_DIV =  dNewCguDiv | BITM_CGU_DIV_UPDT;
   if { $_CHIPNAME == "adspsc598" } {
      mww phys $cgu0_div 0x42034482
   } else {
      mww phys $cgu0_div 0x42024482
   }

   # Wait until Update bit is set
   # while(!(pDevice->pCguRegs->CGU_DIV & BITM_CGU_DIV_UPDT)) {}
   set data [memread32_phys $cgu0_div]
   while { ![expr {$data & 0x40000000}] } {
      set data [memread32_phys $cgu0_div]
   }

   # Wait until clocks are aligned, and PLL is locked
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu0_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu0_stat]
   }

   # CGU1 Configuration

   # adi_pwr_ClockInitLocal()

   # If PLL is disabled, then enable it
   # if(!(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLEN))
   #  {pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLEN;}
   set data [memread32_phys $cgu1_stat]
   if { ![expr {$data & 0x1}] } {
      pmmw $cgu1_pllctl 0x8 0x0
   }

   # If PLL is bypassed, then switch power mode from Active to Full on
   # if(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLBP)
   # pDevice->pCguRegs->CGU_PLLCTL = BITM_CGU_PLLCTL_PLLBPCL;
   set data [memread32_phys $cgu1_stat]
   if { [expr {$data & 0x2}] } {
      mww phys $cgu1_pllctl 0x2
   }

   # Wait for alignment to be done
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN){}
   set data [memread32_phys $cgu1_stat]
   while { [expr {$data & 0x8}] } {
     set data [memread32_phys $cgu1_stat]
  }


   # Put PLL in to Bypass Mode

   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLEN;
   # Wait till PLL is enabled
   # while((pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLEN) != BITM_CGU_STAT_PLLEN)
   pmmw $cgu1_pllctl 0x8 0x0
   set data [memread32_phys $cgu1_stat]
   while { ![expr {$data & 0x1}] } {
      set data [memread32_phys $cgu1_stat]
   }

   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLBPST;
   # Wait for Bypass to reflect in the status
   # while(!(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLBP)) {};
   pmmw $cgu1_pllctl 0x1 0x0
   set data [memread32_phys $cgu1_stat]
   while { ![expr {$data & 0x2}] } {
      set data [memread32_phys $cgu1_stat]
   }

   # Wait till clocks are aligned
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu1_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu1_stat]
   }

   # Program the CTL register
   # pDevice->pCguRegs->CGU_CTL =  dNewCguCtl;
   if { $_CHIPNAME == "adspsc598" } {
      mww phys $cgu1_ctl 0x34800
   } else {
      mww phys $cgu1_ctl 0x4000
   }

   # Take PLL out of Bypass Mode
   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLEN;
   pmmw $cgu1_pllctl 0x8 0

   # Wait till PLL is enabled
   # while((pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLEN) != BITM_CGU_STAT_PLLEN)
   set data [memread32_phys $cgu1_stat]
   while { ![expr {$data & 0x1}] } {
      set data [memread32_phys $cgu1_stat]
   }

   # pDevice->pCguRegs->CGU_PLLCTL |= BITM_CGU_PLLCTL_PLLBPCL;
   pmmw $cgu1_pllctl 0x2 0

   # Wait until clocks are aligned
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu1_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu1_stat]
   }

   # Wait for No-Bypass to reflect in the status
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_PLLBP) {}
   set data [memread32_phys $cgu1_stat]
   while { [expr {$data & 0x2}] } {
      set data [memread32_phys $cgu1_stat]
   }

   if { $_CHIPNAME == "adspsc598" } {
      # adi_pwr_WriteDIVCTLLocal()
      # Wait until the S1SELEXEN enable or S0SELEXEN enable bit is actually set
      # while(!(pDevice->pCguRegs->CGU_CTL & (BITM_CGU_CTL_S1SELEXEN|BITM_CGU_CTL_S0SELEXEN))) {}
      set data [memread32_phys $cgu1_ctl]
      while { ![expr {$data & 0x30000}] } {
         set data [memread32_phys $cgu1_ctl]
      }

      # Update the new Divider values for S1SELEX via DIVEX
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.divex_S1SELEX   = 90;
      # pDevice->pCguRegs->CGU_DIVEX = dNewCguSCLKExDiv;
      mww phys $cgu1_divex 0x5a0024
   }

   # wait till clocks are aligned
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu1_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu1_stat]
   }

   # Set CGU1_DIV
   if { $_CHIPNAME == "adspsc598" } {
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_CSEL        = 16;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_SYSSEL      = 8;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_S0SEL       = 4;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_S1SEL       = 2;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_DSEL        = 2;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_OSEL        = 18;
   } else {
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_CSEL        = 2;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_S0SEL       = 4;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_SYSSEL      = 4;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_S1SEL       = 2;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_DSEL        = 2;
      # pADI_CGU_Param_List.cgu1_settings.clocksettings.div_OSEL        = 16;
   }

   # Update the new Divider values with UPDT bit
   # pDevice->pCguRegs->CGU_DIV =  dNewCguDiv | BITM_CGU_DIV_UPDT;
   if { $_CHIPNAME == "adspsc598" } {
      mww phys $cgu1_div 0x44824890
   } else {
      mww phys $cgu1_div 0x44024482
   }

   # Wait until Update bit is set
   # while(!(pDevice->pCguRegs->CGU_DIV & BITM_CGU_DIV_UPDT)) {}
   set data [memread32_phys $cgu1_div]
   while { ![expr {$data & 0x40000000}] } {
      set data [memread32_phys $cgu1_div]
   }

   # wait till clocks are aligned
   # while(pDevice->pCguRegs->CGU_STAT & BITM_CGU_STAT_CLKSALGN)
   set data [memread32_phys $cgu1_stat]
   while { [expr {$data & 0x8}] } {
      set data [memread32_phys $cgu1_stat]
   }

   # Clear DMC Lane reset by clearing DMC_DDR_LANE0_CTL0.CB_RSTDLL
   # and DMC_DDR_LANE1_CTL0.CB_RSTDLL bits
   # *pREG_DMC0_DDR_LANE0_CTL0 &= ~BITM_DMC_DDR_LANE0_CTL0_CB_RSTDLL;
   # *pREG_DMC0_DDR_LANE1_CTL0 &= ~BITM_DMC_DDR_LANE1_CTL0_CB_RSTDLL;
   pmmw $dmc_ddr_lane0_ctl0 0x0 0x100
   pmmw $dmc_ddr_lane1_ctl0 0x0 0x100

   # Wait for DLL lock - 9000 DCLK cycles
   # 1ms should be enough
   after 1

   # Workaround for DDR calibration is required for SC59x 0.0 silicon (see
   # anomaly 20000117).
   set ddr_workaround 1

   if { $ddr_workaround } {

      # DMC Phy Initialization function with workaround 2
      # Reset trigger
      #  *pREG_DMC0_DDR_CA_CTL = 0x0;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x0;
      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x0;
      #  *pREG_DMC0_DDR_SCRATCH_2 = 0x0;
      mww phys $dmc_ddr_ca_ctl 0
      mww phys $dmc_ddr_root_ctl 0
      mww phys $dmc_ddr_scratch3 0
      mww phys $dmc_ddr_scratch2 0

      # Writing internal registers IN calib pad to zero. Calib mode set to 1 [26], trig M1 S1 write [16],
      # this enables usage of scratch registers instead of ZQCTL registers
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x04010000;
      #  DmcDelay(2500);
      mww phys $dmc_ddr_root_ctl 0x04010000
      after 1

      # TRIGGER FOR M2-S2 WRITE -> slave id 31:26  trig m2,s2 write bit 1->1
      # Slave1 address is 4
      #  *pREG_DMC0_DDR_CA_CTL = 0x10000002 ;
      #  DmcDelay(2500);
      mww phys $dmc_ddr_ca_ctl 0x10000002
      after 1

      # Reset Trigger
      #  *pREG_DMC0_DDR_CA_CTL = 0x0;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x0
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      # Write to slave 1, make the power down bit high
      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x1<<12;
      #  *pREG_DMC0_DDR_SCRATCH_2 = 0x0;
      #  DmcDelay(2500);
      mww $dmc_ddr_scratch3 0x1000
      mww $dmc_ddr_scratch2 0
      after 1

      # Calib mode set to 1 [26], trig M1 S1 write [16]
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x04010000;
      #  DmcDelay(2500);
      mww $dmc_ddr_root_ctl 0x04010000
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x10000002;
      #  DmcDelay(2500);
      mww $dmc_ddr_ca_ctl 0x10000002

      after 1
      #  *pREG_DMC0_DDR_CA_CTL = 0x0;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x0;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      # For slave 0
      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x0;
      #  *pREG_DMC0_DDR_SCRATCH_2 = pConfig->ulDDR_ZQCTL0;
      mww $dmc_ddr_scratch3 0
      mww $dmc_ddr_scratch2 0x785a64

      # Calib mode set to 1 [26], trig M1 S1 write [16]
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x04010000;
      #  DmcDelay(2500);
      mww $dmc_ddr_root_ctl 0x04010000
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x0C000002 ;
      #  DmcDelay(2500);
      mww $dmc_ddr_ca_ctl 0x0C000002
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x0;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x0;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      # Writing to slave 1
      # calstrt is 0, but other programming is done
      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x0; /* make power down LOW again, to kickstart BIAS circuit */
      #  *pREG_DMC0_DDR_SCRATCH_2 = 0x30000000;
      mww $dmc_ddr_scratch3 0x0
      mww $dmc_ddr_scratch2 0x30000000

      # Write to ca_ctl lane, calib mode set to 1 [26], trig M1 S1 write [16]
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x04010000;
      #  DmcDelay(2500);
      mww $dmc_ddr_root_ctl 0x04010000
      after 1

      # Copies data to lane controller slave
      # TRIGGER FOR M2-S2 WRITE -> slave id 31:26  trig m2,s2 write bit 1->1
      # slave1 address is 4
      #  *pREG_DMC0_DDR_CA_CTL = 0x10000002 ;
      #  DmcDelay(2500);
      mww $dmc_ddr_ca_ctl 0x10000002
      after 1

      # Reset Trigger
      #  *pREG_DMC0_DDR_CA_CTL = 0x0;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x0;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x0ul;
      #  *pREG_DMC0_DDR_SCRATCH_2 = 0x0ul;
      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x0ul;
      #  *pREG_DMC0_DDR_SCRATCH_2 = 0x0ul;
      mww $dmc_ddr_scratch3 0x0
      mww $dmc_ddr_scratch2 0x0
      mww $dmc_ddr_scratch3 0x0
      mww $dmc_ddr_scratch2 0x0

      #  *pREG_DMC0_DDR_ROOT_CTL = 0x04010000ul;
      #  dmcdelay(2500u);
      mww $dmc_ddr_root_ctl 0x04010000
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x10000002ul;
      #  dmcdelay(2500u);
      mww $dmc_ddr_ca_ctl 0x10000002
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x0ul;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x0ul;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x0ul;
      #  *pREG_DMC0_DDR_SCRATCH_2 = 0x0ul;
      #  *pREG_DMC0_DDR_SCRATCH_3 = 0x0ul;
      #  *pREG_DMC0_DDR_SCRATCH_2 = 0x50000000ul;
      mww $dmc_ddr_scratch3 0x0
      mww $dmc_ddr_scratch2 0x0
      mww $dmc_ddr_scratch3 0x0
      mww $dmc_ddr_scratch2 0x50000000

      #  *pREG_DMC0_DDR_ROOT_CTL = 0x04010000ul;
      #  dmcdelay(2500u);
      mww $dmc_ddr_root_ctl 0x04010000
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x10000002ul;
      #  dmcdelay(2500u);
      mww $dmc_ddr_ca_ctl 0x10000002
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0u;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0u;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      #  *pREG_DMC0_DDR_CA_CTL = 0x0C000004u;
      #  dmcdelay(2500u);
      mww $dmc_ddr_ca_ctl 0x0C000004
      after 1
      #  *pREG_DMC0_DDR_ROOT_CTL = BITM_DMC_DDR_ROOT_CTL_TRIG_RD_XFER_ALL;
      #  dmcdelay(2500u);
      mww $dmc_ddr_root_ctl 0x00200000
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0u;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0u;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      # Calculate ODT PU and PD values
      #  stat_value = ((*pREG_DMC0_DDR_SCRATCH_7 & 0x0000FFFFu)<<16);
      #  stat_value |= (*pREG_DMC0_DDR_SCRATCH_6 & 0xFFFF0000u)>>16;
      #  ClkDqsDrvImpedance = ((pConfig->ulDDR_ZQCTL0) & BITM_DMC_DDR_ZQ_CTL0_IMPWRDQ) >> BITP_DMC_DDR_ZQ_CTL0_IMPWRDQ;
      #  ROdt = ((pConfig->ulDDR_ZQCTL0) & BITM_DMC_DDR_ZQ_CTL0_IMPRTT) >> BITP_DMC_DDR_ZQ_CTL0_IMPRTT;
      #  drv_pu = stat_value & 0x0000003Fu;
      #  drv_pd = (stat_value>>12) & 0x0000003Fu;
      #  odt_pu = (drv_pu * ClkDqsDrvImpedance)/ ROdt;
      #  odt_pd = (drv_pd * ClkDqsDrvImpedance)/ ROdt;
      #  *pREG_DMC0_DDR_SCRATCH_2 |= ((1uL<<24)                     |
      #                              ((drv_pd & 0x0000003Fu))       |
      #                              ((odt_pd & 0x0000003Fu)<<6)    |
      #                              ((drv_pu & 0x0000003Fu)<<12)   |
      #                              ((odt_pu & 0x0000003Fu)<<18));
      set scratch7 [memread32_phys $dmc_ddr_scratch7]
      set scratch6 [memread32_phys $dmc_ddr_scratch6]
      set stat_value [expr {(($scratch7 & 0x0000ffff) << 16) | (($scratch6 & 0xffff0000) >> 16)}]
      set ClkDqsDrvImpedance 0x0000005A
      set ROdt 0x00000078
      set drv_pu [expr {$stat_value & 0x3f}]
      set drv_pd [expr {($stat_value >> 12) & 0x3f}]
      set odt_pu [expr {($drv_pu * $ClkDqsDrvImpedance) / $ROdt}]
      set odt_pd [expr {($drv_pd * $ClkDqsDrvImpedance) / $ROdt}]
      set scratch2 [expr {0x01000000 | $drv_pd | ($odt_pd << 6) | ($drv_pu << 12) | ($odt_pu << 18)}]
      pmmw $dmc_ddr_scratch2 $scratch2 0x0

      #  *pREG_DMC0_DDR_ROOT_CTL = 0x0C010000u;
      #  dmcdelay(2500u);
      mww $dmc_ddr_root_ctl 0x0C010000
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x08000002u;
      #  dmcdelay(2500u);
      mww $dmc_ddr_ca_ctl 0x08000002
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0u;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0u;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

      #  *pREG_DMC0_DDR_ROOT_CTL = 0x04010000u;
      #  dmcdelay(2500u);
      mww $dmc_ddr_root_ctl 0x04010000
      after 1

      #  *pREG_DMC0_DDR_CA_CTL = 0x80000002u;
      #  dmcdelay(2500u);
      mww $dmc_ddr_ca_ctl 0x08000002

      after 1
      #  *pREG_DMC0_DDR_CA_CTL = 0u;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0u;
      mww $dmc_ddr_ca_ctl 0
      mww $dmc_ddr_root_ctl 0

   } else {

      # adi_dmc_phy_calibration()
      # Begin DMC phy ZQ calibration routine
      # Program the ODT and drive strength values
      if { $_CHIPNAME == "adspsc598" } {
         # *pREG_DMC0_DDR_ZQ_CTL0 = 0x00786464;
         # *pREG_DMC0_DDR_ZQ_CTL1 = 0;
         # *pREG_DMC0_DDR_ZQ_CTL2 = 0x70000000;
         mww phys $dmc_ddr_zq_ctl0 0x786464
      } else {
         #  *pREG_DMC0_DDR_ZQ_CTL0 = 0x00785A64;
         #  *pREG_DMC0_DDR_ZQ_CTL1 = 0;
         #  *pREG_DMC0_DDR_ZQ_CTL2 = 0x70000000;
         mww phys $dmc_ddr_zq_ctl0 0x785a64
      }
      mww phys $dmc_ddr_zq_ctl1 0
      mww phys $dmc_ddr_zq_ctl2 0x70000000

      # Generate the trigger
      #  *pREG_DMC0_DDR_CA_CTL = 0x00000000ul ;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x00000000ul;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x00010000ul;
      #  dmcdelay(8000);
      #
      # The [31:26] bits may change if pad ring changes
      #  *pREG_DMC0_DDR_CA_CTL = 0x0C000001ul|TrigCalib;
      #  dmcdelay(8000);
      #  *pREG_DMC0_DDR_CA_CTL = 0x00000000ul ;
      #  *pREG_DMC0_DDR_ROOT_CTL = 0x00000000ul ;

      mww phys $dmc_ddr_ca_ctl 0
      mww phys $dmc_ddr_root_ctl 0
      mww phys $dmc_ddr_root_ctl 0x10000
      after 1

      mww phys $dmc_ddr_ca_ctl 0x0c000001
      after 1
      mww phys $dmc_ddr_ca_ctl 0
      mww phys $dmc_ddr_root_ctl 0

   }

   if { $_CHIPNAME == "adspsc598" } {
      # 800 MHz
      set ulDDR_DLLCTLCFG 0x0cf00622
      set ulDDR_EMR2EMR3  0x00180004
      set ulDDR_CTL       0x08004a05
      set ulDDR_MREMR1    0x0d7000c0
      set ulDDR_TR0       0x4271cb6b
      set ulDDR_TR1       0x60d01860
      set ulDDR_TR2       0x0045c620
      set ulDDR_ZQCTL0    0x00785a64
   } else {
      # 800 MHz
      set ulDDR_DLLCTLCFG 0x0cf00722
      set ulDDR_EMR2EMR3  0x00180004
      set ulDDR_CTL       0x08000a05
      set ulDDR_MREMR1    0x0d7000c0
      set ulDDR_TR0       0x4271cb6b
      set ulDDR_TR1       0x61181860
      set ulDDR_TR2       0x0045c620
      set ulDDR_ZQCTL0    0x00785a64
   }
   set ulDDR_ZQCTL1    0x00000000
   set ulDDR_ZQCTL2    0x70000000

   # program timing registers
   # *pREG_DMC0_CFG = (pConfig->ulDDR_DLLCTLCFG) & 0xFFFFul;
   # *pREG_DMC0_TR0 = pConfig->ulDDR_TR0;
   # *pREG_DMC0_TR1 = pConfig->ulDDR_TR1;
   # *pREG_DMC0_TR2 = pConfig->ulDDR_TR2;
   mww phys $dmc_cfg [expr {$ulDDR_DLLCTLCFG & 0xffff} ]
   mww phys $dmc_tr0 $ulDDR_TR0
   mww phys $dmc_tr1 $ulDDR_TR1
   mww phys $dmc_tr2 $ulDDR_TR2

   # program shadow registers
   # *pREG_DMC0_MR   = ((pConfig->ulDDR_MREMR1) >> 16ul) & 0xFFFFul;
   # *pREG_DMC0_MR1  = (pConfig->ulDDR_MREMR1) & 0xFFFFul;
   # *pREG_DMC0_MR2  = (pConfig->ulDDR_EMR2EMR3)>>16ul & 0xFFFFul;
   # *pREG_DMC0_EMR3 = (pConfig->ulDDR_EMR2EMR3) & 0xFFFFul;
   mww phys $dmc_mr [expr {($ulDDR_MREMR1 >> 16) & 0xffff} ]
   mww phys $dmc_mr1 [expr {$ulDDR_MREMR1 & 0xffff} ]
   mww phys $dmc_mr2 [expr {($ulDDR_EMR2EMR3 >> 16) & 0xffff} ]
   mww phys $dmc_emr3 [expr {$ulDDR_EMR2EMR3 & 0xffff} ]

   # program Dll timing register
   # *pREG_DMC0_DLLCTL = ((pConfig->ulDDR_DLLCTLCFG) >> 16ul) & 0xFFFFul;
   mww phys $dmc_dllctl  [expr {($ulDDR_DLLCTLCFG >> 16) & 0xffff} ]

   # dmcdelay(2000u);
   after 1

   # *pREG_DMC0_DDR_CA_CTL |= BITM_DMC_DDR_CA_CTL_SW_REFRESH;
   pmmw $dmc_ddr_ca_ctl 0x4000 0

   # dmcdelay(5u);
   after 1

   # *pREG_DMC0_DDR_ROOT_CTL |= BITM_DMC_DDR_ROOT_CTL_SW_REFRESH | (OfstdCycle << BITP_DMC_DDR_ROOT_CTL_PIPE_OFSTDCYCLE);
   pmmw $dmc_ddr_root_ctl 0x2800 0

   # Start DMC initialization
   # *pREG_DMC0_CTL = pConfig->ulDDR_CTL;
   mww phys $dmc_ctl $ulDDR_CTL

   # dmcdelay(722000u);
   after 1

   # Add necessary delay depending on the configuration
   # while(((*pREG_DMC0_MR1 & BITM_DMC_MR1_WL)>>BITP_DMC_MR1_WL) != 0ul) { }
   set data [memread32_phys $dmc_mr1]
   while { [expr {$data & 0x80}] } {
      set data [memread32_phys $dmc_mr1]
   }

   # while(((*pREG_DMC0_EMR3 & BITM_DMC_EMR3_MPR)>>BITP_DMC_EMR3_MPR) != 0ul) { }
   set data [memread32_phys $dmc_emr3]
   while { [expr {$data & 0x4}] } {
      set data [memread32_phys $dmc_emr3]
   }

   # check if DMC initialization finished, if not return error
   # while((*pREG_DMC0_STAT & BITM_DMC_STAT_INITDONE) != BITM_DMC_STAT_INITDONE) {
   # }
   set data [memread32_phys $dmc_stat]
   while { ![expr {$data & 0x4}] } {
      set data [memread32_phys $dmc_stat]
   }

   # adi_dmc_crtl_init()

   # toggle DCYCLE
   # *pREG_DMC0_DDR_LANE0_CTL1 |= BITM_DMC_DDR_LANE0_CTL1_COMP_DCYCLE;
   # *pREG_DMC0_DDR_LANE1_CTL1 |= BITM_DMC_DDR_LANE1_CTL1_COMP_DCYCLE;
   # dmcdelay(10u);
   pmmw $dmc_ddr_lane0_ctl1 0x2 0
   pmmw $dmc_ddr_lane1_ctl1 0x2 0

   # *pREG_DMC0_DDR_LANE0_CTL1 &= (~BITM_DMC_DDR_LANE0_CTL1_COMP_DCYCLE);
   # *pREG_DMC0_DDR_LANE1_CTL1 &= (~BITM_DMC_DDR_LANE1_CTL1_COMP_DCYCLE);
   pmmw $dmc_ddr_lane0_ctl1 0 0x2
   pmmw $dmc_ddr_lane1_ctl1 0 0x2

   # toggle RSTDAT
   # *pREG_DMC0_DDR_LANE0_CTL0 |= BITM_DMC_DDR_LANE0_CTL0_CB_RSTDAT;
   # *pREG_DMC0_DDR_LANE0_CTL0 &= (~BITM_DMC_DDR_LANE0_CTL0_CB_RSTDAT);
   pmmw $dmc_ddr_lane0_ctl0 0x08000000 0
   pmmw $dmc_ddr_lane0_ctl0 0 0x08000000

   # *pREG_DMC0_DDR_LANE1_CTL0 |= BITM_DMC_DDR_LANE1_CTL0_CB_RSTDAT;
   # *pREG_DMC0_DDR_LANE1_CTL0 &= (~BITM_DMC_DDR_LANE1_CTL0_CB_RSTDAT);
   pmmw $dmc_ddr_lane1_ctl0 0x08000000 0
   pmmw $dmc_ddr_lane1_ctl0 0 0x08000000

   # dmcdelay(2500u);
   wait 1

   # Program phyphase

   # phyphase = (*pREG_DMC0_STAT & BITM_DMC_STAT_PHYRDPHASE)>>BITP_DMC_STAT_PHYRDPHASE;
   # data_cyc = (phyphase << BITP_DMC_DLLCTL_DATACYC) & BITM_DMC_DLLCTL_DATACYC;
   # rd_cnt = ((pConfig->ulDDR_DLLCTLCFG) >> 16);
   # rd_cnt <<= BITP_DMC_DLLCTL_DLLCALRDCNT;
   # rd_cnt &= BITM_DMC_DLLCTL_DLLCALRDCNT;
   # *pREG_DMC0_DLLCTL = rd_cnt|data_cyc;
   # *pREG_DMC0_CTL = (pConfig->ulDDR_CTL & (~BITM_DMC_CTL_INIT) & (~BITM_DMC_CTL_RL_DQS));
   set data_stat [memread32_phys $dmc_stat]
   set phyphase [expr {$data_stat & 0x00f00000}]
   set phyphase [expr {$phyphase >> 20}]
   set datacyc [expr {$phyphase << 8}]
   set datacyc [expr {$datacyc & 0x00000f00}]

   set rd_cnt [expr {$ulDDR_DLLCTLCFG >> 16} ]
   set rd_cnt [expr {$rd_cnt & 0xff} ]


   mww phys $dmc_dllctl [expr {$rd_cnt | $datacyc}]
   mww phys $dmc_ctl [expr {$ulDDR_CTL & 0xfffffffb & 0xfbffffff} ]

   if { $_CHIPNAME == "adspsc598" } {
      # DDR workaround.
      # Restricts the outstanding transactions to 1 for both read/write
      # to force in-order access/response from A55 to/from DDR.
      set scb6_a55_m0_ib_fn_mod 0x30643108
      mww phys $scb6_a55_m0_ib_fn_mod 0x00000003
   }


   # Delay trim required for 800MHz and 900MHz DDR3
   set DelayTrim 1

   if { $DelayTrim } {
      # DQS delay trim

      # For LDQS
      # *pREG_DMC0_DDR_LANE0_CTL1 |= 0x000000D0;
      # dmcdelay(2500u);
      pmmw $dmc_ddr_lane0_ctl1 0x000000d0 0
      after 1

      # *pREG_DMC0_DDR_ROOT_CTL=0x00400000;
      # dmcdelay(2500u);
      mww phys $dmc_ddr_root_ctl 0x00400000
      after 1

      # *pREG_DMC0_DDR_ROOT_CTL =0x0;
      # WL_code_LDQS = (*pREG_DMC0_DDR_SCRATCH_4 & 0x001F0000u) >> 16;
      mww phys $dmc_ddr_root_ctl 0x0
      set data [memread32_phys $dmc_ddr_scratch4]
      set wl_code_ldqs [expr {($data & 0x001f0000) >> 16}]

      # *pREG_DMC0_DDR_LANE0_CTL1 &= ~(BITM_DMC_DDR_LANE0_CTL1_BYPCODE|BITM_DMC_DDR_LANE0_CTL1_BYPDELCHAINEN);
      # *pREG_DMC0_DDR_LANE0_CTL1 |= (((WL_code_LDQS + Lane0_DQ_Delay)<<BITP_DMC_DDR_LANE0_CTL1_BYPCODE) & BITM_DMC_DDR_LANE0_CTL1_BYPCODE)|BITM_DMC_DDR_LANE0_CTL1_BYPDELCHAINEN;
      # dmcdelay(2500u);
      pmmw $dmc_ddr_lane0_ctl1 0 0x0000fc00
      set lane0_dq_delay 1
      set ctl_val [expr {((($wl_code_ldqs + $lane0_dq_delay) << 10) & 0x00007c00) | 0x00008000}]
      pmmw $dmc_ddr_lane0_ctl1 $ctl_val 0
      after 1

      # For UDQS
      # *pREG_DMC0_DDR_LANE1_CTL1 |= 0x000000D0u;
      # dmcdelay(2500u);
      pmmw $dmc_ddr_lane1_ctl1 0x000000d0 0
      after 1

      # *pREG_DMC0_DDR_ROOT_CTL=0x00800000;
      # dmcdelay(2500u);
      mww phys $dmc_ddr_root_ctl 0x00800000
      after 1

      # *pREG_DMC0_DDR_ROOT_CTL =0x0;
      # WL_code_UDQS = (*pREG_DMC0_DDR_SCRATCH_5 & 0x001F0000u) >> 16;
      # *pREG_DMC0_DDR_LANE1_CTL1 &= ~(BITM_DMC_DDR_LANE1_CTL1_BYPCODE|BITM_DMC_DDR_LANE1_CTL1_BYPDELCHAINEN);
      # *pREG_DMC0_DDR_LANE1_CTL1 |= (((WL_code_UDQS + Lane1_DQ_Delay)<<BITP_DMC_DDR_LANE1_CTL1_BYPCODE) & BITM_DMC_DDR_LANE1_CTL1_BYPCODE)|BITM_DMC_DDR_LANE1_CTL1_BYPDELCHAINEN;
      mww phys $dmc_ddr_root_ctl 0x0
      set data [memread32_phys $dmc_ddr_scratch5]
      set wl_code_udqs [expr {($data & 0x001f0000) >> 16}]
      pmmw $dmc_ddr_lane1_ctl1 0 0x0000fc00

      set lane1_dq_delay 1
      set data [expr {((($wl_code_udqs + $lane1_dq_delay) << 10) & 0x00007c00) | 0x00008000}]
      pmmw $dmc_ddr_lane1_ctl1 $data 0
      after 1
   }

   if { $_CHIPNAME == "adspsc598" } {
      # Invalidate the DDR prefetch buffer after DMC initialization. The
      # invalidation bit must be cleared manually.
      # *pREG_DDRPFB0_CTL0 |= BITM_DDRPFB_CTL0_DATA_INVALIDATION;
      # *pREG_DDRPFB0_CTL0 &= ~BITM_DDRPFB_CTL0_DATA_INVALIDATION;
      pmmw 0x31076000 0x00000010 0
      pmmw 0x31076000 0 0x00000010
   }

}

proc adspsc59x_init_emac { } {
   set pads0_pcfg0 0x31004604

   # Take EMAC0 (RGMII) out of reset
   # *pREG_PADS0_PCFG0 |= BITM_PADS_PCFG0_EMACRESET
   pmmw $pads0_pcfg0 0x00000004 0
}

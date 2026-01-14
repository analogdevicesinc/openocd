# SPDX-License-Identifier: GPL-2.0-or-later

set CURRENT_DIR [file dirname [info script]]
source [file join $CURRENT_DIR "elf_parser.tcl"]
source [file join $CURRENT_DIR "auto_boot.tcl"]

if {![info exists BOOT_SEQ]} {
    echo "========================================================================"
    echo " ERROR: No boot sequence provided!"
    echo "========================================================================"
    echo " You are running generic_boot.tcl but did not provide the file list."
    echo " Please pass the 'BOOT_SEQ' variable using the -c flag."
    echo ""
    echo " Example:"
    echo "   openocd ... -c \"set BOOT_SEQ { {<path/to/firmware> <breakpoint>} {<path/to/firmware> <<breakpoint>>} }\" -f boot.tcl"
    echo "========================================================================"
    shutdown
}

set AUTOBOOT_FILES $BOOT_SEQ

global _TARGETNAME
$_TARGETNAME configure -event reset-init {
    # 150ms delay to ensure stability after reset
    after 150 { _autoboot_logic }
}

init
reset init
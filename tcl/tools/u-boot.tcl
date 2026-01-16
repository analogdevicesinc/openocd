# SPDX-License-Identifier: GPL-2.0-or-later

set CURRENT_DIR [file dirname [info script]]
source [file join $CURRENT_DIR "elf_parser.tcl"]
source [file join $CURRENT_DIR "auto_boot.tcl"]

set AUTOBOOT_FILES {
    {"u-boot-spl" "board_init_r"}
    {"u-boot"     ""}
}

global _TARGETNAME
$_TARGETNAME configure -event reset-init {
    after 150 { autoboot_elf }
}

init
reset init


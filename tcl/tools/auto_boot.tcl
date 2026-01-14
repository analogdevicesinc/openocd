# SPDX-License-Identifier: GPL-2.0-or-later

proc _autoboot_logic {} {
    global AUTOBOOT_FILES

    halt
    poll on

    foreach entry $AUTOBOOT_FILES {
        lassign $entry filename symbol

        set elf "[pwd]/$filename"
        if {![file exists $elf]} {
            echo "ERROR: File not found: $elf"
            shutdown
        }

        echo "Processing $filename..."

        set elf_info [parse_elf $elf $symbol]
        set entry_point [lindex $elf_info 0]
        set symbol_addr [lindex $elf_info 1]

        set entry_hex [format "0x%x" $entry_point]
        
        echo "Loading image..."
        load_image $elf

        echo "Setting PC to Entry Point: $entry_hex"
        reg pc $entry_point

        set bp_set 0
        if {$symbol ne "" && $symbol_addr != 0} {
            set sym_hex [format "0x%x" $symbol_addr]
            echo "Setting HW Breakpoint on '$symbol' at $sym_hex"
            bp $sym_hex 4 hw
            set bp_set 1
        }

        resume

        if {$bp_set} {
            echo "Waiting for breakpoint ($symbol)..."
            wait_halt 10000
            echo "Breakpoint hit at $symbol"
            rbp $sym_hex
        } else {
            echo "$filename started running."
        }
    }
    echo "All boot stages executed successfully."
}
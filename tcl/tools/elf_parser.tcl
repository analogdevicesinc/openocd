# SPDX-License-Identifier: GPL-2.0-or-later

# Capable of
#   -Automatically detects and parses both ELF32 and ELF64.'.
#   -Can resolve specific symbol names to their virtual 
#   addresses for hardware breakpoints.
#
# Limitations
#   -The logic contains provisions for endianness, but is 
#   primarily verified for Little Endian. Big Endian targets may
#   require verification.
#   -It only scans the static symbol table (.symtab). 
#   -Symbol lookup is performed via linear scan. This is optimal 
#    for bootloaders but may be slow for very large applications.

proc parse_elf {filename symbol_name} {
    if {![file exists $filename]} {
        echo "ERROR: ELF file $filename not found."
        return [list 0 0]
    }

    set fp [open $filename rb]
    
    set magic [read $fp 4]
    if {$magic ne "\x7fELF"} { close $fp; return [list 0 0] }

    binary scan [read $fp 1] c class
    binary scan [read $fp 1] c endian
    
    if {$class == 2} { # 64-bit arch
        set is_64 1
        set entry_offset 0x18
        set shoff_offset 0x28
        set shnum_offset 0x3C
        set shstrndx_offset 0x3E
        set sym_entry_size 24
        set addr_scan_fmt "w"
    } else { # 32-bit arch
        set is_64 0
        set entry_offset 0x18
        set shoff_offset 0x20
        set shnum_offset 0x30
        set shstrndx_offset 0x32
        set sym_entry_size 16
        set addr_scan_fmt "i"
    }

    seek $fp $entry_offset start
    binary scan [read $fp [expr {$is_64 ? 8 : 4}]] $addr_scan_fmt entry_point
    
    if {$entry_point < 0} { set entry_point [expr {$entry_point & 0xFFFFFFFF}] }

    if {$symbol_name eq ""} { close $fp; return [list $entry_point 0] }

    seek $fp $shoff_offset start
    binary scan [read $fp [expr {$is_64 ? 8 : 4}]] $addr_scan_fmt shoff
    
    seek $fp $shnum_offset start
    binary scan [read $fp 2] s shnum
    
    seek $fp $shstrndx_offset start
    binary scan [read $fp 2] s shstrndx

    set sh_entry_size [expr {$is_64 ? 64 : 40}]
    
    set sec_offset_loc [expr {$is_64 ? 0x18 : 0x10}]

    set shstrtab_hdr_offset [expr {$shoff + ($shstrndx * $sh_entry_size)}]
    seek $fp [expr {$shstrtab_hdr_offset + $sec_offset_loc}] start
    binary scan [read $fp [expr {$is_64 ? 8 : 4}]] $addr_scan_fmt shstrtab_offset

    set symtab_offset 0; set symtab_size 0; set strtab_offset 0
    
    for {set i 0} {$i < $shnum} {incr i} {
        set current_sh_offset [expr {$shoff + ($i * $sh_entry_size)}]
        
        seek $fp $current_sh_offset start
        binary scan [read $fp 4] i name_idx

        seek $fp [expr {$shstrtab_offset + $name_idx}] start
        set s_name ""
        while {1} {
            binary scan [read $fp 1] c char
            if {$char == 0} break
            append s_name [format %c $char]
        }

        if {$s_name eq ".symtab"} {
            seek $fp [expr {$current_sh_offset + $sec_offset_loc}] start
            binary scan [read $fp [expr {$is_64 ? 8 : 4}]] $addr_scan_fmt symtab_offset
            # Size hemen arkasinda
            binary scan [read $fp [expr {$is_64 ? 8 : 4}]] $addr_scan_fmt symtab_size
        }
        if {$s_name eq ".strtab"} {
            seek $fp [expr {$current_sh_offset + $sec_offset_loc}] start
            binary scan [read $fp [expr {$is_64 ? 8 : 4}]] $addr_scan_fmt strtab_offset
        }
    }

    if {$symtab_offset == 0} { close $fp; return [list $entry_point 0] }

    set num_symbols [expr {$symtab_size / $sym_entry_size}]
    set found_addr 0
    
    set val_offset [expr {$is_64 ? 8 : 4}]

    for {set i 0} {$i < $num_symbols} {incr i} {
        set sym_offset [expr {$symtab_offset + ($i * $sym_entry_size)}]
        
        seek $fp $sym_offset start
        binary scan [read $fp 4] i st_name_idx

        seek $fp [expr {$strtab_offset + $st_name_idx}] start
        set current_sym_name ""
        while {1} {
            binary scan [read $fp 1] c char
            if {$char == 0} break
            append current_sym_name [format %c $char]
        }

        if {$current_sym_name eq $symbol_name} {
            seek $fp [expr {$sym_offset + $val_offset}] start
            binary scan [read $fp [expr {$is_64 ? 8 : 4}]] $addr_scan_fmt found_addr
            break
        }
    }

    close $fp
    return [list $entry_point $found_addr]
}
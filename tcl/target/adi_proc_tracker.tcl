# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2025 Analog Devices, Inc.
#
# Proc (processor) tracker file
# The purpose of this file is provide routines to track the processors and corresponding global variables in the JTAG
# scan chain during a single debug session. For example: two devices in a scan chain will be tracked as _CHIPNAME_Proc0 and
# _CHIPNAME_Proc1 with the number indicating their place in the scan chain (Proc0 is closest to TDO)
#
# Updated to also track core information for each processor in the scan chain. Global variables will be indicated with
# both processor and core number. For example USE_CTI_Proc0_Core0 for Processor 0 Core 0.

# Procedure to set up the chip name based on the proc number in the scan chain if applicable
proc setup_chip_name { chip_name } {
	# Check if PROCNUM exists, if not, initialize it to 0
	if { ![info exists ::PROCNUM] } {
		set ::PROCNUM 0
	} else {
		set ::PROCNUM [expr {$::PROCNUM + 1}]
	}

	# Check if chip_name is valid (non-empty)
	if { $chip_name ne "" } {
		return "${chip_name}_Proc$::PROCNUM"
	} else {
		return $chip_name
	}
}

# Procedure to track core information for each processor
proc update_core_info { target_name core_num_var } {
	upvar $core_num_var core_num
	if {![info exists ::CORE_INFO] || [llength $::CORE_INFO] == 0} {
		set ::CORE_INFO [list $target_name $core_num]
	} else {
		lappend ::CORE_INFO $target_name $core_num
	}
	set core_num [expr {$core_num + 1}]
}

# Retrieve core number given the target
proc get_core_number { target_name } {
	foreach {name num} $::CORE_INFO {
		if {$name eq $target_name} {
			return $num
		}
	}
	return ""
}

# Procedure to iterate over keys and their default values
# This will set up the global processor variables uniquely based on location in scan chain
# For example _USERKEY0_Proc0 for proc 0 in scan chain, _USERKEY0_Proc1, etc...
proc process_default_proc_settings { default_settings } {
	foreach {key default_value} $default_settings {
		set temp_var "${key}_Proc$::PROCNUM"
		set dynamic_var "_$temp_var"
		if {[info exists ::$temp_var]} {
			set ::$dynamic_var [set ::$temp_var]
		} else {
			set ::$dynamic_var $default_value
		}
	}
}

# Procedure to iterate over keys and their default values
# This will set up the global core variables uniquely based on location in scan chain and core number
# For example _USE_CTI_Proc0_Core0 for proc 0 core 0 in scan chain, _USE_CTI_Proc0_Core1 for proc 0 core 1,
# _USE_CTI_Proc1_Core0 for proc 1 core 0, etc...
proc process_default_core_settings { default_settings num_cores } {
	for {set i 0} {$i < $num_cores + 1} {incr i} {
		foreach {key default_value} $default_settings {
			set temp_var "${key}_Proc${::PROCNUM}_Core${i}"
			set dynamic_var "_$temp_var"
			if {[info exists ::$temp_var]} {
				set ::$dynamic_var [set ::$temp_var]
			} else {
				set ::$dynamic_var $default_value
			}
		}
	}
}

# Procedure to get current running target and return the base name and proc number
# Example usage:
#
# global _CHIPNAME
# set result [get_current_chipname $_CHIPNAME]
proc get_current_chipname { chip_name } {
	set current_target [target current]

	# Find the position of the first period
	set dot_pos [string first "." $current_target]
	# Before the period will be the chip name
	set base_name [string range $current_target 0 [expr {$dot_pos - 1}]]

	# Extract the device number from the extracted base name
	# The number following the last underscore should indicate the proc number
	set underscore_pos [string last "_" $base_name]
	if { $underscore_pos != -1 } {
		set proc_str [string range $base_name [expr {$underscore_pos + 1}] end]
		# Extract numerical part, assuming format Proc0, Prc1, etc.
		if {[regexp {Proc([0-9]+)} $proc_str -> proc_number]} {
			# proc_number now contains only the digits after Proc
		} else {
			echo "Warning: Could not extract numeric part after P"
			set proc_number ""
		}
	} else {
		echo "Warning: Could not extract proc string after underscore"
		set proc_number ""
	}

	if { $chip_name != $base_name } {
		return [list $base_name $proc_number]
	} else {
		return [list $chip_name $proc_number]
	}
}
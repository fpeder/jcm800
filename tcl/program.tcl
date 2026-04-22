# =====================================================================
# program.tcl — program Arty A7-100T over JTAG
#
#   Usage:  vivado -mode batch -source tcl/program.tcl
#   Requires: build/arty_top/arty_top.runs/impl_1/arty_top.bit
# =====================================================================

set repo_root [file normalize [file join [file dirname [info script]] ..]]
set bit_file  [file join $repo_root build arty_top arty_top.runs impl_1 arty_top.bit]

if {![file exists $bit_file]} {
    error "Bitstream not found: $bit_file (run tcl/synth.tcl first)"
}

open_hw_manager
connect_hw_server -quiet
open_hw_target

# Match the xc7a100t on the Arty A7-100T
set device [lindex [get_hw_devices xc7a100t_0] 0]
current_hw_device $device
refresh_hw_device -update_hw_probes false $device

set_property PROGRAM.FILE $bit_file $device
program_hw_devices $device
refresh_hw_device $device

close_hw_target
disconnect_hw_server
close_hw_manager

puts "Programmed: $bit_file"

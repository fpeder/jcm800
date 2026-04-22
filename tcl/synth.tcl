# =====================================================================
# synth.tcl — synth + impl + bitstream for arty_top
#
#   Usage:  vivado -mode batch -source tcl/synth.tcl
#   Requires: build/arty_top/arty_top.xpr (run tcl/project.tcl first)
#   Output:   build/arty_top/arty_top.runs/impl_1/arty_top.bit
# =====================================================================

set repo_root [file normalize [file join [file dirname [info script]] ..]]
set proj_file [file join $repo_root build arty_top arty_top.xpr]

open_project $proj_file

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1

if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "Synthesis failed"
}

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    error "Implementation failed"
}

set bit [file join [get_property DIRECTORY [get_runs impl_1]] arty_top.bit]
puts "Bitstream: $bit"

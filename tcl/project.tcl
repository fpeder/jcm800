# =====================================================================
# project.tcl — create Vivado project for arty_top (I2S2 loopback)
#
#   Usage:  vivado -mode batch -source tcl/project.tcl
#   Output: build/arty_top/arty_top.xpr
# =====================================================================

set repo_root [file normalize [file join [file dirname [info script]] ..]]
set proj_name arty_top
set proj_dir  [file join $repo_root build $proj_name]
set part      xc7a100tcsg324-1

file mkdir $proj_dir
create_project -force $proj_name $proj_dir -part $part

add_files -fileset sources_1 [glob $repo_root/rtl/*.sv]
add_files -fileset sources_1 [glob $repo_root/rtl/oversample/*.sv]
add_files -fileset constrs_1 [glob $repo_root/constraints/*.xdc]
add_files -fileset sources_1 [glob $repo_root/lut_out/*.mem]

set_property file_type SystemVerilog [get_files -of_objects [get_filesets sources_1] *.sv]
set_property file_type {Memory Initialization Files} \
    [get_files -of_objects [get_filesets sources_1] *.mem]
# Lets $readmemh("v1a_lut.mem", ...) resolve at elab + synth.
set_property include_dirs $repo_root/lut_out [get_filesets sources_1]
set_property top arty_top [get_filesets sources_1]

# ---------------------------------------------------------------
# Clocking Wizard IP: 100 MHz -> 12.288 MHz MCLK
# ---------------------------------------------------------------
create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name clk_wiz_mclk
set_property -dict [list \
    CONFIG.PRIMITIVE                  {MMCM} \
    CONFIG.PRIM_IN_FREQ               {100.000} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {12.288} \
    CONFIG.USE_RESET                  {true} \
    CONFIG.RESET_TYPE                 {ACTIVE_LOW} \
    CONFIG.RESET_PORT                 {resetn} \
    CONFIG.USE_LOCKED                 {true} \
] [get_ips clk_wiz_mclk]

generate_target all [get_ips clk_wiz_mclk]

puts "Project created: $proj_dir/$proj_name.xpr"

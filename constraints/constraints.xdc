# ---------------------------------------------------------------
# JCM800 2203 Amplifier Model -- Digilent Arty A7-100T
# Part: xc7a100tcsg324-1
#
# Board-level wrapper: arty_top
# Audio I/O via Digilent Pmod I2S2 on JA header
# ---------------------------------------------------------------

# === Downgrade unconstrained-port DRCs to warnings ===
set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]

# ---------------------------------------------------------------
# Clock: 100 MHz oscillator (Y1, pin E3)
# ---------------------------------------------------------------
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.000 -name sys_clk [get_ports clk]

# ---------------------------------------------------------------
# Reset: active-LOW dedicated RESET button (C2)
# ---------------------------------------------------------------
set_property -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS33} [get_ports rst_btn]
set_false_path -from [get_ports rst_btn]

# ---------------------------------------------------------------
# Pmod JA — Digilent Pmod I2S2 (CS5343 ADC + CS4344 DAC)
#   Ref: github.com/Digilent/Arty-A7-35-Pmod-I2S2
#
# Top row (TX / DAC):
# | Signal       | Pmod Pin | FPGA Pin | Dir |
# |--------------|----------|----------|-----|
# | TX MCLK      | 1        | G13      | Out |
# | TX LRCK      | 2        | B11      | Out |
# | TX SCLK      | 3        | A11      | Out |
# | TX DATA      | 4        | D12      | Out |
#
# Bottom row (RX / ADC):
# | RX MCLK      | 7        | D13      | Out |
# | RX LRCK      | 8        | B18      | Out |
# | RX SCLK      | 9        | A18      | Out |
# | RX DATA      | 10       | K16      | In  |
# ---------------------------------------------------------------
set_property -dict {PACKAGE_PIN G13 IOSTANDARD LVCMOS33} [get_ports ja_mclk]
set_property -dict {PACKAGE_PIN B11 IOSTANDARD LVCMOS33} [get_ports ja_lrck]
set_property -dict {PACKAGE_PIN A11 IOSTANDARD LVCMOS33} [get_ports ja_dac_sclk]
set_property -dict {PACKAGE_PIN D12 IOSTANDARD LVCMOS33} [get_ports ja_dac_sdin]
set_property -dict {PACKAGE_PIN D13 IOSTANDARD LVCMOS33} [get_ports ja_adc_mclk]
set_property -dict {PACKAGE_PIN B18 IOSTANDARD LVCMOS33} [get_ports ja_adc_lrck]
set_property -dict {PACKAGE_PIN A18 IOSTANDARD LVCMOS33} [get_ports ja_adc_sclk]
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports ja_adc_sdout]

# ---------------------------------------------------------------
# Status LEDs
#   LD0 (H5): MMCM locked
#   LD1 (J5): LRCK blink
# ---------------------------------------------------------------
set_property -dict {PACKAGE_PIN H5 IOSTANDARD LVCMOS33} [get_ports led_locked]
set_property -dict {PACKAGE_PIN J5 IOSTANDARD LVCMOS33} [get_ports led_lrck]
set_property -dict {PACKAGE_PIN T9 IOSTANDARD LVCMOS33} [get_ports led_out_valid]
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports led_nonzero]
set_false_path -to [get_ports {led_locked led_lrck led_out_valid led_nonzero}]

# ---------------------------------------------------------------
# UART RX from USB-UART bridge (FT2232H interface 1)
#   Arty A7 schematic: UART_TXD_IN = A9 (FPGA input)
# ---------------------------------------------------------------
set_property -dict {PACKAGE_PIN A9 IOSTANDARD LVCMOS33} [get_ports uart_rx]
set_false_path -from [get_ports uart_rx]

# ---------------------------------------------------------------
# CDC false paths between sys_clk and MCLK domains
# The clk_wiz IP auto-creates clock "clk_out1_clk_wiz_mclk"
# ---------------------------------------------------------------
set_false_path -from [get_clocks sys_clk]                -to [get_clocks clk_out1_clk_wiz_mclk]
set_false_path -from [get_clocks clk_out1_clk_wiz_mclk]  -to [get_clocks sys_clk]

# ---------------------------------------------------------------
# I/O timing for Pmod I2S pins (all false-pathed)
# ---------------------------------------------------------------
set_false_path -from [get_ports ja_adc_sdout]
set_false_path -to   [get_ports {ja_lrck ja_dac_sdin ja_dac_sclk ja_mclk ja_adc_sclk ja_adc_lrck ja_adc_mclk}]

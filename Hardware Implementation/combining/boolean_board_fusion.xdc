## RealDigital Boolean Board - XC7S50-CSGA324-1
## Top module: underwater_fusion_top
## Verified against the official RealDigital Boolean board constraints file.
##
## Clock: 100 MHz oscillator on F14.
## UART : UART_txd on U11 (FPGA -> FTDI -> PC USB-UART, 115200 8-N-1).
## Buttons:
##   btn[0] (J2) = start (press once to compute & stream; press again to re-stream)
##   btn[1] (J5) = reset
## LEDs:
##   led[0]   = busy
##   led[1]   = done
##   led[2]   = UART streaming active
##   led[5:3] = stream stage (000=WB, 001=Gamma, 010=Unsharp, 011=Fused, 100=Weights)
##   led[8:6] = compute phase indicator (1..3=WB, 4=Unsharp MM, 5=Lab Mean, 6=Weight MM, 7=Stream)

set_property -dict { PACKAGE_PIN F14 IOSTANDARD LVCMOS33 } [get_ports { clk }]
create_clock -name sys_clk -period 10.000 -waveform {0.000 5.000} [get_ports { clk }]

set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports { btn[0] }]
set_property -dict { PACKAGE_PIN J5 IOSTANDARD LVCMOS33 } [get_ports { btn[1] }]

set_property -dict { PACKAGE_PIN G1 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN G2 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN F1 IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN F2 IOSTANDARD LVCMOS33 } [get_ports { led[3] }]
set_property -dict { PACKAGE_PIN E1 IOSTANDARD LVCMOS33 } [get_ports { led[4] }]
set_property -dict { PACKAGE_PIN E2 IOSTANDARD LVCMOS33 } [get_ports { led[5] }]
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { led[6] }]
set_property -dict { PACKAGE_PIN E5 IOSTANDARD LVCMOS33 } [get_ports { led[7] }]
set_property -dict { PACKAGE_PIN E6 IOSTANDARD LVCMOS33 } [get_ports { led[8] }]
set_property -dict { PACKAGE_PIN C3 IOSTANDARD LVCMOS33 } [get_ports { led[9] }]
set_property -dict { PACKAGE_PIN B2 IOSTANDARD LVCMOS33 } [get_ports { led[10] }]
set_property -dict { PACKAGE_PIN A2 IOSTANDARD LVCMOS33 } [get_ports { led[11] }]
set_property -dict { PACKAGE_PIN B3 IOSTANDARD LVCMOS33 } [get_ports { led[12] }]
set_property -dict { PACKAGE_PIN A3 IOSTANDARD LVCMOS33 } [get_ports { led[13] }]
set_property -dict { PACKAGE_PIN B4 IOSTANDARD LVCMOS33 } [get_ports { led[14] }]
set_property -dict { PACKAGE_PIN A4 IOSTANDARD LVCMOS33 } [get_ports { led[15] }]

set_property -dict { PACKAGE_PIN U11 IOSTANDARD LVCMOS33 } [get_ports { UART_txd }]

## Configuration-bank voltage settings required for Artix-7/Spartan-7 bitstream DRC.
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]


## RealDigital Boolean Board - Spartan-7 XC7S50
## Top-level ports: clk_100MHz, rst, uart_tx

## 100 MHz onboard oscillator: FPGA pin F14
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports {clk_100MHz}]
create_clock -name sys_clk -period 10.000 [get_ports {clk_100MHz}]

## Reset: slide switch SW0, FPGA pin V2
## Set the switch ON to assert reset; OFF to run.
set_property -dict {PACKAGE_PIN V2 IOSTANDARD LVCMOS33} [get_ports {rst}]

## UART transmit: onboard USB-UART TX connection, FPGA pin U11
set_property -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports {uart_tx}]

## Board configuration voltage
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

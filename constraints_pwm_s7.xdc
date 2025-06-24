#SYS_CLK External Gen  100MHz Schematic pg. 8/7
set_property PACKAGE_PIN G11 [get_ports clk_100MHz]
set_property IOSTANDARD LVCMOS33 [get_ports clk_100MHz]

set_property PACKAGE_PIN G4 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

#leds 0 thru 3
set_property PACKAGE_PIN E11 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property PACKAGE_PIN M10 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN A10 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN D12 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

#Push Buttons 0 thru 3
set_property PACKAGE_PIN M5 [get_ports {rst}]
set_property IOSTANDARD LVCMOS33 [get_ports {rst}]
#set_property PACKAGE_PIN L5 [get_ports {setpt_latch}]
#set_property IOSTANDARD LVCMOS33 [get_ports {setpt_latch}]
#set_property PACKAGE_PIN N4 [get_ports {btn[2]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {btn[2]}]
#set_property PACKAGE_PIN P5 [get_ports {btn[3]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {btn[3]}]


     
#Switches 0 thru 3
set_property PACKAGE_PIN A12 [get_ports {rc_en}]
set_property IOSTANDARD LVCMOS33 [get_ports {rc_en}]
#set_property PACKAGE_PIN A13 [get_ports {lt}]
#set_property IOSTANDARD LVCMOS33 [get_ports {lt}]
#set_property PACKAGE_PIN B13 [get_ports {rt}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rt}]
#set_property PACKAGE_PIN B14 [get_ports {rc_en}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rc_en}]


#RP2040 Interface     1st 4 Functions:
#RP Port: GPIO0   SPI0 RX    UART0 TX    I2C0 SDA    PWM0 A
set_property PACKAGE_PIN E13 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
#RP Port: GPIO1   SPI0 CSn   UART0 RX    I2C0 SCL    PWM0 B
set_property PACKAGE_PIN C14 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
#RP Port: GPIO4     UART1 TX    
#set_property PACKAGE_PIN F12 [get_ports uart_rx]
#set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
#RP Port: GPIO5     UART1 RX    
#set_property PACKAGE_PIN F14 [get_ports uart_tx]
#set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]

#PMOD0
#Pin 1
set_property PACKAGE_PIN H12 [get_ports {in1_l298n_dir}]          
set_property IOSTANDARD LVCMOS33 [get_ports {in1_l298n_dir}]
#Pin 2     
set_property PACKAGE_PIN H11 [get_ports {in2_l298n_dir}]          
set_property IOSTANDARD LVCMOS33 [get_ports {in2_l298n_dir}]
#Pin 3
set_property PACKAGE_PIN H14 [get_ports {encoder_a_m1}]          
set_property IOSTANDARD LVCMOS33 [get_ports {encoder_a_m1}]
#Pin 4
set_property PACKAGE_PIN H13 [get_ports {encoder_b_m1}]          
set_property IOSTANDARD LVCMOS33 [get_ports {encoder_b_m1}]
#Pin 7
set_property PACKAGE_PIN J14 [get_ports {in3_l298n_dir}]          
set_property IOSTANDARD LVCMOS33 [get_ports {in3_l298n_dir}]
#Pin 8
set_property PACKAGE_PIN J13 [get_ports {in4_l298n_dir}]          
set_property IOSTANDARD LVCMOS33 [get_ports {in4_l298n_dir}]
#Pin 9
set_property PACKAGE_PIN J12 [get_ports {encoder_a_m2}]          
set_property IOSTANDARD LVCMOS33 [get_ports {encoder_a_m2}]
#Pin 10
set_property PACKAGE_PIN J11 [get_ports {encoder_b_m2}]          
set_property IOSTANDARD LVCMOS33 [get_ports {encoder_b_m2}]

#PMOD1
#Pin 1
set_property PACKAGE_PIN L13 [get_ports {pwm_l298n_enA}]           
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_l298n_enA}]
#Pin 2
set_property PACKAGE_PIN L12 [get_ports {pwm_l298n_enB}]          
set_property IOSTANDARD LVCMOS33 [get_ports {pwm_l298n_enB}]
#Pin 3 - Debug
#set_property PACKAGE_PIN L14 [get_ports {o_Tx_Done}]       Port looks bad, stays high all the time.
#set_property IOSTANDARD LVCMOS33 [get_ports {o_Tx_Done}]
#Pin 4
set_property PACKAGE_PIN M13 [get_ports {o_Tx_DV}]          
set_property IOSTANDARD LVCMOS33 [get_ports {o_Tx_DV}]
#Pin 7
set_property PACKAGE_PIN K12 [get_ports {uart_tx}]          
set_property IOSTANDARD LVCMOS33 [get_ports {uart_tx}]
#Pin 8
#set_property PACKAGE_PIN K11 [get_ports {pwm_l298n_enB}]          
#set_property IOSTANDARD LVCMOS33 [get_ports {pwm_l298n_enB}]
#Pin 9
#set_property PACKAGE_PIN M12 [get_ports {encoder_a_m2}]          
#set_property IOSTANDARD LVCMOS33 [get_ports {encoder_a_m2}]
#Pin 10
#set_property PACKAGE_PIN M11 [get_ports {encoder_b_m2}]          
#set_property IOSTANDARD LVCMOS33 [get_ports {encoder_b_m2}]


#PMOD2
set_property PACKAGE_PIN M14 [get_ports {rc_fwd}]          
set_property IOSTANDARD LVCMOS33 [get_ports {rc_fwd}] 
set_property PULLDOWN true [get_ports {rc_fwd}]
#Pin 2     
set_property PACKAGE_PIN N14 [get_ports {rc_rev}]          
set_property IOSTANDARD LVCMOS33 [get_ports {rc_rev}]
set_property PULLDOWN true [get_ports {rc_rev}]
#Pin 3
set_property PACKAGE_PIN P13 [get_ports {rc_lft}]          
set_property IOSTANDARD LVCMOS33 [get_ports {rc_lft}]
set_property PULLDOWN true [get_ports {rc_lft}]
#Pin 4
set_property PACKAGE_PIN P12 [get_ports {rc_rt}]          
set_property IOSTANDARD LVCMOS33 [get_ports {rc_rt}]
set_property PULLDOWN true [get_ports {rc_rt}]

#Pin 7   Schematic labels as PMOD2_4 for FPGA connect
#set_property PACKAGE_PIN N11 [get_ports {rc_lft}]          
#set_property IOSTANDARD LVCMOS33 [get_ports {rc_lft}]
#Pin 8   Schematic labels as PMOD2_5 for FPGA connect
#set_property PACKAGE_PIN N10 [get_ports {rc_rt}]          
#set_property IOSTANDARD LVCMOS33 [get_ports {rc_rt}]
#Pin 9
#set_property PACKAGE_PIN P11 [get_ports {rc_dec}]          
#set_property IOSTANDARD LVCMOS33 [get_ports {rc_dec}]
#Pin 10
#set_property PACKAGE_PIN P10 [get_ports {encoder_b_m2}]          
#set_property IOSTANDARD LVCMOS33 [get_ports {encoder_b_m2}]

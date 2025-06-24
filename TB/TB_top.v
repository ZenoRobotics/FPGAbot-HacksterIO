`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/06/2025 10:15:18 PM
// Design Name: 
// Module Name: TB_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module TB_top;

// Inputs
reg clk = 1'b1;
reg rst_n = 1'b1;
reg uart_rx = 1'b1;

reg rc_fwd_r = 1'b0;
reg rc_rev_r = 1'b0;
reg rc_lft_r = 1'b0; 
reg rc_rt_r  = 1'b0;

wire pwm_l298n_enA_w;
wire pwm_l298n_enB_w;
wire in1_l298n_dir_w;
wire in2_l298n_dir_w;
wire in3_l298n_dir_w;
wire in4_l298n_dir_w;

wire encoder_a_m1_w;
wire encoder_b_m1_w;
wire encoder_a_m2_w;
wire encoder_b_m2_w;

wire pwm_256kHz_master_clk;

reg [6:0]  lft_motor_drag = 0;
reg [6:0]  rt_motor_drag = 0;
	
parameter c_CLOCK_PERIOD_NS = 10;   //100 MHz
parameter c_CLKS_PER_BIT    = 1302; //50 MHz(clks/sec)/ 38400 bits/sec = 1302
	
top uut(
    .clk_100MHz(clk),          //input
    .rst_n(rst_n),             //input
    .rst(1'b0),                //input
    // UART to RS2040
    .uart_rx(),  //input
    .uart_tx(),  //output
    //switches
    .rc_en(1'b1),    // sw in
    
    // PMOD Connections to motors and motor driver
    // Encoder Inputs from Motors 1 & 2
    .encoder_a_m1(encoder_a_m1_w),   // Left Motor
    .encoder_b_m1(encoder_b_m1_w),
    .encoder_a_m2(encoder_a_m2_w),   // Right Motor
    .encoder_b_m2(encoder_b_m2_w),
    
    // L298N PWM Input Fwd, Rev.
    // L298N PWM Enable = Speed
    .pwm_l298n_enA(pwm_l298n_enA_w),   //output
    .pwm_l298n_enB(pwm_l298n_enB_w),   //output
    .in1_l298n_dir(in1_l298n_dir_w),   //output
    .in2_l298n_dir(in2_l298n_dir_w),   //output
    .in3_l298n_dir(in3_l298n_dir_w),   //output
    .in4_l298n_dir(in4_l298n_dir_w),   //output
    
   
    .rc_fwd(rc_fwd_r),   //input
    .rc_rev(rc_rev_r),   //input
    .rc_lft(rc_lft_r),   //input 
    .rc_rt(rc_rt_r),     //input 
    
    //PZ for simulation only
    .o_sim_pwm_master_clk(pwm_256kHz_master_clk),
    
    // LEDs - Debug of RC Fwd, Rev, Lft, & Rt logic levels rcvd by FPGA
    .led()    // [3:0] output
     
    );
    
sim_motor_sys_emulator l_motor_inst(
    .clk(pwm_256kHz_master_clk),
    .rst(~rst_n),
    .motorId(1'b0),        // 0 = left, 1 = right
    .pwm(pwm_l298n_enA_w),
    .in1_l298n_dir(in1_l298n_dir_w),
    .in2_l298n_dir(in2_l298n_dir_w),
    .drag(lft_motor_drag),
    .enc_a(encoder_a_m1_w),
    .enc_b(encoder_b_m1_w)
    );
    
sim_motor_sys_emulator r_motor_inst(
    .clk(pwm_256kHz_master_clk),
    .rst(~rst_n),
    .motorId(1'b1),        // 0 = left, 1 = right
    .pwm(pwm_l298n_enB_w),
    .in1_l298n_dir(in3_l298n_dir_w),
    .in2_l298n_dir(in4_l298n_dir_w),
    .drag(rt_motor_drag),              //making this motor slower
    .enc_a(encoder_a_m2_w),
    .enc_b(encoder_b_m2_w)
    );
    
    
    always
       #(c_CLOCK_PERIOD_NS/2) clk <= ~clk;
       
    initial begin
		// Initialize Inputs
		rst_n    = 1'b1;
		rc_fwd_r = 1'b0;
		rc_rev_r = 1'b0;
		rc_lft_r = 1'b0;
		rc_rt_r  = 1'b0;
		lft_motor_drag = 7'h05;
		rt_motor_drag  = 7'h00;
		
		@(posedge clk);
		#1000;
		rst_n   = 1'b0;
		repeat (5)
		   @(posedge clk);  
		rst_n   = 1'b1;
		#1000;
		rc_fwd_r = 1'b1;
		@(posedge clk);
		#100000000;
		//add stimulus here: enable various RC directions
		@(posedge clk);
		#100000000;
		rc_fwd_r = 1'b0;
		rc_lft_r = 1'b1;
		@(posedge clk);
		#1000000;
		$finish;
     end  
endmodule

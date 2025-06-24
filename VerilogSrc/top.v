`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/21/2024 10:05:51 AM
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Changed PMOD JB connections and names to match Spartan 7's same base design.
//               - Works fine by switch setting control. Had to modify motor1 (left motor) and 
//                 motor2 (right motor) pwm connections in HDL to get proper motor directions.
// Revision 0.03 - Changed RC input to be on digital input lines. Required use of 47 ohm pulldown
//                 resistors.
// Revision 0.04 - Changed motor controller to interface to a single motor instead of dual motors.
//                 Added PID to motor controller module.
// Revision 0.05 - 
//
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//`define SIMULATION  //PZ for simulation only
`undef SIMULATION

module top(
    input clk_100MHz,       
    input rst_n,            
    input rst,
    // UART to RS2040
    input  uart_rx,
    output uart_tx,
    //switches
    input  rc_en,
    
    // PMOD Connections to motors and motor driver
    // Encoder Inputs from Motors 1 & 2
    input  encoder_a_m1,   // Left Motor
    input  encoder_b_m1,
    input  encoder_a_m2,   // Right Motor
    input  encoder_b_m2,
    
    // L298N PWM Enable = Speed
    output     pwm_l298n_enA,
    output     pwm_l298n_enB,
    output     in1_l298n_dir,
    output     in2_l298n_dir,
    output     in3_l298n_dir,
    output     in4_l298n_dir,
   
    input rc_fwd,
    input rc_rev,
    input rc_lft,
    input rc_rt,
    
    //PZ for simulation only
    //output     o_sim_pwm_master_clk,
    
    // LEDs - Debug of RC Fwd, Rev, Lft, & Rt logic levels rcvd by FPGA
    output [3:0] led,    // 
    
    //uart tx debug
    output    o_Tx_DV
     
    );
    
    // Internal wires for connecting inner modules
    wire        w_80Hz;
    wire [23:0] w_enc_cnt_m1;
    wire        w_dir_m1;
    wire [23:0] w_enc_cnt_m2;
    wire        w_dir_m2;
    wire [23:0] w_pos_enc_cnt_m1;
    wire [23:0] w_pos_enc_cnt_m2;
    reg  [23:0] r_enc_cnts_diff;
    wire [12:0] w_count;
    
    wire  [7:0] w_fwd_dir, w_rev_dir, w_lft_dir, w_rt_dir;
    wire  [7:0] w_setptL,w_setptR;
    reg         in1_l298n_dir_r = 1;
    reg         in2_l298n_dir_r = 0;
    reg         in3_l298n_dir_r = 1;
    reg         in4_l298n_dir_r = 0;
    
    wire   [6:0] motor1SpdMod_out_w;
    wire   [6:0] motor2SpdMod_out_w; 
    
    wire         zero_cntrs_w;
    wire         zero_encoders_w;

    //---------  rx-tx related signals and parameters
    parameter c_CLKS_PER_BIT   = 2604; //868; //100 MHz(clks/sec)/ 38400 bits/sec = 2604
	//for uart tx driver state machine
	parameter  SEND_TX_DV    = 2'b00;
	parameter  WAIT_TX_DONE  = 2'b01;
	parameter  DELAY_N_CLKS  = 2'b10;
	parameter  DELAY_PERIOD  = 2'b11;
	 
	reg  [1:0] uart_tx_state = 2'b11;
	 
	reg  [3:0] delay_space_tx = 8'h00;
	reg  [7:0] rx_data_buf = 8'h00;
	reg  [7:0] r_led = 8'h00;
	wire [7:0] w_Rx_Byte;
	wire       w_Rx_DV;
	reg  [7:0] r_Rx_Byte = 8'h00;
	reg        r_Rx_DV = 1'b0;
	reg  [7:0] r_Tx_Byte = 8'h00;
	reg        r_Tx_DV = 1'b0;
	wire       w_Tx_Done;
	reg        r_Tx_Done = 1'b0;
	
	reg  [2:0] data_sel_indx  = 3'b000;
	reg  [1:0] three_byte_cnt = 2'b00;
	wire [7:0] uart_tx_data_byte;
	 
	reg  [6:0] rx_data_lsbs = 6'b000000;
	reg        rx_inc_dec_bit = 1'b0;
	
	wire       Data_0_Delay;
	wire       Data_1_Delay;
	reg  [1:0] rx_in_delay_data_buf;
	reg        r_rx_byte_cnt = 1'b0;
	//-------------- rx-tx end ----------------
	    
    wire pb_start_sample;
    wire sampleCntOnOff_w;
    
    assign pb_start_sample = 1'b0;  //will move to UART Rx Register
    
    // from sensors proj
    parameter c_DATA_SIZE 	   = 8;	       // number of bits in a data word
    parameter c_ADDR_SPACE_EXP = 4;        // number of address bits (2^4 = 16 addresses)
    
    // motor commands
    parameter  [7:0]  som_byte = 8'h55;   // start of message byte
    // --command list-- 
    // The decoder will decide where the data gets stored (register bank addr) or routed 
    parameter  [7:0]  tics_per_rev = 8'h11; // a two byte value follows: High byte, then low byte.
    parameter  [7:0]  set_speed1 = 8'h21;   // set point speed 0-255 or 0x00 to 0xff
    parameter  [7:0]  set_speed2 = 8'h22;   // and 0-127 is reverse (0 max spd) or 0x00 to 0x7f
    parameter  [7:0]  set_accel  = 8'h23;   // 
    parameter  [7:0]  get_encdr1 = 8'h24;   // 
    parameter  [7:0]  get_encdr2 = 8'h25;   //   
    parameter  [7:0]  rst_encdrs = 8'h26;   // 
    
    wire l_dir, r_dir;
    
    wire [7:0] speed1;  // range = 0 to 127 + direction bit
    wire [7:0] speed2;  // range = 0 to 127 + direction bit
    ///////////// SetPoints and Simulation global settings ///////////////////
    `ifdef SIMULATION
       reg  [6:0] curr_setpt1 = 7'h48;
       reg  [6:0] curr_setpt2 = 7'h48;
       localparam sim_val = 1;
    `else
       reg  [6:0] curr_setpt1 = 7'h40;
       reg  [6:0] curr_setpt2 = 7'h40;
       localparam sim_val = 0;
    `endif
    ////////////////////////////////////////////////////////
    wire pwm_master_clk_w;  //256 kHz
    wire clk_8_192_MHz_w;
    wire debouce_clk_w;
    wire rst_db_w;
    wire reset;
    wire pid_sample_clk_w;
    wire sample_mode_en;
    wire stop; 
    wire clk2khz_delay_source;
    reg  delay_pulse   = 1'b0;
    reg  delay_pulse_r = 1'b0;
    reg  delay_pulse_256k = 1'b0;
    reg  delay_pulse_256k_r = 1'b0;
    
    assign sample_mode_en = 1'b0; //temp
    assign stop = 1'b0;           //temp
    assign zero_cntrs_w = 1'b0;   //temp
    
    //uart tx debug
    assign o_Tx_DV = r_Tx_DV;

    assign led = (rc_en == 1'b1) ?{rc_fwd,rc_rev,rc_lft,rc_rt} : 4'hf;
    assign reset = ~rst_n;
    
    //PZ for simulation only
    //assign o_sim_pwm_master_clk = pwm_master_clk_w;
    
    // example: assign x = (val==0) ?   a : 
    //                     (val==1) ?   b : 
    //                                'bx ;
    
    //These wires will be used in the future during non-RC mode
    //They will be muxed with RC direction signals and mode will be select.
    //temp
    assign l_dir = 1'b0;
    assign r_dir = 1'b0; 
    
    assign in1_l298n_dir = in1_l298n_dir_r;
    assign in2_l298n_dir = in2_l298n_dir_r;
    assign in3_l298n_dir = in3_l298n_dir_r;
    assign in4_l298n_dir = in4_l298n_dir_r;
    
    //ouput direction assignments to IN1, IN2 for motor1 (L), and IN3, IN4 for motor2 (R)
    
    always@(w_setptL[7], w_setptR[7]) begin
       case({w_setptL[7], w_setptR[7]})
          2'b00 : begin //fwd
                    in1_l298n_dir_r <= 1'b1;
                    in2_l298n_dir_r <= 1'b0;
                    in3_l298n_dir_r <= 1'b1;
                    in4_l298n_dir_r <= 1'b0;
                  end
          2'b11 : begin //rev
                    in1_l298n_dir_r <= 1'b0;
                    in2_l298n_dir_r <= 1'b1;
                    in3_l298n_dir_r <= 1'b0;
                    in4_l298n_dir_r <= 1'b1;
                  end
          2'b10 : begin //lft turn
                    in1_l298n_dir_r <= 1'b0;
                    in2_l298n_dir_r <= 1'b1;
                    in3_l298n_dir_r <= 1'b1;
                    in4_l298n_dir_r <= 1'b0;
                  end
          2'b01 : begin //rt turn
                     in1_l298n_dir_r <= 1'b1;
                     in2_l298n_dir_r <= 1'b0;
                     in3_l298n_dir_r <= 1'b0;
                     in4_l298n_dir_r <= 1'b1;
                   end
         default : begin //fwd
                     in1_l298n_dir_r <= 1'b1;
                     in2_l298n_dir_r <= 1'b0;
                     in3_l298n_dir_r <= 1'b1;
                     in4_l298n_dir_r <= 1'b0;
                    end
       endcase      
    end
   
    /*
    uart_rx #(.CLKS_PER_BIT(c_CLKS_PER_BIT)) UART_RX_INST
    (.i_Clock(clk_100MHz),
     .i_Rst(rst),
     .i_Rx_Serial(uart_rx),
     .o_Rx_DV(w_Rx_DV),
     .o_Rx_Byte(w_Rx_Byte)
     );

    uart_tx #(.CLKS_PER_BIT(c_CLKS_PER_BIT)) UART_TX_INST
    (.i_Clock(clk_100MHz),
     .i_Rst(rst),
     .i_Tx_DV(r_Tx_DV),
     .i_Tx_Byte(uart_tx_data_byte), //r_Tx_Byte),
     .o_Tx_Active(),
     .o_Tx_Serial(uart_tx),
     .o_Tx_Done(w_Tx_Done)
     );
    */
    
    // Instantiate inner design modules
    eightyHz_gen hz80(.clk_256kHz(pwm_master_clk_w), .reset(reset), .clk_80Hz_out(w_80Hz));
                      
    //debounce db_pb(.clk(w_80Hz),.i(start_sample),.o(pb_start_sample));
 
    encoder_cntr_module enc_cntr_Lmotor 
    (.clk(clk_100MHz),        
     .rst(reset),
     .zero_cntrs(zero_cntrs_w),
     .sampleModeEn(sample_mode_en),
     .quadA(encoder_a_m1), 
     .quadB(encoder_b_m1), 
     .count(w_enc_cnt_m1),
     .pos_count(w_pos_enc_cnt_m1),
     .direction(w_dir_m1),
     .sampleCntOnOff(sampleCntOnOff_w)
     );
     
    encoder_cntr_module enc_cntr_Rmotor 
    (.clk(clk_100MHz),        
     .rst(reset),
     .zero_cntrs(zero_cntrs_w),
     .sampleModeEn(sample_mode_en),
     .quadA(encoder_a_m2), 
     .quadB(encoder_b_m2), 
     .count(w_enc_cnt_m2), // output
     .pos_count(w_pos_enc_cnt_m2),
     .direction(w_dir_m2),  // output
     .sampleCntOnOff(sampleCntOnOff_w)
     );
     
    motor_cntlr
    #(.SIM(sim_val)
    )  motor_cntlr_inst
   (.i_Clock(pwm_master_clk_w),  //Master PWM clock: 256 kHz
    .i_Rst(reset),
    .i_Stop(stop),
    .i_StartSample(pb_start_sample),
    .i_SampleModeEn(sample_mode_en),
    //rc values if going to use with PID as well
    .i_RC_setptL(w_setptL),
    .i_RC_setptR(w_setptR),
    .i_SetPt(curr_setpt1),  // user set point for PID
    .i_EncCntL(w_pos_enc_cnt_m1),    //input
    .i_EncCntR(w_pos_enc_cnt_m2),
    //.i_WhlDirR(w_dir_m2),
    //debug 
    .o_motor1SpdMod(motor1SpdMod_out_w),
    .o_motor2SpdMod(motor2SpdMod_out_w),
    //end debug
    .o_zero_encoders(zero_encoders_w),
    .o_sampleStateFlag(sampleCntOnOff_w),
    .o_PWM_Out_L(pwm_l298n_enA),   //to motor 1 = left motor
    .o_PWM_Out_R(pwm_l298n_enB),   //to motor 2 = right motor
    .o_clk2khz(clk2khz_delay_source)
    );
     
     //Generate lowest frequency possible with pll that is multiple of pwm clk 
    clk_wiz_1  u_pll_pwm 
    (
        .clk_8_192_out(clk_8_192_MHz_w),
        .reset(reset),      
        .locked(),    
        .clk_in1(clk_100MHz)
    );
    
    //Derive 2kHz  x 128 count for pwm  = master clock 
    clk_gen_256khz u_pwm_clk_gen(
        .clk_8_192_MHz(clk_8_192_MHz_w),
        .reset(reset),
        .clk_256khz_out(pwm_master_clk_w)
    );
    
     rc_signal_select rc_signal_select_inst(
        .clk(w_80Hz),
        .rst(reset),
        .rc_fwd(rc_fwd),
        .rc_rev(rc_rev),
        .rc_lft(rc_lft),
        .rc_rt(rc_rt),
        .rc_en(rc_en),
        .rw_dir_in(r_dir),  //change to autonomous control via Pico rather than switches
        .lw_dir_in(l_dir),  //"  "  "  "
        .usr_setpt(curr_setpt1),      //Used for sampling mode only
        .cmd_mode_en(cmd_mode_en),    //Future: this bit will be set by a write to a set of command registers via uart-rpi connect
        .setptL(w_setptL),
        .setptR(w_setptR)
    );
    
    
endmodule

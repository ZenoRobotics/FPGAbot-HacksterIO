`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/16/2024 08:04:45 AM
// Design Name: 
// Module Name: motor_cntlr
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


module motor_cntlr #(parameter SIM = 0)
   (
   input        i_Clock,  //Master PWM clock: 256 kHz
   input        i_Rst,
   input        i_Stop,
   input        i_StartSample,
   input        i_SampleModeEn,
   input  [6:0] i_SetPt,  // user set point for PID
   //rc values if going to use with PID as well
   input  [7:0] i_RC_setptL,
   input  [7:0] i_RC_setptR,
   // From Motor's Encoder Counter
   input [23:0] i_EncCntL,
   input [23:0] i_EncCntR,
   input        i_WhlDirL,  // Encoder Cntr Logic defines 1 = Forward, 0 = Reverse
   input        i_WhlDirR,
   output       o_sampleStateFlag,
   // debug
   output  [6:0] o_motor1SpdMod,
   output  [6:0] o_motor2SpdMod,
   // end debug
   output        o_zero_encoders,
   // To L298N Motor Driver
   output       o_PWM_Out_L,
   output       o_PWM_Out_R,
   output       o_clk2khz 
    );
    
   // In this case, the number of tics (counts) of the encoder/rev is approx. 2110 
   // Note, for future use possibly, the circumference of the tire = 8.75 inches
   
   wire [15:0] SAMPLE_CYCLE_CLK_PRESCALER; 
   
   wire  [7:0] pid_out_setptL_w;
   wire  [7:0] pid_out_setptR_w;
   
   assign SAMPLE_CYCLE_CLK_PRESCALER = (SIM == 0) ? 16'h0640 : 16'h0640; 
   
   pwm_module pwm_motor_lft_inst(
    .clk_256k(i_Clock),      // divided by 128 = pwm freq of 2kHz
    .rst(i_Rst),
    .setpt_cnt_in(pid_out_setptL_w),  // direction (0 = fwd, 1 = rev) + 7 bits range [0,277]
    .pwm_out(o_PWM_Out_L),   
    .clk2khz_out(o_clk2khz)
    );  
    
    pwm_module pwm_motor_rt_inst(
    .clk_256k(i_Clock),      // divided by 128 = pwm freq of 2kHz
    .rst(i_Rst),
    .setpt_cnt_in(pid_out_setptR_w),  
    .pwm_out(o_PWM_Out_R),  
    .clk2khz_out()
    );  
    
    pid_module  #(.SIM(SIM)) pid_inst
    (
     .clk(i_Clock),          //Master PWM clock: 256 kHz
     .rst(i_Rst),
     .startSample(i_StartSample),
     .sampleModeEn(i_SampleModeEn),
     .clk_prescaler(SAMPLE_CYCLE_CLK_PRESCALER), // 25600 dec for 100ms
     .usr_setpt(i_SetPt),     //user setpoint into PID system
     .setpt_in_rc_mod1(i_RC_setptL),
     .setpt_in_rc_mod2(i_RC_setptR),
     .feedback_cnt1(i_EncCntL),         //feedback delta count calculated from samples encoder values
     .feedback_cnt2(i_EncCntR),         //feedback delta count calculated from samples encoder values
     .wheel_dir_1(i_RC_setptL[7]),
     .wheel_dir_2(i_RC_setptR[7]),
     //debug
     .motor1SpdMod_out(o_motor1SpdMod),
     .motor2SpdMod_out(o_motor2SpdMod),
     //end debug
     .zero_encoders(o_zero_encoders),
     .pid_out_setpt1(pid_out_setptL_w),     //fed to pwm_module then out to motor
     .pid_out_setpt2(pid_out_setptR_w),     //fed to pwm_module then out to motor
     .sampleFlag(o_sampleStateFlag)
    );
    
endmodule

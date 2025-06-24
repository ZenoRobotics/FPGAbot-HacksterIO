`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/16/2024 08:28:57 AM
// Design Name: 
// Module Name: pwm_module
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


module pwm_module(
    input  clk_256k,      // divided by 128 = pwm freq of 2kHz
    input  rst,
    input  [7:0] setpt_cnt_in,  // direction (0 = fwd, 1 = rev) + 7 bits range [0 to 127]
    output       pwm_out,  
    output       clk2khz_out // a 2kHz clock source for use (delay) at top level
    );
    
    //reg [7:0] rpm_to_duty_cycle_cnt_val = 0;  
    reg [6:0] pwm_duty_cycle_cnt = 0;
    
    assign pwm_out = (rst == 1'b1) ? 1'b0 :
                     (pwm_duty_cycle_cnt < setpt_cnt_in[6:0]) ? 1'b1 : 1'b0;
    
    
    //pwm with calculated duty cycle period generation
    always@(posedge clk_256k, posedge rst) begin
      if(rst == 1'b1) 
        pwm_duty_cycle_cnt <= 0;
      else 
        pwm_duty_cycle_cnt <= pwm_duty_cycle_cnt + 7'h01;
     end
    
     assign clk2khz_out = (pwm_duty_cycle_cnt[6] == 1'b0) ? 1'b0 : 1'b1;
     
endmodule

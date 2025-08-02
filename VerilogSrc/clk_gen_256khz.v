`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/25/2025 08:49:06 PM
// Design Name: 
// Module Name: clk_gen_256hz
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


module clk_gen_256khz(
    input clk_8_192_MHz,
    input reset,
    output clk_256khz_out
    );
    
   reg  [4:0] clk_256khz_out_r = 5'b00000;
   
   always @(posedge clk_8_192_MHz or posedge reset)
		if(reset)
		  clk_256khz_out_r <= 0;
		else
		  clk_256khz_out_r <= clk_256khz_out_r + 1;
	
	assign clk_256khz_out = (clk_256khz_out_r[4] == 1'b0) ? 1'b1 : 1'b0; // assert tick half of the time
   
    
endmodule


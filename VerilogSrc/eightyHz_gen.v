`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/21/2024 10:05:51 AM
// Design Name: 
// Module Name: eightyHz
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


module eightyHz_gen(
    input clk_256kHz,
    input reset,
    output clk_80Hz_out
    );
    
    reg [10:0] ctr_reg = 0; // 11 bits to cover 3,200
    reg clk_out_reg = 0;
    
    always @(posedge clk_256kHz or posedge reset)
        if(reset) begin
            ctr_reg <= 0;
            clk_out_reg <= 0;
        end
        else
            if(ctr_reg == 1_599) begin  // 256kHz / (80Hz * 2) = 1600
                ctr_reg <= 0;
                clk_out_reg <= ~clk_out_reg;
            end
            else
                ctr_reg <= ctr_reg + 1;
    
    assign clk_80Hz_out = clk_out_reg;
endmodule

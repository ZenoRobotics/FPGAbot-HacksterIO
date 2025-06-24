`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/15/2024 11:08:29 AM
// Design Name: 
// Module Name: encoder_cntr_module
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


module encoder_cntr_module(

   input clk, rst, quadA, quadB,zero_cntrs,
   input sampleCntOnOff,sampleModeEn,
   output direction,  // 1 = Forward, 0 = Reverse
   output reg [23:0] count = 24'h000000,
   output reg [23:0] pos_count = 24'h000000
);
   reg [2:0] quadA_delayed = 3'b000;
   reg [2:0] quadB_delayed = 3'b000;
   always @(posedge clk) quadA_delayed <= {quadA_delayed[1:0], quadA};
   always @(posedge clk) quadB_delayed <= {quadB_delayed[1:0], quadB};

   wire count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2];
   wire count_direction = quadA_delayed[1] ^ quadB_delayed[2];  // defined here as 1'b1 = fwd, 0 = rev.

   
   always @(posedge clk, posedge rst, posedge zero_cntrs) begin
     if ((rst == 1'b1) || (zero_cntrs == 1'b1))
        count = 24'h000000;
     else begin
       if ((sampleCntOnOff == 1'b0) && (sampleModeEn == 1'b1))
         count <= count;
       else if(count_enable) begin
         if(count_direction) 
           count<=count+1; 
         else 
           count<=count-1;
       end
     end
   end
   
   assign direction = count_direction;
   
   //positive count only
   
 always @(posedge clk, posedge rst, posedge zero_cntrs) begin
     if ((rst == 1'b1) || (zero_cntrs == 1'b1))
        pos_count = 24'h000000;
     else begin
       if(count_enable) begin
           pos_count<=pos_count+1; 
       end
     end
   end

endmodule

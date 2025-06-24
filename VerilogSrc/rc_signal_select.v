`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/27/2025 10:31:26 AM
// Design Name: 
// Module Name: rc_signal_select
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


module rc_signal_select(
     input  clk,
     input  rst,
     //RC Signals
     input  rc_fwd,
     input  rc_rev,
     input  rc_lft,
     input  rc_rt,
     //RC Enable
     input  rc_en,
     //Command Value
     input  rw_dir_in,   // used when rc_en = 1'b0
     input  lw_dir_in,   // " "
     //Registered Value
     input  [6:0] usr_setpt,
     //Command Mode
     input  cmd_mode_en,
     output [7:0] setptL,  // msbs turn right, turn leftL rt = (1,0), lft = (0,1), forward = (0,0), rev = (1,1)
     output [7:0] setptR
    );
    
    reg [7:0] setptL_r = 0;
    reg [7:0] setptR_r = 0;
    
    assign setptL = setptL_r;
    assign setptR = setptR_r;
    
     always @(posedge clk or posedge rst)
       begin
		 if(rst) 
		   begin
		     setptL_r <= 8'h00;
		     setptR_r <= 8'h00;
		   end
		 //--------- RC Control Section --------//
		 else if (rc_en == 1'b1)
		   begin
		    if (rc_fwd == 1'b1) //_val > 220)
		      begin
		        setptL_r <= {1'b0,usr_setpt};
		        setptR_r <= {1'b0,usr_setpt};
		      end
		       
		    else if (rc_rev == 1'b1) //_val > 220) 
		      begin
		        setptL_r <= {1'b1,usr_setpt};
		        setptR_r <= {1'b1,usr_setpt};
		      end
		    
		    else if (rc_lft == 1'b1) //_val > 220)
		      begin
		        setptL_r <= {1'b1,usr_setpt}; //rev
		        setptR_r <= {1'b0,usr_setpt}; //fwd
		      end
		    
		    else if (rc_rt == 1'b1) //_val > 220)
		      begin
		        setptL_r <= {1'b0,usr_setpt}; //fwd
		        setptR_r <= {1'b1,usr_setpt}; //rev
		      end
		    else //stop
		      begin
		        setptL_r <= 8'h00;
		        setptR_r <= 8'h00;
		      end
		  end
		  //------- End of RC Control Section --------//
		  //------ Else, Sensor and Alg. Drive Movements supplied by:
		  //------ a) FPGA and/or Pico alone, or
		  //------ b) Main computer running ROS1/2 SLAM algorithm
		  else 
		    begin
		      if (lw_dir_in == 1'b0 && rw_dir_in == 1'b0) // forward
		        begin
		          setptL_r <= {1'b0,usr_setpt};
		          setptR_r <= {1'b0,usr_setpt};
		        end 
		      else if (lw_dir_in == 1'b1 && rw_dir_in == 1'b0) // left turn
		        begin
		          setptL_r <= {1'b1,usr_setpt};
		          setptR_r <= {1'b0,usr_setpt};
		        end 
		      else if (lw_dir_in == 1'b0 && rw_dir_in == 1'b1) // right turn
		        begin
		          setptL_r <= {1'b0,usr_setpt};
		          setptR_r <= {1'b1,usr_setpt};
		        end 
		      else if (lw_dir_in == 1'b1 && rw_dir_in == 1'b1) // reverse
		        begin
		          setptL_r <= {1'b1,usr_setpt};
		          setptR_r <= {1'b1,usr_setpt};
		        end 
		      else //stop
		        begin
		          setptL_r <= 8'h00;
		          setptR_r <= 8'h00;
		        end
		    end
		 end
endmodule

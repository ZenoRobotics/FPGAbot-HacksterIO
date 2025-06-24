`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/06/2025 10:57:02 PM
// Design Name: 
// Module Name: sim_motor_sys_emulator
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


module sim_motor_sys_emulator(
    input clk,  //256K PWM master clock from Top
    input rst,
    input motorId,        // 0 = left, 1 = right
    input pwm,
    input in1_l298n_dir,
    input in2_l298n_dir,
    input [6:0] drag,
    output enc_a,
    output enc_b
    );
    
    //Sample States PWM
    localparam IDLE           = 2'b00;
    localparam START_SAMPLE   = 2'b01;
    localparam END_SAMPLE     = 2'b10;
    
    
    reg [1:0]  curr_pwm_state = IDLE;
    
    localparam MOVEMENT_THRESHOLD = 7'h05; 
    
    reg [6:0] pwm_sample_count      = 7'h00;
    reg [6:0] zero_count_pwm        = 7'h00;
    reg [6:0] setPointMeasured      = 7'h00;
    reg       pwm_sample_complete   = 1'b0;
    reg       r_pwm_sample          = 0;
    reg [6:0] setpt                 = 7'h00;
    reg [7:0] clk_freq_cnt          = 8'h00;
    reg [7:0] encoder_pulse_cnt_out = 8'h00;
    reg       quarter_pulse_out     = 1'b0;
    reg       pos_pulse_edge        = 1'b0;
    reg       neg_pulse_edge        = 1'b0;
 
    reg       encoder_pulse_out = 1'b1;
    reg       spin_direction = 1'b0; // defined in rc_signal_select.v as 1'b0 = fwd, 1 = rev.
    reg       prev_spin_dir  = 1'b0;
    
    //pwm input to pseudo random initial speed (angular rotation) of motor
    //angular rotation => encoder pulses
    //systematic + non-systematic (e.g., environment) errors
    
    //falling edge detection PWMs
    always@(posedge clk, posedge rst) begin
      if (rst == 1'b1) begin
         r_pwm_sample <= 1'b0;
      end
      else begin
         r_pwm_sample <= pwm;
      end
    end
    
    // 1) determine PWM input duty cycle
    always@(posedge clk, posedge rst) begin
       if (rst == 1'b1) begin
          pwm_sample_count         <= 7'h00;  
          zero_count_pwm           <= 7'h00;
          setPointMeasured         <= 7'h00;
          pwm_sample_complete      <= 1'b0;
          curr_pwm_state           <= IDLE;     
       end
       else begin
         // Regular autonomous mode, PID Calc section
         case(curr_pwm_state) 
            IDLE: begin
               pwm_sample_complete <= 1'b0;
               if ((r_pwm_sample == 1'b0) && (pwm == 1'b1)) begin //start of high duty cycle state - rising edge of pwm
                  pwm_sample_count <= pwm_sample_count + 7'h01;
                  curr_pwm_state   <= START_SAMPLE;  
               end
               else if ((r_pwm_sample == 1'b0) && (pwm == 1'b0)) begin //check for zero duty cycle
                 if (zero_count_pwm == 127) begin
                   setPointMeasured <= 7'h00; 
                   curr_pwm_state  <= END_SAMPLE;
                 end
                 else 
                  zero_count_pwm  <= zero_count_pwm + 7'h01;
               end  
            end
            START_SAMPLE : begin
               if ((r_pwm_sample == 1'b1) && (pwm == 1'b0)) begin //end of high duty cycle state
                  setPointMeasured  <= pwm_sample_count;
                  pwm_sample_complete <= 1'b1;
                  curr_pwm_state      <= END_SAMPLE;
               end
               else if (pwm_sample_count == 127) begin //check limit
                  setPointMeasured  <= 7'h7f;
                  pwm_sample_complete <= 1'b1;
                  curr_pwm_state      <= END_SAMPLE;
               end   
               else
                  pwm_sample_count <= pwm_sample_count + 7'h01;
            end
            END_SAMPLE: begin
               //give encoder time to grab duty cycle info
               pwm_sample_complete  <= 1'b0;
               pwm_sample_count     <= 7'h00;
               setPointMeasured     <= 7'h00;
               zero_count_pwm       <= 7'h00;
               curr_pwm_state       <= IDLE;
            end
           
         endcase
       end
    end
    
    // Wheel direction info
    always@(posedge clk, posedge rst) begin 
      if (rst == 1'b1) begin
         spin_direction   <= 1'b0;     // defined here as 1'b0 = fwd, 1 = rev.
         prev_spin_dir    <= 1'b0;
      end
      else begin
         prev_spin_dir    <= spin_direction;
         if (in1_l298n_dir == 1'b1 && in2_l298n_dir == 1'b0) // forward
		    spin_direction <= 1'b0;
		 else if (in1_l298n_dir == 1'b0 && in2_l298n_dir == 1'b1) // reverse
		    spin_direction <= 1'b1;
		 else //stop
		    spin_direction <= prev_spin_dir;
	  end
	end  
	
    
    // assign sampled setpoint count to current motor setpoint
    always@(posedge clk, posedge rst) begin 
      if (rst == 1'b1) begin
         spin_direction   <= 1'b1;     // defined here as 1'b1 = fwd, 0 = rev.
         setpt <= 7'h00;
      end
      else if (pwm_sample_complete == 1'b1)  begin
         if (setPointMeasured <= MOVEMENT_THRESHOLD)  //motor is off   
            setpt <= 7'h00;
         else if (setPointMeasured >= 7'h20) //forward (?)
            setpt <= setPointMeasured - drag;
         else
            setpt <= 7'h48; // *same as base set in top -didn't want to put in special I/O to pass here
       end
    end 
    
    //Counter Channels A/B Generator
    //create encoder pulses proportional and direct correct with respect to PWM duty cycles settings obtained
    //using discrete lookup table with only best guess approximations
    always@(posedge clk) begin 
      if(pwm_sample_complete == 1'b1)  begin
         if (setpt < 7'h05)
           clk_freq_cnt <= 8'h00;
         else if ((setpt > 8'h30) && (setpt <= 8'h32)) //1600 Hz
           clk_freq_cnt <= 8'ha0;
         else if (setpt <= 8'h35) //1700 Hz
           clk_freq_cnt <= 8'h96;
         else if (setpt <= 8'h38) //1800 Hz
           clk_freq_cnt <= 8'h8e;
         else if (setpt <= 8'h3b) //1900 Hz
           clk_freq_cnt <= 8'h86;
         else if (setpt <= 8'h3e) //2000 Hz
           clk_freq_cnt <= 8'h80;
         else if (setpt <= 8'h41) //2100 Hz
           clk_freq_cnt <= 8'h79;
         else if (setpt <= 8'h44) //2200 Hz
           clk_freq_cnt <= 8'h74;
         else if (setpt <= 8'h47) //2300 Hz
           clk_freq_cnt <= 8'h6f;
         else if (setpt <= 8'h4a) //2400 Hz
           clk_freq_cnt <= 8'h6a;
         else if (setpt <= 8'h4d) //2500 Hz
           clk_freq_cnt <= 8'h66;
         else if (setpt <= 8'h50) //2600 Hz
           clk_freq_cnt <= 8'h62;
         else if (setpt <= 8'h53) //2700 Hz
           clk_freq_cnt <= 8'h5e;
         else if (setpt <= 8'h56) //2800 Hz
           clk_freq_cnt <= 8'h5b;
         else if (setpt <= 8'h59) //2900 Hz
           clk_freq_cnt <= 8'h58;
         else if (setpt <= 8'h5c) //3000 Hz
           clk_freq_cnt <= 8'h5f;
         //just defaulting to 3500 Hz for any value larger for setpt (85)
         else
           clk_freq_cnt <= 8'h49;
       end
    end
    
    always@(posedge clk, posedge rst) begin 
      if (rst == 1'b1) begin
         encoder_pulse_cnt_out <= 8'h00;
         encoder_pulse_out     <= 1'b0;
         quarter_pulse_out     <= 1'b0;
         pos_pulse_edge        <= 1'b0;
         neg_pulse_edge        <= 1'b0;
      end
      else begin
         if ((encoder_pulse_cnt_out == (clk_freq_cnt >> 1)) && (encoder_pulse_cnt_out > 8'h00)) begin
            encoder_pulse_out <= ~encoder_pulse_out;
            encoder_pulse_cnt_out <= 8'h00;
            if (encoder_pulse_out == 1'b0)
               pos_pulse_edge <= 1'b1;
            else
               neg_pulse_edge <= 1'b0;
         end
         else if ((encoder_pulse_cnt_out == (clk_freq_cnt >> 2)) && (encoder_pulse_cnt_out > 8'h00)) begin
            encoder_pulse_cnt_out <= encoder_pulse_cnt_out + 8'h01;
            pos_pulse_edge <= 1'b0;
            neg_pulse_edge <= 1'b0;
            if((pos_pulse_edge == 1'b1) || (neg_pulse_edge == 1'b1))
               quarter_pulse_out <= 1'b1;
            else
               quarter_pulse_out <= 1'b0;
         end
         else
            encoder_pulse_cnt_out <= encoder_pulse_cnt_out + 8'h01; 
      end
    end
    
    assign enc_a = (spin_direction == 1'b0) ? encoder_pulse_out : quarter_pulse_out;
    assign enc_b = (spin_direction == 1'b1) ? encoder_pulse_out : quarter_pulse_out;
endmodule

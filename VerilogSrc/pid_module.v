`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/07/2025 03:55:18 PM
// Design Name: 
// Module Name: pid_module
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


module pid_module #(parameter SIM = 0)
(
    input         clk,
    input         rst,
    input         startSample,
    input         sampleModeEn,    // pass through usr_setpt to pid_out_setpt for test
    input  [15:0] clk_prescaler,
    input   [7:0] usr_setpt,
    input   [7:0] setpt_in_rc_mod1,
    input   [7:0] setpt_in_rc_mod2,
    input  [23:0] feedback_cnt1, //needs conversion to setpt format - positive only from encoder counter 
    input  [23:0] feedback_cnt2, //needs conversion to setpt format - positive only from encoder counter
    input         wheel_dir_1,   //fwd = 0, rev = 1
    input         wheel_dir_2,   //fwd = 0, rev = 1
    //debug       
    output  [6:0] motor1SpdMod_out,
    output  [6:0] motor2SpdMod_out,
    //end debug
    output        zero_encoders,  //goes to encoder counters L&R
    output  [7:0] pid_out_setpt1,
    output  [7:0] pid_out_setpt2,
    output  sampleFlag // encoder counters on/off switch for sampleMode
    );
    
    // Local Parameter (constant)
    //++++++++++++++++++++++++++++++++++++++++++++++++++++
    localparam Ki  = 1;
    localparam Ke  = 2;
    localparam Kv  = 3;
    //++++++++++++++++++++++++++++++++++++++++++++++++++++
    localparam SAMPLE_RUN_LOOPS = 10;  // Sample Time = 10 x clk_prescaler x 1/clk_freq = 1sec
    localparam SETPT_TO_TICS_PER_UNIT_TIME  = 3; // 3 tics/0.1sec/setpt
    localparam MAX_SPEED = 127;
    localparam MIN_SETPT_ALLOWED = 48; // motors don't move well at lesser settings.
    localparam THRESHOLD_DXDT = 12; // make sure motor is moving
    localparam DOMINANT_CNT_MAX = 9'h06;
    
    // PID States
    localparam IDLE           = 3'b000;
    localparam CALCULATIONS_1 = 3'b001;
    localparam CALCULATIONS_2 = 3'b010;
    localparam CALCULATIONS_3 = 3'b011;
    localparam CALCULATIONS_4 = 3'b100;
    localparam CALCULATIONS_5 = 3'b101;
    localparam CALCULATIONS_6 = 3'b110;
    localparam SET_NEW_SPDS   = 3'b111;
    
    reg [2:0]  curr_state, next_state = IDLE;
    
    // Internal signals
    
    reg signed [23:0] motor1SpdMod = 24'sh000000;
    reg signed [23:0] motor2SpdMod = 24'sh000000;
    reg signed [6:0]  control_signal1 = 7'sh00; 
    reg signed [6:0]  control_signal2 = 7'sh00;
    reg signed [23:0] prevMotor1SpdMod_Ki = 24'sh000000;
    reg signed [23:0] prevMotor2SpdMod_Ki = 24'sh000000;
  
    reg  [1:0] prev_direction = 2'b00; 
    
    // Clock divider for sampling rate
    reg [15:0] clk_divider = 0;
    reg pidCalcFlag = 0;
    
    reg sampleStateFlag_r = 0;  // high when and sampleModeEn and startCntPulse_r goes high, else low
    reg startSample_r = 0;
    reg  [3:0] sampleLoopCnt = 4'h0;
    reg  startCntPulse_r = 0;
    
    reg signed [23:0] dxdt_enc1  = 24'sh000000; 
    reg signed [23:0] dxdt_enc2  = 24'sh000000;
    reg signed [23:0] ek_21      = 24'sh000000;
    reg signed [23:0] ek_12      = 24'sh000000;
    reg [23:0] prev_encdr_cnt1   = 24'h000000;
    reg [23:0] prev_encdr_cnt2   = 24'h000000;
    reg signed [23:0] Kvdx1dx2   = 24'sh000000;
    reg signed [23:0] Kvdx2dx1   = 24'sh000000;
    reg signed [23:0] Ke_ek_12    = 24'sh000000;
    reg signed [23:0] Ke_ek_21    = 24'sh000000;
    reg signed [23:0] dx1dx2      = 24'sh000000;
    reg signed [23:0] dx2dx1      = 24'sh000000;
     
    reg  [7:0] pid_out_setpt1_r;
    reg  [7:0] pid_out_setpt2_r;
    reg  [7:0] curr_pid_setpt1_r = 8'h00;
    reg  [7:0] curr_pid_setpt2_r = 8'h00;
    reg  [7:0] sample_setpt1_r;
    reg  [7:0] sample_setpt2_r;
    reg  [9:0] target_delta_tics1 = 10'h000;
    reg  [9:0] target_delta_tics2 = 10'h000;
    reg        zero_encoders_r = 1'b0;
    reg        keep_curr_pid_setpts_flag = 0;
    
    reg        dominant_motor = 0;
    reg  [7:0] dominant_motor0_count = 8'h00;
    reg  [7:0] dominant_motor1_count = 8'h00;
    reg  [8:0] dominant_motor_sample_count = 9'h000;
    reg        dominant_motor_found = 1'b0;
    
    wire [23:0] feedback_cnt1_w;
    wire [23:0] feedback_cnt2_w;
    
    assign sampleFlag = sampleStateFlag_r;
    //assign zero_encoders = zero_encoders_r;
    
    assign pid_out_setpt1 = (sampleModeEn == 1) ? sample_setpt1_r : pid_out_setpt1_r; 
    assign pid_out_setpt2 = (sampleModeEn == 1) ? sample_setpt2_r : pid_out_setpt2_r;
    assign motor1SpdMod_out = motor1SpdMod[6:0];
    assign motor2SpdMod_out = motor2SpdMod[6:0];
    
    assign feedback_cnt1_w = (SIM == 0) ? feedback_cnt1 : feedback_cnt1*10;
    assign feedback_cnt2_w = (SIM == 0) ? feedback_cnt2 : feedback_cnt2*10;

    always @(posedge clk or posedge rst) begin
    //$display("Clock trigered");
        if (rst) begin
            clk_divider <= 16'h0000;
            sampleLoopCnt <= 4'h0;
            pidCalcFlag <= 0;
        end
        else if (((startCntPulse_r == 1) || (sampleStateFlag_r == 1'b0)) && (sampleModeEn == 1)) begin
            clk_divider <= 16'h0000;
            sampleLoopCnt <= 4'h0;
            pidCalcFlag <= 0;
        end
        // Logic Below: pidCalcFlag goes high for one clock cycle every clk_prescalar clock tics
        else if (clk_divider == clk_prescaler) begin // clk_prescaler determines the sampling rate, thus sampling rate would be clk freq/clk_prescaler
            clk_divider <= 16'h0000;
            if (sampleStateFlag_r == 1'b1)
                sampleLoopCnt <= sampleLoopCnt + 4'h1;
            pidCalcFlag <= 1;
        end else begin
            clk_divider <= clk_divider + 1;
            pidCalcFlag <= 0;
        end
    end
    
    // sample mode logic only
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset logic generally specific to application
            startSample_r <= 0;
            sampleStateFlag_r <= 0;
            startCntPulse_r <= 0;
            sample_setpt1_r <= 8'h00;
            sample_setpt2_r <= 8'h00;
        end 
        else begin
            startSample_r <= startSample;
            if ((startSample_r == 0) && (startSample == 1) && (sampleModeEn == 1)) begin
                sampleStateFlag_r <= 1'b1;
                startCntPulse_r <= 1'b1;
                sample_setpt1_r <= usr_setpt;
                sample_setpt2_r <= usr_setpt;
            end
            else if ((startCntPulse_r == 1) && (sampleModeEn == 1)) begin
                startCntPulse_r <= 1'b0;
                sample_setpt1_r <= usr_setpt;
                sample_setpt2_r <= usr_setpt;
            end
            else if (sampleLoopCnt == SAMPLE_RUN_LOOPS) begin
                sampleStateFlag_r <= 1'b0;
                sample_setpt1_r <= 8'h00;
                sample_setpt2_r <= 8'h00;
            end
        end
    end
    
    //StateMachine current state register/synchronization 
    always@(posedge clk or posedge rst) begin
        if(rst == 1'b1)
           curr_state <= IDLE;
        else
           curr_state <= next_state;
     end
     
     
    //PID logic
    always@(posedge clk or posedge rst) begin
        if(rst == 1'b1) begin
           dxdt_enc1     <= 24'sh000000;
           dxdt_enc2     <= 24'sh000000;
           next_state    <= IDLE;
           motor1SpdMod  <= 24'sh000000;
           motor2SpdMod  <= 24'sh000000;
           zero_encoders_r   <= 1'b0;
           prev_direction <= 2'b00; 
           pid_out_setpt1_r <= 8'h00;
           pid_out_setpt2_r <= 8'h00;
           curr_pid_setpt1_r <= setpt_in_rc_mod1;
           curr_pid_setpt2_r <= setpt_in_rc_mod2;
           keep_curr_pid_setpts_flag <= 1'b1;
           dominant_motor <= 0;
           dominant_motor0_count <= 8'h00; //8 bit counter
           dominant_motor1_count <= 8'h00; //8 bit counter
           dominant_motor_sample_count <= 9'h000; //9 bit counter
           dominant_motor_found <= 1'b0;
        end
       else begin
       // Regular autonomous mode, PID Calc section
       case(curr_state) 
       IDLE: begin
            if(pidCalcFlag && ~sampleModeEn) begin  // 100 ms pass?
                if (setpt_in_rc_mod1[6:0] > 7'h00)begin
                   if(curr_pid_setpt1_r[6:0] < 7'h20) begin //low threshold for initial movement
                      pid_out_setpt1_r <= setpt_in_rc_mod1;
                      pid_out_setpt2_r <= setpt_in_rc_mod2;
                   end
                   /*
                   if ((({wheel_dir_2,wheel_dir_1} == 2'b11) && (prev_direction) == 2'b00)  ||
                   (({wheel_dir_2,wheel_dir_1} == 2'b00) && (prev_direction == 2'b11)) ||
                    ({wheel_dir_2,wheel_dir_1} == prev_direction)) 
                      zero_encoders_r   <= 1'b0;
                   else
                      zero_encoders_r   <= 1'b1;
                   prev_direction <= {wheel_dir_2,wheel_dir_1};
                   */
                   zero_encoders_r   <= 1'b0;
                   next_state <= CALCULATIONS_1;
                end
                else begin
                   pid_out_setpt1_r  <= 8'h00;
                   pid_out_setpt2_r  <= 8'h00;
                   zero_encoders_r   <= 1'b0;
                   next_state <= IDLE;
                end
              end
            end
         CALCULATIONS_1: begin
                //--not dealing with rollover right now
                dxdt_enc1   <= feedback_cnt1_w - prev_encdr_cnt1; //ok until rollover occurs - always positive
                dxdt_enc2   <= feedback_cnt2_w - prev_encdr_cnt2; //ok until rollover occurs - always positive
                //--two variable discrete time analysis: error
                ek_12 <= feedback_cnt1_w - feedback_cnt2_w;  //e[k]  two calculations used for abs
                ek_21 <= feedback_cnt2_w - feedback_cnt1_w;  //e[k]
                //find dominant motor initially, then keep
                if (dominant_motor_found == 1'b0) begin
                   if (dominant_motor_sample_count == DOMINANT_CNT_MAX) begin  
                      dominant_motor_found <= 1'b1;
                      if (dominant_motor0_count > dominant_motor1_count)
                         dominant_motor <= 1'b0;
                      else
                         dominant_motor <= 1'b1;
                   end
                   else begin
                      if ((feedback_cnt1_w > feedback_cnt2_w) && (motor1SpdMod == 24'h000000)) begin
                         dominant_motor0_count <= dominant_motor0_count + 8'h01; //8 bit counter
                         dominant_motor <= 1'b0;
                      end
                      else if ((feedback_cnt2_w > feedback_cnt1_w) && (motor2SpdMod == 24'h000000))begin
                         dominant_motor1_count <= dominant_motor1_count + 8'h01; //8 bit counter
                         dominant_motor <= 1'b1;
                      end
                      dominant_motor_sample_count <= dominant_motor_sample_count + 9'h001; //9 bit counter
                   end
                end
                next_state <= CALCULATIONS_2;
              end
         CALCULATIONS_2: begin
                Ke_ek_12 <= (ek_12 >>> Ke); 
                   
                Ke_ek_21 <= (ek_21 >>> Ke); 
                   
                dx1dx2  <= dxdt_enc1 - dxdt_enc2;
                dx2dx1  <= dxdt_enc2 - dxdt_enc1;
                next_state  <=  CALCULATIONS_3;
              end
         CALCULATIONS_3: begin
                Kvdx1dx2 <= dx1dx2 >>> Kv;
                Kvdx2dx1 <= dx2dx1 >>> Kv;
                next_state  <=  CALCULATIONS_4;
              end
         CALCULATIONS_4: begin 
                if (dominant_motor == 1'b0) begin
                   motor2SpdMod <= Kvdx1dx2 + Ke_ek_12; 
                   motor1SpdMod <= 0;
                end
                else if (dominant_motor == 1'b1) begin
                   motor1SpdMod <= Kvdx2dx1 + Ke_ek_21; 
                   motor2SpdMod <= 0;
                end
                else begin 
                   motor1SpdMod <= 0;
                   motor2SpdMod <= 0;
                end
                
                //check for any motor not moving, lead or not
                if (dxdt_enc2 < THRESHOLD_DXDT) begin //check for motor not moving
                   motor2SpdMod <= 0; 
                   motor1SpdMod <= 10;
                end
                if (dxdt_enc1 < THRESHOLD_DXDT) begin //check for motor not moving
                      motor2SpdMod <= 10; 
                      motor1SpdMod <= 0;
                end
             
                next_state  <=  CALCULATIONS_5;
             end
         CALCULATIONS_5: begin
                prev_encdr_cnt1 <= feedback_cnt1_w;
                prev_encdr_cnt2 <= feedback_cnt2_w;  
                //check for reset needed for encoder counters winding up for whatever reason
                if (motor1SpdMod[6:0] > 7'h15) begin 
                   //zero_encoders_r   <= 1'b1;
                   motor1SpdMod <= 10;
                end
                else if (motor2SpdMod > 7'h15) begin 
                   //zero_encoders_r   <= 1'b1;
                   motor2SpdMod <= 10;
                end
                else if (prevMotor1SpdMod_Ki[6:0] > 10)
                   prevMotor1SpdMod_Ki[6:0] <= 5;
                else if (prevMotor2SpdMod_Ki[6:0] > 10)
                   prevMotor2SpdMod_Ki[6:0] <= 5;
                   
                next_state  <=  CALCULATIONS_6;
             end
         CALCULATIONS_6: begin
                  //integration by using previous pid_out_setpt1/2_r
                  control_signal1 <= pid_out_setpt1_r[6:0] + motor1SpdMod[6:0] + prevMotor1SpdMod_Ki[6:0]; //derivative1 + integral1
                  control_signal2 <= pid_out_setpt2_r[6:0] + motor2SpdMod[6:0] + prevMotor2SpdMod_Ki[6:0]; //derivative2 + integral2
                  prevMotor1SpdMod_Ki <= motor1SpdMod >>> Ki;
                  prevMotor2SpdMod_Ki <= motor2SpdMod >>> Ki;
                  next_state  <=  SET_NEW_SPDS; //CALCULATIONS_5;
                end
        
         SET_NEW_SPDS: begin  //Add appropriate wheel direction. Check for max or above max settings.
                   curr_pid_setpt1_r <= {wheel_dir_1, control_signal1};
                   curr_pid_setpt2_r <= {wheel_dir_2, control_signal2}; 
                   pid_out_setpt1_r <=  {wheel_dir_1, control_signal1};
                   pid_out_setpt2_r <=  {wheel_dir_2, control_signal2};
                   next_state  <= IDLE;
                end
       endcase
     end
  end

endmodule

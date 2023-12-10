 module pid_tuner #(
  parameter DATA_WIDTH = 16, //only used at the 3rd subroutine
  parameter ENCODER_WIDTH = 3 //stop using it
)(
  input wire clk,              		// clock input. when there is no clk signal motor does not run, CLK @50MHz, T=20ns
  input wire reset,            		// when it is 1 motor does run
  input wire [2:0] pid_select,			//select control type: 100 for P, 110 for PI, 111 for PID (?)
  input wire [DATA_WIDTH-1:0] period_speed,//system output of speed measured in clock cycles
  output reg tuning_done,
	output reg [7:0] Kp,//proportional constant
	 output reg [7:0] Ki,//integral constant
	 output reg [6:0] Kd//derivative constant
  
  
);


	reg division_trig =1'b0;
	reg [31:0] dividend=32'd0;
	reg [31:0] divisor =32'd0;
	wire [31:0] quotient ;
	wire [31:0] remainder ;
	wire division_running;
   	reg reset_divider;
   	wire div_done;

	Divider32bit divider_inst(
   .clk(clk),
   .reset(reset_divider),
   .start_division(division_trig),  // Start signal for division
   .dividend(dividend),       // Dividend input
   .divisor(divisor),        // Divisor input
   .quotient(quotient),   // Quotient output
   .remainder(remainder),  // Remainder output
      .division_active(division_running),   // Declaration of division_active
      .division_done(div_done)
);

	
	reg [DATA_WIDTH-1:0] period_counter=16'd0;	//register for time between peaks
	reg [DATA_WIDTH-1:0] peak_period=16'd0;		//register for last time between peaks
	reg [DATA_WIDTH-1:0] period_speed_reg=16'd0;
	reg [DATA_WIDTH-1:0] prev_period_speed=16'd0;		//save previous speed
	reg [DATA_WIDTH-1:0] peak_level=16'd0;
	reg [DATA_WIDTH-1:0] prev_peak_level=16'd0;
   reg [DATA_WIDTH-1:0]	Kp_max=16'd1;				//temporary Kp for incrementation and find max, init to decimal 1 
	
   //find peaks
	reg autotune_finalized=1'b0;
	reg increasing_flag=1'b0;
	reg Kp_done;
	reg Ki_done;
	reg Kd_done;
	
	always @(posedge clk or posedge reset) begin
    if (reset) begin
		peak_period<=16'd0;
		Kp_done<=1'b0;
		Ki_done<=1'b0;
		Kd_done<=1'b0;
		division_trig <=1'b0;
		dividend<=32'd0;
		divisor<=32'd0;
      	period_counter<=16'd0;
      	period_speed_reg<=16'd0;
      	prev_period_speed<=16'd0;
      	peak_level<=16'd0;
      	prev_peak_level<=16'd0;
      	autotune_finalized<=1'b0;
      	increasing_flag<=1'b0;
      	Kp_max<=16'd1;
      	Kp<=8'b0;
      	Ki<=8'b0;
      	Kd<=7'b0;
      	tuning_done<=1'b0;
      	reset_divider<=1'b1;
    end else
	 begin 
		if(autotune_finalized==1'b0)
			begin
      	prev_period_speed<=period_speed_reg;
		period_speed_reg<=period_speed;
		if(prev_period_speed<period_speed_reg)	//increasing signal
		begin
			increasing_flag<=1'b1;
		end else if (increasing_flag==1'b1 && prev_period_speed>period_speed_reg)	//found peak
		begin
          
          	peak_period<=period_counter;	//replace current period with counted period
          	period_counter<=16'd0;			//TODO: convert to non-blocking type
			prev_peak_level<=peak_level;
			peak_level<=prev_period_speed;	//made prev_period_speed instead of period_speed to get real peak			
          if (prev_peak_level!=16'b0) 
            begin
          if(peak_level==prev_peak_level)	//Found oscillation Kp if peaks are at the same level (?)
			begin
					autotune_finalized<=1'b1;
					
			end else if (peak_level<prev_peak_level)	//decreasing peaks, increment Kp to get closer to max value
			begin
					Kp_max<=Kp_max+1;
			end else begin	//increasing peaks case, unstable Kp has been achieved right after decreasing peaks,
							//decrement for Kpmax 
					Kp_max<=Kp_max-1;
			end
            end
          	Kp<=Kp_max;
          	//prev_peak_flag<=peak_flag;
          
          
          	//prev_peak_flag<=peak_flag;
			//peak_flag<=1'b1;
          	increasing_flag<=1'b0;
		end else begin
          	period_counter<=period_counter+1;		//count period at each clock cycle
        end
		
		 
		
		
		//if(peak_flag==1'b1) begin	//if peak is found, 
		//	
		//	peak_flag<=1'b0;					//reset peak found flag
		//end
		end
		
		else 
			begin
			//TODO:set outputs for different cases
					case (pid_select)
					3'b100: begin //SELECT PROPORTIONAL CONTROL
						Kp<=Kp_max>>1;	//right shift (div by 2) Kpmax to get Kp
						Kp_done<=1'b1;
						Ki<=0;
						Ki_done<=1'b1;
						Kd<=0;
						Kd_done<=1'b1;
					end
					3'b110: begin //SELECT PROPORTIONAL-INTEGRAL CONTROL
						if(Kp_done!=1'b1)
						begin
							//Kp<=Kp_max*45/100;
							if(division_trig==1'b0)
							begin
                              	reset_divider<=1'b1;
								dividend<=45*Kp_max;
								divisor<=100;
								division_trig<=1'b1;
							end else if (division_running==1'b0) begin
								Kp<=quotient;
								division_trig<=1'b0;
								Kp_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
                         	
						end else if (Ki_done!=1'b1)
							begin
								
							//Ki<=54*Kp_max/(peak_period*100);
								if(division_trig==1'b0)
							begin
								dividend<=54*Kp_max;
								divisor<=100*peak_period;
								division_trig<=1'b1;
							end else if (division_running==1'b0) begin
								Ki<=quotient;
								division_trig<=1'b0;
								Ki_done<=1'b1;
							end
							end
						Kd<=0;
						Kd_done<=1'b1;
					end
					3'b111: begin //SELECT PROPORTIONA-INTEGRAL-DERIVATIVE CONTROL
						if(Kp_done!=1'b1)
						begin
							//Kp<=Kp_max*6/10;
							if(division_trig==1'b0)
							begin
                              	reset_divider<=1'b1;
								dividend<=6*Kp_max;
								divisor<=10;
								division_trig<=1'b1;
                            end else if (div_done==1'b1) begin
								Kp<=quotient;
								division_trig<=1'b0;
								Kp_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
							end else if (Ki_done!=1'b1)
							begin
								
							//Ki<=12*Kp_max/(peak_period*10);
								if(division_trig==1'b0)
							begin
                              	
                              	reset_divider<=1'b1;
								dividend<=12*Kp_max;
								divisor<=10*peak_period;
								division_trig<=1'b1;
                            end else if (div_done) begin
								Ki<=quotient;
								division_trig<=1'b0;
								Ki_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
							end else if (Kd_done!=1'b1)
							begin
								//Kd<=3*Kp_max*peak_period/40;
								if(division_trig==1'b0)
							begin
                              	reset_divider<=1'b01;
                              	dividend<=(3*Kp_max*peak_period)>>2;
								divisor<=10;
								division_trig<=1'b1;
							end else if (div_done) begin
                              	
								Kd<=quotient;
								division_trig<=1'b0;
								Kd_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
							end
					end
					default:begin
						Kp<=0;	//STOP PID operation in case of invalid input
						Ki<=0;
						Kd<=0;
						Kp_done<=1'b1;
						Ki_done<=1'b1;
						Kd_done<=1'b1;
					end
					endcase
					tuning_done<=Kp_done & Ki_done & Kd_done;
            end		
    	end
	end
	

endmodule 




 module pid_tuner #(
  parameter DATA_WIDTH = 16, //only used at the 3rd subroutine
  parameter ENCODER_WIDTH = 3 //stop using it
)(
  input wire clk,              		// clock input. when there is no clk signal motor does not run, CLK @50MHz, T=20ns
  input wire reset,            		// when it is 1 motor does run
  input wire [2:0] pid_select,			//select control type: 100 for P, 110 for PI, 111 for PID (?)
  input wire [DATA_WIDTH-1:0] period_speed,//system output of speed measured in clock cycles
  output reg [DATA_WIDTH/2-1:0] Kp,//proportional constant
  output reg [DATA_WIDTH/2-1:0] Ki,//integral constant
  output reg [DATA_WIDTH/2-1:0] Kd//derivative constant
  
);
	
	reg [DATA_WIDTH-1:0] period_counter=16'd0;	//register for time between peaks
	reg [DATA_WIDTH-1:0] peak_period=16'd0;		//register for last time between peaks
	reg [DATA_WIDTH-1:0] prev_peak_period=16'd0;	//register for previous time between peaks
	reg [DATA_WIDTH-1:0] period_speed_reg=16'd0;
	reg [DATA_WIDTH-1:0] prev_period_speed=16'd0;		//save previous speed
	reg [DATA_WIDTH-1:0] peak_level=16'd0;
	reg [DATA_WIDTH-1:0] prev_peak_level=16'd0;
   	reg [DATA_WIDTH/2-1:0]	Kp_max=8'd1;				//temporary Kp for incrementation and find max, init to decimal 1 
	
   //find peaks
	reg autotune_finalized=1'b0;
   	reg instability_flag=1'b0;
	reg increasing_flag=1'b0;
	reg decreasing_peaks_flag=1'b1;
	reg peak_flag=1'b0;
	
	always @(posedge clk or posedge reset) begin
    if (reset) begin
		peak_period<=16'd0;
		prev_peak_period<=16'd0;
    end else begin
      prev_period_speed<=period_speed_reg;
		period_speed_reg<=period_speed;
		if(prev_period_speed<=period_speed_reg)	//increasing signal
		begin
			increasing_flag<=1'b1;
		end else if (increasing_flag==1'b1 && prev_period_speed>=period_speed_reg)	//found peak
		begin
			peak_flag<=1'b1;
		end
		
		
		period_counter<=period_counter+1;		//count period at each clock cycle
		
		if(peak_flag==1'b1) begin	//if peak is found, 
			prev_peak_period<=peak_period;//replace last period with past current
			peak_period<=period_counter;	//replace current period with counted period
			prev_peak_level<=peak_level;
			peak_level<=period_speed;
			peak_flag<=1'b0;					//reset peak found flag
		end
    end
  end
  
	always@(negedge peak_flag)
	begin
		if(autotune_finalized==1'b0) begin	//if autotune isn't final
		if(peak_level==prev_peak_level || instability_flag==1'b1)	//Found oscillation Kp if peaks are at the same level (?)
		begin
				autotune_finalized<=1'b1;
				//TODO:set outputs for different cases
				Kp<=Kp_max>>1;	//right shift (div by 2) Kpmax to get Kp
				Ki<=0;
				Kd<=0;
		end else if (peak_level<prev_peak_level)	//decreasing peaks, increment Kp to get closer to max value
		begin
				Kp_max<=Kp_max+1;
		end else begin	//increasing peaks case, unstable Kp has been achieved right after decreasing peaks,
						//decrement for Kpmax 
				Kp_max<=Kp_max-1;
		end
		end
		
	end
	

endmodule

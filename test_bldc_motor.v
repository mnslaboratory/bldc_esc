 module test_bldc_motor #(//MOTOR EMULATION MODULE
  parameter DATA_WIDTH = 16
)(
  input wire clk,              		// clock input. when there is no clk signal motor does not run
  input wire reset,            		// when it is 1 motor does run
  output reg encoder_a,  				// encoder A pin output
  output reg encoder_b,  				// encoder B pin output
  input wire motor_positive,   		// BLDC motor PWM signal
  input wire motor_negative 			// Motor direction control (1 bit)
  
);

//Infer PWM Duty Cycle
	reg [DATA_WIDTH-1:0] pwm_counter;

	reg [DATA_WIDTH-1:0] pwm_duty_cycle_ones;
	reg [DATA_WIDTH-1:0] duty_cycle;
	reg[1:0] count_direction;
	
	always @(posedge clk or posedge reset or posedge motor_positive or posedge motor_negative) 
	begin
		if(reset) begin
			count_direction<=2'b00;
		end else begin
			duty_cycle<=(100-pwm_duty_cycle_ones*100/pwm_counter)*1+10;//suspicious division operation
			pwm_counter<={DATA_WIDTH-1{1'b0}};
			pwm_duty_cycle_ones<={DATA_WIDTH-1{1'b0}};
			if(motor_positive) begin
				count_direction<=2'b10;//forward motion
			end else if(motor_negative) begin
				count_direction<=2'b01;//backward motion
			end else begin
				count_direction<=2'b00;//default case when turning without encoders triggering or stopping motor
			end
			
		end
		
	end
	
	always @(posedge clk or posedge reset) 
	begin
		if(reset==1'b1) begin //reset clk counters when reset or at the end when pwm=100% is given
			pwm_counter<={DATA_WIDTH-1{1'b0}};
			pwm_duty_cycle_ones<={DATA_WIDTH-1{1'b0}};
		end else begin
			case (count_direction)//count pwm with the direciton given
        2'b01:begin
          pwm_counter<=pwm_counter+1;
			 if(motor_negative==1'b1) begin	//count from negative pole
				pwm_duty_cycle_ones<=pwm_duty_cycle_ones+1;
			 end
        end
        2'b10:begin
          pwm_counter<=pwm_counter+1;		
			 if(motor_positive==1'b1) begin	//count from positive pole
				pwm_duty_cycle_ones<=pwm_duty_cycle_ones+1;
			 end
        end
        default: begin
				pwm_counter<={DATA_WIDTH-1{1'b0}};	//reset in any other case
				pwm_duty_cycle_ones<={DATA_WIDTH-1{1'b0}};
        end
      endcase
		end
		
	end
	
	reg [DATA_WIDTH-1:0] fsm_ctr;
	
	always @(posedge clk or posedge reset) 
	begin
		if(reset) begin
			fsm_ctr<={DATA_WIDTH-1{1'b0}};
		end else begin
			if(fsm_ctr==duty_cycle) begin
				fsm_ctr<={DATA_WIDTH-1{1'b0}};//reset fsm each period
			end else begin
				case(fsm_ctr)
        duty_cycle-3:begin	//01 encoder state sent first
				encoder_a<=1'b0;
				encoder_a<=1'b1;
        end
        duty_cycle-2:begin	//11 encoder state sent second
				encoder_a<=1'b1;
				encoder_b<=1'b1;
        end
        duty_cycle-1:begin	//10 encoder state sent third
				encoder_a<=1'b1;
				encoder_b<=1'b0;
        end
        default: begin
				encoder_a<=1'b0;//00 state in between turns or not turning
				encoder_a<=1'b0;
        end
      endcase
			
			
				fsm_ctr<=fsm_ctr+1;//increment state counter
				
			end
		end
		
	end
	

endmodule

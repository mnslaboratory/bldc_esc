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
	reg signed[DATA_WIDTH-1:0] pwm_counter;

	reg signed[DATA_WIDTH-1:0] pwm_duty_cycle_ones;
   reg signed[DATA_WIDTH-1:0] prev_motor_period;
	reg signed[DATA_WIDTH-1:0] motor_period;
   reg signed[DATA_WIDTH-1:0] ideal_period;
	reg[1:0] count_direction;
	
	always @(posedge motor_positive or posedge motor_negative) begin
		if(motor_positive==1'b1) begin
				count_direction<=2'b10;//forward motion
			end else if(motor_negative==1'b1) begin
				count_direction<=2'b01;//backward motion
			end else begin
				count_direction<=2'b00;//default case when turning without encoders triggering or stopping motor
			end
      ideal_period<=(100-pwm_duty_cycle_ones*100/pwm_counter)*2+20;
      if((100-pwm_duty_cycle_ones*100/pwm_counter)*2+20>16'd10+prev_motor_period) begin
        		
      			prev_motor_period<=motor_period;
        		motor_period<=motor_period+16'd10;
      end else if (prev_motor_period>16'd10+(100-pwm_duty_cycle_ones*100/pwm_counter)*2+20) begin
        		prev_motor_period<=motor_period;
        		motor_period<=motor_period-16'd10;
      end else begin
        		prev_motor_period<=motor_period;
        		motor_period<=(100-pwm_duty_cycle_ones*100/pwm_counter)*2+20 -2;
      end
			pwm_counter<={DATA_WIDTH{1'b0}};
			pwm_duty_cycle_ones<={DATA_WIDTH{1'b0}};
      
	end
	always @(posedge clk or posedge reset) 
	begin
		if(reset==1'b1) begin //reset clk counters when reset or at the end when pwm=100% is given
			pwm_counter<={DATA_WIDTH{1'b0}};
			pwm_duty_cycle_ones<={DATA_WIDTH{1'b0}};
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
				pwm_counter<={DATA_WIDTH{1'b0}};	//reset in any other case
				pwm_duty_cycle_ones<={DATA_WIDTH{1'b0}};
        end
      endcase
		end
		
	end
	
	reg [DATA_WIDTH-1:0] fsm_ctr;
	
	always @(posedge clk or posedge reset) 
	begin
		if(reset) begin
			fsm_ctr<={DATA_WIDTH{1'b0}};
          	encoder_a<=1'b0;
          	encoder_b<=1'b0;
		end else begin
          if(fsm_ctr>=motor_period) begin
				fsm_ctr<={DATA_WIDTH{1'b0}};//reset fsm each period
			end else begin
			if (count_direction==2'b10) begin
              case(fsm_ctr)
        motor_period-3:begin	//01 encoder state sent first
				encoder_a<=1'b0;
				encoder_b<=1'b1;
        end
        motor_period-2:begin	//11 encoder state sent second
				encoder_a<=1'b1;
				encoder_b<=1'b1;
        end
        motor_period-1:begin	//10 encoder state sent third
				encoder_a<=1'b1;
				encoder_b<=1'b0;
        end
        default: begin
				encoder_a<=1'b0;//00 state in between turns or not turning
				encoder_b<=1'b0;
        end
      endcase
		end else begin
		case(fsm_ctr)
        motor_period-3:begin	//01 encoder state sent first
				encoder_a<=1'b1;
				encoder_b<=1'b0;
        end
        motor_period-1:begin	//11 encoder state sent second
				encoder_a<=1'b1;
				encoder_b<=1'b1;
        end
        motor_period:begin	//10 encoder state sent third
				encoder_a<=1'b0;
				encoder_b<=1'b1;
        end
        default: begin
				encoder_a<=1'b0;//00 state in between turns or not turning
				encoder_b<=1'b0;
        end
      endcase
		
		end
		
			
			
				fsm_ctr<=fsm_ctr+1;//increment state counter
				
			end
		end
		
	end
	

endmodule

 module bldc_esc#(
  parameter DATA_WIDTH = 16, //only used at the 3rd subroutine
  parameter ENCODER_WIDTH = 3 //stop using it
)(
  input wire clk,              		// clock input. when there is no clk signal motor does not run
  input wire reset,            		// when it is 1 motor does run
  input wire encoder_a,  				// encoder A pin input
  input wire encoder_b,  				// encoder B pin input
  input [DATA_WIDTH/2-1:0] period_reference,	// period_reference speed input, commented out since the ambiguity, will try with constant 900RPM period_reference speed
  input wire [DATA_WIDTH/2-1:0] Kp_ext,//External proportional constant
  input wire [DATA_WIDTH/2-1:0] Ki_ext,//External integral constant
  input wire [DATA_WIDTH/2-1:0] Kd_ext,//External derivative constant
  input wire override_internal_pid,		//select pin for external/internal pid constants
  //output reg locked,					//uncertain output pin for "locked" -accurate to a margin- output
  output reg motor_positive,   		// BLDC motor PWM signal
  output reg motor_negative 			// Motor direction control (1 bit)
  
);
  
  reg motor_pwm;
   parameter PWM_PERIOD = {16'd15}; //change period here e.g. 3,4,5,6
   reg [DATA_WIDTH-1:0] pwm_counter;

  reg [DATA_WIDTH-1:0] pwm_duty_cycle;
  // Ask encoder's current and previous states in the 2nd subroutine
  reg [1:0]encoder_state;
  reg [1:0]prev_encoder_state;
	reg [1:0]pwm_direction;
	//locked=1'b1;
  //assign pwm_duty_cycle = pid_output; // BU BAĞLANTI ÇOK ÖNEMLİ ÇALIŞMIYOR OLABİLİR.
  //assign reg ile çalışmıyor, eşitliğin always içinde yaratılması gerek.
//1st subroutin=PWM GENERATOR////////////////////////////////////////////// 
 always @(posedge clk or posedge reset) begin
    if (reset) begin
      pwm_counter <= {DATA_WIDTH{1'b0}};
    end else begin
      pwm_counter <= pwm_counter + 1;

      motor_pwm <= (pwm_counter < pwm_duty_cycle); // Generate PWM signal and send it to the motor

      // Reset PWM counter at the end of the PWM period
      if (pwm_counter == PWM_PERIOD) begin
        pwm_counter <= 16'd0;
      end
    end
  end
 
///////////////////////////////////////////////////////
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      encoder_state <= 2'b0;
      prev_encoder_state <= 2'b0;
      motor_positive <= 1'b0;
      motor_negative <= 1'b0;
      pwm_direction<=2'b00;
		
    end else begin
    encoder_state <= {encoder_a, encoder_b};
      case ({encoder_state, prev_encoder_state})

      // Rotate in one direction (e.g., forward)
      4'b0100, 4'b1101,4'b1011: begin
        pwm_direction<=2'b10;
      end

      // Rotate in the opposite direction (e.g., reverse)
      4'b1000,4'b1110,4'b0111: begin
        pwm_direction<=2'b01;
      end

      // No change (no rotation)
      4'b1100, 4'b0011, 4'b1001,4'b0110: begin
        pwm_direction<=2'b00;
      end

      default: begin
        // Keep the previous direction in case of an unexpected state
        pwm_direction<=pwm_direction;
      end
    endcase
	 
      prev_encoder_state <= encoder_state;
      case(pwm_direction)
        2'b00: begin
          
      	motor_positive <= 1'b0;
      	motor_negative <= 1'b0;
        end
        2'b01:begin
          motor_positive<=1'b0;
          motor_negative<=motor_pwm;
        end
        2'b10:begin
          motor_positive<=motor_pwm;
          motor_negative<=1'b0;
        end
        default: begin
          
      		motor_positive <= 1'b0;
      		motor_negative <= 1'b0;
        end
      endcase
    end
  end
//PID control////////////////////////////////////////////////
 // PID constants
   
   reg countspeed_en;
   reg [DATA_WIDTH-1:0] period_speed = 16'h0;// Counter connected to rise of encoder A that will infer speed by measuring period of clk cycles
  reg [DATA_WIDTH-1:0] speed_ctr;
  reg [DATA_WIDTH/2-1:0] Kp; // was = {DATA_WIDTH/2{1'b0}};
  reg [DATA_WIDTH/2-1:0] Ki; // was = {DATA_WIDTH/2{1'b0}};
  reg [DATA_WIDTH/2-1:0] Kd; // was = {DATA_WIDTH/2{1'b0}};
  
  //MUX Internal/External PID constants
  always @(posedge clk or posedge reset) begin
    if (reset) begin
		Kp <= {DATA_WIDTH/2{1'b0}}; //   was     Kp <= {DATA_WIDTH/2{1'b1}};
		Ki <= {DATA_WIDTH/2{1'b0}};
		Kd <= {DATA_WIDTH/2{1'b0}};
    end else begin
		if(override_internal_pid) begin
			Kp<=Kp_ext;
			Ki<=Ki_ext;
			Kd<=Kd_ext;
		end else begin
			Kp<=Kp;
			Ki<=Ki;
			Kd<=Kd;
		end
	 end
  end
  
  
	

  // Variables for PID control
  reg [DATA_WIDTH/2-1:0] error;
  reg [(DATA_WIDTH)/2-1:0] integral;
  reg [DATA_WIDTH/2-1:0] derivative;
  reg [(DATA_WIDTH)-1:0] pid_output;
  reg [DATA_WIDTH/2-1:0] previous_error;
	//Calculate Speed
	//dubious logic of carrying out speed<->clock cycle conversion:
  //We will give constant period_reference of 900RPM=>15 rounds each second, with clock period t=1ms, period_reference=15e3 clock cycles
	always @(posedge reset or posedge clk)
	begin
		if(reset) begin
			countspeed_en<=1'b0;
			speed_ctr<={DATA_WIDTH{1'b0}};
		end
		else begin
			if(prev_encoder_state[0]==1'b0 && encoder_a==1'b1) begin
				period_speed<=speed_ctr;
				speed_ctr<={DATA_WIDTH{1'b0}};
			end else begin
				speed_ctr<=speed_ctr+1;
			end
		end
	end
  
  
  // Calculate the error
  always @(*) begin
	if(reset) begin
	error=8'b0;
	end else begin
    error = period_reference - period_speed[7:0]; // was error = period_reference - period_speed; there is a problem bcs
	 end                                          // error waits 8 bit value but period_speed is 16 bit
  end                                             // I just select first 8 bit to try on OpenLane as dummy selective

  // Calculate the integral
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      integral <= 8'b0;
		previous_error <= {DATA_WIDTH/2{1'b0}};
    end else begin
      if (integral + error > 127) begin //These are very high values for ASIC, need to push for lower register(?)

        integral <= 127;  
      end else if (integral + error < -128) begin
      
        integral <= -128; 
      end else begin
        integral <= integral + error;  
      end
    end
  end

  // Calculate the derivative
  always @(*) begin
    derivative = error - previous_error;
  end

  // Calculate the PID output
  always @(*) begin
    pid_output = Kp * error + Ki * integral + Kd * derivative;
	 
	 pwm_duty_cycle = pid_output;
  end
  
 

endmodule

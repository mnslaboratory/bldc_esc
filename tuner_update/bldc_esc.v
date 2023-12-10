module bldc_esc_1 #(parameter DATA_WIDTH = 16,parameter debounce = 3)(
  input clk,              		// clock input. when there is no clk signal motor does not run
  input reset,            		// when it is 1 motor does run active high
  input tuner_reset,
  input pwm_en,						//Pin to enable pwm output	active high
  input [2:0] autotune_select,
  input encoder_a,  				// encoder A pin input
  input encoder_b,  				// encoder B pin input
  input [DATA_WIDTH-1:0] pwm_period,	//change period in clock cycles
  input [DATA_WIDTH-1:0] period_reference,	// period_reference speed input, commented out since the ambiguity, will try with constant 900RPM period_reference speed
  input [7:0] Kp_ext,//External proportional constant
  input [7:0] Ki_ext,//External integral constant
  input [6:0] Kd_ext,//External derivative constant
  input override_internal_pid,		//select pin for external/internal pid constants
  output reg motor_positive,   		// BLDC motor PWM signal
  output reg motor_negative 	    // Motor direction control (1 bit)
);
	reg [DATA_WIDTH-1:0] period_speed ;// Counter connected to rise of encoder A that will infer speed by measuring period of clk cycles
	wire tuning_done;
	reg [DATA_WIDTH-1:0] Kp_int;
	reg [DATA_WIDTH-1:0] Ki_int;
	reg [DATA_WIDTH-1:0] Kd_int;
pid_tuner tuner_inst_1(
	.clk(clk),
	.reset(reset | tuner_reset),
	.pid_select(autotune_select),
	.period_speed(period_speed),
	.tuning_done(tuning_done),
	.Kp(Kp_int),	//internal registers are read
	.Ki(Ki_int),
	.Kd(Kd_int)

);
  reg motor_pwm;
  reg [DATA_WIDTH-1:0] pwm_counter;
  reg [DATA_WIDTH-1:0] pwm_duty_cycle;
  reg [1:0]encoder_state;
  reg [1:0]prev_encoder_state;
  reg [1:0]pwm_direction;
  //PID control////////////////////////////////////////////////
  // PID constants
  reg [DATA_WIDTH-1:0] speed_ctr;
  reg [7:0] Kp ;
  reg [7:0] Ki ;
  reg [6:0] Kd ;
  // Variables for PID control
  reg signed[DATA_WIDTH-1:0] error;
  reg signed[(DATA_WIDTH)-1:0] integral;
  reg signed[DATA_WIDTH-1:0] derivative;
  reg signed[(DATA_WIDTH)-1:0] pid_output;
  reg signed[DATA_WIDTH-1:0] previous_error;
  
  reg [15:0] clk_counter ;
  reg [2:0] encoder_a_shift_reg;
  reg encoder_a_reg;
  reg [2:0] encoder_b_shift_reg;
  reg encoder_b_reg;
  reg [2:0] pwm_en_shift_reg;
  reg pwm_en_reg;
  reg flag;
  reg counter_rst;
  reg [1:0]encoder_a_set;
 always @(posedge clk) begin
    if (reset) begin
        clk_counter <= 16'b0;
        counter_rst <= 1'b1;
        encoder_a_set <= 2'b0;
        flag <= 1'b0;
        integral <= 16'b0;
        pwm_counter <= {DATA_WIDTH{1'b0}};
        encoder_state <= 2'b0;
        prev_encoder_state <= 2'b0;
        pwm_direction<=2'b00;
        motor_positive<=1'b0;
        motor_negative<=1'b0;
        Kp <= 8'h1;
        Ki <= 8'd0;
        Kd <= 7'd0;
        speed_ctr<={DATA_WIDTH{1'b0}};
        previous_error <= {DATA_WIDTH{1'b0}};
        error<=16'b0;
        period_speed <= 16'b0;
        pid_output <= 16'b0;
        derivative <= 16'b0;
        pwm_duty_cycle <= 16'b0;
        motor_pwm <= 1'b0;
        
        encoder_a_shift_reg <= 3'b0;
        encoder_a_reg <= 1'b0;
        encoder_b_shift_reg <= 3'b0;
        encoder_b_reg <= 1'b0;
        pwm_en_shift_reg <= 3'b0;
        pwm_en_reg <= 1'b0;
    end 
    else begin
        if(clk_counter == 16'h0040)begin
            clk_counter <= 16'b0;
            
            pwm_en_shift_reg <= {pwm_en_shift_reg[debounce-2:0],pwm_en};
            if (pwm_en_shift_reg == {debounce{1'b0}} || pwm_en_shift_reg == {debounce{1'b1}})begin
                pwm_en_reg <= pwm_en_shift_reg[0];
            end
            
            encoder_a_shift_reg <= {encoder_a_shift_reg[debounce-2:0],encoder_a};
            if (encoder_a_shift_reg == {debounce{1'b0}} || encoder_a_shift_reg == {debounce{1'b1}})begin
                encoder_a_reg <= encoder_a_shift_reg[0];
            end
            
            encoder_b_shift_reg <= {encoder_b_shift_reg[debounce-2:0],encoder_b};
            if (encoder_b_shift_reg == {debounce{1'b0}} || encoder_b_shift_reg == {debounce{1'b1}})begin
                encoder_b_reg <= encoder_b_shift_reg[0];
            end
            
            pid_output <= (Kp * error) + (Ki * integral)>>3 + (Kd * derivative);	//compute pid response
            if(pid_output<1) begin			//if pid output is negative, cap at 0
                pwm_duty_cycle<=pwm_period;
            end 
            else if (pid_output>pwm_period) begin	//if pid output above period, give 100% pwm
                pwm_duty_cycle<=pwm_period>>2;
            end
            else begin
                pwm_duty_cycle<=pid_output;	//if in between values, give pid output as pwm
            end
            
            derivative <= error - previous_error;
            if (integral + error > 2048) begin //These are very high values for ASIC, need to push for lower register(?)
                integral <= 2047;
            end
            else if (integral + error < -2048) begin
                integral <= -2047;
            end
            else begin
                integral <= integral + error;
            end
            
            previous_error<=error;
            error <= period_reference - period_speed;
            motor_pwm <= (pwm_counter < pwm_duty_cycle) & pwm_en_reg; // Generate PWM signal and send it to the motor
            // Reset PWM counter at the end of the PWM period
            if (pwm_counter == pwm_period) begin
                pwm_counter <= 16'd0;
            end
            else begin
                pwm_counter <= pwm_counter + 1;
            end
            encoder_state <= {encoder_a_reg, encoder_b_reg};
            prev_encoder_state <= encoder_state;
            if(pwm_en_reg)begin
                case ({encoder_state, prev_encoder_state})
                    4'b0100, 4'b1101,4'b1011: begin// Rotate in one direction (e.g., forward)
                        pwm_direction<=2'b10;
                    end
                    4'b1000,4'b1110,4'b0111: begin// Rotate in the opposite direction (e.g., reverse)
                        pwm_direction<=2'b01;
                    end
                    4'b1100, 4'b0011, 4'b1001,4'b0110: begin// No change (no rotation)
                        pwm_direction<=2'b00;
                    end
                    default: begin
                        pwm_direction<=pwm_direction;// Keep the previous direction in case of an unexpected state
                    end
                endcase
            end
//            else begin
//                encoder_state <= 2'b0;
//                prev_encoder_state <= 2'b0;
//            end
            
            if(!flag && (pwm_period!=0))begin
                motor_positive <= (period_reference<32767) ? (1'b1) : (1'b0);
                motor_negative <= (period_reference>=32767) ? (1'b1) : (1'b0);
                flag <= ((period_speed>=150) && (period_reference>=period_speed)) ? (1'b1): (1'b0);
            end
            if(pwm_en_reg==1'b0)begin
                motor_positive<=1'b0;
                motor_negative<=1'b0;
            end
            else if (flag)begin
                case(pwm_direction)
                    2'b00: begin
                        if(period_reference>32767) begin //check negative references by 2's complement
                            motor_positive<=1'b0;
                            motor_negative<=motor_pwm;
                        end
                        else begin
                            motor_positive<=motor_pwm;
                            motor_negative<=1'b0;
                        end
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
            if(override_internal_pid) begin
                Kp<=Kp_ext;
                Ki<=Ki_ext;
                Kd<=Kd_ext;
            end 
		else if (tuning_done) begin
			Kp<=Kp_int;
			Ki<=Ki_int;
			Kd<=Kd_int;
		end
		else begin
			Kp<=Kp_int;
			Ki<=Ki;
			Kd<=Kd;
		end
            
            if(encoder_a_reg && encoder_a_set==2'b00)begin
              counter_rst <= 1'b0;
              encoder_a_set <= 2'b01;
            end
            else if(encoder_a_set==2'b01 && !encoder_a_reg)begin
              encoder_a_set <= 2'b10;
            end
            else if (encoder_a_reg && encoder_a_set==2'b10)begin
              counter_rst <= 1'b1;
              encoder_a_set <= 2'b00;
              period_speed<=speed_ctr;
            end
            
            if(counter_rst)begin
                speed_ctr <= 16'b0;
            end
            else begin
                speed_ctr<=speed_ctr+1;
            end
        end
        else begin
            clk_counter <= clk_counter + 1;
        end
    end
 end
endmodule

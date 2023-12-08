module esc_1#(parameter depth = 20)(
    input clk,
    input rst,
    // I2C Ports
    inout sda,
    input scl,
    output sda_enable,
    // 
//    output [2:0]fsm_wire_t,
//    output i2c_restart,
    //
    input pwm_en,
    input encoder_a,
    input encoder_b,
    output motor_positive,
    output motor_negative
    );
    // PID inputs
    wire [15:0] pwm_period_wire;
    wire [15:0] period_reference_i2c_wire;
    wire [15:0] Kp_ext_i2c_wire;
    wire [15:0] Ki_ext_i2c_wire;
    wire [15:0] Kd_ext_i2c_wire;
    wire [15:0] override_internal_pid_i2c_wire;
    
    wire busy_wire;
    wire valid_wire;
    wire [7:0] data_out_i2c;
    wire [7:0] data_out_ram;
    wire write_i2c;
    wire read_1_i2c;
    wire [7:0] index_1_i2c;
//    wire [2:0] fsm_wire_top;
//    wire slow_clk;
    
//    assign fsm_wire_t = fsm_wire_top;
//    assign i2c_restart = i2c_restart2;
//    wire i2c_restart2;
    
//    Divided_Clock Slow_Clock(
//        . clk(clk),
//        . rst(rst),
//        . div_clk(slow_clk)
//    );
    
    I2C_SLAVE_1 I2C_SLAVE(
        . clk(clk),
        . rst(rst),
        // I2C Ports
        . sda(sda),
        . scl(scl),
        . sda_enable(sda_enable),
        // RAM control signals
        . write(write_i2c),
        . read_1(read_1_i2c),
        . index_1(index_1_i2c),
        . data_out(data_out_i2c),
        . data_in(data_out_ram),
        // control signals
//        . fsm_wire(fsm_wire_top),
//        . restart_o(i2c_restart2),
        . busy(busy_wire),
        . valid(valid_wire)
    );
    
    Register_Module_1 RAM_BLOCK(
        . clk(clk),
        . rst(rst),
        . write(write_i2c),
        . read_1(read_1_i2c),
        . index_1(index_1_i2c),
        . data_in(data_out_i2c),
        . data_out_1(data_out_ram),
        //
        . pwm_period(pwm_period_wire),           // 0x40-0x41
        . period_reference(period_reference_i2c_wire),  //0x40
        . Kp_ext(Kp_ext_i2c_wire),            //0x41
        . Ki_ext(Ki_ext_i2c_wire),            //0x42
        . Kd_ext(Kd_ext_i2c_wire),            //0x43
        . override_internal_pid(override_internal_pid_i2c_wire[0])   //0x44
    );
    
    bldc_esc_1 BLDC_ESC(
      . clk(clk),
      . reset(rst),
      . pwm_en(pwm_en),
      . encoder_a(encoder_a),
      . encoder_b(encoder_b),
      . pwm_period(pwm_period_wire),
      . period_reference(period_reference_i2c_wire),
      . Kp_ext(Kp_ext_i2c_wire),
      . Ki_ext(Ki_ext_i2c_wire),
      . Kd_ext(Kd_ext_i2c_wire),
      . override_internal_pid(override_internal_pid_i2c_wire[0]),
      . motor_positive(motor_positive),
      . motor_negative(motor_negative)
    );
endmodule
module I2C_SLAVE_1#(parameter debounce = 3)(
    input clk,
    input rst,
    // I2C Ports
    input scl,
    inout sda,
    output sda_enable,
    // RAM control signals 
    output write,
    output read_1,
    output reg [7:0] index_1,
    output [7:0] data_out,
    input [7:0] data_in,
    // control signals
//    output [2:0] fsm_wire,
//    output restart_o,
    output reg busy,
    output reg valid
    );
    
    assign write = (!data_will_send && valid && !send_operation) ? (1'b1) : (1'b0);
    assign data_out = temp_data;
    
    assign read_1 = ((data_will_send && bit_count==5'd18) || (data_will_send && bit_count==5'd9)) ? (1'b1) : (1'b0);
//    assign fsm_wire = fsm_state;
//    assign restart_o = restart;
    assign sda_enable = sda_en;
    
    localparam TIME_THDSTA  =30;         //(0x6us/clock);     // 0.6 us 29 
    localparam TIME_TLOW    =30;        //(0x6us/clock);    // 1.3 us 63
    localparam I2CBITS      =29;
    
    
    reg [7:0] I2C_SLAVE_ADDR;
    
    reg [7:0] i2c_data;
    reg [28:0] i2c_capt;
    
    reg [31:0]counter;
    reg counter_reset;
    reg send_operation;
    
    reg [4:0] bit_count;
    reg [7:0] temp_data;
    reg capture_en;
    
    reg ack_sended;
    reg nack_sended;
    reg half_ok;
    reg data_will_send;
    reg received_one;
    
    reg sda_en;
    reg scl_en;
    reg done_high;
    
    reg [31:0] t_high;
    wire [31:0] t_high_2 = t_high>>1;
    reg [31:0] t_low;
    wire [31:0] t_low_2 = t_low>>1;
    reg captured;
    reg distance;
    reg sda_high;
    reg restart;
    
    reg scl_reg;
    reg [debounce-1:0] scl_shift_reg;
    reg sda_reg;
    reg [debounce-1:0] sda_shift_reg;
    
    assign sda = (sda_en) ? (1'bz) : (1'b0);
    
    reg [2:0] fsm_state;
    parameter IDLE  = 0;
    parameter START = 1;
    parameter TLOW  = 2;
    parameter THIGH = 3;
    parameter TSTO  = 4;
    
    always @(posedge clk)begin
        if (rst)begin
        
            I2C_SLAVE_ADDR  <= 8'h72;
            temp_data       <= 8'b0;
            index_1         <= 8'b0;
            send_operation  <= 1'b0;
            
            scl_reg         <= 1'b1;
            scl_shift_reg   <= {debounce{1'b1}};
//            scl_shift_reg   <= 3'b111;
            sda_reg         <= 1'b1;
            sda_shift_reg   <= {debounce{1'b1}};
            
            scl_en          <= 1'b0;
            sda_en          <= 1'b1;
            counter_reset   <= 1'b0;
            counter         <= 32'b0;
            bit_count       <= 5'b0;
            captured        <= 1'b0;
            distance        <= 1'b0;
            sda_high        <= 1'b0;
            restart         <= 1'b0;
            done_high       <= 1'b0;
            
            t_high          <= 32'b0;
            t_low           <= 32'b0;
            
            fsm_state       <= 3'b0;
            busy            <= 1'b0;
            valid           <= 1'b0;
            data_will_send  <= 1'b0;
            received_one    <= 1'b0;
            
            i2c_data        <= 8'b0;
            i2c_capt        <= 29'b0;
            
            ack_sended      <= 1'b0;
            nack_sended     <= 1'b0;
            half_ok         <= 1'b0;
        end
        else begin
        
            scl_shift_reg <= {scl_shift_reg[debounce-2:0],scl};
            
            if (scl_shift_reg == {debounce{1'b0}} || scl_shift_reg == {debounce{1'b1}})begin
                scl_reg <= scl_shift_reg[0];
            end
        
            sda_shift_reg <= {sda_shift_reg[debounce-2:0],sda};
            
            if (sda_shift_reg == {debounce{1'b0}} || sda_shift_reg == {debounce{1'b1}})begin
                sda_reg <= sda_shift_reg[0];
            end
            
            capture_en <= i2c_capt[I2CBITS - bit_count - 1];
            
            if((data_will_send && !received_one && (bit_count==5'd18)) || (data_will_send && !received_one && (bit_count==5'd9) && restart))begin
                i2c_data <= data_in;
                received_one <= 1'b1;
            end
            if(counter_reset)begin
                counter <= 32'b0;
                counter_reset <= 1'b0;
            end
            else begin
                counter <= counter + 1'b1;
            end
            case (fsm_state)
                IDLE:begin
                    sda_en <= 1'b1;
                    if ((counter == (t_high + t_low)) || distance )begin
                        i2c_capt        <= {1'b0, 8'hFF,   1'b0, 8'hFF,    1'b0, 8'hFF,  1'b0, 1'b0};
                        send_operation  <= 1'b0;
                        bit_count       <= 5'b0;
                        ack_sended      <= 1'b0;
                        nack_sended     <= 1'b0;
                        received_one    <= 1'b0;
                        distance        <= 1'b1;
                        busy            <= 1'b0;
                        sda_high        <= 1'b0;
                        restart         <= 1'b0;
                        valid           <= 1'b0;
                        captured        <= 1'b0;
                        counter_reset   <= (scl_reg && !sda_reg) ? (1'b0) : (1'b1);
                        fsm_state       <= (scl_reg && !sda_reg) ? (START) : (IDLE);
                    end
                end
                START:begin
                    distance <= 1'b0;
                    done_high <= 1'b0;
                    if(!scl_reg)begin
                        if(counter >= TIME_THDSTA)begin
                            fsm_state <= TLOW;
                            busy <= 1'b1; 
                            t_high <= (restart) ? (t_high) : (counter);
                            counter_reset <= 1'b1;
                        end
                        else begin
                            fsm_state <= IDLE;
                            counter_reset <= 1'b1;
                        end
                    end
                    if(sda_reg)begin
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                    end
                end
                TLOW:begin
                    if(scl_reg)begin
                        if(counter >= TIME_TLOW)begin
                            bit_count <= (restart && (bit_count==5'd9)) ? (5'd19) : (bit_count + 1'b1);
//                            fsm_state <= THIGH;
                            fsm_state <= (bit_count == I2CBITS-2) ? (TSTO) : (THIGH);
                            t_low <= counter;
                            captured <= 1'b1;
                            counter_reset <= 1'b1;
                        end
                        else begin
                            fsm_state <= IDLE;
                            counter_reset <= 1'b1;
                        end
                    end
                    if(captured)begin
                        if(counter == t_low_2)begin
                            if((data_will_send && (bit_count>17) && (bit_count<28)) || (data_will_send && (bit_count==5'd9)))begin
                                sda_en  <= (bit_count==5'd9) ? (i2c_data[7]) : (i2c_data[I2CBITS - bit_count - 4]);// problem in here
                            end
                            else if(((bit_count==5'd8) || (bit_count==5'd17) || (bit_count==5'd26)))begin
                                if(ack_sended)begin
                                    sda_en <= 1'b0;
                                end
                                else if(nack_sended)begin
                                    sda_en <= 1'b1;
                                    fsm_state <= IDLE;
                                    counter_reset <= 1'b1;
                                end
                            end
                            else begin
                                sda_en <= 1'b1;
                            end
                        end
                        else if ((counter >= (t_low<<3)) && !counter_reset)begin
                            fsm_state <= IDLE;
                            counter_reset <= 1'b1;
                        end
                    end
                end
                THIGH:begin
                    if(scl_reg && sda_reg && !restart && (bit_count == 5'd19))begin
                        sda_high <= 1'b1;
                    end
                    else if (scl_reg && !sda_reg && sda_high && !restart && (bit_count == 5'd19))begin
                        sda_high <= 1'b0;
                        restart <= 1'b1;
                        fsm_state <= START;
                        half_ok <= 1'b0;
                        bit_count <= 5'b0;
                    end
                    
                    if(!scl_reg && done_high)begin
                        fsm_state <= (TLOW);
                        done_high <= 1'b0;
                    end
                    
                    if((counter == t_high_2) && !done_high )begin
                        if((bit_count==5'd8) && half_ok)begin
                            if(temp_data[7:1]==I2C_SLAVE_ADDR)begin
                                ack_sended <= 1'b1;
                                i2c_capt  <= (!temp_data[0]) ? ({1'b0, 8'hFF,1'b0,   8'hFF,1'b0,   8'hFF,1'b0,   1'b0}): // write
                                                               ({1'b0, 8'hFF,1'b0,   8'hFF,1'b0,   8'h00,1'b0,   1'b0}); // read
                                data_will_send <= (!temp_data[0]) ? (1'b0) : (1'b1);
                                send_operation <= (!temp_data[0]) ? (send_operation) : (1'b1);
                            end
                            else begin
                                nack_sended <= 1'b1;
                            end
                        end
                        else if ((bit_count==5'd17) && half_ok)begin
                            if((temp_data>=8'h40) && (temp_data<=8'h53))begin
                                ack_sended <= 1'b1;
                                index_1 <= temp_data;
                            end
                            else begin
                                nack_sended <= 1'b1;
                            end
                        end
                        else if ((bit_count==5'd26) && half_ok)begin
                            data_will_send <= 1'b0;
                            ack_sended <= 1'b1;
                        end
                        else begin
                            ack_sended <= 1'b0;
                        end
                        
                        if(capture_en && !half_ok)begin
                            temp_data <= (temp_data << 1) | sda_reg;
                        end
                        half_ok <= ~half_ok;
                        done_high <= (half_ok);
                        if (!half_ok)begin
                            fsm_state <= (THIGH);
                        end
                        counter_reset <= 1'b1;
                    end
                    else if ((counter >= (t_high<<3)) && !counter_reset)begin
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                    end
                end
                TSTO:begin
                    
                    if(scl_reg && !sda_reg )begin
                        sda_high <= 1'b1;
                    end
                    else if (scl_reg && sda_reg && sda_high )begin
                        sda_high <= 1'b0;
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                        valid <= 1'b1;
                        busy <= 1'b0;
                    end 
                    if ((counter >= (t_high<<1)) && !counter_reset)begin
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                    end
                end
            endcase
        end
    end
endmodule
module Register_Module_1#(parameter depth = 20)(
    input clk,
    input rst,
    input write,
    input read_1,
    input [7:0] index_1,
    input [7:0] data_in,
    output [7:0] data_out_1,
    //
    output [15:0] pwm_period,           // 0x40-0x41
    output [15:0] period_reference,     // 0x42-0x43
    output [15:0] Kp_ext,               // 0x44-0x45
    output [15:0] Ki_ext,               // 0x46-0x47
    output [15:0] Kd_ext,               // 0x48-0x49
    output override_internal_pid        // 0x50[0]
    );
    
    reg [7:0] internal_register [0:depth-1]; // start from 0x40 
    assign data_out_1 = (read_1) ? (internal_register[index_1[4:0]]) : (8'b0);
    
    assign pwm_period            = {internal_register[0],internal_register[1]};
    assign period_reference      = {internal_register[2],internal_register[3]};
    assign Kp_ext                = {internal_register[4],internal_register[5]};
    assign Ki_ext                = {internal_register[6],internal_register[7]};
    assign Kd_ext                = {internal_register[8],internal_register[9]};
    assign override_internal_pid = {internal_register[10][0]};
    
    always @(posedge clk)begin
        if(rst)begin
            internal_register [0] <= 8'b0;
            internal_register [1] <= 8'b0;
            internal_register [2] <= 8'b0;
            internal_register [3] <= 8'b0;
            internal_register [4] <= 8'b0;
            internal_register [5] <= 8'b0;
            internal_register [6] <= 8'b0;
            internal_register [7] <= 8'b0;
            internal_register [8] <= 8'b0;
            internal_register [9] <= 8'b0;
            internal_register [10] <= 8'b0;
            internal_register [11] <= 8'b0;
            internal_register [12] <= 8'b0;
            internal_register [13] <= 8'b0;
            internal_register [14] <= 8'b0;
            internal_register [15] <= 8'b0;
            internal_register [16] <= 8'b0;
            internal_register [17] <= 8'b0;
            internal_register [18] <= 8'b0;
            internal_register [19] <= 8'b0;
        end
        else begin
            internal_register[index_1[4:0]] <= (write) ? (data_in) : (internal_register[index_1[4:0]]);
        end
    end
endmodule
module bldc_esc_1 #(
  parameter DATA_WIDTH = 16, //only used at the 3rd subroutine
  parameter ENCODER_WIDTH = 3 //stop using it
)(
  input clk,              		// clock input. when there is no clk signal motor does not run
  input reset,            		// when it is 1 motor does run active high
  input pwm_en,						//Pin to enable pwm output	active high
  input encoder_a,  				// encoder A pin input
  input encoder_b,  				// encoder B pin input
  input [DATA_WIDTH-1:0] pwm_period,	//change period in clock cycles
  input [DATA_WIDTH-1:0] period_reference,	// period_reference speed input, commented out since the ambiguity, will try with constant 900RPM period_reference speed
  input [DATA_WIDTH-1:0] Kp_ext,//External proportional constant
  input [DATA_WIDTH-1:0] Ki_ext,//External integral constant
  input [DATA_WIDTH-1:0] Kd_ext,//External derivative constant
  input override_internal_pid,		//select pin for external/internal pid constants
  output reg motor_positive,   		// BLDC motor PWM signal
  output reg motor_negative 			// Motor direction control (1 bit)
);
  
  reg motor_pwm;
   reg [DATA_WIDTH-1:0] pwm_counter;

  reg [DATA_WIDTH-1:0] pwm_duty_cycle;
  // Ask encoder's current and previous states in the 2nd subroutine
  reg [1:0]encoder_state;
  reg [1:0]prev_encoder_state;
	reg [1:0]pwm_direction;
	//locked=1'b1;
  //assign pwm_duty_cycle = pid_output; // BU BA?LANTI ÇOK ÖNEML? ÇALI?MIYOR OLAB?L?R.
  //assign reg ile çal??m?yor, e?itli?in always içinde yarat?lmas? gerek.
//1st subroutin=PWM GENERATOR////////////////////////////////////////////// 
 always @(posedge clk or posedge reset) begin
    if (reset) begin
      pwm_counter <= {DATA_WIDTH{1'b0}};
    end else begin
      pwm_counter <= pwm_counter + 1;

      motor_pwm <= (pwm_counter < pwm_duty_cycle) & pwm_en; // Generate PWM signal and send it to the motor

      // Reset PWM counter at the end of the PWM period
      if (pwm_counter == pwm_period) begin
        pwm_counter <= 16'd0;
      end
    end
  end
 
///////////////////////////////////////////////////////
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      encoder_state <= 2'b0;
      prev_encoder_state <= 2'b0;
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
    end
  end
  
  always @(posedge clk or posedge reset) begin
	if(reset) begin
		motor_positive<=1'b0;
		motor_negative<=1'b0;
	end else begin
		if(period_reference>127) begin
			motor_positive<=1'b0;
			motor_negative<=motor_pwm;
		end else if (period_reference>0) begin
			motor_positive<=motor_pwm;
			motor_negative<=1'b0;
		end else begin
			motor_positive<=1'b0;
			motor_negative<=1'b0;
			end
		end
	
  
  end
  
  //always @(pwm_direction or pwm_en or reset) begin //method to determine motor pins from encoder
  //  if (reset) begin
  //    motor_positive <= 1'b0;
  //    motor_negative <= 1'b0;
	//	
    //end else begin
	 //
		//if(pwm_en==1'b0)begin
		//	motor_positive<=1'b0;
		//	motor_negative<=1'b0;
	//	end else begin
   //   case(pwm_direction)
   //     2'b00: begin
   //       
   //   	motor_positive <= 1'b0;
   //   	motor_negative <= 1'b0;
   //     end
   //     2'b01:begin
   //       motor_positive<=1'b0;
   //       motor_negative<=motor_pwm;
   //     end
   //     2'b10:begin
   //       motor_positive<=motor_pwm;
   //       motor_negative<=1'b0;
   //     end
   //     default: begin
   //       
   //   		motor_positive <= 1'b0;
   //   		motor_negative <= 1'b0;
   //     end
   //   endcase
	//	end
   // end
  //end
  
//PID control////////////////////////////////////////////////
 // PID constants
   reg [DATA_WIDTH-1:0] period_speed = 16'h0;// Counter connected to rise of encoder A that will infer speed by measuring period of clk cycles
  reg [DATA_WIDTH-1:0] speed_ctr;
  reg [DATA_WIDTH-1:0] Kp = {DATA_WIDTH{1'b0}};
  reg [DATA_WIDTH-1:0] Ki = {DATA_WIDTH{1'b0}};
  reg [DATA_WIDTH-1:0] Kd = {DATA_WIDTH{1'b0}};
  
  //MUX Internal/External PID constants
  always @(posedge clk or posedge reset) begin
    if (reset) begin
		Kp <= {DATA_WIDTH{1'b1}};
		Ki <= {DATA_WIDTH{1'b0}};
		Kd <= {DATA_WIDTH{1'b0}};
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
  reg signed[DATA_WIDTH-1:0] error;
  reg signed[(DATA_WIDTH)-1:0] integral;
  reg signed[DATA_WIDTH-1:0] derivative;
  reg signed [(DATA_WIDTH-1):0] pid_output;
  reg signed[DATA_WIDTH-1:0] previous_error;
	//Calculate Speed
	//dubious logic of carrying out speed<->clock cycle conversion:
  //We will give constant period_reference of 900RPM=>15 rounds each second, with clock period t=1ms, period_reference=15e3 clock cycles
	always @(posedge reset or posedge clk)
	begin
		if(reset) begin
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
   always @(posedge clk or posedge reset) begin
	if(reset) begin
		previous_error <= {DATA_WIDTH{1'b0}};
	error<=16'b0;
	end else begin
      previous_error<=error;
    error <= period_reference - period_speed;
	 end
  end

  // Calculate the integral
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      integral <= 16'b0;
    end else begin
      if (integral + error > 2047) begin //These are very high values for ASIC, need to push for lower register(?)

        integral <= 2047;  
      end else if (integral + error < -2048) begin
      
        integral <= -2048; 
      end else begin
        integral <= integral + error;  
      end
    end
  end

  // Calculate the derivative
   always @(posedge clk) begin
    derivative <= error - previous_error;
  end

  // Calculate the PID output
   always @(posedge clk) begin
    pid_output <= Kp * error + Ki * integral + Kd * derivative;	//compute pid response
     if(pid_output<1) begin			//if pid output is negative, cap at 0
			pwm_duty_cycle<=pwm_period;
		end else if (pid_output>pwm_period) begin	//if pid output above period, give 100% pwm
			pwm_duty_cycle<=1;
		end else begin
			pwm_duty_cycle<= pid_output[15:0];	//if in between values, give pid output as pwm
		end
  end

endmodule

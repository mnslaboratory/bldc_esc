module esc_1(
    input clk,
    input rst,  // SW0
    // I2C Ports
    inout sda,  // JB1
    input scl,  // JB2
    output sda_enable, // JB3
    
    input pwm_en,   // SW15
    input encoder_a,    // JB4
    input encoder_b,    // JB7
    output motor_positive,  // JB8
    output motor_negative   // JB9
    );
    
    // PID inputs
    wire [15:0] pwm_period_wire;
    wire [15:0] period_reference_i2c_wire;
    wire [7:0] Kp_ext_i2c_wire;
    wire [7:0] Ki_ext_i2c_wire;
    wire [6:0] Kd_ext_i2c_wire;
    wire override_internal_pid_i2c_wire;
    
    wire busy_wire;
    wire valid_wire;
    wire [7:0] data_out_i2c;
    wire [7:0] data_out_ram;
    wire write_i2c;
    wire read_1_i2c;
    wire [7:0] index_1_i2c;
    wire slow_clk;
    wire [7:0] Kp_int_wire;
    
    Divided_Clock(
        . clk(clk),
        . rst(rst),
        . div_clk(slow_clk)
    );
    
    I2C_SLAVE_1 I2C_SLAVE(
        . clk(slow_clk),
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
        . busy(busy_wire),
        . valid(valid_wire)
    );
    
    Register_Module_1 RAM_BLOCK(
        . clk(slow_clk),
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
        . Kp_int_i(Kp_int_wire), 
        . override_internal_pid(override_internal_pid_i2c_wire),   //0x44
        . autotune(tunerreset_autotune)             // 0x48
    );
    
    bldc_esc_1 BLDC_ESC(
      . clk(slow_clk),
      . reset(rst),
      . tunerreset_autotune(tunerreset_autotune),
      . pwm_en(pwm_en),
      . encoder_a(encoder_a),
      . encoder_b(encoder_b),
      . pwm_period(pwm_period_wire),
      . period_reference(period_reference_i2c_wire),
      . Kp_ext(Kp_ext_i2c_wire),
      . Ki_ext(Ki_ext_i2c_wire),
      . Kd_ext(Kd_ext_i2c_wire),
      . override_internal_pid(override_internal_pid_i2c_wire),
      . Kp_int_o(Kp_int_wire),
      . motor_positive(motor_positive),
      . motor_negative(motor_negative)
    );
endmodule
module top_module(clk,reset,pwm_en,override_internal_pid,pwm_period,period_reference,Kp_ext,Ki_ext,Kd_ext,motorpo,motorne);

parameter DATA_WIDTH=16;
	input clk,reset,pwm_en,override_internal_pid;
	input [DATA_WIDTH-1:0] pwm_period;
	input [DATA_WIDTH-1:0] period_reference,Kp_ext,Ki_ext,Kd_ext;
	output motorpo,motorne;
	
	
	wire motor_positive,motor_negative,encoder_a,encoder_b;

bldc_esc inst1 (
	.clk(clk),
	.reset(reset),
	.pwm_en(pwm_en),
	.encoder_a(encoder_a),
	.encoder_b(encoder_b),
	.override_internal_pid(override_internal_pid),
	.pwm_period(pwm_period),
	.period_reference(period_reference),
	.Kp_ext(Kp_ext),
	.Ki_ext(Ki_ext),
	.Kd_ext(Kd_ext),
	.motor_positive(motor_positive),
	.motor_negative(motor_negative)
);

test_bldc_motor test_motor_inst(
	.clk(clk),
	.reset(reset),
	.encoder_a(encoder_a),
	.encoder_b(encoder_b),
	.motor_positive(motor_positive),
	.motor_negative(motor_negative)
	);

	assign motorpo=motor_positive;
	assign motorne=motor_negative;









endmodule
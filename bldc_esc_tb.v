

module bldc_esc_tb ;
parameter DATA_WIDTH=16;
reg clk,reset,pwm_en,override_internal_pid;
reg [DATA_WIDTH-1:0] pwm_period;
reg [DATA_WIDTH-1:0] period_reference,Kp_ext,Ki_ext,Kd_ext;
wire motorpo,motorne;

top_module inst1 (
	.clk(clk),
	.reset(reset),
	.pwm_en(pwm_en),
	.override_internal_pid(override_internal_pid),
	.pwm_period(pwm_period),
	.period_reference(period_reference),
	.Kp_ext(Kp_ext),
	.Ki_ext(Ki_ext),
	.Kd_ext(Kd_ext),
	.motorpo(motorpo),
	.motorne(motorne)
);

initial
	begin	
      $dumpfile("dump.vcd");
      $dumpvars();
	clk=1'b0; reset=1'b1; pwm_en=1'b1; override_internal_pid=1'b1;
	Kp_ext=1;Ki_ext=0;Kd_ext=0;
	
	#10 reset=1'b0; pwm_period=16'd250;period_reference=8'd100;
	#100000;
	$stop;
	$finish;
		
	
	
	end
always #10 clk=~clk;

endmodule
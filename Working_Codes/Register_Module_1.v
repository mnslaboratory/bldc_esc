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
    
//    assign pwm_period            = 16'd5000;  
//    assign period_reference      = 16'd2290;
    assign pwm_period            = {internal_register[0],internal_register[1]};
    assign period_reference      = {internal_register[2],internal_register[3]};
    assign Kp_ext                = {internal_register[4],internal_register[5]};
    assign Ki_ext                = {internal_register[6],internal_register[7]};
    assign Kd_ext                = {internal_register[8],internal_register[9]};
    assign override_internal_pid = {internal_register[10]};
    
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

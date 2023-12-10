module Divided_Clock(
    input clk,
    input rst,
    output div_clk
    );
    reg [7:0] counter= 8'b0;
    assign div_clk = counter[1];
    always @(posedge clk)begin
        if (rst)begin
            counter <= counter + 1;
        end
        else begin
            counter <= counter + 1;
        end
    end
endmodule

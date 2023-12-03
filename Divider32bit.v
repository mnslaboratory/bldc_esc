// Code your design here
module Divider32bit (
   input      clk,
   input      reset,
   input      start_division,  // Start signal for division
   input [31:0] dividend,       // Dividend input
   input [31:0] divisor,        // Divisor input
   output reg [31:0] quotient,   // Quotient output
   output reg [31:0] remainder,  // Remainder output
   output reg division_active   // Declaration of division_active
);

  reg [5:0] division_cycle = 6'b100001;
   // Internal registers
   reg [32:0] store_divisor;      // Assigned as the original divisor
   reg [32:0] shifting_divisor;     // One bit more than divisor
   reg [31:0] shifting_dividend;     // Assigned as the original dividend

   // State machine
   always @(posedge clk or posedge reset) begin
      if (reset) begin
         store_divisor <= 33'b0;
         shifting_divisor <= 33'b0;
         shifting_dividend <= dividend;
         //division_active <= 0;  // Initialize division_active
      	 //division_cycle <= 6'b100001; // Initialize division_cycle
      end
		else if (division_cycle == 6'b000000) begin
         division_active <= 0;
         quotient <= shifting_dividend;
         remainder <= shifting_divisor[32:1];
         //remainder <= shifting_divisor[31:0];
         //quotient <= shifting_divisor[31:0];
      end
      else if (start_division && (division_cycle > 0)) begin
         // Move these assignments to the beginning of the block
         shifting_divisor <= {shifting_divisor[31:0], shifting_dividend[31]};
         shifting_dividend <= {shifting_dividend[30:0], 1'b0};
         
        if (shifting_divisor >= store_divisor) begin
            store_divisor <= {1'b0, divisor};  // Assign divisor to store_divisor
            shifting_divisor <= 33'b0;
            //shifting_dividend <= dividend;
            shifting_divisor[32:1] <= shifting_divisor[31:0] - store_divisor[31:0];
            shifting_divisor[0] <= shifting_dividend[31]; 
            shifting_dividend[0] <= 1;
         end
        division_cycle <= division_cycle - 1;
      end else begin
         division_active <= 1;
      end

      /* if (division_cycle == 1) begin
         division_active <= 1;
         //remainder <= shifting_dividend;
         remainder <= shifting_divisor[32:1];
      end */
     
   end
endmodule



`timescale 1ns/1ns

module Divider32bit_tb;

   reg clk;
   reg reset;
   reg start_division;
   reg [31:0] dividend;
   reg [31:0] divisor;

   wire [31:0] quotient;
   wire [31:0] remainder;

   Divider32bit dut(
      .clk(clk),
      .reset(reset),
      .start_division(start_division),
      .dividend(dividend),
      .divisor(divisor),
      .quotient(quotient),
     .remainder(remainder),
     .division_active(division_active )
   );

   // Clock
   initial begin
      clk = 0;
      forever #5 clk = ~clk;
   end

   // Test 
   initial begin
      $dumpfile("dump.vcd");
      $dumpvars;

      reset = 1;
      start_division = 0;
      dividend = 75;  // Change values as needed
      divisor = 10;

      #10 reset = 0;
      #50;
      #10 start_division = 1;
      // Allow some time for the division to complete
      #355 $finish;
   end

   // Display statements
   always @(posedge clk) begin
     $display("Time=%0t: Dividend=%d, Divisor=%d, Quotient=%d, Remainder=%d, store_divisor=%b, shifting_divisor=%b, shifting_dividend=%b, division_cycle=%d",
               $time, dividend, divisor, quotient, remainder, dut.store_divisor, dut.shifting_divisor, dut.shifting_dividend, dut.division_cycle);
   end

endmodule

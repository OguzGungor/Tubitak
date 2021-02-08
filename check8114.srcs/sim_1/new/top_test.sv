`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/02/2019 10:02:19 AM
// Design Name: 
// Module Name: arty_test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top_test;

    logic clk = 0;
    logic rst;
    //logic [3:0] led;

    always #5 clk = ~clk;
     
    initial begin
      rst = 0;
      #10 
      rst = 1;
    end

    
    top_artya7_100 DUT (
        .IO_CLK(clk),
        .IO_RST_N(rst));

endmodule
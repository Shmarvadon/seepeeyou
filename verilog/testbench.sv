`timescale 1ps/1ps

`define INSTRUCTION_QUEUE_LENGTH 64
`define INSTRUCTION_QUEUE_INPUT_WIDTH 4
module tb;
reg clk;
reg rst;
reg en;

// Alternate a global cock at 
always begin
    #10 clk=~clk;
end

initial begin
    en = 1;
    clk = 0;
    rst <= 1;
    #10 rst <= 0;
end


instruction_decoder #(`INSTRUCTION_QUEUE_LENGTH, `INSTRUCTION_QUEUE_INPUT_WIDTH) decoder(clk, rst, en);

endmodule
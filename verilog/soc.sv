`define INSTRUCTION_QUEUE_LENGTH 64
`define INSTRUCTION_QUEUE_INPUT_WIDTH 16


`include "structs.sv"
`include "defines.sv"
`timescale 1ps/1ps

module soc(
    input rst,

    output logic [31:0] mem_addr_sel,
    inout logic [127:0] mem_dat,
    output logic mem_en,
    output logic mem_we,
    output logic mem_re,
    output bit mclk
);

bit cclk;
bit fclk;
//bit mclk;

// Do the cocks.
always begin
    #10 cclk =~cclk;
end
always begin
    #20 fclk =~fclk;
end
always begin
    #60 mclk =~mclk;
end

always @(posedge cclk) $display("clock! %t", $time);

wire noc_bus core_to_mem;
wire noc_bus mem_to_core;

// Instantiate a core here in addition to clock generation for fclk, cclk & mclk.
// Instantiate a memory controller and GPIO block.

// Instantiate a basic memory controller.
memory_interface mem_int(rst, fclk, mclk, mem_addr_sel, mem_dat, mem_en, mem_we, mem_re, core_to_mem, mem_to_core);


// Instantiate a core.
core core_one(cclk, fclk, rst, mem_to_core, core_to_mem);


endmodule
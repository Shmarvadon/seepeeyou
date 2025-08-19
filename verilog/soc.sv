`define INSTRUCTION_QUEUE_LENGTH 64
`define INSTRUCTION_QUEUE_INPUT_WIDTH 16


`include "structs.svh"
`include "defines.svh"
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
    #5 cclk =~cclk;    //10
end
always begin
    #10 fclk =~fclk;    //20
end
always begin
    #30 mclk =~mclk;    //60
end

always @(posedge cclk) $display("clock! %t", $time);

// From core to IMC.
logic [31:0][7:0] noc_bus_inp_dat;
logic [5:0]       noc_bus_inp_bp;
logic             noc_bus_inp_bo;
// From IMC to core.
logic [31:0][7:0] noc_bus_oup_dat;
logic [5:0]       noc_bus_oup_bp;
logic             noc_bus_oup_bo;


// Instantiate a core here in addition to clock generation for fclk, cclk & mclk.
// Instantiate a memory controller and GPIO block.

// Instantiate a basic memory controller.
memory_interface mem_int(rst, fclk, mclk, noc_bus_inp_dat, noc_bus_inp_bp, noc_bus_inp_bo, noc_bus_oup_dat, noc_bus_oup_bp, noc_bus_oup_bo, mem_en, mem_we, mem_re, mem_addr_sel, mem_dat);


// Instantiate a core.
core core_one(cclk, fclk, rst, noc_bus_oup_dat, noc_bus_oup_bp, noc_bus_oup_bo, noc_bus_inp_dat, noc_bus_inp_bp, noc_bus_inp_bo);


endmodule
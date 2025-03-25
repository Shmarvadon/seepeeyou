`timescale 1ps/1ps

`include "defines.svh"
`include "core_memory_access_controller.sv"

module cmac_tb;

localparam NUM_TEST_REQS = 16;

// Test signals.
bit clk;
bit rst;

logic [2]           ts_fs_prts_inp_rp;
line_acc_req [2]    ts_fs_prts_inp_req;
logic [2]           ts_fs_prts_inp_op;

logic [2]           ts_fs_prts_oup_rp;
line_acc_req [2]    ts_fs_prts_oup_req;
logic [2]           ts_fs_prts_oup_op;

noc_ip_port noc_port();

// Instantiate the core memory access controller.
mem_acc_cont #(2) mac(clk, rst, ts_fs_prts_inp_rp, ts_fs_prts_inp_req, ts_fs_prts_inp_op,
ts_fs_prts_oup_rp, ts_fs_prts_oup_req, ts_fs_prts_oup_op, 
noc_port.ip_side);


line_acc_req [2][NUM_TEST_REQS] queued_reqs_for_test;

// Set initial conditions.
initial begin
    clk = 0;
    rst = 1;

    // Generate a bunch of test requests.
    for (int i = 0; i < NUM_TEST_REQS; i = i + 1) begin
        queued_reqs_for_test[0][i].addr = i * 10;
        queued_reqs_for_test[0][i].dat = i * 456;
        queued_reqs_for_test[0][i].wmsk = 16'b1111111111111111;
        queued_reqs_for_test[0][i].rqt = i % 2;
        queued_reqs_for_test[0][i].prt = 0;

        queued_reqs_for_test[1][i].addr = i * 40;
        queued_reqs_for_test[1][i].dat = (i+1) * 287753;
        queued_reqs_for_test[1][i].wmsk = 16'b1111111111110000;
        queued_reqs_for_test[1][i].rqt = (i + 1) % 2;
        queued_reqs_for_test[1][i].prt = 1;
    end

    // Prep a bunch of request to be sent to the mem acc controller.

    #5 rst = 0;
end

// Alternate the clock.
always begin #10 clk = ~clk; end


// always block to submit requests.
integer ind_1 = 0, ind_2 = 0;
always @(posedge clk) begin

    // If the port has accepted the request (ind 1).
    if (ts_fs_prts_inp_op[0]) begin
        // Incriment index 1.
        ind_1 = ind_1 + 1;
    end

    // If the port has accepted the request (ind 2).
    if (ts_fs_prts_inp_op[1]) begin
        // Incriment index 1.
        ind_2 = ind_2 + 1;
    end
end

// Present the requests from the queue of test requests to the MAC.
always_comb begin
    // Default values.
    ts_fs_prts_inp_req = 0;
    ts_fs_prts_inp_rp = 0;

    // if ind 1 is not at the end.
    if (ind_1 != NUM_TEST_REQS) begin
        ts_fs_prts_inp_req[0] = queued_reqs_for_test[0][ind_1];
        ts_fs_prts_inp_rp[0] = 1;
    end

    // If ind 2 is not at the end.
    if (ind_2 != NUM_TEST_REQS) begin
        ts_fs_prts_inp_req[1] = queued_reqs_for_test[1][ind_2];
        ts_fs_prts_inp_rp[1] = 1;
    end
end

// Pin the return of completed request accepted signal high for both ports.
assign ts_fs_prts_oup_op = 2'b11;

// Main memory block.
logic [10][7:0] mem;

// Emulate main memory & the NOC with simple comb block.
always_comb begin
    // Defaults.
    noc_port.tx_re = 0;

    // If the noc is trying to tx to us.
    if (noc_port.tx_av) begin
        // Accept it.
        noc_port.tx_re = 1;

        // Construct reply.
        noc_port.rx_dat.hdr.src_addr = noc_port.tx_dat.hdr.dst_addr;
        noc_port.rx_dat.hdr.src_port = noc_port.tx_dat.hdr.dst_port;
        noc_port.rx_dat.hdr.dst_addr = noc_port.tx_dat.hdr.src_addr;
        noc_port.rx_dat.hdr.dst_port = noc_port.tx_dat.hdr.src_port;

        // If the packet is a read request, generate a reply for it.
        if (noc_port.tx_dat.dat[7:0] == memory_read_request) begin

            // Construct a reply.
            noc_port.rx_dat.dat[0+:8] = memory_read_reply;
            // Write all 16 bytes into the reply.
            for (int i = 0; i < 16; i = i + 1) begin
                noc_port.rx_dat.dat[(8 + (i*8))+:8] = mem[noc_port.tx_dat.dat[8+:32] + i];
            end
            // Write the address into the reply.
            noc_port.rx_dat.dat[136+:32] = noc_port.tx_dat.dat[8+:32];
            // Write in packet type.
            noc_port.rx_dat.dat[7:0] = memory_read_reply;
        end
    end
end

endmodule
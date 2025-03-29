`timescale 1ps/1ps

`include "defines.svh"
`include "core_memory_access_controller.sv"

module cmac_tb;

localparam NUM_TEST_REQS = 16;

// Test signals.
bit clk;
bit rst;

logic [1:0]           ts_fs_prts_inp_rp;
line_acc_req [1:0]    ts_fs_prts_inp_req;
logic [1:0]           ts_fs_prts_inp_op;

logic [1:0]           ts_fs_prts_oup_rp;
line_acc_req [1:0]    ts_fs_prts_oup_req;
logic [1:0]           ts_fs_prts_oup_op;

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
    if (ind_1 < NUM_TEST_REQS) begin
        ts_fs_prts_inp_req[0] = queued_reqs_for_test[0][ind_1];
        ts_fs_prts_inp_rp[0] = 1;
    end

    // If ind 2 is not at the end.
    if (ind_2 < NUM_TEST_REQS) begin
        ts_fs_prts_inp_req[1] = queued_reqs_for_test[1][ind_2];
        ts_fs_prts_inp_rp[1] = 1;
    end
end

// Pin the return of completed request accepted signal high for both ports.
assign ts_fs_prts_oup_op = 2'b11;

// Drive the things to tell the mem acc controller noc stage what addr it is.
assign noc_port.prt_addr = 1;
assign noc_port.prt_num = 1;

// Main memory block.
bit [10][7:0] mem;

// Emulate main memory & the NOC with simple comb block.
always_comb begin
    // Defaults.
    noc_port.rx_av = 0;
    noc_port.rx_dat = 0;

    // If the noc is trying to tx to us.
    if (noc_port.tx_av) begin

        // Construct reply.
        noc_port.rx_dat.hdr.src_addr = noc_port.tx_dat.hdr.dst_addr;
        noc_port.rx_dat.hdr.src_port = noc_port.tx_dat.hdr.dst_port;
        noc_port.rx_dat.hdr.dst_addr = noc_port.tx_dat.hdr.src_addr;
        noc_port.rx_dat.hdr.dst_port = noc_port.tx_dat.hdr.src_port;

        // If the packet is a read request, generate a reply for it.
        if (noc_port.tx_dat.dat[7:0] == memory_read_request) begin

            // Write all 16 bytes into the reply.
            for (int i = 0; i < 16; i = i + 1) begin
                noc_port.rx_dat.dat[(8 + (i*8))+:8] = mem[noc_port.tx_dat.dat[8+:32] + i];
            end
            // Write the address into the reply.
            noc_port.rx_dat.dat[136+:32] = noc_port.tx_dat.dat[8+:32];
            // Write in packet type.
            noc_port.rx_dat.dat[7:0] = memory_read_reply;
            // Write in packet length.
            noc_port.rx_dat.hdr.len = $bits(noc_port.rx_dat.hdr) + $bits(mem_rd_rp);

            // Signal port rx available.
            noc_port.rx_av = 1;
        end
    end
end

always @(negedge clk) begin
    noc_port.tx_re = 0;
    // If it accepts the reply, accept the tx.
    if (noc_port.rx_re) noc_port.tx_re = 1;
end

endmodule


/*
// Test bench for L2 stage of mem acc controller.
module tb;

bit clk;
bit rst = 0;

    // Read port.
    logic            inp_rdp_rp;   // Read port request present.
    line_read_req    inp_rdp_req;  // Read port request.
    logic           inp_rdp_op;   // Read port open.

    // Write port.
    logic            inp_wrp_rp;   // Write port request present.
    line_write_req   inp_wrp_req;  // Write port request.
    logic           inp_wrp_op;   // Write port open.

    // Outputs from this stage.

    // Read port.
    logic            oup_rdp_rp;   // Read port request present.
    line_read_req    oup_rdp_req;  // Read port request.
    logic             oup_rdp_op;   // Read port open.

    // Write port.
    logic            oup_wrp_rp;   // Write port request present.
    line_write_req   oup_wrp_req;  // Write port request.
    logic             oup_wrp_op;   // Write port open.

    // Return channel for completed requests.
    logic rd_rpl_p;
    line_read_reply rd_rpl;
    logic rd_rpl_a;

    mem_acc_l2_stage l2_stg(clk, rst, inp_rdp_rp, inp_rdp_req, inp_rdp_op, inp_wrp_rp,
    inp_wrp_req, inp_wrp_op, oup_rdp_rp, oup_rdp_req, oup_rdp_op, oup_wrp_rp,
    oup_wrp_req, oup_wrp_op, rd_rpl_p, rd_rpl, rd_rpl_a);


initial begin
    rd_rpl_a = 1;
    // Present a write request to address 12345.
    inp_wrp_req.addr = 12345;
    inp_wrp_req.dat = 420;
    inp_wrp_rp = 1;

    // Should take 1 cycles to be accepted.
    #20

    // Stop the write from reoccuring.
    inp_wrp_rp = 0;

    #20

    // Present a read request to the cache at 12345.
    inp_rdp_req.addr = 12345;
    inp_rdp_rp = 1;

    // Wait a cycle then disable the read port.
    #20
    inp_rdp_rp = 0;

    #40

    // Try to do a simultaneous read + write to 2 different addresses.
    inp_wrp_req.addr = 42069;
    inp_wrp_req.dat = 256;
    inp_wrp_rp = 1;

    inp_rdp_req.addr = 12345;
    inp_rdp_rp = 1;

    // Wait 4 cycles then stop submitting requests.
    #80

    inp_rdp_rp = 0;
    inp_wrp_rp = 0;


end

always begin
    #10 clk = ~clk;
end

always @(posedge clk) begin
    $display("Clock! %t", $time);
end

endmodule
*/

/*
// Test bench for L1 stage of mem acc controller.
module tb;

bit clk;
bit rst = 0;

    // Read port.
    logic            inp_rdp_rp;   // Read port request present.
    line_read_req    inp_rdp_req;  // Read port request.
    logic           inp_rdp_op;   // Read port open.

    // Write port.
    logic            inp_wrp_rp;   // Write port request present.
    line_write_req   inp_wrp_req;  // Write port request.
    logic           inp_wrp_op;   // Write port open.

    // Outputs from this stage.

    // Read port.
    logic            oup_rdp_rp;   // Read port request present.
    line_read_req    oup_rdp_req;  // Read port request.
    logic             oup_rdp_op;   // Read port open.

    // Write port.
    logic            oup_wrp_rp;   // Write port request present.
    line_write_req   oup_wrp_req;  // Write port request.
    logic             oup_wrp_op;   // Write port open.

    // Return channel for completed requests.
    logic rd_rpl_p;
    line_read_reply rd_rpl;
    logic rd_rpl_a;

    mem_acc_l1_stage l1_stg(clk, rst, inp_rdp_rp, inp_rdp_req, inp_rdp_op, inp_wrp_rp,
    inp_wrp_req, inp_wrp_op, oup_rdp_rp, oup_rdp_req, oup_rdp_op, oup_wrp_rp,
    oup_wrp_req, oup_wrp_op, rd_rpl_p, rd_rpl, rd_rpl_a);


initial begin
    rd_rpl_a = 1;
    // Present a write request to address 12345.
    inp_wrp_req.addr = 12345;
    inp_wrp_req.dat = 420;
    inp_wrp_rp = 1;

    // Should take 1 cycles to be accepted.
    #20

    // Stop the write from reoccuring.
    inp_wrp_rp = 0;

    #20

    // Present a read request to the cache at 12345.
    inp_rdp_req.addr = 12345;
    inp_rdp_rp = 1;

    // Wait a cycle then disable the read port.
    #20
    inp_rdp_rp = 0;

    #40

    // Try to do a simultaneous read + write to 2 different addresses.
    inp_wrp_req.addr = 42069;
    inp_wrp_req.dat = 256;
    inp_wrp_rp = 1;

    inp_rdp_req.addr = 12345;
    inp_rdp_rp = 1;

    // Wait 4 cycles then stop submitting requests.
    #80

    inp_rdp_rp = 0;
    inp_wrp_rp = 0;


end

always begin
    #10 clk = ~clk;
end

always @(posedge clk) begin
    $display("Clock! %t", $time);
end

endmodule
*/

/*
// Validating new L1 cache.
module tb;

bit clk;
bit rst = 0;

// Instantiate an L1 cache block.
logic [31:0]    cache_rdp_addr;
logic           cache_rdp_en;
logic [127:0]   cache_rdp_dat;
logic           cache_rdp_done;
logic           cache_rdp_suc;

logic [31:0]    cache_wrp_addr;
logic           cache_wrp_en;
logic [127:0]   cache_wrp_dat;
logic           cache_wrp_done;
logic           cache_wrp_suc;

logic [31:0]    cache_bs_addr;
logic [127:0]   cache_bs_dat;
logic           cache_bs_en;
logic           cache_bs_done;

l1_cache #(4, 256, 16, 32) l1(clk, rst, cache_rdp_addr, cache_rdp_en, cache_rdp_dat, cache_rdp_done, cache_rdp_suc,
                                cache_wrp_addr, cache_wrp_en, cache_wrp_dat, cache_wrp_done, cache_wrp_suc,
                                cache_bs_adr, cache_bs_dat, cache_bs_en, cache_bs_done);

initial begin
    // Present a write request to address 12345.
    cache_wrp_addr = 12345;
    cache_wrp_dat = 420;
    cache_wrp_en = 1;

    // Should take 2 cycles to complete.
    #40

    // Stop the write from reoccuring.
    cache_wrp_en = 0;

    // Present a read request to the cache at 12345.
    cache_rdp_addr = 12345;
    cache_rdp_en = 1;

    // Wait a cycle then disable the read port.
    #20
    cache_rdp_en = 0;

    #40

    // Try to do a simultaneous read + write to 2 different addresses.
    cache_wrp_addr = 42069;
    cache_wrp_dat = 256;
    cache_wrp_en = 1;

    cache_rdp_addr = 12345;
    cache_rdp_en = 1;


end

always begin
    #10 clk = ~clk;
end

always @(posedge clk) begin
    $display("Clock! %t", $time);
end

endmodule
*/

/*
module tb;

bit clk;
bit rst;
mac_stage_intf l1_stage_intf();
mac_stage_intf port_intfs [2] ();

mac_request [3] stage_return_requests;
logic       [3] stage_returns_present;
logic       [3] stage_returns_accepted;

initial begin
    l1_stage_intf.inp_ra = 1;
end

mac_scheduler #(2) scheduler(clk, rst, l1_stage_intf, port_intfs, stage_return_requests, stage_returns_present, stage_returns_accepted);


initial begin
    // Create a memory access request to write a line to cache.
    port_intfs[0].inp_rp = 1;
    port_intfs[0].inp_req.addr = 32'h0012014a;
    port_intfs[0].inp_req.dat = 42069;
    port_intfs[0].inp_req.orig = 0;
    port_intfs[0].inp_req.rqt = 1;
    port_intfs[0].inp_req.rsuc = 0;

    # 20

    // Stop trying to submit a request.
    port_intfs[0].inp_rp = 0;

    // Try to return the request just submitted.
    stage_return_requests[0].addr = 32'h0012014a;
    stage_return_requests[0].orig = 0;
    stage_returns_present[0] = 1;

    // signal port 0 of the dispatchers to accept the return.
    port_intfs[0].oup_ra = 1;

    #20

    // Stop trying to return a request.
    stage_returns_present[0] = 0;

    // Read from the addr just written to.
    port_intfs[0].inp_rp = 1;
    port_intfs[0].inp_req.rqt = 0;

    #200

    // Read from somewhere that isnt in cache yet.
    port_intfs[0].inp_rp = 1;
    port_intfs[0].inp_req.addr = 32'h0101a4f8;

    #20

    port_intfs[0].inp_rp = 0;
end


always begin
    #10 clk = ~clk;
end

always @(posedge clk) begin
    $display("Clock! %t", $time);
end

endmodule
*/



/*

module tb;

bit clk;
bit rst;

bit [31:0]      fs_addr;
bit             fs_wr;
bit             fs_go;
bit             fs_go_go;

logic           fs_done;
logic           fs_suc;

bit [127:0]     fs_inp;
logic [127:0]   fs_oup;

logic [31:0]    bs_addr;
logic [127:0]   bs_oup;
logic           bs_we;
bit             bs_done;

integer i;
initial begin
    clk = 0;
    bs_done = 1;
end

always begin
    #10 clk =~clk;
    if (clk) $display("Clock! %t", $time);
end

assign fs_go_go = fs_done ? 0 : fs_go;

bit[7:0] thing;
always @(posedge clk) begin
    case(thing)

    // Write to the cache.
    0:
    begin
        fs_go = 1;
    
        fs_addr = 128;
        fs_addr[12] = 1;

        fs_wr = 1;

        fs_inp = 128;

        if (fs_done) begin
            thing = 1;
            fs_go = 0;
        end
    end

    // Wait a cycle.
    1:
    begin
        thing = 2;
    end

    // Read cache line back.
    2:
    begin

        fs_go = 1;
        fs_wr = 0;

        if (fs_done) begin
            thing = 3;
            fs_go = 0;
        end
    end

    // Write another location.
    3:
    begin

        fs_go = 1;

        fs_addr = 32'h00010110;// + (i * 32'h00100000);       // *** Note to self make sure addr is 128b aligned or issues ensue. ***
        fs_inp =  2;

        fs_wr = 1;


        if (fs_done) thing = 5;
    end

    // Wait a cycle.
    4:
    begin
        thing = 5;
    end

    // Attempt to read back first thing written.
    5:
    begin
        fs_addr = 128;
        fs_addr[12] = 1;

        fs_wr = 0;
        
        if (fs_done) thing = 6;
    end

    // Attempt to modify the first location.
    6:
    begin
        fs_wr = 1;

        fs_inp = 42069;


        if (fs_done) thing = 7;
    end

    // Write should be done by now (modified line instead of new).
    7:
    begin
        fs_wr = 0;
    end

    endcase
end
// 4 bit line position, 8 bit line select & 20 bit tag.
l2_cache #(8, 256, 16, 32, 4) caching(clk, rst, fs_addr, fs_wr, fs_go_go, fs_done, fs_suc, fs_inp, fs_oup, bs_addr, bs_oup, bs_we, bs_done);


endmodule
*/

/*
module tb;

bit clk;
bit rst;

bit [31:0]      fs_addr;
bit             fs_wr;
bit             fs_go;

logic           fs_done;
logic           fs_suc;

bit [127:0]     fs_inp;
logic [127:0]   fs_oup;

logic [31:0]    bs_addr;
logic [127:0]   bs_oup;
logic           bs_we;
bit             bs_done;

integer i;
initial begin
    clk = 0;
    bs_done = 1;
end

always begin
    #10 clk =~clk;
    if (clk) $display("Clock! %t", $time);
end

bit[7:0] thing;
always @(posedge clk) begin
    case(thing)

    // Write to the cache.
    0:
    begin
        fs_go = 1;
    
        fs_addr = 128;
        fs_addr[12] = 1;

        fs_wr = 1;

        fs_inp = 128;

        thing = 1;
    end

    // Wait a cycle.
    1:
    begin
        thing = 2;
    end

    // Read cache line back.
    2:
    begin
        fs_wr = 0;

        thing = 3;
    end

    // Write another location.
    3:
    begin
        fs_addr = 32'h00010110;// + (i * 32'h00100000);       // *** Note to self make sure addr is 128b aligned or issues ensue. ***
        fs_inp =  2;

        fs_wr = 1;

        thing = 4;
    end

    // Wait a cycle.
    4:
    begin
        thing = 5;
    end

    // Attempt to read back first thing written.
    5:
    begin
        fs_addr = 128;
        fs_addr[12] = 1;

        fs_wr = 0;

        thing = 6;
    end

    // Attempt to modify the first location.
    6:
    begin
        fs_wr = 1;

        fs_inp = 42069;

        thing = 7;
    end

    // Write should be done by now (modified line instead of new).
    7:
    begin
        fs_wr = 0;
    end

    endcase
end
// 4 bit line position, 8 bit line select & 20 bit tag.
l1_cache #(4, 256, 16, 32) caching(clk, rst, fs_addr, fs_wr, fs_go, fs_done, fs_suc, fs_inp, fs_oup, bs_addr, bs_oup, bs_we, bs_done);


endmodule
*/

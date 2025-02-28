`timescale 1ps/1ps



`include "defines.svh"
//`include "nocstop.sv"
`include "core_memory_access_controller.sv"

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
    oup_wrp_req, oup_wrp_op, rd_rpl_p, rd_rpl, rd_rpl_a
);


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


end

always begin
    #10 clk = ~clk;
end

always @(posedge clk) begin
    $display("Clock! %t", $time);
end

endmodule

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




/*
module tb;
reg clk;
reg rst;

// Alternate a global cock at 
//always begin
    //#10 clk=~clk;
//end
//always @(posedge clk) $display("clock!");

initial begin
    clk = 0;
    rst <= 1;
    #5 rst <= 0;
end

// Emulate 64KB of memory.
bit [0:7] memory [65535:0];

integer i;

initial begin
    // Setup instructions in memory.

    // ADD R2, R2
    memory[0] = 8'b10000100;
    memory[1] = 8'b00100010;

    // ADD 24, R2
    memory[2] = 8'b10001100;
    memory[3] = 8'b00000010;
    memory[4] = 0;
    memory[5] = 0;
    memory[6] = 0;
    memory[7] = 24;

    //RLS R2, R3
    memory[8] = 8'b01010100;
    memory[9] = 8'b00110010;
    
    //MUL 10, R3
    memory[10] = 8'b11101100;
    memory[11] = 8'b00000011;
    memory[12] = 0;
    memory[13] = 0;
    memory[14] = 0;
    memory[15] = 10;

    // GOT 64
    memory[16] = 8'b01001110;
    memory[17] = 64;
    memory[18] = 0;
    memory[19] = 0;
    memory[20] = 0;

    // LOD 0, R1
    memory[21] = 8'b00000010;
    memory[22] = 8'b00000001;
    memory[23] = 0;
    memory[24] = 0;
    memory[25] = 0;
    memory[26] = 0;

    // STR 128, R1
    memory[27] = 8'b10000010;
    memory[28] = 8'b00000001;
    memory[29] = 128;
    memory[30] = 0;
    memory[31] = 0;
    memory[32] = 0;

    //JMP 0
    memory[33] = 8'b00001110;
    memory[34] = 0;
    memory[35] = 0;
    memory[36] = 0;
    memory[37] = 0;

    // RET
    memory[64] = 8'b11000110;
end


logic [31:0] mem_addr_sel;
wire logic [127:0] mem_dat;
bit [127:0] mem_dat_driver;
logic mem_we;
logic mem_re;
logic mem_en;
logic mclk;
soc sooc(rst, mem_addr_sel, mem_dat, mem_en, mem_we, mem_re, mclk);


// Handle the memory stuffs.
assign mem_dat = (mem_re) ? mem_dat_driver : 'hz;
always @(posedge mclk) begin
    // If the memory is enabled.
    if (mem_en) begin

        // memory write.
        if (mem_we) begin
            memory[mem_addr_sel[31:0]]          <= mem_dat[7:0];
            memory[mem_addr_sel[31:0] + 1]      <= mem_dat[15:8];
            memory[mem_addr_sel[31:0] + 2]      <= mem_dat[23:16];
            memory[mem_addr_sel[31:0] + 3]      <= mem_dat[31:24];
            memory[mem_addr_sel[31:0] + 4]      <= mem_dat[39:32];
            memory[mem_addr_sel[31:0] + 5]      <= mem_dat[47:40];
            memory[mem_addr_sel[31:0] + 6]      <= mem_dat[55:48];
            memory[mem_addr_sel[31:0] + 7]      <= mem_dat[63:56];

            $display("Memory write request. %b %t", mem_addr_sel, $time);

            memory[mem_addr_sel[31:0] + 8]      <= mem_dat[71:64];
            memory[mem_addr_sel[31:0] + 9]      <= mem_dat[79:72];
            memory[mem_addr_sel[31:0] + 10]     <= mem_dat[87:80];
            memory[mem_addr_sel[31:0] + 11]     <= mem_dat[95:88];
            memory[mem_addr_sel[31:0] + 12]     <= mem_dat[103:96];
            memory[mem_addr_sel[31:0] + 13]     <= mem_dat[111:104];
            memory[mem_addr_sel[31:0] + 14]     <= mem_dat[119:112];
            memory[mem_addr_sel[31:0] + 15]     <= mem_dat[127:120];

        end

        // memory read.
        if (mem_re) begin
            mem_dat_driver[7:0]      <=  memory[mem_addr_sel[31:0]];
            mem_dat_driver[15:8]     <=  memory[mem_addr_sel[31:0] + 1];
            mem_dat_driver[23:16]    <=  memory[mem_addr_sel[31:0] + 2];
            mem_dat_driver[31:24]    <=  memory[mem_addr_sel[31:0] + 3];
            mem_dat_driver[39:32]    <=  memory[mem_addr_sel[31:0] + 4];
            mem_dat_driver[47:40]    <=  memory[mem_addr_sel[31:0] + 5];
            mem_dat_driver[55:48]    <=  memory[mem_addr_sel[31:0] + 6];
            mem_dat_driver[63:56]    <=  memory[mem_addr_sel[31:0] + 7];

            $display("Memory read request. %b %t", mem_addr_sel, $time);

            mem_dat_driver[71:64]    <=  memory[mem_addr_sel[31:0] + 8];
            mem_dat_driver[79:72]    <=  memory[mem_addr_sel[31:0] + 9];
            mem_dat_driver[87:80]    <=  memory[mem_addr_sel[31:0] + 10];
            mem_dat_driver[95:88]    <=  memory[mem_addr_sel[31:0] + 11];
            mem_dat_driver[103:96]   <=  memory[mem_addr_sel[31:0] + 12];
            mem_dat_driver[111:104]  <=  memory[mem_addr_sel[31:0] + 13];
            mem_dat_driver[119:112]  <=  memory[mem_addr_sel[31:0] + 14];
            mem_dat_driver[127:120]  <=  memory[mem_addr_sel[31:0] + 15];
        end
    end
end

endmodule
*/

/*

module tb;
reg clk;
reg clk_2;
reg rst;
reg en;

// Alternate a global cock at 
always begin
    #10 clk=~clk;
end

// Alternate secondary clock.
always begin
    #5 clk_2=~clk_2;
end

always @(posedge clk) $display("clock!");


initial begin
    en = 1;
    clk = 0;
    clk_2 = 0;
    rst <= 1;
    #5 rst <= 0;
end

// Emulate 64KB of memory.
bit [0:7] memory [65535:0];

integer i;
initial begin
    // Setup instructions in memory.

    // ADD R2, R2
    memory[0] = 8'b10000100;
    memory[1] = 8'b00100010;

    // ADD 24, R2
    memory[2] = 8'b10001100;
    memory[3] = 8'b00000010;
    memory[4] = 0;
    memory[5] = 0;
    memory[6] = 0;
    memory[7] = 24;

    //RLS R2, R3
    memory[8] = 8'b01010100;
    memory[9] = 8'b00110010;
    
    //MUL 10, R3
    memory[10] = 8'b11101100;
    memory[11] = 8'b00000011;
    memory[12] = 0;
    memory[13] = 0;
    memory[14] = 0;
    memory[15] = 10;

    // GOT 64
    memory[16] = 8'b01001110;
    memory[17] = 64;
    memory[18] = 0;
    memory[19] = 0;
    memory[20] = 0;

    // LOD 0, R1
    memory[21] = 8'b00000010;
    memory[22] = 8'b00000001;
    memory[23] = 0;
    memory[24] = 0;
    memory[25] = 0;
    memory[26] = 0;

    // STR 128, R1
    memory[27] = 8'b10000010;
    memory[28] = 8'b00000001;
    memory[29] = 128;
    memory[30] = 0;
    memory[31] = 0;
    memory[32] = 0;

    //JMP 0
    memory[33] = 8'b00001110;
    memory[34] = 0;
    memory[35] = 0;
    memory[36] = 0;
    memory[37] = 0;

    // RET
    memory[64] = 8'b11000110;

end

wire noc_bus into_core;
wire noc_bus out_of_core;
// Instantiate a core.
core test_core(clk_2, clk, rst, into_core, out_of_core);

// Initialise a secondary NOC stop to act as IMC endpoint.
wire ip_port mem_noc_prt;
noc_stop #(1, 2) memory_endpoint(clk, clk_2, rst, out_of_core, into_core, mem_noc_prt);

packet reply_pckt;
logic submit_tx;
logic rx_complete;

assign mem_noc_prt.dat_to_noc = reply_pckt;
assign mem_noc_prt.tx_submit = submit_tx;
assign mem_noc_prt.rx_complete = rx_complete;

// Very dumb noc port interface.
always @(posedge clk_2) begin
    // If there is a memory access request.
    if (mem_noc_prt.rx_recieve) begin
        // if a memory read request.
        if (mem_noc_prt.dat_from_noc.pt == memory_read_request) begin
            // Fill the reply.
            reply_pckt.pt = memory_read_reply;
            reply_pckt.id = mem_noc_prt.dat_from_noc.id;

            reply_pckt.dat[7:0]      <=  memory[mem_noc_prt.dat_from_noc.dat[31:0]];
            reply_pckt.dat[15:8]     <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 1];
            reply_pckt.dat[23:16]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 2];
            reply_pckt.dat[31:24]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 3];
            reply_pckt.dat[39:32]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 4];
            reply_pckt.dat[47:40]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 5];
            reply_pckt.dat[55:48]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 6];
            reply_pckt.dat[63:56]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 7];

            reply_pckt.dat[71:64]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 8];
            reply_pckt.dat[79:72]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 9];
            reply_pckt.dat[87:80]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 10];
            reply_pckt.dat[95:88]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 11];
            reply_pckt.dat[103:96]   <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 12];
            reply_pckt.dat[111:104]  <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 13];
            reply_pckt.dat[119:112]  <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 14];
            reply_pckt.dat[127:120]  <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 15];

            reply_pckt.dst_addr <= mem_noc_prt.dat_from_noc.src_addr;
            reply_pckt.dst_prt <= mem_noc_prt.dat_from_noc.src_prt;

            reply_pckt.src_addr <= 2;
            reply_pckt.src_prt <= 0;

            // submit the reply.
            submit_tx <= 1;

            // Indicate that the rx is complete
            rx_complete <= 1;
        end

        // If a memory write request.
        if (mem_noc_prt.dat_from_noc.pt == memory_write_request) begin
            // Write to memory.
            memory[mem_noc_prt.dat_from_noc.dat[31:0]] = mem_noc_prt.dat_from_noc.dat[39:32]; 
            memory[mem_noc_prt.dat_from_noc.dat[31:0]+1] = mem_noc_prt.dat_from_noc.dat[47:40]; 
            memory[mem_noc_prt.dat_from_noc.dat[31:0]+2] = mem_noc_prt.dat_from_noc.dat[55:48]; 
            memory[mem_noc_prt.dat_from_noc.dat[31:0]+3] = mem_noc_prt.dat_from_noc.dat[63:56]; 

            // create reply.
            reply_pckt.pt = memory_write_reply;
            reply_pckt.id = mem_noc_prt.dat_from_noc.id;

            reply_pckt.dst_addr <= mem_noc_prt.dat_from_noc.src_addr;
            reply_pckt.dst_prt <= mem_noc_prt.dat_from_noc.src_prt;

            reply_pckt.src_addr <= 2;
            reply_pckt.src_prt <= 0;

            // Submit the reply.
            submit_tx <= 1;

            // indicate that rx is complete.
            rx_complete <= 1;
        end
    end
    // If the reply is complete.
    if (mem_noc_prt.tx_complete) begin
        // Stop signalling to send a tx.
        submit_tx <= 0;

        // signal that reading the rx is not done yet.
        rx_complete <= 0;
    end
end


endmodule

*/


/*
wire noc_bus one_to_two;
wire noc_bus two_to_one;

wire ip_port stop_one_prt_one;
noc_stop #(1, 1) noc_stop_one(clk, clk_2, rst, two_to_one, one_to_two, stop_one_prt_one);

wire ip_port stop_two_prt_one;
noc_stop #(1, 2) noc_stop_two(clk, clk_2, rst, one_to_two, two_to_one, stop_two_prt_one);

// Stop one stuff.
packet stp_one_tx_dat;
logic stp_one_tx_submit;

logic stp_one_rx_complete;

assign stop_one_prt_one.dat_to_noc = stp_one_tx_dat;
assign stop_one_prt_one.tx_submit = stp_one_tx_submit;

assign stop_one_prt_one.rx_complete = stp_one_rx_complete;

// Stop two stuff.
packet stp_two_tx_dat;
logic stp_two_tx_submit;

logic stp_two_rx_complete;

assign stop_two_prt_one.dat_to_noc = stp_two_tx_dat;
assign stop_two_prt_one.tx_submit = stp_two_tx_submit;

assign stop_two_prt_one.rx_complete = stp_two_rx_complete;

// Send a packet to noc stop one to go to noc stop 2.
initial begin

    #10

    stp_one_tx_dat.dst_addr <= 2;
    stp_one_tx_dat.dst_prt <= 0;

    stp_one_tx_dat.src_addr <= 1;
    stp_one_tx_dat.src_prt <= 0;

    stp_one_tx_dat.dat <= 'h42069;

    stp_one_tx_dat.pt = memory_read_request;
    stp_one_tx_dat.id = 0;

    #10 stp_one_tx_submit <= 1;

    #10 stp_one_tx_submit <= 0;

    // Modify packet and transmit another one.
    stp_one_tx_dat.dat <= 'h1234;
    #10 stp_one_tx_submit <= 1;

    #10 stp_one_tx_submit <= 0;
end


packet recieved_packet;
// Handle recieveing the packet at noc stop 2.
always @(posedge clk_2) begin
    // If there is a packet waiting for us to recieve.
    if (stop_two_prt_one.rx_recieve) begin
        // Move the data into recieved_packet variable.
        recieved_packet <= stop_two_prt_one.dat_from_noc;

        // Signal to the port that we have read the data.
        stp_two_rx_complete <= 1;

        // Lets also reply with something.
        stp_two_tx_dat.dst_addr <= 1;
        stp_two_tx_dat.dst_prt <= 0;
        stp_two_tx_dat.src_addr <= 1;
        stp_two_tx_dat.src_prt <= 0;
        stp_two_tx_dat.dat = 'h420420420;
        stp_two_tx_dat.pt = memory_write_request;
        stp_two_tx_dat.id = 69;

        stp_two_tx_submit <= 1;
    end
    // If there is not a packet waiting for us to recieve.
    else begin
        // Signal to the port that we have not read any data.
        stp_two_rx_complete <= 0;
    end

    // If the tx submitted has been processed successfully.
    if (stop_two_prt_one.tx_complete) begin

        // Stop requesting TX.
        stp_two_tx_submit <= 0;

        stp_two_tx_dat.dst_addr <= 0;
        stp_two_tx_dat.dst_prt <= 0;
        stp_two_tx_dat.dat = 0;
        stp_two_tx_dat.pt = memory_write_request;
        stp_two_tx_dat.id = 0;
    end
end

// Handle recieveing the ping back from noc stop 2.
always @(posedge clk_2) begin
    // If there is an RX for us.
    if (stop_one_prt_one.rx_recieve) begin
        // Accept it.
        stp_one_rx_complete <= 1;

    end
    // If there is not an rx for us.
    else begin
        stp_one_rx_complete <= 0;
    end
end
*/



















/*

//          NOC port handling stuff
wire ip_port noc_port;
wire ip_port pcfu_noc_port;

packet reply;
noc_port_status reply_prt_stat;
noc_port_status req_prt_stat;
logic tx_send;

assign noc_port.dat_from_noc = reply;
assign noc_port.from_noc_prt_stat = reply_prt_stat;
assign noc_port.to_noc_prt_stat = req_prt_stat;
assign noc_port.tx_recieve = tx_send;

core cpu_core(clk, rst, noc_port, pcfu_noc_port); 


//         Main memory emulator            

// Emulate 64KB of memory.
bit [0:7] memory [65535:0];

always @(posedge clk) begin
    // If the instruction fetch unit has submitted a request to the noc.
    if (noc_port.tx_submit) begin
        // If the request is for data from memory.
        if (noc_port.dat_to_noc.pt == memory_read_request) begin
            // Fill the reply.
            reply.pt = memory_read_reply;
            reply.id = 0;

            reply.dat[7:0]      <=  memory[noc_port.dat_to_noc.dat[31:0]];
            reply.dat[15:8]     <=  memory[noc_port.dat_to_noc.dat[31:0] + 1];
            reply.dat[23:16]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 2];
            reply.dat[31:24]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 3];
            reply.dat[39:32]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 4];
            reply.dat[47:40]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 5];
            reply.dat[55:48]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 6];
            reply.dat[63:56]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 7];

            reply.dat[71:64]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 8];
            reply.dat[79:72]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 9];
            reply.dat[87:80]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 10];
            reply.dat[95:88]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 11];
            reply.dat[103:96]   <=  memory[noc_port.dat_to_noc.dat[31:0] + 12];
            reply.dat[111:104]  <=  memory[noc_port.dat_to_noc.dat[31:0] + 13];
            reply.dat[119:112]  <=  memory[noc_port.dat_to_noc.dat[31:0] + 14];
            reply.dat[127:120]  <=  memory[noc_port.dat_to_noc.dat[31:0] + 15];

            reply.dst_addr <= noc_port.dat_to_noc.src_addr;
            reply.dst_prt <= noc_port.dat_to_noc.src_prt;

            reply.src_addr <= 1;
            reply.src_prt <= 0;

            // Signal that a tx is ready to be sent back to instruction fetch unit.
            tx_send <= 1;

            // Open the port to send the reply.
            reply_prt_stat <= port_open;
        end
    end
    // If there is nothing for us to reply to.
    else begin
        // Stop signalling that a reply is ready.
        tx_send <= 0;

        // Close the reply port.
        reply_prt_stat <= 0;
    end
end


integer i;
initial begin
    // Set NOC port initial conditions.
    req_prt_stat = port_open;

    // Setup an instruction in memory.

    // ADD R2, R2
    memory[0] = 8'b10000100;
    memory[1] = 8'b00100010;

    // ADD 24, R2
    memory[2] = 8'b10001100;
    memory[3] = 8'b00000010;
    memory[4] = 0;
    memory[5] = 0;
    memory[6] = 0;
    memory[7] = 24;

    //RLS R2, R3
    memory[8] = 8'b01010100;
    memory[9] = 8'b00110010;
    
    //MUL 10, R3
    memory[10] = 8'b11101100;
    memory[11] = 8'b00000011;
    memory[12] = 0;
    memory[13] = 0;
    memory[14] = 0;
    memory[15] = 10;

    //JMP 0
    memory[16] = 8'b00001110;
    memory[17] = 0;
    memory[18] = 0;
    memory[19] = 0;
    memory[20] = 0;
end
*/
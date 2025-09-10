`include "defines.svh"
`include "structs.svh"

// Idea to re write CMAC with the in flight buffer being referenced from the caches to reduce the bit width of passing all the requests around.


// Each cache is 2 port: 1 read, 1 write.
// Each cache has a seperate read & write queue.
//`define CMAC_DEBUG_LOG

// Memory access controller scheduler stage.
module mem_acc_scheduler #(parameter NUM_PORTS = 2) (
    input   logic                           clk,                // Clock.
    input   logic                           rst,                // Reset.

    // Communicate with the front end ports.

    // Request submission.
    input   logic [NUM_PORTS-1:0]           fs_prts_inp_rp,     // Front end ports input request present.
    input   line_acc_req [NUM_PORTS-1:0]    fs_prts_inp_req,    // Front end ports input request.
    output  logic [NUM_PORTS-1:0]           fs_prts_inp_ra,     // Front end ports input open.

    // Request reply.
    output  logic [NUM_PORTS-1:0]           fs_prts_oup_rp,     // Front end ports output request present.
    output  line_acc_req [NUM_PORTS-1:0]    fs_prts_oup_req,    // Front end ports output request.
    input   logic [NUM_PORTS-1:0]           fs_prts_oup_ra,     // Front end ports output open.


    // Input to stage 1 of mem access controller.

    // Read port.
    output  logic                           stg_1_rdp_rp,       // Stage 1 read port request present.
    output  line_read_req                   stg_1_rdp_req,      // Stage 1 read port request.
    input   logic                           stg_1_rdp_op,       // Stage 1 read port open.

    // Write port.
    output  logic                           stg_1_wrp_rp,       // Stage 1 write port request present.
    output  line_write_req                  stg_1_wrp_req,      // Stage 1 write port request.
    input   logic                           stg_1_wrp_op,       // Stage 1 write port open.


    // Output from the L1, L2 & NOC stages returning read replies.
    input   logic [3]                       rd_rpl_p,           // Read reply present.
    input   line_read_reply [3]             rd_rpl,             // Read reply.
    output  logic [3]                       rd_rpl_a            // Read reply accepted.
);
    // Local parameters.
    localparam IFRS_LEN = 64;
    localparam RDRPL_Q_LEN = 4;

    // Buffer to hold the in flight requests.
    logic                           ifr_we;                 // In flight requests buffer write enable.
    line_acc_req                    ifr_inp;                // In flight requests buffer input.
    line_acc_req [IFRS_LEN-1:0]     ifr_oup ;               // In flight requests buffer output.
    logic [IFRS_LEN-1:0]            ifr_clrp;               // In flight requests buffer clear positions.
    logic [IFRS_LEN-1:0]            ifr_up;                 // In flight requests buffer used positions.
    logic                           ifr_full;               // In flight requests buffer full.

    sipo_buffer #($bits(line_acc_req), IFRS_LEN) ifrs(clk, rst, ifr_we, ifr_inp, ifr_clrp, ifr_oup, ifr_up);  


    // Comb block to handle retirement of requests.
    bit [1:0]                   rd_rtn_sel;      // Read return path select.
    bit [$clog2(IFRS_LEN):0]    rd_rtn_par_ind;  // Read return parent request index.
    always_comb begin
        // Default values.
        fs_prts_oup_rp = 0;
        stg_1_wrp_rp = 0;
        rd_rpl_a = 0;
        ifr_clrp = 0;
        fs_prts_oup_req = 0;

        // Check if there is a read returning. If so then handle the retirement of the request.
        if (rd_rpl_p != 0) begin

            // Priority encoder to favour return path 0 over 1, 1 over 2 & 2 over 3.
            for (integer i = 2; i >= 0; i = i - 1) begin if (rd_rpl_p[i]) rd_rtn_sel = i; end

            // Now find the parent requests location in the in flight buffer.
            for (integer i = 0; i < IFRS_LEN; i = i + 1) begin
                if (rd_rpl[rd_rtn_sel].addr == ifr_oup[i].addr) rd_rtn_par_ind = i;
            end

            // Now setup the retirement for the parent operation.

            // Drive the L1 write port input appropriately.

            stg_1_wrp_req.addr = ifr_oup[rd_rtn_par_ind].addr;  // Assign address.

            if (ifr_oup[rd_rtn_par_ind].rqt) begin              // If a write operation.

                // Loop over the bytes to be written and write them per mask or preserve them.
                for (integer i = 0; i < 15; i = i + 1) begin
                    // If the mask bit is set, write the byte.
                    if (ifr_oup[rd_rtn_par_ind].wmsk[i]) stg_1_wrp_req.dat[(i*8) +:8] = ifr_oup[rd_rtn_par_ind].dat[(i*8) +:8];
                    else stg_1_wrp_req.dat[(i*8) +:8] = rd_rpl[rd_rtn_sel].dat[(i*8) +:8];
                end
            end
            else begin                                          // If a read operation.
                // Assign the read cache line to the write port of stage 1.
                stg_1_wrp_req.dat = rd_rpl[rd_rtn_sel].dat;
            end

            // Now handle ensuring that the request is retired properly.

            // If the request is a read type.
            if (!ifr_oup[rd_rtn_par_ind].rqt) begin

                // Drive the front end port.
                fs_prts_oup_req[ifr_oup[rd_rtn_par_ind].prt] = ifr_oup[rd_rtn_par_ind];
                fs_prts_oup_req[ifr_oup[rd_rtn_par_ind].prt].dat = rd_rpl[rd_rtn_sel].dat;
                fs_prts_oup_rp[ifr_oup[rd_rtn_par_ind].prt] = 1;

                `ifdef CMAC_DEBUG_LOG
                $display("Read request retiring. %t", $time);
                `endif

                // If the front end port & L1 can both accept the retirement.
                if (fs_prts_oup_ra[ifr_oup[rd_rtn_par_ind].prt] && stg_1_wrp_op) begin
                    // Signal to L1 cache that we are sending it a write.
                    stg_1_wrp_rp = 1;

                    // Signal to the stage returning the reply that we have it.
                    rd_rpl_a[rd_rtn_sel] = 1;

                    // Signal to the in flight requests buffer that we want to erase the entry.
                    ifr_clrp[rd_rtn_par_ind] = 1;

                    `ifdef CMAC_DEBUG_LOG
                    $display("Read request is retiring successfully. %t", $time);
                    `endif
                end
            end
            // If the request is not a read type (its a write).
            else begin
                // signal to L1 cache that we want to submit a write operation.
                stg_1_wrp_rp = 1;

                `ifdef CMAC_DEBUG_LOG
                $display("A write request is attempting to retire, %t", $time);
                `endif

                // If the L1 stage can accept the write request.
                if (stg_1_wrp_op) begin
                    // Signal to the rd rpl path that the return is accepted.
                    rd_rpl_a[rd_rtn_sel] = 1;

                    // Signal to clear the entry from the in flight requests buffer.
                    ifr_clrp[rd_rtn_par_ind] = 1;

                    `ifdef CMAC_DEBUG_LOG
                    $display("A write request is retired, %t", $time);
                    `endif
                end
            end

            // On the next clock edge, if there is a read reply & the conditions are met to retire it then it will be retired & the rd_rpl_p
            // signal will go low for the port currently selected and another request could be retired.
        end
    end 


    // Comb block for handling dispatching of requests.
    bit [$clog2(NUM_PORTS):0]   fs_prt_sel;
    logic                       req_conflict;
    always_comb begin
        // Defaults values.
        fs_prts_inp_ra = 0;
        stg_1_rdp_rp = 0;
        ifr_we = 0;
        req_conflict = 0;

        // Check if the in flights buffer is full or not.
        ifr_full = &ifr_up; // Should be a logical and of the entire packed array which would give 1 if all 1s or 0 otherwise.

        // If one of the front side ports is trying to submit a memory access request.
        if (fs_prts_inp_rp != 0) begin

            // Select a port that wants to submit.
            for (integer i = 0; i < NUM_PORTS; i = i + 1) begin if (fs_prts_inp_rp[i]) fs_prt_sel = i; end  // Forgot the [i] for fs_prts_inp_rp and it caused me HOURS of problems :(

            // Check its not conflicting with an inflight request.
            for (int i = 0; i < IFRS_LEN; i = i + 1) begin
                if (ifr_oup[i].addr == fs_prts_inp_req[fs_prt_sel].addr & ifr_up[i]) begin 
                    req_conflict = 1; 
                    `ifdef CMAC_DEBUG_LOG
                    $display("Found a conflict in cmac, %t", $time); 
                    `endif
                end
            end

            // If stage 1 can accept the request & the ifrs buffer isnt full.
            if (stg_1_rdp_op && !ifr_full & !req_conflict) begin
                `ifdef CMAC_DEBUG_LOG
                $display("This should not run if there is a req conflict, %t", $time);
                `endif
                // Signal to the fs port that the request is to be accepted.
                fs_prts_inp_ra[fs_prt_sel] = 1;

                // Create read request to L1 stage.
                stg_1_rdp_req = fs_prts_inp_req[fs_prt_sel].addr;

                // Signal to L1 cache that we have a new request for it.
                stg_1_rdp_rp = 1;

                // Signal to the in flight requests buffer that we want to log the request.
                ifr_we = 1;

                // Present the request to the ifr buffer.
                ifr_inp = fs_prts_inp_req[fs_prt_sel];
                ifr_inp.prt = fs_prt_sel;
            end
        end
    end
endmodule

// Memory access controller L1 cache stage.
module mem_acc_l1_stage(
    input   logic           clk,          // Clock.
    input   logic           rst,          // Reset.

    // Inputs to this stage.

    // Read port.
    input   logic           inp_rdp_rp,   // Read port request present.
    input   line_read_req   inp_rdp_req,  // Read port request.
    output  logic           inp_rdp_op,   // Read port open.

    // Write port.
    input   logic           inp_wrp_rp,   // Write port request present.
    input   line_write_req  inp_wrp_req,  // Write port request.
    output  logic           inp_wrp_op,   // Write port open.

    // Outputs from this stage.

    // Read port.
    output  logic           oup_rdp_rp,   // Read port request present.
    output  line_read_req   oup_rdp_req,  // Read port request.
    input   logic           oup_rdp_op,   // Read port open.

    // Write port.
    output  logic           oup_wrp_rp,   // Write port request present.
    output  line_write_req  oup_wrp_req,  // Write port request.
    input   logic           oup_wrp_op,   // Write port open.

    // Return channel for completed requests.
    output  logic           rd_rpl_p,     // Read reply present.
    output  line_read_reply rd_rpl,       // Read reply.
    input   logic           rd_rpl_a      // Read reply accepted.
);

    localparam RD_RQ_Q_LEN = 4;
    localparam WR_RQ_Q_LEN = 4;
    localparam RD_RP_Q_LEN = 4;

    // Read request queue.
    line_read_req                   rdrq_q_ar;              // Read request queue active request.
    logic                           rdrq_q_we;              // Read request queue write enable.
    logic                           rdrq_q_se;              // Read request queue shift enable.
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_src_num_avail;   // Read request queue source number available.
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_dst_num_avail;   // Read request queue destination number available.
    fifo_sr #($bits(line_read_req), RD_RQ_Q_LEN, 1, 1) rdrq_q(clk, rst, rdrq_q_we, inp_rdp_req, rdrq_q_src_num_avail, rdrq_q_se, rdrq_q_ar, rdrq_q_dst_num_avail);

    // If we have room for another request in the queue then we accept it.
    assign inp_rdp_op = rdrq_q_src_num_avail != 0 ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign rdrq_q_we = inp_rdp_op && inp_rdp_rp; 

    // Write request queue.
    line_write_req                  wrrq_q_ar;                  // Write request queue active request.
    logic                           wrrq_q_we;                  // Write request queue write enable.
    logic                           wrrq_q_se;                  // Write request queue shift enable.
    logic [$clog2(WR_RQ_Q_LEN):0]   wrrq_q_src_num_avail;       // Write request queue source number available.
    logic [$clog2(WR_RQ_Q_LEN):0]   wrrq_q_dst_num_avail;       // Write request queue destination number available.
    fifo_sr #($bits(line_write_req), WR_RQ_Q_LEN, 1, 1) wrrq_q(clk, rst, wrrq_q_we, inp_wrp_req, wrrq_q_src_num_avail, wrrq_q_se, wrrq_q_ar, wrrq_q_dst_num_avail);

    // If we have room for another request in the queue then we accept it.
    assign inp_wrp_op = wrrq_q_src_num_avail != 0 ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign wrrq_q_we = inp_wrp_op && inp_wrp_rp;

    // Read reply queue.
    line_read_reply                 rdrply_q_ar;                // Read reply queue active request.
    line_read_reply                 rdrply_q_inp;               // Read reply queue input.
    logic                           rdrply_q_we;                // Read reply queue write enable.
    logic                           rdrply_q_se;                // Read reply queue shift enable.
    logic [$clog2(RD_RP_Q_LEN):0]   rdrply_q_src_num_avail;     // Read reply queue source number available.
    logic [$clog2(RD_RP_Q_LEN):0]   rdrply_q_dst_num_avail;     // Read reply queue destination number available.
    fifo_sr #($bits(line_read_reply), RD_RP_Q_LEN, 1, 1) rdrp_q(clk, rst, rdrply_q_we, rdrply_q_inp, rdrply_q_src_num_avail, rdrply_q_se, rdrply_q_ar, rdrply_q_dst_num_avail);

    // If the read reply queue is longer than 0 (it has a reply in it waiting to be sent).
    assign rd_rpl_p = rdrply_q_dst_num_avail != 0 ? 1 : 0;
    // Assign the active request from the read reply queue to drive the reply bit.
    assign rd_rpl = rdrply_q_ar;
    // If the retirement stage accepts the reply, shift it out of the queue.
    assign rdrply_q_se = rd_rpl_a;

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
                                  cache_bs_addr, cache_bs_dat, cache_bs_en, cache_bs_done);


    // Comb block to handle dispatching write & read requests.
    always_comb begin
        // Default values.
        cache_rdp_en = 0;
        cache_wrp_en = 0;
        wrrq_q_se = 0;
        rdrq_q_se = 0;
        rdrply_q_we = 0;
        oup_rdp_rp = 0;
        oup_wrp_rp = 0;

        // If there is a write request present in the queue.
        if (wrrq_q_dst_num_avail != 0) begin
            // Enable the write port on the cache and present the request.
            cache_wrp_en = 1;
            cache_wrp_dat = wrrq_q_ar.dat;
            cache_wrp_addr = wrrq_q_ar.addr;

            // If the backside of the cache wants to evict a line.
            if (cache_bs_en) begin
                // Signal to the write port of the next stage that we have a write request for it.
                oup_wrp_req.dat = cache_bs_dat;
                oup_wrp_req.addr = cache_bs_addr;
                oup_wrp_rp = 1;

                // If the next stage' write port will accept the request then we signal back end done.
                cache_bs_done = oup_wrp_op;
            end

            // If the cache is done.
            if (cache_wrp_done) begin
                // If the operation was a success.
                if (cache_wrp_suc) begin
                    wrrq_q_se = 1;  // Shift to the next request.
                end

                // If the operation was not a success.
                else begin
                    // Add something here to either pass the request along or throw a hardware error or something.
                end
            end
        end

        // If there is a read request present in the queue.
        if (rdrq_q_dst_num_avail != 0) begin
            // Enable the read port on the cache and present the request.
            cache_rdp_en = 1;
            cache_rdp_addr = rdrq_q_ar.addr;

            // If the cache is done.
            if (cache_rdp_done) begin
                // If the operation was a success.
                if (cache_rdp_suc) begin
                    // Present the data to the read reply queue for it to go down the return path.
                    rdrply_q_inp.dat = cache_rdp_dat;
                    rdrply_q_inp.addr = rdrq_q_ar.addr;

                    // If the read reply queue can accept the request on next cycle.
                    if (rdrply_q_src_num_avail != 0) begin
                        // Signal we want to write it.
                        rdrply_q_we = 1;
                        // Signal to shift the req out of the read request queue.
                        rdrq_q_se = 1;
                    end

                    // If we cant accept the reply to the retirement queue then the cache read port will just sit idle.
                end

                // If the operation was not a success, forward it to the next stage.
                else begin
                    // Present the request to the other stage and wait for it to accept it.
                    oup_rdp_req = rdrq_q_ar;
                    oup_rdp_rp = 1;
                    
                    // If the next stage will accept the request next clock cycle then we can shift it out then too.
                    rdrq_q_se = oup_rdp_op;
                end
            end
        end
    end
endmodule

// Memory access controller L2 cache stage.
module mem_acc_l2_stage(
    input clk,
    input rst,

    // Inputs to this stage.

    // Read port.
    input logic            inp_rdp_rp,   // Read port request present.
    input line_read_req    inp_rdp_req,  // Read port request.
    output logic           inp_rdp_op,   // Read port open.

    // Write port.
    input logic            inp_wrp_rp,   // Write port request present.
    input line_write_req   inp_wrp_req,  // Write port request.
    output logic           inp_wrp_op,   // Write port open.

    // Outputs from this stage.

    // Read port.
    output logic            oup_rdp_rp,   // Read port request present.
    output line_read_req    oup_rdp_req,  // Read port request.
    input logic             oup_rdp_op,   // Read port open.

    // Write port.
    output logic            oup_wrp_rp,   // Write port request present.
    output line_write_req   oup_wrp_req,  // Write port request.
    input logic             oup_wrp_op,   // Write port open.

    // Return channel for completed requests.
    output logic rd_rpl_p,
    output line_read_reply rd_rpl,
    input logic rd_rpl_a
);

    localparam RD_RQ_Q_LEN = 4;
    localparam WR_RQ_Q_LEN = 4;
    localparam RD_RP_Q_LEN = 4;

        // Read request queue.
    line_read_req                   rdrq_q_ar;              // Read request queue active request.
    logic                           rdrq_q_we;              // Read request queue write enable.
    logic                           rdrq_q_se;              // Read request queue shift enable.
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_src_num_avail;   // Read request queue source number available.
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_dst_num_avail;   // Read request queue destination number available.
    fifo_sr #($bits(line_read_req), RD_RQ_Q_LEN, 1, 1) rdrq_q(clk, rst, rdrq_q_we, inp_rdp_req, rdrq_q_src_num_avail, rdrq_q_se, rdrq_q_ar, rdrq_q_dst_num_avail);

    // If we have room for another request in the queue then we accept it.
    assign inp_rdp_op = rdrq_q_src_num_avail != 0 ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign rdrq_q_we = inp_rdp_op && inp_rdp_rp; 

    // Write request queue.
    line_write_req                  wrrq_q_ar;                  // Write request queue active request.
    logic                           wrrq_q_we;                  // Write request queue write enable.
    logic                           wrrq_q_se;                  // Write request queue shift enable.
    logic [$clog2(WR_RQ_Q_LEN):0]   wrrq_q_src_num_avail;
    logic [$clog2(WR_RQ_Q_LEN):0]   wrrq_q_dst_num_avail;
    fifo_sr #($bits(line_write_req), WR_RQ_Q_LEN, 1, 1) wrrq_q(clk, rst, wrrq_q_we, inp_wrp_req, wrrq_q_src_num_avail, wrrq_q_se, wrrq_q_ar, wrrq_q_dst_num_avail);

    // If we have room for another request in the queue then we accept it.
    assign inp_wrp_op = wrrq_q_src_num_avail != 0 ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign wrrq_q_we = inp_wrp_op && inp_wrp_rp;

    // Read reply queue.
    line_read_reply                 rdrply_q_ar;                // Read reply queue active request.
    line_read_reply                 rdrply_q_inp;               // Read reply queue input.
    logic                           rdrply_q_we;                // Read reply queue write enable.
    logic                           rdrply_q_se;                // Read reply queue shift enable.
    logic [$clog2(RD_RP_Q_LEN):0]   rdrply_q_src_num_avail;     // Read reply queue source number available.
    logic [$clog2(RD_RP_Q_LEN):0]   rdrply_q_dst_num_avail;     // Read reply queue destination number available.
    fifo_sr #($bits(line_read_reply), RD_RP_Q_LEN, 1, 1) rdrp_q(clk, rst, rdrply_q_we, rdrply_q_inp, rdrply_q_src_num_avail, rdrply_q_se, rdrply_q_ar, rdrply_q_dst_num_avail);

    // If the read reply queue is longer than 0 (it has a reply in it waiting to be sent).
    assign rd_rpl_p = rdrply_q_dst_num_avail != 0 ? 1 : 0;
    // Assign the active request from the read reply queue to drive the reply bit.
    assign rd_rpl = rdrply_q_ar;
    // If the retirement stage accepts the reply, shift it out of the queue.
    assign rdrply_q_se = rd_rpl_a;

    // Instantiate an L2 cache block.
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

    l2_cache #(8, 256, 16, 32) l1(clk, rst, cache_rdp_addr, cache_rdp_en, cache_rdp_dat, cache_rdp_done, cache_rdp_suc,
                                  cache_wrp_addr, cache_wrp_en, cache_wrp_dat, cache_wrp_done, cache_wrp_suc,
                                  cache_bs_addr, cache_bs_dat, cache_bs_en, cache_bs_done);

    // Comb block to handle dispatching write & read requests.
    always_comb begin
        // Default values.
        cache_rdp_en = 0;
        cache_wrp_en = 0;
        wrrq_q_se = 0;
        rdrq_q_se = 0;
        rdrply_q_we = 0;

        oup_wrp_rp = 0;
        oup_rdp_rp = 0;


        //If there is a write request present in the queue.
        if (wrrq_q_dst_num_avail != 0) begin
            // Enable the write port on the cache & present the request.
            cache_wrp_en = 1;
            cache_wrp_dat = wrrq_q_ar.dat;
            cache_wrp_addr = wrrq_q_ar.addr;

            // Logic to connect eviction to the output write port of this stage.
            oup_wrp_req.dat = cache_bs_dat;
            oup_wrp_req.addr = cache_bs_addr;
            oup_wrp_rp = cache_bs_en;
            cache_bs_done = oup_wrp_op;

            // If the cache is done.
            if (cache_wrp_done) begin
                // If success.
                if (cache_wrp_suc) begin
                    // Shift the request out.
                    wrrq_q_se = 1;
                    // Disable the write port for a single clock cycle.
                    cache_wrp_en = 0;
                end
                // If not success.
                else begin
                    // Forward the write to a higher level of cache / memory.
                    oup_wrp_req.dat = wrrq_q_ar.dat;
                    oup_wrp_req.addr = wrrq_q_ar.addr;
                    oup_wrp_rp = 1;

                    // If the request is to be forwarded successfully.
                    if (oup_wrp_op) begin
                        // Signal to shift the write request queue.
                        wrrq_q_se = 1;

                        // Disable the cache write port.
                        cache_wrp_en = 0;
                    end
                end
            end
        end


        // If there is a read request present in the queue.
        if (rdrq_q_dst_num_avail != 0) begin
            // Enable the read port on the cache and present the request.
            cache_rdp_en = 1;
            cache_rdp_addr = rdrq_q_ar.addr;

            // If the cache is done.
            if (cache_rdp_done) begin
                // If the operation was a success.
                if (cache_rdp_suc) begin
                    // Present the data to the read reply queue.
                    rdrply_q_inp.dat = cache_rdp_dat;
                    rdrply_q_inp.addr = rdrq_q_ar.addr;

                    // Signal to the read reply queue to write in the data.
                    rdrply_q_we = 1;

                    // If the read reply queue can accept the reply.
                    if (rdrply_q_src_num_avail != 0) begin
                        // Shift the request out of the read request queue.
                        rdrq_q_se = 1;
                        // Disable the cache read port for a single cycle.
                        cache_rdp_en = 0;
                    end
                    else rdrq_q_se = 0;
                end

                // If the operation was not a success, forward it to the next stage.
                else begin
                    // Present the request to the other stage and wait for it to accept it.
                    oup_rdp_req = rdrq_q_ar;
                    oup_rdp_rp = 1;

                    // If the output read request port can accept the request.
                    if (oup_rdp_op) begin
                        // Shift to the next request.
                        rdrq_q_se = 1;
                        // Disable the cache read port for a single cycle.
                        cache_rdp_en = 0;
                    end
                end
            end
        end
    end
endmodule

// Memory access controller NOC stage.
// Might also be able to add a snoop filter here later for full coherence.
module mem_acc_noc_stage(
    input logic             clk,            // Clock.
    input logic             rst,            // Clock.

    // Inputs to this stage.

    // Read port.
    input   logic           inp_rdp_rp,     // Read port request present.
    input   line_read_req   inp_rdp_req,    // Read port request.
    output  logic           inp_rdp_op,     // Read port open.

    // Write port.
    input   logic           inp_wrp_rp,     // Write port request present.
    input   line_write_req  inp_wrp_req,    // Write port request.
    output  logic           inp_wrp_op,     // Write port open.

    // Interface with NIU.
    input   logic [3:0]     prt_addr,       // NIU address.
    input   logic [3:0]     prt_num,        // NIU port number.

    input   logic           rx_av,          // NIU received data available.
    output  logic           rx_re,          // NIU received data read.
    input   noc_packet      rx_dat,         // NIU received data.

    output  logic           tx_av,          // NIU transmit data available.
    input   logic           tx_re,          // NIU transmit data read.
    output  noc_packet      tx_dat,         // NIU transmit data.

    // Return channel for completed requests.
    output  logic           rd_rpl_p,       // Read reply present.
    output  line_read_reply rd_rpl,         // Read reply.
    input   logic           rd_rpl_a        // Read reply accepted.
);

    localparam RD_RQ_Q_LEN = 4;
    localparam WR_RQ_Q_LEN = 4;
    localparam RD_RP_Q_LEN = 4;

        // Read request queue.
    line_read_req                   rdrq_q_ar;              // Read request queue active request.
    logic                           rdrq_q_we;              // Read request queue write enable.
    logic                           rdrq_q_se;              // Read request queue shift enable.
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_src_num_avail;   // Read request queue source number available.
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_dst_num_avail;   // Read request queue destination number available.
    fifo_sr #($bits(line_read_req), RD_RQ_Q_LEN, 1, 1) rdrq_q(clk, rst, rdrq_q_we, inp_rdp_req, rdrq_q_src_num_avail, rdrq_q_se, rdrq_q_ar, rdrq_q_dst_num_avail);

    // If we have room for another request in the queue then we accept it.
    assign inp_rdp_op = rdrq_q_src_num_avail != 0 ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign rdrq_q_we = inp_rdp_op && inp_rdp_rp; 

    // Write request queue.
    line_write_req                  wrrq_q_ar;                  // Write request queue active request.
    logic                           wrrq_q_we;                  // Write request queue write enable.
    logic                           wrrq_q_se;                  // Write request queue shift enable.
    logic [$clog2(WR_RQ_Q_LEN):0]   wrrq_q_src_num_avail;
    logic [$clog2(WR_RQ_Q_LEN):0]   wrrq_q_dst_num_avail;
    fifo_sr #($bits(line_write_req), WR_RQ_Q_LEN, 1, 1) wrrq_q(clk, rst, wrrq_q_we, inp_wrp_req, wrrq_q_src_num_avail, wrrq_q_se, wrrq_q_ar, wrrq_q_dst_num_avail);

    // If we have room for another request in the queue then we accept it.
    assign inp_wrp_op = wrrq_q_src_num_avail != 0 ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign wrrq_q_we = inp_wrp_op && inp_wrp_rp;

    // Read reply queue.
    line_read_reply                 rdrply_q_ar;                // Read reply queue active request.
    line_read_reply                 rdrply_q_inp;               // Read reply queue input.
    logic                           rdrply_q_we;                // Read reply queue write enable.
    logic                           rdrply_q_se;                // Read reply queue shift enable.
    logic [$clog2(RD_RP_Q_LEN):0]   rdrply_q_src_num_avail;     // Read reply queue source number available.
    logic [$clog2(RD_RP_Q_LEN):0]   rdrply_q_dst_num_avail;     // Read reply queue destination number available.
    fifo_sr #($bits(line_read_reply), RD_RP_Q_LEN, 1, 1) rdrp_q(clk, rst, rdrply_q_we, rdrply_q_inp, rdrply_q_src_num_avail, rdrply_q_se, rdrply_q_ar, rdrply_q_dst_num_avail);

    // If the read reply queue is longer than 0 (it has a reply in it waiting to be sent).
    assign rd_rpl_p = rdrply_q_dst_num_avail != 0 ? 1 : 0;
    // Assign the active request from the read reply queue to drive the reply bit.
    assign rd_rpl = rdrply_q_ar;
    // If the retirement stage accepts the reply, shift it out of the queue.
    assign rdrply_q_se = rd_rpl_a;


    //          NOC stuff
    assign tx_dat.hdr.src_addr = prt_addr;
    assign tx_dat.hdr.src_port = prt_num;

    bit snd_r_or_w = 0; // 0 means send read, 1 means send write.

    // Some packed structs to represent the 2 types of packet we could send.
    mem_rd_rq mem_rr;
    mem_wr_rq mem_wr;

    // If 1 assign write req, if 0 assign read req.
    assign tx_dat.dat = snd_r_or_w ? mem_wr : mem_rr;

    // Logic to handle sending requests (read & write) to main memory.
    always_comb begin
        // Default values.
        tx_av = 0;

        wrrq_q_se = 0;
        rdrq_q_se = 0;

        // If we are to be sending a write request.
        if (snd_r_or_w) begin

            // Present the request to the noc port.
            tx_dat.hdr.dst_addr = `MEMORY_INTERFACE_NOC_ADDR;
            tx_dat.hdr.dst_port = `MEMORY_INTERFACE_NOC_PORT;
            tx_dat.hdr.len = ($bits(tx_dat.hdr) + $bits(mem_wr)) / 8;
            mem_wr.addr = wrrq_q_ar.addr;
            mem_wr.dat = wrrq_q_ar.dat;
            mem_wr.pt = memory_write_request;

            // Signal that the request is present if one actually is.
            if (wrrq_q_dst_num_avail != 0) begin
                tx_av = 1;

                // If the noc port will accept the request on next clock, then shift the write request queue.
                wrrq_q_se = tx_re;
            end
        end

        // If we are to be sending a read request.
        else begin

            // Present the request to the noc port.
            tx_dat.hdr.dst_addr = `MEMORY_INTERFACE_NOC_ADDR;
            tx_dat.hdr.dst_port = `MEMORY_INTERFACE_NOC_PORT;
            tx_dat.hdr.len = ($bits(noc_packet_header) + $bits(mem_rd_rq)) / 8;
            mem_rr.addr = rdrq_q_ar;
            mem_rr.pt = memory_read_request;

            `ifdef CMAC_DEBUG_LOG
            $display("Size of read req is %d, %d", $bits(tx_dat.hdr), $bits(mem_rr));
            `endif
            // Signal that the request is present if one actually is.
            if (rdrq_q_dst_num_avail != 0) begin
                tx_av = 1;

                // If the noc port will accept the request on next clock, then shift the read request queue.
                if (tx_re) begin
                    rdrq_q_se = 1;
                end
                else rdrq_q_se = 0;
            end
        end
    end

    // Logic to handle alternating between sending off write requests & read requests.
    always @(posedge clk) begin
        // If last cycle a write was sent.
        if (snd_r_or_w) begin
            // If there is a read request to be sent next cycle.
            if (rdrq_q_dst_num_avail != 0) snd_r_or_w <= 0;
        end
        // If last cycle a read was sent.
        else begin
            // If there is a write request to be sent next cycle.
            if (wrrq_q_dst_num_avail != 0) snd_r_or_w <= 1;
        end
    end


    // Logic to handle recieveing replies from the NOC.
    always_comb begin
        // Default values.
        rx_re = 0;
        rdrply_q_we = 0;
        rdrply_q_inp = 0;

        // If the noc port has an rx for us.
        if (rx_av) begin
            // If the packet is a read reply.
            if (rx_dat.dat[7:0] == memory_read_reply) begin
                // Present data to the read reply queue.
                rdrply_q_inp.addr = rx_dat.dat[136+:32];     // Pass the address data.
                rdrply_q_inp.dat = rx_dat.dat[8+:128];    // Pass the line data.

                // If there is room to shift into the read reply queue.
                if (rdrply_q_src_num_avail != 0) begin
                    // Signal to the read reply queue that data is available.
                    rdrply_q_we = 1;
                    // Signal to the noc port that we have read the rx.
                    rx_re = 1;
                end
            end
        end
    end
endmodule


// Memory access controller.
module mem_acc_cont #(parameter NUM_PORTS = 2)(
    input logic                         clk,                // Clock.
    input logic                         rst,                // Reset.

    // Front end (core side) ports that requests are submitted to and replied from.
       
    // Request submission.
    input  logic [NUM_PORTS-1:0]        fs_prts_inp_rp,     // Front end ports input request present.
    input  line_acc_req [NUM_PORTS-1:0] fs_prts_inp_req,    // Front end ports input request.
    output logic [NUM_PORTS-1:0]        fs_prts_inp_ra,     // Front end ports input open.

    // Request reply.
    output logic [NUM_PORTS-1:0]        fs_prts_oup_rp,     // Front end ports output request present.
    output line_acc_req [NUM_PORTS-1:0] fs_prts_oup_req,    // Front end ports output request.
    input  logic [NUM_PORTS-1:0]        fs_prts_oup_ra,     // Front end ports output open.


    // Interface with NIU.
    input   logic [3:0]                 prt_addr,           // NIU address.
    input   logic [3:0]                 prt_num,            // NIU port number.

    input   logic                       rx_av,              // NIU received data available.
    output  logic                       rx_re,              // NIU received data read.
    input   noc_packet                  rx_dat,             // NIU received data.

    output  logic                       tx_av,              // NIU transmit data available.
    input   logic                       tx_re,              // NIU transmit data read.
    output  noc_packet                  tx_dat              // NIU transmit data.
);

    // request scheduler stage.
    logic               stg_1_rdp_rp;   // Stage 1 read port request present.
    line_read_req       stg_1_rdp_req;  // Stage 1 read port request.
    logic               stg_1_rdp_op;   // Stage 1 read port open.

    logic               stg_1_wrp_rp;   // Stage 1 write port request present.
    line_write_req      stg_1_wrp_req;  // Stage 1 write port request.
    logic               stg_1_wrp_op;   // Stage 1 write port open.

    logic [3]           rd_rpl_p;       // Read reply present.
    line_read_reply [3] rd_rpl;         // Read reply.
    logic [3]           rd_rpl_a;       // Read reply accepted.

    mem_acc_scheduler #(NUM_PORTS) scheduler(clk, rst, 
    fs_prts_inp_rp, fs_prts_inp_req, fs_prts_inp_ra, fs_prts_oup_rp, fs_prts_oup_req, fs_prts_oup_ra,
    stg_1_rdp_rp, stg_1_rdp_req, stg_1_rdp_op, stg_1_wrp_rp, stg_1_wrp_req, stg_1_wrp_op,
    rd_rpl_p, rd_rpl, rd_rpl_a);


    // L1 cache stage.
    logic               stg_2_rdp_rp;   // Stage 2 read port request present.
    line_read_req       stg_2_rdp_req;  // Stage 2 read port request.
    logic               stg_2_rdp_op;   // Stage 2 read port open.

    logic               stg_2_wrp_rp;   // Stage 2 write port request present.
    line_write_req      stg_2_wrp_req;  // Stage 2 write port request.
    logic               stg_2_wrp_op;   // Stage 2 write port open.

    mem_acc_l1_stage l1_cache(clk, rst, 
    stg_1_rdp_rp, stg_1_rdp_req, stg_1_rdp_op, stg_1_wrp_rp, stg_1_wrp_req, stg_1_wrp_op,
    stg_2_rdp_rp, stg_2_rdp_req, stg_2_rdp_op, stg_2_wrp_rp, stg_2_wrp_req, stg_2_wrp_op,
    rd_rpl_p[0], rd_rpl[0], rd_rpl_a[0]);


    // L2 cache.
    logic               stg_3_rdp_rp;   // Stage 3 read port request present.
    line_read_req       stg_3_rdp_req;  // Stage 3 read port request.
    logic               stg_3_rdp_op;   // Stage 3 read port open.

    logic               stg_3_wrp_rp;   // Stage 3 write port request present.
    line_write_req      stg_3_wrp_req;  // Stage 3 write port request.
    logic               stg_3_wrp_op;   // Stage 3 write port open.

    mem_acc_l2_stage l2_cache(clk, rst, 
    stg_2_rdp_rp, stg_2_rdp_req, stg_2_rdp_op, stg_2_wrp_rp, stg_2_wrp_req, stg_2_wrp_op,
    stg_3_rdp_rp, stg_3_rdp_req, stg_3_rdp_op, stg_3_wrp_rp, stg_3_wrp_req, stg_3_wrp_op,
    rd_rpl_p[1], rd_rpl[1], rd_rpl_a[1]);


    // NOC interface stage.

    mem_acc_noc_stage noc_stage(clk, rst,
    stg_3_rdp_rp, stg_3_rdp_req, stg_3_rdp_op, stg_3_wrp_rp, stg_3_wrp_req, stg_3_wrp_op,
    prt_addr, prt_num, rx_av, rx_re, rx_dat, tx_av, tx_re, tx_dat,
    rd_rpl_p[2], rd_rpl[2], rd_rpl_a[2]);
endmodule
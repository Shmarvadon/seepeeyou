`include "defines.svh"
`include "structs.sv"

// Each cache is 2 port: 1 read, 1 write.
// Each cache has a seperate read & write queue.

// Struct to store the currently active requests.
typedef struct packed {
    logic [31:0]    addr;   // Request address.
    logic [127:0]   dat;    // Request data.
    logic [15:0]    wmsk;   // Request write mask.
    logic           rqt;    // Request type (0 = read, 1 = write).
    logic [3:0]     prt;    // Request origin port.
} line_acc_req;

// Struct to pass a line read request.
typedef struct packed {
    logic [31:0] addr;
} line_read_req;

// Struct to pass a line read reply.
typedef struct packed {
    logic [31:0] addr;
    logic [127:0] dat;
} line_read_reply;

// Struct to pass line write request.
typedef struct packed {
    logic [31:0] addr;
    logic [127:0] dat;
} line_write_req;

// Memory access controller scheduler stage.
module mem_acc_scheduler #(parameter NUM_PORTS = 2) (
    input clk,
    input rst,

    // Communicate with the front end ports.

    // Request submission.
    input  logic [NUM_PORTS-1:0]         fs_prts_inp_rp,     // Front end ports input request present.
    input  line_acc_req [NUM_PORTS-1:0]  fs_prts_inp_req,    // Front end ports input request.
    output logic [NUM_PORTS-1:0]         fs_prts_inp_op,     // Front end ports input open.

    // Request reply.
    output logic [NUM_PORTS-1:0]         fs_prts_oup_rp,     // Front end ports output request present.
    output line_acc_req [NUM_PORTS-1:0]  fs_prts_oup_req,    // Front end ports output request.
    input logic [NUM_PORTS-1:0]          fs_prts_oup_op,     // Front end ports output open.


    // Input to stage 1 of mem access controller.

    // Read port.
    output logic            stg_1_rdp_rp,   // Stage 1 read port request present.
    output line_read_req    stg_1_rdp_req,  // Stage 1 read port request.
    input  logic            stg_1_rdp_op,   // Stage 1 read port open.

    // Write port.
    output logic            stg_1_wrp_rp,   // Stage 1 write port request present.
    output line_write_req   stg_1_wrp_req,  // Stage 1 write port request.
    input  logic            stg_1_wrp_op,   // Stage 1 write port open.


    // Output from the L1, L2 & NOC stages returning read replies.
    input logic [3]             rd_rpl_p,   // Read reply present.
    input line_read_reply [3]   rd_rpl,     // Read reply.
    output logic [3]            rd_rpl_a    // Read reply accepted.
);
    // Local parameters.
    localparam IFRS_LEN = 64;
    localparam RDRPL_Q_LEN = 4;

    // Buffer to hold the in flight requests.
    logic                           ifr_we;                 // In flight requests buffer write enable.
    line_acc_req                    ifr_inp;                // In flight requests buffer input.
    line_acc_req                    ifr_oup [IFRS_LEN-1:0]; // In flight requests buffer output.
    logic [IFRS_LEN-1:0]            ifr_clrp;               // In flight requests buffer clear positions.
    logic [IFRS_LEN-1:0]            ifr_up;                 // In flight requests buffer used positions.
    logic                           ifr_full;               // In flight requests buffer full.

    sipo_buffer #($bits(line_acc_req), IFRS_LEN) ifrs(clk, rst, ifr_we, ifr_inp, ifr_clrp, ifr_oup, ifr_up);  


    // Comb block to handle retirement of requests.
    bit [1:0] rd_rtn_sel;   // Read return path select.
    bit [$clog2(IFRS_LEN):0] rd_rtn_par_ind;  // Read return parent request index.
    always_comb begin
        // Default values.
        fs_prts_oup_rp = 0;
        stg_1_wrp_rp = 0;
        rd_rpl_a = 0;
        ifr_clrp = 0;
        //fs_prts_oup_req = 0;

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
                fs_prts_oup_req[ifr_oup[rd_rtn_par_ind].prt].dat = stg_1_wrp_req.dat;

                $display("Read request retiring. %t", $time);

                // If the front end port & L1 can both accept the retirement.
                if (fs_prts_oup_op[ifr_oup[rd_rtn_par_ind].prt] && stg_1_wrp_op) begin
                    // Signal to the front end that we are sending it a reply.
                    fs_prts_oup_rp[ifr_oup[rd_rtn_par_ind].prt] = 1;
                    // Signal to L1 cache that we are sending it a write.
                    stg_1_wrp_rp = 1;

                    // Signal to the stage returning the reply that we have it.
                    rd_rpl_a[rd_rtn_sel] = 1;

                    // Signal to the in flight requests buffer that we want to erase the entry.
                    ifr_clrp[rd_rtn_par_ind] = 1;

                    $display("Read request is retiring successfully. %t", $time);
                end
            end
            // If the request is not a read type (its a write).
            else begin
                // signal to L1 cache that we want to submit a write operation.
                stg_1_wrp_rp = 1;

                $display("A write request is attempting to retire, %t", $time);

                // If the L1 stage can accept the write request.
                if (stg_1_wrp_op) begin
                    // Signal to the rd rpl path that the return is accepted.
                    rd_rpl_a[rd_rtn_sel] = 1;

                    // Signal to clear the entry from the in flight requests buffer.
                    ifr_clrp[rd_rtn_par_ind] = 1;

                    $display("A write request is retired, %t", $time);
                end
            end

            // On the next clock edge, if there is a read reply & the conditions are met to retire it then it will be retired & the rd_rpl_p
            // signal will go low for the port currently selected and another request could be retired.
        end
    end 


    // Comb block for handling dispatching of requests.
    bit [$clog2(NUM_PORTS):0] fs_prt_sel;
    always_comb begin
        // Defaults values.
        fs_prts_inp_op = 0;
        stg_1_rdp_rp = 0;
        ifr_we = 0;

        // Check if the in flights buffer is full or not.
        ifr_full = &ifr_up; // Should be a logical and of the entire packed array which would give 1 if all 1s or 0 otherwise.

        // If one of the front side ports is trying to submit a memory access request.
        if (fs_prts_inp_rp != 0) begin

            // Select a port that wants to submit.
            for (integer i = 0; i < NUM_PORTS; i = i + 1) begin if (fs_prts_inp_rp[i]) fs_prt_sel = i; end  // Forgot the [i] for fs_prts_inp_rp and it caused me HOURS of problems :(


            // If stage 1 can accept the request & the ifrs buffer isnt full.
            if (stg_1_rdp_op && !ifr_full) begin
                // Signal to the fs port that the request is to be accepted.
                fs_prts_inp_op[fs_prt_sel] = 1;

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
    line_read_req                   rdrq_q_ar;  // Read request queue active request.
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_len; // Read request queue length.
    logic                           rdrq_q_we;  // Read request queue write enable.
    logic                           rdrq_q_se;  // Read request queue shift enable.
    fifo_queue #($bits(line_read_req), RD_RQ_Q_LEN) rdrq_q(clk, rst, inp_rdp_req, rdrq_q_ar, rdrq_q_len, rdrq_q_we, rdrq_q_se);

    // If we have room for another request in the queue then we accept it.
    assign inp_rdp_op = rdrq_q_len < RD_RQ_Q_LEN ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign rdrq_q_we = inp_rdp_op && inp_rdp_rp; 

    // Write request queue.
    line_write_req                  wrrq_q_ar;  // Write request queue active request.
    logic [$clog2(WR_RQ_Q_LEN):0] wrrq_q_len; // Write request queue length.
    logic                           wrrq_q_we;  // Write request queue write enable.
    logic                           wrrq_q_se;  // Write request queue shift enable.
    fifo_queue #($bits(line_write_req), WR_RQ_Q_LEN) wrrq_q(clk, rst, inp_wrp_req, wrrq_q_ar, wrrq_q_len, wrrq_q_we, wrrq_q_se);

    // If we have room for another request in the queue then we accept it.
    assign inp_wrp_op = wrrq_q_len < WR_RQ_Q_LEN ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign wrrq_q_we = inp_wrp_op && inp_wrp_rp;

    // Read reply queue.
    line_read_reply                 rdrply_q_ar;    // Read reply queue active request.
    line_read_reply                 rdrply_q_inp;   // Read reply queue input.
    logic [$clog2(RD_RP_Q_LEN):0] rdrply_q_len;   // Read reply queue length.
    logic                           rdrply_q_we;    // Read reply queue write enable.
    logic                           rdrply_q_se;    // Read reply queue shift enable.
    fifo_queue #($bits(line_read_reply), RD_RP_Q_LEN) rdrp_q(clk, rst, rdrply_q_inp, rdrply_q_ar, rdrply_q_len, rdrply_q_we, rdrply_q_se);

    // If the read reply queue is longer than 0 (it has a reply in it waiting to be sent).
    assign rd_rpl_p = rdrply_q_len > 0 ? 1 : 0;
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
        if (wrrq_q_len != 0) begin
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
        if (rdrq_q_len != 0) begin
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
                    if (rdrply_q_len < RD_RP_Q_LEN) begin
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
    line_read_req                   rdrq_q_ar;  // Read request queue active request.   
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_len; // Read request queue length.
    logic                           rdrq_q_we;  // Read request queue write enable.
    logic                           rdrq_q_se;  // Read request queue shift enable.
    fifo_queue #($bits(line_read_req), RD_RQ_Q_LEN) rdrq_q(clk, rst, inp_rdp_req, rdrq_q_ar, rdrq_q_len, rdrq_q_we, rdrq_q_se);

    // If we have room for another request in the queue then we accept it.
    assign inp_rdp_op = rdrq_q_len < RD_RQ_Q_LEN ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign rdrq_q_we = inp_rdp_op && inp_rdp_rp; 

    // Write request queue.
    line_write_req                  wrrq_q_ar;  // Write request queue active request.
    logic [$clog2(WR_RQ_Q_LEN):0] wrrq_q_len; // Write request queue length.
    logic                           wrrq_q_we;  // Write request queue write enable.
    logic                           wrrq_q_se;  // Write request queue shift enable.
    fifo_queue #($bits(line_write_req), WR_RQ_Q_LEN) wrrq_q(clk, rst, inp_wrp_req, wrrq_q_ar, wrrq_q_len, wrrq_q_we, wrrq_q_se);

    // If we have room for another request in the queue then we accept it.
    assign inp_wrp_op = wrrq_q_len < WR_RQ_Q_LEN ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign wrrq_q_we = inp_wrp_op && inp_wrp_rp;

    // Read reply queue.
    line_read_reply                 rdrply_q_ar;    // Read reply queue active request.
    line_read_reply                 rdrply_q_inp;   // Read reply queue input.
    logic [$clog2(RD_RP_Q_LEN):0] rdrply_q_len;   // Read reply queue length.
    logic                           rdrply_q_we;    // Read reply queue write enable.
    logic                           rdrply_q_se;    // Read reply queue shift enable.
    fifo_queue #($bits(line_read_reply), RD_RP_Q_LEN) rdrp_q(clk, rst, rdrply_q_inp, rdrply_q_ar, rdrply_q_len, rdrply_q_we, rdrply_q_se);

    // If the read reply queue is longer than 0 (it has a reply in it waiting to be sent).
    assign rd_rpl_p = rdrply_q_len > 0 ? 1 : 0;
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
        if (wrrq_q_len != 0) begin
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
        if (rdrq_q_len != 0) begin
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
                    if (rdrply_q_len != RD_RP_Q_LEN) begin
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

    noc_ip_port.ip_side noc_port,

    // Return channel for completed requests.
    output logic rd_rpl_p,
    output line_read_reply rd_rpl,
    input logic rd_rpl_a
);

    localparam RD_RQ_Q_LEN = 4;
    localparam WR_RQ_Q_LEN = 4;
    localparam RD_RP_Q_LEN = 4;

    // Read request queue.
    line_read_req                   rdrq_q_ar;  // Read request queue active request.   
    logic [$clog2(RD_RQ_Q_LEN):0]   rdrq_q_len; // Read request queue length.
    logic                           rdrq_q_we;  // Read request queue write enable.
    logic                           rdrq_q_se;  // Read request queue shift enable.
    fifo_queue #($bits(line_read_req), RD_RQ_Q_LEN) rdrq_q(clk, rst, inp_rdp_req, rdrq_q_ar, rdrq_q_len, rdrq_q_we, rdrq_q_se);

    // If we have room for another request in the queue then we accept it.
    assign inp_rdp_op = rdrq_q_len < RD_RQ_Q_LEN ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign rdrq_q_we = inp_rdp_op && inp_rdp_rp; 

    // Write request queue.
    line_write_req                  wrrq_q_ar;  // Write request queue active request.
    logic [$clog2(WR_RQ_Q_LEN):0]   wrrq_q_len; // Write request queue length.
    logic                           wrrq_q_we;  // Write request queue write enable.
    logic                           wrrq_q_se;  // Write request queue shift enable.
    fifo_queue #($bits(line_write_req), WR_RQ_Q_LEN) wrrq_q(clk, rst, inp_wrp_req, wrrq_q_ar, wrrq_q_len, wrrq_q_we, wrrq_q_se);

    // If we have room for another request in the queue then we accept it.
    assign inp_wrp_op = wrrq_q_len < WR_RQ_Q_LEN ? 1 : 0;
    // If we have room & the prev stage wants to input a request to queue, accept it.
    assign wrrq_q_we = inp_wrp_op && inp_wrp_rp;

    // Read reply queue.
    line_read_reply                 rdrply_q_ar;    // Read reply queue active request.
    line_read_reply                 rdrply_q_inp;   // Read reply queue input.
    logic [$clog2(RD_RP_Q_LEN):0]  rdrply_q_len;   // Read reply queue length.
    logic                           rdrply_q_we;    // Read reply queue write enable.
    logic                           rdrply_q_se;    // Read reply queue shift enable.
    fifo_queue #($bits(line_read_reply), RD_RP_Q_LEN) rdrp_q(clk, rst, rdrply_q_inp, rdrply_q_ar, rdrply_q_len, rdrply_q_we, rdrply_q_se);

    // If the read reply queue is longer than 0 (it has a reply in it waiting to be sent).
    assign rd_rpl_p = rdrply_q_len > 0 ? 1 : 0;
    // Assign the active request from the read reply queue to drive the reply bit.
    assign rd_rpl = rdrply_q_ar;
    // If the retirement stage accepts the reply, shift it out of the queue.
    assign rdrply_q_se = rd_rpl_a;


    //          NOC stuff
    assign noc_port.tx_dat.hdr.src_addr = noc_port.prt_addr;
    assign noc_port.tx_dat.hdr.src_port = noc_port.prt_num;

    bit snd_r_or_w = 0; // 0 means send read, 1 means send write.

    // Some packed structs to represent the 2 types of packet we could send.
    mem_rd_rq mem_rr;
    mem_wr_rq mem_wr;

    // If 1 assign write req, if 0 assign read req.
    assign noc_port.tx_dat.dat = snd_r_or_w ? mem_wr : mem_rr;

    // Logic to handle sending requests (read & write) to main memory.
    always_comb begin
        // Default values.
        noc_port.tx_av = 0;

        wrrq_q_se = 0;
        rdrq_q_se = 0;

        // If we are to be sending a write request.
        if (snd_r_or_w) begin

            // Present the request to the noc port.
            noc_port.tx_dat.hdr.dst_addr = `MEMORY_INTERFACE_NOC_ADDR;
            noc_port.tx_dat.hdr.dst_port = `MEMORY_INTERFACE_NOC_PORT;
            noc_port.tx_dat.hdr.len = $bits(noc_port.tx_dat.hdr) + $bits(mem_wr);
            mem_wr.addr = wrrq_q_ar.addr;
            mem_wr.dat = wrrq_q_ar.dat;
            mem_wr.pt = memory_write_request;

            // Signal that the request is present if one actually is.
            if (wrrq_q_len != 0) begin
                noc_port.tx_av = 1;

                // If the noc port will accept the request on next clock, then shift the write request queue.
                wrrq_q_se = noc_port.tx_re;
            end
        end

        // If we are to be sending a read request.
        else begin

            // Present the request to the noc port.
            noc_port.tx_dat.hdr.dst_addr = `MEMORY_INTERFACE_NOC_ADDR;
            noc_port.tx_dat.hdr.dst_port = `MEMORY_INTERFACE_NOC_PORT;
            noc_port.tx_dat.hdr.len = $bits(noc_port.tx_dat.hdr) + $bits(mem_rr);
            mem_rr.addr = rdrq_q_ar;
            mem_rr.pt = memory_read_request;

            // Signal that the request is present if one actually is.
            if (rdrq_q_len != 0) begin
                noc_port.tx_av = 1;

                // If the noc port will accept the request on next clock, then shift the read request queue.
                if (noc_port.tx_re) begin
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
            if (rdrq_q_len != 0) snd_r_or_w <= 0;
        end
        // If last cycle a read was sent.
        else begin
            // If there is a write request to be sent next cycle.
            if (wrrq_q_len != 0) snd_r_or_w <= 1;
        end
    end


    // Logic to handle recieveing replies from the NOC.
    always_comb begin
        // Default values.
        noc_port.rx_re = 0;
        rdrply_q_we = 0;

        // If the noc port has an rx for us.
        if (noc_port.rx_av) begin
            // If the packet is a read reply.
            if (noc_port.rx_dat.dat[7:0] == memory_read_reply) begin
                // Present data to the read reply queue.
                rdrply_q_inp.addr = noc_port.rx_dat.dat[136+:32];     // Pass the address data.
                rdrply_q_inp.dat = noc_port.rx_dat.dat[8+:128];    // Pass the line data.

                // If there is room to shift into the read reply queue.
                if (rdrply_q_len < RD_RP_Q_LEN) begin
                    // Signal to the read reply queue that data is available.
                    rdrply_q_we = 1;
                    // Signal to the noc port that we have read the rx.
                    noc_port.rx_re = 1;
                end
            end
        end
    end
endmodule


// Memory access controller.
module mem_acc_cont #(parameter NUM_PORTS = 2)(
    input clk,
    input rst,

    // Front end (core side) ports that requests are submitted to and replied from.
       
    // Request submission.
    input  logic [NUM_PORTS-1:0]         fs_prts_inp_rp,    // Front end ports input request present.
    input  line_acc_req [NUM_PORTS-1:0]  fs_prts_inp_req,   // Front end ports input request.
    output logic [NUM_PORTS-1:0]         fs_prts_inp_op,    // Front end ports input open.

    // Request reply.
    output logic [NUM_PORTS-1:0]         fs_prts_oup_rp,    // Front end ports output request present.
    output line_acc_req [NUM_PORTS-1:0]  fs_prts_oup_req,   // Front end ports output request.
    input logic [NUM_PORTS-1:0]          fs_prts_oup_op,    // Front end ports output open.


    // Back end (noc side) port to interfave with the NOC.

    noc_ip_port.ip_side             noc_port            // Noc interface port.
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
    fs_prts_inp_rp, fs_prts_inp_req, fs_prts_inp_op, fs_prts_oup_rp, fs_prts_oup_req, fs_prts_oup_op,
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
    noc_port,
    rd_rpl_p[2], rd_rpl[2], rd_rpl_a[2]);

endmodule

/*
module memory_access_controller #(parameter NUMBER_OF_PORTS = 2)(
    input clk,
    input rst,

    // Basic interface for testing. ASSUME ALL REQUESTS ARE ALIGNED.
    mac_stage_intf      testing_interface,

    noc_ip_port.ip_side            noc_prt
);
    // request dispatcher interfaces.
    mac_stage_intf req_dispatcher_interfaces[NUMBER_OF_PORTS]();

    

    // Shared successful request return path.
    mem_acc_req [3] rtn_req;  // Successful request(s).
    logic [3] rtn_req_pres;   // Successful request(s) present.
    logic [3] rtn_req_acc;    // Successful request(s) accepted.


    // Stage 0b.
    mac_scheduler #(2) stage_0b(clk, rst, stage_1_intf.drive_input, req_dispatcher_interfaces.drive_output, rtn_req, rtn_req_pres, rtn_req_acc);


    // Stage 1.
    mac_stage_intf  stage_1_intf();     // Stage 1 interface.

    mac_l1_stage stage_1(clk, rst, stage_1_intf.drive_output, rtn_req[0], rtn_req_pres[0], rtn_req_acc[0]);

    // Stage 2.
    mac_stage_intf  stage_2_intf();     // Stage 2 interface.

    mac_l2_stage stage_2(clk, rst, stage_2_intf.drive_output, rtn_req[1], rtn_req_pres[1], rtn_req_acc[1]);

    // Stage 3.
    mac_stage_intf stage_3_intf();      // Stage 3 interface.

    mac_noc_stage stage_3(clk, rst, stage_3_intf.drive_output, rtn_req[2], rtn_req_pres[2], rtn_req_acc[2], noc_prt);

    // Connect stage 1 output to stage 2 input.
    assign stage_2_intf.inp_rp = stage_1_intf.oup_rp;
    assign stage_2_intf.inp_req = stage_1_intf.oup_req;
    assign stage_1_intf.oup_ra = stage_2_intf.inp_ra;

    // Connect stage 2 output to stage 3 input.
    assign stage_3_intf.inp_rp = stage_2_intf.oup_rp;
    assign stage_3_intf.inp_req = stage_2_intf.oup_req;
    assign stage_2_intf.oup_ra = stage_3_intf.inp_ra;


    // Handle driving stage 0b from the testing interface.
    assign req_dispatcher_interfaces[0].inp_rp = testing_interface.inp_rp;
    assign req_dispatcher_interfaces[0].inp_req = testing_interface.inp_req;
    assign testing_interface.inp_ra = req_dispatcher_interfaces[0].inp_ra;

    assign testing_interface.oup_rp = req_dispatcher_interfaces[0].oup_rp;
    assign testing_interface.oup_req = req_dispatcher_interfaces[0].oup_req;
    assign req_dispatcher_interfaces[0].oup_ra = testing_interface.oup_ra;


    // Read requests will proceed through a series of stages.
    // Write requests will always go to L1 first.
    // If L1 needs to evict something then it generates a write request for the L2 stage.
    // If L2 needs to evict something then it generates a write request for main memory.
    // memory access controllers will handle dispatching of aligned requests from the IP blocks they interface with.
    // order of requests is not guaranteed by the access controller (this would slow it down too much).
    // There will be a single return queue capable of returning 1 access per clock cycle to the ports.

    // Each port takes in memory request.
    // Each port decomposes it into a series of aligned operations.
    // Each port has a queue of inflight requests.
    // Requests from a given port have to be completed in order.
    // Each port doesnt track any request id, it simply holds an aligned request present
    // at the mem access controller until it signals completion of the overall larger operation.

    // No pipelineing as that is effort. Ignore all the previous plan I had written pls.
    // One requet at a time.
    
    // Memory access controller initiates a read from both L1 & L2 at the same time (regardless of operation being a read or write).
    // This is so we can check if the line is already in cache for a read, modify write or for a read.

    // Read:
    // Check both L1 & L2 to see if line is present.
    // If both report not present, submit a memory request to NOC and wait. NO OTHER REQUESTS CAN BE DONE AT THIS TIME.


    // Write:
    // Check both L1 & L2 to see if line is present.
    // If line is present then read, modify & write.
    // If line is not present then off to RAM we go. Submit NOC request and wait. NO OTHER REQUESTS CAN BE DONE AT THIS TIME.
    // Once NOC replies with the data then we modify and write to L1.

    // In the case of an eviction of data being neccesary:
    // L1 will signal to L2 that eviction is neccesary, L2 will recognise the request and attempt to write the line.
    // If L2 needs to evict then it will submit a request to NOC and be done with it.
endmodule
*/
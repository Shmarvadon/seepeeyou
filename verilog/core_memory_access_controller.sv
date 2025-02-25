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
    logic [3:0]     prt:    // Request origin port.
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

module mem_acc_scheduler #(parameter NUM_PORTS = 2) (
    input clk,
    input rst,

    // Communicate with the front end ports.

    // Request submission.
    input  logic [NUM_PORTS]         fs_prts_inp_rp,     // Front end ports input request present.
    input  line_acc_req [NUM_PORTS]  fs_prts_inp_req,    // Front end ports input request.
    output logic [NUM_PORTS]         fs_prts_inp_op,     // Front end ports input open.

    // Request reply.
    output logic [NUM_PORTS]         fs_prts_oup_rp,     // Front end ports output request present.
    output line_acc_req              fs_prts_oup_req,    // Front end ports output request.
    input logic [NUM_PORTS]          fs_prts_oup_op,     // Front end ports output open.


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
    bit [$clog2(IFRS_LEN)-1:0] rd_rtn_par_ind;  // Read return parent request index.
    always_comb begin
        // Default values.
        fs_prts_oup_rp = 0;
        stg_1_wrp_rp = 0;
        rd_rpl_a = 0;

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

                // If the front end port & L1 can both accept the retirement.
                if (fs_prts_oup_op[ifr_oup[rd_rtn_par_ind].prt] && stg_1_wrp_op) begin
                    // Signal to the front end that we are sending it a reply.
                    fs_prts_oup_rp[ifr_oup[rd_rtn_par_ind].prt] = 1;
                    // Signal to L1 cache that we are sending it a write.
                    stg_1_wrp_rp = 1;

                    // Signal to the stage returning the reply that we have it.
                    rd_rpl_a[rd_rtn_sel] = 1;
                end
            end
            // If the request is not a read type (its a write).
            else begin
                // signal to L1 cache that we want to submit a write operation.
                stg_1_wrp_rp = 1;

                // Signal to the stage that returned us the read reply that its done if stage 1 accepts it.
                rd_rpl_a[rd_rtn_sel] = stg_1_wrp_op;
            end

            // On the next clock edge, if there is a read reply & the conditions are met to retire it then it will be retired & the rd_rpl_p
            // signal will go low for the port currently selected and another request could be retired.
        end
    end 


    // Comb block for handling dispatching of requests.
    bit [$clog2(NUM_PORTS)-1:0] fs_prt_sel;
    always_comb begin
        // Defaults values.
        fs_prts_inp_op = 0;
        stg_1_rdp_rp = 0;

        // Check if the in flights buffer is full or not.
        ifr_full = &ifr_up; // Should be a logical and of the entire packed array which would give 1 if all 1s or 0 otherwise.

        // If one of the front side ports is trying to submit a memory access request.
        if (fs_prts_inp_rp != 0) begin

            // Select a port that wants to submit.
            for (integer i = 0; i < NUM_PORTS; i = i + 1) begin if (fs_prts_inp_rp) fs_prt_sel = i; end

            // Create read request to L1 stage.
            stg_1_rdp_req = fs_prts_inp_req[fs_prt_sel].addr;

            // Signal to L1 cache that we have a new request for it.
            stg_1_rdp_rp = 1;

            // Signal to the port that request is accepted if stage 1 read port is open (open means it can accept input on next posedge).
            fs_prts_inp_op[fs_prt_sel] = stg_1_rdp_op;
        end
    end
endmodule

// Memory access controller L1 cache stage.
module mem_acc_l1_stage(
    input clk,
    input rst,

    // Read port.
    input logic            rdp_rp,   // Read port request present.
    input line_read_req    rdp_req,  // Read port request.
    output logic           rdp_op,   // Read port open.

    // Write port.
    input logic            wrp_rp,   // Write port request present.
    input line_write_req   wrp_req,  // Write port request.
    output logic           wrp_op,   // Write port open.

    // Return channel for completed requests.
    output logic rd_rpl_p,
    output line_read_reply rd_rpl,
    input logic rd_rpl_a
);

    localparam RD_RQ_Q_LEN = 4;
    localparam WR_RQ_Q_LEN = 4;

    // Read request queue.
    line_read_req                   rdrq_q_ar;  // Read request queue active request.   
    logic [$clog2(RD_RQ_Q_LEN)-1:0] rdrq_q_len; // Read request queue length.
    logic                           rdrq_q_we;  // Read request queue write enable.
    logic                           rdrq_q_se;  // Read request queue shift enable.
    fifo_queue #($bits(line_read_req), RD_RQ_Q_LEN) rdrq_q(clk, rst, rdp_req, rdrq_q_ar, rdrq_q_len, rdrq_q_we, rdrq_q_se);

    // Write request queue.
    line_write_req                  wrrq_q_ar;  // Write request queue active request.
    logic [$clog2(WR_RQ_Q_LEN)-1:0] wrrq_q_len; // Write request queue length.
    logic                           wrrq_q_we;  // Write request queue write enable.
    logic                           wrrq_q_se;  // Write request queue shift enable.
    fifo_queue #($bits(line_write_req), WR_RQ_Q_LEN) wrrq_q(clk, rst, wrp_req, wrrq_q_ar, wrrq_q_len, wrrq_q_we, wrrq_q_se);


endmodule

// Memory access controller L1 cache stage.
module mac_l1_stage(
    input clk,
    input rst,

    // Interface to input and output of requests from this stage.
    mac_stage_intf req_intf,

    // Return channel for completed requests.
    output mem_acc_req completed_request_output,
    output logic completed_request_present,
    input   completed_request_accepted
);

    // L1 cache.
    bit [31:0] fs_addr;
    bit fs_wr;
    bit fs_go;
    bit [127:0] fs_inp;
    bit bs_done;

    logic fs_done;
    logic fs_suc;
    logic [127:0] fs_oup;
    logic [31:0] bs_addr;
    logic [127:0] bs_oup;
    logic bs_we;

    l1_cache #(4, 256, 16, 32) l1_cache(clk, rst, fs_addr, fs_wr, fs_go, fs_done, fs_suc, fs_inp, fs_oup, bs_addr, bs_oup, bs_we, bs_done);

    // Input request buffer.
    mem_acc_req         ar;         // Active request.
    logic [3:0]         rql;        // Request queue length.
    logic               rq_we;      // Request queue write enable.
    logic               rq_se = 0;      // Request queue shift enable.
    fifo_queue #($bits(mem_acc_req), 8) request_queue(clk, rst, req_intf.inp_req, ar, rql, rq_we, rq_se);

    // If there is a request to input a new request & the queue has room then we bring write enable high.
    assign rq_we = rql != 8 ? req_intf.inp_rp : 0;
    // If write enable is high then we must have accepted the request.
    assign req_intf.inp_ra = rq_we ? 1 : 0;

    // Present active request to the cache.
    assign fs_addr = ar.addr;
    assign fs_inp = ar.dat;
    assign fs_wr   = ar.rqt;

    // represents the state of this stage: cache enabled, return success (cache paused), forward to next stage (cache paused).
    bit [3:0] state;

    always_latch begin
        // default value.
        fs_go = 0;
        completed_request_present = 0;
        req_intf.oup_rp = 0;
        rq_se = 0;
        bs_done = 0;

        case(state) 
        // Stage 1, run the request through the cache.
        0:
        begin
            // If there is a request present in the request queue.
            if (rql != 0) begin
                // If the cache is not done.
                if (!fs_done) begin
                    fs_go = 1;  // Tell the cache to do the request.

                    // If the back side of the cache is signalling that it wants to evict a line.
                    if (bs_we) begin
                        // Translate the backside request into something to be forwarded to the next stage.
                        req_intf.oup_rp = 1;
                        req_intf.oup_req.addr = bs_addr;
                        req_intf.oup_req.dat = bs_oup;
                        req_intf.oup_req.orig = 0;
                        req_intf.oup_req.rqt = 1;

                        // If the next stage accepts the request.
                        if (req_intf.oup_ra) begin
                            bs_done = 1;
                        end
                    end
                end
                // If the cache is done.
                else begin
                    // Tell the cache to stop going (this wont affect its output till next clk cycle).
                    fs_go = 0;

                    // If the operation was successful.
                    if (fs_suc) begin  
                        // Signal to the rest of the memory access controller that the request has been completed successfully.
                        completed_request_output.addr = ar.addr;            // Request address.
                        completed_request_output.dat = fs_oup;              // Request data (if any is returned).
                        completed_request_output.rqt = ar.rqt;              // Request type.
                        completed_request_output.orig = ar.orig;            // Request origin (used to route it to the correct port).
                        completed_request_output.rsuc = 1;                  // Mark the request as successful.

                        // Indicate a request is ready to be returned.
                        completed_request_present = 1;
                    end
                    // If not successful then we need to propogate it forwards probably.
                    else begin
                        // Set the output request from this stage to be the current request.
                        req_intf.oup_req = ar;

                        // Signal that a request is ready.
                        req_intf.oup_rp = 1;
                    end

                    // Set shift enable high for 1 clk cycle for the input queue to shift to the next request (if one is present).
                    rq_se = 1;
                end
            end
        end

        // Returning result because it was successful stage.
        1:
        begin
            // Indicate a request is ready to be returned.
            completed_request_present = 1;
        end

        // Forwarding result to the next stage.
        2:
        begin
            // Signal that a request is ready.
            req_intf.oup_rp = 1;
        end
        endcase
    end

    always @(posedge clk) begin
        case(state)
        // Stage 1, run the request through the cache.
        0:
        begin
            // If the cache is done.
            if (fs_done) begin
                // If the operation was successful, we need to return the result.
                if (fs_suc) begin  
                    // Move to waiting to return successful request to rest of MAC stage if return accepted not signalled yet.
                    if (!completed_request_accepted) state = 1;
                end
                // If not successful then we need to propogate it forwards probably.
                else begin
                    // Move to wait for request to forward to next stage if next stage has not accepted request yet.
                    if (!req_intf.oup_ra) state = 2;
                end
            end
        end

        // Waiting for memory access controller to signal it has accepted the return.
        1:
        begin
            // If the rest of the memory access controller accepts the returned request.
            if (completed_request_accepted) begin
                // move back to the stage where cache is enabled.
                state = 0;
            end
        end

        // Waiting for the next stage to signal that it has accepted the request.
        2:
        begin
            // If the next stage has accepted the result.
            if (req_intf.oup_ra) begin
                // Move back to stage where the cache is enabled.
                state = 0;
            end
        end
        endcase
    end

    // Handle back side of the cache.
endmodule

// Memory access controller L2 cache stage.
module mac_l2_stage(
    input clk,
    input rst,

    // Interface to input and output requests from this stage.
    mac_stage_intf req_intf,

    // Return channel for completed requests.
    output mem_acc_req  comp_req_oup,   // Completed request output.
    output logic        comp_req_pre,   // Completed request present.
    input               comp_req_ac     // Completed request accepted.
);
    // L2 cache.
    bit [31:0]      fs_addr;
    bit             fs_wr;
    bit             fs_go;
    bit [127:0]     fs_inp;
    bit             bs_done;

    logic           fs_done;
    logic           fs_suc;
    logic [127:0]   fs_oup;
    logic [31:0]    bs_addr;
    logic [127:0]   bs_oup;
    logic           bs_we;

    l2_cache #(8, 256, 16, 32, 4) l2_cache(clk, rst, fs_addr, fs_wr, fs_go, fs_done, fs_suc, fs_inp, fs_oup, bs_addr, bs_oup, bs_we, bs_done);

    // request input buffer.
    mem_acc_req         ar;         // Active request.
    logic [3:0]         rql;        // Request queue length.
    logic               rq_we;      // Request write enable.
    logic               rq_se = 0;  // Request shift enable.
    fifo_queue #($bits(mem_acc_req), 8) request_queue(clk, rst, req_intf.inp_req, ar, rql, rq_we, rq_se);

    // If there is a request to input a new request & the queue has room then we bring write enable high.
    assign rq_we = rql != 8 ? req_intf.inp_rp : 0;
    // If write enable is high then we must have accepted the request.
    assign req_intf.inp_ra = rq_we ? 1 : 0;

    // Present active request to the cache.
    assign fs_addr  = ar.addr;
    assign fs_inp   = ar.dat;
    assign fs_wr    = ar.rqt;

    // Represent the stage of this stage: cache enabled, return success (cache paused), forward to next stage (cache pauded).
    bit [3:0] state;

    always_latch begin
        // default value.
        fs_go = 0;
        comp_req_pre = 0;
        req_intf.oup_rp = 0;
        rq_se = 0;

        case(state) 
        // Stage 1, run the request through the cache.
        0:
        begin
            // If there is a request present in the request queue.
            if (rql != 0) begin
                // If the cache is not done.
                if (!fs_done) fs_go = 1;  // Tell the cache to do the request.
                // If the cache is done.
                else begin
                    // Tell the cache to stop going (this wont affect its output till next clk cycle).
                    fs_go = 0;

                    // If the operation was successful.
                    if (fs_suc) begin  
                        // If the request was a read (we dont respond to writes as writes to L2 exclusively come from L1 eviction).
                        if (!ar.rqt) begin
                            // Signal to the rest of the memory access controller that the request has been completed successfully.
                            comp_req_oup.addr = ar.addr;            // Request address.
                            comp_req_oup.dat = fs_oup;              // Request data (if any is returned).
                            comp_req_oup.rqt = ar.rqt;              // Request type.
                            comp_req_oup.orig = ar.orig;            // Request origin (used to route it to the correct port).
                            comp_req_oup.rsuc = 1;                  // Mark the request as successful.

                            // Indicate a request is ready to be returned.
                            comp_req_pre = 1;
                        end
                    end
                    // If not successful then we need to propogate it forwards probably.
                    else begin
                        // Set the output request from this stage to be the current request.
                        req_intf.oup_req = ar;

                        // Signal that a request is ready.
                        req_intf.oup_rp = 1;
                    end

                    // Set shift enable high for 1 clk cycle for the input queue to shift to the next request (if one is present).
                    rq_se = 1;
                end
            end
        end

        // Returning result because it was successful stage.
        1:
        begin
            // Indicate a request is ready to be returned.
            comp_req_pre = 1;
        end

        // Forwarding result to the next stage.
        2:
        begin
            // Signal that a request is ready.
            req_intf.oup_rp = 1;
        end
        endcase
    end

    always @(posedge clk) begin
        case(state)
        // Stage 1, run the request through the cache.
        0:
        begin
            // If the cache is done.
            if (fs_done) begin
                // If the operation was successful, we need to return the result.
                if (fs_suc) begin  
                    // If the request was a read request (we dont report back on writes as they exclusively come from L1 eviction).
                    if (!ar.rqt) begin
                        // Move to waiting to return successful request to rest of MAC stage if return accepted not signalled yet.
                        if (!comp_req_ac) state = 1;
                    end
                end
                // If not successful then we need to propogate it forwards probably.
                else begin
                    // Move to wait for request to forward to next stage if next stage has not accepted request yet.
                    if (!req_intf.oup_ra) state = 2;
                end
            end
        end

        // Waiting for memory access controller to signal it has accepted the return.
        1:
        begin
            // If the rest of the memory access controller accepts the returned request.
            if (comp_req_ac) begin
                // move back to the stage where cache is enabled.
                state = 0;
            end
        end

        // Waiting for the next stage to signal that it has accepted the request.
        2:
        begin
            // If the next stage has accepted the result.
            if (req_intf.oup_ra) begin
                // Move back to stage where the cache is enabled.
                state = 0;
            end
        end
        endcase
    end
endmodule

// Memory access controller NOC stage.
module mac_noc_stage(
    input clk,
    input rst,


    // Interface to input and output requests from this stage.
    mac_stage_intf.drive_output req_intf,

    // Return channel for completed requests.
    output mem_acc_req  comp_req_oup,   // Completed request output.
    output logic        comp_req_pre,   // Completed request present.
    input               comp_req_ac,    // Completed request accepted.

    // NOC stop port.
    noc_ip_port.ip_side noc_prt         // NOC IP interface port to dispatch requests to SOC IMC.
);

    // request input buffer.
    mem_acc_req         ar;         // Active request.
    logic [3:0]         rql;        // Request queue length.
    logic               rq_we;      // Request write enable.
    logic               rq_se = 0;  // Request shift enable.
    fifo_queue #($bits(mem_acc_req), 8) request_queue(clk, rst, req_intf.inp_req, ar, rql, rq_we, rq_se);

    // If there is a request to input a new request & the queue has room then we bring write enable high.
    assign rq_we = rql != 8 ? req_intf.inp_rp : 0;
    // If write enable is high then we must have accepted the request.
    assign req_intf.inp_ra = rq_we ? 1 : 0;

    // Assign stuff to drive noc ip port tx lines.
    assign noc_prt.tx_dat.dat = ar;

    assign noc_prt.tx_dat.hdr.src_port = noc_prt.prt_num;
    assign noc_prt.tx_dat.hdr.src_addr = noc_prt.prt_addr;

    assign noc_prt.tx_dat.hdr.dst_addr = `MEMORY_INTERFACE_NOC_ADDR;
    assign noc_prt.tx_dat.hdr.dst_port = `MEMORY_INTERFACE_NOC_PORT;

    assign noc_prt.tx_dat.hdr.len = ($bits(mem_acc_req) / 8) + ($bits(noc_packet_header) / 8);   // Length of packet is length of header + length of payload.


    // Comb block for submitting the request to NOC.
    always_comb begin
        // Default values.
        rq_se = 0;
        noc_prt.tx_av = 0;

        // Check if request queue length is > 0 then lets signal to noc port that we want to send it a request.
        if (rql > 0) begin
            // Signal to the noc stop that we want to send a packet to it.
            noc_prt.tx_av = 1;

            // If the noc stop accepts the request then we signal to the request queue to shift to next request on clk.
            if (noc_prt.tx_re) begin
                rq_se = 1;
            end
        end
    end

    // Logic to recieve replies from the NOC for the memory access requests & buffer them in an 8 long shift reg for return to the scheduler when its able to accept them.

    // request output buffer.
    mem_acc_req         comp_req_q_oup;         // Active request.
    logic [3:0]         comp_req_q_len;         // Completed request queue length.
    logic               comp_req_q_we;          // Completed request write enable.
    logic               comp_req_q_se;          // Completed request shift enable.
    fifo_queue #($bits(mem_acc_req), 8) comptd_req_q(clk, rst, noc_prt.rx_dat.dat[0+:$bits(mem_acc_req)], comp_req_q_oup, comp_req_q_len, comp_req_q_we, comp_req_q_se);

    // If the noc has an rx to return & queue isnt full accept it.
    assign comp_req_q_we = comp_req_q_len != 8 ? noc_prt.rx_av : 0;

    // Assign write enable for the queue to drive the "request read" response to the NOC stop.
    assign noc_prt.rx_re = comp_req_q_we;


    // Some assigns to handle returning stuff from the queue to the scheduler.


    // Assign the output of the completed request buffer to drive the return path.
    assign comp_req_oup = comp_req_q_oup;
    // Signal a completed request is present if the queue is longer than 0.
    assign comp_req_pre = comp_req_q_len != 0 ? 1 : 0;
    // If the scheduler accepts the return then signal the queue to shift.
    assign comp_req_q_se = comp_req_ac;
endmodule


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

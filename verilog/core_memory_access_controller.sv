// Memory access controller port interface.
interface mem_acc_prt_intf;
    // Recieve replies from memory access controller.
    logic           rx_av;      // Rx available.
    logic           rx_re;      // Rx read.
    logic [31:0]    rx_addr;    // Rx memory access address.
    logic [127:0]   rx_dat;     // Rx data.
    logic [3:0]     rx_len;     // Rx length.

    // Send memory access requests to the memory access controller.
    logic           tx_av;      // Tx available.
    logic           tx_re;      // Tx read.
    logic [31:0]    tx_addr;    // Tx memory access address.
    logic [127:0]   tx_dat;     // Tx data.
    logic [3:0]     tx_len;     // Tx length.

    // Core backend side of the interface.
    modport front_side (input rx_av, output rx_re, input rx_addr, input rx_dat, input rx_len,
                        output tx_av, input tx_re, output tx_addr, output tx_dat, output tx_len);

    // memory access controller port side of the interface.
    modport back_side (output rx_av, input rx_re, output rx_addr, output rx_dat, output rx_len,
                       input tx_av, output tx_re, input tx_addr, input tx_dat, input tx_len);
endinterface

typedef struct packed {
    logic [31:0]    addr;  // Request address.
    logic [127:0]   dat;   // Request data.
    logic [3:0]     orig;  // Request origin port. (only applicable if read operation).
    logic           rqt;   // Request type.
    logic           rsuc;  // Request success.
} mac_request;

interface mac_stage_intf;

    // Input to the stage.
    logic           inp_rp;     // Input request present.
    mac_request     inp_req;    // Input request.
    logic           inp_ra;     // Input request accepted.

    // Output from the stage.
    logic           oup_rp;     // Output request present.
    mac_request     oup_req;    // Output request.
    logic           oup_ra;     // Output request accepted.

    // Input to the stage.
    modport stage_input (input inp_rp, input inp_req, output inp_ra,
                         output oup_rp, output oup_req, input oup_ra);
    
    // Output from the stage.
    modport stage_output (output inp_rp, output inp_req, input inp_ra,
                          input oup_rp, input oup_req, output oup_ra);

endinterface

// Memory access controller L1 cache stage.
module mac_l1_stage(
    input clk,
    input rst,

    // Interface to input and output of requests from this stage.
    mac_stage_intf req_intf,

    // Return channel for completed requests.
    output mac_request completed_request_output,
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
    mac_request         ar;         // Active request.
    logic [3:0]         rql;        // Request queue length.
    logic               rq_we;      // Request queue write enable.
    logic               rq_se = 0;      // Request queue shift enable.
    fifo_queue #($bits(mac_request), 8) request_queue(clk, rst, req_intf.inp_req, ar, rql, rq_we, rq_se);

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
    output mac_request  comp_req_oup,   // Completed request output.
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
    mac_request         ar;         // Active request.
    logic [3:0]         rql;        // Request queue length.
    logic               rq_we;      // Request write enable.
    logic               rq_se = 0;  // Request shift enable.
    fifo_queue #($bits(mac_request), 8) request_queue(clk, rst, req_intf.inp_req, ar, rql, rq_we, rq_se);

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

// Memory access controller operation scheduler stage.
module mac_scheduler #(parameter NUMBER_OF_PORTS = 2)(
    input clk,
    input rst,

    // Interface to the input of L1 cache stage.
    mac_stage_intf stage_1_intf,

    // Interface to the memory access ports.
    mac_stage_intf port_interfaces [NUMBER_OF_PORTS],
);

    localparam IFRS_LEN = 64;

    // single input parallel output buffer to hold the addresses of inflight operations.
    logic                           ifr_we;                 // In flight requests buffer write enable.
    logic [31:0]                    ifr_inp;                // In flight requests buffer input.
    logic [31:0]                    ifr_oup [IFRS_LEN];     // In flight requests buffer output.
    logic [IFRS_LEN]                ifr_clrp;               // In flight requests buffer clear positions.
    logic [$clog2(IFRS_LEN)-1:0]    ifr_up;                 // In flight requests buffer used positions.

    sipo_buffer #(32, IFRS_LEN) ifrs(clk, rst, ifr_we, ifs_inp, ifr_clrp, ifr_oup, ifr_up);   


    // comb block to handle dispatching requests from the ports.
    bit clash_found;
    always_comb begin
        // Default values.
        clash_found = 0;


        // Loop over all the ports that are attempting to submit an operation.
        for (integer i = 0; i < NUMBER_OF_PORTS; i = i + 1) begin
            // Set clash_found to 0.
            clash_found = 0;
            
            // Loop over all the in flight requests addresses.
            for (integer j = 0; j < IFRS_LEN; j = j + 1) begin
                if (port_interfaces[i].inp_req.addr == ifr_oup[j]) clash_found = 0;
            end
        end
    end

    // This stage should contain a list of all active operations addresses, this way it can block any incoming requests that modify an in flight address.
    // This stage is also responsible for accepting requests from the ports via a priority decoder.
    // This stage is also responsible for recieveing successful requests and removing them from the in flight list.

endmodule

module memory_access_controller #(parameter NUMBER_OF_PORTS = 2)(
    input clk,
    input rst,

    // Basic interface for testing. ASSUME ALL REQUESTS ARE ALIGNED.
    mac_stage_intf      testing_interface,

    noc_port            noc_acc_prt
);
    // request dispatcher interfaces.
    mac_stage_intf req_dispatcher_interfaces[NUMBER_OF_PORTS]();

    // Shared successful request return path.
    mac_request suc_req [3];  // Successful request(s).
    logic [3] suc_req_pres;   // Successful request(s) present.
    logic [3] suc_req_acc;    // Successful request(s) accepted.


    // Stage 1.
    mac_stage_intf  stage_1_intf();     // Stage 1 interface.
    mac_request     stage_1_res;        // Stage 1 result.
    logic           stage_1_res_pres;   // Stage 1 result present.
    logic           stage_1_res_acc;    // Stage 1 result accepted.

    mac_l1_stage stage_1(clk, rst, stage_1_intf.stage_input, suc_req[0], suc_req_pres[0], suc_req_acc[0]);

    // Stage 2.
    mac_stage_intf  stage_2_intf();     // Stage 2 interface.
    mac_request     stage_2_res;        // Stage 2 result.
    logic           stage_2_res_pres;   // Stage 2 result present.
    logic           stage_2_res_acc;    // Stage 2 result accepted.

    mac_l2_stage stage_2(clk, rst, stage_2_intf.stage_input, suc_req[1], suc_req_pres[1], suc_req_acc[1]);

    // Connect stage 1 output to stage 2 input.
    assign stage_2_intf.inp_rp = stage_1_intf.oup_rp;
    assign stage_2_intf.inp_req = stage_1_intf.oup_req;
    assign stage_1_intf.oup_ra = stage_2_intf.inp_ra;


    // Handle queuing up requests into the L1 stage.
    assign stage_1_intf.inp_rp = test_interface.inp_rp;
    assign stage_1_intf.inp_req = test_interface.inp_req;
    assign test_interface.inp_ra = stage_1_intf.inp_ra;

    // Handle returning successful requests from either of the 2 stages.
    mac_request [NUMBER_OF_PORTS] req_rtn_req [3];  // Request return path request.
    logic [NUMBER_OF_PORTS]       req_rtn_prs [3];  // Request return path request present.
    logic [NUMBER_OF_PORTS]       req_rtn_acc [3];  // Request return path request accepted.

    // Handle steering the return of requests from each stage to the right port.
    genvar i;
    generate
        for (i = 0; i < 3; i = i + 1) begin : router
        always_comb begin
            for (int j = 0; j < NUMBER_OF_PORTS; j = j + 1) begin
            end
        end
        end
    endgenerate


    // Have each stage have a mux to select the port to return the request to.
    // Have each port have a priority encoder to select which stage to take reply from if more than 1 stage is trying to return something in a given clock cycle.

    mac_request []

    



    

    // Handle stage 1 returning a successful request. This is super dumb for now as we just let the testing interface control it.
    always_comb begin
        // Just hook em up and let them do their thing.
        testing_interface.oup_rp = stage_1_res_pres;
        testing_interface.oup_req = stage_1_res;
        stage_1_res_acc = testing_interface.oup_ra;
    end


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

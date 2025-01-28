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

module mac_l1_stage #(parameter MEM_ACC_PORT_COUNT = 2)(
    input clk,
    input rst,

    // Interface for input and output of requests from this stage.
    mac_stage_intf req_intf,

    // Return channel for completed requests.
    output mac_request completed_request_output,
    output logic completed_request_present,
    input   completed_request_accepted
);

    // L1 cache.
    bit [31:0] l1_cache_fs_addr;
    bit l1_cache_fs_wr;
    bit l1_cache_fs_go;
    bit [127:0] l1_cache_fs_dinp;
    bit l1_cache_bs_done;

    logic l1_cache_fs_done;
    logic l1_cache_fs_suc;
    logic [127:0] l1_cache_fs_doup;
    logic [31:0] l1_cache_bs_addr;
    logic [127:0] l1_cache_bs_doup;
    logic l1_cache_bs_we;

    l1_cache #(4, 256, 16, 32) l1_cache(clk, rst, l1_cache_fs_addr, l1_cache_fs_wr, l1_cache_fs_go, l1_cache_fs_done, l1_cache_fs_suc, l1_cache_fs_dinp, l1_cache_fs_doup, l1_cache_bs_addr, l1_cache_bs_doup, l1_cache_bs_we, l1_cache_bs_done);

    // Input request buffer.
    mac_request         cr;         // Current request.
    logic [$clog(8):0]  rql;        // Request queue length.
    logic               rq_we;      // Request queue write enable.
    logic               rq_se;      // Request queue shift enable.
    fifo_queue #($bits(mac_request), 8) request_queue(clk, rst, req_intf.inp_req, cr, rql, rq_we, rq_se);

    // If there is a request to input a new request & the queue has room then we bring write enable high.
    assign rq_we = rql != 8 ? req_intf.inp_rp : 0;
    // If write enable is high then we must have accepted the request.
    assign req_intf.inp_ra = rq_we ? 1 : 0;
    // Assign the current request to drive the front side inputs of the cache.
    assign l1_cache_fs_addr = cr.addr;
    assign l1_cache_fs_dinp = cr.dat;
    assign l1_cache_fs_wr   = cr.rqt;


    bit [2:0] stage_state;

    always_comb begin
        case (stage_state)

        // 
        endcase
    end

    // Handle the current request.
    always_comb begin
        // If there is a request present.
        if (rql != 0) begin
            // Set l1_cache_fs_go to true.
            l1_cache_fs_go = 1;

        end
        else begin
            // Set l1_cache_fs_go to false.
            l1_cache_fs_go = 0;
        end
    end

    // Request is input on one cycle.
    // Read result or write (if modify) is given on the next.
    // Request is retired on 3rd cycle (if successful).

endmodule

module memory_access_controller(
    input clk,
    input rst,

    // Basic interface for testing.
    mem_acc_prt_intf    core_interface,

    noc_port            noc_acc_prt
);


    // L2 cache.
    bit [31:0] l2_cache_fs_addr;
    bit l2_cache_fs_wr;
    bit l2_cache_fs_go;
    bit [127:0] l2_cache_fs_dinp;
    bit l2_cache_bs_done;

    logic l2_cache_fs_done;
    logic l2_cache_fs_suc;
    logic [127:0] l2_cache_fs_doup;
    logic [31:0] l2_cache_bs_addr;
    logic [127:0] l2_cache_bs_doup;
    logic l2_cache_bs_we;

    l2_cache #(16, 512, 16, 32, 4) l2_cache(clk, rst, l2_cache_fs_addr, l2_cache_fs_wr, l2_cache_fs_go, l2_cache_fs_done, l2_cache_fs_suc, l2_cache_fs_dinp, l2_cache_fs_doup, l2_cache_bs_addr, l2_cache_bs_doup, l2_cache_bs_we, l2_cache_bs_done);



    // Handle queuing up requests into the L1 stage.
    always @(posedge clk) begin
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

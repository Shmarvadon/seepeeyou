
// Interface to allow CPU to submit memory access requests to memory subsystem.
interface cpu_mem_subsys_interface;

    // *** Submit request to memory access controller. ***
    
    // Lines to the memory access controller.
    logic [31:0]    req_addr;
    logic [127:0]   req_dat;
    logic [3:0]     req_len;
    logic           req_r_w;        // 1 for read 0 for write.
    logic           req_submit;

    // Lines from memory access controller.
    logic req_acc;


    // *** Recieve response from memory access controller. ***

    // Lines from the memory access controller.
    logic [31:0] res_addr;
    logic [127:0] res_dat;
    logic [3:0] res_len;
    logic res_rdy;

    // Lines to memory access controller.
    logic res_read;
endinterface


module memory_access_controller(
    input clk,
    input rst,
    
    input [31:0]        address,
    input [127:0]       dat_in,
    output bit [127:0]  dat_out,
    input               re_we,
    input               submit
);

    bit [31:0] l1_cache_fs_addr;
    bit l1_cache_fs_re;
    bit l1_cache_fs_we;
    bit [127:0] l1_cache_fs_dinp;
    bit l1_cache_bs_done;

    logic [127:0] l1_cache_fs_doup;
    logic l1_cache_fs_suc;
    logic l1_cache_fs_done;
    logic [31:0] l1_cache_bs_addr;
    logic [127:0] l1_cache_bs_doup;
    logic l1_cache_bs_we;

    cache #(8, 256, 16, 32) l1_cache(clk, rst, l1_cache_fs_addr, l1_cache_fs_re, l1_cache_fs_we, l1_cache_fs_doup, l1_cache_fs_dinp, l1_cache_fs_suc, l1_cache_fs_done, l1_cache_bs_addr, l1_cache_bs_doup, l1_cache_bs_done, l1_cache_bs_we);

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

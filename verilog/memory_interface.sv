`include "structs.svh"
`include "defines.svh"

module memory_interface(
    input logic rst,
    input logic fclk,
    input logic mclk,

    // Interface to the NOC.

    input logic [31:0][7:0]     bus_inp_dat,    // Input bus data.
    input logic [5:0]           bus_inp_bp,     // Input bus bytes present.
    output logic                bus_inp_bo,     // Input bus open.

    output logic [31:0][7:0]    bus_oup_dat,    // Output bus data.
    output logic [5:0]          bus_oup_bp,     // Output bus bytes present.
    input logic                 bus_oup_bo,      // Output bus open.

    // Interface to memory.
    output logic mem_en,
    output logic mem_we,
    output logic mem_re,

    output logic [31:0] mem_addr_sel,
    input logic [127:0] mem_dat
);

    /*          Network Interface Unit          */
    localparam NIU_PORTS = 1;
    logic [NIU_PORTS-1:0][3:0]  niu_prt_addr;
    logic [NIU_PORTS-1:0][3:0]  niu_prt_num;

    logic [NIU_PORTS-1:0]       niu_rx_av;
    logic [NIU_PORTS-1:0]       niu_rx_re;
    noc_packet [NIU_PORTS-1:0]  niu_rx_dat;

    logic [NIU_PORTS-1:0]       niu_tx_av;
    logic [NIU_PORTS-1:0]       niu_tx_re;
    noc_packet [NIU_PORTS-1:0]  niu_tx_dat;

    network_interface_unit #(NIU_PORTS, `MEMORY_INTERFACE_NOC_ADDR) niu(
    mclk, fclk, rst, niu_prt_addr, niu_prt_num,
    niu_rx_av, niu_rx_re, niu_rx_dat,
    niu_tx_av, niu_tx_re, niu_tx_dat,
    bus_inp_dat, bus_inp_bp, bus_inp_bo,
    bus_oup_dat, bus_oup_bp, bus_oup_bo
    );


    // logic to handle read and write requests.
    mem_rd_rq rd_rq_rx;
    mem_wr_rq wr_rq_rx;
    assign rd_rq_rx = niu_rx_dat[0].dat;
    assign wr_rq_rx = niu_rx_dat[0].dat;

    mem_rd_rp rd_rp_tx;
    assign niu_tx_dat[0].dat = rd_rp_tx;

    bit [127:0] mem_rd_buff;
    bit [127:0] mem_wr_buff;
    // If write enable is high, drive wr_buff into mem_dat.
    assign mem_dat = (mem_we) ? mem_wr_buff : 'hz;
    

    bit [3:0] state;
    always @(posedge mclk) begin
        // If memory is reading, stash the result.
        if (mem_en & mem_re & !mem_we) mem_rd_buff = mem_dat;

        // Defaults.
        mem_en = 0;
        mem_we = 0;
        mem_re = 0;
        mem_addr_sel = 0;
        niu_rx_re = 0;
        niu_tx_av = 0;

        // If there is an incoming rx.
        if (niu_rx_av) begin

            // If the request is a write.
            if(niu_rx_dat[0].dat[7:0] == memory_write_request) begin

                $display("Processing DRAM write request, %t", $time);

                // Present memory address.
                mem_addr_sel <= wr_rq_rx.addr;

                // Enable memory & set write enable.
                mem_en <= 1;
                mem_re <= 0;
                mem_we <= 1;

                // Present the data.
                mem_wr_buff <= wr_rq_rx.dat;

                // Signal rx read to the NIU.
                niu_rx_re <= 1;
            end

            // If the request is a read.
            if (niu_rx_dat[0].dat[7:0] == memory_read_request) begin

                $display("Processing DRAM read request, %t", $time);
                case(state)
                // Read the line from RAM.
                0:
                begin
                    // Present memory address.
                    mem_addr_sel <= rd_rq_rx.addr;

                    // Enable memory & set read enable.
                    mem_en <= 1;
                    mem_re <= 1;
                    mem_we <= 0;

                    // Incriment to next state.
                    state = 1;
                end

                // Prepare read reply.
                1:
                begin
                    // If the NIU port can read the request on next cycle.
                    if (niu_tx_re) begin
                        // Set routing information.
                        niu_tx_dat[0].hdr.src_addr <= niu_rx_dat[0].hdr.dst_addr;
                        niu_tx_dat[0].hdr.src_port <= niu_rx_dat[0].hdr.dst_port;

                        niu_tx_dat[0].hdr.dst_addr <= niu_rx_dat[0].hdr.src_addr;
                        niu_tx_dat[0].hdr.dst_port <= niu_rx_dat[0].hdr.src_port;

                        niu_tx_dat[0].hdr.len <= ($bits(noc_packet_header) + $bits(mem_rd_rp)) / 8;

                        // Set read reply data.
                        rd_rp_tx.addr <= rd_rq_rx.addr;
                        rd_rp_tx.dat <= mem_rd_buff;
                        rd_rp_tx.pt <= memory_read_reply;

                        // Signal to NIU port that there is a request to read.
                        niu_tx_av <= 1;

                        // signal that the rx has been read.
                        niu_rx_re <= 1;

                        // Set state to 2.
                        state = 2;

                    end
                end

                // Wait a cycle.
                2:
                begin
                    // Set state back to 0.
                    state = 0;
                end
                endcase
            end
        end
    end

endmodule

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

    output logic mem_addr_sel,
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

    network_interface_unit #(NIU_PORTS, 0) niu(
    mclk, fclk, rst, niu_prt_addr, niu_prt_num,
    niu_rx_av, niu_rx_re, niu_rx_dat,
    niu_tx_av, niu_tx_re, niu_tx_dat
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
        if (mem_en & mem_re & !mem_we) mem_rd_buff <= mem_dat;

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

                    end
                end
                endcase
            end
        end
    end

endmodule

/*
module memory_interface_old(
    input rst,
    input fclk,
    input mclk,

    output bit [31:0] mem_addr_sel,
    inout logic [127:0] mem_dat,

    output bit mem_en,
    output bit mem_we,
    output bit mem_re,

    input noc_bus noc_bus_inp,
    output noc_bus noc_bus_oup
);

bit [127:0] read_buff;
bit [127:0] write_buff;
assign mem_dat = (mem_we) ? write_buff : 'hz;

// NOC interface stuff.
wire ip_port noc_port;
noc_stop #(1, 2) noc_int(fclk, mclk, rst, noc_bus_inp, noc_bus_oup, noc_port);

packet rpl_pkt;
bit submit_tx;
bit rx_complete;

assign noc_port.dat_to_noc = rpl_pkt;
assign noc_port.tx_submit = submit_tx;
assign noc_port.rx_complete = rx_complete;

// Handle memory access requests.
bit [3:0] stage = 0;
always @(posedge mclk) begin
    rx_complete = 0;
    submit_tx = 0;
    // If there is a memory access request.
    if (noc_port.rx_recieve) begin

        // 3 cases: write, read or not valid.
        case(noc_port.dat_from_noc.pt)

            memory_write_request:
            begin
                 // State machine to handle the RMW.
                case (stage)
                
                    // Start RMW sequence.
                    0:
                    begin
                        // Present memory address.
                        mem_addr_sel <= noc_port.dat_from_noc.dat[31:0];

                        // Enable memory and set read enable.
                        mem_re <= 1;
                        mem_en <= 1;

                        // Move onto next stage.
                        stage = stage + 1;
                    end

                    // Wait a cycle for read.
                    1:
                    begin
                        // Advance to next stage.
                        stage = stage + 1;
                    end

                    // Perform the read and modify.
                    2:
                    begin
                        // Read from the data bus into the read buffer.
                        read_buff[127:32] <= mem_dat[127:32];
                        // Write the data to be saved to the read buffer.
                        read_buff[31:0] <= noc_port.dat_from_noc.dat[63:32];

                        // Move onto next stage.
                        stage = stage + 1;
                    end

                    // Perform the write.
                    3:
                    begin
                        // Set the memory to write mode.
                        mem_re <= 0;
                        mem_we <= 1;
                        mem_en <= 1;

                        // Present the read buffer back to the data bus.
                        write_buff <= read_buff;

                        // Move to next stage.
                        stage = stage + 1;
                    end

                    // Wait a cycle for the write to finish.
                    4:
                    begin
                        // move to next stage.
                        stage = stage + 1;
                    end

                    // Finish the RMW.
                    5:
                    begin
                        // Disable the memory.
                        mem_en <= 0;

                        // Create a reply packet.
                        rpl_pkt.pt <= memory_write_reply;
                        rpl_pkt.id <= noc_port.dat_from_noc.id;

                        rpl_pkt.dst_addr <= noc_port.dat_from_noc.src_addr;
                        rpl_pkt.dst_prt <= noc_port.dat_from_noc.src_prt;

                        rpl_pkt.src_addr <= noc_port.port_address;
                        rpl_pkt.src_prt <= noc_port.port_number;

                        // Signal to submit the tx reply.
                        submit_tx <= 1;

                        // Move to next stage.
                        stage = stage + 1;
                    end

                    // Finish the write request.
                    6:
                    begin

                        mem_en <= 0;
                        // If the NOC port accepts our tx.
                        if (noc_port.tx_complete) begin

                            // Stop trying to submit a tx.
                            submit_tx <= 0;

                            // Signal that we have finished the rx.
                            rx_complete <= 1;

                            // Go back to stage 0.
                            stage += 1;
                        end
                    end

                    7:
                    begin
                        stage = 0;
                    end
                endcase
            end

            memory_read_request:
            begin
                // State machine to handle the read.
                case (stage)
                    // Start read sequence.
                    0:
                    begin
                        // Present the memory address.
                        mem_addr_sel <= noc_port.dat_from_noc.dat[31:0];

                        $display("Memory being read from at: %b", noc_port.dat_from_noc.dat[31:0]);

                        // Enable the memory and set it to read.
                        mem_en <= 1;
                        mem_re <= 1;
                        mem_we <= 0;

                        // Move onto next stage.
                        stage = stage + 1;
                    end

                    // Wait a cycle for the read to happen.
                    1:
                    begin
                        // Move onto next stage.
                        stage = stage + 1;
                    end

                    // Read the data presented by the memory.
                    2:
                    begin
                        // Read the data.
                        rpl_pkt.dat <= mem_dat;

                        // Construct reply packet.
                        rpl_pkt.dst_addr = noc_port.dat_from_noc.src_addr;
                        rpl_pkt.dst_prt = noc_port.dat_from_noc.src_prt;

                        rpl_pkt.src_addr = noc_port.port_address;
                        rpl_pkt.src_prt = noc_port.port_number;

                        rpl_pkt.pt = memory_read_reply;
                        rpl_pkt.id = noc_port.dat_from_noc.id;

                        // Submit the tx.
                        submit_tx <= 1;

                        // progress to next stage.
                        stage = stage + 1;
                    end

                    // Wait for the tx to be accepted by NOC stop.
                    3:
                    begin

                        // Disable the memory.
                        mem_en <= 0;

                        // If the noc stop has accepted the tx.
                        if (noc_port.tx_complete) begin
                            // stop submitting the tx.
                            submit_tx <= 0;

                            // Signal rx complete.
                            rx_complete <= 1;

                            // reset stage to 0.
                            stage += 1;
                        end
                    end

                    4:
                    begin
                        stage = 0;
                    end
                endcase
            end

            default:
            begin
                submit_tx <= 0;
                rx_complete <= 0;
                stage <= 0;

                mem_en <= 0;
                mem_re <= 0;
                mem_we <= 0;
            end
        endcase
    end
    // If there is not a memory access request.
    else begin
        // Reset stage.
        stage <= 0;
        // Disable memory.
        mem_en <= 0;
    end
end
endmodule
*/
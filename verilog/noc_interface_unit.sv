`include "structs.svh"

module network_interface_unit #(parameter PORTS = 1, ADDR = 0)(
    input ipclk,    // IP block clock.
    input fclk,     // Bus clock.
    input rst,

    // Interface to the IP blocks.
    output  logic [PORTS-1:0][3:0]  prt_addr,
    output  logic [PORTS-1:0][3:0]  prt_num,

    output  logic [PORTS-1:0]       rx_av,
    input   logic [PORTS-1:0]       rx_re,
    output  noc_packet [PORTS-1:0]  rx_dat,

    input   logic [PORTS-1:0]       tx_av,
    output  logic [PORTS-1:0]       tx_re,
    input   noc_packet [PORTS-1:0]  tx_dat,


    // Interface to the NOC bus.

    // Input to this NIU.
    input logic [31:0][7:0]     bus_inp_dat,    // Input bus data.
    input logic [5:0]           bus_inp_bp,     // Input bus bytes present.
    output logic                bus_inp_bo,     // Input bus open.

    // Output from this NIU.
    output logic [31:0][7:0]    bus_oup_dat,    // Output bus data.
    output logic [5:0]          bus_oup_bp,     // Output bus bytes present.
    input logic                 bus_oup_bo      // Output bus open.
);

    // Check that params are good.
    generate
        if (PORTS > 14) $error("NIU can not have more than 14 ports.");
    endgenerate

    localparam BUS_BUFFER_LENGTHS = 256;
    localparam IP_INTF_BUFFER_LEN = 8;

    //          FIFO shift register for bus rx.
    
    logic [31:0][7:0]                           rx_buff_inp;    // 32 byte rx buffer input.
    logic [35:0][7:0]                           rx_buff_oup;    // 36 byte rx buffer output.
    logic [31:0]                                rx_buff_push;
    logic [35:0]                                rx_buff_pop;
    logic [$clog2(BUS_BUFFER_LENGTHS):0]        rx_buff_src_num_avail;
    logic [$clog2(BUS_BUFFER_LENGTHS):0]        rx_buff_dst_num_avail;

    fifo_sr #(8,BUS_BUFFER_LENGTHS,32,35) rx_buff(fclk, rst, rx_buff_push, rx_buff_inp, rx_buff_src_num_avail, rx_buff_pop, rx_buff_oup, rx_buff_dst_num_avail);


    //          FIFO shift register for bus tx.

    logic [35:0][7:0]                           tx_buff_inp;    // 36 byte tx buffer input.
    logic [31:0][7:0]                           tx_buff_oup;    // 32 byte tx buffer output.
    logic [35:0]                                tx_buff_push;
    logic [31:0]                                tx_buff_pop;
    logic [$clog2(BUS_BUFFER_LENGTHS):0]        tx_buff_src_num_avail;
    logic [$clog2(BUS_BUFFER_LENGTHS):0]        tx_buff_dst_num_avail;

    fifo_sr #(8,BUS_BUFFER_LENGTHS,35,32) tx_buff(fclk, rst, tx_buff_push, tx_buff_inp, tx_buff_src_num_avail, tx_buff_pop, tx_buff_oup, tx_buff_dst_num_avail);
    

    // always block to handle autonomousy filling the rx buffer.
    always_comb begin
        // Default values.
        rx_buff_inp = 0;
        rx_buff_push = 0;
        bus_inp_bo = 0;

        // If the rx fifo has enough room to shift in a maximum size phit.
        if (rx_buff_src_num_avail >= 32) bus_inp_bo = 1;

        // If there is an incoming transmission.
        if (bus_inp_bp != 0) begin
            // Loop over the heads of the fifo to signify a write to it needs to happen on next clock.
            for (int i = 0; i < 32; i = i + 1) begin
                if (i < bus_inp_bp) begin
                    // Signal a write from the head is to happen & present the data to the head.
                    rx_buff_push[i] = 1;
                    rx_buff_inp[i] = bus_inp_dat[i];
                end
            end
        end
    end

    // always block to handle autonomously transmitting from tx buffer.
    always_comb begin
        // Default values.
        tx_buff_pop = 0;
        bus_oup_dat = 0;
        bus_oup_bp = 0;

        // If there are bytes to transmit.
        if (tx_buff_dst_num_avail != 0) begin
            // Loop over the tails available and present the bytes to the bus.
            for (int i = 0; i < 32; i = i + 1) begin
                if (i < tx_buff_dst_num_avail) begin
                    // Present the bytes to the bus.
                    bus_oup_dat[i] = tx_buff_oup[i];

                    // If the bus indicates it will accept the tx on next clock.
                    if (bus_oup_bo) tx_buff_pop[i] = 1;
                end
            end
            // Signal the number of bytes available to the bus.
            bus_oup_bp = tx_buff_dst_num_avail;
        end
    end

    // clock domain crossing FIFO shift registers for rx to ports.
    logic [PORTS-1:0][287:0]                            prt_rx_inp;
    logic [PORTS-1:0]                                   prt_rx_push;
    logic [PORTS-1:0][$clog2(IP_INTF_BUFFER_LEN):0]     prt_rx_inp_num_avail;
    logic [PORTS-1:0]                                   prt_rx_inp_full;

    logic [PORTS-1:0][287:0]                            prt_rx_oup;
    logic [PORTS-1:0]                                   prt_rx_pop;
    logic [PORTS-1:0][$clog2(IP_INTF_BUFFER_LEN):0]     prt_rx_oup_num_avail;
    logic [PORTS-1:0]                                   prt_rx_oup_empty;

    // generate the cdc fifo srs for port rx.
    generate
        for (genvar i = 0; i < PORTS; i = i + 1) begin
            cdc_fifo_sr #(288, IP_INTF_BUFFER_LEN) prt_rx_queue(
                fclk, rst, ipclk, rst,
                prt_rx_inp[i], prt_rx_push[i], prt_rx_inp_num_avail[i], prt_rx_inp_full[i],
                prt_rx_oup[i], prt_rx_pop[i], prt_rx_oup_num_avail[i], prt_rx_oup_empty[i]);
        end
    endgenerate

    // clock domain crossing FIFO shift registers for tx from ports.
    logic [PORTS-1:0][287:0]                            prt_tx_inp;
    logic [PORTS-1:0]                                   prt_tx_push;
    logic [PORTS-1:0][$clog2(IP_INTF_BUFFER_LEN):0]     prt_tx_inp_num_avail;
    logic [PORTS-1:0]                                   prt_tx_inp_full;

    logic [PORTS-1:0][287:0]                            prt_tx_oup;
    logic [PORTS-1:0]                                   prt_tx_pop;
    logic [PORTS-1:0][$clog2(IP_INTF_BUFFER_LEN):0]     prt_tx_oup_num_avail;
    logic [PORTS-1:0]                                   prt_tx_oup_empty;

    // generate the cdc fifo srs for port tx.
    generate
        for (genvar i = 0; i < PORTS; i = i + 1) begin
            cdc_fifo_sr #(288, IP_INTF_BUFFER_LEN) prt_tx_queue(
                ipclk, rst, fclk, rst,
                prt_tx_inp[i], prt_tx_push[i], prt_tx_inp_num_avail[i], prt_tx_inp_full[i],
                prt_tx_oup[i], prt_tx_pop[i], prt_tx_oup_num_avail[i], prt_tx_oup_empty[i]);
        end
    endgenerate

    // Comb block to handle routing the packets.
    bit prt_tx_accepted;
    always_comb begin
        // Default values.
        prt_rx_push = 0;
        prt_tx_pop = 0;
        rx_buff_pop = 0;
        tx_buff_push = 0;
        prt_tx_accepted = 0;


        // Check if the entire packet is present in the buffer.
        if (rx_buff_dst_num_avail >= rx_buff_oup[0] & rx_buff_dst_num_avail != 0) begin
            // Check if the packet is destined for this NIU.
            if (rx_buff_oup[1][7:4] == ADDR) begin
                // Present the rx to the appropriate port's rx queue.
                // Done as a for loop for readability.
                for (int i = 0; i < PORTS; i = i + 1) begin
                    if (rx_buff_oup[1][3:0] == i) begin
                        // Signal to the rx queue the number of bytes to accept.
                        prt_rx_push[i] = 1;

                        // Present paket data & signal pops to rx buff.
                        for (int j = 0; j < 35; j = j + 1) begin
                            if (j < rx_buff_oup[0]) begin 
                                // Present the data to prt rx queue.
                                prt_rx_inp[i][j+:8] = rx_buff_oup[j];
                                // Signal to rx buffer to pop this byte.
                                rx_buff_pop[j] = 1;
                            end
                        end
                    end
                end

                //
                //  Add some logic here to pass the packet to the NIU mailbox if its port is set to 15.
                //

            end
            // If the packet is not for this NIU.
            else begin
                // Check if there is enough room in tx buffer to push in the packet.
                if (tx_buff_src_num_avail >= rx_buff_oup[0]) begin
                    // Present packet data to tx buff & tell rx buff to pop bytes.
                    for (int i = 0; i < 35; i = i + 1) begin
                        if (i < rx_buff_oup[0]) begin 
                            // Present the data to tx buffer.
                            tx_buff_inp[i] = rx_buff_oup[i];
                            // Signal to rx buffer to pop this byte.
                            rx_buff_pop[i] = 1;
                            // Signal to tx buffer to push this byte.
                            tx_buff_push[i] = 1;
                        end
                    end
                end
            end
        end
        // If there is not a complete packet in the rx buffer.
        else begin
            // Check if any of the tx ports have anything to send.
            for (int i = 0; i < PORTS; i = i + 1) begin
                // If the tx port queue is not empty & the tx buff can accept the packet it wants to send.
                if (!prt_tx_oup_empty[i] & !prt_tx_accepted & tx_buff_src_num_avail >= prt_tx_oup[i][7:0]) begin
                    // loop over the tx queues bytes to push them to the tx buffer.
                    for (int j = 0; j < 35; j = j + 1) begin
                        if (j < prt_tx_oup[i][7:0]) begin
                            // Present the byte to the tx buffer.
                            tx_buff_inp[j] = prt_tx_oup[i][j+:8];
                            // Tell tx buff to push the byte.
                            tx_buff_push[j] = 1;
                        end
                    end

                    // Signal to the port tx queue to pop the packet at its head.
                    prt_tx_pop[i] = 1;

                    // Signal prt tx accepted.
                    prt_tx_accepted = 1;
                end
            end
        end
    end


    // Comb block to handle interface between the tx & rx ports and their fifos.
    always_comb begin

        // Loop over the ports.
        for (int i = 0; i < PORTS; i = i + 1) begin
            // Hook up the things.
            prt_tx_inp[i] = tx_dat[i];
            prt_tx_push[i] = tx_av[i];
            tx_re[i] = !prt_tx_inp_full[i];
            
            rx_dat[i] = prt_rx_oup[i];
            rx_av[i] = !prt_rx_oup_empty[i];
            prt_rx_pop[i] = rx_re[i];
        end
    end

    // Async reset.
    always @(posedge fclk, ipclk, rst) begin
        if (rst) begin
        end
    end
endmodule
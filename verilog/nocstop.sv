`include "structs.sv"

module noc_stop #(parameter PORTS = 1, ADDR = 0)(
    input fclk,
    input cclk,
    input rst,

    input noc_bus bus_in,
    output noc_bus bus_out,

    inout ip_port [PORTS] ip_prts_inp
);

// our address.
reg [7:0] stop_address;

// Noc bus driver
noc_bus bus_out_drvr;
noc_bus bus_out_inp;
bit bus_out_write;
assign bus_out = bus_out_drvr;

always @(posedge fclk) begin
    bus_out_drvr <= bus_out_inp;
end

// Packet recieved from noc bus queue.
bit rcv_q_full;
bit rcv_q_we;
packet rcv_q_inp;
packet rcv_q_oup;
bit rcv_q_pp;
bit rcv_q_se;
noc_packet_queue rcv_q(fclk, cclk, rst, rcv_q_full, rcv_q_we, rcv_q_inp, rcv_q_pp, rcv_q_se, rcv_q_oup);

// Packet recieved from noc bus queue.
bit snd_q_full;
bit snd_q_we;
packet snd_q_inp;
packet snd_q_oup;
bit snd_q_pp;
bit snd_q_se;
noc_packet_queue snd_q(cclk, fclk, rst, snd_q_full, snd_q_we, snd_q_inp, snd_q_pp, snd_q_se, snd_q_oup);

// Each fabric clock we look at if there is anything to recieve, send or pass forwards.
always_comb begin

    // If there is a packet on the bus.
    if (bus_in.packet_present) begin

        // Disable send q shift enable.
        snd_q_se <= 0;

        // If its for us.
        if (bus_in.data.dst_addr == stop_address) begin
            
            // If there is enough room in recieve queue.
            if (rcv_q_full == 0) begin
                // Present the data to the recieve queue.
                rcv_q_inp <= bus_in.data;

                // Signal to write to the recieve queue.
                rcv_q_we <= 1;
            end
            // If there is not enough room in the recieve queue, send the instruction around the ring again.
            else begin
                // Send the packet onwards.
                bus_out_inp <= bus_in;

                // Signal to the recieve queue to not input.
                rcv_q_we <= 0;
            end
        end
        // If the packet is not for us.
        else begin
            // Send the packet onwards.
            bus_out_inp <= bus_in;

            // Signal to the recieve queue to not input.
            rcv_q_we <= 0;
        end
    end
    // If there is not a packet on the bus.
    else begin

        // Stop the rcv q from reading new data.
        rcv_q_we <= 0;

        // If we have a packet to send.
        if (snd_q_pp) begin
            // Put the packet on the bus and signal its presence.
            bus_out_inp.data <= snd_q_oup;
            bus_out_inp.packet_present <= 1;

            // Enable shifting of the snd q.
            snd_q_se <= 1;
        end
        // If we dont have any packets to send.
        else begin
            // Signal to the bus that we have no packet to send.
            bus_out_inp.packet_present <= 0;

            // Put all 0s on the bus.
            bus_out_inp.data <= 0;

            // Disable shifting of the send q.
            snd_q_se <= 0;
        end
    end
end

// Priority decoder for NOC IP ports.
logic [PORTS] pri_dec_inp;
logic [$clog2(PORTS) + 1] pri_dec_oup;
priority_decoder #(PORTS, $clog2(PORTS) + 1) noc_ip_prt_pri_dec(pri_dec_inp, pri_dec_oup);

// hook up the priority decoder inputs.
integer i;
always_comb begin
    for(i = 0; i < PORTS; i = i + 1) begin
        if (ip_prts_inp[i].tx_submit == 1 && ip_prts_inp[i].to_noc_prt_stat == port_open) pri_dec_inp[i] <= 1;
        else pri_dec_inp[i] <= 0;
    end
end

// Hook up stuff to drive IP ports.
noc_port_status [PORTS] to_noc_prt_stat;
noc_port_status [PORTS] from_noc_prt_stat;
logic [PORTS] prt_tx_cmplt;
logic [PORTS] prt_rx_rcv;
packet [PORTS] dat_frm_noc;

genvar j;
generate
    for (j = 0; j < PORTS; j = j + 1) begin :gen_block
        // Assign bits we drive.
        assign ip_prts_inp[j].to_noc_prt_stat = to_noc_prt_stat[j];
        assign ip_prts_inp[j].tx_complete = prt_tx_cmplt[j];
        
        assign ip_prts_inp[j].from_noc_prt_stat = from_noc_prt_stat[j];
        assign ip_prts_inp[j].rx_recieve = prt_rx_rcv[j];
        assign ip_prts_inp[j].dat_from_noc = dat_frm_noc[j];
    end
endgenerate



// Handle tx requests from IP ports.
integer k;
always_comb begin
    // Loop over each IP port.
    for (k = 0; k < PORTS; k = k + 1) begin
        // If the current port is the one priority decoder is selecting.
        if (k == pri_dec_oup) begin

            $display("A TX request has been sent.");


            // If the current port is requesting to send a tx.
            if (ip_prts_inp[k].tx_submit) begin
                // If the send q isnt full.
                if (!snd_q_full) begin
                    // Present the data to the send queue.
                    snd_q_inp <= ip_prts_inp[k].dat_to_noc;

                    // Tell the send q to write in the data.
                    snd_q_we <= 1;

                    // Mark tx as complete.
                    prt_tx_cmplt[k] <= 1;
                end
                // If the send q is full.
                else begin
                    // Disable write to send q.
                    snd_q_we <= 0;

                    // Mark tx as incomplete.
                    prt_tx_cmplt[k] <= 0;
                end
            end
            // If the port does not want to submit a tx.
            else begin
                // Tell the send q to not write data.
                snd_q_we <= 0;

                // Write tx incomplete to IP port.
                prt_tx_cmplt[k] <= 0;
            end
        end
        // If the current port is not the one the priority decoder is selecting.
        else begin
            // Write tx incomplete to the IP port.
            prt_tx_cmplt[k] <= 0;
        end
    end
end

// Handle sending rx packets to IP ports.
integer l;
always_comb begin
    // If there is a packet present in the recieve queue.
    if (rcv_q_pp) begin
        // Loop over all the ports.
        for (l = 0; l < PORTS; l = l + 1) begin
            // If the rx is for this port.
            if (rcv_q_oup.dst_prt == l) begin
                // Signal that there is a packet ready to be recieved  by the port.
                prt_rx_rcv[l] <= 1;

                // Present the packet to the port.
                dat_frm_noc[l] <= rcv_q_oup;

                // Drive the rcv_q_se from the rx_complete wire from the ip port.
                rcv_q_se <= ip_prts_inp[l].rx_complete;
            end
            // If the packet is not for this port.
            else begin
                // Signal that there is nothing ready to recieve.
                prt_rx_rcv[l] <= 0;
            end
        end
    end
    // If there isnt a packet present.
    else begin
        // Stop the rcv q from shifting.
        rcv_q_se <= 0;

        // Tell the ports that there is no rx for them.
        for (l = 0; l < PORTS; l = l + 1) begin
            prt_rx_rcv[l] <= 0;
        end
    end
end

// handle reset.
integer m;
always @(posedge rst) begin
    stop_address <= ADDR;

    for (m = 0; m < PORTS; m = m + 1) begin
        to_noc_prt_stat[m] <= port_open;
        from_noc_prt_stat[m] <= port_open;
    end

end
endmodule




module noc_packet_queue(
    input clk_1,
    input clk_2,
    input rst,

    // Input stuff.
    output bit qf,
    input  we,
    input packet dat_inp,

    // Output stuff.
    output bit oup_rdy,
    input  se,
    output packet dat_oup
);

    // Queue 1.
    packet q1_oup;
    bit q1_pp;
    bit q1_se;
    packet q1_prev_oup;
    packet_queue q1(clk_1, rst, dat_inp, q1_oup, qf, q1_pp, we, q1_se);

    // Queue 2.
    bit q2_pp;
    bit q2_oe;
    bit q2_full;
    bit q2_we;
    packet_queue q2(clk_2, rst, q1_prev_oup, dat_oup, q2_full, q2_pp, q2_we, se);

    assign oup_rdy = q2_pp;

    always_comb begin
        // If current output == previous & pp is indicated.
        if (q1_prev_oup == q1_oup && q1_pp && q1_se == 0) begin
            q2_we <= 1;
        end
        else q2_we <= 0;
    end
  
    // Each time q1 is clocked update the prev output variable & check that a packet is present.
    always @(posedge clk_1) begin
        q1_prev_oup <= q1_oup;

        if (q1_se) begin
            q1_se <= 0;
        end
    end

    always @(posedge clk_2) begin
        if (q2_we) begin
            q1_se <= 1;
        end
    end

    /*

    // Logic for it to shift from q1 to q2.
    bit [1:0] q1_clkd_yet;
    always @(posedge clk_2) begin
        // give q1 a chance to update itself before we try and pull any data from it.
        if (q1_clkd_yet == 2) begin
            // If the current output of q1 is the same as previous.
            if (q1_oup == q1_prev_oup) begin
                // If q1 says there is a packet present.
                if (q1_pp) begin
                    // If q2 has enough space.
                    if (q2_full == 0) begin
                        // Signal that we want to input to q2.
                        q2_we <= 1;

                        // Signal that we want q1 to shift on next clock_1.
                        q1_se <= 1;

                        // Signal that Q1 has not clocked since this has ran.
                        q1_clkd_yet <= 0;
                    end
                    // If q2 does not have enough space.
                    else begin
                        // Signal that we dont want to write to q2.
                        q2_we <= 0;

                        // Signal that we dont want q1 to shift on next clock_1.
                        q1_se <= 0;
                    end
                end
                // if q1 says there is not a packet present.
                else begin
                    // Signal that we dont want to write to q2.
                    q2_we <= 0;

                    // Signal that we dont want q1 to shift on next clock_1.
                    q1_se <= 0;
                end
            end
            // If the current & prev output of q1 does not match.
            else begin
                // Signal that we dont want to write to q2.
                q2_we <= 0;

                // Signal that we dont want q1 to shift on next clock_1.
                q1_se <= 0;
            end
        end

        // Update the previous q1 output value register.
        q1_prev_oup <= q1_oup;
    end

    always @(posedge clk_1) begin
        // Set Q1 shift enable low now that it has shifted.
        if (q1_se) q1_se <= 0;

        // Signal that Q1 has now clocked.
        if (q1_clkd_yet == 0) q1_clkd_yet <= 1;
        if (q1_clkd_yet == 1) q1_clkd_yet <= 2;
    end
    */
endmodule


/*          Instruction queue           */
module packet_queue(
    input           clk,
    input           rst,
    
    input packet    d_in,
    output packet   d_out,
    output          full,
    output          packet_present,

    input           we,
    input           oe
);

    packet packets [7:0];
    reg [7:0] used_positions;

    assign full = used_positions == 8 ? 1 : 0;
    assign packet_present = used_positions == 0 ? 0 : 1;
    assign d_out = packets[0];


    // handle input & output.
    integer i;
    always @(posedge clk) begin
        
        case ({oe, we})

            // Doing nothing.
            2'b00:
            begin
            end

            // Just writing new value.
            2'b01:
            begin
                if (used_positions < 8) begin
                    // Write the new value & incriment the number of used positions.
                    packets[used_positions] <= d_in;
                    used_positions <= used_positions + 1;
                end
            end

            // Just outputting.
            2'b10:
            begin
                if (used_positions != 0) begin
                    // Shift the values.
                    for (i = 0; i < 8; i = i + 1) begin
                        packets[i] <= packets[i + 1];
                    end

                    // Decriment the used_positions variable.
                    used_positions <= used_positions - 1;
                end
            end

            // Outputting & Writing.
            2'b11:
            begin
                // If we have between 1 and 7 values stored.
                if (used_positions != 0 && used_positions < 8) begin
                    // Shift existing values.  
                    for (i = 0; i < used_positions-1; i = i + 1) begin
                        packets[i] <= packets[i + 1];
                    end

                    // Append new value.
                    packets[used_positions-1] <= d_in;
                end
                if (used_positions == 0) begin
                    // Store the value.
                    packets[used_positions] <= d_in;

                    // Incriment used_positions.
                    used_positions <= used_positions + 1;
                end
            end
        endcase
    end


    // Handle reset.
    always @(posedge rst) begin
        for (i = 0; i < 8; i = i + 1) begin
            packets[i] = 0;
        end

        used_positions = 0;
    end
endmodule

/*          Priority decoder            */
module priority_decoder #(parameter INPUT_WIDTH = 4, OUTPUT_WIDTH = 2)(
    input logic [INPUT_WIDTH] inp,
    output logic [OUTPUT_WIDTH] oup
);

    // Decode the priority.
    integer i;
    always_comb begin
        // Last assignment wins in always block so we just do simple loop to find highest priority bit.
        for (i = 0; i <= INPUT_WIDTH; i = i + 1) begin
            if (inp[i]) oup = i;
        end
    end
endmodule
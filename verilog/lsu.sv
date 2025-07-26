`include "structs.svh"
`include "defines.svh"

// Will significantly reduce duplicate logic here once I re do instruction encodings.
module load_store_unit(
    input   logic               clk,
    input   logic               rst,
    input   logic               en,
    output  logic               dn,

    input   queued_instruction  instr,

    input   logic [31:0]        gpr_oup [15:0],
    output  logic [31:0]        gpr_inp [15:0],
    output  logic [15:0]        gpr_we,

    // Interface to the memory subsystem.
    output  logic               mac_prt_tx_rp,
    output  line_acc_req        mac_prt_tx_req,
    input   logic               mac_prt_tx_ra,

    input   logic               mac_prt_rx_rp,
    input   line_acc_req        mac_prt_rx_req,
    output  logic               mac_prt_rx_ra
);

logic [31:0] load_lb;
logic [31:0] load_ub;
logic [3:0] state;
int byte_cnt;
always @(posedge clk) begin
    // Default values.
    dn = 0;
    gpr_we = 0;
    mac_prt_tx_rp = 0;
    mac_prt_rx_ra = 0;
    byte_cnt = 0;

    // If enabled.
    if (en) begin
        case (instr.bits[7:3])
        // LOD k, A
        5'b00000:
        begin
            $display ("Executing LSU LOD k, A instruction, %t", $time);
       
            // Assign the lower bound and upper bound of the load.
            load_lb = instr.bits[47:16];
            load_ub = load_lb + 3;

            // Check the load can be done aligned.
            if (load_lb[31:4] == load_ub[31:4]) begin
                case(state)
                // Read the line.
                0:
                begin
                    // If the cmac accepts the read request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign some stuffs.
                        mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                        mac_prt_tx_req.rqt <= 0;

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // Wait for read reply from cmac.
                1:
                begin
                    // If cmac replies with the requested read.
                    if (mac_prt_rx_rp) begin

                        // Signal request accepted.
                        mac_prt_rx_ra <= 1;

                        // Check that the read reply is the addr we want.
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 0}) begin

                            // Present bytes to the gpr input.
                            gpr_inp[instr.bits[11:8]] <= mac_prt_rx_req.dat[(load_lb[3:0] * 8) +:32];
                            // Signal write to the GPR.
                            gpr_we[instr.bits[11:8]] <= 1;

                            // Write will be performed on next edge so signal done.
                            dn <= 1;

                            // Reset state to 0.
                            state <= 0;
                        end

                        // Could add an else here to signal error or panic.
                    end
                end
                endcase
            end
            // If an aligned load can not be performed.
            else begin
                case (state)

                // State 0 (read request to top line).
                0:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign some stuffs.
                        mac_prt_tx_req.addr <= {load_ub[31:4],0};
                        mac_prt_tx_req.rqt <= 0;

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for the read to return)
                1:
                begin
                    // If the cmac has a reply for our read.
                    if (mac_prt_rx_rp) begin

                        // Signal reply accepted to cmac.
                        mac_prt_rx_ra <= 1;

                        // Check that the read reply is the addr we want.
                        if (mac_prt_rx_req.addr == {load_ub[31:4], 0}) begin

                            // Assign bits to the gpr value input.
                            for (int i = 3; i >= 0; i = i - 1) begin
                                // If the byte is in range.
                                if ( i <= load_ub[3:0]) begin
                                    // Assign the byte from read reply to the gpr_inp.
                                    gpr_inp[instr.bits[11:8]][(3-byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i * 8) +:8];
                                    // Incriment byte_cnt.
                                    byte_cnt = byte_cnt + 1;
                                end
                            end

                            // Once all bytes are assigned, move to stage 2.
                            state <= 2;
                        end
                    end
                end

                // State 2 (submit second line read)
                2:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 3;
                    end
                    else begin
                        // Assign some stuffs.
                        mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                        mac_prt_tx_req.rqt <= 0;

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 3 (wait for second line read reply)
                3:
                begin
                    // If the cmac has a reply for our read.
                    if (mac_prt_rx_rp) begin

                        // Signal reply accepted to cmac.
                        mac_prt_rx_ra <= 1;

                        // Check that the read reply is the addr we want.
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 0}) begin

                            // Assign bits to the gpr value input.
                            for (int i = 12; i <= 15; i = i + 1) begin
                                // If the byte is in range.
                                if ( i >= load_lb[3:0]) begin
                                    // Assign the byte from read reply to the gpr_inp.
                                    gpr_inp[instr.bits[11:8]][(byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i * 8) +:8];
                                    // Incriment byte_cnt.
                                    byte_cnt = byte_cnt + 1;
                                end
                            end

                            // Signal gpr write
                            gpr_we[instr.bits[11:8]] <= 1;

                            // Signal done & return to state 0.
                            dn <= 1;
                            state <= 0;
                        end

                        // Could add error handling here in future to send interrupt and produce trace.
                    end
                end
                endcase
            end
        end

        // LOD A, B
        5'b11001:
        begin
            $display ("Executing LSU LOD A, B instruction, %t", $time);

            // Assign the lower bound and upper bound of the load.
            load_lb = gpr_oup[instr.bits[11:8]];
            load_ub = load_lb + 3;

            // Check if the load can be done aligned.
            if (load_lb[31:4] == load_ub[31:4]) begin
                case(state)
                // Read line.
                0:
                begin
                    // If the cmac accepts the read request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign request addr & request type read.
                        mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                        mac_prt_tx_req.rqt <= 0;

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // Wait for read reply from cmac.
                1:
                begin
                    // If cmac replies with the requested read.
                    if (mac_prt_rx_rp) begin

                        // Signal request accepted.
                        mac_prt_rx_ra <= 1;

                        // Check that the read reply is the addr we want.
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 0}) begin
                            // Present bytes to the gpr input.
                            gpr_inp[instr.bits[15:12]] <= mac_prt_rx_req.dat[(load_lb[3:0] *8 )+:32];
                            // Signal write to the GPR.
                            gpr_we[instr.bits[15:12]] <= 1;

                            // Signal done.
                            dn <= 1;

                            // Reset state to 0.
                            state <= 0;
                        end

                        // Could add an interrupt here to signal an error.
                    end
                end
                endcase
            end
            // If aligned read can not be performed.
            else begin
                case (state)

                // State 0 (read request to top line).
                0:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign some stuffs.
                        mac_prt_tx_req.addr <= {load_ub[31:4],0};
                        mac_prt_tx_req.rqt <= 0;

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for the read to return)
                1:
                begin
                    // If the cmac has a reply for our read.
                    if (mac_prt_rx_rp) begin

                        // Signal reply accepted to cmac.
                        mac_prt_rx_ra <= 1;

                        // Check that the read reply is the addr we want.
                        if (mac_prt_rx_req.addr == {load_ub[31:4], 0}) begin

                            // Assign bits to the gpr value input.
                            for (int i = 3; i >= 0; i = i - 1) begin
                                // If the byte is in range.
                                if ( i <= load_ub[3:0]) begin
                                    // Assign the byte from read reply to the gpr_inp.
                                    gpr_inp[instr.bits[15:12]][(3-byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i * 8) +:8];
                                    // Incriment byte_cnt.
                                    byte_cnt = byte_cnt + 1;
                                end
                            end

                            // Once all bytes are assigned, move to stage 2.
                            state <= 2;
                        end
                    end
                end

                // State 2 (submit second line read)
                2:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 3;
                    end
                    else begin
                        // Assign some stuffs.
                        mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                        mac_prt_tx_req.rqt <= 0;

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 3 (wait for second line read reply)
                3:
                begin
                    // If the cmac has a reply for our read.
                    if (mac_prt_rx_rp) begin

                        // Signal reply accepted to cmac.
                        mac_prt_rx_ra <= 1;

                        // Check that the read reply is the addr we want.
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 0}) begin

                            // Assign bits to the gpr value input.
                            for (int i = 12; i <= 15; i = i + 1) begin
                                // If the byte is in range.
                                if ( i >= load_lb[3:0]) begin
                                    // Assign the byte from read reply to the gpr_inp.
                                    gpr_inp[instr.bits[15:12]][(byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i * 8) +:8];
                                    // Incriment byte_cnt.
                                    byte_cnt = byte_cnt + 1;
                                end
                            end

                            // Signal gpr write
                            gpr_we[instr.bits[15:12]] <= 1;

                            // Signal done & return to state 0.
                            dn <= 1;
                            state <= 0;
                        end

                        // Could add error handling here in future to send interrupt and produce trace.
                    end
                end
                endcase
            end

        end

        // STR A, k
        5'b10000:
        begin
            $display ("Executing LSU STR A, k instruction, %t", $time);

            // Assign the lower bound and upper bound of the store operation.
            load_lb = instr.bits[16+:32];
            load_ub = load_lb + 3;

            // Check if the store can be done aligned.
            if (load_lb[31:4] == load_ub[31:4]) begin

                // If the cmac accepts the request.
                if (mac_prt_tx_ra) begin
                    // Signal done.
                    dn <= 1;
                end
                else begin
                    // Assign address & rqt.
                    mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                    mac_prt_tx_req.rqt <= 1; // Write req.

                    // Assign write mask and data.
                    for (int i = 0; i < 16; i = i + 1) begin
                        // If the byte is in range.
                        if (i >= load_lb[3:0] & i <= load_ub[3:0]) begin
                            mac_prt_tx_req.wmsk[i] <= 1;
                        end
                    end
                    mac_prt_tx_req.dat[load_lb[3:0]+:32] <= gpr_oup[instr.bits[11:8]];

                    // Signal request present to the mem acc cont.
                    mac_prt_tx_rp <= 1;
                end
            end
            // If can not be performed aligned.
            else begin
                case(state)
                // Submit write request for lower line
                0:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign address & rqt.
                        mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                        mac_prt_tx_req.rqt <= 1; // Write req.

                        // Assign write mask and data.
                        for (int i = 12; i < 16; i = i + 1) begin
                            // If the byte is in range.
                            if (i >= load_lb[3:0]) begin
                                // Set write mask.
                                mac_prt_tx_req.wmsk[i] <= 1;
                                // Set byte to write.
                                mac_prt_tx_req.dat[(i*8)+:8] <= gpr_oup[instr.bits[11:8]][(byte_cnt*8)+:8];
                                // Incriment byte_cnt.
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // Submit write request for upper line
                1:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Reset state to 0.
                        state <= 0;
                        // Signal done.
                        dn <= 1;
                    end
                    else begin
                        // Assign address & rqt.
                        mac_prt_tx_req.addr <= {load_ub[31:4], 0};
                        mac_prt_tx_req.rqt <= 1; // Write req.

                        // Assign write mask and data.
                        for (int i = 3; i >= 0; i = i - 1) begin
                            // If the byte is in range.
                            if (i <= load_ub[3:0]) begin
                                // Set write mask.
                                mac_prt_tx_req.wmsk[i] <= 1;
                                // Set byte to write.
                                mac_prt_tx_req.dat[(i*8)+:8] <= gpr_oup[instr.bits[11:8]][(3-byte_cnt)*8+:8];
                                // Incriment byte_cnt.
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end
                endcase
            end
        end

        // MODIFY THIS.
        // STR A, B
        5'b10000:
        begin
            $display ("Executing LSU STR A, B instruction, %t", $time);

            // Assign the lower bound and upper bound of the store operation.
            load_lb = gpr_oup[instr.bits[12+:4]];
            load_ub = load_lb + 3;

            // Check if the store can be done aligned.
            if (load_lb[31:4] == load_ub[31:4]) begin

                // If the cmac accepts the request.
                if (mac_prt_tx_ra) begin
                    // Signal done.
                    dn <= 1;
                end
                else begin
                    // Assign address & rqt.
                    mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                    mac_prt_tx_req.rqt <= 1; // Write req.

                    // Assign write mask and data.
                    for (int i = 0; i < 16; i = i + 1) begin
                        // If the byte is in range.
                        if (i >= load_lb[3:0] & i <= load_ub[3:0]) begin
                            mac_prt_tx_req.wmsk[i] <= 1;
                        end
                    end
                    mac_prt_tx_req.dat[load_lb[3:0]+:32] <= gpr_oup[instr.bits[11:8]];

                    // Signal request present to the mem acc cont.
                    mac_prt_tx_rp <= 1;
                end
            end
            // If can not be performed aligned.
            else begin
                case(state)
                // Submit write request for lower line
                0:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign address & rqt.
                        mac_prt_tx_req.addr <= {load_lb[31:4], 0};
                        mac_prt_tx_req.rqt <= 1; // Write req.

                        // Assign write mask and data.
                        for (int i = 12; i < 16; i = i + 1) begin
                            // If the byte is in range.
                            if (i >= load_lb[3:0]) begin
                                // Set write mask.
                                mac_prt_tx_req.wmsk[i] <= 1;
                                // Set byte to write.
                                mac_prt_tx_req.dat[(i*8)+:8] <= gpr_oup[instr.bits[11:8]][(byte_cnt*8)+:8];
                                // Incriment byte_cnt.
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // Submit write request for upper line
                1:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Reset state to 0.
                        state <= 0;
                        // Signal done.
                        dn <= 1;
                    end
                    else begin
                        // Assign address & rqt.
                        mac_prt_tx_req.addr <= {load_ub[31:4], 0};
                        mac_prt_tx_req.rqt <= 1; // Write req.

                        // Assign write mask and data.
                        for (int i = 3; i >= 0; i = i - 1) begin
                            // If the byte is in range.
                            if (i <= load_ub[3:0]) begin
                                // Set write mask.
                                mac_prt_tx_req.wmsk[i] <= 1;
                                // Set byte to write.
                                mac_prt_tx_req.dat[(i*8)+:8] <= gpr_oup[instr.bits[11:8]][(3-byte_cnt)*8+:8];
                                // Incriment byte_cnt.
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end
                endcase
            end
        end
        endcase
    end
    // If not enabled.
    else begin
        state <= 0;
    end

end
endmodule


// OLD MODULE
/*
module memory_and_io_unit (
    input               clk,
    input               rst,
    input               en,
    output bit          done,

    input queued_instruction        instruction,
    input [31:0]    gpr_oup [15:0],
    output bit [31:0]   gpr_inp [15:0],
    output bit [15:0]   gpr_we,

    inout ip_port noc_port
);

// noc port stuff.
packet tx_packet;
bit submit_tx;
bit complete_rx;
assign noc_port.dat_to_noc = tx_packet;
assign noc_port.tx_submit = submit_tx;
assign noc_port.rx_complete = complete_rx;

// Process instruction.
bit [3:0] mcics;    // Multi Cycle Instruction Current Stage.
always_comb begin
    // default values.
    done <= 0;
    complete_rx <= 0;
    submit_tx <= 0;
    gpr_we <= 0;


    // If enabled.
    if (en) begin
        case(instruction.inst[7:3])
        // LOD k, A
        5'b00000:
        begin
            $display("Executing M&IO LOD k, A instruction, %t", $time);

            // Do the work but dont handle transitioning state.
            case(mcics)
            // Submit tx to read in data.
            0:
            begin
                // Submit the tx request.
                submit_tx <= 1;

                // signal done is 0.
                done <= 0;
                
                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 2;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= memory_read_request;
                tx_packet.dat[31:0] <= instruction.inst[47:16]; // Location to read from.
                tx_packet.dat[127:32] <= 0;
            end

            // Wait for reply.
            1:
            begin
                // Stop asking to submit a tx.
                submit_tx <= 0;

                // If we have an rx waiting for us and its a memory read reply.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin

                    // Signal that we have completed the rx.
                    complete_rx <= 1;

                    //prepare to write the value to gprs.
                    gpr_inp[instruction.inst[11:8]] <= noc_port.dat_from_noc.dat[31:0];
                    gpr_we[instruction.inst[11:8]] <= 1;

                    // Signal done.
                    done <= 1;
                end
            end
            endcase
        end

        // LOD A, B
        5'b11001:
        begin
            $display("Executing M&IO LOD A, B instruction, %t", $time);

            // Do the work but dont handle transitioning state.
            case(mcics)
            // Submit tx to read in data.
            0:
            begin
                // Submit the tx request.
                submit_tx <= 1;

                // signal done is 0.
                done <= 0;
                
                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 2;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= memory_read_request;
                tx_packet.dat[31:0] <= gpr_oup[instruction.inst[11:8]]; // Location to read from.
                tx_packet.dat[127:32] <= 0;
            end

            // Wait for reply.
            1:
            begin
                // Stop asking to submit a tx.
                submit_tx <= 0;

                // If we have an rx waiting for us and its a memory read reply.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin

                    // Signal that we have completed the rx.
                    complete_rx <= 1;

                    //prepare to write the value to gprs.
                    gpr_inp[instruction.inst[15:12]] <= noc_port.dat_from_noc.dat[31:0];
                    gpr_we[instruction.inst[15:12]] <= 1;

                    // Signal done.
                    done <= 1;
                end
            end
            endcase
        end

        // STR A, k
        5'b10000:
        begin

            $display("Executing M&IO STR A, k instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 2;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= memory_write_request;
                tx_packet.dat[31:0] <= instruction.inst[47:16]; // Location to write to.
                tx_packet.dat[63:32] <= gpr_oup[instruction.inst[11:8]]; // load in the data from GPR that is to be written to memory.
                tx_packet.dat[127:64] <= 0;
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_write_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;
                end
            end
            endcase
        end

        // STR A, B
        5'b01001:
        begin
            
            $display("Executing M&IO STR A, B instruction, A = %d, B = %d  %t", gpr_oup[instruction.inst[11:8]], gpr_oup[instruction.inst[15:12]], $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 2;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= memory_write_request;
                tx_packet.dat[31:0] <=  gpr_oup[instruction.inst[15:12]]; // GPR containing address to write to.
                tx_packet.dat[63:32] <= gpr_oup[instruction.inst[11:8]]; // load in the data from GPR that is to be written to memory.
                tx_packet.dat[127:64] <= 0;
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_write_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;
                end
            end
            endcase
        end

        // GIO k, I
        5'b00010:
        begin

            $display("Executing M&IO GIO k, I instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 3;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= gpio_set_val_request;
                tx_packet.dat[11:0] <= instruction.inst[19:8]; // Data to send to GPIO block.
                tx_packet.dat[127:12] <= 0; // Zero the rest of the transaction.
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;
                end
            end
            endcase
        end

        // GIO A, I
        5'b00001:
        begin

            $display("Executing M&IO GIO A, I instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 3;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= gpio_set_val_request;
                tx_packet.dat[3:0] <= instruction.inst[11:8]; // write in the GPIO pin to be written to.
                tx_packet.dat[11:4] <= gpr_oup[instruction.inst[15:12]][7:0]; // Data to send to GPIO block.
                tx_packet.dat[127:12] <= 0; // Zero the rest of the transaction.
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;
                end
            end
            endcase
        end

        // GII I, A
        5'b10001:
        begin

            $display("Executing M&IO GII I, A instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 3;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= gpio_read_val_request;
                tx_packet.dat[3:0] <= instruction.inst[11:8]; // write in the GPIO pin to be read from.
                tx_packet.dat[127:4] <= 0; // Zero the rest of the transaction.
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;

                    // Write back the result to gprs.
                    gpr_inp[instruction.inst[15:12]][7:0] <= noc_port.dat_from_noc.dat[7:0];
                    gpr_inp[instruction.inst[15:12]][31:8] <= 0;

                    gpr_we[instruction.inst[15:12]] <= 1;
                end
            end
            endcase
        end
        endcase
    end
end

always @(posedge clk) begin
    case(instruction.inst[7:3])
    // LOD k, A
    5'b00000:
    begin
        case (mcics)
        // Beginning.
        0:
        begin
            // If the port is open and the NOC replies tx_complete then we can move onto next stage.
            if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                mcics <= mcics + 1;
            end
        end
        // Wait for reply to come back.
        1:
        begin
            // If the NOC signals that it has an rx for us.
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // LOD A, B
    5'b11001:
    begin
        case (mcics)
        // Beginning.
        0:
        begin
            // If the port is open and the NOC replies tx_complete then we can move onto next stage.
            if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                mcics <= mcics + 1;
            end
        end
        // Wait for reply to come back.
        1:
        begin
            // If the NOC signals that it has an rx for us.
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // STR k, A
    5'b10000:
    begin
        case (mcics)
        // Beginning.
        0:
        begin
            // If the port is open and the NOC replies tx_complete then we can move onto next stage.
            if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                mcics <= mcics + 1;
            end
        end
        // Wait for reply to come back.
        1:
        begin
            // If the NOC signals that it has an rx for us.
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_write_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // STR A, B
    5'b01001:
    begin
        case (mcics)
        // Beginning.
        0:
        begin
            // If the port is open and the NOC replies tx_complete then we can move onto next stage.
            if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                mcics <= mcics + 1;
            end
        end
        // Wait for reply to come back.
        1:
        begin
            // If the NOC signals that it has an rx for us.
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_write_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // GIO k, I
    5'b00010:
    begin
        case (mcics)
        // Beginning.
        0:
        begin
            // If the port is open and the NOC replies tx_complete then we can move onto next stage.
            if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                mcics <= mcics + 1;
            end
        end
        // Wait for reply to come back.
        1:
        begin
            // If the NOC signals that it has an rx for us.
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // GIO A, I
    5'b00001:
    begin
        case (mcics)
        // Beginning.
        0:
        begin
            // If the port is open and the NOC replies tx_complete then we can move onto next stage.
            if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                mcics <= mcics + 1;
            end
        end
        // Wait for reply to come back.
        1:
        begin
            // If the NOC signals that it has an rx for us.
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // GII I, A
    5'b10001:
    begin
        case (mcics)
        // Beginning.
        0:
        begin
            // If the port is open and the NOC replies tx_complete then we can move onto next stage.
            if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                mcics <= mcics + 1;
            end
        end
        // Wait for reply to come back.
        1:
        begin
            // If the NOC signals that it has an rx for us.
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_read_val_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end
    endcase
end

endmodule
*/
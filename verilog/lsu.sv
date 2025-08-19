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
    dn <= 0;
    gpr_we <= 0;
    mac_prt_tx_rp <= 0;
    mac_prt_rx_ra <= 0;
    mac_prt_tx_req <= 0;
    byte_cnt = 0;

    // If enabled.
    if (en) begin
        case (instr.bits[7:3])
        // LOD k, A
        5'b00000:
        begin
            //$display ("Executing LSU LOD k, A instruction, %t", $time);
            $display ("Exec LSU LOD k(0x%h), A(0x%h) IP:0x%h %t", instr.bits[16+:32], instr.bits[8+:4], instr.addr, $time);
       
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
                        mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
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
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 4'b0000}) begin

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
                        mac_prt_tx_req.addr <= {load_ub[31:4], 4'b0000};
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
                        if (mac_prt_rx_req.addr == {load_ub[31:4], 4'b0000}) begin

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
                        mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
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
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 4'b0000}) begin

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
            $display ("Exec LSU LOD A(0x%h), B(0x%h) IP:0x%h %t", gpr_oup[instr.bits[8+:4]], instr.bits[12+:4], instr.addr, $time);

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
                        mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
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
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 4'b0000}) begin
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
                        mac_prt_tx_req.addr <= {load_ub[31:4], 4'b0000};
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
                        if (mac_prt_rx_req.addr == {load_ub[31:4], 4'b0000}) begin

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
                        mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
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
                        if (mac_prt_rx_req.addr == {load_lb[31:4], 4'b0000}) begin

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
            $display("Exec LSU STR A(0x%h), k(0x%h) IP:0x%h %t", gpr_oup[instr.bits[8+:4]], instr.bits[16+:32], instr.addr, $time);

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
                    mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
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
                        mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
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
                        mac_prt_tx_req.addr <= {load_ub[31:4], 4'b0000};
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

        // STR A, B
        5'b01001:
        begin
            $display("Exec LSU STR A(0x%h), B(0x%h) IP:0x%h %t", gpr_oup[instr.bits[8+:4]], gpr_oup[instr.bits[12+:4]], instr.addr, $time);


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
                    mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
                    mac_prt_tx_req.rqt <= 1; // Write req.

                    // Assign write mask and data.
                    for (int i = 0; i < 16; i = i + 1) begin
                        // If the byte is in range.
                        if (i >= load_lb[3:0] & i <= load_ub[3:0]) begin
                            mac_prt_tx_req.wmsk[i] <= 1;
                        end
                    end
                    mac_prt_tx_req.dat[(load_lb[3:0] * 8)+:32] <= gpr_oup[instr.bits[11:8]];

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
                        mac_prt_tx_req.addr <= {load_lb[31:4], 4'b0000};
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
                        mac_prt_tx_req.addr <= {load_ub[31:4], 4'b0000};
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
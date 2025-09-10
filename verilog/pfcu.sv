`include "structs.svh"
`include "defines.svh"

//`define PFCU_DEBUG_LOG

module program_flow_control #(parameter NUM_PHYSICAL_REGS = 64) (
    input   logic                                       clk,                // Clock.
    input   logic                                       rst,                // Reset.
    input   logic                                       en,                 // PFCU enable.
    output  logic                                       dn,                 // PFCU done.

    // IFE interface.
    output  logic                                       jmp,                // Jump signal.
    output  logic [31:0]                                new_pc,             // new program counter value.
    input   micro_op                                    inst,               // Instruction to execute.

    // PRF interface.
    output  logic [3:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  rf_rd_trgt,         // RF read port target register.
    input   logic [3:0][31:0]                           rf_rd_dat,          // RF read port register data.

    output  logic [$clog2(NUM_PHYSICAL_REGS)-1:0]       rf_wr_trgt,         // RF write port target register.
    output  logic [31:0]                                rf_wr_dat,          // RF write port register data.
    output  logic                                       rf_we,              // RF write port write enable.


    // Interface to the memory subsystem.
    output  logic                                       mac_prt_tx_rp,      // Mem acc cont tx request present.
    output  line_acc_req                                mac_prt_tx_req,     // Mem acc cont tx request.
    input   logic                                       mac_prt_tx_ra,      // Mem acc cont tx request accepted.

    input   logic                                       mac_prt_rx_rp,      // Mem acc cont rx present.
    input   line_acc_req                                mac_prt_rx_req,     // Mem acc cont rx data.
    output  logic                                       mac_prt_rx_ra       // Mem acc cont rx accepted.
);

    // Operands.
    logic [31:0] jmp_addr;
    logic [31:0] alu_stat;
    logic [31:0] shp_read;
    logic [31:0] curr_pc;
    logic [31:0] shp_write;
    logic write_shp_val;



    // Handle assigning PRF read and write ports.
    always_comb begin
        // Defaults.
        rf_rd_trgt  = 0;
        rf_wr_trgt  = 0;
        rf_wr_dat   = 0;
        rf_we       = 0;

        // If enabled hook up the operands to PRF.
        if (en) begin
            rf_rd_trgt[0] = inst.operand_a;
            jmp_addr = rf_rd_dat[0];

            rf_rd_trgt[1] = inst.operand_b;
            alu_stat = rf_rd_dat[1];

            rf_rd_trgt[2] = inst.operand_c;
            shp_read = rf_rd_dat[2];

            rf_rd_trgt[3] = inst.operand_d;
            curr_pc = rf_rd_dat[3];

            rf_wr_trgt = inst.operand_e;
            rf_wr_dat = shp_write;
            rf_we = write_shp_val;
        end
    end

    int         byte_cnt;
    logic [3:0] state;
always @(posedge clk) begin
    // Default values.
    jmp             <= 0;
    dn              <= 0;
    mac_prt_tx_rp   <= 0;
    mac_prt_tx_req  <= 0;
    mac_prt_rx_ra   <= 0;
    byte_cnt         = 0;
    write_shp_val   <= 0;

    // If enabled.
    if (en) begin
        case (inst.operation)

        // JMP k
        4'b0000:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display("Exec PFCU JMP K \toprnds: %d, %d, %d, %d, %d %t", inst.operand_a, inst.operand_b, inst.operand_c, inst.operand_d, inst.operand_e , $time);
            `endif

            // For testing purposes to finish simulation once test program has completed.
            if (jmp_addr == 32'b0000_0000_0000_0000_0000_0000_1011_1001) begin $display("Program Finished %t", $time); $stop; end

            // Present new pc, signal jump & done.
            new_pc <= jmp_addr;
            dn <= 1;
            jmp <= 1;
        end

        // JIZ k
        4'b0001:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display("Exec PFCU JIZ K \toprnds: %d, %d, %d, %d, %d %t", inst.operand_a, inst.operand_b, inst.operand_c, inst.operand_d, inst.operand_e , $time);
            `endif
            // Check if ALU status [0] is set (last alu result = 0).
            if (alu_stat[0]) begin
                // Present new pc and signal jump
                new_pc <= jmp_addr;
                jmp <= 1;
            end
            // Signal done.
            dn <= 1;
        end

        // GOT k
        4'b0010:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display("Exec PFCU GOT K \toprnds: %d, %d, %d, %d, %d %t", inst.operand_a, inst.operand_b, inst.operand_c, inst.operand_d, inst.operand_e , $time);
            `endif
            // prepare new stack head pointer value.
            shp_write = shp_read - 4;       // shp_write = new stack head pointer val,  shp_read = current stack head pointer val.

            // Check if an aligned write can be done.
            if (shp_write[31:4] == shp_read[31:4]) begin
                `ifdef PFCU_DEBUG_LOG
                $display("Aligned PFCU GOT can be performed.");
                `endif
                // If the cmac accepts the request.
                if (mac_prt_tx_ra) begin
                    // Signal done.
                    dn <= 1;
                    // Signal jmp & set new pc.
                    jmp <= 1;
                    new_pc <= jmp_addr;
                    // Signal to overwrite the stack head pointer.
                    write_shp_val <= 1;
                    // Signal that there is no write tx present to cmac.
                    mac_prt_tx_rp <= 0;
                end
                else begin
                    // Assign some write stuffs.
                    mac_prt_tx_req.addr <= {shp_write[31:4], 4'b0000};    // Dont forget the 4'b0000 or else it interprets 0 as 32'b0...
                    mac_prt_tx_req.rqt <= 1;

                    // Loop over the 16 bytes and set writemask.
                    for (int i = 0; i < 16; i = i + 1) begin
                        // If the byte is in range of the bytes to write, set wmsk.
                        if (i >= shp_write[3:0] & i < shp_read[3:0]) begin
                            mac_prt_tx_req.wmsk[i] <= 1;
                        end
                        else mac_prt_tx_req.wmsk[i] <= 0;
                    end
                    // Write the current PC value.
                    mac_prt_tx_req.dat[(shp_write[3:0] * 8)+:32] <= curr_pc;        // Little endian

                    // Signal request present to the mem acc cont.
                    mac_prt_tx_rp <= 1;
                end
            end
            // If an aligned write can not be performend.
            else begin
                `ifdef PFCU_DEBUG_LOG
                $display("Aligned PFCU GOT k can not be performed.");
                `endif
                case(state)
                // First line write (top).
                0:
                begin

                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Move to the next state.
                        state <= 1;
                    end
                    else begin
                        // Assign some write stuffs.
                        mac_prt_tx_req.addr <= {shp_read[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 1;

                        // Loop down through the bottom 3 bytes, assign write mask and data.
                        for (int i = 3; i >= 0; i = i - 1) begin
                            // If the byte is in range to write PC to.
                            if (i < shp_read[3:0]) begin
                                // Assign write mask.
                                mac_prt_tx_req.wmsk[i] <= 1;
                                // Assign data byte.
                                mac_prt_tx_req.dat[(8*i)+:8] <= curr_pc[8*(3-byte_cnt)+:8];
                                // Incriment byte_cnt.
                                byte_cnt = byte_cnt + 1;
                            end
                        end


                        // I feel like this must be wrong, looks like its filling the bytes backwards??
                        // Loop over bottom 4 bytes and assign write mask.
                        //for (int i = 0; i < 4; i = i + 1) begin
                            //if (i < shp_read[3:0]) begin
                                //mac_prt_tx_req.wmsk[i] <= 1;
                                //mac_prt_tx_req.dat[(8*i)+:8] <= curr_pc[8*(4-byte_cnt)+:8];
                                //byte_cnt = byte_cnt + 1;
                            //end
                        //end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // Second line write (bottom).
                1:
                begin

                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Signal done.
                        dn <= 1;
                        // Signal jmp & set new pc.
                        jmp <= 1;
                        new_pc <= jmp_addr;
                        // Signal to overwrite the stack head pointer.
                        write_shp_val <= 1;
                        // Signal that there is no write tx present to cmac.
                        mac_prt_tx_rp <= 0;
                        // Reset state to 0.
                        state <= 0;
                    end
                    else begin
                        // Assign some write stuffs.
                        mac_prt_tx_req.addr <= {shp_write[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 1;

                        // Loop over top 4 bytes and assign write mask.
                        // Can prolly just do i = 11; i < 16 here but oh well...
                        for (int i = 0; i < 5; i = i + 1) begin
                            if (i + 11 >= shp_write[3:0]) begin
                                mac_prt_tx_req.wmsk[11+i] <= 1;
                                mac_prt_tx_req.dat[(11 + i)*8 +:8] <= curr_pc[8*(byte_cnt)+:8];
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

        // RET
        4'b0011:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display("Exec PFCU RET \toprnds: %d, %d, %d, %d, %d %t", inst.operand_a, inst.operand_b, inst.operand_c, inst.operand_d, inst.operand_e , $time);
            `endif
            // Prepare new stack head pointer value.
            shp_write = shp_read + 4;

            // Check if an aligned read can be done.
            if (shp_write[31:4] == shp_read[31:4]) begin

                case(state) 

                // State 0 (request to read the line).
                0:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_read[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for reply).
                1:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // Assign the data to the new_pc
                        new_pc <= mac_prt_rx_req.dat[(shp_read[3:0] * 8)+:32];

                        // Signal jump, write_shp_val & done.
                        jmp <= 1;
                        dn <= 1;
                        write_shp_val <= 1;

                        // signal rx accepted.
                        mac_prt_rx_ra <= 1;

                        // move state back to 0.
                        state <= 0;
                    end
                end
                endcase
            end
            // If an aligned request can not be made. 
            else begin

                case (state)

                // State 0 (read request to top line).
                0:
                begin
                    if (mac_prt_tx_ra) state <= 1;
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_write[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for reply for top line read).
                1:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // assign bytes to new pc val.
                        for (int i = 3; i >= 0; i = i - 1) begin
                            if (i < shp_write[3:0]) begin
                                new_pc[(3-byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i*8)+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal rx accepted.
                        mac_prt_rx_ra <= 1;

                        // Move to state 2.
                        state <= 2;
                    end
                end

                // State 2 (read request to the bottom line).
                2:
                begin
                    if (mac_prt_tx_ra) state <= 3;
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_read[31:0], 4'b0000};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that the request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 3 (wait for reply for bottom line read).
                3:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // assign bytes to new pc val.
                        for (int i = 0; i < 5; i = i + 1) begin
                            if (i + 11 >= shp_read[3:0]) begin
                                new_pc[byte_cnt+:8] <= mac_prt_rx_req.dat[(i+11)*8+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal rx accepted to the cmac.
                        mac_prt_rx_ra <= 1;

                        // Signal done.
                        dn <= 1;
                        // Signal jump.
                        jmp <= 1;
                        // Signal write_shp_val.
                        write_shp_val <= 1;
                        // Reset state to 0.
                        state <= 0;
                    end
                end
                endcase
            end
        end
        endcase
    end
    else begin
        // Reset state.
        state <= 0;
    end
end
endmodule
/*
module program_flow_control_old #(parameter NUM_REGS = 32) (
    input   logic                               clk,                // Clock.
    input   logic                               rst,                // Reset.
    input   logic                               en,                 // PFCU enable.
    output  logic                               dn,                 // PFCU done.

    // IFE interface.
    output  logic                               jmp,                // Jump signal.
    output  logic [31:0]                        new_pc,             // new program counter value.

    // Interface to stack head pointer register.
    output  logic                               shp_we,             // Stack head pointer write enable.
    output  logic [31:0]                        shp_inp,            // Stack head pointer data input.
    input   logic [31:0]                        shp_oup,            // Stack head pointer data output.

    // Instruction queue interface.
    input   decoded_instruction                 inst,               // Instruction to execute

    // GPR interface.
    output  logic [$clog2(NUM_REGS)-1:0]        gpr_rd_trgt_reg,    // RF read port target register.
    input   logic [31:0]                        gpr_rd_reg_dat,     // RF read port register data.


    // Interface to the memory subsystem.
    output  logic                               mac_prt_tx_rp,      // Mem acc cont tx request present.
    output  line_acc_req                        mac_prt_tx_req,     // Mem acc cont tx request.
    input   logic                               mac_prt_tx_ra,      // Mem acc cont tx request accepted.

    input   logic                               mac_prt_rx_rp,      // Mem acc cont rx present.
    input   line_acc_req                        mac_prt_rx_req,     // Mem acc cont rx data.
    output  logic                               mac_prt_rx_ra       // Mem acc cont rx accepted.
);

// Handle selecting value of k from PRF.
assign gpr_rd_trgt_reg = inst.operand_a;

logic [31:0]    curr_pc;
int             byte_cnt;
logic [3:0]     state;
always @(posedge clk) begin
    // Default values.
    jmp <= 0;
    dn <= 0;
    mac_prt_tx_rp <= 0;
    mac_prt_tx_req <= 0;
    mac_prt_rx_ra <= 0;
    byte_cnt = 0;
    shp_we <= 0;

    // If enabled.
    if (en) begin
        case (instr.operation)

        // JMP k
        4'b0000:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display("Exec PFCU JMP K(0x%h) IP:0x%h %t", gpr_rd_reg_dat, instr.addr, $time);
            `endif
            // For testing purposes to finish simulation once test program has completed.
            if (instr.bits == 40'b0000_0000_0000_0000_0000_0000_1011_1001_0000_1110) begin $display("Program Finished %t", $time); $stop; end
            // Present new pc, signal jump & done.
            new_pc <= gpr_rd_reg_dat;
            dn <= 1;
            jmp <= 1;
        end

        // JIZ k
        4'b0001:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display("Exec PFCU JIZ k(0x%h) IP:0x%h %t", gpr_rd_reg_dat, instr.addr, $time);
            `endif
            // Check if ALU status [0] is set (last alu result = 0).
            if (alu_stat[0]) begin
                // Present new pc and signal jump
                new_pc <= gpr_rd_reg_dat;
                jmp <= 1;
            end
            // Signal done.
            dn <= 1;
        end

        // GOT k
        4'b0010:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display("Exec PFCU GOT k(0x%h) IP:0x%h %t", gpr_rd_reg_dat, instr.addr, $time);
            `endif
            // prepare new stack head pointer value.
            shp_inp = shp_oup - 4;

            // Check if an aligned write can be done.
            if (shp_inp[31:4] == shp_oup[31:4]) begin
                `ifdef PFCU_DEBUG_LOG
                $display("Aligned PFCU GOT can be performed.");
                `endif
                // If the cmac accepts the request.
                if (mac_prt_tx_ra) begin
                    // Signal done.
                    dn <= 1;
                    // Signal jmp & set new pc.
                    jmp <= 1;
                    new_pc <= instr.bits[47:8];
                    // Signal to overwrite the stack head pointer.
                    shp_we <= 1;
                    // Signal that there is no write tx present to cmac.
                    mac_prt_tx_rp <= 0;
                end
                else begin
                    // Assign some write stuffs.
                    mac_prt_tx_req.addr <= {shp_inp[31:4], 4'b0000};    // Dont forget the 4'b0000 or else it interprets 0 as 32'b0...
                    mac_prt_tx_req.rqt <= 1;

                    // Loop over the 16 bytes and set writemask.
                    for (int i = 0; i < 16; i = i + 1) begin
                        // If the byte is in range of the bytes to write, set wmsk.
                        if (i >= shp_inp[3:0] & i < shp_oup[3:0]) begin
                            mac_prt_tx_req.wmsk[i] <= 1;
                        end
                        else mac_prt_tx_req.wmsk[i] <= 0;
                    end
                    // Write the data to the data field.
                    mac_prt_tx_req.dat[(shp_inp[3:0] * 8)+:32] <= instr.addr + instr.len;

                    // Signal request present to the mem acc cont.
                    mac_prt_tx_rp <= 1;
                end
            end
            // If an aligned write can not be performend.
            else begin
                `ifdef PFCU_DEBUG_LOG
                $display("Aligned PFCU GOT k can not be performed.");
                `endif
                case(state)
                // First line write (top).
                0:
                begin

                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Move to the next state.
                        state <= 1;
                    end
                    else begin
                        // Assign some write stuffs.
                        mac_prt_tx_req.addr <= {shp_oup[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 1;

                        // Set current PC value.
                        curr_pc = instr.addr + instr.len;

                        // Loop over bottom 4 bytes and assign write mask.
                        for (int i = 0; i < 4; i = i + 1) begin
                            if (i < shp_oup[3:0]) begin
                                mac_prt_tx_req.wmsk[i] <= 1;
                                mac_prt_tx_req.dat[(8*i)+:8] <= curr_pc[8*(4-byte_cnt)+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // Second line write (bottom).
                1:
                begin

                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Signal done.
                        dn <= 1;
                        // Signal jmp & set new pc.
                        jmp <= 1;
                        new_pc <= instr.bits[47:8];
                        // Signal to overwrite the stack head pointer.
                        shp_we <= 1;
                        // Signal that there is no write tx present to cmac.
                        mac_prt_tx_rp <= 0;
                        // Reset state to 0.
                        state <= 0;
                    end
                    else begin
                        // Assign some write stuffs.
                        mac_prt_tx_req.addr <= {shp_inp[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 1;

                        // Set current PC value.
                        curr_pc = instr.addr + instr.len;

                        // Loop over top 4 bytes and assign write mask.
                        for (int i = 0; i < 5; i = i + 1) begin
                            if (i + 11 >= shp_inp[3:0]) begin
                                mac_prt_tx_req.wmsk[11+i] <= 1;
                                mac_prt_tx_req.dat[(11 + i)*8 +:8] <= curr_pc[8*(byte_cnt)+:8];
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

        // RET
        4'b0011:
        begin
            `ifdef PFCU_DEBUG_LOG
            $display ("Exec PFCU RET IP:0x%h %t", instr.addr, $time);
            `endif
            // Prepare new stack head pointer value.
            shp_inp = shp_oup + 4;

            // Check if an aligned read can be done.
            if (shp_inp[31:4] == shp_oup[31:4]) begin

                case(state) 

                // State 0 (request to read the line).
                0:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_inp[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for reply).
                1:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // Assign the data to the new_pc
                        new_pc <= mac_prt_rx_req.dat[(shp_oup[3:0] * 8)+:32];

                        // Signal jump, shp_we & done.
                        jmp <= 1;
                        dn <= 1;
                        shp_we <= 1;

                        // signal rx accepted.
                        mac_prt_rx_ra <= 1;

                        // move state back to 0.
                        state <= 0;
                    end
                end
                endcase
            end
            // If an aligned request can not be made. 
            else begin

                case (state)

                // State 0 (read request to top line).
                0:
                begin
                    if (mac_prt_tx_ra) state <= 1;
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_inp[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for reply for top line read).
                1:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // assign bytes to new pc val.
                        for (int i = 3; i >= 0; i = i - 1) begin
                            if (i < shp_inp[3:0]) begin
                                new_pc[(3-byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i*8)+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal rx accepted.
                        mac_prt_rx_ra <= 1;

                        // Move to state 2.
                        state <= 2;
                    end
                end

                // State 2 (read request to the bottom line).
                2:
                begin
                    if (mac_prt_tx_ra) state <= 3;
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_oup[31:0], 4'b0000};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that the request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 3 (wait for reply for bottom line read).
                3:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // assign bytes to new pc val.
                        for (int i = 0; i < 5; i = i + 1) begin
                            if (i + 11 >= shp_oup[3:0]) begin
                                new_pc[byte_cnt+:8] <= mac_prt_rx_req.dat[(i+11)*8+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal rx accepted to the cmac.
                        mac_prt_rx_ra <= 1;

                        // Signal done.
                        dn <= 1;
                        // Signal jump.
                        jmp <= 1;
                        // Signal shp_we.
                        shp_we <= 1;
                        // Reset state to 0.
                        state <= 0;
                    end
                end
                endcase
            end
        end
        endcase
    end
    else begin
        // Reset state.
        state <= 0;
    end
end
endmodule
*/
`include "structs.svh"
`include "defines.svh"

//`define LSU_DEBUG_LOG

// Requires scheduler to enforce in order execution. (IN_ORDER = 1)
module load_store_unit #(parameter NUM_PHYSICAL_REGS = 64, ROB_LEN = 16, LSQ_LEN = 4)(
    input   logic                                       i_clk,                  // Clock signal.
    input   logic                                       i_rst,                  // Reset signal.

    // Interface from scheduler.
    input   logic                                       i_uop_p,                // Input uop present.
    input   micro_op_t                                  i_uop,                  // Input uop.
    output  logic                                       o_stall,                // Output stall signal.

    // Interface with PRF.
    output  logic [1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  o_rf_rd_trgt,           // RF read port target register.
    input   logic [1:0][31:0]                           i_rf_rd_dat,            // RF read port register data.

    output  logic [$clog2(NUM_PHYSICAL_REGS)-1:0]       o_rf_wr_trgt,           // RF write port target register.
    output  logic [31:0]                                o_rf_wr_dat,            // RF write port register data.
    output  logic                                       o_rf_we,                // RF write port write enable.

    // Interface with ROB.
    output  logic                                       o_uop_dn,               // ROB signal uop done.
    output  logic [$clog2(ROB_LEN)-1:0]                 o_uop_ptr               // ROB uop pointer.
);

    // Set the read target for each PRF read port.
    assign o_rf_rd_trgt[0]  =   i_uop.operand_a;
    assign o_rf_rd_trgt[1]  =   i_uop.operand_b;

    // Variables to load uop and operands into.
    micro_op_t      uop;
    logic           uop_p;
    logic [31:0]    opr_a;
    logic [31:0]    opr_b;

    // Clock uop and operands into this unit.
    always @(posedge i_clk) begin
        // Default values.
        uop     <= 0;
        uop_p   <= 0;

        // If not reset.
        if (!i_rst) begin
            // If not stalling.
            if (!o_stall) begin

                `ifdef LSU_DEBUG_LOG
                $display("Reading PRF for LSU uop %h at %t", i_uop.operation, $time);
                `endif

                uop_p   <= i_uop_p;
                uop     <= i_uop;
                opr_a   <= i_rf_rd_dat[0];
                opr_b   <= i_rf_rd_dat[1];
            end
            // If stalling
            else begin
                uop     <= uop;
                uop_p   <= uop_p;
                opr_a   <= opr_a;
                opr_b   <= opr_b;
            end
        end

    end


    // Load Store Queue
    lsq_entry_t                 lsq_dat [LSQ_LEN-1:0];
    logic [$clog2(LSQ_LEN):0]   lsq_wptr;
    logic [$clog2(LSQ_LEN):0]   lsq_rptr;
    logic 

endmodule


module load_store_queue #(parameter LEN = 16, HEADS = 1, TAILS = 1)(
    input   logic                                   clk,            // Clock.
    input   logic                                   rst,            // Reset.

    // heads.
    input   logic [HEADS-1:0]                       inp_p,          // Input push.
    input   lsq_entry_t [HEADS-1:0]                 inp_dat,        // Input data.
    output  logic [$clog2(LEN)-1:0]                 src_num_avail,  // Source number of slots available.

    // tails.
    input   logic [TAILS-1:0]                       oup_p,          // Output pop.
    output  lsq_entry_t [TAILS-1:0]                 oup_dat,        // Output data.
    output  logic [$clog2(LEN)-1:0]                 dst_num_avail,  // Destination number of slots available.

    // side ports.  
    input   logic [LEN-1:0]                         sp_md,          // Side port mark done.
    input   logic [LEN-1:0][$clog2(LEN)-1:0]        sp_ptr          // Side port pointer.
);

    // Ensure that LEN parameter is a power of 2.
    generate
    if (LEN & (LEN - 1)) $error("Length must be power of 2.");
    endgenerate


    // ROB data entries.
    lsq_entry_t [LEN-1:0]       lsq_entries;

    // Read pointer.
    logic [$clog2(LEN):0]       rptr;

    // Write pointer.
    logic [$clog2(LEN):0]       wptr;

    // Logic to handle pushing and popping from the ROB.
    logic [$clog2(LEN):0]         rptr_tmp;
    logic [$clog2(LEN):0]         wptr_tmp;
    always @(posedge clk) begin
        
        // Loop over each of the push ports.
        for (int i = 0; i < ROB_HEADS; i = i + 1) begin
            if (inp_p[i]) begin
                // If the shift register is not full.
                if (!(wptr[$clog2(LEN)-1:0] == rptr[$clog2(LEN)-1:0] && wptr[$clog2(LEN)] != rptr[$clog2(LEN)])) begin
                    // Push the data.
                    lsq_entries[wptr[$clog2(LEN)-1:0]] = inp_dat[i];

                    // Incriment the wptr.
                    wptr = wptr + 1;
                end
            end
        end

        // Update the number of entries that are unoccupied.
        src_num_avail <= (rptr[$clog2(LEN)] == wptr[$clog2(LEN)]) ? (LEN - (wptr[$clog2(LEN)-1:0] - rptr[$clog2(LEN)-1:0])) : (LEN - (wptr[$clog2(LEN)-1:0] + LEN - rptr[$clog2(LEN)-1:0]));
    

        // loop over each of the side ports and mark the entries they are pointing to as fully executed.
        for (int i = 0; i < EXEC_UNITS; i = i + 1) begin
            if (sp_md[i]) begin
                lsq_entries[sp_ptr[i]].dn = 1;
            end
        end


        // Loop over each of the pop ports.
        for (int i = 0; i < ROB_TAILS; i = i + 1) begin
            // If the position wants to pop & the sr is not empty.
            if (oup_p[i] & rptr != wptr) begin
                // Incriment the rptr.
                lsq_entries[rptr] = 0;
                rptr = rptr + 1;
            end
        end

        // Loop over each of the pop ports to present data to them.
        rptr_tmp = rptr;
        for (int i = 0; i < ROB_TAILS; i = i + 1) begin
            // if not empty.
            if (rptr_tmp != wptr) begin
                // Present the data.
                oup_dat[i] <= lsq_entries[rptr_tmp[$clog2(LEN)-1:0]];
                oup_ptr[i] <= rptr_tmp;
            end
            else begin
                oup_dat[i] <= 0;
                oup_ptr[i] <= 0;
            end
            // Incriment rptr_tmp.
            rptr_tmp = rptr_tmp + 1;
        end

        // Update the number of entries that are occupied.
        dst_num_avail <= (rptr[$clog2(LEN)] == wptr[$clog2(LEN)]) ? (wptr[$clog2(LEN)-1:0] - rptr[$clog2(LEN)-1:0]) : (wptr[$clog2(LEN)-1:0] + LEN - rptr[$clog2(LEN)-1:0]);



        // Loop over the write ports and present the ROB pointer to them.
        wptr_tmp = wptr;
        for (int i = 0; i < ROB_HEADS; i = i + 1) begin
            // Present the pointer for where the head will input to.
            inp_ptr[i] <= wptr_tmp[$clog2(LEN)-1:0];

            // Incriment temporary write pointer.
            wptr_tmp <= wptr_tmp + 1;
        end
    end

    // Handle sync reset.
    always @(posedge clk) begin
        if (rst) begin
            inp_ptr         <= 0;
            src_num_avail   <= LEN;
            oup_dat         <= 0;
            dst_num_avail   <= 0;
            rptr            <= 0;
            wptr            <= 0;
            rptr_tmp        <= 0;
            wptr_tmp        <= 0;
        end
    end
endmodule




// Look what I am doing :) (what I said I would do in commend above old LSU).
module load_store_unit_old #(parameter NUM_PHYSICAL_REGS = 64)(
    input   logic                                       clk,                // Clock.
    input   logic                                       rst,                // Reset.
    input   logic                                       en,                 // ALU enabled.
    output  bit                                         dn,                 // ALU done.

    // Instruction queue interface.
    input   micro_op                                    inst,               // Instruction to execute.

    // PRF interface.
    output  logic [1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  rf_rd_trgt,         // RF read port target register.
    input   logic [1:0][31:0]                           rf_rd_dat,          // RF read port register data.

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
    // Set the RF read and write ports targets.
    always_comb begin
        // defaults.
        rf_rd_trgt = 0;
        rf_wr_trgt = 0;
        // If enabled.
        if (en) begin
            rf_rd_trgt[0] = inst.operand_a;
            rf_rd_trgt[1] = inst.operand_b;

            rf_wr_trgt    = inst.operand_b;
        end
    end

    // Bits to make the upper and lower bounds of memory operation.
    logic [31:0] acc_lb;
    logic [31:0] acc_ub;

    logic [3:0] state;
    int         byte_cnt;

    always @(posedge clk) begin
        // Defaults.
        dn              <= 0;
        rf_wr_dat       <= 0;
        rf_we           <= 0;
        mac_prt_tx_rp   <= 0;
        mac_prt_tx_req  <= 0;
        mac_prt_rx_ra   <= 0;

        // If enabled.
        if (en) begin
            // Check which operation is to be done.
            case (inst.operation) 
            
            // LOD A, R
            4'b0001:
            begin
                `ifdef LSU_DEBUG_LOG
                $display ("Exec LSU LOD A, R \toprnds: %d, %d %t", inst.operand_a, inst.operand_b, $time);
                `endif

                // Figure out the bounds of the load operation.
                acc_lb = rf_rd_dat[0];
                acc_ub = acc_lb + 3;

                // Check the load can be done aligned.
                if (acc_lb[31:4] == acc_ub[31:4]) begin
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
                            mac_prt_tx_req.addr <= {acc_lb[31:4], 4'b0000};
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
                            if (mac_prt_rx_req.addr == {acc_lb[31:4], 4'b0000}) begin

                                // Present bytes to the gpr input.
                                rf_wr_dat <= mac_prt_rx_req.dat[(acc_lb[3:0] * 8) +:32];

                                // Signal write enable to PRF write port.
                                rf_we <= 1;

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
                            mac_prt_tx_req.addr <= {acc_ub[31:4], 4'b0000};
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
                            if (mac_prt_rx_req.addr == {acc_ub[31:4], 4'b0000}) begin

                                // Assign bits to the gpr value input.
                                for (int i = 3; i >= 0; i = i - 1) begin
                                    // If the byte is in range.
                                    if ( i <= acc_ub[3:0]) begin
                                        // Assign the byte from read reply to the gpr_inp.
                                        rf_wr_dat[(3-byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i*8)+:8];
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
                            mac_prt_tx_req.addr <= {acc_lb[31:4], 4'b0000};
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
                            if (mac_prt_rx_req.addr == {acc_lb[31:4], 4'b0000}) begin

                                // Assign bits to the gpr value input.
                                for (int i = 12; i <= 15; i = i + 1) begin
                                    // If the byte is in range.
                                    if ( i >= acc_lb[3:0]) begin
                                        // Assign the byte from read reply to the gpr_inp.
                                        rf_wr_dat[(byte_cnt*8) +:8] <= mac_prt_rx_req.dat[(i * 8) +:8];
                                        // Incriment byte_cnt.
                                        byte_cnt = byte_cnt + 1;
                                    end
                                end

                                // Signal gpr write
                                rf_we <= 1;

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

            // STR A, B
            4'b0000:
            begin
                `ifdef LSU_DEBUG_LOG
                $display ("Exec LSU STR A, B \toprnds: %d, %d %t", inst.operand_a, inst.operand_b, $time);
                `endif

                // Assign the lower bound and upper bound of the store operation.
                acc_lb = rf_rd_dat[0];
                acc_ub = acc_lb + 3;

                // Check if the store can be done aligned.
                if (acc_lb[31:4] == acc_ub[31:4]) begin

                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Signal done.
                        dn <= 1;
                    end
                    else begin
                        // Assign address & rqt.
                        mac_prt_tx_req.addr <= {acc_lb[31:4], 4'b0000};
                        mac_prt_tx_req.rqt <= 1; // Write req.

                        // Assign write mask and data.
                        for (int i = 0; i < 16; i = i + 1) begin
                            // If the byte is in range.
                            if (i >= acc_lb[3:0] & i <= acc_ub[3:0]) begin
                                mac_prt_tx_req.wmsk[i] <= 1;
                            end
                        end
                        mac_prt_tx_req.dat[(acc_lb[3:0] * 8)+:32] <= rf_rd_dat[1];

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
                            mac_prt_tx_req.addr <= {acc_lb[31:4], 4'b0000};
                            mac_prt_tx_req.rqt <= 1; // Write req.

                            // Assign write mask and data.
                            for (int i = 12; i < 16; i = i + 1) begin
                                // If the byte is in range.
                                if (i >= acc_lb[3:0]) begin
                                    // Set write mask.
                                    mac_prt_tx_req.wmsk[i] <= 1;
                                    // Set byte to write.
                                    mac_prt_tx_req.dat[(i*8)+:8] <= rf_rd_dat[1][(byte_cnt*8)+:8];
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
                            mac_prt_tx_req.addr <= {acc_ub[31:4], 4'b0000};
                            mac_prt_tx_req.rqt <= 1; // Write req.

                            // Assign write mask and data.
                            for (int i = 3; i >= 0; i = i - 1) begin
                                // If the byte is in range.
                                if (i <= acc_ub[3:0]) begin
                                    // Set write mask.
                                    mac_prt_tx_req.wmsk[i] <= 1;
                                    // Set byte to write.
                                    mac_prt_tx_req.dat[(i*8)+:8] <= rf_rd_dat[1][(3-byte_cnt)*8 +:8];
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

    // Handle async reset.
    always @(posedge clk, rst) begin
        // If reset.
        if (rst) begin
            mac_prt_tx_rp <= 0;
            mac_prt_tx_req <= 0;
            mac_prt_rx_ra <= 0;

            rf_wr_dat <= 0;
            rf_we <= 0;

            dn <= 0;
        end
    end
endmodule

// Will significantly reduce duplicate logic here once I re do instruction encodings.
/*
module load_store_unit_old(
    input   logic                       clk,                // Clock.
    input   logic                       rst,                // Reset.
    input   logic                       en,                 // LSU enable.
    output  logic                       dn,                 // LSU done.

    // Instruction queue interface.
    input   decoded_instruction                 inst,               // Instruction to execute

    // GPR interface.
    output  logic [1:0][$clog2(NUM_REGS)-1:0]   gpr_rd_trgt_reg,    // RF read port target register.
    input   logic [1:0][31:0]                   gpr_rd_reg_dat,     // RF read port register data.

    output  logic [$clog2(NUM_REGS)-1:0]        gpr_wr_trgt_reg,    // RF write port target register.
    output  logic [21:0]                        gpr_wr_reg_dat,     // RF write port register data.
    output  logic                               gpr_wr_we,          // RF write port write enable.


    input   logic [31:0]        gpr_oup [15:0],     // GPRs output data.
    output  logic [31:0]        gpr_inp [15:0],     // GPRs input data.
    output  logic [15:0]        gpr_we,             // GPRs write enable.

    // Interface to the memory subsystem.
    output  logic               mac_prt_tx_rp,      // Mem acc cont tx request present.
    output  line_acc_req        mac_prt_tx_req,     // Mem acc cont tx request.
    input   logic               mac_prt_tx_ra,      // Mem acc cont tx request accepted.

    input   logic               mac_prt_rx_rp,      // Mem acc cont rx present.
    input   line_acc_req        mac_prt_rx_req,     // Mem acc cont rx data.
    output  logic               mac_prt_rx_ra       // Mem acc cont rx accepted.
);

    logic [31:0]    load_lb;
    logic [31:0]    load_ub;
    logic [3:0]     state;
    int             byte_cnt;
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
                `ifdef LSU_DEBUG_LOG
                $display ("Exec LSU LOD k(0x%h), A(0x%h) IP:0x%h %t", instr.bits[16+:32], instr.bits[8+:4], instr.addr, $time);
                `endif       
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
                `ifdef LSU_DEBUG_LOG            
                $display ("Exec LSU LOD A(0x%h), B(0x%h) IP:0x%h %t", gpr_oup[instr.bits[8+:4]], instr.bits[12+:4], instr.addr, $time);
                `endif
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
                `ifdef LSU_DEBUG_LOG
                $display("Exec LSU STR A(0x%h), k(0x%h) IP:0x%h %t", gpr_oup[instr.bits[8+:4]], instr.bits[16+:32], instr.addr, $time);
                `endif
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
                `ifdef LSU_DEBUG_LOG
                $display("Exec LSU STR A(0x%h), B(0x%h) IP:0x%h %t", gpr_oup[instr.bits[8+:4]], gpr_oup[instr.bits[12+:4]], instr.addr, $time);
                `endif

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
*/
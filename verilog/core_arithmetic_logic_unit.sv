`include "structs.svh"
`include "defines.svh"
//`define ALU_DEBUG_LOG

module arithmetic_and_logic_unit #(parameter NUM_PHYSICAL_REGS = 64, ROB_LEN = 16)(
    input   logic                                       i_clk,                  // Clock signal.
    input   logic                                       i_rst,                  // Reset signal.

    // Interface from scheduler.
    input   logic                                       i_uop_p,                // Input uop present.
    input   micro_op_t                                  i_uop,                  // Input uop.
    output  logic                                       o_stall,                // Output stall signal.

    // Interface with PRF.
    output  logic [1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  o_rf_rd_trgt,           // RF read port target register.
    input   logic [1:0][31:0]                           i_rf_rd_dat,            // RF read port register data.

    output  logic [1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  o_rf_wr_trgt,           // RF write port target register.
    output  logic [1:0][31:0]                           o_rf_wr_dat,            // RF write port register data.
    output  logic [1:0]                                 o_rf_we,                // RF write port write enable.

    // Interface with ROB.
    output  logic                                       o_uop_dn,               // ROB signal uop done.
    output  logic [$clog2(ROB_LEN)-1:0]                 o_uop_ptr               // ROB uop pointer.
);
    // Set the read target for each PRF port.
    assign o_rf_rd_trgt[0]  =   i_uop.operand_a;
    assign o_rf_rd_trgt[1]  =   i_uop.operand_b;

    // ALU never really needs to stall so just set stall to 0 constantly.
    assign o_stall = 0;

    // uop that has had its input operands read.
    micro_op_t      uop;
    logic           uop_p;
    logic [31:0]    opr_a;
    logic [31:0]    opr_b;
    logic [31:0]    res;
    logic [31:0]    stat;

    // Clock the uop & its read operands into the ALU
    always @(posedge i_clk) begin
        // Default value.
        uop_p   <= 0;
        // If not reset.
        if (!i_rst) begin

            `ifdef ALU_DEBUG_LOG
            $display("Reading PRF for ALU uop %h at %t", i_uop.operation, $time);
            `endif

            uop_p       <= i_uop_p;
            uop         <= i_uop;
            opr_a       <= i_rf_rd_dat[0];
            opr_b       <= i_rf_rd_dat[1];
        end
    end

    // If there is a uop clocked in with its operands read, signal to ROB that it needs to retire next clock cycle.
    assign o_uop_dn = i_rst ? 0 : uop_p;
    assign o_uop_ptr = uop.rob_ptr;
    // Also setup the PRF write ports.
    assign o_rf_wr_trgt[0] = uop.operand_c;
    assign o_rf_wr_trgt[1] = uop.operand_d;
    assign o_rf_wr_dat[0]  = res;
    assign o_rf_wr_dat[1]  = stat;


    // process the uop that has its operands ready.
    bit rwe;
    always @(posedge i_clk) begin
        // default values.
        o_rf_we <= 0;

        // If there is a uop to be executed.
        if (uop_p) begin

            `ifdef ALU_DEBUG_LOG
            $display("Executing ALU uop %h at %t", uop.operation, $time);
            `endif

            // Process the instruction
            case(uop.operation)
                `UOP_ALU_ADD: begin res = opr_a + opr_b;     rwe = 1; end // ADD
                `UOP_ALU_SUB: begin res = opr_b - opr_a;     rwe = 1; end // SUB
                `UOP_ALU_AND: begin res = opr_a & opr_b;     rwe = 1; end // AND
                `UOP_ALU_IOR: begin res = opr_a | opr_b;     rwe = 1; end // IOR
                `UOP_ALU_XOR: begin res = opr_a ^ opr_b;     rwe = 1; end // XOR
                `UOP_ALU_NOT: begin res = ~opr_a;            rwe = 1; end // NOT
                `UOP_ALU_MUL: begin res = opr_a * opr_b;     rwe = 1; end // MUL
                `UOP_ALU_MOV: begin res = opr_a;             rwe = 1; end // MOV
                `UOP_ALU_RLS: begin res = {opr_a[30:0], 0};  rwe = 1; end // RLS
                `UOP_ALU_RRS: begin res = {0, opr_a[30:0]};  rwe = 1; end // RRS
                `UOP_ALU_CMP: begin res = opr_b - opr_a;     rwe = 0; end // CMP

                default: rwe = 0;
            endcase

            // Set ALU status bits.
            if (res == 0) stat[0] <= 1;
            else          stat[0] <= 0;

            // Write the result register if neccesary.
            if (rwe) o_rf_we[0] <= 1;

            // Write the ALU status register.
            o_rf_we[1] <= 1;
        end
        // If there is a reset.
        if (i_rst) begin
            o_rf_we <= 0;
        end
    end
endmodule

/*
// hobbled together temporary ALU for testing purposes.
module arithmetic_and_logic_unit_old #(parameter NUM_PHYSICAL_REGS = 64) (
    input   logic                                       clk,                // Clock.
    input   logic                                       rst,                // Reset.
    input   logic                                       en,                 // ALU enabled.
    output  bit                                         done,               // ALU done.

    // Instruction queue interface.
    input   micro_op                                    inst,               // Instruction to execute.

    // PRF interface.
    output  logic [1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  rf_rd_trgt,         // RF read port target register.
    input   logic [1:0][31:0]                           rf_rd_dat,          // RF read port register data.

    output  logic [1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  rf_wr_trgt,         // RF write port target register.
    output  logic [1:0][31:0]                           rf_wr_dat,          // RF write port register data.
    output  logic [1:0]                                 rf_we               // RF write port write enable.
);

    // Operands.
    bit [31:0] op_a;
    bit [31:0] op_b;
    bit [31:0] result;
    bit [31:0] alu_stat;
    bit rwe;


    // Handle executing ALU instructions.
    always_comb begin
        // Defaults.
        rf_rd_trgt = 0;
        rf_wr_trgt = 0;
        rf_wr_dat  = 0;
        rf_we      = 0;

        done       = 0;


        // If ALU is enabled.
        if (en) begin
            `ifdef ALU_DEBUG_LOG
            $display("Executing ALU instruction %h at %t", inst, $time);
            `endif

            // Set input operands.
            rf_rd_trgt[0] = inst.operand_a;
            rf_rd_trgt[1] = inst.operand_b;

            op_a = rf_rd_dat[0];
            op_b = rf_rd_dat[1];

            // Set output operands.
            rf_wr_trgt[0] = inst.operand_c;
            rf_wr_trgt[1] = inst.operand_d;

            rf_wr_dat[0] = result;
            rf_wr_dat[1] = alu_stat;

            //          ***Process Instructions***
            case(inst.operation)
                // Simple ones.
                4'b0000: begin result = op_a + op_b;       rwe = 1; end // ADD
                4'b0001: begin result = op_b - op_a;       rwe = 1; end // SUB
                4'b0010: begin result = op_a & op_b;       rwe = 1; end // AND
                4'b0011: begin result = op_a | op_b;       rwe = 1; end // IOR
                4'b0100: begin result = op_a ^ op_b;       rwe = 1; end // XOR
                4'b0101: begin result = ~op_a;             rwe = 1; end // NOT
                4'b0110: begin result = op_a * op_b;       rwe = 1; end // MUL
                4'b0111: begin result = op_a;              rwe = 1; end // MOV            THIS CAUSED ME 3 DAYS OF LOST PROGRESS THINKING MY HANDWRITTEN MACHINE CODE WAS WRONG.
                4'b1000: begin result = op_b - op_a;       rwe = 0; end // CMP
                4'b1001: begin result = {op_a[30:0], 0};   rwe = 1; end // RLS
                4'b1010: begin result = {0, op_a[30:0]};   rwe = 1; end // RRS
                default: rwe = 0;
            endcase

            //          ***Write result & ALU stat if needed***
            
            // Write the result if it is one that needs to be written.
            if (rwe) begin rf_we[0] = 1; end

            // update the ALU status flags.

            // If result is equal to 0.
            if (result == 0) begin alu_stat[0] = 1; end
            else             begin alu_stat[0] = 0; end

            // Write the new ALU status value.
            rf_we[1] = 1;


            // signal done.
            done = 1;
        end
    end
endmodule


module alu_status_register(
    input   logic       clk,    // Clock.
    input   logic       rst,    // Reset.
    
    input   logic [7:0] we,     // Write enable.
    input   logic [7:0] inp,    // Data input.
    output  logic [7:0] oup     // Data output.
);

    // The registers.
    bit [7:0] status;
    // Assign them to drive the output.
    assign oup = status;

    // Handle input to the registers.
    always @(posedge clk) begin
        // Loop over each reg and input if the flag is det.
        for (int i = 0; i < 16; i = i + 1) begin
            if (we[i]) begin
                status[i] <= inp[i];
            end
        end
    end

    // Handle reset.
    always @(posedge rst, clk) begin
        // if reset.
        if (rst) begin
            for(int i = 0; i < 16; i = i + 1) begin
                status[i] <= 0;
            end
        end
    end
endmodule
*/
`include "structs.svh"
//`define ALU_DEBUG_LOG

// hobbled together temporary ALU for testing purposes.
module arithmetic_and_logic_unit #(parameter NUM_PHYSICAL_REGS = 64) (
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

//`define ALU_DEBUG_LOG

// hobbled together temporary ALU for testing purposes.
module arithmetic_and_logic_unit(
    input   logic        clk,               // Clock.
    input   logic        rst,               // Reset.
    input   logic        en,                // ALU enabled.
    output  bit          done,              // ALU done.

    input   logic [47:0] inst,              // Instruction bits.
    input   logic [31:0] gpr_oup [15:0],    // GPR outputs.
    output  logic [31:0] gpr_inp [15:0],    // GPR inputs.
    output  logic [15:0] gpr_we,            // GPR write enable.

    output  bit    [7:0] status             // ALU status register value.
);

    // ALU status register.
    bit [7:0] stat_we;
    bit [7:0] stat_inp;
    alu_status_register stat_reg(clk, rst, stat_we, stat_inp, status);

    // Figure out the operands.
    bit [31:0] op_a;
    bit [31:0] op_b;
    bit [31:0] result;
    bit [3:0]  res_dst;
    bit rwe;


    // Handle executing ALU instructions.
    always_comb begin
        // Defaults.
        gpr_we = 0;
        stat_we = 0;
        done = 0;

        // If ALU is enabled.
        if (en) begin
            `ifdef ALU_DEBUG_LOG
            $display("Executing ALU instruction %h at %t", inst, $time);
            `endif
            //          ***Select Operands***
            case (inst[3:0])
            
            // Not immediate.
            4'b0100:
            begin
                op_a = gpr_oup[inst[11:8]];
                op_b = gpr_oup[inst[15:12]];
                res_dst = inst[15:12];
            end

            //  Has an immediate.
            4'b1100:
            begin
                op_a = inst[47:16];
                op_b = gpr_oup[inst[11:8]];
                res_dst = inst[11:8];
            end
            endcase

            //          ***Process Instructions***
            case(inst[7:4])
                // Simple ones.
                4'b1000: begin result = op_a + op_b;       rwe = 1; end // ADD
                4'b0010: begin result = op_b - op_a;       rwe = 1; end // SUB
                4'b1100: begin result = op_a & op_b;       rwe = 1; end // AND
                4'b0010: begin result = op_a | op_b;       rwe = 1; end // IOR
                4'b1010: begin result = op_a ^ op_b;       rwe = 1; end // XOR
                4'b0110: begin result = ~op_a;             rwe = 1; end // NOT
                4'b1110: begin result = op_a * op_b;       rwe = 1; end // MUL
                4'b0001: begin result = op_a;              rwe = 1; end // MOV            THIS CAUSED ME 3 DAYS OF LOST PROGRESS THINKING MY HANDWRITTEN MACHINE CODE WAS WRONG.
                4'b1001: begin result = op_b - op_a;       rwe = 0; end // CMP
                4'b0101: begin result = {op_a[30:0], 0};   rwe = 1; end // RLS
                4'b1101: begin result = {0, op_a[30:0]};   rwe = 1; end // RRS
                default: rwe = 0;
            endcase
            
            // if result write enable.
            if (rwe) begin gpr_inp[res_dst] = result; gpr_we[res_dst] = 1; end

            //          ***Update ALU Flags***

            // If result is equal to 0.
            if (result == 0) begin stat_we[0] = 1; stat_inp[0] = 1; end
            else             begin stat_we[0] = 1; stat_inp[0] = 0; end


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
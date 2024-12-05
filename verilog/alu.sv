
module arithmetic_and_logic_unit(
    input clk,
    input rst,
    input en,
    output bit done,

    input [47:0] inst,
    input bit [31:0] gpr_oup [15:0],
    output bit [31:0] gpr_inp [15:0],
    output bit [15:0] gpr_we,

    output bit [7:0] status
);

// ALU status register.
bit [7:0] stat_we;
bit [7:0] stat_inp;
alu_status_register stat_reg(clk, rst, stat_we, stat_inp, status);

// Figure out the operands.
bit [31:0] op_a;
bit [31:0] op_b;
bit [31:0] result;
bit rwe;

// Select operands.
integer i;
always_comb begin
    // If enabled.
    if (en) begin

        // Debug print out.
        if (inst[2:0] == 3'b100) $display("ALU active, inst 3:0 = %b", inst[3:0]);

        case (inst[3:0]) 
        
        // Case of no immediate.
        4'b0100:
        begin
            op_a <= gpr_oup[inst[11:8]];
            op_b <= gpr_oup[inst[15:12]];
            gpr_inp[inst[15:12]] <= result;

            // If the instruction wants to write back the result.
            if (rwe) begin
                // Signal to GPRs to write the result.
                for (i = 0; i < 16; i = i + 1) begin
                    if (i == inst[15:12]) gpr_we[i] <= 1;
                    else gpr_we[i] <= 0;
                end
            end
            // If the instruction doesnt want to write any data.
            else gpr_we <= 0;
        end

        // Case of an immediate being present.
        4'b1100:
        begin
            op_a <= gpr_oup[inst[11:8]];
            op_b <= inst[47:16];
            gpr_inp[inst[11:8]] <= result;

            // If the instruction wants to write back the result.
            if (rwe) begin
                // Signal to GPRs to write the result.
                for (i = 0; i < 16; i = i + 1) begin
                    if (i == inst[11:8]) gpr_we[i] <= 1;
                    else gpr_we[i] <= 0;
                end
            end
            // If the instruction doesnt want to write any data.
            else gpr_we <= 0;
        end
        
        endcase
    end
    else begin
        gpr_we <= 0;
    end
end

// Process instructions.
always_comb begin
    // If enabled.
    if (en) begin
        // If an ALU instruction is presented.
        if (inst[2:0] == 3'b100) begin

            $display("ALU instruction executing %t", $time);

            // Do what the instruction says to do.
            case(inst[7:4])
                // Simple ones.
                4'b1000: begin result <= op_a + op_b;       rwe <= 1; end // ADD
                4'b0010: begin result <= op_b - op_a;       rwe <= 1; end // SUB
                4'b1100: begin result <= op_a & op_b;       rwe <= 1; end // AND
                4'b0010: begin result <= op_a | op_b;       rwe <= 1; end // IOR
                4'b1010: begin result <= op_a ^ op_b;       rwe <= 1; end // XOR
                4'b0110: begin result <= ~op_a;             rwe <= 1; end // NOT
                4'b1110: begin result <= op_a * op_b;       rwe <= 1; end // MUL
                4'b0001: begin result <= op_a;              rwe <= 1; end // MOV
                4'b1001: begin result <= op_b - op_a;       rwe <= 0; end // CMP
                4'b0101: begin result <= {op_a[30:0], 0};   rwe <= 1; end // RLS
                4'b1101: begin result <= {0, op_a[30:0]};   rwe <= 1; end // RRS
                default: rwe <= 0;
            endcase

            // Signal that we are done.
            done <= 1;
        end
        // If an ALU instruction is not presented.
        else begin 
            rwe <= 0;
            done <= 0;
        end
    end
    else begin
        rwe <= 0;
        done <= 0;
    end
end

// Update ALU status flags.
always_comb begin
    // If enabled.
    if (en) begin
        // If an ALU instruction is in flight.
        if (inst[2:0] == 3'b100) begin

            // If result of operation is 0.
            if (result == 0) begin
                // Signal write for zero status bit.
                stat_we[0] <= 1;

                // Write a 1 to indicate the result is a 0.
                stat_inp[0] <= 1;
            end
            else begin
                // Signal write for zero status bit.
                stat_we[0] <= 1;
                stat_inp[0] <= 0;
            end
        end
        // If an ALU instruction is not in fliht.
        else begin
            stat_we <= 0;
            stat_inp <= 0;
        end
    end
    else begin
        stat_we <= 0;
        stat_inp <= 0;
    end
end
endmodule



module alu_status_register(
    input          clk,
    input          rst,
    
    input   [7:0] we,
    input   [7:0] inp,
    output  [7:0] oup
);

    // The registers.
    bit [7:0] status;
    // Assign them to drive the output.
    assign oup = status;

    // Handle input to the registers.
    integer i;
    always @(posedge clk) begin
        // Loop over each reg and input if the flag is det.
        for (i = 0; i < 16; i = i + 1) begin
            if (we[i]) begin
                status[i] <= inp[i];
            end
        end
    end

    // Handle reset.
    always @(posedge rst) begin
        for(i = 0; i < 16; i = i + 1) begin
            status[i] <= 0;
        end
    end
endmodule
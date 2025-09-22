`include "structs.svh"
`include "defines.svh"

module address_generation_unit #(parameter NUM_PHYSICAL_REGS = 64, ROB_LEN = 16)(
    input   logic                                       i_clk,                  // Clock signal.
    input   logic                                       i_rst,                  // Reset signal.

    // Interface from scheduler.
    input   logic                                       i_uop_p,                // Input uop present.
    input   micro_op_t                                  i_uop,                  // Input uop.
    output  logic                                       o_stall,                // Output stall signal.

    // Interface with PRF.
    output  logic [$clog2(NUM_PHYSICAL_REGS)-1:0]       o_rf_rd_trgt,           // RF read port target register.
    input   logic [31:0]                                i_rf_rd_dat,            // RF read port register data.

    output  logic [$clog2(NUM_PHYSICAL_REGS)-1:0]       o_rf_wr_trgt,           // RF write port target register.
    output  logic [31:0]                                o_rf_wr_dat,            // RF write port register data.
    output  logic                                       o_rf_we,                // RF write port write enable.

    // Interface with ROB.
    output  logic                                       o_uop_dn,               // ROB signal uop done.
    output  logic [$clog2(ROB_LEN)-1:0]                 o_uop_ptr               // ROB uop pointer.
);
    // Set the read target for each PRF port.
    assign o_rf_rd_trgt  =   i_uop.operand_a;

    // AGU never really needs to stall so just set stall to 0 constantly.
    assign o_stall = 0;

    // uop that has had its input operands read.
    micro_op_t      uop;
    logic           uop_p;
    logic [31:0]    opr_a;
    logic [31:0]    opr_b;
    logic [31:0]    res;

    // Clock the uop & its read operands into the AGU
    always @(posedge i_clk) begin
        // Default value.
        uop_p   <= 0;
        // If not reset.
        if (!i_rst) begin

            `ifdef AGU_DEBUG_LOG
            $display("Reading PRF for AGU uop %h at %t", i_uop.operation, $time);
            `endif

            uop_p       <= i_uop_p;
            uop         <= i_uop;
            opr_a       <= i_rf_rd_dat;
            opr_b       <= i_uop.operand_c;
        end
    end

    // If there is a uop clocked in with its operands read, signal to ROB that it needs to retire next clock cycle.
    assign o_uop_dn = i_rst ? 0 : uop_p;
    assign o_uop_ptr = uop.rob_ptr;
    // Also setup the PRF write ports.
    assign o_rf_wr_trgt = uop.operand_b;
    assign o_rf_wr_dat  = res;


    // process the uop that has its operands ready.
    always @(posedge i_clk) begin
        // default values.
        o_rf_we <= 0;

        // If there is a uop to be executed.
        if (uop_p) begin

            `ifdef AGU_DEBUG_LOG
            $display("Executing AGU uop %h at %t", uop.operation, $time);
            `endif

            // Process the instruction
            case(uop.operation)
                `UOP_AGU_ADD: begin res = opr_a + opr_b;     end // ADD
                `UOP_AGU_SUB: begin res = opr_a - opr_b;     end // SUB
            endcase

            // Write the result.
            o_rf_we <= 1;
        end
        // If there is a reset.
        if (i_rst) begin
            o_rf_we <= 0;
        end
    end
endmodule
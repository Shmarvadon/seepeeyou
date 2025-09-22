`include "structs.svh"
`include "defines.svh"

// Scheduler handles despatching uops to the execution unit in order of availability of input operands.
module exec_unit_scheduler #(parameter SCHEDULER_SLOTS = 4, NUM_PHYSICAL_REGS = 64, UOP_TYPE_RD_OPRNDS = 2, IN_ORDER = 0) (
    input   logic                                                           i_clk,                  // Clock signal.
    input   logic                                                           i_rst,                  // Reset signal.

    // Interface from despatch unit.
    input   logic                                                           i_inp_uop_p,            // Input uop present.
    input   micro_op_t                                                      i_inp_uop,              // Input uop from despatch.
    output  logic                                                           o_sched_full,           // scheduler full signal.

    // Interface with the score board.
    input   logic [NUM_PHYSICAL_REGS-1:0]                                   i_pr_valid,             // Score board valid signals.

    // Interface with the execution unit we are driving.
    output  logic                                                           o_exec_unit_uop_p,      // Execution unit uop present signal.
    input   logic                                                           i_exec_unit_stall,      // Execution unit stall signal.
    output  micro_op_t                                                      o_exec_unit_uop         // Execution unit input uop.
);
    // Next uop to send to the execution unit.
    micro_op_t                                                      nxt_actv_uop;       // Next active uop.
    logic                                                           nxt_actv_uop_p;     // Next active uop present.
    logic                                                           nxt_actv_uop_s;     // Next active uop scheduled.

    // Genblock to generate different scheduler queue behaviour.
    // Job is to find the next uop to run if present.
    generate
        // If scheduler is to force in order.
        if (IN_ORDER == 1) begin
            // Scheduler queue.
            logic       q_we;
            logic       q_re;
            micro_op_t  q_inp;
            micro_op_t  q_oup;
            logic [$clog2(SCHEDULER_SLOTS)-1:0] q_src_num_avail;
            logic [$clog2(SCHEDULER_SLOTS)-1:0] q_dst_num_avail;
            logic       q_rst;

            assign q_we             = i_inp_uop_p;
            assign q_inp            = i_inp_uop;
            assign o_sched_full     = (q_src_num_avail == 0) ? 1 : 0;
            assign q_rst            = i_rst;

            // Scheduler queue.
            fifo_sr #($bits(micro_op_t), SCHEDULER_SLOTS, 1, 1) q(i_clk, q_rst, q_we, q_inp, q_src_num_avail, q_re, q_oup, q_dst_num_avail);

            // Check if the uop is ready to run.
            always_comb begin
                // Default value.
                nxt_actv_uop_p = 0;

                // If the queue dst is not empty.
                if (q_dst_num_avail != 0) begin
                    nxt_actv_uop_p = 1;
                    // Loop over each of the input register operands and check if its set to valid in scoreboard.
                    for (int i = 0; i < UOP_TYPE_RD_OPRNDS; i = i + 1) begin
                        nxt_actv_uop_p = nxt_actv_uop_p & i_pr_valid[q_oup.operand_a[i*6+:6]];
                    end
                end
            end

            // If the uop is scheduled, pop it from the queue.
            assign q_re = nxt_actv_uop_s;
        end
        // If the scheduler does not care about order.
        if (IN_ORDER == 0) begin
            // scheduler slots.
            logic                               slots_we;
            micro_op_t                          slots_inp;
            micro_op_t [SCHEDULER_SLOTS-1:0]    slots_oup;
            logic [SCHEDULER_SLOTS-1:0]         slots_clrp;
            logic [SCHEDULER_SLOTS-1:0]         slots_up;
            logic [SCHEDULER_SLOTS-1:0]         slots_uop_rdy;
            logic                               slots_rst;
            logic [$clog2(SCHEDULER_SLOTS)-1:0] slots_uop_sel;

            assign slots_we     = i_inp_uop_p;
            assign slots_inp    = i_inp_uop;
            assign o_sched_full = (&slots_up) ? 1 : 0;
            assign slots_rst    = i_rst;

            // Scheduler slots.
            sipo_buffer #($bits(micro_op_t), SCHEDULER_SLOTS) slot(i_clk, slots_rst, slots_we, slots_inp, slots_clrp, slots_oup, slots_up);

            // Determine if each uop in the slot is ready, then select one to be the next uop to run.
            always_comb begin
                // Defaults.
                nxt_actv_uop_p = 0;

                for (int i = 0; i < SCHEDULER_SLOTS; i = i + 1) begin
                    // Set it to ready by default.
                    slots_uop_rdy[i] = 1;
                    for (int j = 0; j < UOP_TYPE_RD_OPRNDS; j = j + 1) begin
                        // And the value with the PR valid value for the operand being tested.
                        slots_uop_rdy[i] = slots_uop_rdy[i] & i_pr_valid[slots_oup[i][7 + (j*6) +:6]];
                    end
                end

                // Find one that is ready.
                for (int i = 0; i < SCHEDULER_SLOTS; i = i + 1) begin
                    if (slots_uop_rdy[i]) begin slots_uop_sel = i; nxt_actv_uop = slots_oup[i]; end
                end

                // if there is a ready uop.
                if (slots_uop_rdy != 0) nxt_actv_uop_p = 1;
            end

            // If the uop is to be sent on next posedge.
            always_comb begin
                // Default value.
                slots_clrp = 0;

                // If the uop is scheduled to go next cycle.
                if (nxt_actv_uop_s) slots_clrp[slots_uop_sel] = 1;
            end
        end
    endgenerate

    // If the execution unit is not stalling & a uop is present, signal that it will be scheduled on next posedge.
    always_comb begin
        // Default value.
        nxt_actv_uop_s = 0;

        if (!i_exec_unit_stall & nxt_actv_uop_p) begin
            nxt_actv_uop_s = 1;
        end 
    end

    // Clock the uop into the outputs.
    always @(posedge i_clk) begin
        // Default values.
        o_exec_unit_uop_p   <= 0;
        o_exec_unit_uop     <= 0;

        // If not reset.
        if (!i_rst) begin
            o_exec_unit_uop_p <= nxt_actv_uop_s;
            o_exec_unit_uop   <= nxt_actv_uop;
        end
    end
endmodule
`include "structs.svh"
`include "defines.svh"


// Despatcher will have an input buffer with a bypass, buffer will be 8 entries.
// Despatcher will despatch up to 2 uops per clock to the schedulers.
// routing logic to pass uops to correct scheduler will be combinatorial.
// Despatcher will despatch uops to each scheduler in order for that scheduler.
module uop_despatcher #(parameter BUFF_HEADS = 3, BUFF_TAILS = 2, BUFFER_LEN = 8, NUM_SCHEDULERS = 4)(
    input   logic                                                           i_clk,                  // Clock.
    input   logic                                                           i_rst,                  // Reset.
    
    // Interface from instruction front end.
    input   logic [BUFF_HEADS-1:0]                                          i_desp_inp_p,           // Despatch stage uop present.
    input   micro_op_t [BUFF_HEADS-1:0]                                     i_desp_inp,             // Despatch input.
    output  logic [$clog2(BUFFER_LEN)-1:0]                                  o_desp_src_num_avail,   // Despatch stage number of uops it can accept.

    // Interface to each of the schedulers.
    output  logic [NUM_SCHEDULERS-1:0]                                      o_sched_uop_p,          // output to scheduler is present.
    output  micro_op_t [NUM_SCHEDULERS-1:0]                                 o_sched_inp,            // output to scheduler.
    input   logic [NUM_SCHEDULERS-1:0]                                      i_sched_full            // Scheduler full.
);

    // IO for despatcher input buffer.
    logic [BUFF_HEADS-1:0]          buff_inp_p;
    micro_op_t [BUFF_HEADS-1:0]     buff_inp;
    logic [$clog2(BUFFER_LEN)-1:0]  buff_src_num_avail;

    logic [BUFF_TAILS-1:0]          buff_oup_p;
    micro_op_t [BUFF_TAILS-1:0]     buff_oup;
    logic [$clog2(BUFFER_LEN)-1:0]  buff_dst_num_avail;
    logic                           buff_rst;

    assign buff_inp =   i_desp_inp;
    assign o_desp_src_num_avail = buff_src_num_avail;
    assign buff_rst = i_rst;

    // input buffer.
    fifo_sr #($bits(micro_op_t), BUFFER_LEN, BUFF_HEADS, BUFF_TAILS) buff(i_clk, buff_rst, buff_inp_p, buff_inp, buff_src_num_avail, buff_oup_p, buff_oup, buff_dst_num_avail);


    // Variables to hold uops to despatch.
    logic [BUFF_TAILS-1:0]          uop_p;      // micro op present.
    micro_op_t [BUFF_TAILS-1:0]     uop;        // micro op to despatch.
    logic [BUFF_TAILS-1:0]          uop_dn;     // micro op done.

    // Logic for handling inputs from decode.
    int uop_inp_ind;
    always_comb begin
        // default value.
        uop_p = 0;
        uop = 0;

        buff_inp_p = i_desp_inp_p;

        uop_inp_ind = 0;

        // Assign available entries from the buffer & bypass when possible.
        for (int i = 0; i < BUFF_TAILS; i = i + 1) begin
            // If an entry is available in the buffer.
            if (i < buff_dst_num_avail) begin
                uop_p[i]        = 1;
                uop[i]          = buff_oup[i];
                buff_oup_p[i]   = uop_dn[i];
            end
            // Attempt to bypass the buffer for remaining entries.
            else begin
                // Bypass buffer & signal write to it if uop not despatched by posedge.
                uop_p[i]                = buff_inp_p[uop_inp_ind];
                uop[i]                  = buff_inp[uop_inp_ind];
                buff_inp_p[uop_inp_ind] = !uop_dn[i];

                uop_inp_ind = uop_inp_ind + 1;
            end
        end
    end


    // Logic to handle despatching uops.
    always_comb begin
        // Default values.
        o_sched_uop_p = 0;
        o_sched_inp = 0;

        // Attempt to despatch each uop.
        for (int i = 0; i < BUFF_TAILS; i = i + 1) begin
            // If the uop is present.
            if (uop_p[i]) begin
                // if the scheduler it wants to go to is not full and has not already had a uop routed to it.
                if (!i_sched_full[uop[i].exec_unit] & !o_sched_uop_p[uop[i].exec_unit]) begin
                    // Route the uop to the scheduler.
                    o_sched_uop_p[uop[i].exec_unit]     = 1;
                    o_sched_inp[uop[i].exec_unit]       = uop[i];

                    // Signal the uop has been routed.
                    uop_dn[i]                           = 1;
                end
            end
        end
    end
endmodule
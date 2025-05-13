
/*          fifo queue           */
module fifo_queue #(parameter WIDTH, LENGTH)(
    input                           clk,
    input                           rst,
    
    input [WIDTH-1:0]               d_in,
    output [WIDTH-1:0]              d_out,
    output bit [$clog2(LENGTH):0]   used_pos,

    input                           we,
    input                           se
);

    bit [WIDTH-1:0] dat [LENGTH];
    assign d_out = dat[0];


    // handle input & output.
    integer i;
    always @(posedge clk) begin
        
        case ({se, we})

            // Doing nothing.
            2'b00:
            begin
            end

            // Just writing new value.
            2'b01:
            begin
                if (used_pos < LENGTH) begin
                    // Write the new value & incriment the number of used positions.
                    dat[used_pos] <= d_in;
                    used_pos <= used_pos + 1;
                end
            end

            // Just outputting.
            2'b10:
            begin
                if (used_pos != 0) begin
                    // Shift the values.
                    for (i = 0; i < LENGTH; i = i + 1) begin
                        dat[i] <= dat[i + 1];
                    end

                    // Decriment the used_pos variable.
                    used_pos <= used_pos - 1;
                end
            end

            // Outputting & Writing.
            2'b11:
            begin
                // If we have between 1 and 7 values stored.
                if (used_pos != 0 && used_pos < LENGTH) begin
                    // Shift existing values.  
                    for (i = 0; i < used_pos-1; i = i + 1) begin
                        dat[i] <= dat[i + 1];
                    end

                    // Append new value.
                    dat[used_pos-1] <= d_in;
                end
                if (used_pos == 0) begin
                    // Store the value.
                    dat[used_pos] <= d_in;

                    // Incriment used_pos.
                    used_pos <= used_pos + 1;
                end
            end
        endcase
    end


    // Handle reset.
    always @(posedge rst) begin
        for (i = 0; i < LENGTH; i = i + 1) begin
            dat[i] = 0;
        end

        used_pos = 0;
    end
endmodule

module fifo_sr #(parameter WIDTH, DEPTH, HEADS, TAILS)(
    input logic clk,
    input logic rst,

    // Source signals.
    input logic  [HEADS-1:0]                push,
    input logic  [HEADS-1:0][WIDTH-1:0]     dinp,
    output logic [$clog2(DEPTH):0]          src_num_avail,

    // Destination signals.
    input logic  [TAILS-1:0]                pop,
    output logic [TAILS-1:0][WIDTH-1:0]     doup,
    output logic [$clog2(DEPTH):0]          dst_num_avail
);
    // Check that parameters are required values.

    // Ensure that DEPTH parameter is a power of 2.
    generate
    if (DEPTH & (DEPTH - 1)) $error("Depth must be power of 2.");
    endgenerate

    // Head side variables.
    logic [$clog2(DEPTH):0]         wptr;

    // Tail side variables.
    logic [$clog2(DEPTH):0]         rptr;

    // Global variables.
    logic [DEPTH-1:0][WIDTH-1:0]    data;

    // Logic to handle pushing & popping the shift reg.
    logic [$clog2(DEPTH):0]         rptr_tmp;
    always @(posedge clk) begin

        // Loop over each of the push ports.
        for (int i = 0; i < HEADS; i = i + 1) begin
            if (push[i]) begin
                // If the shift register is not full.
                if (!(wptr[$clog2(DEPTH)-1:0] == rptr[$clog2(DEPTH)-1:0] && wptr[$clog2(DEPTH)] != rptr[$clog2(DEPTH)])) begin
                    // Push the data.
                    data[wptr[$clog2(DEPTH)-1:0]] <= dinp[i];

                    // Incriment the wptr.
                    wptr = wptr + 1;
                end
            end
        end

        // Update the number of entries that are unoccupied.
        src_num_avail <= (rptr[$clog2(DEPTH)] == wptr[$clog2(DEPTH)]) ? (DEPTH - (wptr[$clog2(DEPTH)-1:0] - rptr[$clog2(DEPTH)-1:0])) : (DEPTH - (wptr[$clog2(DEPTH)-1:0] + DEPTH - rptr[$clog2(DEPTH)-1:0]));



        // Loop over each of the pop ports.
        for (int i = 0; i < TAILS; i = i + 1) begin
            // If the position wants to pop & the sr is not empty.
            if (pop[i] & rptr != wptr) begin
                // Incriment the rptr.
                rptr = rptr + 1;
            end
        end

        // Loop over each of the pop ports to present data to them.
        rptr_tmp = rptr;
        for (int i = 0; i < TAILS; i = i + 1) begin
            // if not empty.
            if (rptr_tmp != wptr) begin
                // Present the data.
                doup[i] <= data[rptr_tmp[$clog2(DEPTH)-1:0]];

                // Incriment rptr_tmp.
                rptr_tmp = rptr_tmp + 1;
            end
        end

        // Update the number of entries that are occupied.
        dst_num_avail <= (rptr[$clog2(DEPTH)] == wptr[$clog2(DEPTH)]) ? (wptr[$clog2(DEPTH)-1:0] - rptr[$clog2(DEPTH)-1:0]) : (wptr[$clog2(DEPTH)-1:0] + DEPTH - rptr[$clog2(DEPTH)-1:0]);
    end

    // Async reset.
    always @(posedge rst, clk) begin
        wptr = 0;
        rptr = 0;
        data = 0;
        rptr_tmp = 0;
    end

endmodule

module cdc_fifo_sr #(parameter WIDTH, DEPTH, localparam DL2 = $clog2(DEPTH))(
    input logic                 sclk,               // Source clock.
    input logic                 srst,               // Source reset.
    input logic                 dclk,               // Destination clock.
    input logic                 drst,               // Destination reset.

    // source clock signals.
    input logic [WIDTH-1:0]     dinp,               // Data input
    input logic                 push,               // Push.
    output logic [DL2:0]        src_num_avail,      // Source number available.
    output logic                src_full,           // Source full.


    // destination clock signals.
    output logic [WIDTH-1:0]    doup,               // Data output.
    input logic                 pop,                // Pop.
    output logic [DL2:0]        dst_num_avail,      // Destination number available.
    output logic                dst_empty           // Destination empty.
);

    // Ensure that DEPTH parameter is a power of 2.
    generate
    if (DEPTH & (DEPTH - 1)) $error("Depth must be power of 2.");
    endgenerate

    // Push side variables.
    logic [DL2:0]   wptr;
    logic [DL2:0]   rptr_fs;

    // Pop side variables.
    logic [DL2:0]   rptr;
    logic [DL2:0]   wptr_bs;

    // Some gray code variables for CDC.
    logic [DL2:0]   wptr_bs_gray;
    logic [DL2:0]   wptr_fs_gray;

    logic [DL2:0]   rptr_bs_gray;
    logic [DL2:0]   rptr_fs_gray;

    // Global variables.
    logic [DEPTH-1:0][WIDTH-1:0] data;


    // Logic to handle pushing stuff.
    always @(posedge sclk) begin : push_side

        // Update front side rptr from the graycode value.
        for (int i = 0; i <= DL2; i = i + 1) begin
            rptr_fs[i] = ^(rptr_fs_gray >> i);
        end

        // If the push side wants to write.
        if (push) begin
            // Check that the sr is not full.
            if (!(wptr[DL2-1:0] == rptr_fs[DL2-1:0] && wptr[DL2] != rptr_fs[DL2])) begin
                // Write the data to the register.
                data[wptr[DL2-1:0]] <= dinp;
                // Incriment wptr.
                wptr = wptr + 1;
            end
        end

        // Set the full condition.
        src_full <= (wptr[DL2-1:0] == rptr_fs[DL2-1:0] && wptr[DL2] != rptr_fs[DL2]);

        // Set the source number available value.
        src_num_avail <= (rptr_fs[DL2] == wptr[DL2]) ? (DEPTH - (wptr[DL2-1:0] - rptr_fs[DL2-1:0])) : (DEPTH - (wptr[DL2-1:0] + DEPTH - rptr_fs[DL2-1:0]));

        // Update the graycode value of wptr on the front side.
        wptr_fs_gray <= wptr ^(wptr >> 1);
        // Update the graycode value of rptr on the front side.
        rptr_fs_gray <= rptr_bs_gray;
    end

    // Logic to handle popping stuff.
    always @(posedge dclk) begin : pop_side

        // Update back side wptr from the graycode value.
        for (int i = 0; i <= DL2; i = i + 1) begin
            wptr_bs[i] = ^(wptr_bs_gray >> i);
        end

        // If the pop side wants to read.
        if (pop) begin
            // if the sr is not empty.
            if (rptr != wptr_bs) begin
                // Incriment rptr.
                rptr = rptr + 1;
            end
        end

        // Present the data stored at rptr to the output of the module.
        doup <= data[rptr[DL2-1:0]];

        // Set the empty condition.
        dst_empty <= (rptr == wptr_bs);

        // Set the destination number available value.
        dst_num_avail <= (rptr[DL2] == wptr_bs[DL2]) ? (wptr_bs[DL2-1:0] - rptr[DL2-1:0]) : (wptr_bs[DL2-1:0] + DEPTH - rptr[DL2-1:0]);

        // Update the graycode value for rptr on the back side.
        rptr_bs_gray <= rptr ^ (rptr >> 1);
        // Update the graycode value for wptr on the back side.
        wptr_bs_gray <= wptr_fs_gray;
    end

    // Async reset.
    always @(posedge drst, srst, dclk, sclk) begin
        wptr = 0;
        rptr = 0;

        wptr_bs = 0;
        rptr_fs = 0;

        wptr_bs_gray = 0;
        wptr_fs_gray = 0;

        rptr_bs_gray = 0;
        rptr_fs_gray = 0;

        data = 0;

    end
endmodule

/*          Priority decoder            */
module priority_decoder #(parameter INPUT_WIDTH = 4, OUTPUT_WIDTH = 2)(
    input logic [INPUT_WIDTH] inp,
    output logic [OUTPUT_WIDTH] oup
);

    // Decode the priority.
    integer i;
    always_comb begin
        // Last assignment wins in always block so we just do simple loop to find highest priority bit.
        for (i = 0; i <= INPUT_WIDTH; i = i + 1) begin
            if (inp[i]) oup = i;
        end
    end
endmodule


/*          single input parallel output buffer          */
module sipo_buffer #(parameter WIDTH, LENGTH)(
    input clk,
    input rst,

    input we,

    input [WIDTH-1:0] d_in,
    input [LENGTH-1:0] clr_ps,

    output logic [WIDTH-1:0] d_oup [LENGTH-1:0],
    output bit [LENGTH-1:0] used_pos
);

    // Priority decoder to figure out which slot is free.
    bit [LENGTH] pri_dec_inp;
    bit [$clog2(LENGTH)+1] pri_dec_oup;
    priority_decoder #(LENGTH, $clog2(LENGTH)+1) pri_dec(pri_dec_inp, pri_dec_oup);
    integer i;
    always_comb begin
        for (i = 0; i < LENGTH; i = i + 1) begin
            if (used_pos[i]) pri_dec_inp[i] = 0;
            else pri_dec_inp[i] = 1;
        end
    end

    // Handle input to the buffer & erasure of items.
    integer j;
    always @(posedge clk) begin
        // Loop over all the data positions and erase them if clr_ps is high.
        for (j = 0; j < LENGTH; j = j + 1) begin
            if (clr_ps[j]) used_pos[j] = 0;
        end

        // If we is high.
        if (we) begin
            // Shift data into the desired slot.
            d_oup[pri_dec_oup] = d_in;
            // mark the slot as in use.
            used_pos[pri_dec_oup] = 1;
        end
    end

    // handle reset.
    integer k;
    always @(posedge rst) begin
        // Zero the data registers.
        for (k = 0; k < LENGTH; k = k + 1) d_oup[k] = 0;
        // Zero the used positions register.
        used_pos = 0;
    end
endmodule


/*          Register            */
module register #(parameter WIDTH, RESET_VAL = 'hFFFFFFFF)(
    input clk,
    input rst,

    input [WIDTH-1:0] inp,
    output bit [WIDTH-1:0] oup,

    input we
);

    always @(posedge clk) begin
        // if write enable.
        if (we) oup <= inp;
    end

    // handle reset.
    always @(posedge rst) begin
        oup <= RESET_VAL;
    end
endmodule


/*          Mux         */
module multiplexer #(parameter INPUT_WIDTH, INPUT_COUNT)(
    // Inputs to the mux.
    input [INPUT_WIDTH-1:0]         mux_inp [INPUT_COUNT-1:0],
    input [$clog2(INPUT_COUNT)-1:0] inp_sel,

    // Outputs.
    output bit [INPUT_WIDTH-1:0]    mux_oup
);

    always_latch begin
        mux_oup = mux_inp[inp_sel];
    end

endmodule


/*          De-mux         */
module demultiplexer #(parameter INPUT_WIDTH, OUTPUT_COUNT)(
    // Inputs to the mux.
    input [INPUT_WIDTH-1:0]         demux_inp,
    input [$clog2(OUTPUT_COUNT)-1:0] oup_sel,

    // Outputs.
    output bit [INPUT_WIDTH-1:0]    demux_oup [OUTPUT_COUNT-1:0]
);

    always_comb begin
        for (int i = 0; i < OUTPUT_COUNT; i = i + 1) begin
            if (oup_sel == i) demux_oup[i] = demux_inp;
            else demux_oup[i] = 0;
        end
    end
    
endmodule

module demultiplexer_packed #(parameter OUTPUT_COUNT)(
    // Inputs to the mux.
    input                            mux_inp,
    input [$clog2(OUTPUT_COUNT)-1:0] oup_sel,

    // Outputs.
    output bit [OUTPUT_COUNT-1:0]    mux_oup
);

    always_comb begin
        mux_oup = 0;
        mux_oup[oup_sel] = mux_inp;
    end
    
endmodule

/*          Gray code counter           */
//***************** FINISH THIS LATER.
module graycode_counter #(parameter WIDTH)(
    input clk,
    input rst,
    input ce,

    output bit [WIDTH-1:0] val
);

    bit [WIDTH-1:0] prev_val;

    // Handle counting.
    integer i;
    always @(posedge clk) begin
        // If count enable is high.
        if (ce) begin
            // Loop over the bits right to left.
            for (i = 0; i < WIDTH; i = i + 1) begin
                // If the digit is the same.
                if (prev_val[i] == val[i]) begin
                    //val <= val & 
                end
            end
        end
    end

    // Handle reset.
    always @(posedge rst) begin
        val <= 0;
        prev_val <= 0;
    end
endmodule
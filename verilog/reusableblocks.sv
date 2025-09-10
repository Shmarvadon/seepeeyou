
/*          first in first out shift register          */
module fifo_sr #(parameter WIDTH, DEPTH, HEADS, TAILS)(
    input   logic                               clk,            // Clock.
    input   logic                               rst,            // Reset.

    // Source signals.
    input   logic  [HEADS-1:0]                  push,           // Push.
    input   logic  [HEADS-1:0][WIDTH-1:0]       dinp,           // Data input.
    output  logic [$clog2(DEPTH):0]             src_num_avail,  // number of spaces unoccupied.

    // Destination signals.
    input   logic  [TAILS-1:0]                  pop,            // pop.
    output  logic [TAILS-1:0][WIDTH-1:0]        doup,           // Data output.
    output  logic [$clog2(DEPTH):0]             dst_num_avail   // number of spaces occupied.
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
                    data[wptr[$clog2(DEPTH)-1:0]] = dinp[i];

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
                data[rptr] = 0;
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
            end
            else begin
                doup[i] <= 0;
            end
            // Incriment rptr_tmp.
            rptr_tmp = rptr_tmp + 1;
        end

        // Update the number of entries that are occupied.
        dst_num_avail <= (rptr[$clog2(DEPTH)] == wptr[$clog2(DEPTH)]) ? (wptr[$clog2(DEPTH)-1:0] - rptr[$clog2(DEPTH)-1:0]) : (wptr[$clog2(DEPTH)-1:0] + DEPTH - rptr[$clog2(DEPTH)-1:0]);
    end

    // Async reset.
    always @(posedge rst, clk) begin
        if (rst) begin
            wptr <= 0;
            rptr <= 0;
            data <= 0;
            rptr_tmp <= 0;
            dst_num_avail <= 0;

            for (int i = 0; i < TAILS; i = i + 1) doup[i] = 0;
        end
    end
endmodule


/*          first in first out shift register that crosses clock domain         */
module cdc_fifo_sr #(parameter WIDTH, DEPTH, localparam DL2 = $clog2(DEPTH))(
    input   logic               sclk,               // Source clock.
    input   logic               srst,               // Source reset.
    input   logic               dclk,               // Destination clock.
    input   logic               drst,               // Destination reset.

    // source clock signals.
    input   logic [WIDTH-1:0]   dinp,               // Data input
    input   logic               push,               // Push.
    output  logic [DL2:0]       src_num_avail,      // Source number available.
    output  logic               src_full,           // Source full.


    // destination clock signals.
    output  logic [WIDTH-1:0]   doup,               // Data output.
    input   logic               pop,                // Pop.
    output  logic [DL2:0]       dst_num_avail,      // Destination number available.
    output  logic               dst_empty           // Destination empty.
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
        if (srst | drst) begin
            wptr <= 0;
            rptr <= 0;

            wptr_bs <= 0;
            rptr_fs <= 0;

            wptr_bs_gray <= 0;
            wptr_fs_gray = 0;

            rptr_bs_gray <= 0;
            rptr_fs_gray <= 0;

            data <= 0;
        end
    end
endmodule


/*          single input parallel output buffer          */
module sipo_buffer #(parameter WIDTH, LENGTH)(
    input   logic                             clk,        // Clock.
    input   logic                             rst,        // Reset.
    
    input   logic                             we,         // Write enable.
    input    logic [WIDTH-1:0]                d_inp,      // Data input.
    input   logic [LENGTH-1:0]                clr_ps,     // Clear position(s).

    output logic [LENGTH-1:0][WIDTH-1:0]      d_oup,      // Data output.
    output logic [LENGTH-1:0]                 used_pos    // Used Positions.
);

    logic dn;
    always @(posedge clk) begin
        // default value.
        dn = 0;

        // If a write is signalling.
        if (we) begin
            for (int i = 0; i < LENGTH; i = i + 1) begin
                if (!used_pos[i] & !dn) begin d_oup[i] <= d_inp; used_pos[i] = 1; dn = 1; end
            end
        end

        // if clr_ps is signalled.
        for (int i = 0; i < LENGTH; i = i + 1) begin
            if (clr_ps[i]) begin d_oup[i] <= 0; used_pos[i] <= 0; end
        end
    end

    // Handle reset.
    always @(posedge rst, clk) begin
        if (rst) begin
            used_pos <= 0;
            d_oup <= 0;
        end
    end
endmodule


/*          Register            */
module register #(parameter WIDTH, RESET_VAL = 'hFFFFFFFF)(
    input   logic               clk,    // Clock.
    input   logic               rst,    // Reset.

    input   logic [WIDTH-1:0]   inp,    // Data input.
    output  bit [WIDTH-1:0]     oup,    // Data output.

    input   logic               we      // Write enable.
);

    always @(posedge clk) begin
        // if write enable.
        if (we) oup <= inp;
    end

    // handle reset.
    always @(posedge rst, clk) begin
        if (rst) oup <= RESET_VAL;
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
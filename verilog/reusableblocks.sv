
/*          fifo queue           */
module fifo_queue #(parameter WIDTH, LENGTH)(
    input                           clk,
    input                           rst,
    
    input [WIDTH-1:0]               d_in,
    output [WIDTH-1:0]              d_out,
    output bit [$clog2(LENGTH)+1]   used_pos,

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
            if (used_pos[i]) pri_dec_inp[i] <= 0;
            else pri_dec_inp[i] <= 1;
        end
    end

    // Handle input to the buffer & erasure of items.
    integer j;
    always @(posedge clk) begin
        // Loop over all the data positions and erase them if clr_ps is high.
        for (j = 0; j < LENGTH; j = j + 1) begin
            if (clr_ps[j]) used_pos[j] <= 0;
        end

        // If we is high.
        if (we) begin
            // Shift data into the desired slot.
            d_oup[pri_dec_oup] <= d_in;
            // mark the slot as in use.
            used_pos[pri_dec_oup] <= 1;
        end
    end

    // handle reset.
    integer k;
    always @(posedge rst) begin
        // Zero the data registers.
        for (k = 0; k < LENGTH; k = k + 1) d_oup[k] <= 0;
        // Zero the used positions register.
        used_pos <= 0;
    end
endmodule


/*          Register                */
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
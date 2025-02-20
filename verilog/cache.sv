// https://verificationacademy.com/forums/t/blocking-or-non-blocking-statement-vs/42389    dave_59
// Big up dave_59, that man has all the wisdom in the world.


/*

    Distinctions between L1 & L2 caches:
    - on read L1 just reads, L2 will read and evict the line in the same go.
    - On write L1 will write and evict if needed (same as l2).
    - L1 will check the entire set in 1 clock cycle, L2 will take multiple cycles depending on set size.
*/

// Really dumb SRAM that enables us to access a line at a time.
module cache_sram_block #(parameter LINES = 256, LINE_LENGTH = 256)(
    input clk,
    input rst,

    input [$clog2(LINES)-1:0]   line_sel,
    input [LINE_LENGTH-1:0]     line_inp,
    input                       line_we,
    output [LINE_LENGTH-1:0]    line_oup
);
    // block of lines, lhs is packed & rhs is not packed.
    bit [LINE_LENGTH-1:0] sram [LINES-1:0];


    // Continuous assignment for output.
    assign line_oup = sram[line_sel];

    // always block for writing to the line.
    always @(posedge clk) begin
        if (line_we) begin
            sram[line_sel] <= line_inp;

            // Debug printout.
            $display("A write is happening on line %d writing: %h", line_sel, line_inp);
        end
    end
endmodule

// This cache is designed to be fast and respond quickly, also somewhat area efficient I hope (pls dont go above 4 ways).
module l1_cache #(parameter WAYS = 2, LINES = 128, LINE_LENGTH = 16, ADDR_W = 32)(
    input clk,
    input rst,

    // Front side of the cache.
    input [ADDR_W-1:0]                      fs_addr,    // Front side address select.
    input                                   fs_wr,      // Front side write request.
    input                                   fs_go,      // Front side go.
    output bit                              fs_done,    // Front side done.
    output bit                              fs_suc,     // Front side successful.

    input [(LINE_LENGTH * 8)-1:0]           fs_inp,     // Front side data input.
    output bit [(LINE_LENGTH * 8)-1:0]      fs_oup,     // Front side data output.

    // Back side of the cache.

    output bit [ADDR_W-1:0]                 bs_addr,    // Back side address select.
    output bit [(LINE_LENGTH * 8)-1:0]      bs_oup,     // Back side data output.
    output bit                              bs_we,      // Back side write enable.
    input                                   bs_done     // Back side done.
);

    // define a structure to represent a cache line.
    typedef struct packed {
        bit [(LINE_LENGTH * 8)-1:0]                                       dat;      // Data, 128 bits / 16 bytes.
        bit [(ADDR_W - ($clog2(LINES) + $clog2(LINE_LENGTH)))-1:0]        tag;      // tage 20 its, 31:12
        bit                                                               dirty;    // dirty bit, indicates if this cache line is pfs_resent in main memory yet or not.
        bit                                                               valid;    // valid bit indicates if there is any data loaded.
    } cache_line;

    // generate some cache SRAM blocks. in the configuration and size defined by parameters.
    cache_line                    set_out [WAYS-1:0];   // Output from each of the lines in the set currently selected.
    cache_line                    set_inp [WAYS-1:0];   // Input to each of the lines in the set currently selected.
    logic [WAYS-1:0]              line_we;              // Controls if a line is to be written to or not.
    logic [$clog2(LINES)-1:0]             set_sel;              // Controls which set is selected.

    generate
        for (genvar i = 0; i < WAYS; i=i+1) begin
            cache_sram_block #(LINES, $bits(cache_line)) way(clk, rst, set_sel, set_inp[i], line_we[i], set_out[i]);
        end
    endgenerate

    // continuous assignment to set_sel based on conditions.
    assign set_sel = (fs_go && line_we == 0) ? fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)] : set_sel;
    
    // Comb block to determine if the set is full or not.
    logic set_full;
    always_comb begin
        set_full = 1;
        // loop over all lines in the set.
        for (int i = 0; i < WAYS; i = i + 1) begin
            // If the line is not valid (isnt occupied) then the set isnt full.
            if (set_out[i].valid == 0) set_full = 0;
        end
    end



    // Handle read and write requests to this cache.
    bit [3:0] write_state;
    bit [$clog2(WAYS)-1:0] line_evic_sel;
    bit [7:0] ctr;
    always @(posedge clk) begin
        // Default conditions.
        fs_done = 0;
        fs_suc = 0;
        line_we = 0;
        bs_we = 0;
        fs_oup = 0;

        // If there is a request to the cache.
        if (fs_go) begin
            // If there is no write request (a read is being asked for).
            if (!fs_wr) begin

                // Reset the state machine for write requests.
                write_state = 0;

                // Loop over all the lines in the set to locate tag & check if that line is valid or not.
                for (int i = 0; i < WAYS; i = i + 1) begin
                    // If tags match and valid bit is set.
                    if (set_out[i].tag == fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && set_out[i].valid) begin
                        // assign data to output.
                        fs_oup = set_out[i].dat;

                        // Signal done & success.
                        fs_done = 1;
                        fs_suc = 1;

                        // Debug printout.
                        $display("Found cache line mapping to tag %h in way %d set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                    end
                end

                // If not able to find the requested line.
                if (!fs_done) begin
                    // Signal done but not success.
                    fs_done = 1;
                    fs_suc = 0;

                    // Debug printout.
                    $display("L1 cahche");
                    $display("Unable to find cache line with tag %h in set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                end

            end
            // If there is a write request.
            else begin

                // A series of attempts to write the line to cache.
                case(write_state)

                    // Attempt to find a line that already contains data belonging to tag to overwrite.
                    0:
                    begin

                        $display("Attempting to modify if present already. %t", $time);
                        // Loop over all the lines in the set.
                        for (int i = 0; i < WAYS; i = i + 1) begin
                            // If the tag bits and valid match.
                            if (set_out[i].tag == fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && set_out[i].valid) begin
                                // present the data to the line input.
                                set_inp[i].dat = fs_inp;
                                set_inp[i].valid = 1;
                                set_inp[i].dirty = 1;
                                set_inp[i].tag = set_out[i].tag;

                                // Set write enable for the line high.
                                line_we[i] = 1;

                                // Set done and success.
                                fs_done = 1;
                                fs_suc = 1;

                                // Debug printout.
                                $display("Found a line to modify write to with tag %h at way %d set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                            end
                        end

                        // If not able to find line to modify, jump to next possible write scenario.
                        if (!fs_done) begin
                            // Jump to different write states depending on if there is an empty line in the set or not. This saves us a clock cycle in case of full set.
                            case(set_full) 
                            0: write_state = 1; // Set is not full.
                            1: write_state = 2; // Set is full.
                            endcase
                        end
                        else write_state = 0;
                    end

                    // Attempt to find a new empty line to write the data to.
                    1:
                    begin
                        // Loop over all the lines in the set.
                        for (int i = 0; i < WAYS; i = i + 1) begin
                            // If we are not done yet (we only want to write to one line).
                            if (!fs_done) begin
                                // If the line has valid bit set to 0.
                                if (set_out[i].valid == 0) begin
                                    // Setup input for line.
                                    set_inp[i].dat = fs_inp;
                                    set_inp[i].valid = 1;
                                    set_inp[i].dirty = 1;
                                    set_inp[i].tag = fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];

                                    // Set write enable for the line high.
                                    line_we[i] = 1;

                                    // Set done and success.
                                    fs_done = 1;
                                    fs_suc = 1;

                                    // Debug printout.
                                    $display("Found an empty line to write data to, tag %h at way %d set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                                end
                            end
                        end

                        // If not able to find empty line to write to.
                        if (!fs_done) begin
                            // Set fs_done but set success to 0.
                            fs_done = 1;
                            fs_suc = 0;

                            // Debug printout.
                            $display("Unable to find empty cache line to write data to, tag %h set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                        end
                        else write_state = 0;   // Else reset write state to 0.
                    end

                    // Attempt to evict a line from cache to write the new data in.
                    2:
                    begin
                        
                        // Present the line for eviction to the backside.
                        bs_addr = {set_out[line_evic_sel].tag, set_sel, 0};   // This *might* need to be corrected.
                        bs_oup = set_out[line_evic_sel].dat;
                        bs_we = 1;

                        // If the backside is signalling that it is done.
                        if (bs_done) begin
                            // Setup input to overwrite the evicted line.
                            set_inp[line_evic_sel].dat = fs_inp;
                            set_inp[line_evic_sel].valid = 1;
                            set_inp[line_evic_sel].dirty = 1;
                            set_inp[line_evic_sel].tag = fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];

                            // Signal write enable.
                            line_we[line_evic_sel] = 1;

                            // Signal done & success.
                            fs_done = 1;
                            fs_suc = 1;

                            // Reset write state to 0.
                            write_state = 0;

                            // Set timeout counter to 0.
                            ctr = 0;

                            // Incriment or reset line_evic_sel.
                            if (line_evic_sel < WAYS) line_evic_sel = line_evic_sel + 1;
                            else line_evic_sel = 0;

                            // Debug printout.
                            $display("Evicted a line and written data to it at, tag %h at way %d set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], line_evic_sel, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                        end
                        // A counter that if it overflows then we prolly have done something bad and need to let whatever is signalling the cache know so it can bypass L1 or something.
                        else begin
                            // Incriment a counter.
                            if (ctr < 255) ctr = ctr + 1;
                            else begin
                                // Set counter to 0.
                                ctr = 0;

                                // Signal done but not success.
                                fs_done = 1;
                                fs_suc = 0;

                                // Reset write state to 0.
                                write_state = 0;

                                // Debug printout.
                                $display("Write failed, unable to evict line with tag %h at way %d set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], line_evic_sel, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                            end
                        end
                    end
                endcase
            end
        end

        // If fs_go is low (0).
        else begin
            // Reset some state machine stuff.
            write_state = 0;
        end
    end
endmodule


// This cache is designed to be somewhat fast but much higher capcity and very area efficient (targeting 8-16 ways).
module l2_cache #(parameter WAYS = 8, LINES = 512, LINE_LENGTH = 16, ADDR_W = 32, LINES_PER_CLK = 4)(
    input clk,
    input rst,

    // Front side of the cache.
    input [ADDR_W-1:0]                      fs_addr,    // Front side address select.
    input                                   fs_wr,      // Front side write request.
    input                                   fs_go,      // Front side go.
    output bit                              fs_done,    // Front side done.
    output bit                              fs_suc,     // Front side successful.

    input [(LINE_LENGTH * 8)-1:0]           fs_inp,     // Front side data input.
    output bit [(LINE_LENGTH * 8)-1:0]      fs_oup,     // Front side data output.

    // Back side of the cache.

    output bit [ADDR_W-1:0]                 bs_addr,    // Back side address select.
    output bit [(LINE_LENGTH * 8)-1:0]      bs_oup,     // Back side data output.
    output bit                              bs_we,      // Back side write enable.
    input                                   bs_done     // Back side done.
);

    // to do:
    //  - Add local params to make stuff more readable.
    //  - Change some stuff around the writeback / eviction to make it properly synthesiseable.
    //  - General pass on this to ensure it all makes sense in my head :)

    // define a structure to represent a cache line.
    typedef struct packed {
        bit [(LINE_LENGTH * 8)-1:0]                                       dat;      // Data, 128 bits / 16 bytes.
        bit [(ADDR_W - ($clog2(LINES) + $clog2(LINE_LENGTH)))-1:0]        tag;      // tag 20 bits, 31:12
        bit                                                               dirty;    // dirty bit, indicates if this cache line is pfs_resent in main memory yet or not.
        bit                                                               valid;    // valid bit indicates if there is any data loaded.
    } cache_line;

    // generate some cache SRAM blocks. in the configuration and size defined by parameters.
    cache_line                    set_out [WAYS-1:0];   // Output from each of the lines in the set currently selected.
    cache_line                    set_inp [WAYS-1:0];   // Input to each of the lines in the set currently selected.
    logic [WAYS-1:0]              line_we;              // Controls if a line is to be written to or not.
    logic [$clog2(LINES)-1:0]     set_sel;              // Controls which set is selected.

    generate
        for (genvar i = 0; i < WAYS; i=i+1) begin
            cache_sram_block #(LINES, $bits(cache_line)) way(clk, rst, set_sel, set_inp[i], line_we[i], set_out[i]);
        end
    endgenerate

    // continuous assignment to set_sel based on conditions.
    assign set_sel = (fs_go && line_we == 0) ? fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)] : set_sel;
    
    // Comb block to determine if the set is full or not.
    logic set_full;
    always_comb begin
        set_full = 1;
        // loop over all lines in the set.
        for (int i = 0; i < WAYS; i = i + 1) begin
            // If the line is not valid (isnt occupied) then the set isnt full.
            if (set_out[i].valid == 0) set_full = 0;
        end
    end

    // Impliment a bunch of muxes to allow slicing of the set.
    cache_line set_out_slice [LINES_PER_CLK-1:0];
    cache_line set_inp_slice [LINES_PER_CLK-1:0];
    bit [LINES_PER_CLK-1:0] line_slice_we;
    bit [$clog2(WAYS/LINES_PER_CLK)-1:0] mux_sel;

    // Generate the muxes.
    generate
        for (genvar i = 0; i < LINES_PER_CLK; i = i + 1) begin
            multiplexer #($bits(cache_line), WAYS/LINES_PER_CLK) mux(set_out[i*(WAYS/LINES_PER_CLK)+:(WAYS/LINES_PER_CLK)], mux_sel, set_out_slice[i]);
            demultiplexer #($bits(cache_line), WAYS/LINES_PER_CLK) demux(set_inp_slice[i], mux_sel, set_inp[i*(WAYS/LINES_PER_CLK)+:(WAYS/LINES_PER_CLK)]);
            demultiplexer_packed #(WAYS/LINES_PER_CLK) demux_we(line_slice_we[i], mux_sel, line_we[i*(WAYS/LINES_PER_CLK)+:(WAYS/LINES_PER_CLK)]);
        end
    endgenerate


    // Handle read and write requests to this cache.
    bit [3:0] write_state;
    bit [$clog2(WAYS/LINES_PER_CLK)-1:0] line_evic_mux_sel;
    bit [LINES_PER_CLK-1:0] line_evic_line_sel;
    bit [7:0] ctr;
    always @(posedge clk) begin
        // Default conditions.
        fs_done = 0;
        fs_suc = 0;
        bs_we = 0;
        line_slice_we = 0;

        // If there is a request to the cache.
        if (fs_go) begin
            // If there is no write request (a read is being asked for).
            if (!fs_wr) begin

                // Reset the state machine for write requests.
                write_state = 0;

                // Loop over all the lines the mux is currently selecting.
                for (int i = 0; i < LINES_PER_CLK; i = i + 1) begin
                    // If the tag matches and the valid bit is set.
                    if (set_out_slice[i].tag == fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && set_out_slice[i].valid) begin

                        // Assign data to output.
                        fs_oup = set_out_slice[i].dat;

                        // Signal done & success.
                        fs_done = 1;
                        fs_suc = 1;

                        // Reset the line.
                        set_inp_slice[i].valid = 0;
                        line_slice_we[i] = 1;

                        // Reset mux select to to.
                        mux_sel = 0;

                        // Debug printout.
                        $display("Found cache line mapping to tag %h in way %d set %d that is being read and marked invalid.", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                    end
                end

                // If not found the line.
                if (!fs_done) begin

                    // If the mux can be incrimented then do it.
                    if (mux_sel < (WAYS/LINES_PER_CLK)-1) begin
                        mux_sel = mux_sel + 1;
                    end
                    // Else if we have reached the end of the mux we must accept defeat.
                    else begin
                        // Signal done.
                        fs_done = 1;
                        // Signal unsucsessful.
                        fs_suc = 0;
                        // Reset mux_sel to 0.
                        mux_sel = 0;

                        // Debug printout.
                        $display("L2 cahche");
                        $display("Unable to find cache line with tag %h in set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                    end
                end
            end
            // If there is a write request.
            else begin

                // A series of attempts to write the line to cache.
                case(write_state)

                    // Attempt to find a line that already contains data belonging to tag to overwrite.
                    0:
                    begin

                        $display("Attempting to modify if present already.");

                        // Loop over the lines currently selected by the muxes.
                        for (int i = 0; i < LINES_PER_CLK; i = i + 1) begin
                            // If the tag matches and the valid bit is set.
                            if (set_out_slice[i].tag == fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && set_out_slice[i].valid) begin
                                // Modify the cache line.
                                set_inp_slice[i].valid = 1;
                                set_inp_slice[i].dat = fs_inp;
                                set_inp_slice[i].dirty = 1;
                                set_inp_slice[i].tag = set_out_slice[i].tag;

                                // Write enable high for this line.
                                line_slice_we[i] = 1;

                                // Signal done.
                                fs_done = 1;
                                fs_suc = 1;

                                // Debug printout.
                                $display("Found a line to modify write to with tag %h at way %d set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i * mux_sel, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                            end
                        end

                        // If not found the line.
                        if (!fs_done) begin
                            // If we have room, incriment mux_sel.
                            if (mux_sel < (WAYS/LINES_PER_CLK)-1) begin
                                mux_sel = mux_sel + 1;
                                $display("%d", mux_sel);
                            end
                            // If we dont have room to loop over next lot (we reached the end).
                            else begin
                                // Reset mux sel to 0.
                                mux_sel = 0;

                                // Check if the set is empty or not.
                                // Jump to different write case depending on if we have to evict a line or not.
                                case(set_full)
                                0: write_state = 1; // Set is not full.
                                1: begin write_state = 2; mux_sel = line_evic_mux_sel; end // Set is full. We set the mux up here to give it enough time to stabalise.
                                endcase
                            end
                        end
                        // If done.
                        else begin
                            // Reset write state to 0.
                            write_state = 0;
                            // Reset mux_sel to 0.
                            mux_sel = 0;
                        end
                    end

                    // Attempt to find a new empty line to write the data to.
                    1:
                    begin
                        // Loop over all the lines currently selected by the mux.
                        for (int i = 0; i < LINES_PER_CLK; i = i + 1) begin
                            // If havent found a line already.
                            if (!fs_done) begin
                                // If the lines valid tag is set to 0.
                                if (set_out_slice[i].valid == 0) begin
                                    // Setup input to the line.
                                    set_inp_slice[i].dat = fs_inp;
                                    set_inp_slice[i].valid = 1;
                                    set_inp_slice[i].dirty = 1;
                                    set_inp_slice[i].tag = fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];

                                    // Set write enable for the line high.
                                    line_slice_we[i] = 1;

                                    // Signal done and success.
                                    fs_done = 1;
                                    fs_suc = 1;

                                    // Debug printout.
                                    $display("Found an empty line to write data to, tag %h at way %d set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i * mux_sel, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                                end
                            end
                        end

                        // If not done.
                        if (!fs_done) begin
                            // If we have room, incriment mux_sel.
                            if (mux_sel < (WAYS/LINES_PER_CLK)-1) begin
                                mux_sel = mux_sel + 1;
                            end
                            // If we dont have room to incriment the mux (we have reached the end of the set).
                            else begin
                                // Set done but not success.
                                fs_done = 1;
                                fs_suc = 0;

                                // Reset mux_sel.
                                mux_sel = 0;

                                // Debug printout.
                                $display("Unable to find empty cache line to write data to, tag %h set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                            end
                        end
                        // If done.
                        else begin
                            // Reset write state & mux_sel.
                            write_state = 0;
                            mux_sel = 0;
                        end
                    end

                    // Attempt to evict a line from cache to write the new data in.
                    2:
                    begin
                        
                        // Present the line for eviction to the backside.
                        bs_addr = {set_out_slice[line_evic_line_sel].tag, set_sel, 0};   // This *might* need to be corrected.
                        bs_oup = set_out_slice[line_evic_line_sel].dat;
                        bs_we = 1;

                        // If the backside is signalling that it is done.
                        if (bs_done) begin
                            // Setup input to overwrite the evicted line.
                            set_inp_slice[line_evic_line_sel].dat = fs_inp;
                            set_inp_slice[line_evic_line_sel].valid = 1;
                            set_inp_slice[line_evic_line_sel].dirty = 1;
                            set_inp_slice[line_evic_line_sel].tag = fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];

                            // Signal write enable.
                            line_slice_we[line_evic_line_sel] = 1;

                            // Signal done & success.
                            fs_done = 1;
                            fs_suc = 1;

                            // Reset write state to 0.
                            write_state = 0;

                            // Set mux_sel to 0 cause why not?
                            mux_sel = 0;

                            // Set timeout counter to 0.
                            ctr = 0;

                            // Incriment or reset line_evic_sel.
                            if (line_evic_line_sel < LINES_PER_CLK-1) line_evic_line_sel = line_evic_line_sel + 1;
                            else begin 
                                line_evic_line_sel = 0; 
                                if (line_evic_mux_sel < (WAYS/LINES_PER_CLK)-1) line_evic_mux_sel = line_evic_mux_sel + 1; 
                                else line_evic_mux_sel = 0;
                            end

                            // Debug printout.
                            $display("Evicted a line and written data to it at, tag %h at way %d (INACCURATE) set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], line_evic_line_sel, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                        end
                        // A counter that if it overflows then we prolly have done something bad and need to let whatever is signalling the cache know so it can bypass L1 or something.
                        else begin
                            // Incriment a counter.
                            if (ctr < 255) ctr = ctr + 1;
                            else begin
                                // Set counter to 0.
                                ctr = 0;

                                // Signal done but not success.
                                fs_done = 1;
                                fs_suc = 0;

                                // Reset write state to 0.
                                write_state = 0;

                                // Reset mux_sel to 0 cause why not?
                                mux_sel = 0;

                                // Debug printout.
                                $display("Write failed, unable to evict line with tag %h at way %d (INACCURATE) set %d", fs_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], line_evic_line_sel, fs_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                            end
                        end
                    end
                endcase
            end
        end

        // If fs_go is low (0).
        else begin
            // Reset some state machine stuff.
            write_state = 0;
            mux_sel = 0;
        end
    end

endmodule
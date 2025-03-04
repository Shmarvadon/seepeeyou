// https://verificationacademy.com/forums/t/blocking-or-non-blocking-statement-vs/42389    dave_59
// Big up dave_59, that man has all the wisdom in the world.


/*

    Distinctions between L1 & L2 caches:
    - on read L1 just reads, L2 will read and evict the line in the same go.
    - On write L1 will write and evict if needed (same as l2).
    - L1 will check the entire set in 1 clock cycle, L2 will take multiple cycles depending on set size.
    - L2 uses unified line RAM & tag ram but L2 will have seperate so that it can miss quicker if a miss occures.
*/

// Really dumb SRAM that enables us to access 2 lines at a time.
module dual_port_sram #(parameter LINES = 256, LINE_LENGTH = 256)(
    input clk,
    input rst,

    // Port 1.
    input logic [$clog2(LINES)-1:0]     prt_1_addr,
    input logic                         prt_1_we,
    input logic [LINE_LENGTH-1:0]       prt_1_dinp,
    output logic [LINE_LENGTH-1:0]      prt_1_doup,

    // Port 2.
    input logic [$clog2(LINES)-1:0]     prt_2_addr,
    input logic                         prt_2_we,
    input logic [LINE_LENGTH-1:0]       prt_2_dinp,
    output logic [LINE_LENGTH-1:0]      prt_2_doup
);
    // block of lines, lhs is packed & rhs is not packed.
    bit [LINE_LENGTH-1:0] sram [LINES-1:0];


    // Continuous assignment for read.
    assign prt_1_doup = sram[prt_1_addr];
    assign prt_2_doup = sram[prt_2_addr];

    // always block to handle write port.
    always @(posedge clk) begin
        // If port 1 is writing.
        if (prt_1_we) begin
            sram[prt_1_addr] = prt_1_dinp;

            // Debug printout.
            $display("A write is happening on line %d writing: %h", prt_1_addr, prt_1_dinp);
        end
        // If port 2 is writing.
        if (prt_2_we) begin
            sram[prt_2_addr] = prt_2_dinp;

            // Debug printout.
            $display("A write is happening on line %d writing: %h", prt_2_addr, prt_2_dinp);
        end
    end
endmodule

// This cache is designed to be fast and respond quickly, also somewhat area efficient I hope (pls dont go above 4 ways).
module l1_cache #(parameter WAYS = 2, LINES = 128, LINE_LENGTH = 16, ADDR_W = 32)(
    input clk,
    input rst,

    // Front side of the cache.

    // Read port.
    input logic [ADDR_W-1:0]                fs_rdp_addr,    // Front side read port addr.
    input logic                             fs_rdp_en,      // Front side read port enable.
    output logic [(LINE_LENGTH * 8) - 1:0]  fs_rdp_dat,     // Front side read port data.
    output logic                            fs_rdp_done,    // Front side read port done.
    output logic                            fs_rdp_suc,     // Front side read port successful.

    // Write port.
    input logic [ADDR_W-1:0]                fs_wrp_addr,    // Front side write port addr.
    input logic                             fs_wrp_en,      // Front side write port enable.
    input logic [(LINE_LENGTH * 8) - 1:0]   fs_wrp_dat,     // Front side write port data.
    output logic                            fs_wrp_done,    // Front side write port done.
    output logic                            fs_wrp_suc,     // Front side write port successful.


    // Back side of the cache.

    output logic [ADDR_W-1:0]               bs_addr,        // Back side address select.
    output logic [(LINE_LENGTH * 8)-1:0]    bs_dat,         // Back side data output.
    output logic                            bs_we,          // Back side write enable.
    input logic                             bs_dn           // Back side done.
);

    // define a structure to represent a cache line.
    typedef struct packed { // MSB
        bit [(LINE_LENGTH * 8)-1:0]                                       dat;      // Data, 128 bits / 16 bytes.
        bit [(ADDR_W - ($clog2(LINES) + $clog2(LINE_LENGTH)))-1:0]        tag;      // tage 20 its, 31:12
        bit                                                               dirty;    // dirty bit, indicates if this cache line is pfs_resent in main memory yet or not.
        bit                                                               valid;    // valid bit indicates if there is any data loaded.
    } cache_line;           // LSB


    //          Generate some SRAM blocks.  

    logic [$clog2(LINES)-1:0]   rdp_set_sel;            // Read port set select.
    cache_line                  rdp_set  [WAYS-1:0];    // Read port set data.
    
    logic [$clog2(LINES)-1:0]   wrp_set_sel;            // Write port set select.
    cache_line                  wrp_set_rd  [WAYS-1:0]; // Write port set data read.
    cache_line                  wrp_set_wr  [WAYS-1:0]; // Write port set data write.
    logic [WAYS-1:0]            wrp_line_we;            // Write port line we.

    generate
        for (genvar i = 0; i < WAYS; i = i + 1) begin
            dual_port_sram #(LINES, $bits(cache_line)) way(.clk(clk), .rst(rst), .prt_1_addr(rdp_set_sel), .prt_1_doup(rdp_set[i]), 
            .prt_2_addr(wrp_set_sel), .prt_2_dinp(wrp_set_wr[i]), .prt_2_doup(wrp_set_rd[i]), .prt_2_we(wrp_line_we[i]));
        end
    endgenerate

    // Assign the index bits to drive set sel for read port.
    assign rdp_set_sel = fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)];
    // Assign the index bits to drive set sel for write port.
    assign wrp_set_sel = fs_wrp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)];

    // Comb block to handle read requests.
    always_comb begin
        // Defaults.
        fs_rdp_done = 0;
        fs_rdp_suc = 0;
        fs_rdp_dat = 0;

        // If readport enable is high.
        if (fs_rdp_en) begin

            // Debug printout.
            $display("Attempting to read a line from cache.");

            // Loop over all the lines currently selected by rdp_set_sel.
            for (integer i = 0; i < WAYS; i = i + 1) begin
                // If the line has matching tag & is valid.
                if (rdp_set[i].tag == fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && rdp_set[i].valid) begin
                    // Present data to output and signal done & success.
                    fs_rdp_dat = rdp_set[i].dat;

                    fs_rdp_done = 1;
                    fs_rdp_suc = 1;

                    // Debug printout.
                    $display("Found cache line mapping to tag %h in way %d set %d", fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                end
            end

            // If not able to find the line in this cache.
            if (!fs_rdp_done) begin
                // Signal that we are done but not able to find the line.
                fs_rdp_done = 1;
                fs_rdp_suc = 0;

                // Debug printout.
                $display("L1 cahche");
                $display("Unable to find cache line with tag %h in set %d", fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
            end
        end
    end

    // Comb block to handle write requests.
    bit [3:0] write_state;
    bit [$clog2(WAYS)-1:0] rres;    // Round robin eviction select.
    always_comb begin
        // Defaults.
        fs_wrp_suc = 0;
        fs_wrp_done = 0;
        wrp_line_we = 0;

        // If write port enable is high.
        if (fs_wrp_en) begin

            // Different behaviours for different write states.
            case (write_state)

            // Case for writing to a line we already have.
            0:
            begin
                // Debug printout.
                $display("Attempting to modify if present already. %t", $time);

                // Loop over the lines to look for an existing line matching tag bits.
                for (integer i = 0; i < WAYS; i = i + 1) begin
                    // If the tag bits match and it is valid.
                    if (wrp_set_rd[i].tag == fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && wrp_set_rd[i].valid && !fs_wrp_done) begin
                        // Set write enable for this line high.
                        wrp_line_we[i] = 1;

                        // Present data to be written.
                        wrp_set_wr[i].dat = fs_wrp_dat;
                        wrp_set_wr[i].valid = 1;
                        wrp_set_wr[i].dirty = 1;
                        wrp_set_wr[i].tag = fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];

                        // Signal success.
                        fs_wrp_suc = 1;
                        fs_wrp_done = 1;

                        // Debug printout.
                        $display("Found a line to modify write to with tag %h at way %d set %d", fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_wrp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                    end
                end
            end

            // Case for writing a new line.
            1:
            begin
                // Debug printout.
                $display("Writing a new line to cache. %t", $time);

                // Loop over the lines in the selected set and look for an empty line.
                for (integer i = 0; i < WAYS; i = i + 1) begin
                    // If the valid bit is not set & we are not done.
                    if (!wrp_set_rd[i].valid && !fs_wrp_done) begin 
                        // Setup input to the line.
                        wrp_set_wr[i].dat = fs_wrp_dat;
                        wrp_set_wr[i].valid = 1;
                        wrp_set_wr[i].dirty = 1;
                        wrp_set_wr[i].tag = fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];

                        // Set write enable high.
                        wrp_line_we[i] = 1;

                        // Signal success and done.
                        fs_wrp_done = 1;
                        fs_wrp_suc = 1;

                        // Debug printout.
                        $display("Found an empty line to write data to, tag %h at way %d set %d", fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_wrp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
                    end
                end

                // If for some reason this is not done (idk how that would happen NGL).
                if (!fs_wrp_done) begin
                    // mark as done but not successful, wrapper module can just forward the write to the next cache level.
                    fs_wrp_done = 1;
                    fs_wrp_suc = 0;
                end
            end

            // Case for evicting a line to next level of cache.
            2:
            begin
                // Present the line for eviction to the back side.
                bs_addr = {wrp_set_rd[rres].tag, wrp_set_sel, 0};
                bs_dat = wrp_set_rd[rres].dat;
                bs_we = 1;

                // If the backside is signalling that it will accept the write on the next clock cycle.
                if (bs_dn) begin
                    // Setup the line to be overwritten.
                    wrp_set_wr[rres].tag = fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];
                    wrp_set_wr[rres].dat = fs_wrp_dat;
                    wrp_set_wr[rres].valid = 1;
                    wrp_set_wr[rres].dirty = 1;

                    // Signal for it to be overwritten.
                    wrp_line_we[rres] = 1;

                    // Signal success.
                    fs_wrp_done = 1;
                    fs_wrp_suc = 1;
                end
            end

            // Super secret write state for when eviction takes too long.
            3:
            begin
                // Hold done high and success low untill the fs_wrp_en goes low or something.
                fs_wrp_done = 1;
                fs_wrp_suc = 0;
            end
            endcase
        end
    end

    // always block to handle transitioning write_state for the write bit.
    bit set_full;
    bit [7:0] tmout_tmr;
    always @(posedge clk) begin
        // Default value.
        set_full = 1;
        // If the write port is active.
        if (fs_wrp_en) begin
            // Check which state we are in currently.
            case (write_state)
            // Case for modifying an existing line.
            0:
            begin
                // If not a success.
                if (!fs_wrp_done) begin
                    // Check if the set is full or not.
                    for (integer i = 0; i < WAYS; i = i + 1) begin if (!wrp_set_rd[i].valid) set_full = 0; end

                    // If the set is full then transition to evict a line next clock.
                    if (set_full) begin
                        write_state = 2;    // Move write state to the "evict a line" state.
                        tmout_tmr = 0;      // Reset the timeout timer.
                    end
                    // If the set is not full, then just write a new line to it.
                    else begin
                        write_state = 1;    // Move write state to the "write a new line" state.
                    end
                end
            end

            // Case for writing a new line.
            1:
            begin
                // If done.
                if (fs_wrp_done) begin
                    // Transition write state back to 0.
                    write_state = 0;
                end
            end

            // Case for evicting a line.
            2:
            begin
                // If done.
                if (fs_wrp_done) begin
                    // Transition write state back to 0.
                    write_state = 0;

                    // Incriment the round robin eviction select bits.
                    if (rres < WAYS) rres = rres + 1;
                    else rres = 0;
                end
                // If not done.
                else begin
                    // Incriment the timeout timer.
                    tmout_tmr = tmout_tmr + 1;

                    // If the timeout timer reaches the end.
                    if (tmout_tmr >= 255) begin
                        write_state = 3;        // Move to write state 3 where we can just sit and wait for fs_wrp_en to be deasserted.
                    end
                end
            end
            endcase

        end
        // If the write port is not active.
        else begin
            write_state = 0;
        end
    end

endmodule
/*
// This cache is designed to be fast and respond quickly, also somewhat area efficient I hope (pls dont go above 4 ways).
module l1_cache_old #(parameter WAYS = 2, LINES = 128, LINE_LENGTH = 16, ADDR_W = 32)(
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
            dual_port_sram #(LINES, $bits(cache_line)) way(clk, rst, set_sel, set_inp[i], line_we[i], set_out[i]);
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
*/

// This cache is designed to be somewhat fast but much higher capcity and very area efficient (targeting 8-16 ways).
module l2_cache #(parameter WAYS = 8, LINES = 512, LINE_LENGTH = 16, ADDR_W = 32)(
    input clk,
    input rst,

    // Front side of the cache.

    // Read port.
    input logic [ADDR_W-1:0]                fs_rdp_addr,    // Front side read port addr.
    input logic                             fs_rdp_en,      // Front side read port enable.
    output logic [(LINE_LENGTH * 8) - 1:0]  fs_rdp_dat,     // Front side read port data.
    output logic                            fs_rdp_done,    // Front side read port done.
    output logic                            fs_rdp_suc,     // Front side read port successful.

    // Write port.
    input logic [ADDR_W-1:0]                fs_wrp_addr,    // Front side write port addr.
    input logic                             fs_wrp_en,      // Front side write port enable.
    input logic [(LINE_LENGTH * 8) - 1:0]   fs_wrp_dat,     // Front side write port data.
    output logic                            fs_wrp_done,    // Front side write port done.
    output logic                            fs_wrp_suc,     // Front side write port successful.


    // Back side of the cache.

    output logic [ADDR_W-1:0]               bs_addr,        // Back side address select.
    output logic [(LINE_LENGTH * 8)-1:0]    bs_dat,         // Back side data output.
    output logic                            bs_we,          // Back side write enable.
    input logic                             bs_dn           // Back side done.
);

    ///         Setup a bunch of tag RAM            

    // define the tag ram line structure.
    typedef struct packed { // MSB
        bit [(ADDR_W - ($clog2(LINES) + $clog2(LINE_LENGTH)))-1:0]        tag;      // tage 20 its, 31:12
        bit                                                               dirty;    // dirty bit, indicates if the cache line is pfs_resent in main memory yet or not.
        bit                                                               valid;    // valid bit indicates if there is any data loaded.
    } tag_ram_line;         // LSB

    // Generate the SRAM blocks for the tag RAM.
    logic [$clog2(LINES)-1:0]   tr_rdp_set_sel;             // Tag ram read port set select.
    tag_ram_line                tr_rdp_set_rd [WAYS-1:0];   // Tag ram read port set data read.
    tag_ram_line                tr_rdp_set_wr [WAYS-1:0];   // Tag ram read port set data write.
    logic [WAYS-1:0]            tr_rdp_line_we;             // Tag ram read port line we.

    logic [$clog2(LINES)-1:0]   tr_wrp_set_sel;             // Tag ram write port set select.
    tag_ram_line                tr_wrp_set_rd [WAYS-1:0];   // Tag ram write port set data read.
    tag_ram_line                tr_wrp_set_wr [WAYS-1:0];   // Tag ram write port set data write.
    logic [WAYS-1:0]            tr_wrp_line_we;             // Tag ram write port line we.

    generate 
        for (genvar i = 0; i < WAYS; i = i + 1) begin
            dual_port_sram #(LINES, $bits(tag_ram_line)) tag_ram_way(clk, rst, tr_rdp_set_sel, tr_rdp_line_we[i], tr_rdp_set_wr[i], tr_rdp_set_rd[i], tr_wrp_set_sel, tr_wrp_line_we[i], tr_wrp_set_wr[i], tr_wrp_set_rd[i]);
        end    
    endgenerate

    // Assign the index bits to drive set sel for read port.
    assign tr_rdp_set_sel = fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)];
    // Assign the index bits to drive set sel for write port.
    assign tr_wrp_set_sel = fs_wrp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)];


    //          Setup a bunch of SRAM for the lines.

    // Generate a bunch of SRAM banks, 1 for each way.
    logic [$clog2(LINES)-1:0]           rdp_set_sel;            // Read port set select.
    logic [(LINE_LENGTH * 8) - 1 : 0]   rdp_set_rd  [WAYS-1:0]; // Read port set data read.
    logic [(LINE_LENGTH * 8) - 1 : 0]   rdp_set_wr  [WAYS-1:0]; // Read port set data write.
    logic [WAYS-1:0]                    rdp_line_we;            // Read port line we.
    
    logic [$clog2(LINES)-1:0]           wrp_set_sel;            // Write port set select.
    logic [(LINE_LENGTH * 8) - 1 : 0]   wrp_set_rd  [WAYS-1:0]; // Write port set data read.
    logic [(LINE_LENGTH * 8) - 1 : 0]   wrp_set_wr  [WAYS-1:0]; // Write port set data write.
    logic [WAYS-1:0]                    wrp_line_we;            // Write port line we.

    generate 
        for (genvar i = 0; i < WAYS; i = i + 1) begin
            dual_port_sram #(LINES, LINE_LENGTH * 8) way(clk, rst, rdp_set_sel, rdp_line_we[i], rdp_set_wr[i], rdp_set_rd[i], wrp_set_sel, wrp_line_we[i], wrp_set_wr[i], wrp_set_rd[i]);
        end
    endgenerate

    // Assign the index bits to drive set sel for read port.
    assign rdp_set_sel = fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)];
    // Assign the index bits to drive set sel for write port.
    assign wrp_set_sel = fs_wrp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)];


    //          Logic to handle processing the read and write requests.

    // Comb block to handle read requests.
    bit [1:0] read_state;
    bit [$clog2(WAYS)-1:0] read_way_sel;
    always_comb begin
        // Default values.
        fs_rdp_done = 0;
        fs_rdp_suc = 0;

        // Loop over the tag lines to check if we have the line.
        for (integer i = 0; i < WAYS; i = i + 1) begin
            // If the tag bits match & the line is valid.
            if (tr_rdp_set_rd[i].tag == fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && tr_rdp_set_rd[i].valid) begin
                // Signal that we have the cache line.
                fs_rdp_suc = 1;
                // Store the index in the read_way_sel so that it can be used to read out the data.
                read_way_sel = i;
            end
        end

        // If the read port is enabled.
        if (fs_rdp_en) begin
            // Do different things based on the state of the read state machine.
            case (read_state) 
            
            // If we are at the checking tag ram stage of the state machine.
            0:
            begin

                // If fs_rdp_suc == 0 then we dont have the line.
                if (!fs_rdp_suc) begin
                    fs_rdp_done = 1;    // Signal that we are done but dont have the line.
                end
            end

            // If we have found the tag for the line in tag RAM.
            1:
            begin
                // Signal done to the front side read port.
                fs_rdp_done = 1;

                // Present data to the readport.
                fs_rdp_dat = rdp_set_rd[read_way_sel];

                // Signal to the tag ram that we need to invalidate the line.
                tr_rdp_line_we[read_way_sel] = 1;
                tr_rdp_set_wr[read_way_sel] = 0;
            end
            endcase
        end
        // If the read port is not enabled.
        else begin
        end
    end

    always @(posedge clk) begin

        // If the read port is enabled.
        if (fs_rdp_en) begin
            // Different behaviour depending on stage of the read.
            case (read_state)
            // If we are currently checking tags.
            0:
            begin
                // If we do poses the cache line, transition to read state where we read the data from the line ram.
                if (fs_rdp_suc) begin
                    read_state = 1;
                end
            end
            endcase
        end
        // If the read port is not enabled.
        else begin
            // Reset read state to 0.
            read_state = 0;
        end
    end

endmodule

/*
// This cache is designed to be somewhat fast but much higher capcity and very area efficient (targeting 8-16 ways).
module l2_cache_old #(parameter WAYS = 8, LINES = 512, LINE_LENGTH = 16, ADDR_W = 32, LINES_PER_CLK = 4)(
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
            dual_port_sram #(LINES, $bits(cache_line)) way(clk, rst, set_sel, set_inp[i], line_we[i], set_out[i]);
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
*/
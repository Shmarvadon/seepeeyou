// https://verificationacademy.com/forums/t/blocking-or-non-blocking-statement-vs/42389    dave_59
// Big up dave_59, that man has all the wisdom in the world.
`include "structs.svh"

`define CACHE_DEBUG_LOG

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

`ifdef CACHE_DEBUG_LOG
            $display("A write is happening on line %d writing: %h", prt_1_addr, prt_1_dinp);
`endif
        end
        // If port 2 is writing.
        if (prt_2_we) begin
            sram[prt_2_addr] = prt_2_dinp;

`ifdef CACHE_DEBUG_LOG
            $display("A write is happening on line %d writing: %h", prt_2_addr, prt_2_dinp);
`endif
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

`ifdef CACHE_DEBUG_LOG
            $display("Attempting to read a line from cache.");
`endif
            // Loop over all the lines currently selected by rdp_set_sel.
            for (integer i = 0; i < WAYS; i = i + 1) begin
                // If the line has matching tag & is valid.
                if (rdp_set[i].tag == fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))] && rdp_set[i].valid) begin
                    // Present data to output and signal done & success.
                    fs_rdp_dat = rdp_set[i].dat;

                    fs_rdp_done = 1;
                    fs_rdp_suc = 1;

`ifdef CACHE_DEBUG_LOG
                    $display("Found cache line mapping to tag %h in way %d set %d", fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
`endif
                end
            end

            // If not able to find the line in this cache.
            if (!fs_rdp_done) begin
                // Signal that we are done but not able to find the line.
                fs_rdp_done = 1;
                fs_rdp_suc = 0;

`ifdef CACHE_DEBUG_LOG
                $display("L1 cahche");
                $display("Unable to find cache line with tag %h in set %d", fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
`endif
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
`ifdef CACHE_DEBUG_LOG
                $display("Attempting to modify if present already. %t", $time);
`endif
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

`ifdef CACHE_DEBUG_LOG
                        $display("Found a line to modify write to with tag %h at way %d set %d", fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_wrp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
`endif
                    end
                end
            end

            // Case for writing a new line.
            1:
            begin
`ifdef CACHE_DEBUG_LOG
                $display("Writing a new line to cache. %t", $time);
`endif
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

`ifdef CACHE_DEBUG_LOG
                        $display("Found an empty line to write data to, tag %h at way %d set %d", fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], i, fs_wrp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
`endif
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

// This cache is designed to be somewhat fast but much higher capcity and very area efficient (targeting 8-16 ways), also an exclusive cache.
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

    // always block to handle read operations.
    bit [$clog2(WAYS)-1:0] read_way_sel;
    bit [ADDR_W-1:0] prev_rdp_addr;
    always @(posedge clk) begin
        // Default values.
        tr_rdp_line_we = 0;
        read_way_sel = 0;

        // If the read port is enabled.
        if (fs_rdp_en) begin
`ifdef CACHE_DEBUG_LOG
            $display("Attempting to read a line from cache.");
`endif
            fs_rdp_done = 0;
            fs_wrp_done = 0;

            // Update the previous addr for fs read port address.
            prev_rdp_addr <= fs_rdp_addr;

            // If the address has changed, then we stop holding output and process new request.
            if (prev_rdp_addr != fs_rdp_addr) begin fs_rdp_done = 0; fs_rdp_suc = 0; end
            // Above should mean that output is held until next request even if en is kept high.

            // If the read port is enabled & not done yet.
            if (fs_rdp_en & !fs_rdp_done) begin

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

                // If success.
                if (fs_rdp_suc) begin
                    // Signal to tag ram to mark the line as invalid next cycle.
                    tr_rdp_set_wr[read_way_sel].valid <= 0;
                    tr_rdp_line_we[read_way_sel] <= 1;
                end
                else begin
`ifdef CACHE_DEBUG_LOG
                    $display("L2 cahche");
                    $display("Unable to find cache line with tag %h in set %d", fs_rdp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))], fs_rdp_addr[$clog2(LINE_LENGTH)+:$clog2(LINES)]);
`endif
                end

                // Signal done.
                fs_rdp_done <= 1;
            end
        end
        else begin
            fs_rdp_suc = 0;
            fs_rdp_done = 0;
        end
    end

    // Assign the read_way_sel bits to drive the output of the read port.
    assign fs_rdp_dat = rdp_set_rd[read_way_sel];



    // always block to handle write operations.
    bit [$clog2(WAYS)-1:0] write_way_sel;
    bit [$clog2(WAYS)-1:0] rres;
    bit [ADDR_W-1:0] prev_wrp_addr;
    bit [3:0] write_state;
    bit line_found;
    bit [7:0] evic_timeout;
    always @(posedge clk) begin
        // Default values.
        line_found = 0;

        tr_wrp_line_we = 0;
        wrp_line_we = 0;

        bs_we = 0;

        // If the front side write port enable is high.
        if (fs_wrp_en) begin

            fs_wrp_suc = 0;
            fs_wrp_done = 0;

            // Update the value of prev wrp addr for next clk.
            prev_wrp_addr <= fs_wrp_addr;

            // If the input addr is not the same as prev cycle, then back to stage 0.
            if (prev_wrp_addr != fs_wrp_addr) begin
                write_state = 0;
            end

            if (!fs_wrp_done) begin
                case (write_state) 
                // Stage 1, locate an empty line if possible
                0:
                begin
                    // Loop over all the ways looking for one that is empty.
                    for (int i = 0; i < WAYS; i = i + 1) begin
                        // If the line exists already.
                        if (tr_wrp_set_rd[i].tag == fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))]) begin
                            write_way_sel = i;
                            write_state = 1;
                            line_found = 1;
                        end
                        // If the way is empty (valid bit is set to 0), set write state to 1 to indicate that a new line can be written.
                        if (!tr_wrp_set_rd[i].valid & !line_found) begin write_way_sel = i; write_state = 1; end
                    end

                    // If write state is still 0, we need to evict a line from this cache to be able to write another line.
                    if (write_state == 0) write_state = 2;

                    // zero eviction timeout.
                    evic_timeout <= 0;
                end

                // Case 1 for stage 2, an empty line is available.
                1:
                begin
                    // Setup the line for a write (data).
                    wrp_set_wr[write_way_sel] <= fs_wrp_dat;
                    wrp_line_we[write_way_sel] <= 1;

                    // Setup the line for a write (tag ram).
                    tr_wrp_set_wr[write_way_sel].tag <= fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];
                    tr_wrp_set_wr[write_way_sel].valid <= 1;
                    tr_wrp_set_wr[write_way_sel].dirty = 1;
                    tr_wrp_line_we[write_way_sel] <= 1;

                    // Signal done & success.
                    fs_wrp_done <= 1;
                    fs_wrp_suc <= 1;
                end

                // Case 2 for stage 2, no line is available and one needs to be evicted.
                2:
                begin
                    // Present the line for eviction.
                    bs_addr <= {tr_wrp_set_rd[rres].tag, wrp_set_sel, 0};
                    bs_dat <= wrp_set_rd[rres];
                    bs_we <= 1;

                    // If the back side is done.
                    if (bs_dn) begin
                        // Setup the line for a write (data).
                        wrp_set_wr[rres] <= fs_wrp_dat;
                        wrp_line_we[rres] <= 1;

                        // Setup the line for a write (tag ram).
                        tr_wrp_set_wr[rres].tag <= fs_wrp_addr[ADDR_W-1:($clog2(LINES) + $clog2(LINE_LENGTH))];
                        tr_wrp_set_wr[rres].valid <= 1;
                        tr_wrp_set_wr[rres].dirty = 1;
                        tr_wrp_line_we[rres] <= 1;

                        // Signal done & success.
                        fs_wrp_done <= 1;
                        fs_wrp_suc <= 1;
                    end
                    // If the backside is not done.
                    else begin
                        // Incriment the eviction timeout.
                        evic_timeout = evic_timeout + 1;
                        // If the timeout is exceeded then cancel the write and signal unsuccsessful.
                        if (evic_timeout == 255) begin
                            fs_wrp_done <= 1;
                            fs_wrp_suc = 0;
                        end
                    end
                end
                endcase
            end
        end
        else begin
            fs_wrp_done <= 0;
            fs_wrp_suc <= 0;
            write_state <= 0;
        end
    end
endmodule



typedef struct packed {
    bit [127:0]   dat;      // Data, 128 bits / 16 bytes.
    bit [19:0]    tag;      // tage 20 its, 31:12
    bit [15:0]    bo;       // byte occupied, bitmask for all 16 bytes indicating if they contain data or not.
    bit           dirty;    // dirty bit, indicates if this cache line is pfs_resent in main memory yet or not.
} cache_line;

// https://verificationacademy.com/forums/t/blocking-or-non-blocking-statement-vs/42389    dave_59

module cache #(parameter WAYS = 16, LINES = 256)(
    input clk,                      // clock.
    input rst,                      // reset.

    // Front side.

    input [31:0] fs_addr,           // Front side address.
    input fs_re,                    // Front side read enable.
    input fs_we,                    // Front side write enable.

    output bit [127:0] fs_doup,     // Front side Data output.
    output bit [15:0] fs_booup,     // Front side byte occupied output.

    input [127:0] fs_dinp,          // Front side data input.
    input [15:0] fs_boinp,          // Front side byte occupied input.

    output bit fs_lp,               // Front side line present.
    output bit fs_done,             // Front side done.

    // Back side.

    output bit [31:0] bs_addr,      // Back side address.
    output bit [127:0] bs_doup,     // Back side data output.
    output bit [15:0] bs_booup,     // Back side byte occupied output.

    input bs_done,                  // Back side done.
    output bit bs_we                // Back side write enable.

);

// Cache lines.
cache_line [LINES] lines [WAYS];        // 16 way, 256 lines per way.

bit [$clog2(WAYS):0] evic_line_sel;

// Handle accessing the cache.
integer i;
integer j;
always @(posedge clk) begin
    // Default case.
    fs_doup = 0;
    fs_booup = 0;
    fs_lp = 0;
    fs_done = 0;

    case ({fs_re, fs_we})
    // Cache read.
    2'b10:
    begin
        // Loop over the set pointed to by the line index bits to find which way has valid line.
        for (i = 0; i < WAYS; i = i + 1) begin
            // If the tag bits match & the line is valid (contains some data).
            if (lines[i][fs_addr[4+:$clog2(LINES)]].tag[0+: (31 - ($clog2(LINES) + 4))] == fs_addr[31:($clog2(LINES) + 4)] && lines[i][fs_addr[4+:$clog2(LINES)]].bo != 0) begin

                $display("Found cache line holding data, way %d, set %d", i, fs_addr[4+:$clog2(LINES)]);
                // Pfs_resent the cache line at output.
                fs_doup = lines[i][fs_addr[4+:$clog2(LINES)]].dat;
                fs_booup = lines[i][fs_addr[4+:$clog2(LINES)]].bo;

                // Signal that the cache line is present.
                fs_lp = 1;

                // Signal that fs_we afs_re fs_done.
                fs_done = 1;
            end
        end

        // If the line is not present then default conditions above will set fs_lp = 0 and zero d_oup & fs_booup.
        if (!fs_done) $display("Could not find line containing tag.");
    end

    // Cache write.
    2'b01:
    begin
        // We want to first check if the line is already present in cache.
        // If it is we overwrite and set the dirty bit.
        // If not then we look for a free cache line.
        // If there is no free cache line we need to evict a cache line.


        // Loop over the set pointed to by the line index bits to find which way has valid line.
        for (i = 0; i < WAYS; i = i + 1) begin

            // If the tag bits match & the line is valid (contains some data).
            if (lines[i][fs_addr[4+:$clog2(LINES)]].tag[0+: (31 - ($clog2(LINES) + 4))] == fs_addr[31:($clog2(LINES) + 4)] && lines[i][fs_addr[4+:$clog2(LINES)]].bo != 0) begin

                $display("Found cache line alfs_ready holding data belonging to tag bits:%d way %d, set %d", fs_addr[31:($clog2(LINES) + 4)], i, fs_addr[4+:$clog2(LINES)]);

                // Write to the cache line.
                for (j = 0; j < WAYS; j = j + 1) begin
                    if (fs_boinp[j]) begin lines[i][fs_addr[4+:$clog2(LINES)]].dat[8*j+:8] = fs_dinp[8*j+:8]; end
                end
                // Or the byte occupied bitmask input with the one pfs_resent in the cache line.
                lines[i][fs_addr[4+:$clog2(LINES)]].bo = lines[i][fs_addr[4+:$clog2(LINES)]].bo | fs_boinp;
                // Set the dirty bit indicating that the cache line has not been written back to memory.
                lines[i][fs_addr[4+:$clog2(LINES)]].dirty = 1;

                // Signal that we are done.
                fs_done = 1;

                $display("%d", lines[i][fs_addr[4+:$clog2(LINES)]].dat);

                $display("Written data to cache line: way %d, set %d", i, fs_addr[4+:$clog2(LINES)]);
            end
        end

        // If not done.
        if (!fs_done) begin

            // I think this should act like a priority encoder to select smallest i line thats empty (I cope and pray it does).
            // Look for an empty cache line in the set that the address maps to.
            for (i = 0; i < WAYS; i = i + 1) begin

                // If not done.
                if (!fs_done) begin
                    // If the line is empty.
                    if (lines[i][fs_addr[4+:$clog2(LINES)]].bo == 0) begin

                        // Write the fs_dinp, fs_boinp to cache line & set dirty bit & set tag.
                        lines[i][fs_addr[4+:$clog2(LINES)]].dat = fs_dinp;
                        lines[i][fs_addr[4+:$clog2(LINES)]].bo = fs_boinp;
                        lines[i][fs_addr[4+:$clog2(LINES)]].dirty = 1;
                        lines[i][fs_addr[4+:$clog2(LINES)]].tag[0+: (31 - ($clog2(LINES) + 4))] = fs_addr[31:($clog2(LINES) + 4)];

                        // Set fs_done bit.
                        fs_done = 1;

                        $display("Found empty cache line to write data to: way %d, set %d    %t", i, fs_addr[4+:$clog2(LINES)], $time);
                        $display("%b", fs_boinp);
                        $display("%b", lines[i][fs_addr[4+:$clog2(LINES)]].bo);
                    end
                end
            end
        end

        // If still not done.
        if (!fs_done) begin

            $display("Evicting cache line from way %d to write new data: set %d", evic_line_sel, fs_addr[4+:$clog2(LINES)]);

            // If not already put in a write request to memory for the cache line selected for eviction.
            case (bs_we)

                // bs_we set to 0, eviction write to memory request not sent.
                0:
                begin
                    // Submit request to backside to write cache line to memory.
                    bs_we = 1;
                    bs_doup = lines[evic_line_sel][fs_addr[4+:$clog2(LINES)]].dat;
                    bs_booup = lines[evic_line_sel][fs_addr[4+:$clog2(LINES)]].bo;
                    bs_addr = {lines[evic_line_sel][fs_addr[4+:$clog2(LINES)]].tag[0+: (31 - ($clog2(LINES) + 4))], fs_addr[11:0]};

                end

                // bs_we set to 1, eviction write to memory request sent.
                1:
                begin
                    // If the back side has finished its memory write.
                    if (bs_done) begin

                        // Lets write the cache line.
                        lines[i][fs_addr[4+:$clog2(LINES)]].dat = fs_dinp;
                        lines[i][fs_addr[4+:$clog2(LINES)]].bo = fs_boinp;
                        lines[i][fs_addr[4+:$clog2(LINES)]].dirty = 1;
                        lines[i][fs_addr[4+:$clog2(LINES)]].tag[0+: (31 - ($clog2(LINES) + 4))] = fs_addr[31:($clog2(LINES) + 4)];

                        // stop signalling we to back side and signal done to fs.
                        bs_we = 0;
                        fs_done = 1;

                        // incriment evic_line_sel or set to 0 if it loops around.
                        if (evic_line_sel < WAYS) evic_line_sel = evic_line_sel + 1;
                        else evic_line_sel = 0;
                    end
                end
            endcase
        end
    end

    // default condition.
    default:
    begin
        $display("Here for some reason?");
    end
    endcase    
end

// reset condition.
integer k;
integer l;
always @(clk, rst) begin
    if (rst) begin
        for (k = 0; k < LINES; k = k + 1) begin
            for (l = 0; l < WAYS; l = l + 1) begin
                lines[l][k] = 0;
            end
        end
    end
end
endmodule
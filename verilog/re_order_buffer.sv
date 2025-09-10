`include "structs.svh"

module re_order_buffer #(parameter ROB_LEN = 16, ROB_HEADS = 3, ROB_TAILS = 1, EXEC_UNITS = 4)(
    input   logic                                       clk,            // Clock.
    input   logic                                       rst,            // Reset.

    // ROB heads.
    input   logic [ROB_HEADS-1:0]                       inp_p,          // Input push.
    input   rob_entry [ROB_HEADS-1:0]                   inp_dat,        // Input data.
    output  logic [ROB_HEADS-1:0][$clog2(ROB_LEN)-1:0]  inp_ptr,        // Input ROB pointer.
    output  logic [$clog2(ROB_LEN)-1:0]                 src_num_avail,  // Source number of slots available.

    // ROB tails.
    input   logic [ROB_TAILS-1:0]                       oup_p,          // Output pop.
    output  rob_entry [ROB_TAILS-1:0]                   oup_dat,        // Output data.
    output  logic [$clog2(ROB_LEN)-1:0]                 dst_num_avail,  // Destination number of slots available.

    // ROB side ports.  
    input   logic [EXEC_UNITS-1:0]                      sp_md,          // Side port mark done.
    input   logic [EXEC_UNITS-1:0][$clog2(ROB_LEN)-1:0] sp_ptr          // Side port pointer.
);

    // Ensure that ROB_LEN parameter is a power of 2.
    generate
    if (ROB_LEN & (ROB_LEN - 1)) $error("Length must be power of 2.");
    endgenerate


    // ROB data entries.
    rob_entry [ROB_LEN-1:0]         rob_entries;

    // Read pointer.
    logic [$clog2(ROB_LEN):0]       rptr;

    // Write pointer.
    logic [$clog2(ROB_LEN):0]       wptr;

    // Logic to handle pushing and popping from the ROB.
    logic [$clog2(ROB_LEN):0]         rptr_tmp;
    logic [$clog2(ROB_LEN):0]         wptr_tmp;
    always @(posedge clk) begin
        
        // Loop over each of the push ports.
        for (int i = 0; i < ROB_HEADS; i = i + 1) begin
            if (inp_p[i]) begin
                // If the shift register is not full.
                if (!(wptr[$clog2(ROB_LEN)-1:0] == rptr[$clog2(ROB_LEN)-1:0] && wptr[$clog2(ROB_LEN)] != rptr[$clog2(ROB_LEN)])) begin
                    // Push the data.
                    rob_entries[wptr[$clog2(ROB_LEN)-1:0]] = inp_dat[i];

                    // Incriment the wptr.
                    wptr = wptr + 1;
                end
            end
        end

        // Update the number of entries that are unoccupied.
        src_num_avail <= (rptr[$clog2(ROB_LEN)] == wptr[$clog2(ROB_LEN)]) ? (ROB_LEN - (wptr[$clog2(ROB_LEN)-1:0] - rptr[$clog2(ROB_LEN)-1:0])) : (ROB_LEN - (wptr[$clog2(ROB_LEN)-1:0] + ROB_LEN - rptr[$clog2(ROB_LEN)-1:0]));
    

        // loop over each of the side ports and mark the entries they are pointing to as fully executed.
        for (int i = 0; i < EXEC_UNITS; i = i + 1) begin
            if (sp_md[i]) begin
                rob_entries[sp_ptr[i]].dn = 1;
            end
        end


        // Loop over each of the pop ports.
        for (int i = 0; i < ROB_TAILS; i = i + 1) begin
            // If the position wants to pop & the sr is not empty.
            if (oup_p[i] & rptr != wptr) begin
                // Incriment the rptr.
                rob_entries[rptr] = 0;
                rptr = rptr + 1;
            end
        end

        // Loop over each of the pop ports to present data to them.
        rptr_tmp = rptr;
        for (int i = 0; i < ROB_TAILS; i = i + 1) begin
            // if not empty.
            if (rptr_tmp != wptr) begin
                // Present the data.
                oup_dat[i] <= rob_entries[rptr_tmp[$clog2(ROB_LEN)-1:0]];
            end
            else begin
                oup_dat[i] <= 0;
            end
            // Incriment rptr_tmp.
            rptr_tmp = rptr_tmp + 1;
        end

        // Update the number of entries that are occupied.
        dst_num_avail <= (rptr[$clog2(ROB_LEN)] == wptr[$clog2(ROB_LEN)]) ? (wptr[$clog2(ROB_LEN)-1:0] - rptr[$clog2(ROB_LEN)-1:0]) : (wptr[$clog2(ROB_LEN)-1:0] + ROB_LEN - rptr[$clog2(ROB_LEN)-1:0]);



        // Loop over the write ports and present the ROB pointer to them.
        wptr_tmp = wptr;
        for (int i = 0; i < ROB_HEADS; i = i + 1) begin
            // Present the pointer for where the head will input to.
            inp_ptr[i] <= wptr_tmp[$clog2(ROB_LEN)-1:0];

            // Incriment temporary write pointer.
            wptr_tmp <= wptr_tmp + 1;
        end
    end
endmodule
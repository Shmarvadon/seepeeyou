`include "structs.svh"

// Impliment ISA alloc port for allocating ISA regs.

// Tap PRF write lines to update register validity but NOT update RAT_c.
// RAT_c is updated by the commit / retirement stage which is also responsible for clearing Immediates and non ISA PR allocation.

module scoreboard #(parameter PR_ALLOC_PRTS = 2, PR_FREE_PRTS = 2, NUM_PHYSICAL_REGS = 64, NUM_PRF_WR_PRTS = 4)(
    input   logic                                                           clk,            // Clock.
    input   logic                                                           rst,            // Reset.

    // Register allocation ports.
    input   logic   [PR_ALLOC_PRTS-1:0]                                     alloc_pr,       // Allocate physical register
    output  logic   [PR_ALLOC_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]      allocd_reg,     // Allocated physical register (PR).
    output  logic   [$clog2(NUM_PHYSICAL_REGS)-1:0]                         alloc_av,       // Number of free PRs.

    // Register freeing ports.
    input   logic   [PR_FREE_PRTS-1:0]                                      free_pr,        // Signal to free the physical register specified in pr_to_free.
    input   logic   [PR_FREE_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]       pr_to_free,     // Signal which physical register to free.

    // Output data to represent which of the PRs are valid. (goes to despatch)
    output  logic   [NUM_PHYSICAL_REGS-1:0]                                 pr_valid,       // Signal if the physical register holds valid data yet or not.


    // Tap PRF write ports to update PR status from invalid to valid on write.
    input   logic   [NUM_PRF_WR_PRTS-1:0]                                   prf_we,         // PRF write enable.
    input   logic   [NUM_PRF_WR_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]    prf_wr_trgt     // PRF write target.
);

    // Scoreboard data.
    register_status [NUM_PHYSICAL_REGS-1:0]         sb_dat;


    // Order of events:
    // Free PRs indicated by the commit / retirement stage.
    // Update allocation with requested allocations from alloc ports.
    // Update PR validity based on RF write signals.
    // Present data to outputs.


    int curr_alloc_prt;
    int num_free_prs;
    always @(posedge clk) begin
        // Default values.
        alloc_av <= 0;
        allocd_reg <= 0;
        pr_valid <= 0;
        curr_alloc_prt = 0;
        num_free_prs = 0;

        // Free PRs that are being requested to be freed.
        for (int i = 0; i < PR_FREE_PRTS; i = i + 1) begin
            // If it needs to be freed, free it.
            if (free_pr[i]) begin
                sb_dat[pr_to_free[i]].free = 1;
                sb_dat[pr_to_free[i]].valid = 0;
            end
        end

        // Allocate registers that are being requested by alloc ports.
        for (int i = 0; i < PR_ALLOC_PRTS; i = i + 1) begin
            // If this port is requesting to allocate its register.
            if (alloc_pr[i]) begin
                sb_dat[allocd_reg[i]].free = 0;
                sb_dat[allocd_reg[i]].valid = 0;
            end
        end

        // Set PRs that can be allocated at next posedge.
        for (int i = 0; i < NUM_PHYSICAL_REGS; i = i + 1) begin
            // If curr_alloc_prt is in range of number of alloc ports & PR is free.
            if (curr_alloc_prt < PR_ALLOC_PRTS & sb_dat[i].free) begin
                // Present the register to the allocation port.
                allocd_reg[curr_alloc_prt] <= i;

                // Signal that the port can allocate on next posedge.
                alloc_av[curr_alloc_prt] <= 1;

                // Incriment curr_alloc_prt.
                curr_alloc_prt = curr_alloc_prt + 1;
            end

            // If the current PR is free.
            if (sb_dat[i].free) begin
                num_free_prs = num_free_prs + 1;
            end
        end

        // Update validity of PR data based off of PRF write port signals.
        for (int i = 0; i < NUM_PRF_WR_PRTS; i = i + 1) begin
            // If the port is writing to a register.
            if (prf_we[i]) begin
                // Set the valid bit on the PR that it is writing to.
                sb_dat[prf_wr_trgt[i]].valid = 1;
            end
        end

        // Present outputs.
        for (int i = 0; i < NUM_PHYSICAL_REGS; i = i + 1) begin
            // Present valid.
            pr_valid[i] <= sb_dat[i].valid;
        end
        alloc_av <= num_free_prs;
    end


    // Async reset behaviour.
    always @(posedge clk, rst) begin
        // If reset.
        if (rst) begin
            alloc_av <= 0;
            allocd_reg <= 0;
            pr_valid <= 0;

            // Allocate default locations for ISA regs.
            for (int i = 0; i < NUM_PHYSICAL_REGS; i = i + 1) begin
                // If an ISA reg.
                if (i <= 18) begin
                    sb_dat[i].free = 0;
                    sb_dat[i].valid = 1;
                end
                else begin
                    sb_dat[i].free = 1;
                    sb_dat[i].valid = 0;
                end
            end
        end
    end

endmodule
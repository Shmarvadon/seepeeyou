/*
module general_purpose_registers #(parameter REG_COUNT = 32)(
    input   logic                       clk,        // Clock.
    input   logic                       rst,        // Reset.

    input   logic  [REG_COUNT-1:0]      we,         // Write enable.
    input   logic  [31:0]               inp [REG_COUNT-1:0], // Data input.
    output  logic  [31:0]               oup [REG_COUNT-1:0]  // Data output.
);

// The registers.
logic [31:0] registers [REG_COUNT-1:0];
// Assign them to drive the output.
assign oup = registers;

// Handle input to the registers.
integer i;
always @(posedge clk) begin
    // Loop over each reg and input if the flag is det.
    for (i = 0; i < 16; i = i + 1) begin
        if (we[i]) begin
            registers[i] <= inp[i];
        end
    end
end

// Handle reset.
always @(posedge rst) begin
    for(i = 0; i < REG_COUNT; i = i + 1) begin
        registers[i] <= 0;
    end
end
endmodule
*/



// Simple wrapper around the re-usable register module.
module general_purpose_registers #(parameter REG_WIDTH = 32, NUM_REGS = 32, NUM_WR_PRTS = 4, NUM_RD_PRTS = 4) (
    input   logic                                           clk,        // Clock.
    input   logic                                           rst,        // Reset.

    // Read ports.
    input   logic [NUM_RD_PRTS-1:0][$clog2(NUM_REGS)-1:0]   rd_trgt,    // Read target register.
    output  logic [NUM_RD_PRTS-1:0][REG_WIDTH-1:0]          rd_dat,     // Rad data.

    // Write ports.
    input   logic [NUM_RD_PRTS-1:0]                         we,         // Write enable.
    input   logic [NUM_RD_PRTS-1:0][$clog2(NUM_REGS)-1:0]   wr_trgt,    // Write target.
    input   logic [NUM_RD_PRTS-1:0][REG_WIDTH-1:0]          wr_dat      // Write data.
);
    // register interfaces internally.
    logic [NUM_REGS-1:0][REG_WIDTH-1:0] reg_dinp;
    logic [NUM_REGS-1:0][REG_WIDTH-1:0] reg_doup;
    logic [NUM_REGS-1:0]                reg_we;

    // Generate an array of standard registers.
    generate
        for (genvar i = 0; i < NUM_REGS; i = i + 1) begin
            register #(REG_WIDTH, 0) reg_alloc(.clk(clk), .rst(rst), .inp(reg_dinp[i]), .oup(reg_doup[i]), .we(reg_we[i]));
        end
    endgenerate

    // Handle reading.
    always_comb begin
        // Default.
        rd_dat = 0;

        // Loop over all the read ports and present data that their target is asking for.
        for (int i = 0; i < NUM_RD_PRTS; i = i + 1) begin
            rd_dat[i] = reg_doup[rd_trgt[i]];
        end
    end

    // Handle writing.
    always_comb begin
        // Default.
        reg_we = 0;

        // Loop over all the write ports and present data to their target.
        for (int i = 0; i < NUM_WR_PRTS; i = i + 1) begin
            // Set write enable on the target register.
            reg_we[wr_trgt[i]] = we[i];
            // Present the data to the target register.
            reg_dinp[wr_trgt[i]] = wr_dat[i];
        end
    end
endmodule

module general_purpose_registers(
    input          clk,
    input          rst,
    
    input   [15:0] we,
    input   [31:0] inp [15:0],
    output  bit [31:0] oup [15:0]
);

// The registers.
bit [31:0] registers [15:0];
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
    for(i = 0; i < 16; i = i + 1) begin
        registers[i] <= 0;
    end
end
endmodule
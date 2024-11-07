
module clock_generator(
    input           en,
    input           rst,
    input   [7:0]   scalar_inp,
    output  [7:0]   scalar_oup,
    input           scalar_write,

    input clk_inp,
    output clk_oup
);

reg [7:0] clk_scalar;
reg [7:0] ctr;

reg gen_clk;

// Assignments for outputs.
assign clk_oup = gen_clk;
assign scalar_oup = clk_scalar;

// handle incrimenting of the ctr.
always @(clk_inp) begin

    case (en)
    1:
        begin
            if (ctr == clk_scalar) begin
                ctr <= 0;
                gen_clk =~ gen_clk;
            end
            else begin
                ctr <= ctr + 1;
            end
        end
    0:
        begin
        end
    endcase
end

// Handle input of the scalar.
always @(posedge scalar_write) begin
    if (en) begin
        clk_scalar <= scalar_inp;
    end
end

// Reset conditions.
always @(posedge rst) begin
    gen_clk <= 0;
    clk_scalar <= 0;
    ctr <= 0;
end


endmodule
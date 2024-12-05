module multi_clk_domain_data_adapter #(parameter WIDTH = 128)(
    input               clk_1,
    input               clk_2,
    input               rst,

    output              available,
    output              tx_complete,
    output [WIDTH-1:0]  dat_out,
    input               start_tx,
    input [WIDTH-1:0]   dat_in
);

// data reg.
reg [WIDTH-1:0] data;
assign dat_out = data;

// transaction status.
reg[1:0] tx_status;
// 0 - ready.
// 1 - in progress.
// 2 - recieved on other side.

// If we are not busy then mark as available.
assign available = tx_status == 0 ? 1 : 0;

// If tx is complete indicate to external connection.
assign tx_complete = tx_status == 2 ? 1 : 0;

// Handle clk_1 side of things.
always @(posedge clk_1) begin
    // If we are able to start a new tx & one is requested.
    if (tx_status == 0 & start_tx) begin
        // Move data into the reg.
        data <= dat_in;

        // Move to next state of tx.
        tx_status <= 1;
    end
end

// Handle clk 2 side of things.
always @(posedge clk_2) begin

    case (tx_status) 
    1: tx_status <= 2;  // Mark tx as recieved.
    2: tx_status <= 0;  // Go back to allowing next tx.
    endcase
end

// reset case.
always @(posedge rst) begin
    data <= 0;
    tx_status <= 0;
end
endmodule
module instruction_decoder #(parameter INST_QUEUE_LEN = 64, INP_LEN = 4)(
    input clk,
    input rst,
    input en
);

// Instruction decode queue stuff.
reg queue_we;
reg queue_se;
reg [7:0] queue_sa;
wire [(8*INP_LEN) -1:0] queue_din;
wire [7:0] queue_dout [INST_QUEUE_LEN -1: 0];
wire [7:0] queue_len;

decoder_input_register #(INST_QUEUE_LEN, INP_LEN) inst_decode_queue(clk, rst, queue_we, queue_se, queue_sa, queue_din, queue_dout, queue_len);

// Instruction queue stuff.
reg inst_queue_we;
reg inst_queue_oe;
reg [43:0] inst_queue_inp;
wire [43:0] inst_queue_oup;
wire [7:0] inst_queue_used_pos;
instruction_queue inst_queue(clk, rst, inst_queue_inp, inst_queue_oup, inst_queue_used_pos, inst_queue_we, inst_queue_oe);


// Handle decoding the instruction.
always_comb begin
    // If there are bytes to be decoded.
    if (queue_len > 0) begin
        // Check which instruction is present.
        case (queue_dout[0])
            // Add A, B
            8'b00000001:
            begin
                // If enough bytes are present.
                if (queue_len > 2) begin
                    // Setup the values ready for input.
                    inst_queue_inp[15:0] <= {queue_dout[0], queue_dout[1]};
                    // Enable input to the queue.
                    inst_queue_we <= 1;
                end
                else begin
                // Signal that more bytes are needed.
                end

            end
        endcase 
    end
end

endmodule


module decoder_input_register #(parameter INST_QUEUE_LEN = 64, INP_LEN = 4)(
    input           clk,
    input           rst,
    
    input           we,
    input           se,     // Shift enable
    input [7:0]     sa,     // Shift ammount

    input [(8*INP_LEN)-1:0]    data_in,
    output [7:0]    data_out [INST_QUEUE_LEN-1:0],
    output [7:0] length
);

// parameters.


reg [7:0] regs [INST_QUEUE_LEN-1:0];        // Stores the data.
reg [7:0] current_length;  // Stores how much data is stored.

assign data_out = regs;
assign length = current_length;

// Handle shifting.
integer i;
always @(posedge clk) begin

    if (se) begin
        for (i = 0; i < INST_QUEUE_LEN - sa; i = i + 1) begin
            regs[i] <= regs[i + sa];
        end

        for (i = INST_QUEUE_LEN - sa; i < INST_QUEUE_LEN; i = i + 1) begin
            regs[i] <= 0;
        end

        // If there is no input also being done on this clock cycle.
        if (we == 0) begin
            current_length <= current_length - sa;
        end
        
    end
end

// Handle input.
always @(posedge clk) begin
    if (we) begin
        case (se)
        0:
        begin
            // Proceeduralise this later. 
            regs[current_length+0] <= data_in[7:0];
            regs[current_length+1] <= data_in[15:8];
            regs[current_length+2] <= data_in[23:16];
            regs[current_length+3] <= data_in[31:24];

            current_length <= current_length + 4;
        end

        1:
        begin
            // Proceeduralise this later. 
            regs[current_length - sa +0] <= data_in[7:0];
            regs[current_length - sa +1] <= data_in[15:8];
            regs[current_length - sa +2] <= data_in[23:16];
            regs[current_length - sa +3] <= data_in[31:24];

            current_length <= current_length + 4 - sa;
        end
        endcase 

    end
end

// Handle reset.
always @(posedge rst) begin
    for (i = 0; i < INST_QUEUE_LEN; i = i + 1) begin
        regs[i] = 0;
    end

    current_length = 0;
end


endmodule



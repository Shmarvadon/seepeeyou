module instruction_queue(
    input           clk,
    input           rst,
    
    input [43:0]    d_in,
    output [43:0]   d_out,
    output [7:0]    used_pos,

    input           we,
    input           oe
);

reg [43:0] instructions [7:0];
reg [7:0] used_positions;

assign used_pos = used_positions;
assign d_out = instructions[0];


// handle input & output.
integer i;
always @(posedge clk) begin
    
    case ({oe, we})

        // Doing nothing.
        2'b00:
        begin
        end

        // Just writing new value.
        2'b01:
        begin
            if (used_positions < 8) begin
                // Write the new value & incriment the number of used positions.
                instructions[used_positions] <= d_in;
                used_positions <= used_positions + 1;
            end
        end

        // Just outputting.
        2'b10:
        begin
            if (used_positions != 0) begin
                // Shift the values.
                for (i = 0; i < 8; i = i + 1) begin
                    instructions[i] <= instructions[i + 1];
                end

                // Decriment the used_positions variable.
                used_positions <= used_positions - 1;
            end
        end

        // Outputting & Writing.
        2'b11:
        begin
            // If we have between 1 and 7 values stored.
            if (used_positions != 0 && used_positions < 8) begin
                // Shift existing values.  
                for (i = 0; i < used_positions-1; i = i + 1) begin
                    instructions[i] <= instructions[i + 1];
                end

                // Append new value.
                instructions[used_positions-1] <= d_in;
            end
            if (used_positions == 0) begin
                // Store the value.
                instructions[used_positions] <= d_in;

                // Incriment used_positions.
                used_positions <= used_positions + 1;
            end
        end
    endcase
end


// Handle reset.
always @(posedge rst) begin
    for (i = 0; i < 8; i = i + 1) begin
        instructions[i] = 0;
    end

    used_positions = 0;
end

endmodule
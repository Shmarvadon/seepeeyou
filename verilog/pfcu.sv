`include "structs.sv"

module program_flow_control_unit(
    input clk,
    input rst,
    input en,
    output bit done,

    input [47:0] inst,
    input [7:0] alu_status,
    output bit mdfy_pc,
    output bit [31:0] new_pc_val,

    inout ip_port noc_port

);

always_comb begin
    // If enabled.
    if (en) begin
        case(inst[7:0])
        // JMP k
        8'b00001110:
        begin

            $display("Executing PFCU JMP k instruction");

            // Present the new PC value.
            new_pc_val <= inst[47:8];

            // Signal that we are done.
            done <= 1;

            // Signal to modify pc.
            mdfy_pc <= 1;
        end

        // JIZ k
        8'b10001110:
        begin

            $display("Executing PFCU JIZ k instruction");

            // check if ALU status[0] is set or not.
            if (alu_status[0]) begin
                // Signal that we are going to modify the program counter.
                mdfy_pc <= 1;

                // Present addr of next instruction.
                new_pc_val <= inst[47:8];
            end
            // If alu status is not set.
            else begin
                // do not write a new value to PC.
                mdfy_pc <= 0;
            end

            // tell the rest of the core that we are done.
            done <= 1;
        end

        // GOT k
        8'b01001110:
        begin
        end

        // When no instruction is here for us.
        default:
        begin
            // Signal to not modify PC.
            mdfy_pc <= 0;

            // Signal not done.
            done <= 0;
        end
        endcase
    end
    else begin
        done <= 0;
        mdfy_pc <= 0;
    end
end

endmodule
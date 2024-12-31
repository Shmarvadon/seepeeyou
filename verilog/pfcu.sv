`include "structs.sv"

module program_flow_control_unit(
    input clk,
    input rst,
    input en,
    output bit done,

    input queued_instruction instruction,
    input [7:0] alu_status,
    output bit mdfy_pc,
    output bit [31:0] new_pc_val,

    output bit shp_we,
    output bit [31:0] shp_inp,
    input [31:0] shp_oup,

    inout ip_port noc_port

);

// NOC interface stuff.
packet tx_packet;
bit submit_tx;
bit complete_rx;
assign noc_port.dat_to_noc = tx_packet;
assign noc_port.tx_submit = submit_tx;
assign noc_port.rx_complete = complete_rx;

bit [3:0] mcics;    // Multi Cycle Instruction Current Stage.
always_comb begin
    // If enabled.
    if (en) begin
        case(instruction.inst[7:3])
        // JMP k
        5'b00001:
        begin

            $display("Executing PFCU JMP k instruction");

            // Present the new PC value.
            new_pc_val <= instruction.inst[47:8];

            // Signal that we are done.
            done <= 1;

            // Signal to modify pc.
            mdfy_pc <= 1;
        end

        // JIZ k
        5'b10001:
        begin

            $display("Executing PFCU JIZ k instruction");

            // check if ALU status[0] is set or not.
            if (alu_status[0]) begin
                // Signal that we are going to modify the program counter.
                mdfy_pc <= 1;

                // Present addr of next instruction.
                new_pc_val <= instruction.inst[47:8];
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
        5'b01001:
        begin
            $display("Executing a PFCU GOT k instruction");

            // Do the work but dont evaluate conditions for progressing to next stage.
            case(mcics) 
            // Submit the tx.
            0:
            begin
                // Submit the tx request.
                submit_tx <= 1;

                // signal done is 0.
                done <= 0;
                
                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 2;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= memory_write_request;
                tx_packet.dat[31:0] <= shp_oup-4; // Location to write to.
                tx_packet.dat[63:32] <= instruction.addr + instruction.len; // Address of next instruction.
                tx_packet.dat[127:64] <= 0;
            end

            // Wait for reply.
            1:
            begin
                // No longer need to submit tx so stop setting it.
                submit_tx <= 0;

                // signal done is 0.
                done <= 0;

                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_write_reply) begin

                    // We now need to adjust the stack head pointer.
                    shp_inp <= shp_oup - 4;
                    shp_we <= 1;

                    // signal rx complete.
                    complete_rx <= 1;

                    // We now also need to hot reset the instruction fetch unit & set new program counter value.
                    new_pc_val <= instruction.inst[43:8];
                    mdfy_pc <= 1;

                    //done <= 1;

                    $display("Modify PC signalled %t", $time);
                end
            end
            endcase
        end

        // RET
        5'b11000:
        begin
            $display("Executing a PPFCU RET instruction, %t", $time);

            // Set the outputs here but let the other block drive the state machine.
            case(mcics) 
            // Submit tx asking for location to jump to from stack.
            0:
            begin
                // construct the tx request.
                tx_packet.dst_addr <= 2;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= memory_read_request;
                tx_packet.dat[31:0] <= shp_oup;
                tx_packet.dat[127:32] <= 0;

                // submit the reply.
                submit_tx <= 1;

                // Indicate not done yet.
                done <= 0;
            end

            // Wait for reply.
            1:
            begin
                // stop asking to submit the tx.
                submit_tx <= 0;

                // If we have an rx waiting for us & its probably the right one.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin
                    // Change the stack head pointer value.
                    shp_inp <= shp_oup + 4;
                    shp_we <= 1;

                    // Signal to the noc that rx is complete.
                    complete_rx <= 1;

                    // Write new PC val to pc.
                    new_pc_val <= noc_port.dat_from_noc.dat[31:0];
                    mdfy_pc <= 1;

                    //done <= 1;

                    $display("Modify PC signalled %t", $time);
                end
            end
            endcase
        end

        // When there is an invalid instruction for us.
        default:
        begin
            // Signal to not modify PC.
            mdfy_pc <= 0;

            // Signal not done.
            done <= 0;

            // set tx submit to 0.
            submit_tx <= 0;
        end
        endcase
    end
end

always @(posedge clk) begin

    case (instruction.inst[7:3])
        // GOT k
        5'b01001:
        begin
            case (mcics)
            // Beginning.
            0:
            begin
                // If the port is open and the NOC replies tx_complete then we can move onto next stage.
                if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                    mcics <= mcics + 1;
                end
            end
            // Wait for reply to come back.
            1:
            begin
                // If the NOC signals that it has an rx for us.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_write_reply) begin
                    mcics <= 0;
                end
            end
            2: mcics <= 0;
            endcase
        end

        // RET
        5'b11000:
        begin
            case (mcics)
            // Beginning.
            0:
            begin
                if (noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                    mcics <= mcics + 1;
                end
            end
            // Wait for reply to come back.
            1:
            begin
                // If the NOC signals that it has a reply rx for us.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin
                    mcics <= 0;
                end
            end
            2: mcics <= 0;
            endcase
        end

        default:
        begin
            // reset the instruction stage.
            mcics <= 0;
        end
    endcase

end

endmodule
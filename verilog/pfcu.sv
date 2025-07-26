`include "structs.svh"

module program_flow_control(
    input   logic               clk,
    input   logic               rst,
    input   logic               en,
    output  logic               dn,

    input   queued_instruction  instr,
    input   logic [7:0]         alu_stat,
    output  logic               jmp,
    output  logic [31:0]        new_pc,

    output  logic               shp_we,
    output  logic [31:0]        shp_inp,
    input   logic [31:0]        shp_oup,

    // Interface to the memory subsystem.
    output  logic               mac_prt_tx_rp,
    output  line_acc_req        mac_prt_tx_req,
    input   logic               mac_prt_tx_ra,

    input   logic               mac_prt_rx_rp,
    input   line_acc_req        mac_prt_rx_req,
    output  logic               mac_prt_rx_ra
);

logic [31:0] curr_pc;
int byte_cnt;
logic [3:0] state;
always @(posedge clk) begin
    // Default values.
    jmp = 0;
    dn = 0;
    mac_prt_tx_rp = 0;
    mac_prt_tx_req = 0;
    mac_prt_rx_ra = 0;
    byte_cnt = 0;

    // If enabled.
    if (en) begin
        case (instr.bits[7:3])

        // JMP k
        5'b00001:
        begin
            $display("Executing PFCU JMP k instruction. %t", $time);

            // Present new pc, signal jump & done.
            new_pc <= instr.bits[47:8];
            dn <= 1;
            jmp <= 1;
        end

        // JIZ k
        5'b10001:
        begin
            $display("Executing PFCU JIZ k instruction. %t", $time);

            // Check if ALU status [0] is set (last alu result = 0).
            if (alu_stat[0]) begin
                // Present new pc and signal jump
                new_pc <= instr.bits[47:8];
                jmp <= 1;
            end
            // Signal done.
            dn <= 1;
        end

        // GOT k
        5'b01001:
        begin
            $display("Executing PFCU GOT k (%h) instruction. %t", instr.bits[47:8], $time);

            // prepare new stack head pointer value.
            shp_inp = shp_oup - 4;

            // Check if an aligned write can be done.
            if (shp_inp[31:4] == shp_oup[31:4]) begin
                
                // If the cmac accepts the request.
                if (mac_prt_tx_ra) begin
                    // Signal done.
                    dn <= 1;
                    // Signal jmp & set new pc.
                    jmp <= 1;
                    new_pc <= instr.bits[47:8];
                    // Signal to overwrite the stack head pointer.
                    shp_we <= 1;
                    // Signal that there is no write tx present to cmac.
                    mac_prt_tx_rp <= 0;
                end
                else begin
                    // Assign some write stuffs.
                    mac_prt_tx_req.addr <= {shp_inp[31:4], 0};
                    mac_prt_tx_req.rqt <= 1;

                    // Loop over the 16 bytes and set writemask.
                    for (int i = 0; i < 16; i = i + 1) begin
                        // If the byte is in range of the bytes to write, set wmsk.
                        if (i >= shp_inp[3:0] & i < shp_oup[3:0]) begin
                            mac_prt_tx_req.wmsk[i] <= 1;
                        end
                        else mac_prt_tx_req.wmsk[i] <= 0;
                    end
                    // Write the data to the data field.
                    mac_prt_tx_req.dat[shp_inp[3:0]+:32] <= instr.addr + instr.len;

                    // Signal request present to the mem acc cont.
                    mac_prt_tx_rp <= 1;
                end
            end
            // If an aligned write can not be performend.
            else begin
                case(state)
                // First line write (top).
                0:
                begin

                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Move to the next state.
                        state <= 1;
                    end
                    else begin
                        // Assign some write stuffs.
                        mac_prt_tx_req.addr <= {shp_oup[31:4], 0};
                        mac_prt_tx_req.rqt <= 1;

                        // Set current PC value.
                        curr_pc = instr.addr + instr.len;

                        // Loop over bottom 4 bytes and assign write mask.
                        for (int i = 0; i < 4; i = i + 1) begin
                            if (i < shp_oup[3:0]) begin
                                mac_prt_tx_req.wmsk[i] <= 1;
                                mac_prt_tx_req.dat[(8*i)+:8] <= curr_pc[8*(4-byte_cnt)+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // Second line write (bottom).
                1:
                begin

                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        // Signal done.
                        dn <= 1;
                        // Signal jmp & set new pc.
                        jmp <= 1;
                        new_pc <= instr.bits[47:8];
                        // Signal to overwrite the stack head pointer.
                        shp_we <= 1;
                        // Signal that there is no write tx present to cmac.
                        mac_prt_tx_rp <= 0;
                        // Reset state to 0.
                        state <= 0;
                    end
                    else begin
                        // Assign some write stuffs.
                        mac_prt_tx_req.addr <= {shp_inp[31:4], 0};
                        mac_prt_tx_req.rqt <= 1;

                        // Set current PC value.
                        curr_pc = instr.addr + instr.len;

                        // Loop over top 4 bytes and assign write mask.
                        for (int i = 0; i < 5; i = i + 1) begin
                            if (i + 11 >= shp_inp[3:0]) begin
                                mac_prt_tx_req.wmsk[11+i] <= 1;
                                mac_prt_tx_req.dat[(11 + i)*8 +:8] <= curr_pc[8*(byte_cnt)+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal request present to the mem acc cont.
                        mac_prt_tx_rp <= 1;
                    end
                end
                endcase 
            end
        end

        // RET
        5'b11000:
        begin
            $display ("Executing PFCU RET instruciton. %t", $time);

            // Prepare new stack head pointer value.
            shp_inp = shp_oup + 4;

            // Check if an aligned read can be done.
            if (shp_inp[31:4] == shp_oup[31:4]) begin

                case(state) 

                // State 0 (request to read the line).
                0:
                begin
                    // If the cmac accepts the request.
                    if (mac_prt_tx_ra) begin
                        state <= 1;
                    end
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_inp[31:4], 0};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for reply).
                1:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // Assign the data to the new_pc
                        new_pc <= mac_prt_rx_req.dat[(shp_oup[3:0] * 8)+:32];

                        // Signal jump, shp_we & done.
                        jmp <= 1;
                        dn <= 1;
                        shp_we <= 1;

                        // move state back to 0.
                        state <= 0;
                    end
                end
                endcase
            end
            // If an aligned request can not be made. 
            else begin

                case (state)

                // State 0 (read request to top line).
                0:
                begin
                    if (mac_prt_tx_ra) state <= 1;
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_inp[31:4], 0};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 1 (wait for reply for top line read).
                1:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // assign bytes to new pc val.
                        for (int i = 3; i >= 0; i = i - 1) begin
                            if (i < shp_inp[3:0]) begin
                                new_pc[(3-byte_cnt)*8 +:8] <= mac_prt_rx_req.dat[(i*8)+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal rx accepted.
                        mac_prt_rx_ra <= 1;

                        // Move to state 2.
                        state <= 2;
                    end
                end

                // State 2 (read request to the bottom line).
                2:
                begin
                    if (mac_prt_tx_ra) state <= 3;
                    else begin
                        // Assign some address stuffs.
                        mac_prt_tx_req.addr <= {shp_oup[31:0], 0};
                        mac_prt_tx_req.rqt <= 0;

                        // Tell the mem acc cont that the request is wanting to be sent.
                        mac_prt_tx_rp <= 1;
                    end
                end

                // State 3 (wait for reply for bottom line read).
                3:
                begin
                    // If the cmac replies with the read data.
                    if (mac_prt_rx_rp) begin
                        // assign bytes to new pc val.
                        for (int i = 0; i < 5; i = i + 1) begin
                            if (i + 11 >= shp_oup[3:0]) begin
                                new_pc[byte_cnt+:8] <= mac_prt_rx_req.dat[(i+11)*8+:8];
                                byte_cnt = byte_cnt + 1;
                            end
                        end

                        // Signal rx accepted to the cmac.
                        mac_prt_rx_ra <= 1;

                        // Signal done.
                        dn <= 1;
                        // Signal jump.
                        jmp <= 1;
                        // Signal shp_we.
                        shp_we <= 1;
                        // Reset state to 0.
                        state <= 0;
                    end
                end
                endcase
            end
        end
        endcase
    end
    else begin
        // Reset state.
        state <= 0;
    end
end

endmodule

/*
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

            // If the instruction is the infinite loop thing, stop.
            if (instruction.inst[39:0] == 40'b0000_0000_0000_0000_0000_0000_1011_1001_0000_1110) $stop;
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
                new_pc_val <= instruction.inst[8+:32];

                //if (instruction.inst[39:0] == 40'b0000_0000_0000_0000_0001_0000_0011_1100_1000_1110) $stop;
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
            $display("Executing a PFCU GOT %d instruction at %t", instruction.inst[8+:32], $time);

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


                    // This triggers when the matmul program is complete.
                    if (instruction.inst[39:0] == 40'b0000_0000_0000_0000_0001_0000_0001_0011_0100_1110) $stop;
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
*/
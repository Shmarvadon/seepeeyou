`include "structs.sv"
`include "defines.sv"

module memory_and_io_unit (
    input               clk,
    input               rst,
    input               en,
    output bit          done,

    input queued_instruction        instruction,
    input [31:0]    gpr_oup [15:0],
    output bit [31:0]   gpr_inp [15:0],
    output bit [15:0]   gpr_we,

    inout ip_port noc_port
);

// noc port stuff.
packet tx_packet;
bit submit_tx;
bit complete_rx;
assign noc_port.dat_to_noc = tx_packet;
assign noc_port.tx_submit = submit_tx;
assign noc_port.rx_complete = complete_rx;

// Process instruction.
bit [3:0] mcics;    // Multi Cycle Instruction Current Stage.
always_comb begin
    // default values.
    done <= 0;
    complete_rx <= 0;
    submit_tx <= 0;
    gpr_we <= 0;


    // If enabled.
    if (en) begin
        case(instruction.inst[7:3])
        // LOD k, A
        5'b00000:
        begin
            $display("Executing M&IO LOD k, A instruction, %t", $time);

            // Do the work but dont handle transitioning state.
            case(mcics)
            // Submit tx to read in data.
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
                tx_packet.pt <= memory_read_request;
                tx_packet.dat[31:0] <= instruction.inst[47:16]; // Location to read from.
                tx_packet.dat[127:32] <= 0;
            end

            // Wait for reply.
            1:
            begin
                // Stop asking to submit a tx.
                submit_tx <= 0;

                // If we have an rx waiting for us and its a memory read reply.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin

                    // Signal that we have completed the rx.
                    complete_rx <= 1;

                    //prepare to write the value to gprs.
                    gpr_inp[instruction.inst[11:8]] <= noc_port.dat_from_noc.dat[31:0];
                    gpr_we[instruction.inst[11:8]] <= 1;

                    // Signal done.
                    done <= 1;
                end
            end
            endcase
        end

        // STR k, A
        5'b10000:
        begin

            $display("Executing M&IO STR k, A instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 2;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= memory_write_request;
                tx_packet.dat[31:0] <= instruction.inst[47:16]; // Location to write to.
                tx_packet.dat[63:32] <= gpr_oup[instruction.inst[11:8]]; // load in the data from GPR that is to be written to memory.
                tx_packet.dat[127:64] <= 0;
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_write_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;
                end
            end
            endcase
        end

        // GIO k, I
        5'b00010:
        begin

            $display("Executing M&IO GIO k, I instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 3;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= gpio_set_val_request;
                tx_packet.dat[11:0] <= instruction.inst[19:8]; // Data to send to GPIO block.
                tx_packet.dat[127:12] <= 0; // Zero the rest of the transaction.
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;
                end
            end
            endcase
        end

        // GIO A, I
        5'b00001:
        begin

            $display("Executing M&IO GIO A, I instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 3;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= gpio_set_val_request;
                tx_packet.dat[3:0] <= instruction.inst[11:8]; // write in the GPIO pin to be written to.
                tx_packet.dat[11:4] <= gpr_oup[instruction.inst[15:12]][7:0]; // Data to send to GPIO block.
                tx_packet.dat[127:12] <= 0; // Zero the rest of the transaction.
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;
                end
            end
            endcase
        end

        // GII I, A
        5'b10001:
        begin

            $display("Executing M&IO GII I, A instruction, %t", $time);

            // Do the work here but dont handle transitioning state.
            case (mcics) 
            // submit tx to write data to memory.
            0:
            begin
                // Signal that the tx has been submitted.
                submit_tx <= 1;

                // Construct packet to send over NOC.
                tx_packet.dst_addr <= 3;
                tx_packet.dst_prt <= 0;
                tx_packet.src_addr <= noc_port.port_address;
                tx_packet.src_prt <= noc_port.port_number;
                tx_packet.id <= 0;
                tx_packet.pt <= gpio_read_val_request;
                tx_packet.dat[3:0] <= instruction.inst[11:8]; // write in the GPIO pin to be read from.
                tx_packet.dat[127:4] <= 0; // Zero the rest of the transaction.
            end
            // Wait for reply to confirm success of write.
            1:
            begin
                // If there is a reply waiting for us & its the one we want.
                if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                    // signal that the rx has been recieved.
                    complete_rx <= 1;

                    // Signal that we are done.
                    done <= 1;

                    // Write back the result to gprs.
                    gpr_inp[instruction.inst[15:12]][7:0] <= noc_port.dat_from_noc.dat[7:0];
                    gpr_inp[instruction.inst[15:12]][31:8] <= 0;

                    gpr_we[instruction.inst[15:12]] <= 1;
                end
            end
            endcase
        end
        endcase
    end
end

always @(posedge clk) begin
    case(instruction.inst[7:3])
    // LOD k, A
    5'b00000:
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
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == memory_read_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // STR k, A
    5'b10000:
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
        endcase
    end

    // GIO k, I
    5'b00010:
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
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // GIO A, I
    5'b00001:
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
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_set_val_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end

    // GII I, A
    5'b10001:
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
            if (noc_port.rx_recieve && noc_port.dat_from_noc.pt == gpio_read_val_reply) begin
                mcics <= 0;
            end
        end
        endcase
    end
    endcase
end

endmodule

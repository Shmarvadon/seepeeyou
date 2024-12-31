`include "structs.sv"

/*          Main module         */
module instruction_decoder #(parameter INST_QUEUE_LEN = 64, INP_LEN = 16)(
    input clk,
    input rst_inp,

    input [31:0] pc,
    input       mdfy_pc,

    output queued_instruction iq_oup,
    output inst_pres,
    input req_nxt_inst,

    inout ip_port noc_port
);
    bit rst;

    // Instruction decode buffer stuff.
    logic db_we;
    logic db_se;
    logic [7:0] db_sa;
    logic [(8*INP_LEN) -1:0] db_din;
    logic [7:0] db_dout [INST_QUEUE_LEN -1: 0];
    logic [7:0] db_len;
    // decode buffer base address register.
    reg [31:0] dbba;
    decoder_input_register #(INST_QUEUE_LEN, INP_LEN) inst_decode_queue(clk, rst, db_we, db_se, db_sa, db_din, db_dout, db_len);


    // Instruction queue stuff.
    bit iq_we;
    queued_instruction iq_inp;
    logic [$clog2(8)+1] iq_len;
    assign inst_pres = iq_len > 0 ? 1 : 0;  // If no more instructions then signal instruction present as 0.
    fifo_queue #($bits(queued_instruction), 8) inst_q(clk, rst, iq_inp, iq_oup, iq_len, iq_we, req_nxt_inst);


    // Instruction fetcher.
    instruction_fetcher_2 #(INST_QUEUE_LEN) inst_fetcher(clk, rst, db_len, dbba, noc_port, db_din, db_we);


    // Handle decoding the instruction.
    always_comb begin
        // If there are bytes to be decoded and there is space in the instruction queue.
        if (db_len > 0  && iq_len < 7) begin
            // Check which instruction is present.
            casez (db_dout[0])

                // ALU instruction.
                8'b?????100:
                begin

                    $display("ALU instruction decoded.");
                    $display("%b", db_dout[0]);
                    $display("%b", db_dout[0][3]);

                    // If instruction is an immediate.
                    if (db_dout[0][3] == 1) begin
                        // If there are enough bytes to decode the instruction.
                        if (db_len >= 6) begin
                            // Setup the values ready for input.
                            iq_inp.inst[15:0] <= {db_dout[1], db_dout[0]};
                            iq_inp.inst[31:16] <= {db_dout[3], db_dout[2]};
                            iq_inp.inst[47:32] <= {db_dout[5], db_dout[4]};
                            iq_inp.addr <= dbba;
                            iq_inp.len <= 6;
                            // Enable input to the queue.
                            iq_we <= 1;
                            // Tell the decode queue to shift out the 2 bytes.
                            db_sa <= 6;
                            db_se <= 1;
                        end
                        // If there are not enough bytes.
                        else begin
                            // Disable decode buffer shift enable (se).
                            db_sa <= 0;
                            db_se <= 0;

                            // Disable instruction queue input.
                            iq_we <= 0;
                        end
                    end

                    // If the instruction is not an immediate.
                    if (db_dout[0][3] == 0) begin
                        // If there are enough bytes to decode.
                        if (db_len >= 2) begin
                            // Setup the values ready for input.
                            iq_inp.inst[15:0] <= {db_dout[1], db_dout[0]};
                            iq_inp.inst[47:16] <= 0;
                            iq_inp.addr <= dbba;
                            iq_inp.len <= 2;
                            // Enable input to the queue.
                            iq_we <= 1;
                            // Tell the decode queue to shift out the 2 bytes.
                            db_sa <= 2;
                            db_se <= 1;
                        end
                        // If there are not enough bytes.
                        else begin
                            // Disable decode buffer shift enable (se).
                            db_sa <= 0;
                            db_se <= 0;

                            // Disable instruction queue input.
                            iq_we <= 0;
                        end
                    end
                end

                // Memory & IO instruction.
                8'b?????010:
                begin

                    $display("M&IO instruction decoded.");

                    // If instruction is  6 bytes.
                    if (db_dout[0][4:3] == 0) begin
                        // If there are enough bytes to decode the instruction.
                        if (db_len >= 6) begin
                            // Setup the values ready for input.
                            iq_inp.inst[15:0] <= {db_dout[1], db_dout[0]};
                            iq_inp.inst[31:16] <= {db_dout[3], db_dout[2]};
                            iq_inp.inst[47:32] <= {db_dout[5], db_dout[4]};
                            iq_inp.addr <= dbba;
                            iq_inp.len <= 6;
                            // Enable input to the queue.
                            iq_we <= 1;
                            // Tell the decode queue to shift out the 2 bytes.
                            db_sa <= 6;
                            db_se <= 1;
                        end
                        // If there are not enough bytes.
                        else begin
                            // Disable decode buffer shift enable (se).
                            db_sa <= 0;
                            db_se <= 0;

                            // Disable instruction queue input.
                            iq_we <= 0;
                        end
                    end

                    // If the instruction is 3 bytes.
                    if (db_dout[0][4:3] == 2) begin
                        // If there are enough bytes to decode.
                        if (db_len >= 3) begin
                            // Setup the values ready for input.
                            iq_inp.inst[15:0] <= {db_dout[1], db_dout[0]};
                            iq_inp.inst[31:16] <= {0, db_dout[2]};
                            iq_inp.inst[47:32] <= 0;
                            iq_inp.addr <= dbba;
                            iq_inp.len <= 3;
                            // Enable input to the queue.
                            iq_we <= 1;
                            // Tell the decode queue to shift out the 2 bytes.
                            db_sa <= 3;
                            db_se <= 1;
                        end
                        // If there are not enough bytes.
                        else begin
                            // Disable decode buffer shift enable (se).
                            db_sa <= 0;
                            db_se <= 0;

                            // Disable instruction queue input.
                            iq_we <= 0;
                        end
                    end

                    // If the instruction is 2 bytes
                    if (db_dout[0][4:3] == 1) begin
                        // If there are enough bytes to decode.
                        if (db_len >= 2) begin
                            // Setup the values ready for input.
                            iq_inp.inst[15:0] <= {db_dout[1], db_dout[0]};
                            iq_inp.inst[47:16] <= 0;
                            iq_inp.addr <= dbba;
                            iq_inp.len <= 2;
                            // Enable input to the queue.
                            iq_we <= 1;
                            // Tell the decode queue to shift out the 2 bytes.
                            db_sa <= 2;
                            db_se <= 1;
                        end
                        // If there are not enough bytes.
                        else begin
                            // Disable decode buffer shift enable (se).
                            db_sa <= 0;
                            db_se <= 0;

                            // Disable instruction queue input.
                            iq_we <= 0;
                        end
                    end
                end

                // PFCU instruction.
                8'b?????110:
                begin

                    $display("PFCU instruction decoded.");

                    // If instruction is 5 bytes.
                    if (db_dout[0][3]) begin
                        // If there are enough bytes to decode the instruction.
                        if (db_len >= 5) begin
                            // Setup the values ready for input.
                            iq_inp.inst[15:0] <= {db_dout[1], db_dout[0]};
                            iq_inp.inst[31:16] <= {db_dout[3], db_dout[2]};
                            iq_inp.inst[47:32] <= {0, db_dout[4]};
                            iq_inp.addr <= dbba;
                            iq_inp.len <= 5;
                            // Enable input to the queue.
                            iq_we <= 1;
                            // Tell the decode queue to shift out the 2 bytes.
                            db_sa <= 5;
                            db_se <= 1;
                        end
                        // If there are not enough bytes.
                        else begin
                            // Disable decode buffer shift enable (se).
                            db_sa <= 0;
                            db_se <= 0;

                            // Disable instruction queue input.
                            iq_we <= 0;
                        end
                    end

                    // If the instruction is 1 byte.
                    if (db_dout[0][3] == 0) begin
                        // If there are enough bytes to decode.
                        if (db_len >= 1) begin
                            // Setup the values ready for input.
                            iq_inp.inst[15:0] <= {0, db_dout[0]};
                            iq_inp.inst[47:16] <= 0;
                            iq_inp.addr <= dbba;
                            iq_inp.len <= 1;
                            // Enable input to the queue.
                            iq_we <= 1;
                            // Tell the decode queue to shift out the 2 bytes.
                            db_sa <= 1;
                            db_se <= 1;
                        end
                        // If there are not enough bytes.
                        else begin
                            // Disable decode buffer shift enable (se).
                            db_sa <= 0;
                            db_se <= 0;

                            // Disable instruction queue input.
                            iq_we <= 0;
                        end
                    end
                end

                // This condition runs when bytes in the db are invalid.
                default:
                begin
                    // Stop the queue from shifting out bytes.
                    db_sa <= 0;
                    db_se <= 0;

                    // Stop the instruction queue from shifting in instructions.
                    iq_we <= 0;

                    // Write all 0s to iq_inp
                    iq_inp <= 0;
                end
            endcase 
        end
        else begin  // If the queue is full or if not enough bytes to decode.
            // Disable shift into the instruction queue.
            iq_we <= 0;

            // Disable shift out from the decode queue.
            db_se <= 0;
            db_sa <= 0;

            // Write all 0s to iq_inp
            iq_inp <= 0;
        end
    end

    // Handle modification of decode buffer base address counter.
    logic ht_rst;
    always @(posedge clk) begin
        // Incriment the dbba by however many bytes are shifted out.
        dbba <= dbba + db_sa;

        if (mdfy_pc) begin
            ht_rst <= 1;
        end

        if (ht_rst) ht_rst <= 0;
    end
    // Handle driving module reset from either ht_rst or rst_inp.
    always_comb begin
        rst <= ht_rst | rst_inp;
    end

    // handle an incoming reset.
    always @(posedge rst) begin
        // Set decode buffer base counter.
        dbba <= pc;

        // Rest of reset action is managed implicitly by other submodules and always_comb block.
    end

endmodule

/*          Decoder buffer          */
module decoder_input_register #(parameter INST_QUEUE_LEN = 64, INP_LEN = 16)(
    input           clk,
    input           rst,
    
    input           we,
    input           se,     // Shift enable
    input [7:0]     sa,     // Shift ammount

    input [(8*INP_LEN)-1:0]    data_in,
    output [7:0]    data_out [INST_QUEUE_LEN-1:0],
    output [7:0] length
);


    reg [7:0] regs [INST_QUEUE_LEN-1:0];        // Stores the data.
    reg [7:0] current_length;  // Stores how much data is stored.

    assign data_out = regs;
    assign length = current_length;

    // Handle input.
    integer i;
    always @(posedge clk) begin
        case ({we, se}) 

        // Do nothing.
        2'b00:
        begin
        end

        // Shift enable.
        2'b01:
        begin
            // Shift data.
            for (i = 0; i < INST_QUEUE_LEN - sa; i = i + 1) begin
                regs[i] <= regs[i + sa];
            end

            // Zero the left behind data.
            for (i = INST_QUEUE_LEN - sa; i < INST_QUEUE_LEN; i = i + 1) begin
                regs[i] <= 0;
            end

            // Update the current length of the decode buffer.
            current_length <= current_length - sa;
        end

        // Write enable.
        2'b10:
        begin
            // write in 128 bits of data.

            // First 64 bits.
            regs[current_length+0] <= data_in[7:0];
            regs[current_length+1] <= data_in[15:8];
            regs[current_length+2] <= data_in[23:16];
            regs[current_length+3] <= data_in[31:24];
            regs[current_length+4] <= data_in[39:32];
            regs[current_length+5] <= data_in[47:40];
            regs[current_length+6] <= data_in[55:48];
            regs[current_length+7] <= data_in[63:56];
            // Second 64 bits.
            regs[current_length+8] <= data_in[71:64];
            regs[current_length+9] <= data_in[79:72];
            regs[current_length+10] <= data_in[87:80];
            regs[current_length+11] <= data_in[95:88];
            regs[current_length+12] <= data_in[103:96];
            regs[current_length+13] <= data_in[111:104];
            regs[current_length+14] <= data_in[119:112];
            regs[current_length+15] <= data_in[127:120];

            // Update the current length of the buffer.
            current_length <= current_length + 16;
        end

        // Shift & write enable.
        2'b11:
        begin
            // Shift existing data.
            for (i = 0; i < INST_QUEUE_LEN - sa; i = i + 1) begin
                regs[i] <= regs[i + sa];
            end

            // Write in new data.

            // First 64 bits.
            regs[current_length-sa+0] <= data_in[7:0];
            regs[current_length-sa+1] <= data_in[15:8];
            regs[current_length-sa+2] <= data_in[23:16];
            regs[current_length-sa+3] <= data_in[31:24];
            regs[current_length-sa+4] <= data_in[39:32];
            regs[current_length-sa+5] <= data_in[47:40];
            regs[current_length-sa+6] <= data_in[55:48];
            regs[current_length-sa+7] <= data_in[63:56];
            // Second 64 bits.
            regs[current_length-sa+8] <= data_in[71:64];
            regs[current_length-sa+9] <= data_in[79:72];
            regs[current_length-sa+10] <= data_in[87:80];
            regs[current_length-sa+11] <= data_in[95:88];
            regs[current_length-sa+12] <= data_in[103:96];
            regs[current_length-sa+13] <= data_in[111:104];
            regs[current_length-sa+14] <= data_in[119:112];
            regs[current_length-sa+15] <= data_in[127:120];

            current_length <= current_length + 16 - sa;
        end
        endcase
    end

    // Handle reset.
    always @(posedge rst) begin
        for (i = 0; i < INST_QUEUE_LEN; i = i + 1) begin
            regs[i] = 0;
        end

        current_length <= 0;
    end
endmodule

// Can delete, no longer in use.
/*          Instruction queue           */
module instruction_queue(
    input           clk,
    input           rst,
    
    input [47:0]    d_in,
    output [47:0]   d_out,
    output [7:0]    used_pos,

    input           we,
    input           oe
);

    reg [47:0] instructions [7:0];
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


/*          Instruction fetcher         */
module instruction_fetcher_2 #(parameter DB_SIZE = 64)(
    input clk,
    input rst,
    input [$clog2(DB_SIZE)+1:0] db_len,
    input [31:0] dbba,

    inout ip_port noc_port,

    output bit [127:0] inst_bytes,
    output bit db_we
);

    // Counter to hold the id of tx.
    bit [7:0] mem_req_id;
    bit inc_req_id;
    always @(posedge clk) begin
        if (inc_req_id) begin
            if (mem_req_id < 255) mem_req_id <= mem_req_id + 1;
            else mem_req_id <= 0;
        end
    end

    // fifo queue to log sent requests in.
    packet sr_inp;
    packet sr_oup;
    bit [$clog2(8)+1] sr_len;
    bit sr_we;
    bit sr_se;
    fifo_queue #($bits(packet), 8) sent_reqs(clk, rst, sr_inp, sr_oup, sr_len, sr_we, sr_se);

    // single input parallel output buffer to hold rx.
    bit rxb_we;     // rx buffer write enable.
    packet rxb_oup [7:0];
    bit [7:0] rxb_cl;   // rx buffer clear line.
    bit [7:0] rxb_ol;   // rx buffer occupied lines.
    sipo_buffer #($bits(packet), 8) rx_buff(clk, rst, rxb_we, noc_port.dat_from_noc, rxb_cl, rxb_oup, rxb_ol);

    // noc port control stuff.
    bit tx_submit;
    bit rx_complete;
    assign noc_port.dat_to_noc = sr_inp;
    assign noc_port.tx_submit = tx_submit;
    assign noc_port.rx_complete = rx_complete;

    // Handle sending requests to read instruction bytes.
    always_comb begin
        // If decode buffer length is 16 or more bytes less than maximum, there is room in the reqs sent q and the port is open.
        if (DB_SIZE - (db_len + (16 * sr_len)) >= 16 && sr_len <8 && noc_port.to_noc_prt_stat == port_open) begin
            // Have a request ready to go constantly.
            sr_inp.dst_addr <= 2;
            sr_inp.dst_prt <= 0;
            sr_inp.id <= mem_req_id; // Temporary thing, should make a graycode counter to allow assignment of packet ids.
            sr_inp.pt <= memory_read_request;
            sr_inp.dat <= dbba + db_len + (16 * sr_len);
            sr_inp.src_addr <= noc_port.port_address;
            sr_inp.src_prt <= noc_port.port_number;

            // Hook up the acceptance of the tx into noc stop trigger shifting of request into sent reqs queue.
            // This means that only once the noc stop signals it accepts the tx does it get shifted into our local memory.
            sr_we <= noc_port.tx_complete;

            // Signal that we want to send a tx.
            tx_submit <= 1;

            // Signal that we want to incriment the id.
            inc_req_id <= 1;
        end
        // If conditions to request more instruction bytes are not true.
        else begin
            // Signal that we dont want to submit a tx.
            tx_submit <= 0;

            sr_we <= 0;

            // Signal that we dont want to incriment the id.
            inc_req_id <= 0;
        end
    end

    // Handle recieveing rx requests from noc
    always_comb begin
        // If the noc port is sending is an rx.
        if (noc_port.rx_recieve) begin
            // If the buffer is full.
            if (rxb_ol == 8'b11111111) begin
                // Signal to the recieve buffer that we have nothing to write.
                rxb_we <= 0;

                // Signal to the NOC that we have not read its rx.
                rx_complete <= 0;
            end
            // If the recieve buffer is not full.
            else begin
                // Signal to recieve buffer that we want to write to it.
                rxb_we <= 1;

                // Signal to the NOC that we have recieved its rx.
                rx_complete <= 1;
            end
        end
        // If the NOC port is not sending an rx.
        else begin

            // Signal nothing to write to rx buff.
            rxb_we <= 0;

            // Signal to NOC that nothing recieved.
            rx_complete <= 0;
        end
    end

    // Handle appending the correct reply from noc to the decode buffer.
    bit [3:0] selected_rx;
    integer i;
    always_comb begin
        // If the rx buffer isnt empty.
        if (rxb_ol != 0) begin
            // Loop over the rx packets.
            for (i = 0; i < 8; i = i + 1) begin
                // If the packets id matches the earliest sent mem request (the one that is properly aligned with decode buffer).
                if (rxb_oup[i].id == sr_oup.id) begin
                    // Select the rx as being the valid one to append to decode buffer.
                    selected_rx <= i;
                end
            end

            // If the selected rx is actually present.
            if (rxb_ol[selected_rx]) begin
                // Signal instruction bytes ready.
                db_we <= 1;

                // Present the bytes.
                inst_bytes <= rxb_oup[selected_rx].dat;

                // Signal to clear this line in the rx buffer.
                rxb_cl[selected_rx] <= 1;

                // signal the sent request queue to shift one.
                sr_se <= 1;
            end
            // If the selected line is not carrying a valid instruction.
            else begin
                // Signal that we have nothing to write to decode buffer.
                db_we <= 0;

                // Signal that we have nothing to clear from the rx buffer.
                rxb_cl[selected_rx] <= 0;

                // Signal the sent requests queue to not shift
                sr_se <= 0;
            end
        end
        // If the rxb is empty.
        else begin
            // Signal that we have no instruction bytes to return.
            db_we <= 0;

            // Signal that we also have nothing to clear from rx buffer as its empty.
            rxb_cl <= 0;

            // Signal the sent requests queue to not shift.
            sr_se <= 0;
        end
    end

endmodule


/*          Instruction fetcher         */
module instruction_fetcher #(parameter DB_LEN = 64)(
    input clk,
    input rst,
    input [7:0] db_len,
    input [31:0] dbba,

    inout ip_port noc_port,

    output [127:0] inst_bytes_oup,
    output logic inst_bytes_ready
);
    // The job of this module is to create transactions to submit to the NOC to request instruction bytes.

    logic [127:0] inst_bytes;
    assign inst_bytes_oup = inst_bytes;

    // states for this thing.
    typedef enum bit[1:0] {free, waiting_for_reply} current_state;
    current_state status;

    // noc port connections.
    logic submit_tx;
    assign noc_port.tx_submit = submit_tx;

    packet dat_packet;
    assign noc_port.dat_to_noc = dat_packet;

    logic rx_complete;
    assign noc_port.rx_complete = rx_complete;

    // Keep track of ids.
    bit [7:0] pkt_id;


    always_comb begin
        case (status) 
        // Case where we are free to make a request to the NOC.
        free:
        begin

            // Signal that we are not ready to recieve anything yet.
            rx_complete <= 0;

            // Check if we need to make a request.
            if (db_len < DB_LEN - 16) begin
                // Check if we are able to make a request.
                if (noc_port.to_noc_prt_stat == port_open) begin
                    // Make request for more instruction bytes.
                    dat_packet.pt <= memory_read_request;
                    dat_packet.id <= pkt_id;
                    dat_packet.dat <= dbba + db_len;
                    dat_packet.src_prt <= 0;
                    dat_packet.src_addr <= 0;
                    dat_packet.dst_prt <= 0;
                    dat_packet.dst_addr <= 2;

                    // Signal that we are going to make the request to the NOC.
                    submit_tx <= 1;
                end
                // If we can not make a request.
                else begin
                    // Signal that we are not making a request to the NOC.
                    submit_tx <= 0;
                end
            end
            // If we dont need to make a request.
            else begin
                // Signal that there is no tx to submit.
                submit_tx <= 0;
            end

            // Signal that no instruction bytes are ready.
            inst_bytes_ready <= 0;
        end

        // Case where a request has already been made and we are waiting for a reply.
        waiting_for_reply:
        begin
            // If there is a reply waiting for us.
            if (noc_port.from_noc_prt_stat == port_open && noc_port.rx_recieve) begin

                // Check if its the packet we want.
                if (noc_port.dat_from_noc.id == pkt_id) begin
                    // Signal that there are instruction bytes waiting for the decode buffer.
                    inst_bytes_ready <= 1;

                    // Unpack the transmission.
                    inst_bytes <= noc_port.dat_from_noc.dat;
                end

                // Signal to the noc that we have recieved the rx.
                rx_complete <= 1;
            end
            // If there is no reply waiting for us.
            else begin
                // Signal that there are no instruction bytes ready yet.
                inst_bytes_ready <= 0;

                // Signal that rx is not complete.
                rx_complete <= 0;
            end
            
            // Signal that we no longer intend to submit a tx to noc.
            submit_tx <= 0;
        end
        endcase
    end

    // Handle state transitions.
    always @(posedge clk) begin
        case (status) 
        // If status is free.
        free:
        begin
            // If more bytes are needed and a request can be made.
            if (db_len < DB_LEN - 16  && noc_port.to_noc_prt_stat == port_open && noc_port.tx_complete) begin
                // Change status to waiting for reply. This stops the always_comb block above from doing stuff.
                status <= waiting_for_reply;
            end
        end

        // If status is waiting_for_reply.
        waiting_for_reply:
        begin
            // We wait for the NOC to signal that a reply is ready.
            if (noc_port.from_noc_prt_stat == port_open && noc_port.rx_recieve) begin
                // If we have recieved the packet we want.
                if (noc_port.dat_from_noc.id == pkt_id) begin
                    status <= free;

                    // Incriment pkt_id.
                    if (pkt_id <= 128) pkt_id <= pkt_id + 1;
                    else pkt_id <= 0;
                end
                // If not then we just keep waiting.
            end
        end
        endcase
    end

    // Handle reset.
    always @(posedge rst) begin
        // Set status to free.
        status <= free;
        pkt_id <= 0;
    end

endmodule
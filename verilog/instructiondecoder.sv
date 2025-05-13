`include "structs.svh"

module instruction_front_end #(parameter IQ_LEN = 8, IDB_LEN = 64)(
    input logic clk,
    input logic rst,

    // Interfaces to the back side of CPU core.

    input logic [31:0] pc_inp,
    input logic        jmp,

    output queued_instruction   iq_oup,
    output logic                iq_ip,
    input  logic                iq_pop,

    // Interface to the memory subsystem.
    output logic                mac_prt_tx_rp,
    output line_acc_req         mac_prt_tx_req,
    input logic                 mac_prt_tx_op,

    input logic                 mac_prt_rx_rp,
    input line_acc_req          mac_prt_rx_req,
    output logic                mac_prt_rx_op
);
    // Localparams (stuff to be tweaked globally not per instantiation).
    localparam MAX_INST_LEN = 8;

    // Reset signals.
    logic idb_rst;
    logic ibf_rst;
    logic iq_rst;

    // instruction bytes buffer base address.
    logic [31:0]        idbba; // Represents the base addrress of the instruction bytes buffer.

    // Instruction bytes buffer.
    logic [15:0]                            idb_push;
    logic [15:0][7:0]                       idb_inp;
    logic [$clog2(IDB_LEN):0]               idb_src_num_avail;

    logic [MAX_INST_LEN-1:0]                idb_pop;
    logic [MAX_INST_LEN-1:0][7:0]           idb_oup;
    logic [$clog2(IDB_LEN):0]               idb_dst_num_avail;

    fifo_sr #(8, IDB_LEN, 16, MAX_INST_LEN) idb(clk, idb_rst, idb_push, idb_inp, idb_src_num_avail, idb_pop, idb_oup, idb_dst_num_avail);

    // Instruction bytes fetcher.
    instruction_fetch #(IDB_LEN, 16) ibf(clk, ibf_rst, mac_prt_tx_rp, mac_prt_tx_req, mac_prt_tx_op, mac_prt_rx_rp, mac_prt_rx_req, mac_prt_rx_op, idb_src_num_avail, idb_dst_num_avail, idbba, idb_inp, idb_push);

    // Instruction decoder.
    logic [$bits(queued_instruction)-1:0]       decoded_inst;   // Decoded instruction.
    logic                                       inst_decoded;   // Instruction decoded.
    logic                                       inst_q_full;    // Instruction queue full.

    instruction_decoder #(MAX_INST_LEN) inst_dec(clk, rst, idb_oup, idb_dst_num_avail[$clog2(MAX_INST_LEN):0], idb_pop, decoded_inst, inst_decoded, inst_q_full, idbba);


    // Instruction Queue.
    logic [$clog2(IQ_LEN):0]                iq_src_num_avail;
    logic [$clog2(IQ_LEN):0]                iq_dst_num_avail;

    assign inst_q_full = (iq_src_num_avail == 0) ? 1 : 0;
    assign iq_ip = (iq_dst_num_avail != 0) ? 1 : 0;

    fifo_sr #($bits(queued_instruction), IQ_LEN, 1, 1) inst_q(clk, iq_rst, inst_decoded, decoded_inst, iq_src_num_avail, iq_pop, iq_oup, iq_dst_num_avail);

    // Handle top module level clocked logic.
    always @(posedge clk) begin
        // Update idbba when bytes are popped.
        for (int i = 0; i < MAX_INST_LEN; i = i + 1 ) begin
            if (idb_pop[i]) idbba = idbba + 1;
        end


        // Handle jump being signalled.
        if (jmp) begin
            idbba <= pc_inp;
            idb_rst <= 1;
            ibf_rst <= 1;
            iq_rst <= 1;
        end
        
    end

endmodule

// Very simple for now, focusing on measuring matmul perf with cache before moving to ucode based decode.
module instruction_decoder #(parameter IB_INP_LEN = 8)(
    input logic                                     clk,
    input logic                                     rst,

    // Connections to the instruction byte buffer.
    input logic [IB_INP_LEN-1:0][7:0]               ib,       // Instruction bytes.
    input logic [$clog2(IB_INP_LEN):0]              iba,      // Instruction bytes available.
    output logic [IB_INP_LEN-1:0]                   ibp,      // Pop Instruction bytes.

    // Connections to the instruction queue.
    output logic [$bits(queued_instruction)-1:0]    dio,      // Decoded instruction output.
    output logic                                    dip,      // Decoded instruction present.
    input logic                                     iqf,      // Decoded instruction queue full.

    // Other connections to instruction front end module.
    input logic [31:0]                              idbba     // Instruction decode buffer base address.
);

    queued_instruction decoded_instruction;
    assign dio = decoded_instruction;

    // Decode the instruction.
    always_comb begin
        // Default values.
        ibp = 0;
        decoded_instruction = 0;
        dip = 0;

        // Check which functional unit the inst is for.
        case(ib[0][2:0])
        // ALU instruction
        3'b100:
        begin
            $display("Decoding ALU instruction at %h", idbba);

            // If the instruction is an immediate & enough bytes are present to fully decode & instruction queue isnt full.
            if (ib[0][3] & iba >= 6 & !iqf) begin
                // Send on over the instruction bytes & signal a pop for next clk from idb.
                for (int i = 0; i < 6; i = i + 1) begin
                    decoded_instruction.inst[(i * 8)+:8] = ib[i];
                    ibp[i] = 1;
                end

                // Assign address of instruction.
                decoded_instruction.addr = idbba;
                decoded_instruction.len = 6;

                // Signal to push the instruction to the queue.
                dip = 1;
            end
            else if (!ib[0][3] & iba >= 2 & !iqf) begin
                // Send over instruction bytes & signal pop for next clk from idb.
                decoded_instruction.inst = {ib[1], ib[0]};
                ibp[1:0] = 2'b11;

                // Assign address of instruction.
                decoded_instruction.addr = idbba;
                decoded_instruction.len = 2;

                // Signal to push the instruction to the queue.
                dip = 1;
            end
            // If the instruction is 
        end

        // Memory & IO instruction
        3'b010:
        begin
            $diplay("Decoding Memory & IO instruction at %h", idbba);

            // If the instruction is an immediate.
            if (!ib[0][3] & !iqf) begin
                // If the instruction is lod / str
                if (!ib[0][4] & iba >= 6) begin
                     // Send on over the instruction bytes & signal a pop for next clk from idb.
                    for (int i = 0; i < 6; i = i + 1) begin
                        decoded_instruction.inst[(i * 8)+:8] = ib[i];
                        ibp[i] = 1;
                    end

                    // Assign address of instruction.
                    decoded_instruction.addr = idbba;
                    decoded_instruction.len = 6;

                    // Signal to push the instruction to the queue.
                    dip = 1;
                end

                // If the instruction is GIO.
                if (ib[0][4] & iba >= 3) begin
                    // Send on over the instruction bytes & signal a pop for next clk from idb.
                    for (int i = 0; i < 3; i = i + 1) begin
                        decoded_instruction.inst[(i * 8)+:8] = ib[i];
                        ibp[i] = 1;
                    end

                    // Assign address of instruction.
                    decoded_instruction.addr = idbba;
                    decoded_instruction.len = 3;

                    // Signal to push the instruction to the queue.
                    dip = 1;
                end
            end
            else if (ib[0][3] & !iqf) begin
                // Send over instruction bytes & signal pop for next clk from idb.
                decoded_instruction.inst = {ib[1], ib[0]};
                ibp[1:0] = 2'b11;

                // Assign address of instruction.
                decoded_instruction.addr = idbba;
                decoded_instruction.len = 2;

                // Signal to push the instruction to the queue.
                dip = 1;
            end
        end

        // Program flow control instruction
        3'b110:
        begin
            $display("Decoding Program Flow Control instruction at %h", dbba);

            // If the instruction is an immediate.
            if (ib[0][3] & !iqf & iba >= 5) begin
                // Send on over the instruction bytes & signal a pop for next clk from idb.
                for (int i = 0; i < 5; i = i + 1) begin
                    decoded_instruction.inst[(i * 8)+:8] = ib[i];
                    ibp[i] = 1;
                end

                // Assign address of instruction.
                decoded_instruction.addr = idbba;
                decoded_instruction.len = 5;

                // Signal to push the instruction to the queue.
                dip = 1;
            end
            else if (!ib[0][3] & !iqf & iba >= 1) begin
                // Send over instruction bytes & signal pop for next clk from idb.
                decoded_instruction.inst = ib[0];
                ibp[0] = 1;

                // Assign address of instruction.
                decoded_instruction.addr = idbba;
                decoded_instruction.len = 1;

                // Signal to push the instruction to the queue.
                dip = 1;
            end
        end
        endcase
    end
endmodule

module instruction_fetch #(parameter IDB_SIZE = 64, IDB_HEAD_COUNT = 16)(
    input logic clk,
    input logic rst,

    // Interface to the memory subsystem.
    output logic                mac_prt_tx_rp,
    output line_acc_req         mac_prt_tx_req,
    input logic                 mac_prt_tx_op,

    input logic                 mac_prt_rx_rp,
    input line_acc_req          mac_prt_rx_req,
    output logic                mac_prt_rx_op,

    // Interface to the instruction decode bytes buffer.
    input logic [$clog2(IDB_SIZE):0]            idb_src_num_avail,  // How many bytes can be pushed to at the moment.
    input logic [$clog2(IDB_SIZE):0]            idb_dst_num_avail,  // Current length of the shift reg.
    input logic [31:0]                          idbba,              // decode buffer base address.
    output logic [IDB_HEAD_COUNT-1:0][7:0]      idb_inp,            // Instruction decode buffer input heads.
    output logic [IDB_HEAD_COUNT-1:0]           idb_push            // Instruction decode buffer push.
);
    // local parameters.
    localparam IFQ_LEN = 8;
    localparam MRR_LEN = 8;

    typedef struct packed {
        bit [15:0] rmsk;
        bit [31:0] addr;
    } ifmrr;    // Instruction fetch memory read request.

    typedef struct packed {
        bit [31:0] addr;
        bit [127:0] dat;
    } mrrr;     // Memory read request reply.

    // instruction fetch memory read request queue.
    logic                       ifq_push;
    ifmrr                       ifq_dinp;
    logic [$clog2(IFQ_LEN):0]   ifq_src_num_avail;
    logic [IFQ_LEN-1:0]         ifq_pop;
    ifmrr [IFQ_LEN-1:0]         ifq_oup;
    logic [$clog2(IFQ_LEN):0]   ifq_dst_num_avail;

    fifo_sr #($bits(ifmrr), IFQ_LEN, 1, IFQ_LEN) ifq(clk, rst, ifq_push, ifq_dinp, ifq_src_num_avail, ifq_pop, ifq_oup, ifq_dst_num_avail);

    // memory read replies from mem subsys sipo.
    logic                           mrr_we;
    mrrr                            mrr_inp;
    logic [MRR_LEN-1:0]             mrr_clr_ps;
    mrrr                            mrr_oup [MRR_LEN-1:0];
    logic [MRR_LEN-1:0]             mrr_used_pos;

    sipo_buffer #($bits(mrrr), MRR_LEN) mrr(clk, rst, mrr_we, mrr_inp, mrr_clr_ps, mrr_oup, mrr_used_pos);

    // Logic to request more instruction bytes when the idb is not full.
    logic [31:0] current_fetch_addr;
    always_comb begin
        // default values.
        ifq_push = 0;
        ifq_dinp = 0;
        current_fetch_addr = 0;
        mac_prt_tx_rp = 0;

        // Count up the number of bytes being fetched currently.
        // loop over all pop ports from sr.
        for (int i = 0; i < IFQ_LEN; i = i + 1) begin
            // Loop over all the bytes in rmsk.
            for (int j = 0; j < 16; j = j + 1) begin
                if (i < ifq_dst_num_avail & ifq_oup[i].rmsk[j]) current_fetch_addr = current_fetch_addr + 1;
            end
        end
        current_fetch_addr = current_fetch_addr + idbba + idb_dst_num_avail;

        if (idb_src_num_avail >= 16) begin
            // Setup the ifq input.
            for (int i = 15; i >= 0; i = i - 1) begin
                if (i >= current_fetch_addr[3:0]) ifq_dinp.rmsk[i] = 1;
                else ifq_dinp.rmsk[i] = 0;
            end
            ifq_dinp.addr = {current_fetch_addr[31:4], 0};

            // Setup tx to the mem subsys.
            mac_prt_tx_req.addr = ifq_dinp.addr;
            mac_prt_tx_req.rqt = 0;

            // Signal that the reqest is present.
            mac_prt_tx_rp = 1;

            // If the request is accepted, the ifq needs to be pushed.
            ifq_push = mac_prt_tx_op;
        end
    end

    // Logic to accept read replies from memory subsystem.
    always_comb begin
        // Default values.
        mac_prt_rx_op = 0;
        mrr_we = 0;

        // Set input to MRR.
        mrr_inp.addr = mac_prt_rx_req.addr;
        mrr_inp.dat = mac_prt_rx_req.dat;

        // If the mrr is not full.
        if (!(&mrr_used_pos)) begin
            // Check that the reply is actually one we are expecting.
            for (int i = 0; i < IFQ_LEN; i = i + 1) begin
                // If the reply matches an in flight request.
                if (mac_prt_rx_req.addr == ifq_oup[i].addr) begin
                    mac_prt_rx_op = 1;
                    mrr_we = mac_prt_rx_rp;
                end
            end
        end
    end

    // logic to pass read replies to the idb.
    always_comb begin
        // Default values.
        idb_push = 0;
        idb_inp = 0;
        mrr_clr_ps = 0;
        ifq_pop = 0;

        // Loop over the replies and check if any of them match the next bytes to shift into the idb.
        for (int i = 0; i < MRR_LEN; i = i + 1) begin
            // If the reply matches the request in the head of the ifq.
            if (mrr_oup[i].addr == ifq_oup[0].addr) begin
                // Loop over the bytes and signal push to idb if needed.
                for (int j = 0; j < IDB_HEAD_COUNT; j = j + 1) begin
                    if (ifq_oup[0].rmsk[j]) begin
                        idb_inp[j] = mrr_oup[i].dat[(8*j)+:8];
                        idb_push[j] = 1;
                    end
                end

                // Signal to mrr to pop the reply & ifq to pop the req.
                mrr_clr_ps[i] = 1;
                ifq_pop[0] = 1;
            end
        end
    end
endmodule

/*
//          Main module         
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

    end

    always_comb begin
        
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

//          Decoder buffer          
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


//          Instruction fetcher         
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
    bit [7:0] mem_req_id = 1;
    bit inc_req_id;
    always @(posedge clk) begin
        if (inc_req_id) begin
            if (mem_req_id < 255) mem_req_id <= mem_req_id + 1;
            else mem_req_id <= 1;
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

    // Handle recieveing rx requests from noc.
    always_comb begin
        // If the noc port is sending is an rx.
        if (noc_port.rx_recieve) begin
            // If the buffer is full or the id does not match the one we want we skip it.
            if (rxb_ol == 8'b11111111 || noc_port.dat_from_noc.id != sr_oup.id) begin
                // Signal to the recieve buffer that we have nothing to write.
                rxb_we <= 0;

                // Signal to the NOC that we have not read its rx.
                // Signal that we have read it (Havent really, just want to skip it).
                rx_complete <= 1;
            end
            // If the recieve buffer is not full.
            else begin
                // Signal to recieve buffer that we want to write to it.
                rxb_we <= 1;

                // Signal to the NOC that we have recieved its rx.
                rx_complete <= 1;

                $display("Recieved instruction bytes from NOC. %t", $time);
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
                    selected_rx = i;
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

                $display("nothing present.");

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
*/
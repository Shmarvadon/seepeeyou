`include "structs.svh"
`include "defines.svh"

//`define IFE_DEBUG_LOG

module instruction_front_end #(parameter IQ_LEN = 8, IDB_LEN = 64)(
    input   logic               clk,                // Clock.
    input   logic               rst,                // Reset.

    // Interfaces to the back side of CPU core.

    input   logic [31:0]        pc_inp,             // input to program counter register.
    input   logic               jmp,                // Jump signal.

    output  queued_instruction  iq_oup,             // Instruction queue output.
    output  logic               iq_ip,              // Instruction queue instruction present.
    input   logic               iq_pop,             // Instruction queue pop.

    // Interface to the memory subsystem.
    output  logic               mac_prt_tx_rp,      // Mem acc cont tx request present.
    output  line_acc_req        mac_prt_tx_req,     // Mem acc cont tx request.
    input   logic               mac_prt_tx_ra,      // Mem acc cont tx request accepted.

    input   logic               mac_prt_rx_rp,      // Mem acc cont rx present.
    input   line_acc_req        mac_prt_rx_req,     // Mem acc cont rx data.
    output  logic               mac_prt_rx_ra       // Mem acc cont rx accepted.
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
    instruction_fetch #(IDB_LEN, 16) ibf(clk, ibf_rst, mac_prt_tx_rp, mac_prt_tx_req, mac_prt_tx_ra, mac_prt_rx_rp, mac_prt_rx_req, mac_prt_rx_ra, idb_src_num_avail, idb_dst_num_avail, idbba, idb_inp, idb_push);

    // Instruction decoder.
    logic [$bits(queued_instruction)-1:0]       decoded_inst;   // Decoded instruction.
    logic                                       inst_decoded;   // Instruction decoded.
    logic                                       inst_q_full;    // Instruction queue full.

    instruction_decoder #(MAX_INST_LEN, IDB_LEN) inst_dec(clk, rst, idb_oup, idb_dst_num_avail, idb_pop, decoded_inst, inst_decoded, inst_q_full, idbba);


    // Instruction Queue.
    logic [$clog2(IQ_LEN):0]                iq_src_num_avail;
    logic [$clog2(IQ_LEN):0]                iq_dst_num_avail;

    assign inst_q_full = (iq_src_num_avail == 0) ? 1 : 0;
    assign iq_ip = (iq_dst_num_avail != 0) ? 1 : 0;

    fifo_sr #($bits(queued_instruction), IQ_LEN, 1, 1) inst_q(clk, iq_rst, inst_decoded, decoded_inst, iq_src_num_avail, iq_pop, iq_oup, iq_dst_num_avail);

    // Handle top module level clocked logic.
    always @(posedge clk) begin
        // Default values.
        idb_rst = 0;
        ibf_rst = 0;
        iq_rst = 0;
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

    // Handle module reset.
    always @(posedge rst) begin
        idb_rst <= 1;
        ibf_rst <= 1;
        iq_rst <= 1;
        idbba <= 0;
    end

    always @(negedge rst) begin
        idb_rst <= 0;
        ibf_rst <= 0;
        iq_rst <= 0;
    end
endmodule

// Very simple for now, focusing on measuring matmul perf with cache before moving to ucode based decode.
module instruction_decoder #(parameter IB_INP_LEN = 8, IDB_LEN = 64)(
    input   logic                                     clk,      // Clock.
    input   logic                                     rst,      // Reset.

    // Connections to the instruction byte buffer.
    input   logic [IB_INP_LEN-1:0][7:0]               ib,       // Instruction bytes.
    input   logic [$clog2(IDB_LEN):0]                 iba,      // Instruction bytes available.
    output  logic [IB_INP_LEN-1:0]                    ibp,      // Pop Instruction bytes.

    // Connections to the instruction queue.
    output  logic [$bits(queued_instruction)-1:0]     dio,      // Decoded instruction output.
    output  logic                                     dip,      // Decoded instruction present.
    input   logic                                     iqf,      // Decoded instruction queue full.

    // Other connections to instruction front end module.
    input   logic [31:0]                              idbba     // Instruction decode buffer base address.
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
            `ifdef IFE_DEBUG_LOG
            $display("Decoding ALU instruction at %h", idbba);
            `endif

            // If the instruction is an immediate & enough bytes are present to fully decode & instruction queue isnt full.
            if (ib[0][3] & iba >= 6 & !iqf) begin
                `ifdef IFE_DEBUG_LOG
                $display("6 byte inst.");
                `endif
                // Send on over the instruction bytes & signal a pop for next clk from idb.
                for (int i = 0; i < 6; i = i + 1) begin
                    decoded_instruction.bits[(i * 8)+:8] = ib[i];
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
                decoded_instruction.bits = {ib[1], ib[0]};
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
            `ifdef IFE_DEBUG_LOG
            $display("Decoding Memory & IO instruction at %h", idbba);
            `endif
            // If the instruction is an immediate.
            if (!ib[0][3] & !iqf) begin
                // If the instruction is lod / str
                if (!ib[0][4] & iba >= 6) begin
                     // Send on over the instruction bytes & signal a pop for next clk from idb.
                    for (int i = 0; i < 6; i = i + 1) begin
                        decoded_instruction.bits[(i * 8)+:8] = ib[i];
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
                        decoded_instruction.bits[(i * 8)+:8] = ib[i];
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
                // if there are enough bytes to decode.
                if (iba >= 2) begin
                    // Send over instruction bytes & signal pop for next clk from idb.
                    decoded_instruction.bits = {ib[1], ib[0]};
                    ibp[1:0] = 2'b11;

                    // Assign address of instruction.
                    decoded_instruction.addr = idbba;
                    decoded_instruction.len = 2;

                    // Signal to push the instruction to the queue.
                    dip = 1;
                end
            end
        end

        // Program flow control instruction
        3'b110:
        begin
            `ifdef IFE_DEBUG_LOG
            $display("Decoding Program Flow Control instruction at %h", idbba);
            `endif
            // If the instruction is an immediate.
            if (ib[0][3] & !iqf & iba >= 5) begin
                // Send on over the instruction bytes & signal a pop for next clk from idb.
                for (int i = 0; i < 5; i = i + 1) begin
                    decoded_instruction.bits[(i * 8)+:8] = ib[i];
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
                decoded_instruction.bits = ib[0];
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
    input   logic                                   clk,                // Clock
    input   logic                                   rst,                // Reset.

    // Interface to the memory subsystem.
    output  logic                                   mac_prt_tx_rp,      // Mem acc cont tx request present.
    output  line_acc_req                            mac_prt_tx_req,     // Mem acc cont tx request.
    input   logic                                   mac_prt_tx_op,      // Mem acc cont tx port open.

    input   logic                                   mac_prt_rx_rp,      // Mem acc cont rx present.
    input   line_acc_req                            mac_prt_rx_req,     // Mem acc cont rx data.
    output  logic                                   mac_prt_rx_op,      // Mem acc cont rx port open.

    // Interface to the instruction decode bytes buffer.
    input   logic [$clog2(IDB_SIZE):0]              idb_src_num_avail,  // How many bytes can be pushed to at the moment.
    input   logic [$clog2(IDB_SIZE):0]              idb_dst_num_avail,  // Current length of the shift reg.
    input   logic [31:0]                            idbba,              // decode buffer base address.
    output  logic [IDB_HEAD_COUNT-1:0][7:0]         idb_inp,            // Instruction decode buffer input heads.
    output  logic [IDB_HEAD_COUNT-1:0]              idb_push            // Instruction decode buffer push.
);
    // local parameters.
    localparam IFQ_LEN = 4;
    localparam MRR_LEN = 4;

    typedef struct packed {
        logic [15:0] rmsk;
        logic [31:0] addr;
    } ifmrr;    // Instruction fetch memory read request.

    typedef struct packed {
        logic [31:0] addr;
        logic [127:0] dat;
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
    mrrr  [MRR_LEN-1:0]             mrr_oup;
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
        mac_prt_tx_req = 0;

        // Count up the number of bytes being fetched currently.
        // loop over all pop ports from sr.
        for (int i = 0; i < IFQ_LEN; i = i + 1) begin
            // If the entry is in range.
            if (i < ifq_dst_num_avail) begin
                // Loop over all the bytes in rmsk.
                for (int j = 0; j < 16; j = j + 1) begin
                    if (ifq_oup[i].rmsk[j]) current_fetch_addr = current_fetch_addr + 1;
                end
                `ifdef IFE_DEBUG_LOG
                $display("Current Fetch Address: %d", current_fetch_addr);
                `endif
            end

        end
        `ifdef IFE_DEBUG_LOG
        $display("Current Fetch Addr: %d", current_fetch_addr);
        `endif
        current_fetch_addr = current_fetch_addr + idbba + idb_dst_num_avail;
        `ifdef IFE_DEBUG_LOG
        $display("Current Fetch Addr: %d", current_fetch_addr);
        $display("Current Fetch Addr: %b", current_fetch_addr[31:4]);
        `endif

        if (idb_src_num_avail >= 16 & ifq_src_num_avail > 0) begin
            // Setup the ifq input.
            for (int i = 15; i >= 0; i = i - 1) begin
                if (i >= current_fetch_addr[3:0]) ifq_dinp.rmsk[i] = 1;
                else ifq_dinp.rmsk[i] = 0;
            end
            ifq_dinp.addr = {current_fetch_addr[31:4], 4'b0000}; // For some reason its bugged when 0 instead of 4'b0000...

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
            // Signal that we will accept a request.
            mac_prt_rx_op = 1;

            // Check that the reply is actually one we are expecting.
            for (int i = 0; i < IFQ_LEN; i = i + 1) begin
                // If the reply matches an in flight request.
                if (mac_prt_rx_req.addr == ifq_oup[i].addr) begin
                    mac_prt_rx_op = 1;
                    mrr_we = mac_prt_rx_rp;
                end
            end
            // If the read reply does not match anything in mrr (such as a read reply prior to a jump or GOTO), we will still accept it to clear it from cmac but NOT save it.
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
            if (mrr_oup[i].addr == ifq_oup[0].addr & mrr_used_pos[i] & idb_src_num_avail > 15) begin    // Not having idb src avail check resulted in incorrect fetch addr calc after decoder stall.
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

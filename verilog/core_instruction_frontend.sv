`include "structs.svh"
`include "defines.svh"

//`define IFE_DEBUG_LOG

module instruction_front_end #(parameter IQ_LEN = 8, IDB_LEN = 64, SB_ALLOC_PORTS = 3, PRF_WR_PRTS = 2, NUM_PHYSICAL_REGS = 128, ROB_HEADS = 3, ROB_LEN = 32, DESPATCH_LEN = 8, MAX_UOPS_PER_CLK = 3)(
    input   logic                                                           i_clk,                  // Clock.
    input   logic                                                           i_rst,                  // Reset.

    // Interface with the Commit stage. (not complete yet need to add lines to restore RAT).
    input   logic [31:0]                                                    i_pc_inp,               // input to program counter register.
    input   logic                                                           i_jmp,                  // Jump signal.

    //Interface from RAT_c
    input   logic [18:0][$clog2(NUM_PHYSICAL_REGS)-1:0]                     i_rat_c_alias,          // RAT_c aliases for use to reload RAT_s on jump.

    // Interface to score board.
    output  logic   [SB_ALLOC_PORTS-1:0]                                    o_alloc_pr,             // Allocate physical register
    input   logic   [SB_ALLOC_PORTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]     i_allocd_reg,           // Allocated physical register (PR).
    input   logic   [$clog2(NUM_PHYSICAL_REGS)-1:0]                         i_alloc_av,             // Number of free PRs.

    // Interface to physical register file.
    output  logic [PRF_WR_PRTS-1:0]                                         o_rf_we,                // Register file write enable.
    output  logic [PRF_WR_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]          o_rf_wr_trgt,           // Register file write target register.
    output  logic [PRF_WR_PRTS-1:0][31:0]                                   o_rf_wr_dat,            // Register file write data.    

    // Interface to despatch unit.
    output  logic [MAX_UOPS_PER_CLK-1:0]                                    o_desp_inp_p,           // Despatch stage uop present.
    output  micro_op_t [MAX_UOPS_PER_CLK-1:0]                               o_desp_inp,             // Despatch input.
    input   logic [$clog2(DESPATCH_STAGE_LEN)-1:0]                          i_desp_src_num_avail,   // Despatch stage number of uops it can accept.

    // Interface to ROB.
    output  logic [MAX_UOPS_PER_CLK-1:0]                                    o_rob_inp_p,            // Input push.
    output  rob_entry_t [MAX_UOPS_PER_CLK-1:0]                              o_rob_inp,              // Input data.
    input   logic [MAX_UOPS_PER_CLK-1:0][$clog2(ROB_LEN)-1:0]               i_rob_inp_ptr,          // Input ROB pointer.
    input   logic [$clog2(ROB_LEN)-1:0]                                     i_rob_src_num_avail,    // Source number of slots available.

    // Interface to the memory subsystem.
    output  logic                                                           o_mac_prt_tx_rp,        // Mem acc cont tx request present.
    output  line_acc_req                                                    o_mac_prt_tx_req,       // Mem acc cont tx request.
    input   logic                                                           i_mac_prt_tx_ra,        // Mem acc cont tx request accepted.

    input   logic                                                           i_mac_prt_rx_rp,        // Mem acc cont rx present.
    input   line_acc_req                                                    i_mac_prt_rx_req,       // Mem acc cont rx data.
    output  logic                                                           o_mac_prt_rx_ra         // Mem acc cont rx accepted.
);
    // Localparams (stuff to be tweaked globally not per instantiation).
    localparam MAX_INST_LEN = 6;

    // Reset signals.
    logic idb_rst;
    logic ibf_rst;
    logic ipd_rst;
    logic ide_rst;
    logic rat_rst;

    // instruction bytes buffer base address.
    logic [31:0]                            idbba; // Represents the base addrress of the instruction bytes buffer.

    // Instruction bytes buffer.
    logic [15:0]                            idb_push;
    logic [15:0][7:0]                       idb_inp;
    logic [$clog2(IDB_LEN):0]               idb_src_num_avail;

    logic [MAX_INST_LEN-1:0]                idb_pop;
    logic [MAX_INST_LEN-1:0][7:0]           idb_oup;
    logic [$clog2(IDB_LEN):0]               idb_dst_num_avail;


    // Instruction predecode stuff.
    logic               ipd_stall;
    logic               ipd_dn;
    inst_predec_res_t   ipd_oup;

    // Register alias table stuffs.
    logic [18:0]                                    rat_we;
    logic [18:0][$clog2(NUM_PHYSICAL_REGS)-1:0]     rat_alias;
    logic [18:0][$clog2(NUM_PHYSICAL_REGS)-1:0]     rat_new_alias;


    // Instruction bytes to decode buffer.
    fifo_sr #(8, IDB_LEN, 16, MAX_INST_LEN) idb(i_clk, idb_rst, idb_push, idb_inp, idb_src_num_avail, idb_pop, idb_oup, idb_dst_num_avail);

    // Instruction bytes fetcher.
    instruction_fetch #(IDB_LEN, 16) ibf(i_clk, ibf_rst, o_mac_prt_tx_rp, o_mac_prt_tx_req, i_mac_prt_tx_ra, i_mac_prt_rx_rp, i_mac_prt_rx_req, o_mac_prt_rx_ra, idb_src_num_avail, idb_dst_num_avail, idbba, idb_inp, idb_push);

    // Cool ass fancy pre decoder.
    instruction_pre_decode #(MAX_INST_LEN, IDB_LEN) ipd(i_clk, ipd_rst, ipd_stall, ipd_dn, idb_oup, idb_dst_num_avail, idbba, idb_pop, ipd_oup);

    // Instruction decoder.
    instruction_decode  #(SB_ALLOC_PORTS, NUM_PHYSICAL_REGS, PRF_WR_PRTS, ROB_LEN, DESPATCH_LEN, MAX_INST_LEN, 3) id(i_clk, ide_rst, ipd_stall,
    ipd_oup, ipd_dn, o_alloc_pr, i_allocd_reg, i_alloc_av, o_rf_we, o_rf_wr_trgt, o_rf_wr_dat, rat_alias, rat_we, rat_new_alias, o_desp_inp_p, o_desp_inp, i_desp_src_num_avail, 
    o_rob_inp_p, o_rob_inp, i_rob_inp_ptr, i_rob_src_num_avail);

    // Register Alias Table (speculative).
    register_allocation_table_ #(19, NUM_PHYSICAL_REGS) rat_s(i_clk,  rat_rst, i_jmp, i_rat_c_alias, rat_we, rat_new_alias, rat_alias);


    // Handle propogating sync reset & jump to sub-modules.
    always_comb begin
        // Default values.
        idb_rst = 0;
        ibf_rst = 0;
        ipd_rst = 0;
        ide_rst = 0;
        rat_rst = 0;

        // if jump.
        if (i_jmp) begin
            idb_rst = 1;
            ibf_rst = 1;
            ipd_rst = 1;
            ide_rst = 1;
            rat_rst = 0;
        end

        // If reset.
        if (i_rst) begin
            idb_rst = 1;
            ibf_rst = 1;
            ipd_rst = 1;
            ide_rst = 1;
            rat_rst = 1;
        end
    end

    // Handle top module level clocked logic.
    always @(posedge i_clk) begin

        // Update idbba when bytes are popped.
        for (int i = 0; i < MAX_INST_LEN; i = i + 1 ) begin
            if (idb_pop[i]) idbba = idbba + 1;
        end

        // Handle jump being signalled.
        if (i_jmp) begin
            idbba <= i_pc_inp;
        end

        // Handle reset being signalled.
        if (i_rst) begin
            idbba <= 0;
        end
    end
endmodule

// Instruction pre-decode, decodes the length, number of uops & the number of PRs that need to be allocated for decode.
module instruction_pre_decode #(parameter IB_INP_LEN = 6, IDB_LEN = 64)(
    input   logic                                                           i_clk,                  // Clock signal.
    input   logic                                                           i_rst,                  // Reset signal.
    input   logic                                                           i_stall,                // Stall signal.
    output  logic                                                           o_dn,                   // Done  signal.

    // Connections to the instruction byte buffer.
    input   logic [IB_INP_LEN-1:0][7:0]                                     i_ib,                    // Instruction bytes.
    input   logic [$clog2(IDB_LEN):0]                                       i_iba,                   // Instruction bytes available.
    input   logic [31:0]                                                    i_idbba,                 // Instruction bytes decode buffer base address.
    output  logic [IB_INP_LEN-1:0]                                          o_ibp,                   // Pop Instruction bytes.

    // Connections to the decode stage.
    output inst_predec_res_t                                                o_predecd_inst          // Instruction predecode result.
);

    // Perform pre-decode of the instruction.
    inst_predec_res_t       predec_res;
    logic [IB_INP_LEN-1:0]  inst_bytes_pop;
    logic [3:0]             inst_len;
    always_comb begin
        // Default values.
        predec_res = 0;
        inst_bytes_pop = 0;
        inst_len = 0;
        // A case statement here to set predec_res data.
        case(i_ib[0])
            //--------------------------------------
            //          ALU Instructions            
            //--------------------------------------

            `INST_ADD_A_B,
            `INST_SUB_A_B,
            `INST_AND_A_B,
            `INST_IOR_A_B,
            `INST_XOR_A_B,
            `INST_NOT_A_B,
            `INST_MUL_A_B,
            `INST_MOV_A_B,
            `INST_RLS_A_B,
            `INST_RRS_A_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 2,
                    bits:       i_ib,
                    num_prs:    2,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 2;
            end

            `INST_ADD_K_B,
            `INST_SUB_K_B,
            `INST_AND_K_B,
            `INST_IOR_K_B,
            `INST_XOR_K_B,
            `INST_NOT_K_B,
            `INST_MUL_K_B,
            `INST_MOV_K_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 6,
                    bits:       i_ib,
                    num_prs:    3,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 6;
            end

            `INST_CMP_A_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 2,
                    bits:       i_ib,
                    num_prs:    1,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 2;
            end

            `INST_CMP_K_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 6,
                    bits:       i_ib,
                    num_prs:    2,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 6;
            end

            //--------------------------------------
            //          LSU Instructions            
            //-------------------------------------- 

            `INST_LOD_K_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 6,
                    bits:       i_ib,
                    num_prs:    2,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 6;
            end

            `INST_STR_K_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 6,
                    bits:       i_ib,
                    num_prs:    1,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 6;
            end

            `INST_LOD_A_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 2,
                    bits:       i_ib,
                    num_prs:    1,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 2;
            end

            `INST_STR_A_B:
            begin
                predec_res = '{
                    pc:         i_idbba + 2,
                    bits:       i_ib,
                    num_prs:    0,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 2;
            end

            //--------------------------------------
            //          PFC Instructions            
            //-------------------------------------- 

            `INST_JMP_K,
            `INST_JIZ_K:
            begin
                predec_res = '{
                    pc:         i_idbba + 5,
                    bits:       i_ib,
                    num_prs:    1,
                    num_uops:   1,
                    default:    0
                };
                inst_len = 5;
            end

            `INST_GOT_K:
            begin
                predec_res = '{
                    pc:         i_idbba + 5,
                    bits:       i_ib,
                    num_prs:    3,
                    num_uops:   3,
                    default:    0
                };
                inst_len = 5;
            end

            `INST_RET:
            begin
                predec_res = '{
                    pc:         i_idbba + 1,
                    bits:       i_ib,
                    num_prs:    2,
                    num_uops:   3,
                    default:    0
                };
                inst_len = 1;
            end
        endcase

        // Set inst_bytes_pop based on the length of the instruction.
        for (int i = 0; i < IB_INP_LEN; i = i + 1) begin
            if (i < inst_len) inst_bytes_pop[i] = 1;
        end
    end

    // Determine what to do next.
    int state;
    always_comb begin
        // If not stalling.
        if (!i_stall) begin
            // If we have successfully decoded an instruction.
            if (i_iba != 0 & i_iba >= inst_len) begin
                state = 0;
            end
            // If we have not successfully decoded an instruction.
            else begin
                state = 1;
            end
        end
        // If we are stalling.
        else begin
            state = 2;
        end
    end

    // Set control signals to decode buffer.
    always_comb begin
        // Default value.
        o_ibp = 0;

        // If not stalling and decode valid.
        if (state == 0) o_ibp = inst_bytes_pop;

        // If reset then o_ibp = 0.
        if (i_rst) o_ibp = 0;
    end

    // Set output signals to next stage.
    always @(posedge i_clk) begin
        case(state)
        0:          begin   o_dn <= 1;  o_predecd_inst <= predec_res;   end // Present the new predecode result.
        1:          begin   o_dn <= 0;  o_predecd_inst <= 0;            end // Signal nothing predecoded.
        default:    begin                                               end // Hold previous outputs.
        endcase
    end

    // Handle sync reset.
    always @(posedge i_clk) begin
        if (i_rst) begin
            o_dn <= 0;
            o_predecd_inst <= 0;
        end
    end
endmodule

// Instruction decode, decodes the instruction to uops and allocates PRs.
module instruction_decode #(parameter SB_ALLOC_PORTS = 4, NUM_PHYSICAL_REGS = 64, PRF_WR_PRTS = 3, ROB_LEN = 16, DESPATCH_STAGE_LEN = 8, MAX_INST_LEN = 6, MAX_UOPS_PER_CLK = 3)(
    input   logic                                                           i_clk,                      // Input  clock signal.
    input   logic                                                           i_rst,                      // Input  Reset signal.
    output  logic                                                           o_stall,                    // Output Stall signal.

    // Connections from the pre-decode stage.
    input   inst_predec_res_t                                               i_inst_predec,              // Instruction predecode result.
    input   logic                                                           i_prv_stg_dn,               // Instruction predecode done.

    // Interface to score board.
    output  logic   [SB_ALLOC_PORTS-1:0]                                    o_alloc_pr,                 // Allocate physical register
    input   logic   [SB_ALLOC_PORTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]     i_allocd_reg,               // Allocated physical register (PR).
    input   logic   [$clog2(NUM_PHYSICAL_REGS)-1:0]                         i_alloc_av,                 // Number of free PRs.

    // Interface to physical register file.
    output  logic [PRF_WR_PRTS-1:0]                                         o_rf_we,                    // Register file write enable.
    output  logic [PRF_WR_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]          o_rf_wr_trgt,               // Register file write target register.
    output  logic [PRF_WR_PRTS-1:0][31:0]                                   o_rf_wr_dat,                // Register file write data.    

    // Interface to Register Alias Table (speculative).
    input   logic [18:0][$clog2(NUM_PHYSICAL_REGS)-1:0]                     i_rat_alias,                // Register Alias Table aliases
    output  logic [18:0]                                                    o_rat_we,                   // Register Alias Table write enable.
    output  logic [18:0][$clog2(NUM_PHYSICAL_REGS)-1:0]                     o_rat_new_alias,            // Register Alias Table new alias.

    // Interface to despatch unit.
    output  logic [MAX_UOPS_PER_CLK-1:0]                                    o_desp_inp_p,               // Despatch stage uop present.
    output  micro_op_t [MAX_UOPS_PER_CLK-1:0]                               o_desp_inp,                 // Despatch input.
    input   logic [$clog2(DESPATCH_STAGE_LEN)-1:0]                          i_desp_src_num_avail,       // Despatch stage number of uops it can accept.

    // ROB heads.
    output  logic [MAX_UOPS_PER_CLK-1:0]                                    o_rob_inp_p,                // Input push.
    output  rob_entry_t [MAX_UOPS_PER_CLK-1:0]                              o_rob_inp,                  // Input data.
    input   logic [MAX_UOPS_PER_CLK-1:0][$clog2(ROB_LEN)-1:0]               i_rob_inp_ptr,              // Input ROB pointer.
    input   logic [$clog2(ROB_LEN)-1:0]                                     i_rob_src_num_avail         // Source number of slots available.
);

    // Output signals from decode.

    // Scoreboard signals.
    logic [SB_ALLOC_PORTS-1:0]                              alloc_pr;

    // Register file write port signals.
    logic [PRF_WR_PRTS-1:0]                                 rf_we;
    logic [PRF_WR_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]  rf_wr_trgt;
    logic [PRF_WR_PRTS-1:0][31:0]                           rf_wr_dat;

    // RAT signals.
    logic [18:0]                                            rat_we;
    logic [18:0][$clog2(NUM_PHYSICAL_REGS)-1:0]             rat_new_alias;

    // Despatch unit signals.
    logic [MAX_UOPS_PER_CLK-1:0]                            desp_inp_p;
    micro_op_t   [MAX_UOPS_PER_CLK-1:0]                     decd_uop;

    // ROB signals.
    logic [MAX_UOPS_PER_CLK-1:0]                            rob_inp_p;
    rob_entry_t  [MAX_UOPS_PER_CLK-1:0]                     decd_rob_entry;  

    // Decode instruction.
    always_comb begin
        // check which inst is being decoded.
        case(i_inst_predec.bits[7:0])

            //--------------------------------------
            //          ALU Instructions            
            //--------------------------------------

            `INST_ADD_A_B:
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_ADD,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_ADD,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],      
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0 
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]           = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]]    = i_allocd_reg[0];
                alloc_pr[0]                                 = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]                = 1;
                rat_new_alias[isa_reg_alu_stat]         = i_allocd_reg[1];
                alloc_pr[1]                             = 1;
            end

            `INST_ADD_K_B:
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_ADD,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_ADD,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],      
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]           = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]]    = i_allocd_reg[0];
                alloc_pr[0]                                 = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]                = 1;
                rat_new_alias[isa_reg_alu_stat]         = i_allocd_reg[1];
                alloc_pr[1]                             = 1;
                // Write immediate to PRF.
                rf_we[0]                                = 1;
                rf_wr_trgt[0]                           = i_allocd_reg[2];
                rf_wr_dat[0]                            = i_inst_predec.bits[16+:32];
                alloc_pr[2]                             = 1;
            end

            `INST_SUB_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_SUB,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_SUB,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[1];
                alloc_pr[1]                     = 1;
            end

            `INST_SUB_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_SUB,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_SUB,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
                // Write immediate to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[2];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[2]                          = 1;
            end

            `INST_AND_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_AND,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_AND,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
            end

            `INST_AND_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_AND,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_AND,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
                // Write immediate to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[2];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[2]                          = 1;
            end

            `INST_IOR_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_IOR,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_IOR,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[1];
                alloc_pr[1]                     = 1;
            end

            `INST_IOR_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_IOR,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_IOR,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
                // Write immediate to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[2];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[2]                          = 1;
            end

            `INST_XOR_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_XOR,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_XOR,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[1];
                alloc_pr[1]                     = 1;
            end

            `INST_XOR_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_XOR,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_XOR,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
                // Write immediate to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[2];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[2]                          = 1;
            end

            `INST_NOT_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_NOT,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_NOT,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[1];
                alloc_pr[1]                     = 1;
            end

            `INST_NOT_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_NOT,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_allocd_reg[2],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_NOT,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
                // Write immediate to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[2];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[2]                          = 1;
            end

            `INST_MUL_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MUL,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MUL,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[1];
                alloc_pr[1]                     = 1;
            end

            `INST_MUL_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MUL,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MUL,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
                // Write immediate to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[2];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[2]                          = 1;
            end

            `INST_MOV_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MOV,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MOV,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[1];
                alloc_pr[1]                     = 1;
            end

            `INST_MOV_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MOV,
                    operand_a:          i_allocd_reg[2],
                    operand_b:          i_allocd_reg[2],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_MOV,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b111,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[2],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]             = 1;
                rat_new_alias[isa_reg_alu_stat]      = i_allocd_reg[1];
                alloc_pr[1]                          = 1;
                // Write immediate to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[2];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[2]                          = 1;
            end

            `INST_CMP_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_CMP,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          6'b0,     
                    operand_d:          i_allocd_reg[0],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                // CMP only writes ALU_STAT (no ISA destination write).
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_CMP,
                    commit_regs:        2'b10,              
                    commit_a_pr_alias:  6'b0,                        
                    commit_a_isa_reg:   5'b0,
                    commit_b_pr_alias:  i_allocd_reg[0],           
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b010,
                    stale_a_pr:         6'b0,
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for ALU_STAT only.
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[0];
                alloc_pr[0]                     = 1;
            end

            `INST_CMP_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_CMP,
                    operand_a:          i_allocd_reg[1],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    operand_c:          6'b0,
                    operand_d:          i_allocd_reg[0],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_CMP,
                    commit_regs:        2'b10,
                    commit_a_pr_alias:  6'b0,
                    commit_a_isa_reg:   5'b0,
                    commit_b_pr_alias:  i_allocd_reg[0],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b110,
                    stale_a_pr:         6'b0,
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    stale_c_pr:         i_allocd_reg[1],
                    default:            0
                };

                // Update alias for ALU_STAT only.
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[0];
                alloc_pr[0]                     = 1;
                // Write immediate to PRF (for immediate operand).
                rf_we[0]                        = 1;
                rf_wr_trgt[0]                   = i_allocd_reg[1];
                rf_wr_dat[0]                    = i_inst_predec.bits[16+:32];
                alloc_pr[1]                     = 1;
            end

            `INST_RLS_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_RLS,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_RLS,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]           = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]]    = i_allocd_reg[0];
                alloc_pr[0]                                 = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]            = 1;
                rat_new_alias[isa_reg_alu_stat]     = i_allocd_reg[1];
                alloc_pr[1]                         = 1;
            end

            `INST_RRS_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_RRS,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_c:          i_allocd_reg[0],
                    operand_d:          i_allocd_reg[1],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_ALU,
                    operation:          `UOP_ALU_RRS,
                    commit_regs:        2'b11,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    commit_b_pr_alias:  i_allocd_reg[1],
                    commit_b_isa_reg:   isa_reg_alu_stat,
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_rat_alias[isa_reg_alu_stat],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]           = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]]    = i_allocd_reg[0];
                alloc_pr[0]                                 = 1;
                // Update alias for isa reg ALU_STAT
                rat_we[isa_reg_alu_stat]        = 1;
                rat_new_alias[isa_reg_alu_stat] = i_allocd_reg[1];
                alloc_pr[1]                     = 1;
            end

            //--------------------------------------
            //          LSU Instructions            
            //--------------------------------------

            `INST_LOD_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_LOD,
                    operand_a:          i_allocd_reg[1],
                    operand_b:          i_allocd_reg[1],
                    operand_c:          i_allocd_reg[0],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_LOD,
                    commit_regs:        2'b01,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    stale_regs:         3'b011,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    stale_b_pr:         i_allocd_reg[1],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]        = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]] = i_allocd_reg[0];
                alloc_pr[0]                              = 1;
                // Write immediate address to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[1];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[1]                          = 1;
            end

            `INST_STR_K_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_STR,
                    operand_a:          i_allocd_reg[0],                       
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_STR,
                    commit_regs:        2'b00,
                    stale_regs:         3'b001,
                    stale_a_pr:         i_allocd_reg[0],
                    default:            0
                };

                // Write immediate address to PRF.
                rf_we[0]                             = 1;
                rf_wr_trgt[0]                        = i_allocd_reg[0];
                rf_wr_dat[0]                         = i_inst_predec.bits[16+:32];
                alloc_pr[0]                          = 1;
            end

            `INST_STR_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_STR,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_b:          i_rat_alias[i_inst_predec.bits[12+:4]],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_STR,
                    commit_regs:        2'b0,
                    stale_regs:         3'b0,
                    default:            0
                };
            end

            `INST_LOD_A_B: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_LOD,
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_a:          i_rat_alias[i_inst_predec.bits[8+:4]],
                    operand_c:          i_allocd_reg[0],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_LSU,
                    operation:          `UOP_LSU_LOD,
                    commit_regs:        2'b01,
                    commit_a_pr_alias:  i_allocd_reg[0],
                    commit_a_isa_reg:   i_inst_predec.bits[12+:4],
                    stale_regs:         3'b001,
                    stale_a_pr:         i_rat_alias[i_inst_predec.bits[12+:4]],
                    default:            0
                };

                // Update alias for isa reg destination.
                rat_we[i_inst_predec.bits[12+:4]]           = 1;
                rat_new_alias[i_inst_predec.bits[12+:4]]    = i_allocd_reg[0];
                alloc_pr[0]                                 = 1;
            end

            //--------------------------------------
            //          PFC Instructions            
            //--------------------------------------

            `INST_JMP_K: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_PFC,
                    operation:          `UOP_PFC_JMP,
                    operand_a:          i_allocd_reg[0],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_PFC,
                    operation:          `UOP_PFC_JMP,
                    commit_regs:        2'b00,
                    stale_regs:         3'b001,
                    stale_a_pr:         i_allocd_reg[0],
                    default:            0
                };

                // Write immediate target into PRF.
                rf_we[0]              = 1;
                rf_wr_trgt[0]         = i_allocd_reg[0];
                rf_wr_dat[0]          = i_inst_predec.bits[8+:32];
                alloc_pr[0]           = 1;
            end

            `INST_JIZ_K: 
            begin
                // Decode uop.
                decd_uop[0] = '{
                    exec_unit:          `EXEC_UNIT_PFC,
                    operation:          `UOP_PFC_JIZ,
                    operand_a:          i_allocd_reg[0],
                    operand_b:          i_rat_alias[isa_reg_alu_stat],
                    rob_ptr:            i_rob_inp_ptr[0],
                    default:            0
                };
                // Decode rob entry.
                decd_rob_entry[0] = '{
                    dn:                 1'b0,
                    pc:                 i_inst_predec.pc,
                    exec_unit:          `EXEC_UNIT_PFC,
                    operation:          `UOP_PFC_JIZ,
                    commit_regs:        2'b00,
                    stale_regs:         3'b001,
                    stale_a_pr:         i_allocd_reg[0],
                    default:            0
                };

                // Write immediate target into PRF.
                rf_we[0]              = 1;
                rf_wr_trgt[0]         = i_allocd_reg[0];
                rf_wr_dat[0]          = i_inst_predec.bits[16+:32];
                alloc_pr[0]           = 1;
            end

            `INST_GOT_K:
            begin
                // TODO
            end

            `INST_RET:
            begin
                // TODO
            end
        endcase
    end

    // Determine what to do next.
    int     state;
    int     uops_despatched;
    logic   rob_full;
    logic   desp_full;
    always_comb begin

        // Calculate how many uops were despatched in previous cycle.
        uops_despatched = 0;
        for (int i = 0; i < MAX_UOPS_PER_CLK; i = i + 1) begin if (o_rob_inp_p[i]) uops_despatched = uops_despatched + 1; end

        // Determine if ROB can accept next decode.
        rob_full    = ((i_inst_predec.num_uops + uops_despatched) < i_rob_src_num_avail) ? 0 : 1;
        // Determine if Despatch can accept next decode.
        desp_full   = ((i_inst_predec.num_uops + uops_despatched) < i_desp_src_num_avail) ? 0 : 1;

        // If previous stage has instruction for us.
        if (i_prv_stg_dn) begin
            // If enough room in the scheduler & ROB and the ScoreBoard has enough free PRs.
            if (!rob_full & !desp_full & i_alloc_av >= i_inst_predec.num_prs) begin
                state = 0;
            end
            // If we need to stall due to lack of resources.
            else begin
                state = 1;
            end
        end
        // If the previous stage does not have an instruction for us.
        else begin
            state = 2;
        end
    end

    // Set control signals to the RAT & ScoreBoard & previous stage.
    always_comb begin
        // Default values.
        o_alloc_pr = 0;
        o_rat_we = 0;
        o_rat_new_alias = 0;
        o_stall = 1;

        // If not stalling and decode is valid.
        if (state == 0) begin
            // Hook up the RAT & SB stuff.
            o_alloc_pr          = alloc_pr;
            o_rat_we            = rat_we;
            o_rat_new_alias     = rat_new_alias;
            // Signal that there is no stall.
            o_stall = 0;
        end

        // If reset.
        if (i_rst) begin
            o_alloc_pr = 0;
            o_rat_we = 0;
            o_rat_new_alias = 0;
            o_stall = 1;
        end
    end


    // Set outputs to the ROB, PRF, Despatch unit.
    always @(posedge i_clk) begin
        case(state)
        0:          begin   o_rf_we <= rf_we;   o_rf_wr_trgt <= rf_wr_trgt; o_rf_wr_dat <= rf_wr_dat;   o_desp_inp_p <= desp_inp_p; o_desp_inp <= decd_uop; o_rob_inp_p <= rob_inp_p;   o_rob_inp <= decd_rob_entry;    end // Present new uops.
        1:          begin   o_rf_we <= 0;       o_rf_wr_trgt <= 0;          o_rf_wr_dat <= 0;           o_desp_inp_p <= 0;          o_desp_inp <= 0;        o_rob_inp_p <= 0;           o_rob_inp <= 0;                 end // Signal nothing decoded.
        default:    begin                                                                                                                                                                                               end // Hold previous outputs.
        endcase
    end

    // Handle sync reset.
    always @(posedge i_clk) begin
        if (i_rst) begin
            o_rf_we <= 0;       
            o_rf_wr_trgt <= 0;          
            o_rf_wr_dat <= 0;           
            o_desp_inp_p <= 0;          
            o_desp_inp <= 0;        
            o_rob_inp_p <= 0;           
            o_rob_inp <= 0;
        end
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

// Simple wrapper around the re-usable register module.
module register_allocation_table_s #(parameter REG_COUNT = 19, PHYSICAL_REG_COUNT = 64) (
    input   logic                                                   i_clk,          // Clock signal.
    input   logic                                                   i_rst,          // Reset signal.

    input   logic                                                   i_jmp,          // Jump signal.
    input   logic [REG_COUNT-1:0][$clog2(PHYSICAL_REG_COUNT)-1:0]   i_rat_c_alias,  // RAT_c alias'.

    input   logic [REG_COUNT-1:0]                                   i_we,           // Write enable signal.
    input   logic [REG_COUNT-1:0][$clog2(PHYSICAL_REG_COUNT)-1:0]   i_dinp,         // New alias input.

    output  logic [REG_COUNT-1:0][$clog2(PHYSICAL_REG_COUNT)-1:0]   o_doup          // Current alias output.
);

    logic [REG_COUNT-1:0]                                   we;
    logic [REG_COUNT-1:0][$clog2(PHYSICAL_REG_COUNT)-1:0]   dinp;

    generate
        for (genvar i = 0; i < REG_COUNT; i = i + 1) begin
            // Instantiate a register to hold the alias.
            register #($clog2(PHYSICAL_REG_COUNT), i) reg_alloc(.clk(i_clk), .rst(i_rst), .inp(dinp[i]), .oup(o_doup[i]), .we(we[i]));

            // always block to handle hot reset on jump.
            always_comb begin
                // Default values.
                we[i]   = i_we[i];
                dinp[i] = i_dinp[i];

                // If there is a jump.
                if (i_jmp) begin
                    we[i]   = 1;
                    dinp[i] = i_rat_c_alias[i];
                end
            end

        end
    endgenerate
endmodule
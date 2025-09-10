`include "structs.svh"
`include "defines.svh"

module core(
    input cclk,
    input fclk,
    input rst,

    // NOC bus input.
    input   logic [31:0][7:0]   noc_bus_inp_dat,
    input   logic [5:0]         noc_bus_inp_bp,
    output  logic               noc_bus_inp_bo,

    // NOC bus output.
    output  logic [31:0][7:0]   noc_bus_oup_dat,
    output  logic [5:0]         noc_bus_oup_bp,
    input   logic               noc_bus_oup_bo
);

    //          ***Variables and localparams and such***

        //          Network Interface Unit          
        localparam NIU_PORTS = 1;
        logic [NIU_PORTS-1:0][3:0]  niu_prt_addr;
        logic [NIU_PORTS-1:0][3:0]  niu_prt_num;

        logic [NIU_PORTS-1:0]       niu_rx_av;
        logic [NIU_PORTS-1:0]       niu_rx_re;
        noc_packet [NIU_PORTS-1:0]  niu_rx_dat;

        logic [NIU_PORTS-1:0]       niu_tx_av;
        logic [NIU_PORTS-1:0]       niu_tx_re;
        noc_packet [NIU_PORTS-1:0]  niu_tx_dat;

        //          Memory Access Controller            
        localparam MAC_PORTS = 3;
        logic [MAC_PORTS-1:0]           mac_prt_inp_rp;
        line_acc_req [MAC_PORTS-1:0]    mac_prt_inp_req;
        logic [MAC_PORTS-1:0]           mac_prt_inp_op;

        logic [MAC_PORTS-1:0]           mac_prt_oup_rp;
        line_acc_req [MAC_PORTS-1:0]    mac_prt_oup_req;
        logic [MAC_PORTS-1:0]           mac_prt_oup_op;

        //         Instruction front end           
        logic [31:0]            ife_pc_inp;
        logic                   ife_jmp;
        micro_op                ife_iq_oup;
        logic                   ife_iq_ip;
        logic                   ife_iq_pop;
        logic                   ife_rst;

        //          General Purpose Registers           
        localparam NUM_PHYSICAL_REGS = 64;
        localparam NUMBER_RF_RD_PRTS = 8;       // ALU: 2, PFCU: 4, LSU: 2
        localparam NUMBER_RF_WR_PRTS = 6;       // DEC: 2, ALU: 2, PFCU: 1, LSU: 1

        logic [NUMBER_RF_RD_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]        rf_rd_prt_trgt_reg;
        logic [NUMBER_RF_RD_PRTS-1:0][31:0]                                 rf_rd_prt_dat;

        logic [NUMBER_RF_WR_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]        rf_wr_prt_trgt_reg;
        logic [NUMBER_RF_WR_PRTS-1:0][31:0]                                 rf_wr_prt_dat;
        logic [NUMBER_RF_WR_PRTS-1:0]                                       rf_wr_prt_we;

        logic gpr_rst;

        //          Score Board         
        localparam NUM_SB_ALLOC_PRTS = 3;
        localparam NUM_SB_FREE_PRTS = 2;
        logic [NUM_SB_ALLOC_PRTS-1:0]                                       sb_alloc_pr;
        logic [NUM_SB_ALLOC_PRTS-1:0][$bits(pr_alloc_purpose)-1:0]          sb_alloc_for;
        logic [NUM_SB_ALLOC_PRTS-1:0]                                       sb_alloc_av;
        logic [NUM_SB_ALLOC_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]        sb_allocd_reg;

        logic [NUM_SB_FREE_PRTS-1:0]                                        sb_free_pr;
        logic [NUM_SB_FREE_PRTS-1:0][$clog2(NUM_PHYSICAL_REGS)-1:0]         sb_pr_to_free;

        logic [NUM_PHYSICAL_REGS-1:0]                                       sb_pr_valid;

        logic [NUM_PHYSICAL_REGS-1:0][$bits(pr_alloc_purpose)-1:0]          sb_pr_purpose;

        //          ALU stuff           
        bit alu_rst;
        bit alu_dn;
        bit alu_en;

        //          PFCU stuff          
        bit pfcu_rst;
        bit pfcu_dn;
        bit pfcu_en;

        //          LSU stuff           
        bit lsu_rst;
        bit lsu_dn;
        bit lsu_en;

    //          ***Modules and such***



    //          Stack tracking regs         

    // Both of these are no longer used moving forwards, just exist here because I need their defaults for reference.
    register #(32, 'h000FFFFF) sbp_reg(cclk, rst, sbp_inp, sbp_oup, sbp_we);

    register #(32, 'h000FFFFF) shp_reg(cclk, rst, shp_inp, shp_oup, shp_we);




    //          Network Interface Unit          

    network_interface_unit #(NIU_PORTS, 0) niu(
    cclk, fclk, rst, niu_prt_addr, niu_prt_num,
    niu_rx_av, niu_rx_re, niu_rx_dat,
    niu_tx_av, niu_tx_re, niu_tx_dat,
    noc_bus_inp_dat, noc_bus_inp_bp, noc_bus_inp_bo,
    noc_bus_oup_dat, noc_bus_oup_bp, noc_bus_oup_bo
    );



    //          Memory Access Controller            

    mem_acc_cont #(MAC_PORTS) mac(cclk, rst,
    mac_prt_inp_rp, mac_prt_inp_req, mac_prt_inp_op,
    mac_prt_oup_rp, mac_prt_oup_req, mac_prt_oup_op,
    niu_prt_addr[0], niu_prt_num[0],
    niu_rx_av[0], niu_rx_re[0], niu_rx_dat[0],
    niu_tx_av[0], niu_tx_re[0], niu_tx_dat[0]
    );



    //          Instruction front end           
    // Uses PRF write ports 0,1
    instruction_front_end #(8, 64, 3, 2, NUM_PHYSICAL_REGS) ife(cclk, ife_rst, 
    ife_pc_inp, ife_jmp, ife_iq_oup, ife_iq_ip, ife_iq_pop,
    sb_alloc_pr, sb_alloc_for, sb_alloc_av, sb_allocd_reg,
    rf_wr_prt_we[1:0], rf_wr_prt_trgt_reg[1:0], rf_wr_prt_dat[1:0],
    mac_prt_inp_rp[0], mac_prt_inp_req[0], mac_prt_inp_op[0],
    mac_prt_oup_rp[0], mac_prt_oup_req[0], mac_prt_oup_op[0]);



    //          General Purpose Registers           

    general_purpose_registers #(32, NUM_PHYSICAL_REGS, NUMBER_RF_WR_PRTS, NUMBER_RF_RD_PRTS) gprs (cclk, gpr_rst, 
    rf_rd_prt_trgt_reg, rf_rd_prt_dat, rf_wr_prt_we, rf_wr_prt_trgt_reg, rf_wr_prt_dat);



    //          Score Board         

    scoreboard #(NUM_SB_ALLOC_PRTS, NUM_SB_FREE_PRTS, NUM_PHYSICAL_REGS, NUMBER_RF_WR_PRTS) sb(cclk, rst, 
    sb_alloc_pr, sb_alloc_for, sb_alloc_av, sb_allocd_reg,
    sb_free_pr, sb_pr_to_free, sb_pr_valid, sb_pr_purpose,
    rf_wr_prt_we, rf_wr_prt_trgt_reg);



    //          ALU stuff           
    // Uses PRF read ports 0,1 and PRF write port 2,3
    arithmetic_and_logic_unit #(NUM_PHYSICAL_REGS) alu(cclk, alu_rst, alu_en, alu_dn, ife_iq_oup,
    rf_rd_prt_trgt_reg[1:0], rf_rd_prt_dat[1:0], rf_wr_prt_trgt_reg[3:2], rf_wr_prt_dat[3:2], rf_wr_prt_we[3:2]);


    //          PFCU stuff          
    // Uses PRF read ports 2,3,4,5 and PRF write port 4
    program_flow_control #(NUM_PHYSICAL_REGS) pfcu(cclk, pfcu_rst, pfcu_en, pfcu_dn, ife_jmp, ife_pc_inp, ife_iq_oup,
    rf_rd_prt_trgt_reg[5:2], rf_rd_prt_dat[5:2], rf_wr_prt_trgt_reg[4], rf_wr_prt_dat[4], rf_wr_prt_we[4],
    mac_prt_inp_rp[1], mac_prt_inp_req[1], mac_prt_inp_op[1],
    mac_prt_oup_rp[1], mac_prt_oup_req[1], mac_prt_oup_op[1]);


    //          LSU stuff           
    // Uses PRF read ports 6,7 and PRF write port 5.
    load_store_unit #(NUM_PHYSICAL_REGS) lsu(cclk, lsu_rst, lsu_en, lsu_dn, ife_iq_oup,
    rf_rd_prt_trgt_reg[7:6], rf_rd_prt_dat[7:6], rf_wr_prt_trgt_reg[5], rf_wr_prt_dat[5], rf_wr_prt_we[5],
    mac_prt_inp_rp[2], mac_prt_inp_req[2], mac_prt_inp_op[2],
    mac_prt_oup_rp[2], mac_prt_oup_req[2], mac_prt_oup_op[2]);


    /*
    // Handle enabling the blocks depending on core state / instr.
    always_comb begin
        // Default values.
        ife_iq_pop = 0;
        alu_en = 0;
        pfcu_en = 0;
        lsu_en = 0;

        // If the instruction queue is not empty.
        if (ife_iq_ip) begin
            case(ife_iq_oup.bits[2:0])
            // ALU
            3'b100:
            begin
                ife_iq_pop = alu_dn;
                alu_en = 1;
            end

            // PFCU
            3'b110:
            begin
                ife_iq_pop = pfcu_dn;
                if (!pfcu_dn) pfcu_en = 1;  // Need this to stop PFCU double submitting line write requests.
            end

            // LSU
            3'b010:
            begin
                ife_iq_pop = lsu_dn;

                if (!lsu_dn) lsu_en = 1;
            end
            endcase
        end
    end
    */


    /*
    // Calculate some metrics for jump and GOTO cost.
    real avg_jmp_cost = 0;
    real avg_goto_cost = 0;

    longint num_jmps = 0;
    longint num_gots = 0;

    longint start;
    longint finish; 
    int time_taken;
    bit op_type;
    bit jmp_in_prog;
    always @(posedge pfcu_en) begin
        // default values.
        jmp_in_prog <= 0;

        // If the operation is a GOTO / RET
        if (ife_iq_oup.bits[6]) begin
            // set op type to GOTO / RET (0) and set start time and set jmp in prog to 1.
            op_type = 0;
            start = $time;
            jmp_in_prog <= 1;
        end
        // If the operation is a JMP / JIZ
        if (!ife_iq_oup.bits[6]) begin
            // set op type to JMP / JIZ (1) and set start time and set jmp in prog to 1.
            op_type = 1;
            start = $time;
            jmp_in_prog <= 1;
        end
    end

    always @(posedge ife_iq_ip) begin
        // If a jump is to be measured.
        if (jmp_in_prog) begin
            // Record finish time & calc time taken.
            finish = $time;
            time_taken = finish - start;

            // if op is JMP / JIZ.
            if (op_type) begin
                if (num_jmps != 0) begin
                    avg_jmp_cost = avg_jmp_cost + time_taken;
                    avg_jmp_cost = avg_jmp_cost / 2;
                end
                else begin
                    avg_jmp_cost = avg_jmp_cost + time_taken;
                end

                num_jmps = num_jmps + 1;
            end
            // if op is GOTO / RET.
            if (!op_type) begin
                if (num_gots != 0) begin
                    avg_goto_cost = avg_goto_cost + time_taken;
                    avg_goto_cost = avg_goto_cost / 2;
                end
                else begin
                    avg_goto_cost = avg_goto_cost + time_taken;
                end

                num_gots = num_gots + 1;
            end

        jmp_in_prog <= 0;
        end
    end
    */





    // Handle propogating reset to each IP.
    always @(posedge rst) begin
        // Send reset to each IP block.
        ife_rst <= 1;
        gpr_rst <= 1;
        alu_rst <= 1;
        pfcu_rst <= 1;
        lsu_rst <= 1;
    end
    always @(negedge rst) begin
        // Pull reset low for each IP block.
        ife_rst <= 0;
        gpr_rst <= 0;
        alu_rst <= 0;
        pfcu_rst <= 0;
        lsu_rst <= 0;
    end

    // Hande reset of the core.
    always @(posedge rst) begin
    end
endmodule
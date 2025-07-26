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

    /*          Stack tracking regs         */
    bit sbp_we;
    bit [31:0] sbp_inp;
    bit [31:0] sbp_oup;
    register #(32, 'h000FFFFF) sbp_reg(cclk, rst, sbp_inp, sbp_oup, sbp_we);

    bit shp_we;
    bit [31:0] shp_inp;
    bit [31:0] shp_oup;
    register #(32, 'h000FFFFF) shp_reg(cclk, rst, shp_inp, shp_oup, shp_we);


    /*          Network Interface Unit          */
    localparam NIU_PORTS = 1;
    logic [NIU_PORTS-1:0][3:0]  niu_prt_addr;
    logic [NIU_PORTS-1:0][3:0]  niu_prt_num;

    logic [NIU_PORTS-1:0]       niu_rx_av;
    logic [NIU_PORTS-1:0]       niu_rx_re;
    noc_packet [NIU_PORTS-1:0]  niu_rx_dat;

    logic [NIU_PORTS-1:0]       niu_tx_av;
    logic [NIU_PORTS-1:0]       niu_tx_re;
    noc_packet [NIU_PORTS-1:0]  niu_tx_dat;

    network_interface_unit #(NIU_PORTS, 0) niu(
    cclk, fclk, rst, niu_prt_addr, niu_prt_num,
    niu_rx_av, niu_rx_re, niu_rx_dat,
    niu_tx_av, niu_tx_re, niu_tx_dat,
    noc_bus_inp_dat, noc_bus_inp_bp, noc_bus_inp_bo,
    noc_bus_oup_dat, noc_bus_oup_bp, noc_bus_oup_bo
    );


    /*          Memory Access Controller            */
    localparam MAC_PORTS = 2;
    logic [MAC_PORTS-1:0]           mac_prt_inp_rp;
    line_acc_req [MAC_PORTS-1:0]    mac_prt_inp_req;
    logic [MAC_PORTS-1:0]           mac_prt_inp_op;

    logic [MAC_PORTS-1:0]           mac_prt_oup_rp;
    line_acc_req [MAC_PORTS-1:0]    mac_prt_oup_req;
    logic [MAC_PORTS-1:0]           mac_prt_oup_op;

    mem_acc_cont #(MAC_PORTS) mac(cclk, rst,
    mac_prt_inp_rp, mac_prt_inp_req, mac_prt_inp_op,
    mac_prt_oup_rp, mac_prt_oup_req, mac_prt_oup_op,
    niu_prt_addr[0], niu_prt_num[0],
    niu_rx_av[0], niu_rx_re[0], niu_rx_dat[0],
    niu_tx_av[0], niu_tx_re[0], niu_tx_dat[0]
    );


    /*          Instruction front end           */
    logic [31:0]            ife_pc_inp;
    logic                   ife_jmp;
    queued_instruction      ife_iq_oup;
    logic                   ife_iq_ip;
    logic                   ife_iq_pop;
    logic                   ife_rst;

    instruction_front_end #(8, 64) ife(cclk, ife_rst, 
    ife_pc_inp, ife_jmp, ife_iq_oup, ife_iq_ip, ife_iq_pop,
    mac_prt_inp_rp[0], mac_prt_inp_req[0], mac_prt_inp_op[0],
    mac_prt_oup_rp[0], mac_prt_oup_req[0], mac_prt_oup_op[0]);


    /*          General Purpose Registers           */
    logic [15:0] gpr_we;
    logic [31:0] gpr_inp [15:0];
    logic [31:0] gpr_oup [15:0];
    logic gpr_rst;
    general_purpose_registers gprs(cclk, gpr_rst, gpr_we, gpr_inp, gpr_oup);

    /*          ALU stuff           */
    bit alu_rst;
    bit alu_dn;
    bit [7:0] alu_status;
    bit alu_en;
    logic [31:0] alu_gpr_oup [15:0];
    logic [31:0] alu_gpr_inp [15:0];
    logic [15:0] alu_gpr_we;
    arithmetic_and_logic_unit alu(cclk, alu_rst, alu_en, alu_dn, ife_iq_oup.bits, alu_gpr_oup, alu_gpr_inp, alu_gpr_we, alu_status);


    /*          PFCU stuff          */
    bit pfcu_rst;
    bit pfcu_dn;
    bit pfcu_en;
    program_flow_control pfcu(cclk, pfcu_rst, pfcu_en, pfcu_dn, ife_iq_oup, alu_status, ife_jmp, ife_pc_inp, shp_we, shp_inp, shp_oup, 
    mac_prt_inp_rp[1], mac_prt_inp_req[1], mac_prt_inp_op[1],
    mac_prt_oup_rp[1], mac_prt_oup_req[1], mac_prt_oup_op[1]);


    /*          LSU stuff           */
    bit lsu_rst;
    bit lsu_dn;
    bit lsu_en;
    logic [31:0] lsu_gpr_oup [15:0];
    logic [31:0] lsu_gpr_inp [15:0];
    logic [15:0] lsu_gpr_we;

    load_store_unit lsu(clk, lsu_rst, lsu_en, lsu_dn, ife_iq_oup, lsu_gpr_oup, lsu_gpr_inp, lsu_gpr_we,
    mac_prt_inp_rp[2], mac_prt_inp_req[2], mac_prt_inp_op[2],
    mac_prt_oup_rp[2], mac_prt_oup_req[2], mac_prt_oup_op[2]);


    // Handle enabling the blocks depending on core state / instr.
    always_comb begin
        // Default values.
        ife_iq_pop = 0;
        gpr_we = 0;
        alu_en = 0;
        pfcu_en = 0;
        lsu_en = 0;

        // If the instruction queue is not empty.
        if (ife_iq_ip) begin
            case(ife_iq_oup.bits[2:0])
            // ALU
            3'b100:
            begin
                ife_iq_pop <= alu_dn;
                alu_en <= 1;
                gpr_inp <= alu_gpr_inp;
                alu_gpr_oup <= gpr_oup;
                gpr_we <= alu_gpr_we;
            end

            // PFCU
            3'b110:
            begin
                ife_iq_pop <= pfcu_dn;
                pfcu_en <= 1;
            end

            // LSU
            3'b010:
            begin
                ife_iq_pop <= lsu_dn;
                lsu_en <= 1;

                gpr_inp <= lsu_gpr_inp;
                lsu_gpr_oup <= gpr_oup;
                gpr_we <= lsu_gpr_we;
            end
            endcase
        end
    end

    /*
    always_comb begin
        // default values.
        rq_nxt_inst <= 1;
        gpr_we <= 0;

        // Check which IP needs to run.

        // ALU.
        if (curr_inst.inst[2:0] == 3'b100) begin
            rq_nxt_inst <= alu_done;
            alu_en <= 1;

            gpr_inp <= alu_gpr_inp;
            alu_gpr_oup <= gpr_oup;
            gpr_we <= alu_gpr_we;
        end
        else alu_en <= 0;

        // PFCU
        if (curr_inst.inst[2:0] == 3'b110) begin
            rq_nxt_inst <= pfcu_done;
            pfcu_en <= 1;

        end
        else pfcu_en <= 0;

        // MIOU
        if (curr_inst.inst[2:0] == 3'b010)begin
            rq_nxt_inst <= mio_done;
            mio_en <= 1;

            gpr_inp <= mio_gpr_inp;
            mio_gpr_oup <= gpr_oup;
            gpr_we <= mio_gpr_we;
        end
        else mio_en <= 0;
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
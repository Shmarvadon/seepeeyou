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
    niu_tx_av, niu_tx_re, niu_tx_dat
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

    /*          Instruction fetch, decode & queue logic         */
    bit [31:0] pc;
    queued_instruction curr_inst;
    bit inst_pres;
    bit rq_nxt_inst;
    bit inst_unt_rst;
    bit mdfy_pc;
    instruction_decoder #(`INSTRUCTION_QUEUE_LENGTH, `INSTRUCTION_QUEUE_INPUT_WIDTH) inst_fetch(cclk, inst_unt_rst, pc, mdfy_pc, curr_inst, inst_pres, rq_nxt_inst, noc_ports[0]);

    /*          General Purpose Registers           */
    bit [15:0] gpr_we;
    bit [31:0] gpr_inp [15:0];
    bit [31:0] gpr_oup [15:0];
    bit gpr_rst;
    general_purpose_registers gprs(cclk, gpr_rst, gpr_we, gpr_inp, gpr_oup);

    /*          ALU stuff           */
    bit alu_rst;
    bit alu_done;
    bit [7:0] alu_status;
    bit alu_en;
    bit [31:0] alu_gpr_oup [15:0];
    bit [31:0] alu_gpr_inp [15:0];
    bit [15:0] alu_gpr_we;
    arithmetic_and_logic_unit alu(cclk, alu_rst, alu_en, alu_done, curr_inst.inst, alu_gpr_oup, alu_gpr_inp, alu_gpr_we, alu_status);


    /*          PFCU stuff          */
    bit pfcu_rst;
    bit pfcu_done;
    bit pfcu_en;
    program_flow_control_unit pfcu(cclk, pfcu_rst, pfcu_en, pfcu_done, curr_inst, alu_status, mdfy_pc, pc, shp_we, shp_inp, shp_oup, noc_ports[1]);

    /*          M&IO stuff          */
    bit mio_rst;
    bit mio_done;
    bit mio_en;
    bit [31:0] mio_gpr_oup [15:0];
    bit [31:0] mio_gpr_inp [15:0];
    bit [15:0] mio_gpr_we;
    memory_and_io_unit miou(cclk, mio_rst, mio_en, mio_done, curr_inst, mio_gpr_oup, mio_gpr_inp, mio_gpr_we, noc_ports[2]);


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




    // Handle propogating reset to each IP.
    always @(posedge rst) begin
        // Send reset to each IP block.
        inst_unt_rst <= 1;
        gpr_rst <= 1;
        alu_rst <= 1;
        pfcu_rst <= 1;
        mio_rst <= 1;
    end
    always @(negedge rst) begin
        // Pull reset low for each IP block.
        inst_unt_rst <= 0;
        gpr_rst <= 0;
        alu_rst <= 0;
        pfcu_rst <= 0;
        mio_rst <= 0;
    end

    // Hande reset of the core.
    always @(posedge rst) begin
    end
endmodule
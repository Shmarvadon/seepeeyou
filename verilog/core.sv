`include "structs.sv"
`include "defines.sv"


module core(
    input clk,
    input fclk,
    input rst,

    input noc_bus noc_bus_inp,
    output noc_bus noc_bus_oup
);

    /*          NOC stop            */
    wire ip_port [2] noc_ports;
    noc_stop #(2, 0) noc_interface(fclk, clk, rst, noc_bus_inp, noc_bus_oup, noc_ports);

    /*          Instruction fetch, decode & queue logic         */
    bit [31:0] pc;
    bit [47:0] curr_inst;
    bit inst_pres;
    bit rq_nxt_inst;
    bit inst_unt_rst;
    bit mdfy_pc;
    instruction_decoder #(`INSTRUCTION_QUEUE_LENGTH, `INSTRUCTION_QUEUE_INPUT_WIDTH) inst_fetch(clk, inst_unt_rst, pc, mdfy_pc, curr_inst, inst_pres, rq_nxt_inst, noc_ports[0]);

    /*          General Purpose Registers           */
    bit [15:0] gpr_we;
    bit [31:0] gpr_inp [15:0];
    bit [31:0] gpr_oup [15:0];
    bit gpr_rst;
    general_purpose_registers gprs(clk, gpr_rst, gpr_we, gpr_inp, gpr_oup);

    /*          ALU stuff           */
    bit alu_rst;
    bit alu_done;
    bit [7:0] alu_status;
    arithmetic_and_logic_unit alu(clk, alu_rst, inst_pres, alu_done, curr_inst, gpr_oup, gpr_inp, gpr_we, alu_status);


    /*          PFCU stuff          */
    bit pfcu_rst;
    bit pfcu_done;
    program_flow_control_unit pfcu(clk, pfcu_rst, inst_pres, pfcu_done, curr_inst, alu_status, mdfy_pc, pc, noc_ports[1]);


    always @(posedge clk) begin
        // Check which IP is running.
        case (curr_inst[2:0])
        
        // ALU.
        3'b100:
        begin
            // If the ALU is done then request next instruction.
            if (alu_done) rq_nxt_inst <= 1;
            // If not done dont request next instruction.
            else rq_nxt_inst <= 0;
        end

        // MEMIO
        3'b010:
        begin
        end

        // PFCU
        3'b110:
        begin
            // If the PFCU is done then request next instruction.
            if (pfcu_done) rq_nxt_inst <= 1;
            else rq_nxt_inst <= 0;
        end

        // if no IP is selected, then ask for next instruction.
        default:
        begin
            rq_nxt_inst <= 1;
        end
        endcase
    end




    // Handle propogating reset to each IP.
    always @(posedge rst) begin
        // Send reset to each IP block.
        inst_unt_rst <= 1;
        gpr_rst <= 1;
        alu_rst <= 1;
        pfcu_rst <= 1;
    end
    always @(negedge rst) begin
        // Pull reset low for each IP block.
        inst_unt_rst <= 0;
        gpr_rst <= 0;
        alu_rst <= 0;
        pfcu_rst <= 0;
    end

    // Hande reset of the core.
    always @(posedge rst) begin
        // Reset the program counter.
        rq_nxt_inst <= 0;
    end
endmodule
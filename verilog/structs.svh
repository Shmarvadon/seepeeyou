/*          NOC stuff           */

typedef enum logic [1:0] {port_open, port_closed} noc_port_status;
typedef enum logic [7:0] {nothing, memory_read_request, memory_write_request, memory_read_reply, memory_write_reply, gpio_set_val_request, gpio_set_val_reply, gpio_read_val_request, gpio_read_val_reply} packet_type;

// Struct for NOC transmission infomation.
// Should be 3 bytes in size.
typedef struct packed { // MSB
    logic [3:0] src_addr;
    logic [3:0] src_port;

    logic [3:0] dst_addr;
    logic [3:0] dst_port;

    logic [7:0] len;                // Maximum length of 36B for enire packet, 33B for payload.
} noc_packet_header;    // LSB

// Represents maximum packet size of 36B, should NOT be used to represent an actual packet.
typedef struct packed { // MSB
    logic [263:0]       dat;        // Dont really care whats in here. Its for the IP block on recieveing end to deal with. Also length is just a placeholder, actual length of packet can be shorter.
    noc_packet_header   hdr;        // Standard header for the packet.
} noc_packet;           // LSB


// Memory read request.
typedef struct packed { // MSB
    logic [31:0]        addr;
    logic [7:0]         pt;
} mem_rd_rq;            // LSB

// Memory write request.
typedef struct packed { // MSB
    logic [31:0]        addr;
    logic [127:0]       dat;
    logic [7:0]         pt;
} mem_wr_rq;            // LSB

// Memory read reply.
typedef struct packed { // MSB
    logic [31:0]        addr;
    logic [127:0]       dat;
    logic [7:0]         pt;
} mem_rd_rp;            // LSB




/*          Core Memory Access Controller stuff         */

// Struct to store the currently active requests.
typedef struct packed {
    logic [31:0]    addr;   // Request address.
    logic [127:0]   dat;    // Request data.
    logic [15:0]    wmsk;   // Request write mask.
    logic           rqt;    // Request type (0 = read, 1 = write).
    logic [3:0]     prt;    // Request origin port.
} line_acc_req;

// Struct to pass a line read request.
typedef struct packed {
    logic [31:0] addr;
} line_read_req;

// Struct to pass a line read reply.
typedef struct packed {
    logic [31:0] addr;
    logic [127:0] dat;
} line_read_reply;

// Struct to pass line write request.
typedef struct packed {
    logic [31:0] addr;
    logic [127:0] dat;
} line_write_req;




/*          Instruction stuff           */

typedef struct packed { // MSB
    logic [2:0]     num_uops;       // Number of uops the instruction decodes to.
    logic [2:0]     num_prs;        // Number of PRs that the instruction requires be allocated.
    logic [47:0]    bits;           // Instruction bits.
    logic [31:0]    pc;             // Instruction PC.
} inst_predec_res_t;      // LSB


typedef struct packed { // MSB
    logic [4:0]     rob_ptr;
    logic [5:0]     operand_d;
    logic [5:0]     operand_c;
    logic [5:0]     operand_b;
    logic [5:0]     operand_a;
    logic [3:0]     operation;
    logic [2:0]     exec_unit;
} micro_op_t;             // LSB
 
typedef struct packed { // MSB
    // Stale physical registers.
    logic [5:0]     stale_c_pr;
    logic [5:0]     stale_b_pr;
    logic [5:0]     stale_a_pr;
    logic [2:0]     stale_regs;

    // info to update RAT_C.
    logic [4:0]     commit_b_isa_reg;
    logic [5:0]     commit_b_pr_alias;
    logic [4:0]     commit_a_isa_reg;
    logic [5:0]     commit_a_pr_alias;
    logic [1:0]     commit_regs;

    // uop code.
    logic [3:0]     operation;
    logic [2:0]     exec_unit;

    // tracking bits for stuffs.
    logic [31:0]    pc;
    logic           dn;
} rob_entry_t;            // LSB 



/*          Score Board stuff           */

// enum to signal what the purpose of a PR is.
typedef enum logic [4:0] {isa_reg_gpr_a, isa_reg_gpr_b, isa_reg_gpr_c, isa_reg_gpr_d, isa_reg_gpr_e, isa_reg_gpr_f, isa_reg_gpr_g, isa_reg_gpr_h, isa_reg_gpr_i, isa_reg_gpr_j, isa_reg_gpr_k, isa_reg_gpr_l, isa_reg_gpr_m, isa_reg_gpr_n, isa_reg_gpr_o, isa_reg_gpr_p, isa_reg_shp, isa_reg_sbp, isa_reg_alu_stat} isa_reg;


// Structure to hold data for each GPR.
typedef struct packed { //MSB
    logic               free;               // Indicates if the register is in use or not.
    logic               valid;              // Indicates if the register contains valid data or not.
} register_status;      // LB
  
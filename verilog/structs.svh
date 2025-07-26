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

typedef struct packed {
    bit [47:0] bits;        // Used to be called inst, if errors swap out with bits.
    bit [31:0] addr;
    bit [3:0] len;
} queued_instruction;




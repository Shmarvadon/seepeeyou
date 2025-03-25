
/*          NOC stuff           */
typedef enum logic [1:0] {port_open, port_closed} noc_port_status;
typedef enum logic [7:0] {nothing, memory_read_request, memory_write_request, memory_read_reply, memory_write_reply, gpio_set_val_request, gpio_set_val_reply, gpio_read_val_request, gpio_read_val_reply} packet_type;

// Data packet definition.
typedef struct packed {
    packet_type pt;
    logic [7:0]   id;
    logic [127:0] dat;

    logic [7:0]   src_addr;
    logic [7:0]   src_prt;

    logic [7:0]   dst_addr;
    logic [7:0]   dst_prt;
} packet;

// NOC bus definition
typedef struct packed {
    logic  packet_present;
    packet data;
} noc_bus;

// NOC stop IP interface port definition.
typedef struct packed {

    // Some stuff to inform the port as to its address.
    logic [7:0] port_address;
    logic [7:0] port_number;

    // data to the NOC.
    noc_port_status to_noc_prt_stat;
    logic tx_submit;
    logic tx_complete;
    packet dat_to_noc;

    // data from the NOC.
    noc_port_status from_noc_prt_stat;
    logic rx_recieve;
    logic rx_complete;
    packet dat_from_noc;

} ip_port;

typedef struct packed {
    bit [47:0] inst;
    bit [31:0] addr;
    bit [3:0] len;
} queued_instruction;



/*          NEW NOC DEFINITIONS         */


// port for IP blocks to interface with NOC stop.
// This interface GUARANTEES that it will only pass full, complete & contiguous packets.
interface noc_ip_port;

    // Some stuff to inform the connected IP of its port number and address.
    logic [3:0] prt_addr;
    logic [3:0] prt_num;

    // From the noc_stop to the connected IP block.
    logic rx_av;                    // Port rx available.
    logic rx_re;                    // Port rx read.        Should go high before clock edge where rx is accepted.
    noc_packet rx_dat;              // Port rx data.

    // From the connected IP block to noc_stop.
    logic tx_av;                    // Port tx available.
    logic tx_re;                    // Port tx read.        Should go high before clock edge where tx is accepted.
    noc_packet tx_dat;              // Port tx data.

    // NOC side of the interface.
    modport noc_side (output prt_addr, output prt_num, output rx_av, 
                      input rx_re, output rx_dat, input tx_av, 
                      output tx_re, input tx_dat);

    // IP block side of the interface.
    modport ip_side (input prt_addr, input prt_num, input rx_av,
                     output rx_re, input rx_dat, output tx_av,
                     input tx_re, output tx_dat);

endinterface

// Struct for NOC transmission infomation.
// Should be 3 bytes in size.
typedef struct packed { // MSB
    logic [3:0] src_addr;
    logic [3:0] src_port;

    logic [3:0] dst_addr;
    logic [3:0] dst_port;

    logic [7:0] len;                // Maximum length of 36B for enire packet, 32B for payload.
} noc_packet_header;    // LSB

// Represents maximum packet size of 36B, should NOT be used to represent an actual packet.
typedef struct packed { // MSB
    logic [255:0]       dat;        // Dont really care whats in here. Its for the IP block on recieveing end to deal with. Also length is just a placeholder, actual length of packet can be shorter.
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


/*          NOC stuff           */
typedef enum logic [1:0] {port_open, port_closed} noc_port_status;
typedef enum logic [7:0] {memory_read_request, memory_write_request, memory_read_reply} packet_type;

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
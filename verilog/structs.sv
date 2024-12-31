
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
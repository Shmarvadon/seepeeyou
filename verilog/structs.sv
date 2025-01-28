
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


interface noc_port;

    // Some stuff to inform the connected IP of its port number and address.
    logic [7:0] port_address;
    logic [7:0] port_number;

    // From the noc_stop to the connected IP block.
    noc_port_status prt_rx_stat;    // Rx port status.
    logic rx_av;                    // Port rx available.
    logic rx_re;                    // Port rx read.
    packet rx_dat;                  // Port rx data.

    // From the connected IP block to noc_stop.
    noc_port_status prt_tx_stat;    // Tx port status.
    logic tx_av;                    // Port tx available.
    logic tx_re;                    // Port tx read.
    packet tx_dat;                  // Port tx data.

    // NOC side of the interface.
    modport noc_side (output port_address, output port_number, output prt_rx_stat,
                      output rx_av, input rx_re, output rx_dat, output prt_tx_stat,
                      input tx_av, output tx_re, input tx_dat);

    // IP block side of the interface.
    modport ip_side(input port_address, input port_number, input prt_rx_stat,
                    input rx_av, output rx_re, input rx_dat, input prt_tx_stat,
                    output tx_av, input tx_re, output tx_dat);

endinterface
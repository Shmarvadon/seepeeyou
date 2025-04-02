`timescale 1ps/1ps



`include "defines.svh"
//`include "nocstop.sv"
`include "core_memory_access_controller.sv"


module soc_tb;
reg clk;
reg rst;

// Alternate a global cock at 
//always begin
    //#10 clk=~clk;
//end
//always @(posedge clk) $display("clock!");

initial begin
    clk = 0;
    rst <= 1;
    #5 rst <= 0;
end

// Emulate 64KB of memory.
bit [1048575:0][7:0] memory;

integer i;

// Setup a program to construct 2 matrices in memory & then multiply them.
initial begin

    //          *Fill 2 matrices of 8x8 with values*     

    //      i = row select 
    //      j = column select.
    //      k = final matrix element select.
    //
    // Memory indexing for element of matrix is done as follows:
    //      ind = (((i * 8) + j) * 4) + base_mem_addr
    //

    // Set base indices for the 3 matrices in GPRs.
    memory[5:0]         = 48'b0000_0000_0000_0000_1111_1111_1111_1111_0000_0011_0001_1100;             // Mov 0x0000FFFF, R3
    memory[11:6]        = 48'b0000_0000_0000_0001_0000_0000_1111_1111_0000_0100_0001_1100;             // Mov 0x000100FF, R4
    memory[17:12]       = 48'b0000_0000_0000_0001_0000_0001_1111_1111_0000_0101_0001_1100;             // Mov 0x000101FF, R5

    // Setup R13 (base addr) to pass to matrix filling proc to fill mat A.
    memory[23:18]       = 48'b0000_0000_0000_0000_1111_1111_1111_1111_0000_1101_0001_1100;             // Mov 0x0000FFFF,   R13
    memory[28:24]       = 40'b0000_0000_0000_0000_0001_0000_0001_0011_0100_1110;                       // GOT 4115

    // Setup R13 (base addr) to pass to matrix filling proc to fill mat B.
    memory[34:29]       = 48'b0000_0000_0000_0001_0000_0000_1111_1111_0000_1101_0001_1100;             // Mov 0x000100FF,   R13
    memory[39:35]       = 40'b0000_0000_0000_0000_0001_0000_0001_0011_0100_1110;                       // GOT 4115


    //          *Finally perform computation of matrices.*

    // Zero R6 (i), R7 (j), R8 (k)
    memory[45:40]       = 48'b0000_0000_0000_0000_0000_0000_0000_0000_0000_0110_0001_1100;             // MOV 0,   R6
    memory[51:46]       = 48'b0000_0000_0000_0000_0000_0000_0000_0000_0000_0111_0001_1100;             // MOV 0,   R7
    memory[57:52]       = 48'b0000_0000_0000_0000_0000_0000_0000_0000_0000_1000_0001_1100;             // MOV 0,   R8


    // Zero R9 (accumulator)
    memory[63:58]       = 48'b0000_0000_0000_0000_0000_0000_0000_0000_0000_1001_0001_1100;             // MOV 0,   R9


    // Get the memory index for matrix A.
    memory[65:64]       = 16'b1111_0110_0001_0100;                                                     // MOV R6,  R15
    memory[67:66]       = 16'b1110_1000_0001_0100;                                                     // MOV R8,  R14
    memory[73:68]       = 48'b0000_0000_0000_0000_1111_1111_1111_1111_0000_1101_0001_1100;             // MOV 0x0000FFFF,  R13
    memory[78:74]       = 40'b0000_0000_0000_0000_0001_0000_0000_0000_0100_1110;                       // GOT 4096

    // Load data from address of element from Matrix A.
    memory[80:79]       = 16'b1011_1100_1100_1010;                                                     // LOD R12, R11

    // Get the memory index for matrix B.
    memory[82:81]       = 16'b1111_1000_0001_0100;                                                     // MOV R8,  R15
    memory[84:83]       = 16'b1110_0111_0001_0100;                                                     // MOV R7,  R14
    memory[90:85]       = 48'b0000_0000_0000_0001_0000_0000_1111_1111_0000_1101_0001_1100;             // MOV 0x000100FF,  R13
    memory[95:91]       = 40'b0000_0000_0000_0000_0001_0000_0000_0000_0100_1110;                       // GOT 4096

    // Load data from address of element from Matrix B.
    memory[97:96]       = 16'b1010_1100_1100_1010;                                                     // LOD R12, R10

    // Multiply the 2 values & add to the accumulator.
    memory[99:98]       = 16'b1011_1010_1110_0100;                                                     // MUL R10, R11
    memory[101:100]     = 16'b1001_1011_1000_0100;                                                     // ADD R11, R9

    // Check if R8 (k) == 8.
    memory[107:102]       = 48'b0000_0000_0000_0000_0000_0000_0000_0001_0000_1000_1000_1100;           // ADD 1,    R8
    memory[113:108]       = 48'b0000_0000_0000_0000_0000_0000_0000_1000_0000_1000_1001_1100;           // CMP 8,    R8
    memory[118:114]       = 40'b0000_0000_0000_0000_0000_0000_0111_1100_1000_1110;                     // JIZ 124 (inst after next one).
    memory[123:119]       = 40'b0000_0000_0000_0000_0000_0000_0100_0000_0000_1110;                     // JMP 64

    // Write the value of accumulator to matrix c.
    memory[125:124]       = 16'b1111_0110_0001_0100;                                                   // MOV R6,  R15
    memory[127:126]       = 16'b1110_0111_0001_0100;                                                   // MOV R7,  R14
    memory[133:128]       = 48'b0000_0000_0000_0001_0000_0001_1111_1111_0000_1101_0001_1100;           // MOV 0x000101FF,  R13
    memory[138:134]       = 40'b0000_0000_0000_0000_0001_0000_0000_0000_0100_1110;                     // GOT 4096
    memory[140:139]       = 16'b1100_1001_0100_1010;                                                   // STR R9,  R12

    // Incriment j then check if j == 8.
    memory[146:141]       = 48'b0000_0000_0000_0000_0000_0000_0000_0001_0000_0111_1000_1100;           // ADD 1,   R7
    memory[152:147]       = 48'b0000_0000_0000_0000_0000_0000_0000_1000_0000_0111_1001_1100;           // CMP 8,   R7
    memory[157:153]       = 40'b0000_0000_0000_0000_0000_0000_1010_0011_1000_1110;                     // JIZ (Jumps to 163 if j == 8).
    memory[162:158]       = 40'b0000_0000_0000_0000_0000_0000_0011_0100_0000_1110;                     // JMP 52

    // Incriment i then check if i == 8.
    memory[168:163]       = 48'b0000_0000_0000_0000_0000_0000_0000_0001_0000_0110_1000_1100;           // ADD 1,   R6
    memory[174:169]       = 48'b0000_0000_0000_0000_0000_0000_0000_1000_0000_0110_1001_1100;           // CMP 8,   R6
    memory[179:175]       = 40'b0000_0000_0000_0000_0000_0000_0000_0000_1000_1110;                     // JIZ (Jumps to infinite loop at 185 if i == 8).
    memory[184:180]       = 40'b0000_0000_0000_0000_0000_0000_0010_1110_0000_1110;                     // JMP 46

    // Infinite loop.
    memory[189:185]       = 40'b0000_0000_0000_0000_0000_0000_1011_1001_0000_1110;                     // JMP 185





    //          * Subroutines*          
    // Start at addr 0x0FFF

    //          *Subroutine to calc mem acc index for matrix element*
    // Takes in R15 (i), R14 (j), R13 (base addr). Returns answer in R12.

    memory[4097:4096]   = 16'b1100_1111_0001_0100;                                                  // MOV R15, R12
    memory[4103:4098]   = 48'b0000_0000_0000_0000_0000_0000_0000_1000_0000_1100_1110_1100;          // MUL 8,   R12
    memory[4105:4104]   = 16'b1100_1110_1000_0100;                                                  // ADD R14, R12
    memory[4111:4106]   = 48'b0000_0000_0000_0000_0000_0000_0000_0100_0000_1100_1110_1100;          // MUL 4,   R12
    memory[4113:4112]   = 16'b1100_1101_1000_0100;                                                  // ADD R13, R12
    memory[4114]        = 8'b1100_0110;                                                             // RET



    //          *Loop to fill matrix*
    // Takes in R13 (base addr). Uses R15 (i), R14 (j), R12 & R11.

    // Zero the i & j indices.
    memory[4120:4115]       = 48'b0000_0000_0000_0000_0000_0000_0000_0000_0000_1111_0001_1100;          // Mov 0,   R15
    memory[4122:4121]       = 16'b1110_1111_0001_0100;                                                  // Mov R15,  R14

    // Get memory access index.
    memory[4127:4123]       = 40'b0000_0000_0000_0000_0001_0000_0000_0000_0100_1110;                    // GOT 4096
    // Calculate value to write to addr.
    memory[4129:4128]       = 16'b1011_1111_0001_0100;                                                  // MOV R15, R11
    memory[4131:4130]       = 16'b1011_1110_1110_0100;                                                  // MUL R14, R11
    // Write the value to the addr.
    memory[4133:4132]       = 16'b1100_1011_0100_1010;                                                  // STR R11, R12

    // Incriment j.
    memory[4139:4134]       = 48'b0000_0000_0000_0000_0000_0000_0000_0001_0000_1110_1000_1100;          // ADD 1,   R14

    // If j == 8, jump to reset j to 0 and inc i, else inc j then jump back to beginning of loop.

    // Compare j to 8.
    memory[4145:4140]       = 48'b0000_0000_0000_0000_0000_0000_0000_1000_0000_1110_1001_1100;          // CMP 8,   R14
    memory[4150:4146]       = 40'b0000_0000_0000_0000_0001_0000_0011_1100_1000_1110;                    // JIZ 4156
    memory[4155:4151]       = 40'b0000_0000_0000_0000_0001_0000_0001_1011_0000_1110;                    // JMP 4123
    memory[4161:4156]       = 48'b0000_0000_0000_0000_0000_0000_0000_0000_0000_1110_0001_1100;          // MOV 0,   R14
    memory[4167:4162]       = 48'b0000_0000_0000_0000_0000_0000_0000_0001_0000_1111_1000_1100;          // ADD 1,   R15

    // Test if i == 8, if so then reset i & j to 0, if not then keep doing loop.
    memory[4173:4168]       = 48'b0000_0000_0000_0000_0000_0000_0000_1000_0000_1111_1001_1100;          // CMP 8,   R15
    memory[4178:4174]       = 40'b0000_0000_0000_0000_0001_0000_0101_1000_1000_1110;                    // JIZ 4184
    memory[4183:4179]       = 40'b0000_0000_0000_0000_0001_0000_0001_1011_0000_1110;                    // JMP 4123
    memory[4184]            = 8'b1100_0110;                                                             // RET


    //$stop;
end


logic [31:0] mem_addr_sel;
wire logic [127:0] mem_dat;
bit [127:0] mem_dat_driver;
logic mem_we;
logic mem_re;
logic mem_en;
logic mclk;
soc sooc(rst, mem_addr_sel, mem_dat, mem_en, mem_we, mem_re, mclk);


// Handle the memory stuffs.
assign mem_dat = (mem_re) ? mem_dat_driver : 'hz;
always @(posedge mclk) begin
    // If the memory is enabled.
    if (mem_en) begin

        // memory write.
        if (mem_we) begin
            memory[mem_addr_sel[31:0]]          <= mem_dat[7:0];
            memory[mem_addr_sel[31:0] + 1]      <= mem_dat[15:8];
            memory[mem_addr_sel[31:0] + 2]      <= mem_dat[23:16];
            memory[mem_addr_sel[31:0] + 3]      <= mem_dat[31:24];
            memory[mem_addr_sel[31:0] + 4]      <= mem_dat[39:32];
            memory[mem_addr_sel[31:0] + 5]      <= mem_dat[47:40];
            memory[mem_addr_sel[31:0] + 6]      <= mem_dat[55:48];
            memory[mem_addr_sel[31:0] + 7]      <= mem_dat[63:56];

            $display("Memory write request. %b %t", mem_addr_sel, $time);

            memory[mem_addr_sel[31:0] + 8]      <= mem_dat[71:64];
            memory[mem_addr_sel[31:0] + 9]      <= mem_dat[79:72];
            memory[mem_addr_sel[31:0] + 10]     <= mem_dat[87:80];
            memory[mem_addr_sel[31:0] + 11]     <= mem_dat[95:88];
            memory[mem_addr_sel[31:0] + 12]     <= mem_dat[103:96];
            memory[mem_addr_sel[31:0] + 13]     <= mem_dat[111:104];
            memory[mem_addr_sel[31:0] + 14]     <= mem_dat[119:112];
            memory[mem_addr_sel[31:0] + 15]     <= mem_dat[127:120];

        end

        // memory read.
        if (mem_re) begin
            mem_dat_driver[7:0]      <=  memory[mem_addr_sel[31:0]];
            mem_dat_driver[15:8]     <=  memory[mem_addr_sel[31:0] + 1];
            mem_dat_driver[23:16]    <=  memory[mem_addr_sel[31:0] + 2];
            mem_dat_driver[31:24]    <=  memory[mem_addr_sel[31:0] + 3];
            mem_dat_driver[39:32]    <=  memory[mem_addr_sel[31:0] + 4];
            mem_dat_driver[47:40]    <=  memory[mem_addr_sel[31:0] + 5];
            mem_dat_driver[55:48]    <=  memory[mem_addr_sel[31:0] + 6];
            mem_dat_driver[63:56]    <=  memory[mem_addr_sel[31:0] + 7];

            $display("Memory read request. %b %t", mem_addr_sel, $time);

            mem_dat_driver[71:64]    <=  memory[mem_addr_sel[31:0] + 8];
            mem_dat_driver[79:72]    <=  memory[mem_addr_sel[31:0] + 9];
            mem_dat_driver[87:80]    <=  memory[mem_addr_sel[31:0] + 10];
            mem_dat_driver[95:88]    <=  memory[mem_addr_sel[31:0] + 11];
            mem_dat_driver[103:96]   <=  memory[mem_addr_sel[31:0] + 12];
            mem_dat_driver[111:104]  <=  memory[mem_addr_sel[31:0] + 13];
            mem_dat_driver[119:112]  <=  memory[mem_addr_sel[31:0] + 14];
            mem_dat_driver[127:120]  <=  memory[mem_addr_sel[31:0] + 15];
        end
    end
end

endmodule

/*
module soc_tb;
reg clk;
reg rst;

// Alternate a global cock at 
//always begin
    //#10 clk=~clk;
//end
//always @(posedge clk) $display("clock!");

initial begin
    clk = 0;
    rst <= 1;
    #5 rst <= 0;
end

// Emulate 64KB of memory.
bit [0:7] memory [65535:0];

integer i;

initial begin
    // Setup instructions in memory.

    // ADD R2, R2
    memory[0] = 8'b10000100;
    memory[1] = 8'b00100010;

    // ADD 24, R2
    memory[2] = 8'b10001100;
    memory[3] = 8'b00000010;
    memory[4] = 0;
    memory[5] = 0;
    memory[6] = 0;
    memory[7] = 24;

    //RLS R2, R3
    memory[8] = 8'b01010100;
    memory[9] = 8'b00110010;
    
    //MUL 10, R3
    memory[10] = 8'b11101100;
    memory[11] = 8'b00000011;
    memory[12] = 0;
    memory[13] = 0;
    memory[14] = 0;
    memory[15] = 10;

    // GOT 64
    memory[16] = 8'b01001110;
    memory[17] = 64;
    memory[18] = 0;
    memory[19] = 0;
    memory[20] = 0;

    // LOD 0, R1
    memory[21] = 8'b00000010;
    memory[22] = 8'b00000001;
    memory[23] = 0;
    memory[24] = 0;
    memory[25] = 0;
    memory[26] = 0;

    // STR 128, R1
    memory[27] = 8'b10000010;
    memory[28] = 8'b00000001;
    memory[29] = 128;
    memory[30] = 0;
    memory[31] = 0;
    memory[32] = 0;

    //JMP 0
    memory[33] = 8'b00001110;
    memory[34] = 0;
    memory[35] = 0;
    memory[36] = 0;
    memory[37] = 0;

    // RET
    memory[64] = 8'b11000110;
end


logic [31:0] mem_addr_sel;
wire logic [127:0] mem_dat;
bit [127:0] mem_dat_driver;
logic mem_we;
logic mem_re;
logic mem_en;
logic mclk;
soc sooc(rst, mem_addr_sel, mem_dat, mem_en, mem_we, mem_re, mclk);


// Handle the memory stuffs.
assign mem_dat = (mem_re) ? mem_dat_driver : 'hz;
always @(posedge mclk) begin
    // If the memory is enabled.
    if (mem_en) begin

        // memory write.
        if (mem_we) begin
            memory[mem_addr_sel[31:0]]          <= mem_dat[7:0];
            memory[mem_addr_sel[31:0] + 1]      <= mem_dat[15:8];
            memory[mem_addr_sel[31:0] + 2]      <= mem_dat[23:16];
            memory[mem_addr_sel[31:0] + 3]      <= mem_dat[31:24];
            memory[mem_addr_sel[31:0] + 4]      <= mem_dat[39:32];
            memory[mem_addr_sel[31:0] + 5]      <= mem_dat[47:40];
            memory[mem_addr_sel[31:0] + 6]      <= mem_dat[55:48];
            memory[mem_addr_sel[31:0] + 7]      <= mem_dat[63:56];

            $display("Memory write request. %b %t", mem_addr_sel, $time);

            memory[mem_addr_sel[31:0] + 8]      <= mem_dat[71:64];
            memory[mem_addr_sel[31:0] + 9]      <= mem_dat[79:72];
            memory[mem_addr_sel[31:0] + 10]     <= mem_dat[87:80];
            memory[mem_addr_sel[31:0] + 11]     <= mem_dat[95:88];
            memory[mem_addr_sel[31:0] + 12]     <= mem_dat[103:96];
            memory[mem_addr_sel[31:0] + 13]     <= mem_dat[111:104];
            memory[mem_addr_sel[31:0] + 14]     <= mem_dat[119:112];
            memory[mem_addr_sel[31:0] + 15]     <= mem_dat[127:120];

        end

        // memory read.
        if (mem_re) begin
            mem_dat_driver[7:0]      <=  memory[mem_addr_sel[31:0]];
            mem_dat_driver[15:8]     <=  memory[mem_addr_sel[31:0] + 1];
            mem_dat_driver[23:16]    <=  memory[mem_addr_sel[31:0] + 2];
            mem_dat_driver[31:24]    <=  memory[mem_addr_sel[31:0] + 3];
            mem_dat_driver[39:32]    <=  memory[mem_addr_sel[31:0] + 4];
            mem_dat_driver[47:40]    <=  memory[mem_addr_sel[31:0] + 5];
            mem_dat_driver[55:48]    <=  memory[mem_addr_sel[31:0] + 6];
            mem_dat_driver[63:56]    <=  memory[mem_addr_sel[31:0] + 7];

            $display("Memory read request. %b %t", mem_addr_sel, $time);

            mem_dat_driver[71:64]    <=  memory[mem_addr_sel[31:0] + 8];
            mem_dat_driver[79:72]    <=  memory[mem_addr_sel[31:0] + 9];
            mem_dat_driver[87:80]    <=  memory[mem_addr_sel[31:0] + 10];
            mem_dat_driver[95:88]    <=  memory[mem_addr_sel[31:0] + 11];
            mem_dat_driver[103:96]   <=  memory[mem_addr_sel[31:0] + 12];
            mem_dat_driver[111:104]  <=  memory[mem_addr_sel[31:0] + 13];
            mem_dat_driver[119:112]  <=  memory[mem_addr_sel[31:0] + 14];
            mem_dat_driver[127:120]  <=  memory[mem_addr_sel[31:0] + 15];
        end
    end
end

endmodule
*/

/*

module tb;
reg clk;
reg clk_2;
reg rst;
reg en;

// Alternate a global cock at 
always begin
    #10 clk=~clk;
end

// Alternate secondary clock.
always begin
    #5 clk_2=~clk_2;
end

always @(posedge clk) $display("clock!");


initial begin
    en = 1;
    clk = 0;
    clk_2 = 0;
    rst <= 1;
    #5 rst <= 0;
end

// Emulate 64KB of memory.
bit [0:7] memory [65535:0];

integer i;
initial begin
    // Setup instructions in memory.

    // ADD R2, R2
    memory[0] = 8'b10000100;
    memory[1] = 8'b00100010;

    // ADD 24, R2
    memory[2] = 8'b10001100;
    memory[3] = 8'b00000010;
    memory[4] = 0;
    memory[5] = 0;
    memory[6] = 0;
    memory[7] = 24;

    //RLS R2, R3
    memory[8] = 8'b01010100;
    memory[9] = 8'b00110010;
    
    //MUL 10, R3
    memory[10] = 8'b11101100;
    memory[11] = 8'b00000011;
    memory[12] = 0;
    memory[13] = 0;
    memory[14] = 0;
    memory[15] = 10;

    // GOT 64
    memory[16] = 8'b01001110;
    memory[17] = 64;
    memory[18] = 0;
    memory[19] = 0;
    memory[20] = 0;

    // LOD 0, R1
    memory[21] = 8'b00000010;
    memory[22] = 8'b00000001;
    memory[23] = 0;
    memory[24] = 0;
    memory[25] = 0;
    memory[26] = 0;

    // STR 128, R1
    memory[27] = 8'b10000010;
    memory[28] = 8'b00000001;
    memory[29] = 128;
    memory[30] = 0;
    memory[31] = 0;
    memory[32] = 0;

    //JMP 0
    memory[33] = 8'b00001110;
    memory[34] = 0;
    memory[35] = 0;
    memory[36] = 0;
    memory[37] = 0;

    // RET
    memory[64] = 8'b11000110;

end

wire noc_bus into_core;
wire noc_bus out_of_core;
// Instantiate a core.
core test_core(clk_2, clk, rst, into_core, out_of_core);

// Initialise a secondary NOC stop to act as IMC endpoint.
wire ip_port mem_noc_prt;
noc_stop #(1, 2) memory_endpoint(clk, clk_2, rst, out_of_core, into_core, mem_noc_prt);

packet reply_pckt;
logic submit_tx;
logic rx_complete;

assign mem_noc_prt.dat_to_noc = reply_pckt;
assign mem_noc_prt.tx_submit = submit_tx;
assign mem_noc_prt.rx_complete = rx_complete;

// Very dumb noc port interface.
always @(posedge clk_2) begin
    // If there is a memory access request.
    if (mem_noc_prt.rx_recieve) begin
        // if a memory read request.
        if (mem_noc_prt.dat_from_noc.pt == memory_read_request) begin
            // Fill the reply.
            reply_pckt.pt = memory_read_reply;
            reply_pckt.id = mem_noc_prt.dat_from_noc.id;

            reply_pckt.dat[7:0]      <=  memory[mem_noc_prt.dat_from_noc.dat[31:0]];
            reply_pckt.dat[15:8]     <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 1];
            reply_pckt.dat[23:16]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 2];
            reply_pckt.dat[31:24]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 3];
            reply_pckt.dat[39:32]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 4];
            reply_pckt.dat[47:40]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 5];
            reply_pckt.dat[55:48]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 6];
            reply_pckt.dat[63:56]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 7];

            reply_pckt.dat[71:64]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 8];
            reply_pckt.dat[79:72]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 9];
            reply_pckt.dat[87:80]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 10];
            reply_pckt.dat[95:88]    <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 11];
            reply_pckt.dat[103:96]   <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 12];
            reply_pckt.dat[111:104]  <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 13];
            reply_pckt.dat[119:112]  <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 14];
            reply_pckt.dat[127:120]  <=  memory[mem_noc_prt.dat_from_noc.dat[31:0] + 15];

            reply_pckt.dst_addr <= mem_noc_prt.dat_from_noc.src_addr;
            reply_pckt.dst_prt <= mem_noc_prt.dat_from_noc.src_prt;

            reply_pckt.src_addr <= 2;
            reply_pckt.src_prt <= 0;

            // submit the reply.
            submit_tx <= 1;

            // Indicate that the rx is complete
            rx_complete <= 1;
        end

        // If a memory write request.
        if (mem_noc_prt.dat_from_noc.pt == memory_write_request) begin
            // Write to memory.
            memory[mem_noc_prt.dat_from_noc.dat[31:0]] = mem_noc_prt.dat_from_noc.dat[39:32]; 
            memory[mem_noc_prt.dat_from_noc.dat[31:0]+1] = mem_noc_prt.dat_from_noc.dat[47:40]; 
            memory[mem_noc_prt.dat_from_noc.dat[31:0]+2] = mem_noc_prt.dat_from_noc.dat[55:48]; 
            memory[mem_noc_prt.dat_from_noc.dat[31:0]+3] = mem_noc_prt.dat_from_noc.dat[63:56]; 

            // create reply.
            reply_pckt.pt = memory_write_reply;
            reply_pckt.id = mem_noc_prt.dat_from_noc.id;

            reply_pckt.dst_addr <= mem_noc_prt.dat_from_noc.src_addr;
            reply_pckt.dst_prt <= mem_noc_prt.dat_from_noc.src_prt;

            reply_pckt.src_addr <= 2;
            reply_pckt.src_prt <= 0;

            // Submit the reply.
            submit_tx <= 1;

            // indicate that rx is complete.
            rx_complete <= 1;
        end
    end
    // If the reply is complete.
    if (mem_noc_prt.tx_complete) begin
        // Stop signalling to send a tx.
        submit_tx <= 0;

        // signal that reading the rx is not done yet.
        rx_complete <= 0;
    end
end


endmodule

*/


/*
wire noc_bus one_to_two;
wire noc_bus two_to_one;

wire ip_port stop_one_prt_one;
noc_stop #(1, 1) noc_stop_one(clk, clk_2, rst, two_to_one, one_to_two, stop_one_prt_one);

wire ip_port stop_two_prt_one;
noc_stop #(1, 2) noc_stop_two(clk, clk_2, rst, one_to_two, two_to_one, stop_two_prt_one);

// Stop one stuff.
packet stp_one_tx_dat;
logic stp_one_tx_submit;

logic stp_one_rx_complete;

assign stop_one_prt_one.dat_to_noc = stp_one_tx_dat;
assign stop_one_prt_one.tx_submit = stp_one_tx_submit;

assign stop_one_prt_one.rx_complete = stp_one_rx_complete;

// Stop two stuff.
packet stp_two_tx_dat;
logic stp_two_tx_submit;

logic stp_two_rx_complete;

assign stop_two_prt_one.dat_to_noc = stp_two_tx_dat;
assign stop_two_prt_one.tx_submit = stp_two_tx_submit;

assign stop_two_prt_one.rx_complete = stp_two_rx_complete;

// Send a packet to noc stop one to go to noc stop 2.
initial begin

    #10

    stp_one_tx_dat.dst_addr <= 2;
    stp_one_tx_dat.dst_prt <= 0;

    stp_one_tx_dat.src_addr <= 1;
    stp_one_tx_dat.src_prt <= 0;

    stp_one_tx_dat.dat <= 'h42069;

    stp_one_tx_dat.pt = memory_read_request;
    stp_one_tx_dat.id = 0;

    #10 stp_one_tx_submit <= 1;

    #10 stp_one_tx_submit <= 0;

    // Modify packet and transmit another one.
    stp_one_tx_dat.dat <= 'h1234;
    #10 stp_one_tx_submit <= 1;

    #10 stp_one_tx_submit <= 0;
end


packet recieved_packet;
// Handle recieveing the packet at noc stop 2.
always @(posedge clk_2) begin
    // If there is a packet waiting for us to recieve.
    if (stop_two_prt_one.rx_recieve) begin
        // Move the data into recieved_packet variable.
        recieved_packet <= stop_two_prt_one.dat_from_noc;

        // Signal to the port that we have read the data.
        stp_two_rx_complete <= 1;

        // Lets also reply with something.
        stp_two_tx_dat.dst_addr <= 1;
        stp_two_tx_dat.dst_prt <= 0;
        stp_two_tx_dat.src_addr <= 1;
        stp_two_tx_dat.src_prt <= 0;
        stp_two_tx_dat.dat = 'h420420420;
        stp_two_tx_dat.pt = memory_write_request;
        stp_two_tx_dat.id = 69;

        stp_two_tx_submit <= 1;
    end
    // If there is not a packet waiting for us to recieve.
    else begin
        // Signal to the port that we have not read any data.
        stp_two_rx_complete <= 0;
    end

    // If the tx submitted has been processed successfully.
    if (stop_two_prt_one.tx_complete) begin

        // Stop requesting TX.
        stp_two_tx_submit <= 0;

        stp_two_tx_dat.dst_addr <= 0;
        stp_two_tx_dat.dst_prt <= 0;
        stp_two_tx_dat.dat = 0;
        stp_two_tx_dat.pt = memory_write_request;
        stp_two_tx_dat.id = 0;
    end
end

// Handle recieveing the ping back from noc stop 2.
always @(posedge clk_2) begin
    // If there is an RX for us.
    if (stop_one_prt_one.rx_recieve) begin
        // Accept it.
        stp_one_rx_complete <= 1;

    end
    // If there is not an rx for us.
    else begin
        stp_one_rx_complete <= 0;
    end
end
*/



















/*

//          NOC port handling stuff
wire ip_port noc_port;
wire ip_port pcfu_noc_port;

packet reply;
noc_port_status reply_prt_stat;
noc_port_status req_prt_stat;
logic tx_send;

assign noc_port.dat_from_noc = reply;
assign noc_port.from_noc_prt_stat = reply_prt_stat;
assign noc_port.to_noc_prt_stat = req_prt_stat;
assign noc_port.tx_recieve = tx_send;

core cpu_core(clk, rst, noc_port, pcfu_noc_port); 


//         Main memory emulator            

// Emulate 64KB of memory.
bit [0:7] memory [65535:0];

always @(posedge clk) begin
    // If the instruction fetch unit has submitted a request to the noc.
    if (noc_port.tx_submit) begin
        // If the request is for data from memory.
        if (noc_port.dat_to_noc.pt == memory_read_request) begin
            // Fill the reply.
            reply.pt = memory_read_reply;
            reply.id = 0;

            reply.dat[7:0]      <=  memory[noc_port.dat_to_noc.dat[31:0]];
            reply.dat[15:8]     <=  memory[noc_port.dat_to_noc.dat[31:0] + 1];
            reply.dat[23:16]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 2];
            reply.dat[31:24]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 3];
            reply.dat[39:32]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 4];
            reply.dat[47:40]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 5];
            reply.dat[55:48]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 6];
            reply.dat[63:56]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 7];

            reply.dat[71:64]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 8];
            reply.dat[79:72]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 9];
            reply.dat[87:80]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 10];
            reply.dat[95:88]    <=  memory[noc_port.dat_to_noc.dat[31:0] + 11];
            reply.dat[103:96]   <=  memory[noc_port.dat_to_noc.dat[31:0] + 12];
            reply.dat[111:104]  <=  memory[noc_port.dat_to_noc.dat[31:0] + 13];
            reply.dat[119:112]  <=  memory[noc_port.dat_to_noc.dat[31:0] + 14];
            reply.dat[127:120]  <=  memory[noc_port.dat_to_noc.dat[31:0] + 15];

            reply.dst_addr <= noc_port.dat_to_noc.src_addr;
            reply.dst_prt <= noc_port.dat_to_noc.src_prt;

            reply.src_addr <= 1;
            reply.src_prt <= 0;

            // Signal that a tx is ready to be sent back to instruction fetch unit.
            tx_send <= 1;

            // Open the port to send the reply.
            reply_prt_stat <= port_open;
        end
    end
    // If there is nothing for us to reply to.
    else begin
        // Stop signalling that a reply is ready.
        tx_send <= 0;

        // Close the reply port.
        reply_prt_stat <= 0;
    end
end


integer i;
initial begin
    // Set NOC port initial conditions.
    req_prt_stat = port_open;

    // Setup an instruction in memory.

    // ADD R2, R2
    memory[0] = 8'b10000100;
    memory[1] = 8'b00100010;

    // ADD 24, R2
    memory[2] = 8'b10001100;
    memory[3] = 8'b00000010;
    memory[4] = 0;
    memory[5] = 0;
    memory[6] = 0;
    memory[7] = 24;

    //RLS R2, R3
    memory[8] = 8'b01010100;
    memory[9] = 8'b00110010;
    
    //MUL 10, R3
    memory[10] = 8'b11101100;
    memory[11] = 8'b00000011;
    memory[12] = 0;
    memory[13] = 0;
    memory[14] = 0;
    memory[15] = 10;

    //JMP 0
    memory[16] = 8'b00001110;
    memory[17] = 0;
    memory[18] = 0;
    memory[19] = 0;
    memory[20] = 0;
end
*/
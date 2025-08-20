`timescale 1ps/1ps

/*

Matmul app perf before cache:
    1ps/1ps
    Total run time = 7,036,075 ps
    Total core cycles = 703,607
    penalty for GOTO = 2560ps (256 core cycles).
    penalty for jump = 1200ps (120 core cycles).

Matmul app perf after cache:
    1ps/1ps
    Total run time = 516,525 ps
    Total core cycles = 51,652
    Penalty for GOTO = 101.167ps (10 core cycles).
    penalty for jump = 62.5049ps (6 core cycles).
*/

`include "defines.svh"
`include "structs.svh"


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
    memory[179:175]       = 40'b0000_0000_0000_0000_0000_0000_1011_1001_1000_1110;                     // JIZ (Jumps to infinite loop at 185 if i == 8).
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


logic [31:0]        mem_addr_sel;   // Memory address bus.
wire logic [127:0]  mem_dat;        // Memory data bus.
bit [127:0]         mem_dat_driver; // Memory data buffer.
logic               mem_we;         // memory write enable.
logic               mem_re;         // Memory read enable.
logic               mem_en;         // Memory enable.
logic               mclk;           // Memory clock.
soc sooc(rst, mem_addr_sel, mem_dat, mem_en, mem_we, mem_re, mclk);


// Handle the memory stuffs.
assign mem_dat = (mem_re) ? mem_dat_driver : 'hz;
always @(negedge mclk) begin
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

            //$display("Memory write request. %b %t", mem_addr_sel, $time);

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

            //$display("Memory read request. %b %t", mem_addr_sel, $time);

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
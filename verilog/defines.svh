/*          Instruction decode defines          */
`define INSTRUCTION_QUEUE_LENGTH 64
`define INSTRUCTION_QUEUE_INPUT_WIDTH 16

/*          Instruction Op-Codes            */

// ALU Operations.
`define INST_ADD_A_B 8'b1000_0100
`define INST_ADD_K_B 8'b1000_1100

`define INST_SUB_A_B 8'b0100_0100
`define INST_SUB_K_B 8'b0100_1100

`define INST_AND_A_B 8'b1100_0100
`define INST_AND_K_B 8'b1100_1100

`define INST_IOR_A_B 8'b0010_0100
`define INST_IOR_K_B 8'b0010_1100

`define INST_XOR_A_B 8'b1010_0100
`define INST_XOR_K_B 8'b1010_1100

`define INST_NOT_A_B 8'b0110_0100
`define INST_NOT_K_B 8'b0110_1100

`define INST_MUL_A_B 8'b1110_0100
`define INST_MUL_K_B 8'b1110_1100

`define INST_MOV_A_B 8'b0001_0100
`define INST_MOV_K_B 8'b0001_1100

`define INST_CMP_A_B 8'b1001_0100
`define INST_CMP_K_B 8'b1001_1100

`define INST_RLS_A_B 8'b0101_0100
`define INST_RRS_A_B 8'b1101_0100

// Memory & IO Operations
`define INST_LOD_K_B 8'b0000_0010
`define INST_STR_K_B 8'b1000_0010
`define INST_STR_A_B 8'b0100_1010
`define INST_LOD_A_B 8'b1100_1010

// Program Flow Operations
`define INST_JMP_K   8'b0000_1110
`define INST_JIZ_K   8'b1000_1110
`define INST_GOT_K   8'b0100_1110
`define INST_RET     8'b1100_0110



/*          Micro-Op Op-Codes           */

`define EXEC_UNIT_ALU 3'b001
`define EXEC_UNIT_LSU 3'b010
`define EXEC_UNIT_AGU 3'b011
`define EXEC_UNIT_PFC 3'b100

// ALU Operations
`define UOP_ALU_ADD    4'b0001
`define UOP_ALU_SUB    4'b0010
`define UOP_ALU_AND    4'b0011
`define UOP_ALU_IOR    4'b0100
`define UOP_ALU_XOR    4'b0101
`define UOP_ALU_NOT    4'b0110
`define UOP_ALU_MUL    4'b0111
`define UOP_ALU_MOV    4'b1000
`define UOP_ALU_RLS    4'b1001
`define UOP_ALU_RRS    4'b1010
`define UOP_ALU_CMP    4'b1011

// LSU Operations
`define UOP_LSU_STR    4'b0000
`define UOP_LSU_LOD    4'b0001

// AGU Operations
`define UOP_AGU_ADD    4'b0000
`define UOP_AGU_SUB    4'b0001

// Branching Operations
`define UOP_PFC_JMP 4'b0000
`define UOP_PFC_JIZ 4'b0001


/*          NOC address defines         */
`define GPIO_NOC_ADDR 3
`define GPIO_NOC_PORT 0

`define MEMORY_INTERFACE_NOC_ADDR 2
`define MEMORY_INTERFACE_NOC_PORT 0

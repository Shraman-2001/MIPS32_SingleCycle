`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.09.2025 19:28:19
// Design Name: 
// Module Name: mips32_processor
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mips32_processor (
    input wire clk,
    input wire reset,
    output wire [31:0] pc_out,
    output wire [31:0] instruction_out,
    output wire [31:0] alu_result_out
);

// Internal signals
wire [31:0] pc_current, pc_next, pc_plus4, pc_branch;
wire [31:0] instruction;
wire [31:0] reg_data1, reg_data2, write_data;
wire [31:0] sign_extend, alu_input2;
wire [31:0] alu_result, mem_read_data;
wire [4:0] write_reg;
wire reg_write, alu_src, mem_read, mem_write, mem_to_reg, branch, jump;
wire [3:0] alu_control;
wire zero_flag, branch_taken;

// Assign outputs
assign pc_out = pc_current;
assign instruction_out = instruction;
assign alu_result_out = alu_result;

// Program Counter
program_counter PC (
    .clk(clk),
    .reset(reset),
    .pc_next(pc_next),
    .pc_current(pc_current)
);

// PC Adder (+4)
assign pc_plus4 = pc_current + 4;

// Instruction Memory
instruction_memory IMEM (
    .address(pc_current[7:2]), // Word-aligned addressing
    .instruction(instruction)
);

// Control Unit
control_unit CTRL (
    .opcode(instruction[31:26]),
    .funct(instruction[5:0]),
    .reg_write(reg_write),
    .alu_src(alu_src),
    .mem_read(mem_read),
    .mem_write(mem_write),
    .mem_to_reg(mem_to_reg),
    .branch(branch),
    .jump(jump),
    .alu_control(alu_control)
);

// Register File
register_file REGFILE (
    .clk(clk),
    .reset(reset),
    .reg_write(reg_write),
    .read_reg1(instruction[25:21]),
    .read_reg2(instruction[20:16]),
    .write_reg(write_reg),
    .write_data(write_data),
    .read_data1(reg_data1),
    .read_data2(reg_data2)
);

// Sign Extend
sign_extend SIGNEXT (
    .input_16(instruction[15:0]),
    .output_32(sign_extend)
);

// ALU Input Selection
assign alu_input2 = alu_src ? sign_extend : reg_data2;

// ALU
alu ALU (
    .input1(reg_data1),
    .input2(alu_input2),
    .alu_control(alu_control),
    .result(alu_result),
    .zero(zero_flag)
);

// Data Memory
data_memory DMEM (
    .clk(clk),
    .mem_read(mem_read),
    .mem_write(mem_write),
    .address(alu_result[7:2]), // Word-aligned addressing
    .write_data(reg_data2),
    .read_data(mem_read_data)
);

// Write Register Selection (R-type vs I-type)
assign write_reg = (instruction[31:26] == 6'b000000) ? instruction[15:11] : instruction[20:16];

// Write Data Selection
assign write_data = mem_to_reg ? mem_read_data : alu_result;

// Branch Logic
assign pc_branch = pc_plus4 + (sign_extend << 2);
assign branch_taken = branch & zero_flag;

// Next PC Selection
assign pc_next = jump ? {pc_plus4[31:28], instruction[25:0], 2'b00} :
                (branch_taken ? pc_branch : pc_plus4);

endmodule
module program_counter (
    input wire clk,
    input wire reset,
    input wire [31:0] pc_next,
    output reg [31:0] pc_current
);

always @(posedge clk or posedge reset) begin
    if (reset)
        pc_current <= 32'h00000000;
    else
        pc_current <= pc_next;
end

endmodule
module instruction_memory (
    input wire [5:0] address,
    output wire [31:0] instruction
);

reg [31:0] memory [0:63]; // 64 words of instruction memory
integer i;
initial begin
    // Initialize with some test instructions
    memory[0][31:0] = 32'h20080005;  // addi $t0, $zero, 5
    memory[1][31:0] = 32'h20090007;  // addi $t1, $zero, 7
    memory[2][31:0] = 32'h012a5020;  // add $t2, $t1, $t2
    memory[3][31:0] = 32'h01095022;  // sub $t2, $t0, $t1
    memory[4][31:0] = 32'h01295024;  // and $t2, $t1, $t1
    memory[5][31:0] = 32'h01295025;  // or $t2, $t1, $t1
    memory[6][31:0] = 32'h8d0b0000;  // lw $t3, 0($t0)
    memory[7][31:0] = 32'had0b0004;  // sw $t3, 4($t0)
    memory[8][31:0] = 32'h110a0002;  // beq $t0, $t2, 2
    memory[9][31:0] = 32'h08000000;  // j 0
    
    // Initialize remaining memory to NOPs
    for (i = 10; i < 64; i = i + 1) begin
        memory[i][31:0] = 32'h00000000;
    end
end

assign instruction = memory[address];

endmodule
module control_unit (
    input wire [5:0] opcode,
    input wire [5:0] funct,
    output reg reg_write,
    output reg alu_src,
    output reg mem_read,
    output reg mem_write,
    output reg mem_to_reg,
    output reg branch,
    output reg jump,
    output reg [3:0] alu_control
);

// ALU Control Logic
always @(*) begin
    case (opcode)
        6'b000000: begin // R-type instructions
            case (funct)
                6'b100000: alu_control = 4'b0010; // add
                6'b100010: alu_control = 4'b0110; // sub
                6'b100100: alu_control = 4'b0000; // and
                6'b100101: alu_control = 4'b0001; // or
                6'b101010: alu_control = 4'b0111; // slt
                default:   alu_control = 4'b0010; // default to add
            endcase
        end
        6'b001000: alu_control = 4'b0010; // addi
        6'b100011: alu_control = 4'b0010; // lw
        6'b101011: alu_control = 4'b0010; // sw
        6'b000100: alu_control = 4'b0110; // beq
        default:   alu_control = 4'b0010; // default to add
    endcase
end

// Main Control Logic
always @(*) begin
    // Default values
    reg_write = 0;
    alu_src = 0;
    mem_read = 0;
    mem_write = 0;
    mem_to_reg = 0;
    branch = 0;
    jump = 0;
    
    case (opcode)
        6'b000000: begin // R-type
            reg_write = 1;
            alu_src = 0;
            mem_to_reg = 0;
        end
        6'b001000: begin // addi
            reg_write = 1;
            alu_src = 1;
            mem_to_reg = 0;
        end
        6'b100011: begin // lw
            reg_write = 1;
            alu_src = 1;
            mem_read = 1;
            mem_to_reg = 1;
        end
        6'b101011: begin // sw
            alu_src = 1;
            mem_write = 1;
        end
        6'b000100: begin // beq
            branch = 1;
        end
        6'b000010: begin // j
            jump = 1;
        end
    endcase
end

endmodule

module register_file (
    input wire clk,
    input wire reset,
    input wire reg_write,
    input wire [4:0] read_reg1,
    input wire [4:0] read_reg2,
    input wire [4:0] write_reg,
    input wire [31:0] write_data,
    output wire [31:0] read_data1,
    output wire [31:0] read_data2
);

reg [31:0] registers [0:31];
integer i;

// Initialize registers
always @(posedge reset) begin
    for (i = 0; i < 32; i = i + 1) begin
        registers[i] <= 32'h00000000;
    end
end

// Write operation
always @(posedge clk) begin
    if (reg_write && write_reg != 5'b00000) begin
        registers[write_reg] <= write_data;
    end
end

// Read operations (combinational)
assign read_data1 = (read_reg1 == 5'b00000) ? 32'h00000000 : registers[read_reg1];
assign read_data2 = (read_reg2 == 5'b00000) ? 32'h00000000 : registers[read_reg2];

endmodule

module alu (
    input wire [31:0] input1,
    input wire [31:0] input2,
    input wire [3:0] alu_control,
    output reg [31:0] result,
    output wire zero
);

always @(*) begin
    case (alu_control)
        4'b0000: result = input1 & input2;           // AND
        4'b0001: result = input1 | input2;           // OR
        4'b0010: result = input1 + input2;           // ADD
        4'b0110: result = input1 - input2;           // SUB
        4'b0111: result = (input1 < input2) ? 1 : 0; // SLT
        4'b1100: result = ~(input1 | input2);        // NOR
        default: result = 32'h00000000;
    endcase
end

assign zero = (result == 32'h00000000) ? 1'b1 : 1'b0;

endmodule

module sign_extend (
    input wire [15:0] input_16,
    output wire [31:0] output_32
);

assign output_32 = {{16{input_16}}, input_16};

endmodule

module data_memory (
    input wire clk,
    input wire mem_read,
    input wire mem_write,
    input wire [5:0] address,
    input wire [31:0] write_data,
    output wire [31:0] read_data
);

reg [31:0] memory [0:63]; // 64 words of data memory
integer i;
initial begin
    for ( i = 0; i < 64; i = i + 1) begin
        memory[i][31:0] = 32'h00000000;
    end
    // Initialize some test data
    memory[0][31:0] = 32'h12345678;
    memory[1][31:0] = 32'h9ABCDEF0;
    memory[2][31:0] = 32'h11111111;
    memory[3][31:0] = 32'h22222222;
end

always @(posedge clk) begin
    if (mem_write) begin
        memory[address] <= write_data;
    end
end

assign read_data = mem_read ? memory[address] : 32'h00000000;

endmodule


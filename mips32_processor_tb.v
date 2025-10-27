`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.09.2025 19:40:44
// Design Name: 
// Module Name: mips32_processor_tb
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


`timescale 1ns / 1ps

module mips32_processor_tb;

// Testbench signals
reg clk;
reg reset;
wire [31:0] pc_out;
wire [31:0] instruction_out;
wire [31:0] alu_result_out;

// Instantiate the processor
mips32_processor dut (
    .clk(clk),
    .reset(reset),
    .pc_out(pc_out),
    .instruction_out(instruction_out),
    .alu_result_out(alu_result_out)
);

// Clock generation
initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100MHz clock (10ns period)
end

// Test sequence
initial begin
    $display("Starting MIPS32 Processor Test");
    $display("Time\tPC\tInstruction\tALU Result");
    $display("----\t--\t-----------\t----------");
    
    // Initialize
    reset = 1;
    #20;
    reset = 0;
    
    // Run for several clock cycles
    repeat (20) begin
        @(posedge clk);
        #1; // Small delay for signal propagation
        $display("%0t\t%h\t%h\t%h", $time, pc_out, instruction_out, alu_result_out);
    end
    
    // Test specific scenarios
    $display("\nTesting Register File Contents:");
    display_registers();
    
    $display("\nTesting Memory Contents:");
    display_memory();
    
    $display("\nTest completed successfully!");
    $finish;
end

// Task to display register contents
task display_registers;
    integer i;
    begin
        $display("Register File Contents:");
        for (i = 0; i < 32; i = i + 1) begin
            if (dut.REGFILE.registers[i] != 0) begin
                $display("$%0d: %h", i, dut.REGFILE.registers[i]);
            end
        end
    end
endtask

// Task to display memory contents
task display_memory;
    integer i;
    begin
        $display("Data Memory Contents (non-zero):");
        for (i = 0; i < 64; i = i + 1) begin
            if (dut.DMEM.memory[i] != 0) begin
                $display("Mem[%0d]: %h", i, dut.DMEM.memory[i]);
            end
        end
    end
endtask

// Monitor critical signals
initial begin
    $monitor("Time: %0t | PC: %h | Inst: %h | RegWrite: %b | ALU: %h", 
             $time, pc_out, instruction_out, dut.reg_write, alu_result_out);
end

// Generate VCD file for waveform viewing
initial begin
    $dumpfile("mips32_processor.vcd");
    $dumpvars(0, mips32_processor_tb);
end

endmodule


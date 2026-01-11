`timescale 1ns/1ps

module tb_riscv_pipeline_top;

    reg clk;
    reg reset;

    // Instantiate DUT
    riscv_pipeline_top uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock: 10ns period
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Reset sequence
    initial begin
        reset = 1;
        #20;
        reset = 0;
    end

    // Load Instruction Memory (IMEM)
    initial begin
        $display("Loading IMEM...");
        $readmemh("imem.hex", uut.mem0.imem);
    end

    // Load Data Memory (DMEM)
    initial begin
        $display("Loading DMEM...");
        $readmemh("dmem.hex", uut.mem0.dmem);
    end

    // End simulation
    initial begin
        #2000;
        $display("Simulation finished.");
        $finish;
    end

endmodule



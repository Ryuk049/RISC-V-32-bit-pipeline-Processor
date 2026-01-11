// rv32i_mem.v
`timescale 1ns/1ps
module rv32i_mem #(
    parameter IMEM_WORDS = 1024,
    parameter DMEM_WORDS = 1024
)(
    input  wire              clk,
    input  wire              rst,
    // instruction memory interface (read-only)
    input  wire  [31:0]      imem_addr,
    output wire  [31:0]      imem_data,
    // data memory interface (read/write)
    input  wire              dmem_read,
    input  wire              dmem_write,
    input  wire  [31:0]      dmem_addr,
    input  wire  [31:0]      dmem_wdata,
    output reg   [31:0]      dmem_rdata
);
    // behavioral arrays
    reg [31:0] imem [0:IMEM_WORDS-1];
    reg [31:0] dmem [0:DMEM_WORDS-1];
    integer i;

    initial begin
        for (i = 0; i < IMEM_WORDS; i = i + 1) imem[i] = 32'h00000013; // NOP
        for (i = 0; i < DMEM_WORDS; i = i + 1) dmem[i] = 32'h0;
    end

    // IMEM read (combinational word aligned)
    assign imem_data = imem[imem_addr[11:2]];

    // DMEM read/write (behavioral synchronous write, combinational read of last stored)
    always @(posedge clk) begin
        if (dmem_write) begin
            dmem[dmem_addr[11:2]] <= dmem_wdata;
        end
        if (dmem_read) begin
            dmem_rdata <= dmem[dmem_addr[11:2]];
        end else begin
            dmem_rdata <= 32'b0;
        end
    end

endmodule


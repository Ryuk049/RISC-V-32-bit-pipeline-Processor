# RISC-V-32-bit-pipeline-Processor
This project implements a 5-stage pipelined RISC-V processor based on the RV32I base integer instruction set, designed from scratch using Verilog HDL.

The design follows a classic IF–ID–EX–MEM–WB pipeline architecture and includes complete hazard detection and resolution mechanisms.
The processor has been verified at RTL and synthesized to a gate-level netlist using Cadence Genus, following an industry-standard RTL-to-GDS preparation flow.

-->Pipeline Architecture

The processor consists of the following stages:

Instruction Fetch (IF) – Program counter update and instruction memory access

Instruction Decode (ID) – Register file read, immediate generation, and control decoding

Execute (EX) – ALU operations, branch decision, and forwarding logic

Memory Access (MEM) – Load/store operations using data memory

Write Back (WB) – Writes results back to the register file


--> Hazard Handling

The design correctly handles all major pipeline hazards:

 Data Hazards

RAW hazards resolved using forwarding paths (EX/MEM → EX, MEM/WB → EX)

Load-use hazards handled via pipeline stalling

 Control Hazards

Branch and jump instructions resolved in the EX stage

Incorrectly fetched instructions are flushed from the pipeline
pipeline

 Structural Hazards

Avoided by using separate instruction and data memories (Harvard-style architecture)


 --> RTL Modules

riscv_pipeline_top.v – Top-level pipeline integration

rv32i_alu.v – ALU supporting RV32I operations

rv32i_control.v – Main control unit

rv32i_regfile.v – 32×32-bit register file

rv32i_imm_gen.v – Immediate value generator

rv32i_forward_hazard.v – Forwarding and hazard detection unit

rv32i_mem.v – Instruction and data memory

schemetic of the 32 bit processor using vivado:
<img width="951" height="425" alt="image" src="https://github.com/user-attachments/assets/438d735d-8006-4e41-a5dd-4e5e3490e6e7" />

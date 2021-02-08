// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Synchronous dual-port SRAM register model
// This module is for simulation and small size SRAM.
// Implementing ECC should be done inside wrapper not this model.

module ram_1p #(
    parameter int busW     = 4,
    parameter int Width    = 32, // bit
    parameter int busWidth = busW*Width,
    parameter int Depth    = 128,
    parameter int Aw = $clog2(Depth)

) (
    input                    clk_i,
    input                    rst_ni,

    input                       req_i,
    input                       write_i,
    input        [Aw-1:0]       addr_i,
    input        [busWidth-1:0] wdata_bus_i,  
    
    output logic                rvalid_o,
    output logic [busWidth-1:0] rdata_bus_o,    
    
    // to spm
    input  logic             spm_req_i,
    input  logic [Aw-1:0]    spm_addr_i,
    output logic [Width-1:0] spm_data_o   
     
);
    
    logic [Width-1:0] storage [Depth] = '{default:0}; 
    
    always @(posedge clk_i) begin   // read/write in blocks of 4
        if (req_i) begin 
            if (write_i) begin  
                storage[{addr_i[Aw-1:2],2'b00}]     <= wdata_bus_i[31:0];
                storage[{addr_i[Aw-1:2],2'b01}]     <= wdata_bus_i[63:32];
                storage[{addr_i[Aw-1:2],2'b10}]     <= wdata_bus_i[95:64];
                storage[{addr_i[Aw-1:2],2'b11}]     <= wdata_bus_i[127:96];
            end
            // read
            rdata_bus_o[31:0]     <= storage[{addr_i[Aw-1:2],2'b00}];
            rdata_bus_o[63:32]    <= storage[{addr_i[Aw-1:2],2'b01}];
            rdata_bus_o[95:64]    <= storage[{addr_i[Aw-1:2],2'b10}];
            rdata_bus_o[127:96]   <= storage[{addr_i[Aw-1:2],2'b11}];               
        end
    end
    
    always @(posedge clk_i) begin
        if (spm_req_i) begin
          spm_data_o <= storage[spm_addr_i];
        end
    end
    
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
          rvalid_o <= '0;
        end else begin
          rvalid_o <= req_i;
        end
    end

    localparam DATA_FILE = "data.mem";
    initial begin
      $readmemh(DATA_FILE, storage,500);
    end       

endmodule
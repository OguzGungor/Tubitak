// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
import cache_def::*;

module top_artya7_100 (
    input               IO_CLK,
    input               IO_RST_N
    //output [3:0]        LED
);

    parameter int MEM_SIZE  = 32 * 1024 * 1024; // 32 MB
    parameter int Depth     = MEM_SIZE/4;
    parameter int Aw        = $clog2(Depth);  
    parameter logic [31:0] MEM_START = 32'h00000000;
    parameter logic [31:0] MEM_MASK  = MEM_SIZE-1;

  logic clk_sys, rst_sys_n;

  // Instruction connection so SRAM
  logic        instr_req;
  logic        instr_gnt;
  logic        instr_rvalid;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;

  // Data connection so SRAM
  logic        data_req;
  logic        data_gnt;
  logic        data_rvalid;
  logic        data_we;
  logic [31:0] data_addr;
  logic [31:0] data_wdata;
  logic [31:0] data_rdata;

  // SRAM arbiter
    logic [127:0] mem_rdata_bus;
    logic         mem_rvalid;
    logic [31:0]  mem_rdata;

  // spm related
  parameter int SPM_DEPTH     = 1024; 
  parameter int SPM_AW        = $clog2(SPM_DEPTH);
  
  // spm_lsu <-> spm_controller
  logic [31:0]  spm_addr_top; 
  logic         mem_spm_top;
  logic         spm_reg_top;
  logic         del_spm_top;
  logic         off_base_top;
  logic         data_base_top;
  logic         spm2_reg_top;
  
  logic [31:0]  spm_data_read;
  logic         spm_data_hit;
  logic         spm_data_miss;
  logic         spm_data_gnt;
  
  // spm_controller <-> mem
  logic             spm_mem_req;
  logic [31:0]      lspm_mem_data_ram;
  logic [31:0]      lspm_mem_addr_ctrl;
  logic [Aw-1:0]    lspm_mem_addr_ram;
  assign lspm_mem_addr_ram = {lspm_mem_addr_ctrl[Aw+1:2]};
   
  // spm_controller <-> spm1
  logic                 spm_wr_req;
  logic [31:0]          spm_wr_addr;
  logic [31:0]          spm_wr_data;
  logic                 spm_read_req;
  logic [31:0]          spm_read_addr;
  logic [31:0]          spm_read_data; 
  logic                 spm_read_data_valid; 
  
  // spm_controller <-> spm2
  logic                 spm2_wr_req;
  logic [31:0]          spm2_wr_addr;
  logic [31:0]          spm2_wr_data;
  logic                 spm2_read_req;
  logic [31:0]          spm2_read_addr;
  logic [31:0]          spm2_read_data; 
  logic                 spm2_read_data_valid;     
     
     spm_controller #(
      ) u_spm_ctrl (
         .clk_i                 ( clk_sys ),
         .rst_ni                ( rst_sys_n ),
        
         .core_rd_addr          ( spm_addr_top  ),    
         .mem_spm_i             ( mem_spm_top   ),         
         .spm_reg_i             ( spm_reg_top   ),       
         .del_spm_i             ( del_spm_top   ),
    .off_base_i ( off_base_top ),
                
         .core_data_o           ( spm_data_read ),        
         .core_hit_o            ( spm_data_hit  ),
         .core_gnt_o            ( spm_data_gnt  ),
         .core_miss_o           ( spm_data_miss ),         
                
         .mem_req_data_o        ( spm_mem_req ), 
         .mem_rd_addr_o         ( lspm_mem_addr_ctrl ),
         .mem_data_i            ( lspm_mem_data_ram ),

         .spm_rd_data_i         ( spm_read_data ),
         .spm_data_valid        ( spm_read_data_valid ),
         .spm_rd_req_o          ( spm_read_req ),    
         .spm_rd_addr_o         ( spm_read_addr ),        
         .spm_wr_req_o          ( spm_wr_req ),
         .spm_wr_addr_o         ( spm_wr_addr ),
         .spm_wr_data_o         ( spm_wr_data )        
          
      );  

     spm #(

      ) u_spm (
        .clk_i_w                ( clk_sys ),
        .clk_i_r                ( clk_sys ),         
        .rst_ni                 ( rst_sys_n ),
        
        .rd_req                 ( spm_read_req),
        .rd_addr                ( spm_read_addr),   
        .wr_req                 ( spm_wr_req ),
        .wr_addr                ( spm_wr_addr ),
        .mem_data_i             ( spm_wr_data ),
        .core_data_o            ( spm_read_data ),    
        .spm_rvalid_o           ( spm_read_data_valid)
         
      );    

  ibex_core #(
     .RV32E(0),
     .RV32M(1),
     .DmHaltAddr(32'h00000000),
     .DmExceptionAddr(32'h00000000)
  ) u_core (
     .clk_i                 (clk_sys),
     .rst_ni                (rst_sys_n),

     .test_en_i             ('b0),

     .core_id_i             (4'b0),
     .cluster_id_i          (6'b0),
     // First instruction executed is at 0x0 + 0x80
     .boot_addr_i           (32'h00000000),

     .instr_req_o           (instr_req),
     .instr_gnt_i           (instr_gnt),
     .instr_rvalid_i        (instr_rvalid),
     .instr_addr_o          (instr_addr),
     .instr_rdata_i         (instr_rdata),

     .data_req_o            (data_req),
     .data_gnt_i            (data_gnt),
     .data_rvalid_i         (data_rvalid),
     .data_we_o             (data_we),
     .data_be_o             (),
     .data_addr_o           (data_addr),
     .data_wdata_o          (data_wdata),
     .data_rdata_i          (data_rdata),
     .data_err_i            ('b0),

     .irq_i                 ('b0),
     .irq_id_i              ('b0),
     .irq_ack_o             (),
     .irq_id_o              (),

     .debug_req_i           ('b0),
     
     .spm_addr_o            ( spm_addr_top ),
     .mem_spm_o             ( mem_spm_top ),
     .spm_reg_o             ( spm_reg_top ),
     .del_spm_o             ( del_spm_top ),
    
    .off_base_o   (off_base_top),
    .data_base_o   (data_base_top),
  
  
    .spm2_reg_o   (spm2_reg_top), 
      
     .spm_data_i            ( spm_data_read ),        
     .spm_hit_i             ( spm_data_hit ),
     .spm_gnt_i             ( spm_data_gnt ),
     .spm_miss_i            ( spm_data_miss), 

     .fetch_enable_i        ('b1)
  );

    /*memory controller <-> memory*/
    logic               req_ctrl_mem;
    logic               write_ctrl_mem;
    logic   [31:0]      addr_ctrl_mem;
    logic   [127:0]     wdata_ctrl_mem;
    logic               rvalid_mem_ctrl;
    logic   [127:0]     rdata_mem_ctrl;
  
    cpu_req_type        cpu_req_top;     //CPU request input (CPU->cache)    
    mem_data_type       mem_data_top;    //memory response (memory->cache)   
    mem_req_type        mem_req_top;     //memory request (cache->memory)   
    cpu_result_type     cpu_res_top;     //cache result (cache->CPU) 

    cpu_req_type        cpu_req_top2;     //CPU request input (CPU->cache)    
    mem_data_type       mem_data_top2;    //memory response (memory->cache)   
    mem_req_type        mem_req_top2;     //memory request (cache->memory)   
    cpu_result_type     cpu_res_top2;     //cache result (cache->CPU) 


  ram_1p #(
    .Width(32),
    .Depth(MEM_SIZE / 4)
  ) u_ram (
    .clk_i     ( clk_sys        ),
    .rst_ni    ( rst_sys_n      ),
    
    .req_i          ( req_ctrl_mem      ),
    .write_i        ( write_ctrl_mem    ),
    .addr_i         ( addr_ctrl_mem     ),
    .wdata_bus_i    ( wdata_ctrl_mem    ),
    
    .rvalid_o       ( rvalid_mem_ctrl   ),
    .rdata_bus_o    ( rdata_mem_ctrl    ),
    
    .spm_req_i ( spm_mem_req ),
    .spm_addr_i( lspm_mem_addr_ram ),
    .spm_data_o( lspm_mem_data_ram )    
  );

 mem_controller #(
        .Width(32),
        .Depth(MEM_SIZE / 4)    
    ) u_mem_ctrl (
    
        .clk_i          ( clk_sys        ),
        .rst_ni         ( rst_sys_n      ),
                                 
        .data_req_i     ( data_req),            // from cpu         
        .data_addr_i    ( data_addr),           // from cpu        
        .data_we_i      ( data_we),
        .data_wdata_i   ( data_wdata),          // from cpu
        
        .data_rvalid_o  ( data_rvalid),         // to cpu
        .data_rdata_o   ( data_rdata),          // to cpu
        .data_gnt_o     ( data_gnt),
        
        .cpu_req_o      ( cpu_req_top ),    // to cache controller
        .cpu_res_i      ( cpu_res_top ),    // from cache controller   
        .mem_req_i      ( mem_req_top ),    // memory request from cache controller to memory       
        .mem_data_o     ( mem_data_top ),   // memory response from memory to cache controller   
 
        .conf_addr_i        ( spm_addr_top ),	
        .cache_conf_val_i   ( data_base_top ),
        .cpu_req_o2         ( cpu_req_top2 ),      // from mem controller to cache controller
        .cpu_res_i2         ( cpu_res_top2 ),      // from cache controller to mem controller	
        .mem_req_i2         ( mem_req_top2 ),      // memory request from cache controller to memory       
        .mem_data_o2        ( mem_data_top2 ),     // memory response from memory to cache controller   	 
 
        .req_mem_o          ( req_ctrl_mem ),          // to data memory
        .write_mem_o        ( write_ctrl_mem ),        // to data memory
        .addr_mem_o         ( addr_ctrl_mem ),         // to data memory
        .wdata_mem_bus_o    ( wdata_ctrl_mem  ),        // to data memory
        .rvalid_mem_i       ( rvalid_mem_ctrl ),       // from data memory
        .rdata_mem_bus_i    ( rdata_mem_ctrl )      // from data memory  
           
    );

    dm_cache_fsm  u_data_cache (

        .clk_i      ( clk_sys ),
        .rst_i      ( rst_sys_n ),
        
        .cpu_req    ( cpu_req_top ),     //CPU request input (CPU->cache)    
        .mem_data   ( mem_data_top ),    //memory response (memory->cache)   
        .mem_req    ( mem_req_top ),     //memory request (cache->memory)   
        .cpu_res    ( cpu_res_top )     //cache result (cache->CPU)

    );

    dm_cache_fsm2  u_data_cache2 (

        .clk_i      ( clk_sys ),
        .rst_i      ( rst_sys_n ),
        
        .cpu_req    ( cpu_req_top2 ),     //CPU request input (CPU->cache)    
        .mem_data   ( mem_data_top2 ),    //memory response (memory->cache)   
        .mem_req    ( mem_req_top2 ),     //memory request (cache->memory)   
        .cpu_res    ( cpu_res_top2 )     //cache result (cache->CPU)

    );
    
    
    instr_cache   u_instr_cache(
    
        .clk_i (clk_sys),
        .rst_ni (rst_sys_n),
        
        .instr_req_i    (instr_req),
        .instr_addr_i   (instr_addr),
        .instr_rvalid_o (instr_rvalid),
        .instr_gnt_o    (instr_gnt),    
        .instr_rdata_o  (instr_rdata)  
    );

    assign clk_sys = IO_CLK;
    assign rst_sys_n = IO_RST_N;
    
endmodule


/*
    EdgeSP CTRL and EdgeSPm
*/

module spm_controller #(
    parameter int Width         = 32,
    
    parameter int BLOCK_SIZE    = 32,                   // number of words in block/chunk
    parameter int BLOCK_SIZEw   = $clog2(BLOCK_SIZE),   
    parameter int OFF_CNT       = BLOCK_SIZE/2,
    parameter int OFF_CNTw      = $clog2(OFF_CNT),      
     
    parameter int SPM_CAP       = 4,                    // capacacity in terms of blocks
    parameter int SPM_CAPw      = $clog2(SPM_CAP),      // log2(16)=4 - log2(64)=6
    parameter int spm_Depth     = BLOCK_SIZE*SPM_CAP,                  // number of words in SPM    
    parameter int spm_Aw        = $clog2(spm_Depth),    // x bits to specify spm_Depth unique locations

    //parameter int mem_Aw        = $clog2(mem_Depth),
    
    parameter int BUFF_SIZE     = 32,                  // addr buff 
    parameter int BUFF_SIZEw    = $clog2(BUFF_SIZE)     // log2(1024)=10

) (
    input  logic            clk_i,
    input  logic            rst_ni,             // asynchronous reset active low    
    
    // Core (spm_lsu)
    input  logic    [31:0]  core_rd_addr,       // read address
    input  logic            mem_spm_i,          // use core_rd_addr to write data from mem
    input  logic            spm_reg_i,          // use core_rd_addr to read data from spm
    input  logic            del_spm_i,
    input  logic            off_base_i,
                   
    output logic    [31:0]  core_data_o,        // data read from spm
    output logic            core_hit_o,
    output logic            core_gnt_o,
    output logic            core_miss_o,
    
    // Mem
    output  logic           mem_req_data_o,   
    output  logic   [31:0]  mem_rd_addr_o,
    input   logic   [31:0]  mem_data_i,            
    input   logic           mem_data_val_i,
    
    // SPM1
    input   logic   [31:0]  spm_rd_data_i,
    input   logic           spm_data_valid,
    output  logic           spm_rd_req_o,    
    output  logic   [31:0]  spm_rd_addr_o,    
    output  logic           spm_wr_req_o,
    output  logic   [31:0]  spm_wr_addr_o,
    output  logic   [31:0]  spm_wr_data_o   
     
     
    );
    
    logic [31:0] spm_req_cnt;   
    logic [31:0] spm_hit_cnt;   
    logic [31:0] spm_miss_cnt;   
    
    //      SPMREG - read from spm     //
    logic [31:0]          addr1_prev = '0;
    logic [SPM_CAPw:0]    available       = '0;        // when this is > 0, free space available 
    logic [SPM_CAPw-1:0]  free_table_ind [SPM_CAP];    // shows the free indexes in Spm
    
    //      MEMSPM - write to spm      //   
    logic [BLOCK_SIZEw:0]   lspm_counter;                           
    logic [31:0]            mem_addr_tracer;                        // tracing the mem locs for read  
    logic [31:0]            spm1_addr_table [spm_Depth]     = '{default:32'hffffffff}; 
    logic [31:0]            lspm_addr;
    logic [31:0]            lspm_saved;    

    logic [31:0]    offset1;
    logic [31:0]    offset2;
   
    logic                   reloaded              = '0;
    integer i,k,j;

    logic [31:0]            spm0 [BUFF_SIZE]  = '{default:0}; // read address buffer for pending mem addresses
    logic [BUFF_SIZEw:0]    spm0_cnt          = '0;  

    logic [31:0] off_base_addr;
    logic [31:0]            spm1_read_temp;
    logic                   do_spm1_read;
    logic core1_gnt;
    logic core1_hit;
    logic [31:0] core1_data;    
        
    typedef enum logic [3:0] {L_IDLE, L_READ_OFF1, L_READ_OFF2, L_MEM_INFO, L_MEM_DATA, L_MEM_WRAP, L_SPM_INFO, L_SPM_DATA, L_WRAP } lspm_ctrl_state;
    lspm_ctrl_state lspm_sc, lspm_sn;
    typedef enum logic [1:0] {R_IDLE, R_CHECK, R_REQ, R_WB} rspm_ctrl_state;
    rspm_ctrl_state rspm_sc, rspm_sn;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin                              // reset pulled low
            lspm_sc         <= L_IDLE;
            rspm_sc         <= R_IDLE; 
            available       <= SPM_CAP;
            spm_req_cnt     <= '0;
            spm_hit_cnt     <= '0;
            spm_miss_cnt    <= '0;
        end else begin
            lspm_sc     <= lspm_sn;
            rspm_sc     <= rspm_sn; 
        end
    end 
    
    initial begin
        for ( i=0; i<SPM_CAP; i++) begin
                free_table_ind[i] = i;
        end        
    end 
         
    always @( off_base_i ) begin
        if ( off_base_i ) begin
            off_base_addr = core_rd_addr;   // base addr of the array "col_ind"
        end
    end
    
    //      SPMREG - DELSPM      //
    always @( del_spm_i, reloaded) begin
                   
        if ( reloaded ) begin
            for( i=0; i<(SPM_CAP-1); i++ ) begin
                free_table_ind[i] = free_table_ind[i+1];
            end      
            available = available - 1; 
        end 
        
        if ( del_spm_i ) begin
            for ( k=0; k<(spm_Depth); k+=BLOCK_SIZE ) begin
                if (  spm1_addr_table[k] == core_rd_addr ) begin
                    free_table_ind[available] = (k >> BLOCK_SIZEw);
                    available                 = available + 1;                          
                end
            end   
       end 
        
    end
    
     //    MEM to SPM  // 
    always @( mem_spm_i ) begin         
        if ( (mem_spm_i) && (spm0_cnt < BUFF_SIZE) ) begin  // spm0 = address buffer      
            spm0[spm0_cnt]  <= core_rd_addr;
            spm0_cnt        <= spm0_cnt + 1'b1;           
        end              
    end
   
    always @(lspm_sc,mem_spm_i) begin         
        case (lspm_sc)         
            L_IDLE: begin
                lspm_counter                = '0;
                reloaded = 1'b0;
                if ( (available > '0) && (spm0_cnt > '0) )  begin
                    mem_addr_tracer = spm0[0];          // oldest request in addr buffer
                    mem_req_data_o  = 1'b1;             // for offset1
                    mem_rd_addr_o   = mem_addr_tracer;                       
                    for( i=0; i<(BUFF_SIZE-1); i++ ) begin
                        spm0[i] = spm0[i+1];
                    end 
                    spm0_cnt        = spm0_cnt - 1;                                                       
                    lspm_sn         = L_READ_OFF1;                                                                                                                                               
                end else begin
                    lspm_sn         = L_WRAP; // refresh fsm for the cases where available goes 0->1 without new memspm               
                end                          
            end  
            L_READ_OFF1: begin
                offset1             = mem_data_i;   // row_ptr[i]
                mem_addr_tracer     = mem_addr_tracer + 3'b100;   
                mem_rd_addr_o       = mem_addr_tracer;                                    
                lspm_sn             = L_READ_OFF2;                
            end     
            L_READ_OFF2: begin
                offset2             = mem_data_i;   // row_ptr[i+1]
                if ( offset1 == offset2 ) begin     // node has no edges
                    lspm_sn     = L_IDLE;
                end else begin  // load node's edges to spm
                    mem_addr_tracer = off_base_addr + {offset1[29:0],2'b00};
                    lspm_addr       = free_table_ind[0] << BLOCK_SIZEw;
                    lspm_saved      = free_table_ind[0];  
                    reloaded = 1'b1;
                                      
                    lspm_sn         = L_MEM_INFO;
                end          
            end                                
            L_MEM_INFO: begin    
                if ( lspm_counter < BLOCK_SIZE ) begin  
                    reloaded = 1'b0;
                    mem_req_data_o              = 1'b1;
                    mem_rd_addr_o               = mem_addr_tracer; 
                    spm1_addr_table[lspm_addr]  = mem_addr_tracer;
                    lspm_sn                     = L_MEM_DATA;               // continue reading     
                end else begin
                    lspm_sn                     = L_WRAP;    
                    mem_req_data_o              = 1'b0;
                                   
                end                       
            end            
            L_MEM_DATA: begin                               // get data from mem & prepare new info                                                              
                spm_wr_req_o    = 1'b1;
                spm_wr_data_o   = mem_data_i;
                spm_wr_addr_o   = lspm_addr;
                mem_addr_tracer = mem_addr_tracer + 3'b100; // next word
                lspm_addr++;
                lspm_counter++;            
                lspm_sn              = L_MEM_INFO;                              
            end            
            L_WRAP: begin
                lspm_sn         = L_IDLE;
            end
        endcase
    end
    
    always @(rspm_sc, spm_reg_i) begin
    
        case (rspm_sc) 
            R_IDLE: begin
                do_spm1_read = '0;
                if ((spm_reg_i) && (core_rd_addr != addr1_prev)) begin
                    spm_req_cnt++;
                    for (j=0; j<spm_Depth; j++) begin
                        if ( core_rd_addr == spm1_addr_table[j] ) begin
                            spm1_read_temp      = j;
                            do_spm1_read        = 1'b1; // entry found in spm
                        end
                    end 
                    rspm_sn = R_REQ;                           
                end else begin
                    rspm_sn = R_IDLE;                           
                end
            end
            R_REQ: begin
                if (do_spm1_read) begin
                    core1_hit = 1'b1;  
                    spm_rd_req_o    = 1'b1;
                    spm_rd_addr_o   = spm1_read_temp;
                end else begin
                    core1_hit = 1'b1;     
                end
                rspm_sn = R_WB;                           
            end
            R_WB: begin
                if (do_spm1_read) begin
                    spm_hit_cnt++;
                    core1_gnt  = 1'b1; 
                    core1_data = spm_rd_data_i;    
                end else begin
                    spm_miss_cnt++;                
                    core1_gnt  = 1'b1; 
                    core1_data = 32'hf0f0f0f0;    
                end
                addr1_prev     = core_rd_addr;                
                rspm_sn = R_IDLE;                  
            end
        endcase
    
    end    
 
    assign core_hit_o   = core1_hit;
    assign core_gnt_o   = core1_gnt;
    assign core_data_o  = core1_data;
 
endmodule

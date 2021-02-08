
module spm_lsu (
    input               clk_i,
    input logic         rst_ni,         // asynchronous reset active low    
    
    // IF stage
    input logic         mem_spm,        // load to spm
    input logic         spm_reg,        // read from spm
    input logic         del_spm,        // delete spm
    input logic         off_base,       // used to configure SPM
    input logic         spm2_reg,    
         
    // EX-ID stage
    output logic            spm_data_valid_o,
    output logic            spm_data_miss_o,
    output logic    [31:0]  spm_data_o,
    input logic     [31:0]  alu_mem_addr,       // mem address computed at ALU   
    
    // spm_controller
    input logic     [31:0]  spm_data_i,         // read data from spm
    input logic             spm_data_hit_i, 
    input logic             spm_data_gnt_i,   
    input logic             spm_data_miss_i,    // unused
       
    output logic    [31:0]  spm_addr_o,         // read or write address to controller
    output logic            mem_spm_valid,
    output logic            spm_reg_valid,
    output logic            del_spm_valid,

    output logic            off_base_valid,
    output logic            data_base_valid,
    output logic            data_base_valid2,

    output logic            spm2_reg_valid  
            
    );  

    logic   [31:0]      addr = '0;
    logic               spm_reg_asserted = '0;
    logic   [31:0]      alu_addr_prev    = '0;
    logic   [1:0]       off_counter      = '0;
    logic   [31:0]      edge_spm_off_addr; 
                      
    always_ff @(posedge clk_i) begin
        if ( mem_spm ) begin                    // load to spm   
            spm_addr_o       <= alu_mem_addr;    

            mem_spm_valid    <= 1'b1;
            spm_reg_valid    <= 1'b0;
            del_spm_valid    <= 1'b0; 
            off_base_valid   <= 1'b0;
            spm2_reg_valid   <= 1'b0;       
            
            data_base_valid <= 1'b0;        
            data_base_valid2 <= 1'b0;              
                  
        end else if ( spm_reg ) begin          // read from spm 
            spm_reg_asserted <= 1'b1;
            mem_spm_valid    <= 1'b0;
            spm_reg_valid    <= 1'b1;
            del_spm_valid    <= 1'b0;
            off_base_valid   <= 1'b0;
            spm2_reg_valid   <= 1'b0;       
            
            data_base_valid <= 1'b0;        
            data_base_valid2 <= 1'b0;    
                             
        end else if ( del_spm ) begin
            spm_addr_o       <= alu_mem_addr;    
            mem_spm_valid    <= 1'b0;
            spm_reg_valid    <= 1'b0;
            del_spm_valid    <= 1'b1;
            off_base_valid   <= 1'b0;
            spm2_reg_valid   <= 1'b0;
                  
            data_base_valid <= 1'b0;        
            data_base_valid2 <= 1'b0;      
                            
        end else if ( off_base ) begin
        
                spm_addr_o       <= alu_mem_addr;    
                spm_reg_asserted <= 1'b0;
                mem_spm_valid    <= 1'b0;
                spm_reg_valid    <= 1'b0; 
                del_spm_valid    <= 1'b0;
                spm2_reg_valid   <= 1'b0;     
               
                if ( off_counter == 2'b00 ) begin
                    edge_spm_off_addr   <= alu_mem_addr;
                    off_base_valid      <= 1'b1;
                    data_base_valid     <= 1'b0;        
                    off_counter         <= off_counter + 1'b1;             
                end else if ( off_counter >= 2'b01 ) begin
                    if ( alu_mem_addr != edge_spm_off_addr ) begin 
                        off_base_valid      <= 1'b0;
                        data_base_valid     <= 1'b1;   
                    end     
                end
                
        end else if ( spm2_reg ) begin     
            spm_reg_asserted <= 1'b1;
            mem_spm_valid    <= 1'b0;
            spm_reg_valid    <= 1'b0; 
            del_spm_valid    <= 1'b0;
            off_base_valid   <= 1'b0;
            spm2_reg_valid   <= 1'b1;
            
            data_base_valid <= 1'b0;        
            data_base_valid2 <= 1'b0;                                    
        end else begin   
            spm_reg_asserted <= 1'b0;
            mem_spm_valid    <= 1'b0;
            spm_reg_valid    <= 1'b0; 
            del_spm_valid    <= 1'b0;
            off_base_valid   <= 1'b0;
            spm2_reg_valid   <= 1'b0; 
            
            data_base_valid <= 1'b0;        
            data_base_valid2 <= 1'b0;                               
        end
    end   
    
    typedef enum logic [1:0] {IDLE, WAIT_HIT, WAIT_GNT, WB} spm_lsu_state;
    spm_lsu_state spm_state, spm_statenext;   
    
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin                              // reset pulled low
            spm_state       <=  IDLE; 
        end else begin
            spm_state       <=  spm_statenext; 
        end
    end 
    
    always_comb begin
        case (spm_state) 
            IDLE: begin
                spm_data_miss_o     = 1'b0;
                spm_data_valid_o    = 1'b0;          
                if (spm_reg_asserted && (spm2_reg || spm_reg) ) begin
                    spm_addr_o      = alu_mem_addr;
                    spm_statenext   = WAIT_HIT;                   
                end else begin
                    spm_statenext   = IDLE;
                end
            end
            WAIT_HIT: begin    
                if (spm_data_hit_i && (spm2_reg || spm_reg) ) begin
                    spm_statenext   = WAIT_GNT;
                end else begin
                    spm_statenext   = WAIT_HIT;                  
                end
            end 
            WAIT_GNT: begin
                if (spm_data_gnt_i && (spm2_reg || spm_reg) ) begin
                    spm_statenext    = WB; 
                end else begin
                    spm_statenext    = WAIT_GNT;                   
                end                    
            end
            WB: begin
                alu_addr_prev    = alu_mem_addr;
                spm_data_o       =  spm_data_i;
                spm_data_valid_o =  1'b1; 
                spm_statenext    =  IDLE;
            end    
        endcase
    end
                
endmodule

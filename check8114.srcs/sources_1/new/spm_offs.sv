
module spm_offs #(
    parameter int Width             = 32, 
    parameter int spm_main_Depth    = 64,
    parameter int spm_Depth         = (spm_main_Depth/2),
    parameter int spm_Aw            = $clog2(spm_Depth)
) (
    input  logic                clk_i,
    input  logic                rst_ni,         // asynchronous reset active low      

    // spm_controller
    input  logic                    rd_req,
    input  logic    [31:0]          rd_addr,
    input  logic                    wr_req,
    input  logic    [31:0]          wr_addr,
    input  logic    [Width-1:0]     mem_data_i,     // data to write from mem
    
    output logic    [Width-1:0]     core_data_o,    // data read to core
    output logic                    spm_rvalid_o
               
    );
    
    logic [Width-1:0] spm2 [spm_Depth] = '{default:0}; 
    
    always @(posedge clk_i) begin
        if (rd_req) begin
            core_data_o <= spm2[rd_addr];
        end
    end 
       
    always @(posedge clk_i) begin
        if (wr_req) begin
            spm2[wr_addr] <= mem_data_i;
        end
    end 
        
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            spm_rvalid_o <= '0;
        end else begin
            spm_rvalid_o <= rd_req;
        end
    end
      
endmodule

module spm #(
    parameter int Width         = 32, 
    
    parameter int BLOCK_SIZE    = 32,                   // number of words in block/chunk
    parameter int BLOCK_SIZEw   = $clog2(BLOCK_SIZE),       
    
    parameter int spm_Depth     = BLOCK_SIZE*4,
    parameter int spm_Aw        = $clog2(spm_Depth)
) (
    input  logic                clk_i_w,
    input  logic                clk_i_r,    
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
    
    logic [Width-1:0] spm [spm_Depth] = '{default:0}; 
    
    always @(posedge clk_i_r) begin
        if (rd_req) begin
            core_data_o <= spm[rd_addr];
        end
    end 
       
    always @(posedge clk_i_w) begin
        if (wr_req) begin
            spm[wr_addr] <= mem_data_i;
        end
    end 
        
    always_ff @(posedge clk_i_r or negedge rst_ni) begin
        if (!rst_ni) begin
            spm_rvalid_o <= '0;
        end else begin
            spm_rvalid_o <= rd_req;
        end
    end
      
endmodule

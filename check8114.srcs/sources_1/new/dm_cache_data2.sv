`timescale 1ns / 1ps
import cache_def::*;

/*cache: data memory, single port, 64 blocks*/

module dm_cache_data2(
    input  clk_i,
    input  cache_req_type  data_req,    //data request/command, e.g. RW, valid
    input  cache_data_type data_write,  //write port (128-bit line)     
    output cache_data_type data_read
     ); //read port   

    //cache_data_type data_mem[0:1023] = '{default:0}; // 1024 blocks
    
    cache_data_type data_mem[0:31];
    initial  begin    
        for (int i=0; i<31; i++)           
            data_mem[i] = 128'heeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee;  
    end
    
    assign data_read = data_mem[data_req.index];
    
    always_ff @(posedge(clk_i)) 
    begin    
        if  (data_req.we)      
            data_mem[data_req.index] <= data_write;  
    end
    
endmodule


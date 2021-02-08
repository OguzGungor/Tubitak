`timescale 1ns / 1ps
import cache_def::*;

/*cache: tag memory, single port, 64 blocks*/

module dm_cache_tag2(
    input  clk_i,                       //write clock
    input  cache_req_type tag_req,      //tag request/command, e.g. RW, valid
    input  cache_tag_type tag_write,    //write port        
    output cache_tag_type tag_read      //read port    
    );

    cache_tag_type tag_mem[0:31];
       
    initial  begin      
        for (int i=0; i<31; i++)       
            tag_mem[i] = '0;  
    end  
    
    assign tag_read = tag_mem[tag_req.index];

    always_ff @(posedge(clk_i)) 
    begin 
        if  (tag_req.we)      
            tag_mem[tag_req.index] <= tag_write;  
    end 
    
endmodule

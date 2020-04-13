//Clearable loadable enableable counter
module sch(clk,clk_out);

   input clk;
   output clk_out;

   reg  [7:0] a;
   always @(posedge clk)
     begin
        a<=a+1;
     end

   assign clk_out= a[7];

endmodule


module mult
#(parameter COEF_WIDTH = 24, parameter DATA_WIDTH = 16, parameter ADDR_WIDTH = 9, parameter MULT_WIDTH = COEF_WIDTH + DATA_WIDTH)
    (
    input   wire                                    clk,
    input   wire                                    en,
    input   wire            [ (ADDR_WIDTH-1) :  0 ] ad,
    input   wire    signed  [ (COEF_WIDTH-1) :  0 ] coe,
    input   wire    signed  [ (DATA_WIDTH-1) :  0 ] pip,
    output  wire    signed  [ (DATA_WIDTH-1) :  0 ] dout
    );

wire signed [(MULT_WIDTH-1) :  0 ] mu = coe * pip;

reg signed [ (MULT_WIDTH-1) :  0 ] rac = {(MULT_WIDTH){1'b0}};
reg signed [ (DATA_WIDTH-1) :  0 ] ro = {DATA_WIDTH{1'b0}};

assign dout = ro;

always @(posedge clk)
if(en)
    if(ad == {ADDR_WIDTH{1'b0}})
    begin
        rac <= mu;
        ro <= rac[ (MULT_WIDTH-2) -: (DATA_WIDTH) ];
    end
    else
        rac <= rac + mu;

endmodule


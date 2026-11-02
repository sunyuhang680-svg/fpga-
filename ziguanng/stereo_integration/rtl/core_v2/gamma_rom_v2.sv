// Generated, quantized Q8 Gamma=128..512, 256 entries per candidate.
// Ship gamma_q8_v2.hex with the synthesis project. One synchronous ROM port.
module gamma_rom_v2 #(parameter FILE="gamma_q8_v2.hex")(
    input wire clk,input wire[16:0] addr,output reg[7:0] data
);
    reg[7:0] table_data[0:98559];
    initial $readmemh(FILE,table_data);
    always @(posedge clk) data<=table_data[addr];
endmodule

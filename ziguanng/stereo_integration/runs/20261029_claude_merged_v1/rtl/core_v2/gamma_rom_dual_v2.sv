// One Gamma table for both eyes: two synchronous read ports on one clock.
// Same contents/latency as gamma_rom_v2 (one register stage per port).
module gamma_rom_dual_v2 #(parameter FILE="gamma_q8_v2.hex")(
    input wire clk,input wire[16:0] addr_a,addr_b,output reg[7:0] data_a,data_b
);
    reg[7:0] table_data[0:98559];
    initial $readmemh(FILE,table_data);
    always @(posedge clk) data_a<=table_data[addr_a];
    always @(posedge clk) data_b<=table_data[addr_b];
endmodule

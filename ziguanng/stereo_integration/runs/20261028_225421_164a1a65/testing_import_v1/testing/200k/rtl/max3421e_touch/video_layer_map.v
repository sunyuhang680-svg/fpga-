// SPDX-License-Identifier: MIT
// Pixel compositor. One cycle of latency applied to RGB and all timing equally.
module video_layer_map (
    input wire clk, rst_n,
    input wire [23:0] camera_rgb, menu_rgb,
    input wire camera_de, camera_hs, camera_vs, menu_valid,
    output reg [23:0] video_rgb,
    output reg video_de, video_hs, video_vs
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin video_rgb<=0;video_de<=0;video_hs<=0;video_vs<=0; end
        else begin
            video_de<=camera_de;video_hs<=camera_hs;video_vs<=camera_vs;
            video_rgb<=!camera_de?24'd0:(menu_valid?menu_rgb:camera_rgb);
        end
    end
endmodule

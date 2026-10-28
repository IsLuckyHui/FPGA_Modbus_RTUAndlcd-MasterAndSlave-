`timescale  1ns/1ns 

module tb_lcd_driver();

reg        lcd_pclk     ;
reg        rst_n        ;
reg [23:0] pixel_data   ;
reg [15:0] lcd_id       ;

wire [10:0] pixel_xpos ;
wire [10:0] pixel_ypos ;
wire [10:0] h_disp     ;
wire [10:0] v_disp     ;
wire        data_req   ;
wire        lcd_de     ;
wire        lcd_hs     ;
wire        lcd_vs     ;
wire        lcd_bl     ;
wire        lcd_clk    ;
wire        lcd_rst    ;
wire [23:0] lcd_rgb    ;
initial begin

    lcd_pclk<= 1'b0;
    rst_n <= 1'b0;
    pixel_data <= 24'hffffff;
    lcd_id <= 16'h4384;
    
    #200
    rst_n <= 1'b1;

end 

always#10 lcd_pclk = ~lcd_pclk;

lcd_driver u_lcd_driver
(
    .lcd_pclk        (lcd_pclk   ),
    .rst_n           (rst_n      ),
    .pixel_data      (pixel_data ),
    .lcd_id          (lcd_id     ),
    .pixel_xpos      (pixel_xpos),
    .pixel_ypos      (pixel_ypos),
    .h_disp          (h_disp    ),
    .v_disp          (v_disp    ),
    .data_req        (data_req  ),
    .lcd_de          (lcd_de    ),
    .lcd_hs          (lcd_hs    ),
    .lcd_vs          (lcd_vs    ),
    .lcd_bl          (lcd_bl    ),
    .lcd_clk         (lcd_clk   ),
    .lcd_rst         (lcd_rst   ),
    .lcd_rgb         (lcd_rgb   ) 
);


endmodule
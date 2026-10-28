module lcd_rgb_colorbar
(
    input       sys_clk     ,
    input       sys_rst_n   ,
    
    
    //RGB LCD 接口
    output      lcd_de      , //LCD使能信号
    output      lcd_hs      , //LCD行同步信号 LCD 一行一行画像素。每扫完一整行，`lcd_hs` 产生一个脉冲，告诉屏幕：**准备开始扫描下一行**。
    output      lcd_vs      , //LCD列同步信号 一帧画面由很多行组成。当完整扫完**一整帧所有行**，`lcd_vs`发一个脉冲，告诉屏幕：**回到画面顶部，准备重新画新一帧图像**。
    output      lcd_bl      , //LCD背光
    output      lcd_clk     , //LCD时钟
    output      lcd_rst     , //LCD复位
    inout [23:0]lcd_rgb       //LCD RGB888 颜色数据
);

//wire define
wire    [15:0] lcd_id       ; //读取的LCD的ID 用来判断分辨率是多少
wire           lcd_pclk     ; //LCD屏幕的像素时钟
wire    [10:0] piexl_xpos   ; //当前像素点的X位置
wire    [10:0] piexl_ypos   ; //当前像素点的Y位置
wire    [10:0] h_disp       ; //水平分辨率 例如800
wire    [10:0] v_disp       ; //垂直分辨率 例如480
wire    [23:0] piexl_data   ; //像素数据
wire    [23:0] lcd_rgb_o    ; //输入的像素数据
wire    [23:0] lcd_rgb_i    ; //输出的像素数据


assign lcd_rgb = lcd_de ? lcd_rgb_o :24'bz;
assign lcd_rgb_i = lcd_rgb;

//读 LCD ID 模块
rd_id u_rd_id
(
    .clk          (sys_clk)  ,
    .rst_n        (sys_rst_n),
    .led_rgb      (lcd_rgb_i),
    .lcd_id       (lcd_id)  
);

//时钟分频模块
clk_div u_clk_div
(
    .clk           (sys_clk),
    .rst_n         (sys_rst_n),
    .lcd_id        (lcd_id),
    .lcd_pclk      (lcd_pclk)
);

//LCD 显示模块
lcd_display u_lcd_display
(
   .lcd_plck        (lcd_pclk),
   .rst_n           (sys_rst_n),
   .pixel_xpos      (piexl_xpos),
   .pixel_ypos      (piexl_ypos),
   .h_disp          (h_disp),
   .v_disp          (v_disp),
   .pixel_data      (piexl_data)
);

//LCD 驱动模块
lcd_driver lcd_driver
(
    .lcd_pclk       (lcd_pclk),
    .rst_n          (sys_rst_n),
    .lcd_id         (lcd_id),
    .pixel_data     (piexl_data),
    .pixel_xpos     (piexl_xpos),
    .pixel_ypos     (piexl_ypos),
    .h_disp         (h_disp),
    .v_disp         (v_disp),
    .data_req       (),
    .lcd_de         (lcd_de ),
    .lcd_hs         (lcd_hs ),
    .lcd_vs         (lcd_vs ),
    .lcd_bl         (lcd_bl ),
    .lcd_clk        (lcd_clk),
    .lcd_rst        (lcd_rst),
    .lcd_rgb        (lcd_rgb_o)
);


endmodule
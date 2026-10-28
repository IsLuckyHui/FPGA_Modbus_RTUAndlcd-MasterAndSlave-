// ==========================================================================
// 模块名称 : clk_div
// 功能描述 : LCD像素时钟分频模块，根据rd_id输出的lcd_id，动态选择对应屏幕所需像素时钟lcd_pclk
//            基于输入系统时钟sys_clk，生成2分频(25M)、4分频(12.5M)时钟，多路时钟二选一输出
// 输入参数 : sys_clk    系统基准时钟
//            sys_rst_n  全局低电平复位
//            lcd_id     屏幕识别ID，来自rd_id模块
// 输出信号 : lcd_pclk   输出给LCD驱动的像素时钟
// 分频逻辑 :
//            lcd_pclk_25Mhz : sys_clk 2分频
//            lcd_pclk_12_5MHz : sys_clk 4分频
// 屏幕时钟映射表：
//            16'h4342 : 4.3寸 480*272   -> 12.5MHz
//            16'h7084 : 7寸 800*480     -> 25MHz
//            16'h7016 : 7寸 1024*600    -> sys_clk（原始系统时钟）
//            16'h4384 : 4.3寸 800*480   -> 25MHz
//            16'h1018 :10寸 1280*800    -> sys_clk（原始系统时钟）
//            default  : 输出低电平
// 后续优化可以将时钟的分频改为PLL或者MMCM去分配
// ==========================================================================

module clk_div(
    input             sys_clk   ,
    input             sys_rst_n ,
    input  [15:0]     lcd_id    ,
    
    output  reg       lcd_pclk    


);

reg   lcd_pclk_25Mhz;
reg   lcd_pclk_12_5MHz;
reg   cnt;

//系统时钟2分频
always@(posedge sys_clk or negedge  sys_rst_n)begin
    if(!sys_rst_n)
        lcd_pclk_25Mhz <= 1'b0;
    else  
        lcd_pclk_25Mhz <= ~lcd_pclk_25Mhz;
end 

//系统时钟4分频
always@(posedge sys_clk or negedge sys_rst_n)begin
    if(!sys_rst_n)begin
        lcd_pclk_12_5MHz <= 1'b0;
        cnt <= 1'b0;
    end 
    else begin
        cnt <= cnt + 1'b1;
            if(cnt)
               lcd_pclk_12_5MHz <=  ~lcd_pclk_12_5MHz;
            else
               lcd_pclk_12_5MHz <=  lcd_pclk_12_5MHz; 
    end 

end 

//根据不同的lcd_id赋值不同的lcd_pclk
always@(*)begin
    case(lcd_id)
        16'h4342 : lcd_pclk = lcd_pclk_12_5MHz;
        16'h7084 : lcd_pclk = lcd_pclk_25Mhz;
        16'h7016 : lcd_pclk = sys_clk;
        16'h4384 : lcd_pclk = lcd_pclk_25Mhz;
        16'h1018 : lcd_pclk = sys_clk;
        default  : lcd_pclk = 1'b0;
    endcase
    
end



endmodule 
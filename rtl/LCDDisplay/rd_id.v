// ==========================================================================
// 模块名称 : rd_id
// 功能描述 : 屏幕ID识别模块，上电一次性读取pixel_data指定3bit识别屏幕型号并锁存ID值
//            从pixel_data中提取bit7、bit15、bit23拼接为3bit识别码，查表得到对应LCD屏幕ID
//            rd_flag标志保证仅上电采样一次，采样完成后lcd_id保持不变，不受后续像素数据流变化影响
// 识别编码 : {pixel_data[7],pixel_data[15],pixel_data[23]} 作为3bit屏幕识别码
// 屏幕对照表：
//            3'b000  : 16'h4342  //4.3寸 RGB LCD 分辨率480*272
//            3'b001  : 16'h7084  //7寸  RGB LCD 分辨率800*480
//            3'b010  : 16'h7016  //7寸  RGB LCD 分辨率1024*600
//            3'b100  : 16'h4384  //4.3寸 RGB LCD 分辨率800*480
//            3'b101  : 16'h1018  //10寸 RGB LCD 分辨率1280*800
//            default: 16'd0      //识别失败，ID输出0
// 设计要点 : rd_flag 一次性锁存标志。复位释放后执行一次查表赋值，拉高rd_flag，后续时钟不再更新lcd_id
// 注意事项 : 原逻辑上电第一个时钟直接采样，若pixel_data上电存在毛刺，存在读到错误ID风险；
//            可增加上电延时计数器，等待信号稳定后再采样作为优化方向。
// ==========================================================================
module rd_id
(
    input               sys_clk     ,
    input               sys_rst_n   ,
                        
    input       [23:0]  pixel_data  ,
    output  reg [15:0]  lcd_id
);
reg rd_flag; //保证只在上电读取一次 
always@(posedge sys_clk or negedge sys_rst_n)begin  
    if(!sys_rst_n)begin   
        lcd_id  <= 16'h0;
        rd_flag <= 1'b0;  
    end 
    else   begin
        if(!rd_flag)begin
            rd_flag <= 1'b1; 
            case({pixel_data[7],pixel_data[15],pixel_data[23]})
                3'b000  : lcd_id <= 16'h4342; //4.3' RGB LCD RES:480x272
                3'b001  : lcd_id <= 16'h7084; //7' RGB LCD RES:800x480
                3'b010  : lcd_id <= 16'h7016; //7' RGB LCD RES:1024x600
                3'b100  : lcd_id <= 16'h4384; //4.3' RGB LCD RES:800x480
                3'b101  : lcd_id <= 16'h1018; //10' RGB LCD RES:1280x800
                default : lcd_id <= 16'd0;
            endcase
        end 
    end 
end 
endmodule

// ==========================================================================
// 模块名称 : lcd_driver
// 功能描述 : 多分辨率RGB LCD驱动模块，DE模式，支持5款不同尺寸屏幕自适应时序
//            由rd_id模块输入lcd_id识别屏幕型号，自动加载对应屏幕行场时序参数
//            内部实现行计数器h_cnt、场计数器v_cnt，生成lcd_de数据使能、data_req像素请求信号
//            data_req向上游模块发起像素数据请求，上游返回pixel_data，输出到LCD屏幕RGB端口
// 输入信号 : lcd_pclk    LCD像素时钟，由clk_div模块根据lcd_id动态选择
//            rst_n       全局低电平复位
//            pixel_data  上游送入RGB888 24bit像素数据
//            lcd_id      屏幕识别ID，来自rd_id模块，用来选择屏幕时序参数
// 输出信号 : pixel_xpos  当前有效像素X坐标
//            pixel_ypos  当前有效像素Y坐标
//            h_disp      当前屏幕水平有效分辨率
//            v_disp      当前屏幕垂直有效分辨率
//            data_req    像素数据请求信号，高电平代表当前周期需要输入像素
//            lcd_de      LCD数据使能(DE模式核心信号)
//            lcd_hs      行同步信号，DE模式下固定高
//            lcd_vs      场同步信号，DE模式下固定高
//            lcd_bl      LCD背光控制，固定高，背光常亮 
//            lcd_clk     输出到LCD屏的像素时钟
//            lcd_rgb     输出给LCD的24bit RGB888像素数据
// 屏幕时序参数表：
//            16'h4342 : 4.3寸  480*272
//            16'h7084 : 7寸    800*480
//            16'h7016 : 7寸    1024*600
//            16'h4384 : 4.3寸  800*480
//            16'h1018 : 10.1寸 1280*800
// 设计要点 :
//            1. DE显示模式，不依赖HS/VS同步时序，仅使用DE信号标记有效图像区域
//            2. 组合逻辑根据lcd_id动态切换行场同步、前后沿、总扫描周期参数
//            3. h_cnt行像素计数器，每个lcd_pclk自增；h_cnt计满后v_cnt场计数器+1
//            4. data_req提前两拍发起像素请求，用来给上游像素缓存留出准备时间
//            5. lcd_de滞后data_req一拍，对齐像素输出时序
// 注意事项 & 可优化点：
//            1. lcd_id上电一次性锁定，运行过程不建议动态切换lcd_id，否则时序参数动态变化会造成画面撕裂
//            2. 计数器h_cnt/v_cnt使用组合参数，时序参数变化时，要保证计数器不会出现亚稳态/跳变；
//            3. 本设计简易实现，未增加帧缓存；如果摄像头输入与LCD像素时钟域不同，需要增加FIFO做跨时钟域图像缓存
// ==========================================================================

module lcd_driver(
    input                lcd_pclk    ,//根据clk_div模块 获取到的此屏幕对应的时钟
    input                rst_n       ,
                                     
    input        [23:0]  pixel_data  ,//传入的像素数据 RGB888
    input        [15:0]  lcd_id      ,//根据rd_id模块获取到的lcd_id 共四个字节
                                      
    output   reg [10:0]  pixel_xpos  ,//当前像素的X坐标
    output   reg [10:0]  pixel_ypos  ,//当前像素的Y坐标
    output   reg [10:0]  h_disp     ,//LCD屏幕的水平分辨率
    output   reg [10:0]  v_disp     ,//LCD屏幕的垂直分辨率
    output   reg         data_req   ,//数据请求信号

    //RGB LCD接口
    output   reg         lcd_de     ,//LCD数据使能型号
    output               lcd_hs     ,//LCD行同步信号   因为使用了DE模式 所以固定为1
    output               lcd_vs     ,//LCD列同步信号   因为使用了DE模式 所以固定为1
    output               lcd_bl     ,//LCD的背光控制信号  不做多的处理 固定为1
    output               lcd_clk    ,//LCD像素时钟
    output               lcd_rst    ,//LCD复位信号
    output       [23:0]  lcd_rgb     //输出的LCD RGB888颜色数据

);

//parameter define  
// 4.3' 480*272
parameter  H_SYNC_4342   =  11'd41;     //行同步
parameter  H_BACK_4342   =  11'd2;      //行显示后沿
parameter  H_DISP_4342   =  11'd480;    //行有效数据
parameter  H_FRONT_4342  =  11'd2;      //行显示前沿
parameter  H_TOTAL_4342  =  11'd525;    //行扫描周期
   
parameter  V_SYNC_4342   =  11'd10;     //场同步
parameter  V_BACK_4342   =  11'd2;      //场显示后沿
parameter  V_DISP_4342   =  11'd272;    //场有效数据
parameter  V_FRONT_4342  =  11'd2;      //场显示前沿
parameter  V_TOTAL_4342  =  11'd286;    //场扫描周期
   
// 7' 800*480   
parameter  H_SYNC_7084   =  11'd128;    //行同步
parameter  H_BACK_7084   =  11'd88;     //行显示后沿
parameter  H_DISP_7084   =  11'd800;    //行有效数据
parameter  H_FRONT_7084  =  11'd40;     //行显示前沿
parameter  H_TOTAL_7084  =  11'd1056;   //行扫描周期
   
parameter  V_SYNC_7084   =  11'd2;      //场同步
parameter  V_BACK_7084   =  11'd33;     //场显示后沿
parameter  V_DISP_7084   =  11'd480;    //场有效数据
parameter  V_FRONT_7084  =  11'd10;     //场显示前沿
parameter  V_TOTAL_7084  =  11'd525;    //场扫描周期       
   
// 7' 1024*600   
parameter  H_SYNC_7016   =  11'd20;     //行同步
parameter  H_BACK_7016   =  11'd140;    //行显示后沿
parameter  H_DISP_7016   =  11'd1024;   //行有效数据
parameter  H_FRONT_7016  =  11'd160;    //行显示前沿
parameter  H_TOTAL_7016  =  11'd1344;   //行扫描周期
   
parameter  V_SYNC_7016   =  11'd3;      //场同步
parameter  V_BACK_7016   =  11'd20;     //场显示后沿
parameter  V_DISP_7016   =  11'd600;    //场有效数据
parameter  V_FRONT_7016  =  11'd12;     //场显示前沿
parameter  V_TOTAL_7016  =  11'd635;    //场扫描周期
   
// 10.1' 1280*800   
parameter  H_SYNC_1018   =  11'd10;     //行同步
parameter  H_BACK_1018   =  11'd80;     //行显示后沿
parameter  H_DISP_1018   =  11'd1280;   //行有效数据
parameter  H_FRONT_1018  =  11'd70;     //行显示前沿
parameter  H_TOTAL_1018  =  11'd1440;   //行扫描周期
   
parameter  V_SYNC_1018   =  11'd3;      //场同步
parameter  V_BACK_1018   =  11'd10;     //场显示后沿
parameter  V_DISP_1018   =  11'd800;    //场有效数据
parameter  V_FRONT_1018  =  11'd10;     //场显示前沿
parameter  V_TOTAL_1018  =  11'd823;    //场扫描周期

// 4.3' 800*480   
parameter  H_SYNC_4384   =  11'd128;    //行同步
parameter  H_BACK_4384   =  11'd88;     //行显示后沿
parameter  H_DISP_4384   =  11'd800;    //行有效数据
parameter  H_FRONT_4384  =  11'd40;     //行显示前沿
parameter  H_TOTAL_4384  =  11'd1056;   //行扫描周期
   
parameter  V_SYNC_4384   =  11'd2;      //场同步
parameter  V_BACK_4384   =  11'd33;     //场显示后沿
parameter  V_DISP_4384   =  11'd480;    //场有效数据
parameter  V_FRONT_4384  =  11'd10;     //场显示前沿
parameter  V_TOTAL_4384  =  11'd525;    //场扫描周期    

//reg define
reg [10:0] h_sync   ;
reg [10:0] h_back   ;
reg [10:0] h_total  ;
reg [10:0] v_sync   ;
reg [10:0] v_back   ;
reg [10:0] v_total  ;
reg [10:0] h_cnt    ;
reg [10:0] v_cnt    ;

assign lcd_hs   = 1'b1      ;
assign lcd_vs   = 1'b1      ;
assign lcd_bl   = 1'b1      ;
assign lcd_rst  = rst_n     ;
assign lcd_rgb  = pixel_data;
assign lcd_clk  = lcd_pclk  ; 

//赋值pixel_xpos
always@(posedge lcd_pclk or negedge rst_n)begin
    if(!rst_n)
        pixel_xpos <= 11'd0;
    else begin
        if(data_req)
            pixel_xpos <= h_cnt + 11'd2 - h_sync - h_back;
        else
            pixel_xpos <= 11'd0;
    end 
end 

//赋值pixel_ypos
always@(posedge lcd_pclk or negedge rst_n)begin
    if(!rst_n)
        pixel_ypos <= 11'd0;
    else if((v_cnt >= v_sync + v_back ) && (v_cnt < v_sync + v_back + v_disp ))
        pixel_ypos <= v_cnt + 11'd2 - v_sync - v_back;
    else
        pixel_ypos <= 11'd0;
     
end 

//根据lcd_id赋值当前屏幕的消隐区间和显示区间
always@(*)begin
    case(lcd_id)
        16'h4342 : begin
            h_sync  = H_SYNC_4342; 
            h_back  = H_BACK_4342; 
            h_disp  = H_DISP_4342; 
            h_total = H_TOTAL_4342;
            v_sync  = V_SYNC_4342; 
            v_back  = V_BACK_4342; 
            v_disp  = V_DISP_4342; 
            v_total = V_TOTAL_4342;            
        end
        16'h7084 : begin
            h_sync  = H_SYNC_7084; 
            h_back  = H_BACK_7084; 
            h_disp  = H_DISP_7084; 
            h_total = H_TOTAL_7084;
            v_sync  = V_SYNC_7084; 
            v_back  = V_BACK_7084; 
            v_disp  = V_DISP_7084; 
            v_total = V_TOTAL_7084;        
        end
        16'h7016 : begin
            h_sync  = H_SYNC_7016; 
            h_back  = H_BACK_7016; 
            h_disp  = H_DISP_7016; 
            h_total = H_TOTAL_7016;
            v_sync  = V_SYNC_7016; 
            v_back  = V_BACK_7016; 
            v_disp  = V_DISP_7016; 
            v_total = V_TOTAL_7016;            
        end
        16'h4384 : begin
            h_sync  = H_SYNC_4384; 
            h_back  = H_BACK_4384; 
            h_disp  = H_DISP_4384; 
            h_total = H_TOTAL_4384;
            v_sync  = V_SYNC_4384; 
            v_back  = V_BACK_4384; 
            v_disp  = V_DISP_4384; 
            v_total = V_TOTAL_4384;             
        end        
        16'h1018 : begin
            h_sync  = H_SYNC_1018; 
            h_back  = H_BACK_1018; 
            h_disp  = H_DISP_1018; 
            h_total = H_TOTAL_1018;
            v_sync  = V_SYNC_1018; 
            v_back  = V_BACK_1018; 
            v_disp  = V_DISP_1018; 
            v_total = V_TOTAL_1018;        
        end
        default : begin
            h_sync  = H_SYNC_4342; 
            h_back  = H_BACK_4342; 
            h_disp  = H_DISP_4342; 
            h_total = H_TOTAL_4342;
            v_sync  = V_SYNC_4342; 
            v_back  = V_BACK_4342; 
            v_disp  = V_DISP_4342; 
            v_total = V_TOTAL_4342;          
        end
    endcase
end 

//根据h_cnt和v_cnt控制data_req请求信号的拉高或者拉低
always@(posedge lcd_pclk or negedge rst_n)begin
    if(!rst_n)
        data_req <= 1'd0;
    else if((h_cnt >= h_sync + h_back - 11'd2) && (h_cnt < h_sync + h_back + h_disp - 11'd2)
          &&(v_cnt >= v_sync + v_back ) && (v_cnt < v_sync + v_back + v_disp ) )
        data_req <= 1'd1;
    else
        data_req <= 1'd0;
end 

//拉高lcd_de
always@(posedge lcd_pclk or negedge rst_n)begin
    if(!rst_n)
        lcd_de <= 1'd0;
    else
        lcd_de <= data_req;

end 

//h_cnt计时
always@(posedge lcd_pclk or negedge rst_n)begin 
    if(!rst_n)
        h_cnt <= 11'd0;
    else   if(h_cnt >= h_total - 11'd1)
        h_cnt <= 11'd0;
    else
        h_cnt <= h_cnt + 11'd1; 

end 

//v_cnt计时
always@(posedge lcd_pclk or negedge rst_n)begin 
    if(!rst_n)
        v_cnt <= 11'd0;
    else   begin
        if(h_cnt >= h_total - 11'd1)begin
            if(v_cnt >= v_total)
                v_cnt <= 11'd0;
            else
                v_cnt <= v_cnt + 11'd1;
        end 
        else
            v_cnt <= v_cnt;
    end 
   
end 




endmodule
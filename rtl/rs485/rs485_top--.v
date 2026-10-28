// ==========================================================================
// 模块名称 : rs485_top
// 功能描述 : Modbus RTU 从机顶层（响应帧不含CRC），适配Artix-7开发板
//            板载RS485收发器为自动方向控制(DE/RE由硬件自动翻转)，
//            因此顶层不需要方向控制引脚，只保留 rx/tx 两根数据线。
//            例化：rs485_rx(串口接收) + slave(协议从机) + rs485_tx(串口发送)
// 数据通路 : uart_rx -> rs485_rx -> 字节流 -> slave -> 响应字节 -> rs485_tx -> uart_tx
// 回环处理 : 自动方向电路发送期间接收器通常仍使能，会收到本机自己发出的
//            响应帧。这里在 tx_busy 期间屏蔽接收有效脉冲，防止从机把自己
//            的响应误当成新请求（请求帧结束后间隔4字节时间才会回复，
//            故屏蔽不会误杀正常请求）。
// ==========================================================================
module rs485_top(
    input       sys_clk     ,   //系统时钟50MHz
    input       sys_rst_n   ,   //异步复位，低有效
    input       uart_rx     ,   //RS485接收数据线（接收发器的RO）
    output      uart_tx     ,    //RS485发送数据线（接收发器的DI，DE自动控制）
    
      //RGB LCD 接口
    output      lcd_de      , //LCD使能信号
    output      lcd_hs      , //LCD行同步信号 LCD 一行一行画像素。每扫完一整行，`lcd_hs` 产生一个脉冲，告诉屏幕：**准备开始扫描下一行**。
    output      lcd_vs      , //LCD列同步信号 一帧画面由很多行组成。当完整扫完**一整帧所有行**，`lcd_vs`发一个脉冲，告诉屏幕：**回到画面顶部，准备重新画新一帧图像**。
    output      lcd_bl      , //LCD背光
    output      lcd_clk     , //LCD时钟
    output      lcd_rst     , //LCD复位
    inout [23:0]lcd_rgb       //LCD RGB888 颜色数据
);

//============================== 参数定义 ====================================
parameter   UART_BPS = 115200;       //串口波特率
parameter   CLK_FREQ = 50000000;     //系统时钟频率

//============================== 内部连线 ====================================
//RS485
wire [7:0]  rx_byte;        //rs485_rx 解出的接收字节
wire        byte_done;      //rs485_rx 接收字节有效脉冲（1个时钟周期）
wire [7:0]  resp_byte;      //slave 输出的响应字节
wire        resp_en;        //slave 发送启动脉冲
wire        tx_busy;        //rs485_tx 发送忙标志
//发送期间屏蔽接收：既过滤掉自动方向电路的发送回环，也过滤总线冲突毛刺
wire        byte_valid = byte_done & (~tx_busy);


//LCD
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

//==========================================================================
// 串口接收：串行bit流 -> 8位字节 + 字节有效脉冲
//==========================================================================
rs485_rx #(
    .UART_BPS (UART_BPS),
    .CLK_FREQ (CLK_FREQ)
) u_rs485_rx(
    .sys_clk    (sys_clk   ),
    .sys_rst_n  (sys_rst_n ),
    .rx_data    (uart_rx   ),   //串行接收输入
    .tx_data    (rx_byte   ),   //并行字节输出
    .data_done  (byte_done )    //字节有效脉冲（屏蔽前）
);

//==========================================================================
// Modbus从机协议：解析03读请求，组装无CRC响应帧，逐字节送发送
//==========================================================================
slave #(
    .SLAVE_ADDR (8'h10     ),   //本机从机地址
    .UART_BPS   (UART_BPS  ),
    .CLK_FREQ   (CLK_FREQ  )
) u_slave(
    .sys_clk    (sys_clk   ),
    .sys_rst_n  (sys_rst_n ),
    .rx_data    (rx_byte   ),   //接收到的字节
    .start_en   (byte_valid),   //字节有效脉冲（已屏蔽发送回环）
    .tx_data    (resp_byte ),   //待发送的响应字节
    .tx_en      (resp_en   ),   //发送启动脉冲
    .tx_busy    (tx_busy   )    //发送忙反馈
);

//==========================================================================
// 串口发送：8位字节 -> 串行bit流（收发器方向由硬件自动控制）
//==========================================================================
rs485_tx #(
    .UART_BPS (UART_BPS),
    .CLK_FREQ (CLK_FREQ)
) u_rs485_tx(
    .clk        (sys_clk   ),
    .rst_n      (sys_rst_n ),
    .tx_data    (resp_byte ),   //待发送字节
    .tx_en      (resp_en   ),   //发送启动脉冲
    .uart_txd   (uart_tx   ),   //串行发送输出
    .tx_busy    (tx_busy   )    //发送忙标志（同时用于屏蔽接收回环）
);

//读 LCD ID 模块
rd_id u_rd_id
(
    .sys_clk          (sys_clk)  ,
    .sys_rst_n        (sys_rst_n),
    .pixel_data       (lcd_rgb_i),
    .lcd_id           (lcd_id)  
);

//时钟分频模块
clk_div u_clk_div
(
    .sys_clk           (sys_clk),
    .sys_rst_n         (sys_rst_n),
    .lcd_id            (lcd_id),
    .lcd_pclk          (lcd_pclk)
);

//LCD 显示模块
lcd_display u_lcd_display
(
   .lcd_pclk        (lcd_pclk),
   .rst_n           (sys_rst_n),
   .pixel_xpos      (piexl_xpos),
   .pixel_ypos      (piexl_ypos),
//   .h_disp          (h_disp),
//   .v_disp          (v_disp),
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

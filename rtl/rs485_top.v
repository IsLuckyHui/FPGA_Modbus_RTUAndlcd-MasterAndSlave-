// ==========================================================================
// 模块名称 : rs485_top
// 功能描述 : Modbus RTU 从机应答（响应帧不含CRC校验）基于Artix-7开发板
//            板载RS485收发器为自动收发方向(DE/RE由硬件自动控制)
//            故顶层不需要方向控制引脚，只需 rx/tx 两根数据线。
//            内部例化rs485_rx(串口接收) + slave(协议应答) + rs485_tx(串口发送)
// 数据通路 : uart_rx -> rs485_rx -> 字节流 -> slave -> 响应字节 -> rs485_tx -> uart_tx
// 关键逻辑 : 自动收发方向，因此在监听接收通路时使能发送会造成自发自收
//            响应帧发送期间 tx_busy 在每字节发送无效期间，防止从机接收自己
//            的响应。当本帧响应发送完毕后，等待4字节时间后恢复监听，
//            避免回环（自发自收引起的误触发）。
// ==========================================================================
module rs485_top(
    input       sys_clk     ,   //系统时钟50MHz
    input       sys_rst_n   ,   //异步复位，低有效
    input       uart_rx     ,   //RS485接收数据线，接收发器RO
    output      uart_tx     ,    //RS485发送数据线，接收发器DI（DE自动控制）
    
      //RGB LCD 接口
    output      lcd_de      , //LCD使能信号
    output      lcd_hs      , //LCD行同步信号 LCD一行一行地刷新。每扫描完一行，`lcd_hs` 产生一个脉冲，告诉屏幕**准备开始扫描下一行**。
    output      lcd_vs      , //LCD场同步信号 一帧图像由很多行组成。当扫描完**一整帧图像**后，`lcd_vs`产生一个脉冲，告诉屏幕**回到屏幕顶端，准备开始显示下一帧图像**。
    output      lcd_bl      , //LCD背光
    output      lcd_clk     , //LCD时钟
    output      lcd_rst     , //LCD复位
    inout [23:0]lcd_rgb       //LCD RGB888 颜色数据
);

//============================== 参数定义 ====================================
parameter   UART_BPS = 115200;       //串口波特率
parameter   CLK_FREQ = 50000000;     //系统时钟频率

//============================== 内部信号 ====================================
//RS485
wire [7:0]  rx_byte;        //rs485_rx 输出的接收字节
wire        byte_done;      //rs485_rx 输出字节有效脉冲（1个时钟周期宽）
wire [7:0]  resp_byte;      //slave 输出的响应字节
wire        resp_en;        //slave 输出响应字节使能
wire        tx_busy;        //rs485_tx 发送忙标志
//防止在发送期间接收回环，等发送结束后自动恢复方向的发送机会也会造成总线冲突
wire        byte_valid = byte_done & (~tx_busy);


//LCD
wire    [15:0] lcd_id       ; //读取的LCD的ID 判断分辨率是多少
wire           lcd_pclk     ; //LCD屏幕像素时钟
wire    [10:0] piexl_xpos   ; //当前像素的X位置
wire    [10:0] piexl_ypos   ; //当前像素的Y位置
wire    [10:0] h_disp       ; //水平分辨率 等于800
wire    [10:0] v_disp       ; //垂直分辨率 等于480
wire    [23:0] piexl_data   ; //像素数据
wire    [23:0] lcd_rgb_o    ; //输出像素数据
wire    [23:0] lcd_rgb_i    ; //输入像素数据


assign lcd_rgb = lcd_de ? lcd_rgb_o :24'bz;
assign lcd_rgb_i = lcd_rgb;

//==========================================================================
// 从机响应数据捕获：把 slave 逐字节送出的响应帧缓存下来，
// 提取出各寄存器值送给 LCD 显示。
// 响应帧格式: 地址(1) | 功能码(1) | 字节数n(1) | 寄存器数据(n, 高字节在前)
//==========================================================================
reg [7:0] resp_capture [0:15];  // 响应帧字节缓存
reg [4:0] resp_cnt;             // 已接收响应字节计数
reg [4:0] resp_total;           // 本帧响应总字节数（=3 + 字节数n）
integer   resp_i;               // 复位时清缓存用的循环变量

always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        resp_cnt   <= 5'd0;
        resp_total <= 5'd0;
        for(resp_i = 0; resp_i < 16; resp_i = resp_i + 1)
            resp_capture[resp_i] <= 8'd0;
    end
    else if(resp_en) begin
        resp_capture[resp_cnt] <= resp_byte;
        // 第3个字节(resp_cnt==2)是数据字节数n，总长度=3+n
        if(resp_cnt == 5'd2)
            resp_total <= 5'd3 + {1'b0, resp_byte[4:0]};
        resp_cnt <= resp_cnt + 1'b1;
    end
    else if(resp_cnt > 5'd0 && resp_total > 5'd0 && resp_cnt >= resp_total) begin
        // 一帧响应接收完毕，复位计数等待下一帧（缓存数据保持不变供LCD显示）
        resp_cnt   <= 5'd0;
        resp_total <= 5'd0;
    end
end

// 从响应帧中提取寄存器值（高字节在前，每寄存器占2字节）
// resp_capture[0]=地址  [1]=功能码  [2]=字节数  [3..]=寄存器数据
wire [15:0] reg_val0 = {resp_capture[3],  resp_capture[4]};
wire [15:0] reg_val1 = {resp_capture[5],  resp_capture[6]};
wire [15:0] reg_val2 = {resp_capture[7],  resp_capture[8]};
wire [15:0] reg_val3 = {resp_capture[9],  resp_capture[10]};
wire [15:0] reg_val4 = {resp_capture[11], resp_capture[12]};
wire [15:0] reg_val5 = {resp_capture[13], resp_capture[14]};

//==========================================================================
// 串口接收：串行bit流 -> 8位字节 + 字节有效脉冲
//==========================================================================
rs485_rx #(
    .UART_BPS (UART_BPS),
    .CLK_FREQ (CLK_FREQ)
) u_rs485_rx(
    .sys_clk    (sys_clk   ),
    .sys_rst_n  (sys_rst_n ),
    .rx_data    (uart_rx   ),   //接接收数据线
    .tx_data    (rx_byte   ),   //输出字节数据
    .data_done  (byte_done )    //字节有效脉冲（高电平有效
);

//==========================================================================
// Modbus从机协议：接收03请求，组装（无CRC）响应帧，逐字节送出
//==========================================================================
slave #(
    .SLAVE_ADDR (8'h10     ),   //本机从机地址
    .UART_BPS   (UART_BPS  ),
    .CLK_FREQ   (CLK_FREQ  )
) u_slave(
    .sys_clk    (sys_clk   ),
    .sys_rst_n  (sys_rst_n ),
    .rx_data    (rx_byte   ),   //接收到的字节
    .start_en   (byte_valid),   //字节有效脉冲（发送期间屏蔽回环）
    .tx_data    (resp_byte ),   //要发送的响应字节
    .tx_en      (resp_en   ),   //发送使能脉冲
    .tx_busy    (tx_busy   )    //发送忙标志
);

//==========================================================================
// 串口发送：8位字节 -> 串行bit流，接收发器DI（硬件自动控制方向）
//==========================================================================
rs485_tx #(
    .UART_BPS (UART_BPS),
    .CLK_FREQ (CLK_FREQ)
) u_rs485_tx(
    .clk        (sys_clk   ),
    .rst_n      (sys_rst_n ),
    .tx_data    (resp_byte ),   //要发送的字节
    .tx_en      (resp_en   ),   //发送使能脉冲
    .uart_txd   (uart_tx   ),   //输出发送数据线
    .tx_busy    (tx_busy   )    //发送忙标志，同时用于屏蔽接收回环
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
   .reg_val0        (reg_val0),
   .reg_val1        (reg_val1),
   .reg_val2        (reg_val2),
   .reg_val3        (reg_val3),
   .reg_val4        (reg_val4),
   .reg_val5        (reg_val5),
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

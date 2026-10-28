// ==========================================================================
// 模块 : master_send
// 信息帧格式（参考协议文档）：
//   +-------------+--------+--------+-------+--------+-------------+
//   | 前置        | 地址   | 功能码 | 数据  | CRC16  | 尾部        |
//   | 4字节延时   | 1字节  | 1字节  | N字节 | 2字节  | 4字节延时   |
//   +-------------+--------+--------+-------+--------+-------------+
// 有限状态机风格：三段式状态机，独热码状态编码
// 接收流程 :
//   1) 将每一个收到的字节存入帧缓存 frame_mem
//   2) 每个字节送入 crc16_modbus 模块（收到第一个字节时产生清零脉冲）
//   3) 4字节空闲间隔代表整帧接收完成
// 发送流程    :
//   ST_START : 保持静默4个字节时长（前置间隔）
//   字节发送 : 地址 + 功能码 + 数据（全部按原始接收顺序）
//   CRC      : CRC高字节在前，然后CRC低字节（示例 4A 然后 02）
//   ST_STOP  : 保持静默4个字节时长（尾部间隔）
// 示例: 接收 10 03 10 00 00 03 -> 发送 10 03 10 00 00 03 4A 02
// ==========================================================================
module master_send(
    input               sys_clk     ,
    input               sys_rst_n   ,
    //来自rs485_rx接收模块
    input       [7:0]   rx_data     ,   //接收到的字节数据
    input               start_en    ,   //字节有效脉冲（接收完成标志data_done）
    //输出到rs485_tx发送模块
    output  reg [7:0]   tx_data     ,   //待发送的字节数据
    output  reg         tx_en       ,   //单周期发送启动脉冲
    input               tx_busy     ,   //rs485_tx模块忙标志
    output  reg         busy            //整帧发送期间保持高电平
);
//参数定义
parameter   UART_BPS    = 115200;
parameter   CLK_FREQ    = 50000000;
localparam  BAUD_CNT_MAX = CLK_FREQ/UART_BPS;
localparam  UART_FRAME_BIT = 10;        //1起始位 + 8数据位 + 1停止位
localparam  SEND_BYTE_NUM  = 4;
//延时等于4个字节时长，用于前置间隔和尾部间隔
localparam  FOURBYTE_TIME  = BAUD_CNT_MAX*UART_FRAME_BIT*SEND_BYTE_NUM;
parameter   FRAME_MAX   = 16;           //单帧最大可缓存字节数
//独热码状态定义
localparam  [4:0] ST_RECV  = 5'b00001;  //接收状态 / 等待帧结束
localparam  [4:0] ST_START = 5'b00010;  //前置间隔：4字节静默延时
localparam  [4:0] ST_STRB  = 5'b00100;  //生成单个字节的tx_en发送脉冲
localparam  [4:0] ST_WAIT  = 5'b01000;  //等待单个字节发送完成
localparam  [4:0] ST_STOP  = 5'b10000;  //尾部间隔：4字节静默延时
reg     [4:0]           cur_st;
reg     [4:0]           next_st;
reg     [7:0]           frame_mem [0:FRAME_MAX-1]; //帧缓存数组
reg     [4:0]           wptr;           //接收字节写指针
reg     [4:0]           rptr;           //发送字节读指针
reg     [4:0]           frame_len;      //接收字节计数
reg     [19:0]          gap_cnt;        //RX线路空闲间隔计数器
reg     [19:0]          dly_cnt;        //4字节延时计数器
//状态跳转条件
wire                    frame_end = (wptr > 5'd0) && (gap_cnt >= FOURBYTE_TIME - 1);
wire                    start_dly_done = (dly_cnt >= FOURBYTE_TIME - 1);
//CRC相关连线
wire    [15:0]          crc_dout;
wire                    crc_dout_vld;
reg     [15:0]          crc_r;
//新帧第一个字节到来时，将CRC寄存器清零至0xffff
wire                    crc_sclr = (cur_st==ST_RECV) && start_en && (wptr==5'd0);
//tx_busy下降沿检测（单个字节发送完成）
reg                     tx_busy_d;
wire                    tx_done = tx_busy_d & ~tx_busy;
wire                    last_byte_sent = tx_done && (rptr == frame_len + 5'd1);
crc16_modbus u_crc16_modbus(
    .clk             (sys_clk                                  ),
    .rst_n           (sys_rst_n                                ),
    .sclr            (crc_sclr                                 ),
    .din             (rx_data                                  ),
    .din_vld         (start_en && (cur_st==ST_RECV)            ),
    .crc_dout        (crc_dout                                 ),
    .crc_dout_vld    (crc_dout_vld                             )
);
//锁存最新CRC计算结果（每收到1个字节更新一次CRC值）
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        crc_r <= 16'd0;
    else if(crc_dout_vld)
        crc_r <= crc_dout;
end
//tx_busy信号打一拍寄存
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        tx_busy_d <= 1'b0;
    else
        tx_busy_d <= tx_busy;
end
//==========================================================================
// 第一段：时序逻辑，状态寄存器
//==========================================================================
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        cur_st <= ST_RECV;
    else
        cur_st <= next_st;
end
//==========================================================================
// 第二段：组合逻辑，次态跳转逻辑
//==========================================================================
always@(*) begin
    next_st = cur_st;                    //默认：保持当前状态
    case(cur_st)
        ST_RECV : begin
            if(frame_end)
                next_st = ST_START;
            else
                next_st = ST_RECV;
        end
        ST_START: begin
            if(start_dly_done)
                next_st = ST_STRB;
            else
                next_st = ST_START;
        end
        ST_STRB : next_st = ST_WAIT;
        ST_WAIT : begin
            if(tx_done) begin
                if(rptr == frame_len + 5'd1)
                    next_st = ST_STOP;   //CRC低字节发送完毕
                else
                    next_st = ST_STRB;
            end
            else
                next_st = ST_WAIT;
        end
        ST_STOP : begin
            if(start_dly_done)
                next_st = ST_RECV;
            else
                next_st = ST_STOP;
        end
        default : next_st = ST_RECV;
    endcase
end
//==========================================================================
// 第三段：时序逻辑，输出与数据通路
//==========================================================================
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        wptr      <= 5'd0;
        rptr      <= 5'd0;
        frame_len <= 5'd0;
        gap_cnt   <= 20'd0;
        dly_cnt   <= 20'd0;
        tx_en     <= 1'b0;
        busy      <= 1'b0;
    end
    else begin
        tx_en <= 1'b0;                  //默认脉冲为低
        case(cur_st)
            //------------------------------------------------ 接收帧数据
            ST_RECV : begin
                busy <= 1'b0;
                if(start_en) begin                    //收到一个字节
                    if(wptr < FRAME_MAX)
                        frame_mem[wptr] <= rx_data;   //缓存字节
                    wptr    <= wptr + 1'b1;
                    gap_cnt <= 20'd0;                 //空闲计时器清零
                end
                else if(wptr > 5'd0) begin            //累计空闲间隔
                    if(frame_end) begin               //4字节空闲：判定帧结束
                        frame_len <= wptr;
                        rptr      <= 5'd0;
                        dly_cnt   <= 20'd0;
                        busy      <= 1'b1;
                    end
                    else begin
                        gap_cnt <= gap_cnt + 1'b1;
                    end
                end
            end
            //---------------- 前置间隔：保持4字节时长静默
            ST_START: begin
                if(start_dly_done)
                    dly_cnt <= 20'd0;
                else
                    dly_cnt <= dly_cnt + 1'b1;
            end
            //---------------- 启动单个UART字节发送
            ST_STRB : begin
                tx_en <= 1'b1;
            end
            //---------------- 等待单个字节发送完成
            ST_WAIT : begin
                if(tx_done) begin
                    if(last_byte_sent) begin
                        dly_cnt <= 20'd0;
                    end
                    else begin
                        rptr <= rptr + 1'b1;
                    end
                end
            end
            //---------------- 尾部间隔：保持4字节时长静默
            ST_STOP : begin
                if(start_dly_done) begin
                    busy    <= 1'b0;
                    wptr    <= 5'd0;
                    gap_cnt <= 20'd0;
                    dly_cnt <= 20'd0;
                end
                else begin
                    dly_cnt <= dly_cnt + 1'b1;
                end
            end
            default : ;
        endcase
    end
end
//组合逻辑数据输出：
//  rptr <  frame_len      -> 地址/功能码/数据（原始接收字节）
//  rptr == frame_len      -> CRC高字节（示例4A）
//  rptr == frame_len + 1  -> CRC低字节（示例02）
always@(*) begin
    if(rptr < frame_len)
        tx_data = frame_mem[rptr];
    else if(rptr == frame_len)
        tx_data = crc_r[15:8];
    else
        tx_data = crc_r[7:0];
end
endmodule

// ==========================================================================
// 模块名称 : slave
// 功能描述 : Modbus 0x03 读保持寄存器从机（响应帧不含CRC校验码）
//            rx_data 为外部接收链路送出的字节流（CRC已在外部处理/忽略）
// 响应格式 : 从机地址 | 03 | 字节数n(=寄存器个数*2) | 各寄存器数据(高字节在前)
// 请求示例 : 10 03 10 00 00 03（尾部 4a 02 为CRC，直接忽略）
// 响应示例 : 10 03 06 12 34 67 89 11 22
// 状态机   : 三段式、独热码，共9个状态
//            接收：IDLE -> ADDRESSCODE -> FUNCTIONCODE -> DATADISTRICT
//            判帧：DATADISTRICT -> FRAMECHECK -> BUILDFRAME -> SEND -> IDLE
//            异常：-> SILENT -> IDLE（CRC状态预留，本设计不使用）
// 写法说明 : 第三段 case(cur_st) 当前状态写法，两个配套要点：
//            1. 第二段跳转条件直接用实时信号(start_en/计数满/tx_done)，
//               不再使用寄存器脉冲 st_done，避免状态多停留一拍；
//            2. 各状态的初始化动作只在"进入该状态的第1拍"执行，利用独热码
//               特性生成进入脉冲 enter = cur_st & ~prev_st，避免驻留期间
//               每拍误清指针。
// ==========================================================================
module slave(
    input               sys_clk  ,
    input               sys_rst_n,
    input       [7:0]   rx_data  ,   //接收到的字节（CRC已在外部校验）
    input               start_en ,   //字节有效脉冲（每收到1字节拉高1拍）
    output  reg [7:0]   tx_data  ,   //返回给主机的响应字节（在SEND状态寄存输出）
    output  reg         tx_en    ,   //发送启动脉冲（接 rs485_tx 的 tx_en）
    input               tx_busy      //发送忙标志（接 rs485_tx 的 tx_busy）
);

//============================== 参数定义 ====================================
parameter   SLAVE_ADDR   = 8'h10     ;  //本机从机地址
parameter   FRAME_MAX    = 16        ;  //单帧最多缓存字节数
parameter   REG_DEPTH    = 16        ;  //保持寄存器数量
parameter   QTY_MAX      = 6         ;  //一次最多读取的寄存器个数
parameter   UART_BPS     = 115200    ;  //串口波特率
parameter   CLK_FREQ     = 50000000  ;  //系统时钟频率50MHz

localparam  BAUD_CNT_MAX  = CLK_FREQ/UART_BPS;           //1个bit的时钟数
localparam  UART_FRAME_BIT = 10;                        //1起始位+8数据位+1停止位
localparam  SEND_BYTE_NUM  = 4;                         //帧间隔按4个字节时间判断
//总线上连续4个字节时间无新数据，认为一帧接收结束
localparam  FOURBYTE_TIME  = BAUD_CNT_MAX*UART_FRAME_BIT*SEND_BYTE_NUM;

//============================== 状态定义（独热码）==========================
localparam  IDLE         = 9'b000000001;  //空闲：等待并接收地址码
localparam  ADDRESSCODE  = 9'b000000010;  //地址码校验
localparam  FUNCTIONCODE = 9'b000000100;  //功能码校验
localparam  DATADISTRICT = 9'b000001000;  //数据区接收（起始地址+个数+CRC）
localparam  CRC          = 9'b000010000;  //预留状态，本设计不使用CRC
localparam  FRAMECHECK   = 9'b000100000;  //帧结束后的合法性判断
localparam  BUILDFRAME   = 9'b001000000;  //组装响应帧（无CRC）
localparam  SEND         = 9'b010000000;  //逐字节发送响应帧
localparam  SILENT       = 9'b100000000;  //异常静默：等总线空闲后返回

//功能码定义
localparam  READREGVALUE  = 8'h03;        //读保持寄存器
localparam  WRITEREGVALUE = 8'h10;        //写多个寄存器（预留给后续扩展）

//============================== 寄存器定义 ==================================
reg  [7:0]  rx_data_buffer [0:FRAME_MAX-1]; //接收帧缓存
reg  [7:0]  tx_buffer      [0:FRAME_MAX-1]; //响应帧缓存（不含CRC）
reg  [15:0] reg_mem        [0:REG_DEPTH-1]; //保持寄存器（被读取的数据源）
reg  [4:0]  wptr;                          //接收写指针
reg  [5:0]  rptr;                          //发送读指针
reg  [5:0]  tx_len;                        //响应帧总字节数
reg  [8:0]  cur_st;                        //当前状态
reg  [8:0]  next_st;                       //下一状态
reg  [8:0]  prev_st;                       //当前状态打1拍（用于产生进入脉冲）
reg  [19:0] cnt;                           //总线空闲间隔计数器
reg         send_phase;                    //SEND子节拍：0=启动发送，1=等待发完
reg         tx_busy_d;                     //tx_busy打1拍
integer     k;                             //组帧for循环变量

//保持寄存器演示数据（实际工程把这里换成你的真实数据输入）
//1000H -> reg_mem[0]，1001H -> reg_mem[1]，1002H -> reg_mem[2]
initial begin
    reg_mem[0]  = 16'h1234;
    reg_mem[1]  = 16'h6789;
    reg_mem[2]  = 16'h1122;
    reg_mem[3]  = 16'h0000;
    reg_mem[4]  = 16'h0000;
    reg_mem[5]  = 16'h0000;
    reg_mem[6]  = 16'h0000;
    reg_mem[7]  = 16'h0000;
    reg_mem[8]  = 16'h0000;
    reg_mem[9]  = 16'h0000;
    reg_mem[10] = 16'h0000;
    reg_mem[11] = 16'h0000;
    reg_mem[12] = 16'h0000;
    reg_mem[13] = 16'h0000;
    reg_mem[14] = 16'h0000;
    reg_mem[15] = 16'h0000;
end

//============================== 请求字段解析 ===============================
wire [15:0] d_start = {rx_data_buffer[2], rx_data_buffer[3]}; //起始地址
wire [15:0] d_qty   = {rx_data_buffer[4], rx_data_buffer[5]}; //读取寄存器个数
wire [3:0]  d_idx   = d_start[3:0];                           //起始寄存器索引
wire [4:0]  idx_sum = d_idx + d_qty[3:0];                     //末地址索引(越界判断)

wire        addr_ok = (rx_data_buffer[0] == SLAVE_ADDR);      //地址为本机
wire        func_ok = (rx_data_buffer[1] == READREGVALUE);    //功能码为03
wire        len_ok  = (wptr >= 6);                            //至少收完6字节协议头
wire        qty_ok  = (d_qty >= 1) && (d_qty <= QTY_MAX)      //个数合法且不越界
                   && (idx_sum <= REG_DEPTH);
wire        frame_valid = addr_ok && func_ok && len_ok && qty_ok;

//总线4字节空闲计满（DATADISTRICT/SILENT共用的实时跳转条件）
wire        gap_full = (cnt >= FOURBYTE_TIME - 1);

//tx_busy 下降沿脉冲：表示1个字节发送完成
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        tx_busy_d <= 1'b0;
    else
        tx_busy_d <= tx_busy;
end
wire tx_done = tx_busy_d & ~tx_busy;

//最后一个字节发完的实时条件（供第二段跳转，不打拍）
wire send_all_done = (send_phase == 1'b1) && tx_done
                  && (rptr == tx_len - 1'b1);

//============================== 进入脉冲生成 ===============================
//独热码专用技巧：某位置1表示"刚进入该状态"，仅持续1拍
//（IDLE位为例：刚回到IDLE的那一拍 enter[0]=1，用来做一次性初始化）
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        prev_st <= IDLE;
    else
        prev_st <= cur_st;
end
wire [8:0] enter = cur_st & ~prev_st;

//==========================================================================
// 第一段：状态寄存器
//==========================================================================
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        cur_st <= IDLE;
    else
        cur_st <= next_st;
end

//==========================================================================
// 第二段：组合逻辑，只负责状态跳转（全部使用实时条件，不使用脉冲寄存器）
//==========================================================================
always@(*) begin
    next_st = IDLE;
    case(cur_st)
        //-------------------- 接收流程 --------------------
        IDLE:                                      //收到第0字节（地址码）
            next_st = start_en ? ADDRESSCODE : IDLE;

        ADDRESSCODE: begin                         //第1字节到来时判地址
            if(start_en)
                next_st = addr_ok ? FUNCTIONCODE : SILENT;
            else
                next_st = ADDRESSCODE;
        end

        FUNCTIONCODE: begin                        //第2字节到来时判功能码
            if(start_en)
                next_st = func_ok ? DATADISTRICT : SILENT;
            else
                next_st = FUNCTIONCODE;
        end

        DATADISTRICT:                              //4字节无新数据，帧结束
            next_st = ((!start_en) && gap_full) ? FRAMECHECK : DATADISTRICT;

        CRC:                                       //预留状态
            next_st = IDLE;

        //-------------------- 判帧与响应流程 --------------------
        FRAMECHECK:                                //合法去组帧，非法静默丢弃
            next_st = frame_valid ? BUILDFRAME : IDLE;

        BUILDFRAME:                                //组帧固定1拍，无条件进发送
            next_st = SEND;

        SEND:                                      //最后一字节发完回空闲
            next_st = send_all_done ? IDLE : SEND;

        //-------------------- 异常静默 --------------------
        SILENT:                                    //等总线上这帧发完
            next_st = ((!start_en) && gap_full) ? IDLE : SILENT;

        default:
            next_st = IDLE;
    endcase
end

//==========================================================================
// 第三段：时序逻辑，数据通路与输出（case(cur_st) 当前状态写法）
// 初始化只在 enter 脉冲（进入状态第1拍）执行，其余为状态驻留期间动作
//==========================================================================
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        wptr       <= 5'd0;
        rptr       <= 6'd0;
        tx_len     <= 6'd0;
        cnt        <= 20'd0;
        send_phase <= 1'b0;
        tx_en      <= 1'b0;
        tx_data    <= 8'd0;
    end
    else begin
        tx_en <= 1'b0;                 //发送脉冲默认清零，仅SEND启动拍拉高
        //tx_data 不设默认值：启动发送时赋一次并保持，供 rs485_tx 锁存

        case(cur_st)
            //------------------------------------------------ 空闲/收地址码
            IDLE: begin
                if(enter[0]) begin
                    //仅进入IDLE的第1拍初始化一次，驻留期间不再反复清零
                    wptr       <= 5'd0;
                    rptr       <= 6'd0;
                    cnt        <= 20'd0;
                    send_phase <= 1'b0;
                end
                else if(start_en) begin
                    rx_data_buffer[0] <= rx_data;   //第0字节：从机地址
                    wptr <= 5'd1;
                end
            end

            //------------------------------------------------ 地址码校验
            ADDRESSCODE: begin
                if(start_en) begin                  //此时收到第1字节：功能码
                    if(addr_ok) begin
                        rx_data_buffer[wptr] <= rx_data;
                        wptr <= wptr + 1'b1;
                    end
                    else
                        cnt <= 20'd0;               //不符则清间隔计数，去静默
                end
            end

            //------------------------------------------------ 功能码校验
            FUNCTIONCODE: begin
                if(start_en) begin                  //此时收到第2字节：起始地址高
                    if(func_ok) begin
                        rx_data_buffer[wptr] <= rx_data;
                        wptr <= wptr + 1'b1;
                    end
                    else
                        cnt <= 20'd0;               //不符则清间隔计数，去静默
                end
            end

            //------------------------------------------------ 数据区接收
            DATADISTRICT: begin
                if(start_en) begin                  //起始地址/个数/CRC等剩余字节
                    if(wptr < FRAME_MAX)
                        rx_data_buffer[wptr] <= rx_data;
                    wptr <= wptr + 1'b1;
                    cnt  <= 20'd0;                  //来新字节，间隔计数清零
                end
                else if(!gap_full) begin
                    cnt <= cnt + 1'b1;              //无新字节，累加空闲间隔
                end
            end

            //------------------------------------------------ CRC预留，空操作
            CRC: ;

            //------------------------------------------------ 帧合法性判断
            //纯判断状态，合法性由第二段组合判断，本段无动作
            FRAMECHECK: ;

            //------------------------------------------------ 组装响应帧
            BUILDFRAME: begin
                tx_buffer[0] <= SLAVE_ADDR;             //从机地址
                tx_buffer[1] <= READREGVALUE;           //功能码03
                tx_buffer[2] <= {d_qty[6:0], 1'b0};     //数据字节数 n=2*个数
                for(k=0;k<QTY_MAX;k=k+1) begin
                    if(k < d_qty) begin
                        tx_buffer[3+2*k] <= reg_mem[d_idx+k][15:8]; //寄存器高字节
                        tx_buffer[4+2*k] <= reg_mem[d_idx+k][7:0];  //寄存器低字节
                    end
                end
                tx_len     <= 6'd3 + {1'b0, d_qty[4:0], 1'b0}; //3字节头+2*个数
                rptr       <= 6'd0;                  //发送指针从头开始
                send_phase <= 1'b0;
            end

            //------------------------------------------------ 逐字节发送
            SEND: begin
                if(send_phase == 1'b0) begin
                    //节拍0：输出本字节并给1拍tx_en，rs485_tx下一沿锁存
                    tx_data    <= tx_buffer[rptr];
                    tx_en      <= 1'b1;
                    send_phase <= 1'b1;
                end
                else if(tx_done) begin
                    //节拍1：本字节发完，指针后移（是否回IDLE由第二段判断）
                    if(rptr != tx_len - 1'b1)
                        rptr <= rptr + 1'b1;
                    send_phase <= 1'b0;
                end
            end

            //------------------------------------------------ 异常静默
            SILENT: begin
                if(start_en)
                    cnt <= 20'd0;                     //总线上还有字节，继续等
                else if(!gap_full)
                    cnt <= cnt + 1'b1;                //累加空闲间隔
            end

            default: ;
        endcase
    end
end

endmodule

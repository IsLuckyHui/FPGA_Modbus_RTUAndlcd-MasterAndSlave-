// ==========================================================================
// 模块名称 : slave
// 功能描述 : Modbus 0x03（读保持寄存器）从机应答模块，响应帧不含 CRC 校验。
//            输入 rx_data 为串口收到的字节（请求帧里的 CRC 字节也一并接收，但本模块不做校验）
//            只响应对应本机地址、功能码为 03 的请求；地址/功能码不对则整帧静默丢弃
// 帧格式   : 从机地址 | 功能码 | 字节数n(=寄存器个数*2) | 寄存器数据(每寄存器2字节，高字节在前)
// 请求示例 : 10 03 10 00 00 03        （末尾 4A 02 为 CRC，本模块不校验）
// 应答示例 : 10 03 06 12 34 67 89 11 22   （本设计响应帧不含 CRC）
// 状态机   : 共 9 个状态，独热码编码
//            正常流程 IDLE -> ADDRESS_CODE -> FUNCTION_CODE -> DATA_DISTRICT
//                     DATA_DISTRICT -> FRAME_CHECK -> BUILD_FRAME -> SEND -> IDLE
//            异常流程 IDLE/ADDRESS_CODE/FUNCTION_CODE -> SILENT -> IDLE
//                     （SILENT 等满 4 字节帧间隔后重新开始监听）
// 设计要点 : 三段式状态机，所有动作集中写在 case(cur_st) 的第三个时序块里
//            1. 每个状态只在“刚进入”的那一拍做一次初始化动作
//               （start_en / 读写指针 / 帧间隔计数 / tx_done 都在状态内部各自处理）
//            2. 用“当前状态 & ~上一拍状态”得到“刚进入该状态 1 拍”的使能信号：
//               enter = cur_st & ~prev_st，省掉额外的状态计数器，也不会漏拍
// ==========================================================================
module slave(
    input               sys_clk  ,
    input               sys_rst_n,
    input       [7:0]   rx_data  ,   //接收到的字节数据（含 CRC 字节，本模块不校验）
    input               start_en ,   //接收字节有效脉冲（高电平持续 1 个时钟）
    input               tx_busy  ,    //发送忙标志，来自 rs485_tx 的 tx_busy
    
    output  reg [7:0]   tx_data  ,   //待发送的响应字节，进入 SEND 时装载
    output  reg         tx_en       //发送启动脉冲（送给 rs485_tx 的 tx_en，高电平 1 拍）
);

//============================== 参数定义 ====================================
parameter   SLAVE_ADDR   = 8'h10     ;  //本机从机地址
parameter   FRAME_MAX    = 16        ;  //单帧最大缓存字节数
parameter   REG_DEPTH    = 16        ;  //保持寄存器个数
parameter   QTY_MAX      = 6         ;  //单次最多读取的寄存器个数
parameter   UART_BPS     = 115200    ;  //串口波特率
parameter   CLK_FREQ     = 50000000  ;  //系统时钟 50MHz

localparam  BAUD_CNT_MAX  = CLK_FREQ/UART_BPS;           //1 个 bit 占用的时钟周期数
localparam  UART_FRAME_BIT = 10;                        //1 起始位 + 8 数据位 + 1 停止位
localparam  SEND_BYTE_NUM  = 4;                         //帧间隔按 4 个字节时长判定
//4 个字节时长对应的时钟数，用于判定一帧是否结束（相当于 RTU 的 t3.5 静默）
localparam  FOUR_BYTE_TIME  = BAUD_CNT_MAX*UART_FRAME_BIT*SEND_BYTE_NUM;

//============================== 状态机编码（独热码）==========================
localparam  IDLE          = 9'b000000001;  //空闲，等待首字节
localparam  ADDRESS_CODE  = 9'b000000010;  //地址码校验
localparam  FUNCTION_CODE = 9'b000000100;  //功能码校验
localparam  DATA_DISTRICT = 9'b000001000;  //数据区：起始地址+数量+CRC 字节
localparam  CRC           = 9'b000010000;  //CRC 校验（预留，当前未实现）
localparam  FRAME_CHECK   = 9'b000100000;  //整帧合法性判断
localparam  BUILD_FRAME   = 9'b001000000;  //组响应帧（不含 CRC）
localparam  SEND          = 9'b010000000;  //逐字节发送响应帧
localparam  SILENT        = 9'b100000000;  //静默丢弃非法帧，等帧间隔后回 IDLE

//功能码定义
localparam  READ_REG_VALUE  = 8'h03;        //读保持寄存器
localparam  WRITE_REG_VALUE = 8'h10;        //写多个寄存器（仅定义，未实现）

//============================== 内部信号 ==================================
reg  [7:0]  rx_data_buffer [0:FRAME_MAX-1]; //接收帧缓存
reg  [7:0]  tx_buffer      [0:FRAME_MAX-1]; //发送帧缓存（不含 CRC）
reg  [15:0] reg_mem        [0:REG_DEPTH-1]; //保持寄存器堆
reg  [4:0]  wptr;                          //接收写指针
reg  [5:0]  rptr;                          //发送读指针
reg  [5:0]  tx_len;                        //本帧响应总字节数
reg  [8:0]  cur_st;                        //当前状态
reg  [8:0]  next_st;                       //次态
reg  [8:0]  prev_st;                       //上一拍状态，用于产生“刚进入状态”标志
reg  [19:0] cnt;                           //帧间隔计数器
reg         send_phase;                    //SEND 内部节拍：0=装载并启动发送，1=等待本字节发完
reg         tx_busy_d;                     //tx_busy 打一拍，用于取下降沿
integer     k;                             //for 循环变量

//保持寄存器初始值（综合时按初值实现）
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

//============================== 关键判据译码 ===============================
wire [15:0] d_start = {rx_data_buffer[2], rx_data_buffer[3]}; //起始地址
wire [15:0] d_qty   = {rx_data_buffer[4], rx_data_buffer[5]}; //读取寄存器数量
wire [3:0]  d_idx   = d_start[3:0];                           //起始地址低 4 位，1000H 对应索引 0
wire [4:0]  idx_sum = d_idx + d_qty[3:0];                     //末寄存器索引：如数量 00 03，则 0+3 = 3（实际访问 0~2）
                                                            
wire        addr_ok = (rx_data_buffer[0] == SLAVE_ADDR);      //从机地址匹配
wire        func_ok = (rx_data_buffer[1] == READ_REG_VALUE);  //功能码为 03
wire        len_ok  = (wptr >= 6);                            //至少 6 字节：1地址+1功能码+2起始地址+2数量
wire        qty_ok  = (d_qty >= 1) && (d_qty <= QTY_MAX)      //数量 1~QTY_MAX，且不越界
                   && (idx_sum <= REG_DEPTH);
wire        frame_valid = addr_ok && func_ok && len_ok && qty_ok;

//帧间隔计满标志：DATA_DISTRICT / SILENT 用它判断一帧结束
wire        gap_full = (cnt >= FOUR_BYTE_TIME - 1);

//tx_busy 打一拍，用于取下降沿（即“一个字节发送完成”）
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        tx_busy_d <= 1'b0;
    else
        tx_busy_d <= tx_busy;
end
wire tx_done = tx_busy_d & ~tx_busy;

//整帧是否发送完毕：处于发送第二阶段且最后一个字节已发完
wire send_all_done = (send_phase == 1'b1) && tx_done
                  && (rptr == tx_len - 1'b1);

//============================== 刚进入状态标志 ===============================
//用“当前状态与上一拍状态相与”得到“刚进入该状态”的 1 拍标志
//复位时 prev_st = IDLE，故复位释放后 enter[0]=1，用于清空收发指针与计数
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        prev_st <= IDLE;
    else
        prev_st <= cur_st;
end
wire [8:0] enter = cur_st & ~prev_st;

//==========================================================================
// 第一段：时序逻辑，状态寄存器
//==========================================================================
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)
        cur_st <= IDLE;
    else
        cur_st <= next_st;
end

//==========================================================================
// 第二段：组合逻辑，次态跳转
//==========================================================================
always@(*) begin
    next_st = IDLE;
    case(cur_st)
        //-------------------- 正常接收流程 --------------------
        IDLE:                                      //空闲：收到首字节开始接收
            next_st = start_en ? ADDRESS_CODE : IDLE;

        ADDRESS_CODE: begin                         //地址校验：比较 buffer[0]
            if(start_en)
                next_st = addr_ok ? FUNCTION_CODE : SILENT;
            else
                next_st = ADDRESS_CODE;
        end

        FUNCTION_CODE: begin                        //功能码校验：只支持 03
            if(start_en)
                next_st = func_ok ? DATA_DISTRICT : SILENT;
            else
                next_st = FUNCTION_CODE;
        end

        DATA_DISTRICT:                              //数据区：收满后等 4 字节帧间隔
            next_st = ((!start_en) && gap_full) ? FRAME_CHECK : DATA_DISTRICT;

        CRC:                                       //CRC 校验：预留，当前直接跳过
            next_st = IDLE;

        //-------------------- 组帧与发送流程 --------------------
        FRAME_CHECK:                                //整帧合法则组响应，否则丢弃
            next_st = frame_valid ? BUILD_FRAME : IDLE;

        BUILD_FRAME:                                //组帧占用 1 拍
            next_st = SEND;

        SEND:                                      //全部字节发完回空闲
            next_st = send_all_done ? IDLE : SEND;

        //-------------------- 非法帧静默 --------------------
        SILENT:                                    //等帧间隔结束再重新监听
            next_st = ((!start_en) && gap_full) ? IDLE : SILENT;

        default:
            next_st = IDLE;
    endcase
end

//==========================================================================
// 第三段：时序逻辑，数据通路（所有动作都在 case(cur_st) 内按状态书写）
// 每个状态用 enter 标志做“进入时的一次性初始化”，避免额外的计数判断
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
        tx_en <= 1'b0;                 //默认拉低，只在 SEND 第 0 拍产生 1 拍脉冲
        //tx_data 在进入 SEND 时装载并保持，供 rs485_tx 采样

        case(cur_st)
            //------------------------------------------------ 空闲/收首字节
            IDLE: begin
                if(enter[0]) begin
                    //刚回到 IDLE：清空收发指针、帧间隔计数与发送节拍
                    wptr       <= 5'd0;
                    rptr       <= 6'd0;
                    cnt        <= 20'd0;
                    send_phase <= 1'b0;
                end
                else if(start_en) begin
                    rx_data_buffer[0] <= rx_data;   //第 0 字节：从机地址
                    wptr <= 5'd1;
                end
            end

            //------------------------------------------------ 地址校验状态
            ADDRESS_CODE: begin
                if(start_en) begin                  //等到下一字节（功能码）
                    if(addr_ok) begin
                        rx_data_buffer[wptr] <= rx_data;   //地址匹配：存入功能码
                        wptr <= wptr + 1'b1;
                    end
                    else
                        cnt <= 20'd0;               //地址不匹配：清帧间隔，准备转入 SILENT
                end
            end

            //------------------------------------------------ 功能码校验状态
            FUNCTION_CODE: begin
                if(start_en) begin                  //等到下一字节（起始地址高字节）
                    if(func_ok) begin
                        rx_data_buffer[wptr] <= rx_data;
                        wptr <= wptr + 1'b1;
                    end
                    else
                        cnt <= 20'd0;               //功能码不支持：清帧间隔，准备转入 SILENT
                end
            end

            //------------------------------------------------ 数据区
            DATA_DISTRICT: begin
                if(start_en) begin                  //起始地址/数量/CRC 字节依次入缓存
                    if(wptr < FRAME_MAX)
                        rx_data_buffer[wptr] <= rx_data;
                    wptr <= wptr + 1'b1;
                    cnt  <= 20'd0;                  //每收到一个字节都重新计时
                end
                else if(!gap_full) begin
                    cnt <= cnt + 1'b1;              //无新字节时累计空闲时间
                end
            end

            //------------------------------------------------ CRC 校验（预留）
            CRC: ;

            //------------------------------------------------ 整帧合法性判断
            //判据已在组合逻辑 frame_valid 中给出，本状态无需时序动作
            FRAME_CHECK: ;

            //------------------------------------------------ 组响应帧
            BUILD_FRAME: begin
                tx_buffer[0] <= SLAVE_ADDR;             //从机地址
                tx_buffer[1] <= READ_REG_VALUE;           //功能码 03
                tx_buffer[2] <= {d_qty[6:0], 1'b0};     //字节数 n = 2 * 寄存器个数
                //寄存器个数受 QTY_MAX（6）限制，if 保证不写越界
                //d_qty 由上位机给定，取低位参与运算，超过 16 的情况已被 qty_ok 过滤
                for(k=0;k<QTY_MAX;k=k+1) begin
                    if(k < d_qty) begin
                        tx_buffer[3+2*k] <= reg_mem[d_idx+k][15:8]; //寄存器值高字节
                        tx_buffer[4+2*k] <= reg_mem[d_idx+k][7:0];  //寄存器值低字节
                    end
                end
                tx_len     <= 6'd3 + {1'b0, d_qty[4:0], 1'b0}; //3 字节帧头 + 2*寄存器个数
                rptr       <= 6'd0;                  //发送读指针归零
                send_phase <= 1'b0;
            end

            //------------------------------------------------ 逐字节发送
            SEND: begin
                if(send_phase == 1'b0) begin
                    //第 0 拍：装载当前字节并给出 1 拍 tx_en，启动 rs485_tx
                    tx_data    <= tx_buffer[rptr];
                    tx_en      <= 1'b1;
                    send_phase <= 1'b1;
                end
                else if(tx_done) begin
                    //第 1 拍：本字节发完，未到最后一字节则指针+1 继续，否则等回 IDLE
                    if(rptr != tx_len - 1'b1)
                        rptr <= rptr + 1'b1;
                    send_phase <= 1'b0;
                end
            end

            //------------------------------------------------ 非法帧静默
            SILENT: begin
                if(start_en)
                    cnt <= 20'd0;                     //仍有字节进来：重新计时
                else if(!gap_full)
                    cnt <= cnt + 1'b1;                //累计空闲，满 4 字节时间后回 IDLE
            end

            default: ;
        endcase
    end
end

endmodule

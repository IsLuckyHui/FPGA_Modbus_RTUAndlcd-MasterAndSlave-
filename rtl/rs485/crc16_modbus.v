// ==========================================================================
// 独立模块: CRC16-Modbus
// 输入: din[7:0] + din_vld(单字节计算脉冲), sclr同步清零(回到0xFFFF)
// 输出: crc_dout (已交换高低字节: [7:0]=CRC低字节 [15:8]=CRC高字节)
//       crc_dout_vld(每喂完1字节脉冲1次)
// 验证向量: 10 03 10 00 00 03 -> crc_dout = 16'h4A02 (线上顺序 4A 02)
// ==========================================================================
module crc16_modbus(
    input                clk         ,
    input                rst_n       ,
    input                sclr        , //如果拉高代表有效 会将值回到16'hffff
    input      [7:0]     din         ,
    input                din_vld     , //输入有效状态 
    output reg [15:0]    crc_dout    ,
    output reg           crc_dout_vld
);

reg [3:0]   bit_cnt;
wire        add_bit_cnt;
wire        end_bit_cnt;
reg [7:0]   crc_temp;
reg [15:0]  crc_reg;
//reg [15:0]  crc_dout;
//reg         crc_dout_vld;

//wire [7:0]  data;
//wire [15:0] crcdata;

reg [2:0]   state_c;
reg [2:0]   state_n;

wire        idle2crc_xor;
wire        crc_xor2crc_circle;
wire        crc_circle2crg_0reg;
wire        crc_circle2crg_1reg;
wire        crg_0reg2crc_circle;
wire        crg_0reg2crg_done;
wire        crg_1reg2crc_circle;
wire        crg_1reg2crg_done;
wire        crc_done2idle;

//状态机定义
parameter IDLE        = 0;
parameter CRC_XOR     = 1;
parameter CRC_CIRCLE  = 2;
parameter CRG_0REG    = 3;
parameter CRG_1REG    = 4;
parameter CRC_DONE    = 5;

//bit_cnt 位计数器
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        bit_cnt <= 0;
    end
    else if(add_bit_cnt) begin
        if(end_bit_cnt)
            bit_cnt <= 0;
        else
            bit_cnt <= bit_cnt + 1;
    end
end
assign add_bit_cnt  = (state_c==CRG_0REG || state_c==CRG_1REG);
assign end_bit_cnt  = add_bit_cnt && bit_cnt== 8-1;

//crc_temp寄存器
always @(posedge clk or negedge rst_n) begin
    if(rst_n==1'b0) begin
        crc_temp <= 8'h0;
    end
    else if(idle2crc_xor) begin
        crc_temp <= din;
    end
end

//crc_reg CRC计算寄存器
always @(posedge clk or negedge rst_n) begin
    if(rst_n==1'b0) begin
        crc_reg <= 16'hffff;
    end
    else if(sclr) begin
        crc_reg <= 16'hffff;
    end
    else if(state_c==CRC_XOR) begin
        crc_reg[7:0] <= crc_temp ^ crc_reg[7:0];
    end
    else if(state_c==CRC_CIRCLE) begin
        crc_reg <= crc_reg >> 1;
    end
    else if(state_c==CRG_1REG) begin
        crc_reg <= crc_reg ^ 16'hA001;
    end
end

//crc_dout_vld 输出有效标志
always @(posedge clk or negedge rst_n) begin
    if(rst_n==1'b0) begin
        crc_dout_vld <= 1'b0;
    end
    else if(crc_done2idle) begin
        crc_dout_vld <= 1'b1;
    end
    else begin
        crc_dout_vld <= 1'b0;
    end
end

//crc_dout 输出结果，交换高低字节，适配modbus小端输出
always @(posedge clk or negedge rst_n) begin
    if(rst_n==1'b0) begin
        crc_dout <= 16'h0;
    end
    else if(crc_done2idle) begin
        crc_dout <= { crc_reg[15:8],crc_reg[7:0]};
    end
end

//状态机时序逻辑
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state_c <= IDLE;
    end
    else begin
        state_c <= state_n;
    end
end

//状态机组合逻辑
always@(*) begin
    case(state_c)
        IDLE:begin
            if(idle2crc_xor) begin
                state_n = CRC_XOR;
            end
            else begin
                state_n = state_c;
            end
        end
        CRC_XOR:begin
            if(crc_xor2crc_circle) begin
                state_n = CRC_CIRCLE;
            end
            else begin
                state_n = state_c;
            end
        end
        CRC_CIRCLE:begin
            if(crc_circle2crg_0reg) begin
                state_n = CRG_0REG;
            end
            else if(crc_circle2crg_1reg) begin
                state_n = CRG_1REG;
            end
            else begin
                state_n = state_c;
            end
        end
        CRG_0REG:begin
            if(crg_0reg2crc_circle) begin
                state_n = CRC_CIRCLE;
            end
            else if(crg_0reg2crg_done) begin
                state_n = CRC_DONE;
            end
            else begin
                state_n = state_c;
            end
        end
        CRG_1REG:begin
            if(crg_1reg2crc_circle) begin
                state_n = CRC_CIRCLE;
            end
            else if(crg_1reg2crg_done) begin
                state_n = CRC_DONE;
            end
            else begin
                state_n = state_c;
            end
        end
        CRC_DONE:begin
            if(crc_done2idle) begin
                state_n = IDLE;
            end
            else begin
                state_n = state_c;
            end
        end
        default:begin
            state_n = IDLE;
        end
    endcase
end

//状态跳转条件
assign idle2crc_xor        = state_c == IDLE && din_vld;
assign crc_xor2crc_circle  = state_c == CRC_XOR;
assign crc_circle2crg_0reg = state_c == CRC_CIRCLE && crc_reg[0]==1'b0;
assign crc_circle2crg_1reg = state_c == CRC_CIRCLE && crc_reg[0]==1'b1;
assign crg_0reg2crc_circle = state_c == CRG_0REG && !end_bit_cnt;
assign crg_0reg2crg_done   = state_c == CRG_0REG && end_bit_cnt;
assign crg_1reg2crc_circle = state_c == CRG_1REG && !end_bit_cnt;
assign crg_1reg2crg_done   = state_c == CRG_1REG && end_bit_cnt;
assign crc_done2idle       = state_c == CRC_DONE;

endmodule

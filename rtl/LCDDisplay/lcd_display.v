module lcd_display(
    input       lcd_pclk,
    input       rst_n,
    
    input      [10:0]     pixel_xpos,
    input      [10:0]     pixel_ypos,
    // 从机返回的寄存器值（高字节在前，由 rs485_top 从响应帧中提取）
    input      [15:0]     reg_val0,   // 第1个寄存器 -> 第1行标签后
    input      [15:0]     reg_val1,   // 第2个寄存器 -> 第2行标签后
    input      [15:0]     reg_val2,   // 第3个寄存器 -> 第3行标签后
    input      [15:0]     reg_val3,   // 第4个寄存器 -> 第4行标签后
    input      [15:0]     reg_val4,   // 第5个寄存器 -> 第5行标签后
    input      [15:0]     reg_val5,   // 第6个寄存器 -> 第6行标签后
    output reg [23:0]     pixel_data
);
//parameter define
parameter  FRAME_MAX   = 16        ;  //单帧最多缓存字节数
localparam PIC_X_START = 11'd10; //图片起始横坐标
localparam PIC_Y_START = 11'd10; //图片起始纵坐标
localparam PIC_WIDTH = 11'd167; //图片宽度
localparam PIC_HEIGHT = 11'd134; //图片高度

// 数值显示参数（数值显示在 COE 图片右侧/标签后面）
localparam VALUE_X_START = 11'd200;  // 数值起始横坐标
localparam VALUE_Y_START = 11'd15;   // 第1行数值起始纵坐标
localparam VALUE_Y_STEP  = 11'd19;   // 每行数值的行间距
localparam DIGIT_W       = 11'd8;    // 单个数字宽度(像素)
localparam DIGIT_H       = 11'd16;   // 单个数字高度(像素)
localparam NUM_DIGITS    = 4;        // 每个数值显示4位十六进制
localparam NUM_VALUES    = 6;        // 最多显示6个数值

localparam BACK_COLOR = 24'hffffff; //背景色，白色
localparam CHAR_COLOR = 24'h000000; //字符颜色，黑色

//reg define
reg  [127:0] char [9:0] ; //字符数组，存放数字 0~9 的字模
wire         rom_rd_en  ;
reg  [14:0]  rom_addr;  //ROM地址
wire [23:0]  rom_rd_data ;//ROM 数据

assign rom_rd_en = 1'b1; //读使能拉高，一直从 ROM 读数据

//字符数组初始值,用于存储数字模板(字模数据,每个数字大小:16*8)
always @(posedge lcd_pclk ) begin
    char[0] <= 128'h00000018244242424242424224180000 ; // "0"
    char[1] <= 128'h000000107010101010101010107C0000 ; // "1"
    char[2] <= 128'h0000003C4242420404081020427E0000 ; // "2"
    char[3] <= 128'h0000003C424204180402024244380000 ; // "3"
    char[4] <= 128'h000000040C14242444447E04041E0000 ; // "4"
    char[5] <= 128'h0000007E404040586402024244380000 ; // "5"
    char[6] <= 128'h0000001C244040586442424224180000 ; // "6"
    char[7] <= 128'h0000007E444408081010101010100000 ; // "7"
    char[8] <= 128'h0000003C4242422418244242423C0000 ; // "8"
    char[9] <= 128'h0000001824424242261A020224380000 ; // "9"
end

//==========================================================================
// 当前像素所在的数值行索引 (0 ~ NUM_VALUES-1)
//==========================================================================
reg [2:0] line_idx;
always @(*) begin
    line_idx = 3'd0;
    if(pixel_ypos >= VALUE_Y_START + 5*VALUE_Y_STEP)
        line_idx = 3'd5;
    else if(pixel_ypos >= VALUE_Y_START + 4*VALUE_Y_STEP)
        line_idx = 3'd4;
    else if(pixel_ypos >= VALUE_Y_START + 3*VALUE_Y_STEP)
        line_idx = 3'd3;
    else if(pixel_ypos >= VALUE_Y_START + 2*VALUE_Y_STEP)
        line_idx = 3'd2;
    else if(pixel_ypos >= VALUE_Y_START + 1*VALUE_Y_STEP)
        line_idx = 3'd1;
    else
        line_idx = 3'd0;
end

// 当前像素在数值显示区内的坐标
wire [10:0] line_y_start = VALUE_Y_START + line_idx * VALUE_Y_STEP;
wire [10:0] y_offset     = pixel_ypos - line_y_start;   // 行内 Y 偏移 0~18
wire        y_in_digit   = (y_offset < DIGIT_H);         // 是否在数字高度内 0~15

wire [10:0] x_offset     = pixel_xpos - VALUE_X_START;   // X 偏移 0~31
wire        x_in_val     = (pixel_xpos >= VALUE_X_START)
                        && (pixel_xpos < VALUE_X_START + NUM_DIGITS * DIGIT_W);
wire [2:0]  digit_idx    = x_offset[5:3];   // 第几个数字 0~3 (左->右)
wire [2:0]  x_in_digit   = x_offset[2:0];   // 数字内 X 偏移 0~7

wire        in_val_region = x_in_val && y_in_digit;

//==========================================================================
// 取出当前像素对应的数字值 (0~9)
// 直接取寄存器值的十六进制 nibble：
//   digit_idx=0 -> 最高 nibble [15:12]
//   digit_idx=1 -> [11:8]
//   digit_idx=2 -> [7:4]
//   digit_idx=3 -> 最低 nibble [3:0]
// 例如 reg_val0=16'h1234 -> 显示 "1234"
//==========================================================================
reg [3:0] cur_digit;
always @(*) begin
    case(line_idx)
        3'd0: cur_digit = reg_val0[(3-digit_idx)*4 +: 4];
        3'd1: cur_digit = reg_val1[(3-digit_idx)*4 +: 4];
        3'd2: cur_digit = reg_val2[(3-digit_idx)*4 +: 4];
        3'd3: cur_digit = reg_val3[(3-digit_idx)*4 +: 4];
        3'd4: cur_digit = reg_val4[(3-digit_idx)*4 +: 4];
        3'd5: cur_digit = reg_val5[(3-digit_idx)*4 +: 4];
        default: cur_digit = 4'd0;
    endcase
end

//==========================================================================
// 像素数据输出：优先显示图片，其次显示数值数字，最后背景色
//==========================================================================
always @(posedge lcd_pclk or negedge rst_n) begin
    if (!rst_n)
        pixel_data <= BACK_COLOR;
    else if((pixel_xpos>=PIC_X_START - 11'd1) && (pixel_xpos<PIC_X_START + PIC_WIDTH - 11'd1)
            && (pixel_ypos >= PIC_Y_START) && (pixel_ypos < PIC_Y_START + PIC_HEIGHT) )
        pixel_data <= rom_rd_data ; //显示图片
    else if(in_val_region && (cur_digit <= 4'd9)) begin
        // 仅当 nibble 为 0~9 时显示（A~F 暂不支持，显示为空白）
        // char[数字] 为 128bit 字模，每行 8bit，共 16 行
        // 第 y_offset 行对应字模的第 (15-y_offset) 字节（MSB 在上）
        // 字模内第 x_in_digit 列对应字节的第 (7-x_in_digit) 位（MSB 在左）
        if(char[cur_digit][(15 - y_offset[3:0]) * 8 + (7 - x_in_digit)])
            pixel_data <= CHAR_COLOR;  // 显示数字（黑色）
        else
            pixel_data <= BACK_COLOR;  // 数字背景（白色）
    end
    else
        pixel_data <= BACK_COLOR; //屏幕背景色
end

always@(posedge lcd_pclk or negedge rst_n)begin 
    if(!rst_n)
        rom_addr <= 15'd0;
//当扫描位置在图片显示区域时,累加 ROM 地址
    else if((pixel_ypos >= PIC_Y_START) && (pixel_ypos < PIC_Y_START + PIC_HEIGHT)
            && (pixel_xpos >= PIC_X_START-11'd2) && (pixel_xpos < PIC_X_START + PIC_WIDTH-11'd2))
        rom_addr <= rom_addr + 14'd1;
//当扫描位置在图片区域的最后一行时,ROM 地址清零
    else if((pixel_ypos >= PIC_Y_START + PIC_HEIGHT))
        rom_addr <= 14'd0;
end

blk_mem_gen_1 u_blk_mem_gen_1 (
  .clka(lcd_pclk)  ,    // input wire clka
  .ena(rom_rd_en)     ,      // input wire ena
  .addra(rom_addr) ,  // input wire [14 : 0] addra
  .douta(rom_rd_data)  // output wire [23 : 0] douta
);

endmodule

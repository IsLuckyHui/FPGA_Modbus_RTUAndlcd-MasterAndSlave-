module rs485_rx(
    input                sys_clk        ,  //系统时钟
    input                sys_rst_n      ,//系统复位
    
    input                rx_data        , //传入的数据 异步信号 跨时钟域 所以需要打拍

    output  reg    [7:0] tx_data       , //转换的串行数据
    output  reg          data_done        //转换完成信号
);

parameter   UART_BPS = 115200;
parameter   CLK_FREQ = 50000000; //系统时钟
//为得到指定波特率，对系统时钟计数 BPS_CNT 次
localparam BAUD_CNT_MAX = CLK_FREQ/UART_BPS;

wire      start_en;//开始发送使能 取第二拍和第三拍的上升沿
reg       rx_data_d0;
reg       rx_data_d1;
reg       rx_data_d2;
reg       rx_flag;
reg [8:0] baud_cnt;
reg [3:0] rx_cnt;
reg [7:0] tx_data_t;
//去打拍后的下降沿 如果俩拍的下降沿相同  代表有数据进来
assign start_en = rx_data_d2 & (~rx_data_d1) & (~rx_flag);

always@(posedge sys_clk or negedge sys_rst_n)begin
    if(!sys_rst_n)
        rx_flag <= 1'b0;
    else if(start_en)
        rx_flag <= 1'b1;
    else if((baud_cnt >= BAUD_CNT_MAX /2 - 9'd1)&&
            (rx_cnt == 4'd9))
        rx_flag <= 1'b0;
    else
        rx_flag <= rx_flag;
end 

//对波特率计数器进行计数 以115200，50MZh为例 发送一个bit位的数据需要434个周期 
always@(posedge sys_clk or negedge sys_rst_n)begin
    if(!sys_rst_n)begin
        baud_cnt <= 9'd0;
    end 
    else if(rx_flag)begin
        if (baud_cnt >= BAUD_CNT_MAX - 9'd1)
            baud_cnt <= 9'd0;
        else if((baud_cnt >= BAUD_CNT_MAX /2 - 9'd1)&&
        (rx_cnt == 4'd9))
            baud_cnt <= 9'd0;
        else
            baud_cnt <= baud_cnt + 9'd1;
    end 
    else
         baud_cnt <= 9'd0;
end 

//对发送的数据进行计数 共有开始位 + 8位数据 + 1停止位十位数据，
always@(posedge sys_clk or negedge sys_rst_n)begin
    if(!sys_rst_n)
        rx_cnt <= 4'd0;
    else if(rx_flag)begin
        if(baud_cnt >= BAUD_CNT_MAX - 9'd1)
            rx_cnt <= rx_cnt + 9'd1;
        else if((baud_cnt >= BAUD_CNT_MAX /2 - 9'd1)&&
            (rx_cnt == 4'd9))
            rx_cnt <= 4'd0;
        else
            rx_cnt <= rx_cnt;
    end
    else
        rx_cnt <= 4'd0;
end 

//对传入进来的数据进行打拍 抑制亚稳态
always@(posedge sys_clk or negedge sys_rst_n)begin
    if(!sys_rst_n)begin
        rx_data_d0    <= 1'd0;
        rx_data_d1    <= 1'd0;
        rx_data_d2    <= 1'd0;
    end 
    else begin
        rx_data_d0    <= rx_data;
        rx_data_d1    <= rx_data_d0;
        rx_data_d2    <= rx_data_d1;
    end 
end 

//给tx_data_t赋值
always@(posedge sys_clk or negedge sys_rst_n)begin
    if(!sys_rst_n)
        tx_data_t <= 8'd0;
    else if(rx_flag)begin
        if(baud_cnt == BAUD_CNT_MAX /2 - 9'd1) //判断当前是不是在数值的中间
            case(rx_cnt)
                4'd1: tx_data_t[0] <= rx_data_d2;
                4'd2: tx_data_t[1] <= rx_data_d2;
                4'd3: tx_data_t[2] <= rx_data_d2;
                4'd4: tx_data_t[3] <= rx_data_d2;
                4'd5: tx_data_t[4] <= rx_data_d2;
                4'd6: tx_data_t[5] <= rx_data_d2;
                4'd7: tx_data_t[6] <= rx_data_d2;
                4'd8: tx_data_t[7] <= rx_data_d2;
                default:;
            endcase 
        else
            tx_data_t <= tx_data_t;
    end
    else
         tx_data_t <= 8'd0;
end 

always@(posedge sys_clk or negedge sys_rst_n)begin
    if(!sys_rst_n)begin
        tx_data <= 8'd0;
        data_done <= 1'd0;
    end 
    else if((baud_cnt >= BAUD_CNT_MAX /2 - 9'd1)&&
           (rx_cnt == 4'd9))begin
        tx_data <= tx_data_t;
        data_done <= 1'd1;
    end
    else begin
        tx_data <= tx_data;
        data_done <= 1'd0;
    end 
end 


endmodule
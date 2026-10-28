module rs485_tx
(
    input             clk           ,
    input             rst_n         ,
    input    [7:0]    tx_data       ,
    input             tx_en         ,
    output  reg       uart_txd       ,
    output  reg       tx_busy
);


parameter   UART_BPS = 115200;
parameter   CLK_FREQ = 50000000; //系统时钟
//为得到指定波特率，对系统时钟计数 BPS_CNT 次
localparam BAUD_CNT_MAX = CLK_FREQ/UART_BPS;

reg [8:0] baud_cnt;
reg [3:0] tx_cnt;
reg [7:0] tx_data_t;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        tx_data_t <= 8'd0;
    else if(tx_en)
        tx_data_t <= tx_data;
    else if ((baud_cnt == BAUD_CNT_MAX - 9'd1)
            &&(tx_cnt == 4'd9))begin          
         tx_data_t <= 8'd0;
        end
    else
        tx_data_t <= tx_data_t;

end 

//输出忙信号
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        tx_busy <= 1'b0;
    else if(tx_en)
        tx_busy <= 1'b1;
    else if((baud_cnt == BAUD_CNT_MAX - 9'd1)&&
        (tx_cnt == 4'd9))
        tx_busy <= 1'b0;
    else
        tx_busy <= tx_busy;

end 

//对波特率计数器进行计数 以115200，50MZh为例 发送一个bit位的数据需要434个周期 
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        baud_cnt <= 9'd0;
    end 
    else if(tx_en)
        baud_cnt <= 16'd0;
    else if(tx_busy)begin
        if (baud_cnt < BAUD_CNT_MAX - 9'd1)
            baud_cnt <= baud_cnt + 9'd1;
        else
            baud_cnt <= 9'd0;
    end 
    else
         baud_cnt <= 9'd0;
end 

//对发送的数据进行计数 共有开始位 + 8位数据 + 1停止位十位数据，
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        tx_cnt <= 4'd0;
    else if(tx_en)
        tx_cnt <= 4'd0;    
    else if(tx_busy)begin
        if(baud_cnt == BAUD_CNT_MAX - 9'd1)
            tx_cnt <= tx_cnt + 9'd1;
        else
            tx_cnt <= tx_cnt;
    end
    else
        tx_cnt <= 4'd0;
end 


always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        uart_txd <= 1'b1; //串口空闲必须位高阻态
    else if(tx_busy)
       // if(baud_cnt == BAUD_CNT_MAX /2 - 9'd1)begin
            case(tx_cnt)
                4'd0: uart_txd <= 1'b0;
                4'd1: uart_txd <= tx_data_t[0];
                4'd2: uart_txd <= tx_data_t[1];
                4'd3: uart_txd <= tx_data_t[2];
                4'd4: uart_txd <= tx_data_t[3];
                4'd5: uart_txd <= tx_data_t[4];
                4'd6: uart_txd <= tx_data_t[5];
                4'd7: uart_txd <= tx_data_t[6];
                4'd8: uart_txd <= tx_data_t[7];
                4'd9: uart_txd <= 1'b1;
            default:uart_txd <= 1'b1;
            endcase
  
    else
        uart_txd <= 1'b1;
end 
endmodule
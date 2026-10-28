`timescale 1ns/1ns

module tb_rs485_rx;

reg  sys_clk;
reg  sys_rst_n;
reg  rx_data;

//wire   [7:0] tx_data;
//wire         data_done;
wire uart_tx ;
initial begin   
    sys_clk   <= 1'b0; 
    sys_rst_n <= 1'b0;
    rx_data <= 1'b1;
    #200
    sys_rst_n <= 1'b1;
    
    #8680
    rx_data <= 0;//起始位置
    #8680
    rx_data <= 1;
    #8680
    rx_data <= 1;
    #8680
    rx_data <= 1;
    #8680
    rx_data <= 1;
    #8680
    rx_data <= 1;
    #8680
    rx_data <= 1;
    #8680
    rx_data <= 1;
    #8680
    rx_data <= 0;
    #8680
    rx_data <= 1;// 结束位置
    
end 

always# 10 sys_clk = ~sys_clk;

/*rs485_rx u_rs485_rx
(
    .sys_clk        (sys_clk),
    .sys_rst_n      (sys_rst_n),
    .rx_data        (rx_data),
    .tx_data        (tx_data),
    .data_done      (data_done)                 
    );  */

rs485_top u_rs485_top(
    .sys_clk        (sys_clk  ),
    .sys_rst_n      (sys_rst_n),
    .uart_rx        (rx_data  ),
    .uart_tx        (uart_tx  )
    
);    
    
endmodule

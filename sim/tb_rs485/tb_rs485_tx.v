`timescale 1ns/1ns 

module tb_rs485_tx;
reg         clk      ;
reg         rst_n    ;
reg  [7:0]  tx_data  ;
reg         tx_en    ;
wire        rx_data  ;
wire        tx_busy  ;

initial begin   
    clk <= 1'b0;
    rst_n <= 1'b0;
    tx_en <= 1'b0;
    tx_data <= 8'h00 ;
    #200 
    rst_n <= 1'b1;
    #200
    tx_data <= 8'h55;
    #500
    tx_en <= 1'b1;
    #20
    tx_en <= 1'b0;
   
end 

always#10 clk = ~clk;


rs485_tx u_rs485_tx(
    .clk         (clk    ),
    .rst_n       (rst_n  ),
    .tx_data     (tx_data),
    .tx_en       (tx_en  ),
    .rx_data     (rx_data),
    .tx_busy     (tx_busy)

);


endmodule
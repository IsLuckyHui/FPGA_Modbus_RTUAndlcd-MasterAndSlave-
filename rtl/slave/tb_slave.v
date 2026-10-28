`timescale 1ns/1ns
module tb_slave;

reg         sys_clk;
reg         sys_rst_n;
reg  [7:0]  rx_data;
reg         start_en;
wire [7:0]  tx_data;
wire        tx_en;
reg         tx_busy;

integer i;
integer gN;                       //captured response byte count
integer bc;
reg [7:0] gbuf [0:31];
reg [7:0] ebuf [0:31];
reg       pass;

slave u_slave(
    .sys_clk   (sys_clk  ),
    .sys_rst_n (sys_rst_n),
    .rx_data   (rx_data  ),
    .start_en  (start_en ),
    .tx_data   (tx_data  ),
    .tx_en     (tx_en    ),
    .tx_busy   (tx_busy  )
);

initial begin
    sys_clk   = 1'b0;
    sys_rst_n = 1'b0;
    rx_data   = 8'd0;
    start_en  = 1'b0;
    tx_busy   = 1'b0;
    gN        = 0;
    pass      = 1'b1;
    #200 sys_rst_n = 1'b1;
    #1000;

    //------------------------------------------------------------------
    // test1: read 3 regs @1000H, trailing 4a 02 are crc bytes (ignored)
    // expect: 10 03 06 12 34 67 89 11 22
    //------------------------------------------------------------------
    drv(8'h10); drv(8'h03); drv(8'h10); drv(8'h00);
    drv(8'h00); drv(8'h03); drv(8'h4a); drv(8'h02);
    wait(gN == 9);
    ebuf[0]=8'h10; ebuf[1]=8'h03; ebuf[2]=8'h06; ebuf[3]=8'h12;
    ebuf[4]=8'h34; ebuf[5]=8'h67; ebuf[6]=8'h89; ebuf[7]=8'h11;
    ebuf[8]=8'h22;
    check(9, "test1 read3 @1000");
    #5000;

    //------------------------------------------------------------------
    // test2: read 1 reg @1001H
    // expect: 10 03 02 67 89
    //------------------------------------------------------------------
    drv(8'h10); drv(8'h03); drv(8'h10); drv(8'h01);
    drv(8'h00); drv(8'h01); drv(8'h00); drv(8'h00);
    wait(gN == 14);
    ebuf[0]=8'h10; ebuf[1]=8'h03; ebuf[2]=8'h02;
    ebuf[3]=8'h67; ebuf[4]=8'h89;
    check_at(9, 5, "test2 read1 @1001");
    #5000;

    //------------------------------------------------------------------
    // test3: wrong slave address 0x11 -> must stay silent
    //------------------------------------------------------------------
    drv(8'h11); drv(8'h03); drv(8'h10); drv(8'h00);
    drv(8'h00); drv(8'h01); drv(8'h00); drv(8'h00);
    #500000;                          //> 4-byte gap (347.2us)
    if(gN == 14)
        $display("test3 wrong addr  : OK (silent, gN=%0d)", gN);
    else begin
        $display("test3 wrong addr  : FAIL, unexpected %0d bytes", gN-14);
        pass = 1'b0;
    end

    $display("==========================================");
    if(pass) $display("==== SLAVE(no crc) RESULT: PASS ====");
    else     $display("==== SLAVE(no crc) RESULT: FAIL ====");
    #1000 $stop;
end

always #10 sys_clk = ~sys_clk;

initial begin
    #5000000;
    $display("==== timeout, gN=%0d ====", gN);
    $stop;
end

//drive one byte with a 1-cycle start_en pulse (byte-level uart rx model)
task drv;
    input [7:0] b;
    begin
        @(posedge sys_clk);
        rx_data  <= b;
        start_en <= 1'b1;
        @(posedge sys_clk);
        start_en <= 1'b0;
        repeat(30) @(posedge sys_clk);  //inter-byte spacing < 4-byte gap
    end
endtask

//capture response bytes
always@(posedge sys_clk) begin
    if(tx_en) begin
        if(gN < 32)
            gbuf[gN] <= tx_data;
        gN <= gN + 1;
    end
end

//emulate rs485_tx: busy for 50 clocks after tx_en
always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        tx_busy <= 1'b0;
        bc      <= 0;
    end
    else begin
        if(tx_en) begin
            tx_busy <= 1'b1;
            bc      <= 0;
        end
        else if(tx_busy) begin
            if(bc == 49)
                tx_busy <= 1'b0;
            else
                bc <= bc + 1;
        end
    end
end

task check;
    input integer num;
    input [255:0] tag;
    begin
        for(i=0;i<num;i=i+1) begin
            if(gbuf[i] !== ebuf[i]) begin
                $display("%s byte %0d: got %02h expect %02h MISMATCH",
                         tag, i, gbuf[i], ebuf[i]);
                pass = 1'b0;
            end
        end
        if(pass) $display("%s: OK -> %s", tag, "see waveform/bytes above");
    end
endtask

task check_at;
    input integer base;
    input integer num;
    input [255:0] tag;
    begin
        for(i=0;i<num;i=i+1) begin
            if(gbuf[base+i] !== ebuf[i]) begin
                $display("%s byte %0d: got %02h expect %02h MISMATCH",
                         tag, i, gbuf[base+i], ebuf[i]);
                pass = 1'b0;
            end
        end
        $display("%s: checked", tag);
    end
endtask

//print every response byte
always@(posedge sys_clk) begin
    if(tx_en)
        $display("tx #%0d = 0x%02h", gN, tx_data);
end

endmodule

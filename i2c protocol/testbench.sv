`timescale 1ns/1ns

module tb_i2c;

    // ---------------- Signals ----------------
    logic clk = 0;
    logic rst = 1;

    logic       start;
    logic [6:0] addr;
    logic [7:0] tx_byte;
    logic       rw;

    logic [7:0] rx_byte;
    logic       rx_valid;
    logic       busy;
    logic       ack_error;

    logic scl;
    tri   sda;
    pullup(sda);              // I2C pull-up

    // ---------------- DUT (master) ----------------
    i2c_master dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .addr(addr),
        .tx_byte(tx_byte),
        .rw(rw),
        .rx_byte(rx_byte),
        .rx_valid(rx_valid),
        .busy(busy),
        .ack_error(ack_error),
        .scl(scl),
        .sda(sda)
    );

    // 50 MHz system clock
    always #10 clk = ~clk;

    // ---------------- Very simple echo "slave" ----------------
    // We store the last written byte here:
    logic [7:0] slave_mem = 8'h00;

    // Master state encodings (must match enum order in i2c_master)
    localparam logic [3:0]
        ST_IDLE      = 4'd0,
        ST_START1    = 4'd1,
        ST_SEND_ADDR = 4'd2,
        ST_ADDR_ACK  = 4'd3,
        ST_SEND_DATA = 4'd4,
        ST_DATA_ACK  = 4'd5,
        ST_RECV_DATA = 4'd6,
        ST_RECV_ACK  = 4'd7,
        ST_STOP1     = 4'd8,
        ST_STOP2     = 4'd9;

    // We only ever drive SDA low (0) or release it (Z).
    // '1' comes from the pull-up.
    logic slave_drive;
    assign sda = slave_drive ? 1'b0 : 1'bz;

    always_comb begin
        slave_drive = 1'b0;

        // ACK after address and data (always ACK)
        if (dut.state == ST_ADDR_ACK || dut.state == ST_DATA_ACK) begin
            slave_drive = 1'b1;   // drive 0 -> ACK
        end
        // During RECV_DATA (master is reading), send bits of slave_mem
        else if (dut.state == ST_RECV_DATA) begin
            // For bit = 0, we pull SDA low; for bit = 1, we release (pull-up makes 1)
            if (slave_mem[dut.bit_cnt] == 1'b0)
                slave_drive = 1'b1;
            else
                slave_drive = 1'b0; // release SDA
        end
    end

    // ---------------- I2C helper tasks ----------------
    task automatic i2c_write(input [6:0] a, input [7:0] data);
    begin
        @(negedge clk);
        addr    = a;
        tx_byte = data;
        rw      = 1'b0;      // write
        start   = 1'b1;

        wait (busy == 1'b1);
        @(negedge clk);
        start = 1'b0;

        wait (busy == 1'b0);

        // At the end of a write, remember this byte in the "slave"
        slave_mem = data;
    end
    endtask

    task automatic i2c_read(input [6:0] a);
    begin
        @(negedge clk);
        addr    = a;
        tx_byte = 8'h00;     // don't care on read
        rw      = 1'b1;      // read
        start   = 1'b1;

        wait (busy == 1'b1);
        @(negedge clk);
        start = 1'b0;

        wait (busy == 1'b0);
    end
    endtask

    // ---------------- Main test sequence ----------------
    initial begin
        $dumpfile("tb_i2c.vcd");
        $dumpvars(0, tb_i2c);
        $dumpvars(0, tb_i2c.dut);

        start   = 0;
        addr    = '0;
        tx_byte = '0;
        rw      = 0;

        #100;
        rst = 0;
        #100;

        // Write A5 to pseudo-slave
        i2c_write(7'h50, 8'hA5);

        // Read back from same "slave"
        i2c_read(7'h50);

        $display("TX_BYTE = 0x%02h, RX_BYTE = 0x%02h, ack_error=%0d, rx_valid=%0d",
                 8'hA5, rx_byte, ack_error, rx_valid);

        #2000;
        $finish;
    end

endmodule

// design.sv
module i2c_master #(
    parameter int CLK_FREQ = 50_000_000,
    parameter int I2C_FREQ = 400_000
)(
    input  logic       clk,
    input  logic       rst,

    // Transaction control
    input  logic       start,
    input  logic [6:0] addr,
    input  logic [7:0] tx_byte,
    input  logic       rw,          // 0 = write, 1 = read

    // Returned data
    output logic [7:0] rx_byte,
    output logic       rx_valid,

    // Status
    output logic       busy,
    output logic       ack_error,

    // I2C bus
    output logic       scl,
    inout  tri         sda
);

    // Clock divider: generates enable pulses for SCL / FSM
    localparam int DIV = CLK_FREQ / (I2C_FREQ * 4);
    logic [15:0] div_cnt;
    logic        scl_en;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            div_cnt <= 0;
            scl_en  <= 0;
        end else if (div_cnt == DIV) begin
            div_cnt <= 0;
            scl_en  <= 1;
        end else begin
            div_cnt <= div_cnt + 1;
            scl_en  <= 0;
        end
    end

    // Simple SCL clock
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            scl <= 1;
        else if (scl_en)
            scl <= ~scl;
    end

    // FSM
    typedef enum logic [3:0] {
        IDLE, START1, SEND_ADDR, ADDR_ACK,
        SEND_DATA, DATA_ACK,
        RECV_DATA, RECV_ACK,
        STOP1, STOP2
    } state_t;

    state_t state;

    logic [7:0] shifter;
    logic [3:0] bit_cnt;
    logic       sda_out, sda_oe;

    // Open-drain SDA
    assign sda = sda_oe ? sda_out : 1'bz;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= IDLE;
            busy      <= 0;
            ack_error <= 0;
            rx_valid  <= 0;
            sda_oe    <= 1;
            sda_out   <= 1;
            shifter   <= 8'h00;
            bit_cnt   <= 4'd0;
            rx_byte   <= 8'h00;
        end
        else if (scl_en) begin
            rx_valid <= 0;   // default each scl_en tick

            case (state)
                IDLE: begin
                    sda_out <= 1;
                    sda_oe  <= 1;
                    busy    <= 0;
                    if (start) begin
    if (busy) begin
        // repeated START
        shifter <= {addr, rw};
        bit_cnt <= 7;
        state   <= START1;
    end else begin
        // normal start
        busy      <= 1;
        ack_error <= 0;
        shifter   <= {addr, rw};
        bit_cnt   <= 7;
        state     <= START1;
    end
end
                end

                START1: begin
                    sda_out <= 0;           // START condition
                    state   <= SEND_ADDR;
                end

                SEND_ADDR: begin
                    sda_oe  <= 1;
                    sda_out <= shifter[bit_cnt];
                    if (bit_cnt == 0)
                        state <= ADDR_ACK;
                    else
                        bit_cnt <= bit_cnt - 1;
                end

                ADDR_ACK: begin
                    sda_oe <= 0;            // release for ACK
                    if (scl) begin
                        ack_error <= sda;   // NACK if high
                        sda_oe    <= 1;
                        if (rw) begin
                            bit_cnt <= 7;
                            state   <= RECV_DATA;
                        end else begin
                            shifter <= tx_byte;
                            bit_cnt <= 7;
                            state   <= SEND_DATA;
                        end
                    end
                end

                SEND_DATA: begin
                    sda_oe  <= 1;
                    sda_out <= shifter[bit_cnt];
                    if (bit_cnt == 0)
                        state <= DATA_ACK;
                    else
                        bit_cnt <= bit_cnt - 1;
                end

                DATA_ACK: begin
                    sda_oe <= 0;
                    if (scl) begin
                        ack_error <= sda;
                        sda_oe    <= 1;
                        state     <= STOP1;
                    end
                end

                RECV_DATA: begin
                    sda_oe <= 0;            // slave drives
                    if (scl) begin
                        shifter[bit_cnt] <= sda;
                        if (bit_cnt == 0)
                            state <= RECV_ACK;
                        else
                            bit_cnt <= bit_cnt - 1;
                    end
                end

                RECV_ACK: begin
                    sda_oe   <= 1;
                    sda_out  <= 1;          // NACK after 1 byte
                    rx_byte  <= shifter;
                    rx_valid <= 1;
                    state    <= STOP1;
                end

                STOP1: begin
                    sda_out <= 0;
                    state   <= STOP2;
                end

                STOP2: begin
                    sda_out <= 1;           // STOP
                    busy    <= 0;
                    state   <= IDLE;
                end
            endcase
        end
    end

endmodule

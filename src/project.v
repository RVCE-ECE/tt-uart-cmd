/*
 * CRC-protected UART command interface
 * Tiny Tapeout SKY26e - RV College of Engineering
 *
 * Packet (host -> chip):  [0xA5][CMD][ADDR][DATA][CRC8]
 *   CRC8 = poly 0x07, init 0, no reflection, over CMD, ADDR, DATA
 *   CMD: 0x01 write reg, 0x02 read reg, 0x03 ping
 * Reply (chip -> host):   ACK 0x06 (+ value byte for a read) or NAK 0x15
 *
 * Registers: 0 GPIO_OUT[3:0], 1 GPIO_DIR[3:0], 2 PWM_DUTY[7:0],
 *            3 STATUS (read: {gpio_in[3:0], bad_packet_count[3:0]},
 *                      write: any value clears the bad packet counter)
 */
`default_nettype none

//------------------------------------------------------------------
// UART receiver: 8N1, samples in the middle of each bit
//------------------------------------------------------------------
module uart_rx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [9:0] clks_per_bit,
    input  wire       rx_in,
    output reg  [7:0] data,
    output reg        valid,      // 1-cycle pulse when a byte is received
    output reg        frame_err   // 1-cycle pulse when the stop bit is bad
);
    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [1:0] st;
    reg [9:0] cnt;
    reg [2:0] bitn;
    reg [7:0] sh;
    reg       s1, s2;             // 2-flop synchronizer

    wire [9:0] half = {1'b0, clks_per_bit[9:1]};
    wire [9:0] last = clks_per_bit - 10'd1;

    always @(posedge clk) begin
        valid     <= 1'b0;
        frame_err <= 1'b0;
        s1        <= rx_in;
        s2        <= s1;
        if (!rst_n) begin
            st   <= IDLE;
            cnt  <= 10'd0;
            bitn <= 3'd0;
            sh   <= 8'd0;
            data <= 8'd0;
            s1   <= 1'b1;
            s2   <= 1'b1;
        end else begin
            case (st)
                IDLE: begin
                    if (!s2) begin
                        st  <= START;
                        cnt <= 10'd0;
                    end
                end
                START: begin
                    if (cnt == half) begin
                        if (!s2) begin
                            st   <= DATA;
                            cnt  <= 10'd0;
                            bitn <= 3'd0;
                        end else begin
                            st <= IDLE;       // glitch, not a real start bit
                        end
                    end else begin
                        cnt <= cnt + 10'd1;
                    end
                end
                DATA: begin
                    if (cnt == last) begin
                        cnt <= 10'd0;
                        sh  <= {s2, sh[7:1]};
                        if (bitn == 3'd7) st <= STOP;
                        else              bitn <= bitn + 3'd1;
                    end else begin
                        cnt <= cnt + 10'd1;
                    end
                end
                STOP: begin
                    if (cnt == last) begin
                        cnt <= 10'd0;
                        st  <= IDLE;
                        if (s2) begin
                            data  <= sh;
                            valid <= 1'b1;
                        end else begin
                            frame_err <= 1'b1;
                        end
                    end else begin
                        cnt <= cnt + 10'd1;
                    end
                end
                default: st <= IDLE;
            endcase
        end
    end
endmodule

//------------------------------------------------------------------
// UART transmitter: 8N1
//------------------------------------------------------------------
module uart_tx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [9:0] clks_per_bit,
    input  wire       start,       // 1-cycle pulse, loads 'data'
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);
    reg [9:0] cnt;
    reg [3:0] bitn;
    reg [9:0] sh;                  // {stop, data[7:0], start}

    wire [9:0] last = clks_per_bit - 10'd1;

    always @(posedge clk) begin
        if (!rst_n) begin
            tx   <= 1'b1;
            busy <= 1'b0;
            cnt  <= 10'd0;
            bitn <= 4'd0;
            sh   <= 10'h3FF;
        end else if (!busy) begin
            if (start) begin
                sh   <= {1'b1, data, 1'b0};
                busy <= 1'b1;
                cnt  <= 10'd0;
                bitn <= 4'd0;
                tx   <= 1'b0;      // start bit
            end
        end else begin
            if (cnt == last) begin
                cnt <= 10'd0;
                if (bitn == 4'd9) begin
                    busy <= 1'b0;  // stop bit finished
                    tx   <= 1'b1;
                end else begin
                    bitn <= bitn + 4'd1;
                    sh   <= {1'b1, sh[9:1]};
                    tx   <= sh[1];
                end
            end else begin
                cnt <= cnt + 10'd1;
            end
        end
    end
endmodule

//------------------------------------------------------------------
// CRC-8 (poly 0x07), one byte per clock
//------------------------------------------------------------------
module crc8 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       clear,
    input  wire       en,
    input  wire [7:0] din,
    output reg  [7:0] crc
);
    function [7:0] step(input [7:0] c, input [7:0] d);
        reg [7:0] x;
        integer i;
        begin
            x = c ^ d;
            for (i = 0; i < 8; i = i + 1)
                x = x[7] ? ((x << 1) ^ 8'h07) : (x << 1);
            step = x;
        end
    endfunction

    always @(posedge clk) begin
        if (!rst_n || clear) crc <= 8'd0;
        else if (en)         crc <= step(crc, din);
    end
endmodule

//------------------------------------------------------------------
// Register file
//------------------------------------------------------------------
module reg_file (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       we,
    input  wire [1:0] addr,
    input  wire [7:0] wdata,
    output reg  [7:0] rdata,
    input  wire [3:0] gpio_in,
    input  wire       bad_inc,
    output reg  [3:0] gpio_out,
    output reg  [3:0] gpio_dir,
    output reg  [7:0] pwm_duty
);
    reg [3:0] bad_cnt;             // saturates at 15

    always @(posedge clk) begin
        if (!rst_n) begin
            gpio_out <= 4'd0;
            gpio_dir <= 4'd0;
            pwm_duty <= 8'd0;
            bad_cnt  <= 4'd0;
        end else begin
            if (we && addr == 2'd3)
                bad_cnt <= 4'd0;
            else if (bad_inc && bad_cnt != 4'hF)
                bad_cnt <= bad_cnt + 4'd1;

            if (we) begin
                case (addr)
                    2'd0:    gpio_out <= wdata[3:0];
                    2'd1:    gpio_dir <= wdata[3:0];
                    2'd2:    pwm_duty <= wdata;
                    default: ;
                endcase
            end
        end
    end

    always @* begin
        case (addr)
            2'd0:    rdata = {4'd0, gpio_out};
            2'd1:    rdata = {4'd0, gpio_dir};
            2'd2:    rdata = pwm_duty;
            default: rdata = {gpio_in, bad_cnt};
        endcase
    end
endmodule

//------------------------------------------------------------------
// Packet parser / command executor / reply sender
//------------------------------------------------------------------
module pkt_fsm (
    input  wire       clk,
    input  wire       rst_n,
    // from UART RX
    input  wire [7:0] rx_data,
    input  wire       rx_valid,
    input  wire       rx_frame_err,
    // CRC engine
    input  wire [7:0] crc_val,
    output wire       crc_clear,
    output wire       crc_en,
    // UART TX
    input  wire       tx_busy,
    output reg        tx_start,
    output reg  [7:0] tx_data,
    // register file
    output reg        reg_we,
    output reg  [1:0] reg_addr,
    output reg  [7:0] reg_wdata,
    input  wire [7:0] reg_rdata,
    // status
    output reg        pkt_ok,
    output reg        pkt_err,
    output reg        bad_inc
);
    localparam [3:0] S_IDLE   = 4'd0,
                     S_CMD    = 4'd1,
                     S_ADDR   = 4'd2,
                     S_DATA   = 4'd3,
                     S_CRC    = 4'd4,
                     S_REPLY1 = 4'd5,
                     S_WAIT1  = 4'd6,
                     S_REPLY2 = 4'd7,
                     S_WAIT2  = 4'd8;

    localparam [7:0] START_BYTE = 8'hA5,
                     ACK        = 8'h06,
                     NAK        = 8'h15;

    reg [3:0]  st;
    reg [7:0]  cmd, data;
    reg [1:0]  addr;
    reg        addr_ok, good;
    reg [15:0] tmo;                // receive timeout counter

    wire cmd_wr   = (cmd == 8'h01);
    wire cmd_rd   = (cmd == 8'h02);
    wire cmd_ping = (cmd == 8'h03);
    wire cmd_ok   = ((cmd_wr | cmd_rd) & addr_ok) | cmd_ping;
    wire crc_ok   = (rx_data == crc_val);
    wire in_rx    = (st >= S_CMD) && (st <= S_CRC);

    assign crc_clear = rx_valid && (st == S_IDLE) && (rx_data == START_BYTE);
    assign crc_en    = rx_valid && ((st == S_CMD) || (st == S_ADDR) || (st == S_DATA));

    always @(posedge clk) begin
        tx_start <= 1'b0;
        reg_we   <= 1'b0;
        bad_inc  <= 1'b0;
        if (!rst_n) begin
            st        <= S_IDLE;
            cmd       <= 8'd0;
            data      <= 8'd0;
            addr      <= 2'd0;
            addr_ok   <= 1'b0;
            good      <= 1'b0;
            tmo       <= 16'd0;
            tx_data   <= 8'd0;
            reg_addr  <= 2'd0;
            reg_wdata <= 8'd0;
            pkt_ok    <= 1'b0;
            pkt_err   <= 1'b0;
        end else begin
            // timeout counter only runs while a packet is being received
            if (in_rx) begin
                if (rx_valid)             tmo <= 16'd0;
                else if (tmo != 16'hFFFF) tmo <= tmo + 16'd1;
            end else begin
                tmo <= 16'd0;
            end

            case (st)
                S_IDLE: begin
                    if (rx_valid && rx_data == START_BYTE) begin
                        st      <= S_CMD;
                        pkt_ok  <= 1'b0;
                        pkt_err <= 1'b0;
                    end
                end
                S_CMD: begin
                    if (rx_valid) begin
                        cmd <= rx_data;
                        st  <= S_ADDR;
                    end
                end
                S_ADDR: begin
                    if (rx_valid) begin
                        addr    <= rx_data[1:0];
                        addr_ok <= (rx_data[7:2] == 6'd0);
                        st      <= S_DATA;
                    end
                end
                S_DATA: begin
                    if (rx_valid) begin
                        data <= rx_data;
                        st   <= S_CRC;
                    end
                end
                S_CRC: begin
                    if (rx_valid) begin
                        reg_addr  <= addr;
                        reg_wdata <= data;
                        if (crc_ok && cmd_ok) begin
                            good   <= 1'b1;
                            pkt_ok <= 1'b1;
                            reg_we <= cmd_wr;
                        end else begin
                            good    <= 1'b0;
                            pkt_err <= 1'b1;
                            bad_inc <= 1'b1;
                        end
                        st <= S_REPLY1;
                    end
                end
                S_REPLY1: begin
                    if (!tx_busy) begin
                        tx_data  <= good ? ACK : NAK;
                        tx_start <= 1'b1;
                        st       <= S_WAIT1;
                    end
                end
                S_WAIT1: begin
                    if (!tx_busy && !tx_start)
                        st <= (good && cmd_rd) ? S_REPLY2 : S_IDLE;
                end
                S_REPLY2: begin
                    if (!tx_busy) begin
                        tx_data  <= reg_rdata;
                        tx_start <= 1'b1;
                        st       <= S_WAIT2;
                    end
                end
                S_WAIT2: begin
                    if (!tx_busy && !tx_start) st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase

            // abort a partial packet on timeout or framing error
            if (in_rx && ((tmo == 16'hFFFF) || rx_frame_err)) begin
                st      <= S_IDLE;
                pkt_err <= 1'b1;
                bad_inc <= 1'b1;
                tmo     <= 16'd0;
            end
        end
    end
endmodule

//------------------------------------------------------------------
// Tiny Tapeout top level
//   ui_in[0]   rx (serial in)          uo_out[0] tx (serial out)
//   ui_in[2:1] baud select             uo_out[1] pkt_ok
//                                      uo_out[2] pkt_err
//   uio[3:0]   GPIO (dir by register)  uo_out[3] pwm_out
//                                      uo_out[4] rx_valid (debug pulse)
//------------------------------------------------------------------
module tt_um_rvce_uart_crc_cmd (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    wire       rx_pin   = ui_in[0];
    wire [1:0] baud_sel = ui_in[2:1];

    // clocks per UART bit (assumes a 50 MHz clock)
    reg [9:0] cpb;
    always @* begin
        case (baud_sel)
            2'd0:    cpb = 10'd434;   // 115200 baud
            2'd1:    cpb = 10'd868;   //  57600 baud
            2'd2:    cpb = 10'd217;   // 230400 baud
            default: cpb = 10'd16;    // fast mode (3.125 Mbaud), for simulation
        endcase
    end

    wire [7:0] rx_data;
    wire       rx_valid, rx_ferr;
    wire       tx_pin, tx_busy, tx_start;
    wire [7:0] tx_data;
    wire       crc_clear, crc_en;
    wire [7:0] crc_val;
    wire       reg_we;
    wire [1:0] reg_addr;
    wire [7:0] reg_wdata, reg_rdata;
    wire       pkt_ok, pkt_err, bad_inc;
    wire [3:0] gpio_out, gpio_dir;
    wire [7:0] pwm_duty;

    uart_rx u_rx (
        .clk(clk), .rst_n(rst_n), .clks_per_bit(cpb), .rx_in(rx_pin),
        .data(rx_data), .valid(rx_valid), .frame_err(rx_ferr)
    );

    uart_tx u_tx (
        .clk(clk), .rst_n(rst_n), .clks_per_bit(cpb),
        .start(tx_start), .data(tx_data), .tx(tx_pin), .busy(tx_busy)
    );

    crc8 u_crc (
        .clk(clk), .rst_n(rst_n), .clear(crc_clear), .en(crc_en),
        .din(rx_data), .crc(crc_val)
    );

    pkt_fsm u_fsm (
        .clk(clk), .rst_n(rst_n),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_frame_err(rx_ferr),
        .crc_val(crc_val), .crc_clear(crc_clear), .crc_en(crc_en),
        .tx_busy(tx_busy), .tx_start(tx_start), .tx_data(tx_data),
        .reg_we(reg_we), .reg_addr(reg_addr), .reg_wdata(reg_wdata),
        .reg_rdata(reg_rdata),
        .pkt_ok(pkt_ok), .pkt_err(pkt_err), .bad_inc(bad_inc)
    );

    reg_file u_regs (
        .clk(clk), .rst_n(rst_n),
        .we(reg_we), .addr(reg_addr), .wdata(reg_wdata), .rdata(reg_rdata),
        .gpio_in(uio_in[3:0]), .bad_inc(bad_inc),
        .gpio_out(gpio_out), .gpio_dir(gpio_dir), .pwm_duty(pwm_duty)
    );

    // PWM: 8-bit free-running counter, output high while counter < duty
    reg [7:0] pwm_cnt;
    reg       pwm_q;
    always @(posedge clk) begin
        if (!rst_n) begin
            pwm_cnt <= 8'd0;
            pwm_q   <= 1'b0;
        end else begin
            pwm_cnt <= pwm_cnt + 8'd1;
            pwm_q   <= (pwm_cnt < pwm_duty);
        end
    end

    assign uo_out  = {3'b000, rx_valid, pwm_q, pkt_err, pkt_ok, tx_pin};
    assign uio_out = {4'b0000, gpio_out};
    assign uio_oe  = {4'b0000, gpio_dir};

    wire _unused = &{ena, ui_in[7:3], uio_in[7:4], 1'b0};
endmodule

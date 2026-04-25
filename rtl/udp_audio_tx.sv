
// =====================================================================
// udp_audio_tx — minimalistic MII TX-only UDP audio streamer
//
//   Buffers 64 × 24-bit samples from the sys_clk domain, then emits one
//   fixed-layout Ethernet/IPv4/UDP frame per buffer on the MII TX pins
//   of the Arty A7's on-board DP83848J PHY.
//
//   Frame layout (246 bytes on wire incl. preamble/SFD):
//
//       7B preamble 0x55,  1B SFD 0xD5,
//      14B Ethernet (dst, src, 0x0800),
//      20B IPv4 (DF set, TTL=64, proto=UDP, precomputed hdr checksum),
//       8B UDP (sport, dport, len, csum=0),
//     192B payload (64 × 24-bit big-endian samples),
//       4B Ethernet FCS (CRC32, reflected polynomial 0xEDB88320).
//
//   No RX, no ARP (broadcast dst MAC), no MDIO (PHY default strapping
//   comes up in MII + auto-neg @ 100 Mbps FD), no UDP checksum.
//
//   CDC: sample_valid / sample_in is in the sys_clk (clk) domain. TX is
//   in eth_tx_clk (25 MHz MII TX clock, driven by the PHY). Sample data
//   crosses via a ping-pong RAM; the "new packet" handshake uses a
//   toggle-style synchronizer (same discipline as rtl/i2s_controller.sv).
// =====================================================================
module udp_audio_tx
    import jcm800_pkg::*;
#(
    parameter logic [47:0] SRC_MAC  = 48'h02_00_00_00_00_01,  // locally-administered
    parameter logic [47:0] DST_MAC  = 48'hFF_FF_FF_FF_FF_FF,  // broadcast
    parameter logic [31:0] SRC_IP   = {8'd192, 8'd168, 8'd1, 8'd10},
    parameter logic [31:0] DST_IP   = {8'd192, 8'd168, 8'd1, 8'd255},
    parameter logic [15:0] SRC_PORT = 16'hBEEF,
    parameter logic [15:0] DST_PORT = 16'hBEEF
)(
    // sys_clk (100 MHz) audio-sample ingress
    input  logic     clk,
    input  logic     rst_n,
    input  sample_t  sample_in,
    input  logic     sample_valid,

    // MII TX pins (PHY drives eth_tx_clk at 25 MHz for 100 Mbps)
    input  logic       eth_tx_clk,
    output logic [3:0] eth_txd,
    output logic       eth_tx_en,
    output logic       eth_rst_n
);

    // ------------------------------------------------------------------
    // Compile-time constants
    // ------------------------------------------------------------------
    localparam int N_SAMPS     = 64;
    localparam int PAYLOAD_LEN = N_SAMPS * 3;                    // 192
    localparam int HDR_LEN     = 14 + 20 + 8;                    // 42
    localparam int HDR_BITS    = HDR_LEN * 8;                    // 336

    localparam logic [15:0] IP_TOTAL_LEN = 16'(20 + 8 + PAYLOAD_LEN);  // 220
    localparam logic [15:0] UDP_LEN_16   = 16'(8 + PAYLOAD_LEN);       // 200

    localparam logic [8:0]  HDR_LAST_BYTE = 9'(HDR_LEN - 1);           //  41
    localparam logic [8:0]  PAY_LAST_BYTE = 9'(PAYLOAD_LEN - 1);       // 191

    // ------------------------------------------------------------------
    // Compile-time IPv4 header checksum (all fields constant)
    //   Fields summed (16-bit words, one's complement):
    //     0x4500, IP_TOTAL_LEN, 0x0000(id), 0x4000(DF), 0x4011(TTL+proto),
    //     0x0000(csum placeholder), SRC_IP hi/lo, DST_IP hi/lo
    // ------------------------------------------------------------------
    function automatic logic [15:0] calc_ip_csum(
        input logic [15:0] total_len,
        input logic [31:0] src_ip,
        input logic [31:0] dst_ip
    );
        logic [31:0] s;
    begin
        s = 32'h0;
        s = s + 32'h0000_4500;
        s = s + {16'h0, total_len};
        s = s + 32'h0000_4000;    // DF set, frag offset 0
        s = s + 32'h0000_4011;    // TTL=64, proto=UDP(17)
        s = s + {16'h0, src_ip[31:16]};
        s = s + {16'h0, src_ip[15: 0]};
        s = s + {16'h0, dst_ip[31:16]};
        s = s + {16'h0, dst_ip[15: 0]};
        s = (s & 32'hFFFF) + (s >> 16);
        s = (s & 32'hFFFF) + (s >> 16);
        calc_ip_csum = ~s[15:0];
    end
    endfunction

    localparam logic [15:0] IP_CSUM = calc_ip_csum(IP_TOTAL_LEN, SRC_IP, DST_IP);

    // ------------------------------------------------------------------
    // Packed fixed-header bits, big-endian / network byte order.
    // Byte index `i` (0-based from dst MAC) = HDR_BITS[(HDR_LEN-1-i)*8 +: 8].
    // ------------------------------------------------------------------
    logic [HDR_BITS-1:0] HDR_BITS_ROM;
    assign HDR_BITS_ROM = {
        // Ethernet (14 B)
        DST_MAC,
        SRC_MAC,
        16'h0800,                   // EtherType = IPv4
        // IPv4 (20 B)
        8'h45, 8'h00,
        IP_TOTAL_LEN,
        16'h0000,                   // identification
        16'h4000,                   // flags=DF, frag offset=0
        8'h40, 8'h11,               // TTL=64, proto=UDP
        IP_CSUM,
        SRC_IP,
        DST_IP,
        // UDP (8 B)
        SRC_PORT,
        DST_PORT,
        UDP_LEN_16,
        16'h0000                    // UDP checksum (omitted)
    };

    // ------------------------------------------------------------------
    // PHY reset: release PHY when board rst_n is released. The PHY latches
    // strap pins during reset; the Arty wiring configures MII + auto-neg.
    // ------------------------------------------------------------------
    assign eth_rst_n = rst_n;

    // ------------------------------------------------------------------
    // eth_tx_clk-domain synchronous reset (from sys-domain rst_n).
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic rst_sync_0, rst_sync_1;
    logic eth_rst_n_s;
    always_ff @(posedge eth_tx_clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_sync_0 <= 1'b0;
            rst_sync_1 <= 1'b0;
        end else begin
            rst_sync_0 <= 1'b1;
            rst_sync_1 <= rst_sync_0;
        end
    end
    assign eth_rst_n_s = rst_sync_1;

    // ==================================================================
    // Sample packer (sys_clk domain) + ping-pong sample buffer
    // ==================================================================
    logic        wr_buf;        // buffer currently being filled
    logic [5:0]  wr_idx;        // 0..63
    logic        pkt_req_tgl;   // toggles when a buffer finishes filling
    logic        tx_buf_clk;    // buffer index that was just filled
    sample_t     sample_mem [0:127];   // [{buf,idx}] — dual-clock inferred RAM

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_buf      <= 1'b0;
            wr_idx      <= 6'd0;
            pkt_req_tgl <= 1'b0;
            tx_buf_clk  <= 1'b0;
        end else if (sample_valid) begin
            if (wr_idx == 6'd63) begin
                wr_idx      <= 6'd0;
                wr_buf      <= ~wr_buf;
                pkt_req_tgl <= ~pkt_req_tgl;
                tx_buf_clk  <= wr_buf;
            end else begin
                wr_idx <= wr_idx + 6'd1;
            end
        end
    end

    // sample_mem in its own always_ff with no async reset, so Vivado can
    // infer it as a true dual-port RAM (distributed or BRAM).
    always_ff @(posedge clk) begin
        if (sample_valid)
            sample_mem[{wr_buf, wr_idx}] <= sample_in;
    end

    // ==================================================================
    // CDC: pkt_req_tgl + tx_buf_clk  →  eth_tx_clk
    // ==================================================================
    (* ASYNC_REG = "TRUE" *) logic rq_s0, rq_s1, rq_s2;
    (* ASYNC_REG = "TRUE" *) logic tb_s0, tb_s1;

    always_ff @(posedge eth_tx_clk or negedge eth_rst_n_s) begin
        if (!eth_rst_n_s) begin
            rq_s0 <= 1'b0; rq_s1 <= 1'b0; rq_s2 <= 1'b0;
            tb_s0 <= 1'b0; tb_s1 <= 1'b0;
        end else begin
            rq_s0 <= pkt_req_tgl;
            rq_s1 <= rq_s0;
            rq_s2 <= rq_s1;
            tb_s0 <= tx_buf_clk;
            tb_s1 <= tb_s0;
        end
    end

    wire pkt_start_pulse = rq_s1 ^ rq_s2;

    // ==================================================================
    // TX FSM (eth_tx_clk domain)
    // ==================================================================
    typedef enum logic [2:0] {
        S_IDLE,
        S_PREAMBLE,   // 7 × 0x55
        S_SFD,        // 1 × 0xD5
        S_HDR,        // 42 header bytes
        S_PAYLOAD,    // 192 payload bytes
        S_FCS,        // 4 FCS bytes
        S_IPG         // 12-byte inter-packet gap (tx_en low)
    } tx_state_t;

    tx_state_t st;
    logic [8:0]  byte_cnt;         // up to 192
    logic        nib_sel;          // 0 = low nibble first (MII order)

    logic        tx_buf_sel;       // latched at packet start
    logic [5:0]  pay_samp_idx;     // 0..63, current payload sample
    logic [1:0]  pay_byte_of_samp; // 0..2, byte-of-sample (0=MSB)
    sample_t     cur_sample;       // payload sample being emitted
    logic [31:0] crc;              // running CRC32 (reflected form)
    logic [31:0] fcs_shift;        // ~crc, shifted out nibble-by-nibble

    // --- Read port of sample_mem (eth_tx_clk) ---
    // Prefetch "one sample ahead" during S_PAYLOAD; otherwise point at
    // sample 0 so that sample_rd_data holds sample[0] when S_HDR ends.
    logic [6:0]  sample_rd_addr;
    sample_t     sample_rd_data;

    assign sample_rd_addr = (st == S_PAYLOAD)
                          ? {tx_buf_sel, pay_samp_idx + 6'd1}
                          : {tx_buf_sel, 6'd0};

    always_ff @(posedge eth_tx_clk) begin
        sample_rd_data <= sample_mem[sample_rd_addr];
    end

    // --- Current byte for HDR / PAYLOAD ---
    logic [7:0] hdr_cur_byte;
    logic [7:0] pay_cur_byte;

    assign hdr_cur_byte = HDR_BITS_ROM[(HDR_LEN-1 - byte_cnt)*8 +: 8];

    always_comb begin
        unique case (pay_byte_of_samp)
            2'd0:    pay_cur_byte = cur_sample[23:16];
            2'd1:    pay_cur_byte = cur_sample[15: 8];
            default: pay_cur_byte = cur_sample[ 7: 0];
        endcase
    end

    // --- Nibble to emit this cycle ---
    logic [3:0] tx_nib;
    logic       tx_en_nxt;

    always_comb begin
        tx_en_nxt = 1'b1;
        unique case (st)
            S_PREAMBLE: tx_nib = 4'h5;                                         // 0x55 = 5,5
            S_SFD:      tx_nib = nib_sel ? 4'hD : 4'h5;                        // 0xD5
            S_HDR:      tx_nib = nib_sel ? hdr_cur_byte[7:4] : hdr_cur_byte[3:0];
            S_PAYLOAD:  tx_nib = nib_sel ? pay_cur_byte[7:4] : pay_cur_byte[3:0];
            S_FCS:      tx_nib = fcs_shift[3:0];
            default: begin
                tx_nib    = 4'h0;
                tx_en_nxt = 1'b0;
            end
        endcase
    end

    // --- CRC32 nibble update (reflected, poly 0xEDB88320) ---
    function automatic logic [31:0] crc32_nib(
        input logic [31:0] c_in,
        input logic [ 3:0] nib
    );
        logic [31:0] c;
        integer i;
    begin
        c = c_in;
        for (i = 0; i < 4; i = i + 1)
            c = (c[0] ^ nib[i]) ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
        crc32_nib = c;
    end
    endfunction

    wire crc_en        = (st == S_HDR) || (st == S_PAYLOAD);
    wire [31:0] crc_nxt = crc_en ? crc32_nib(crc, tx_nib) : crc;

    // --- State machine ---
    always_ff @(posedge eth_tx_clk or negedge eth_rst_n_s) begin
        if (!eth_rst_n_s) begin
            st               <= S_IDLE;
            byte_cnt         <= 9'd0;
            nib_sel          <= 1'b0;
            tx_buf_sel       <= 1'b0;
            pay_samp_idx     <= 6'd0;
            pay_byte_of_samp <= 2'd0;
            cur_sample       <= '0;
            crc              <= 32'hFFFF_FFFF;
            fcs_shift        <= 32'h0;
            eth_txd          <= 4'h0;
            eth_tx_en        <= 1'b0;
        end else begin
            // Register MII outputs (1-cycle delay, friendly to IOB packing).
            eth_txd   <= tx_nib;
            eth_tx_en <= tx_en_nxt;

            // Running CRC update (only in HDR/PAYLOAD, see crc_en).
            crc <= crc_nxt;

            // FCS shift-out: one nibble per cycle while in S_FCS.
            if (st == S_FCS)
                fcs_shift <= {4'h0, fcs_shift[31:4]};

            // Nibble / byte progression.
            nib_sel <= ~nib_sel;

            unique case (st)
                S_IDLE: begin
                    nib_sel  <= 1'b0;
                    if (pkt_start_pulse) begin
                        tx_buf_sel <= tb_s1;
                        crc        <= 32'hFFFF_FFFF;
                        byte_cnt   <= 9'd0;
                        st         <= S_PREAMBLE;
                    end
                end

                S_PREAMBLE: if (nib_sel) begin
                    if (byte_cnt == 9'd6) begin
                        byte_cnt <= 9'd0;
                        st       <= S_SFD;
                    end else begin
                        byte_cnt <= byte_cnt + 9'd1;
                    end
                end

                S_SFD: if (nib_sel) begin
                    byte_cnt <= 9'd0;
                    st       <= S_HDR;
                end

                S_HDR: if (nib_sel) begin
                    if (byte_cnt == HDR_LAST_BYTE) begin
                        byte_cnt         <= 9'd0;
                        st               <= S_PAYLOAD;
                        pay_samp_idx     <= 6'd0;
                        pay_byte_of_samp <= 2'd0;
                        cur_sample       <= sample_rd_data;   // = sample[0]
                    end else begin
                        byte_cnt <= byte_cnt + 9'd1;
                    end
                end

                S_PAYLOAD: if (nib_sel) begin
                    // Advance byte-within-sample / sample index.
                    if (pay_byte_of_samp == 2'd2) begin
                        pay_byte_of_samp <= 2'd0;
                        pay_samp_idx     <= pay_samp_idx + 6'd1;
                        cur_sample       <= sample_rd_data;   // prefetched next
                    end else begin
                        pay_byte_of_samp <= pay_byte_of_samp + 2'd1;
                    end

                    if (byte_cnt == PAY_LAST_BYTE) begin
                        byte_cnt  <= 9'd0;
                        st        <= S_FCS;
                        fcs_shift <= ~crc_nxt;                // final CRC
                    end else begin
                        byte_cnt <= byte_cnt + 9'd1;
                    end
                end

                S_FCS: if (nib_sel) begin
                    if (byte_cnt == 9'd3) begin
                        byte_cnt <= 9'd0;
                        st       <= S_IPG;
                    end else begin
                        byte_cnt <= byte_cnt + 9'd1;
                    end
                end

                S_IPG: if (nib_sel) begin
                    if (byte_cnt == 9'd11) begin
                        byte_cnt <= 9'd0;
                        st       <= S_IDLE;
                    end else begin
                        byte_cnt <= byte_cnt + 9'd1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

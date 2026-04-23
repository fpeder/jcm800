
// =====================================================================
// pentode_stage — one EL34 of the push-pull power stage
//
//   Mirrors the LUT + PCHIP datapath of gain_stage.sv, but drops the
//   plate LPF / cathode shelf / coupling HPF blocks (handled in the OT
//   instead) and adds:
//     · a vg2_ratio output scale so that screen sag reduces plate-
//       current swing in real time (simpler proxy for the Koren
//       factorisation Ik(Vgk, k·Vg2) ≈ k·Ik(Vgk, Vg2_nom); the true
//       k^Ex form is available as a side LUT in lut_out/ but not
//       consumed here for simplicity)
//     · an |ip|-proxy screen-current output fed to screen_supply.sv.
//
//   Intra-stage order:
//
//     x_in ──► [Vgk clamp] ──► LUT + PCHIP ──► ×G_EL34 ──► ×vg2_ratio
//                                                      ├──► ip_out
//                                                      └──► |·| · IG2_RATIO ──► ig2_out
//
//   Both tubes A and B share the SAME el34_lut.mem content — the PI
//   upstream delivers anti-phase grid drives (y_pos / y_neg), so the
//   output transformer's Ip_A − Ip_B sum recovers push-pull operation
//   without any LUT content inversion.
//
//   LUT + PCHIP + ×G + ×vg2_ratio + ×IG2 all execute on ONE shared
//   32×32 → 64 signed multiplier exercised in strict dependency order.
//   Products, widths, rounding (`round_conv96`) and saturation are
//   bit-identical to the earlier parallel implementation; only the
//   intra-stage schedule changes.  The 26-cycle sequence fits inside
//   the ≈130 sys_clk per-sample budget at 768 kHz with 5× headroom.
//
//   ip_out / ip_valid drop out at cycle 23; ig2_out / ig2_valid at 27.
// =====================================================================
module pentode_stage
    import jcm800_pkg::*;
    import jcm800_power_pkg::*;
#(
    // Default LUT pair is the EL34 Ik curve; overridable so other nonlinear
    // tables (e.g. the OT saturation curve) can reuse the same LUT+PCHIP+×G
    // pipeline without duplicating hardware.  G_OVERRIDE_Q4_20 = 0 keeps the
    // package default (G_EL34 = +1.0); non-zero value lets callers retune.
    parameter string LUT_FILE = EL34_LUT_FILE,
    parameter string TAN_FILE = EL34_TAN_FILE,
    parameter logic signed [31:0] G_OVERRIDE_Q4_20 = 32'sh0
)(
    input  logic                clk,
    input  logic                rst_n,
    input  sample_t             x_in,
    input  logic                x_valid,
    // vg2_ratio is a slowly-varying Q2.14 unsigned scale (nom=1.0=0x4000)
    // delivered by screen_supply.sv.  Treated as signed [15:0] here so the
    // mul can use a signed DSP cell uniformly; values are always ≥0 in
    // practice so no sign-bit issues.
    input  logic signed [15:0]  vg2_ratio_q2_14,

    output sample_t             ip_out,
    output logic                ip_valid,
    output sample_t             ig2_out,
    output logic                ig2_valid
);

    // ================================================================
    // Vgk clamp — reserve 12 LSB of margin so BRAM addr < 4095
    // ================================================================
    sample_t x_clamped;
    always_comb begin
        if      ($signed(x_in) >  $signed(24'sh7FF000)) x_clamped =  24'sh7FF000;
        else if ($signed(x_in) < -$signed(24'sh7FF000)) x_clamped = -24'sh7FF000;
        else                                            x_clamped =  x_in;
    end

    // ================================================================
    // Latch the sample-domain input on its valid pulse.  Combinational
    // addr/t decode off s0_x remains identical to the original design.
    // ================================================================
    sample_t s0_x;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)        s0_x <= '0;
        else if (x_valid)  s0_x <= x_clamped;
    end

    logic [23:0] biased;
    logic [11:0] addr;
    logic [12:0] addr_p1;
    logic [10:0] t_c;

    assign biased  = s0_x ^ 24'h800000;
    assign addr    = biased[23:12];
    assign addr_p1 = {1'b0, addr} + 13'd1;
    assign t_c     = biased[11:1];

    // ================================================================
    // Dual BRAM reads (1-cycle latency).  BRAM outputs stay stable
    // while s0_x is stable (≥130 cycles between samples at 768 kHz),
    // so the scheduler reads them directly as operands.
    // ================================================================
    sample_t s3_y0, s3_y1;
    sample_t s3_t0, s3_t1;

    triode_bram #(.INIT_FILE(LUT_FILE)) u_lut (
        .clk    (clk),
        .addr_a ({1'b0, addr}),
        .addr_b (addr_p1),
        .q_a    (s3_y0),
        .q_b    (s3_y1)
    );
    triode_bram #(.INIT_FILE(TAN_FILE)) u_tan (
        .clk    (clk),
        .addr_a ({1'b0, addr}),
        .addr_b (addr_p1),
        .q_a    (s3_t0),
        .q_b    (s3_t1)
    );

    // ================================================================
    // Shared sequential multiplier
    //
    //   Exercises one 32×32 → 64 signed multiplier across nine products:
    //
    //     0) t  · t             → t²       (11×11 unsigned, fits 22b)
    //     1) t  · t²            → t³_raw   (11×22 unsigned, fits 33b)
    //     2) h00 · y0           → p00      (26×24 signed,    50b)
    //     3) h10 · t0           → p10
    //     4) h01 · y1           → p01
    //     5) h11 · t1           → p11
    //     6) G_EL34 · s6_y      → g_prod   (32×24 signed,    56b)
    //     7) vg2_ratio · g_y    → s_prod   (16×24 signed,    40b)
    //     8) |s_y| · IG2_RATIO  → ig2_prod (25×24 signed,    48b)
    //
    //   Dependent products each need a one-cycle bubble after the
    //   producing op; the four independent PCHIP products stream back
    //   to back.  Sequence length = 26 cycles (op_cnt 1..26).
    // ================================================================
    logic signed [31:0] mul_a_q, mul_b_q;
    logic signed [31:0] mul_a_d, mul_b_d;
    (* use_dsp = "yes" *) logic signed [63:0] mul_p_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_a_q <= '0;
            mul_b_q <= '0;
            mul_p_q <= '0;
        end else begin
            mul_a_q <= mul_a_d;
            mul_b_q <= mul_b_d;
            mul_p_q <= mul_a_q * mul_b_q;
        end
    end

    // ---- FSM counter -----------------------------------------------
    //   op_cnt == 0        : idle, waiting for x_valid
    //   op_cnt == 1..26    : running the sequence
    localparam int unsigned OP_LAST = 26;

    logic [4:0] op_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            op_cnt <= 5'd0;
        end else if (op_cnt == 5'd0) begin
            if (x_valid) op_cnt <= 5'd1;
        end else if (op_cnt == OP_LAST[4:0]) begin
            op_cnt <= 5'd0;
        end else begin
            op_cnt <= op_cnt + 5'd1;
        end
    end

    // ---- Intermediate holding registers ----------------------------
    logic        [21:0] t2_r;
    logic        [21:0] t3_r;
    logic signed [49:0] p00_r, p10_r, p01_r, p11_r;
    logic signed [51:0] sum52_r;
    logic signed [95:0] lut_y_96_r;
    logic signed [55:0] g_prod_r;
    logic signed [95:0] g_q23_96_r;
    logic signed [39:0] s_prod_r;
    logic signed [95:0] s_q23_96_r;
    logic signed [47:0] ig2_prod_r;
    logic signed [95:0] ig2_q23_96_r;

    // ---- h00..h11 combinational reconstruction ---------------------
    logic signed [25:0] t_q22_s, t2_q22_s, t3_q22_s;
    logic signed [25:0] one_q22;
    logic signed [25:0] h00_c, h10_c, h01_c, h11_c;

    assign t_q22_s  = $signed({4'b0000, t_c,  11'b000_0000_0000});
    assign t2_q22_s = $signed({4'b0000, t2_r});
    assign t3_q22_s = $signed({4'b0000, t3_r});
    assign one_q22  = 26'sd4194304;

    always_comb begin
        h00_c = one_q22 - (t2_q22_s <<< 1) - t2_q22_s + (t3_q22_s <<< 1);
        h01_c =          (t2_q22_s <<< 1) + t2_q22_s - (t3_q22_s <<< 1);
        h10_c = t_q22_s - (t2_q22_s <<< 1) + t3_q22_s;
        h11_c =                              t3_q22_s - t2_q22_s;
    end

    // ---- Sum / round / saturate combinational paths ----------------
    logic signed [51:0] sum52_c;
    logic signed [95:0] sum52_ext;
    logic signed [95:0] lut_y_96_c;
    sample_t            s6_y_c;
    logic signed [95:0] g_prod_ext;
    logic signed [95:0] g_q23_96_c;
    sample_t            g_y_c;
    logic signed [95:0] s_prod_ext;
    logic signed [95:0] s_q23_96_c;
    sample_t            s_y_c;
    sample_t            ip_abs_c;
    logic signed [95:0] ig2_prod_ext;
    logic signed [95:0] ig2_q23_96_c;
    sample_t            ig2_y_c;
    logic signed [23:0] ig2_ratio;

    assign ig2_ratio = IG2_RATIO_Q1_23;

    always_comb begin
        sum52_c      = $signed(p00_r) + $signed(p10_r)
                     + $signed(p01_r) + $signed(p11_r);
        sum52_ext    = {{44{sum52_r[51]}}, sum52_r};
        lut_y_96_c   = round_conv96(sum52_ext, 22);
        s6_y_c       = saturate_s24(lut_y_96_r);
        g_prod_ext   = {{40{g_prod_r[55]}}, g_prod_r};
        g_q23_96_c   = round_conv96(g_prod_ext, SHIFT_G_EL34);
        g_y_c        = saturate_s24(g_q23_96_r);
        s_prod_ext   = {{56{s_prod_r[39]}}, s_prod_r};
        s_q23_96_c   = round_conv96(s_prod_ext, 14);   // Q2.14 → shift 14
        s_y_c        = saturate_s24(s_q23_96_r);
        // abs on the saturated ip sample with the −2^23 corner handled
        // identically to the original design.
        if (s_y_c == 24'sh800000)      ip_abs_c = 24'sh7FFFFF;
        else if (s_y_c[23])            ip_abs_c = -s_y_c;
        else                           ip_abs_c =  s_y_c;
        ig2_prod_ext = {{48{ig2_prod_r[47]}}, ig2_prod_r};
        ig2_q23_96_c = round_conv96(ig2_prod_ext, 23);
        ig2_y_c      = saturate_s24(ig2_q23_96_r);
    end

    // ---- Operand mux -----------------------------------------------
    localparam logic signed [31:0] G_USED =
        (G_OVERRIDE_Q4_20 != 32'sh0) ? G_OVERRIDE_Q4_20 : G_EL34_Q4_20;
    localparam logic signed [31:0] DC = 32'sh0;

    function automatic logic signed [31:0] ext_u11 (input logic [10:0] x);
        ext_u11 = {21'b0, x};
    endfunction
    function automatic logic signed [31:0] ext_u22 (input logic [21:0] x);
        ext_u22 = {10'b0, x};
    endfunction
    function automatic logic signed [31:0] ext_s16 (input logic signed [15:0] x);
        ext_s16 = {{16{x[15]}}, x};
    endfunction
    function automatic logic signed [31:0] ext_s24 (input sample_t x);
        ext_s24 = {{8{x[23]}}, x};
    endfunction
    function automatic logic signed [31:0] ext_s26 (input logic signed [25:0] x);
        ext_s26 = {{6{x[25]}}, x};
    endfunction
    // ip_abs is a non-negative sample_t (Q1.23 with sign bit 0 or the
    // saturated positive rail).  Zero-extend to 32-bit signed.
    function automatic logic signed [31:0] ext_u24 (input sample_t x);
        ext_u24 = {8'b0, x};
    endfunction

    always_comb begin
        mul_a_d = DC;
        mul_b_d = DC;
        unique case (op_cnt)
            // 1: t*t        (t_c combinational off stable s0_x)
            5'd1:  begin mul_a_d = ext_u11(t_c);        mul_b_d = ext_u11(t_c);              end
            // 3: t*t²       (mul_p_q holds t² committed at posedge entering op 3)
            5'd3:  begin mul_a_d = ext_u11(t_c);        mul_b_d = ext_u22(mul_p_q[21:0]);    end
            // 6..9: PCHIP   (h00..h11 comb from t_c, t2_r, t3_r)
            5'd6:  begin mul_a_d = ext_s26(h00_c);      mul_b_d = ext_s24(s3_y0);            end
            5'd7:  begin mul_a_d = ext_s26(h10_c);      mul_b_d = ext_s24(s3_t0);            end
            5'd8:  begin mul_a_d = ext_s26(h01_c);      mul_b_d = ext_s24(s3_y1);            end
            5'd9:  begin mul_a_d = ext_s26(h11_c);      mul_b_d = ext_s24(s3_t1);            end
            // 14: ×G_EL34   (s6_y_c comb off lut_y_96_r latched at op 13)
            5'd14: begin mul_a_d = G_USED;              mul_b_d = ext_s24(s6_y_c);           end
            // 18: ×vg2_ratio (g_y_c comb off g_q23_96_r latched at op 17)
            5'd18: begin mul_a_d = ext_s16(vg2_ratio_q2_14); mul_b_d = ext_s24(g_y_c);       end
            // 22: ×IG2      (|ip_abs_c| comb via s_y_c off s_q23_96_r latched at op 21)
            5'd22: begin mul_a_d = ext_u24(ip_abs_c);   mul_b_d = ext_s24(ig2_ratio);        end
            default: begin mul_a_d = DC; mul_b_d = DC; end
        endcase
    end

    // ---- Capture pipeline ------------------------------------------
    logic    ip_valid_q;
    logic    ig2_valid_q;
    sample_t ip_out_q;
    sample_t ig2_out_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t2_r          <= '0;
            t3_r          <= '0;
            p00_r         <= '0;
            p10_r         <= '0;
            p01_r         <= '0;
            p11_r         <= '0;
            sum52_r       <= '0;
            lut_y_96_r    <= '0;
            g_prod_r      <= '0;
            g_q23_96_r    <= '0;
            s_prod_r      <= '0;
            s_q23_96_r    <= '0;
            ig2_prod_r    <= '0;
            ig2_q23_96_r  <= '0;
            ip_out_q      <= '0;
            ip_valid_q    <= 1'b0;
            ig2_out_q     <= '0;
            ig2_valid_q   <= 1'b0;
        end else begin
            ip_valid_q  <= 1'b0;
            ig2_valid_q <= 1'b0;
            unique case (op_cnt)
                // 3: mul_p_q holds t² (from op 1's t*t)
                5'd3:  t2_r <= mul_p_q[21:0];
                // 5: mul_p_q holds t³_raw = t * t² (from op 3's multiply)
                5'd5:  t3_r <= mul_p_q[32:11];
                // 8..11: four PCHIP products streaming out one per cycle
                5'd8:  p00_r <= mul_p_q[49:0];
                5'd9:  p10_r <= mul_p_q[49:0];
                5'd10: p01_r <= mul_p_q[49:0];
                5'd11: p11_r <= mul_p_q[49:0];
                // 12: sum of four PCHIP products
                5'd12: sum52_r   <= sum52_c;
                // 13: round sum52 by 22 → 96-bit convergent-rounded
                5'd13: lut_y_96_r <= lut_y_96_c;
                // 16: mul_p_q holds g_el34 * s6_y (from op 14's multiply)
                5'd16: g_prod_r   <= mul_p_q[55:0];
                // 17: round g_prod by SHIFT_G_EL34
                5'd17: g_q23_96_r <= g_q23_96_c;
                // 20: mul_p_q holds vg2_ratio * g_y (from op 18's multiply)
                5'd20: s_prod_r   <= mul_p_q[39:0];
                // 21: round s_prod by 14 (Q2.14)
                5'd21: s_q23_96_r <= s_q23_96_c;
                // 22: final saturate → ip_out; emit ip_valid pulse
                5'd22: begin
                    ip_out_q   <= s_y_c;
                    ip_valid_q <= 1'b1;
                end
                // 24: mul_p_q holds |ip_abs| * ig2_ratio (from op 22's multiply)
                5'd24: ig2_prod_r   <= mul_p_q[47:0];
                // 25: round ig2_prod by 23 (Q1.23)
                5'd25: ig2_q23_96_r <= ig2_q23_96_c;
                // 26: final saturate → ig2_out; emit ig2_valid pulse
                5'd26: begin
                    ig2_out_q   <= ig2_y_c;
                    ig2_valid_q <= 1'b1;
                end
                default: ;
            endcase
        end
    end

    assign ip_out    = ip_out_q;
    assign ip_valid  = ip_valid_q;
    assign ig2_out   = ig2_out_q;
    assign ig2_valid = ig2_valid_q;

endmodule

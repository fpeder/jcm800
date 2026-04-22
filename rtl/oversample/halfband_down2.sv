
// =====================================================================
// halfband_down2 — polyphase 2× downsampler (minimum-phase FIR)
//
//   Two input samples → one output sample.  The circular sample buffer
//   tracks every input; every second in_valid triggers a MAC sweep that
//   computes the anti-alias-filtered decimated output.
//
//   Polyphase: the lowpass H(z) = E0(z²) + z⁻¹ E1(z²).  The even-
//   indexed input stream feeds the E0 branch, the odd-indexed stream
//   feeds the E1 branch.  Output y[m] = Σ E0[k]·x[2m−2k] + Σ E1[k]·x[2m−1−2k].
//   Both sums accumulate into the same 48-bit acc for a single output.
//
//   Per-output cycles: K0 + K1 + 4.  Worst case (HB1, K0+K1=209) is
//   213 cycles, versus a 2-input budget of 2083 cycles at 96 kHz input
//   with sys_clk=100 MHz.  ~10% utilisation.
// =====================================================================
module halfband_down2
    import jcm800_pkg::*;
    import oversample_coeffs_pkg::*;
#(
    parameter int    K0      = 105,
    parameter int    K1      = 104,
    parameter string E0_FILE = "hb1_e0.mem",
    parameter string E1_FILE = "hb1_e1.mem"
)(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t in_sample,
    input  logic    in_valid,
    output sample_t out_sample,
    output logic    out_valid
);

    // ── Derived widths ──────────────────────────────────────────────
    localparam int KMAX       = (K0 > K1) ? K0 : K1;
    localparam int SBUF_DEPTH = 1 << $clog2(2 * KMAX);  // stride-2 addressing
    localparam int WP_W       = $clog2(SBUF_DEPTH);
    localparam int CNT_W      = $clog2(KMAX + 3);
    localparam int ACC_W      = 48;
    localparam int SAMPLE_W   = $bits(sample_t);

    localparam int SHIFT_AMT  = COEF_Q;   // downsample: no extra gain
    localparam logic signed [ACC_W-1:0] ROUND_BIT = 48'sd1 <<< (SHIFT_AMT - 1);

    // ── Storage ─────────────────────────────────────────────────────
    (* ram_style = "distributed" *) logic [COEF_W-1:0] e0_rom [0:K0-1];
    (* ram_style = "distributed" *) logic [COEF_W-1:0] e1_rom [0:K1-1];
    (* ram_style = "distributed" *) sample_t           sbuf   [0:SBUF_DEPTH-1];

    initial begin
        for (int i = 0; i < SBUF_DEPTH; i++) sbuf[i]   = '0;
        for (int i = 0; i < K0;         i++) e0_rom[i] = '0;
        for (int i = 0; i < K1;         i++) e1_rom[i] = '0;
        $readmemh(E0_FILE, e0_rom);
        $readmemh(E1_FILE, e1_rom);
    end

    logic [WP_W-1:0] wp;
    logic            phase;    // toggles per in_valid; trigger when phase→0

    // ── FSM ─────────────────────────────────────────────────────────
    typedef enum logic [2:0] {
        S_IDLE,
        S_E0_SCHED, S_E0_DRAIN,
        S_E1_SCHED, S_E1_DRAIN
    } state_t;

    state_t              state;
    logic [CNT_W-1:0]    cyc;
    logic [1:0]          drain_cnt;
    logic [WP_W-1:0]     wp_snap;  // wp snapshot at trigger — MAC uses this

    // ── Pipeline registers ──────────────────────────────────────────
    logic [COEF_W-1:0]        coef_r;
    sample_t                  samp_r;
    logic                     mac_en_r;
    logic signed [ACC_W-1:0]  acc;

    logic [CNT_W-1:0]    tap;
    logic [WP_W-1:0]     samp_addr;
    logic [COEF_W-1:0]   coef_mux;
    logic                scheduling;

    assign tap        = cyc;
    assign scheduling = (state == S_E0_SCHED) || (state == S_E1_SCHED);

    // Sample address — stride 2; base 1 for E0 (x[2m]..) or 2 for E1 (x[2m−1]..)
    always_comb begin
        unique case (state)
            S_E0_SCHED: samp_addr = (wp_snap - WP_W'(1) - (WP_W'(tap) <<< 1))
                                    & WP_W'(SBUF_DEPTH-1);
            S_E1_SCHED: samp_addr = (wp_snap - WP_W'(2) - (WP_W'(tap) <<< 1))
                                    & WP_W'(SBUF_DEPTH-1);
            default:    samp_addr = '0;
        endcase
    end

    always_comb begin
        unique case (state)
            S_E0_SCHED: coef_mux = e0_rom[tap[$clog2(K0)-1:0]];
            S_E1_SCHED: coef_mux = e1_rom[tap[$clog2(K1)-1:0]];
            default:    coef_mux = '0;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            wp         <= '0;
            wp_snap    <= '0;
            phase      <= 1'b0;
            cyc        <= '0;
            drain_cnt  <= '0;
            coef_r     <= '0;
            samp_r     <= '0;
            mac_en_r   <= 1'b0;
            acc        <= '0;
            out_sample <= '0;
            out_valid  <= 1'b0;
        end else begin
            out_valid <= 1'b0;

            // Buffer always tracks the input stream
            if (in_valid) begin
                sbuf[wp] <= in_sample;
                wp       <= wp + 1'b1;
                phase    <= ~phase;
            end

            // Pipeline stage 1: registered ROM + buffer read
            if (scheduling) begin
                coef_r   <= coef_mux;
                samp_r   <= sbuf[samp_addr];
                mac_en_r <= 1'b1;
            end else begin
                mac_en_r <= 1'b0;
            end

            // Pipeline stage 2: MAC (inferred DSP48, MREG + PREG)
            if (mac_en_r) begin
                acc <= acc + $signed(coef_r) * samp_r;
            end

            unique case (state)
                S_IDLE: begin
                    // Trigger output MAC after receiving the 2nd (even-index)
                    // input of a pair — phase was 1, about to toggle to 0.
                    if (in_valid && phase == 1'b1) begin
                        cyc       <= '0;
                        drain_cnt <= '0;
                        acc       <= ROUND_BIT;
                        wp_snap   <= wp + 1'b1;  // post-increment snapshot
                        state     <= S_E0_SCHED;
                    end
                end

                S_E0_SCHED: begin
                    if (cyc == CNT_W'(K0 - 1)) begin
                        drain_cnt <= '0;
                        state     <= S_E0_DRAIN;
                    end
                    cyc <= cyc + 1'b1;
                end

                S_E0_DRAIN: begin
                    drain_cnt <= drain_cnt + 1'b1;
                    // Wait for last E0 product to reach acc (2 cycles), then
                    // chain straight into E1 — acc continues accumulating.
                    if (drain_cnt == 2'd1) begin
                        cyc   <= '0;
                        state <= S_E1_SCHED;
                    end
                end

                S_E1_SCHED: begin
                    if (cyc == CNT_W'(K1 - 1)) begin
                        drain_cnt <= '0;
                        state     <= S_E1_DRAIN;
                    end
                    cyc <= cyc + 1'b1;
                end

                S_E1_DRAIN: begin
                    drain_cnt <= drain_cnt + 1'b1;
                    if (drain_cnt == 2'd1) begin
                        out_sample <= round_sat(acc);
                        out_valid  <= 1'b1;
                        state      <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    function automatic sample_t round_sat(input logic signed [ACC_W-1:0] a);
        logic signed [ACC_W-1:0] shifted;
        shifted = a >>> SHIFT_AMT;
        if (shifted[ACC_W-1] == 1'b0 && |shifted[ACC_W-2:SAMPLE_W-1])
            round_sat = 24'sh7FFFFF;
        else if (shifted[ACC_W-1] == 1'b1 && ~&shifted[ACC_W-2:SAMPLE_W-1])
            round_sat = 24'sh800000;
        else
            round_sat = shifted[SAMPLE_W-1:0];
    endfunction

endmodule

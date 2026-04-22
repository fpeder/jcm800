
// =====================================================================
// halfband_up2 — paced polyphase 2× upsampler (minimum-phase FIR)
//
//   One in_valid → two out_valid pulses spaced evenly across one input
//   period, so downstream modules see a clean stream at 2× the input
//   rate rather than a burst of two close-together valids.
//
//   Pacing is essential because the JCM800 preamp FSM drops `x_valid`
//   pulses that arrive while it is busy (~30 sys_clk cycles per sample),
//   and intermediate cascade stages have similar busy windows.  Without
//   pacing, the E0/E1 polyphase branches emit back-to-back within ~20
//   cycles, and every second sample gets lost.
//
//   Emission schedule (cyc counted from in_valid arrival):
//       E0 emit at cyc == IN_INTERVAL_CYC/4
//       E1 emit at cyc == 3·IN_INTERVAL_CYC/4
//     → spacing = IN_INTERVAL_CYC/2 = one output-rate period  ✓
//
//   MAC budget per branch: K0+2 and K1+2 cycles.  Both fit within the
//   quarter-period window even on the worst stage (HB1: K=105, window
//   ≈ 520 cyc at 48 kHz input).
//
//   Gain: the ×2 upsample amplitude compensation (for the zero-stuffed
//   signal's halved amplitude) is baked into the output rounder as a
//   right-shift by COEF_Q−1 instead of COEF_Q.  The same .mem files are
//   shared between up and down stages.
// =====================================================================
module halfband_up2
    import jcm800_pkg::*;
    import oversample_coeffs_pkg::*;
#(
    parameter int    K0              = 105,
    parameter int    K1              = 104,
    parameter int    IN_INTERVAL_CYC = 2083,   // sys_clk cycles between in_valid pulses
    parameter string E0_FILE         = "hb1_e0.mem",
    parameter string E1_FILE         = "hb1_e1.mem"
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
    localparam int SBUF_DEPTH = 1 << $clog2(KMAX);
    localparam int WP_W       = $clog2(SBUF_DEPTH);
    localparam int CNT_W      = $clog2(KMAX + 3);
    localparam int ACC_W      = 48;
    localparam int SAMPLE_W   = $bits(sample_t);

    localparam int SHIFT_AMT  = COEF_Q - 1;   // ×2 upsample gain
    localparam logic signed [ACC_W-1:0] ROUND_BIT = 48'sd1 <<< (SHIFT_AMT - 1);

    localparam int CYC_W        = $clog2(IN_INTERVAL_CYC + 1);
    localparam int E0_EMIT_CYC  = IN_INTERVAL_CYC / 4;
    localparam int E1_EMIT_CYC  = (IN_INTERVAL_CYC * 3) / 4;

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
    logic [WP_W-1:0] wp_snap;

    // ── FSM ─────────────────────────────────────────────────────────
    typedef enum logic [2:0] {
        S_IDLE,
        S_MAC_E0, S_WAIT_E0,
        S_MAC_E1, S_WAIT_E1
    } state_t;

    state_t              state;
    logic [CNT_W-1:0]    cyc;         // local counter inside the MAC states
    logic [1:0]          drain_cnt;
    logic [CYC_W-1:0]    sample_cyc;  // free-running counter: 0..IN_INTERVAL_CYC-1

    // ── Pipeline registers ──────────────────────────────────────────
    logic [COEF_W-1:0]        coef_r;
    sample_t                  samp_r;
    logic                     mac_en_r;
    logic signed [ACC_W-1:0]  acc;
    sample_t                  e0_result, e1_result;

    logic [CNT_W-1:0]    tap;
    logic [WP_W-1:0]     samp_addr;
    logic [COEF_W-1:0]   coef_mux;
    logic                scheduling;

    assign tap        = cyc;
    assign samp_addr  = (wp_snap - WP_W'(1) - WP_W'(tap)) & WP_W'(SBUF_DEPTH-1);
    assign scheduling = (state == S_MAC_E0) || (state == S_MAC_E1);

    always_comb begin
        unique case (state)
            S_MAC_E0: coef_mux = e0_rom[tap[$clog2(K0)-1:0]];
            S_MAC_E1: coef_mux = e1_rom[tap[$clog2(K1)-1:0]];
            default:  coef_mux = '0;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            wp         <= '0;
            wp_snap    <= '0;
            cyc        <= '0;
            drain_cnt  <= '0;
            sample_cyc <= '0;
            coef_r     <= '0;
            samp_r     <= '0;
            mac_en_r   <= 1'b0;
            acc        <= '0;
            e0_result  <= '0;
            e1_result  <= '0;
            out_sample <= '0;
            out_valid  <= 1'b0;
        end else begin
            out_valid <= 1'b0;

            // Free-running sample cycle counter — reset on in_valid
            if (in_valid)        sample_cyc <= '0;
            else if (state != S_IDLE) sample_cyc <= sample_cyc + 1'b1;

            // Pipeline stage 1: registered ROM / buffer read
            if (scheduling) begin
                coef_r   <= coef_mux;
                samp_r   <= sbuf[samp_addr];
                mac_en_r <= 1'b1;
            end else begin
                mac_en_r <= 1'b0;
            end
            // Pipeline stage 2: MAC (inferred as DSP48 MREG+PREG)
            if (mac_en_r) begin
                acc <= acc + $signed(coef_r) * samp_r;
            end

            unique case (state)
                S_IDLE: begin
                    if (in_valid) begin
                        sbuf[wp]  <= in_sample;
                        wp        <= wp + 1'b1;
                        wp_snap   <= wp + 1'b1;
                        cyc       <= '0;
                        drain_cnt <= '0;
                        acc       <= ROUND_BIT;
                        state     <= S_MAC_E0;
                    end
                end

                S_MAC_E0: begin
                    // Schedule K0 reads, then drain 2 cycles
                    if (cyc == CNT_W'(K0 - 1)) begin
                        drain_cnt <= '0;
                        state     <= S_WAIT_E0;
                    end
                    cyc <= cyc + 1'b1;
                end

                S_WAIT_E0: begin
                    // After 2-cycle drain, latch acc into e0_result,
                    // then sit in this state until E0_EMIT_CYC is reached.
                    if (drain_cnt < 2'd2) begin
                        drain_cnt <= drain_cnt + 1'b1;
                        if (drain_cnt == 2'd1)
                            e0_result <= round_sat(acc);
                    end

                    if (sample_cyc + 1'b1 == CYC_W'(E0_EMIT_CYC)) begin
                        // Emit on the NEXT cycle so it lines up at E0_EMIT_CYC
                        out_sample <= e0_result;
                        out_valid  <= 1'b1;
                        // Start E1 MAC
                        cyc        <= '0;
                        drain_cnt  <= '0;
                        acc        <= ROUND_BIT;
                        state      <= S_MAC_E1;
                    end
                end

                S_MAC_E1: begin
                    if (cyc == CNT_W'(K1 - 1)) begin
                        drain_cnt <= '0;
                        state     <= S_WAIT_E1;
                    end
                    cyc <= cyc + 1'b1;
                end

                S_WAIT_E1: begin
                    if (drain_cnt < 2'd2) begin
                        drain_cnt <= drain_cnt + 1'b1;
                        if (drain_cnt == 2'd1)
                            e1_result <= round_sat(acc);
                    end

                    if (sample_cyc + 1'b1 == CYC_W'(E1_EMIT_CYC)) begin
                        out_sample <= e1_result;
                        out_valid  <= 1'b1;
                        state      <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Round-to-nearest, saturate to s24, with ×2 upsample gain shift
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

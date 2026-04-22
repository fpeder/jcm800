
// =====================================================================
// ir_cab — 1024-tap cabinet-simulation FIR (48 kHz)
//
//   Convolves the 48 kHz DAC-path signal with a measured cabinet IR
//   (Orange 2x12 + SM57 on-axis) — the missing speaker coloration that
//   turns the bare transformer-secondary signal into a miked-cab sound.
//
//   Structure mirrors halfband_down2.sv: circular sample buffer + serial
//   MAC, one output per K_TAPS+3 sys_clk cycles.  Single-rate (no poly-
//   phase) so the FSM is simpler — one SCHED pass of K_TAPS taps, then a
//   2-cycle DRAIN for the pipeline tail, then the output pulse.
//
//   Budget @ 48 kHz / sys_clk = 100 MHz: 2083 cycles/sample.
//   Per-output cycles: K_TAPS + 3 = 1027  (~49% utilisation).
//
//   Fixed-point:
//     coef   = Q1.17, 18-bit signed (matches the halfband .mem format)
//     sample = Q1.23, 24-bit signed (jcm800_pkg::sample_t)
//     prod   = Q2.40, 42-bit signed
//     acc    = 48-bit (fits a DSP48E1 P register so the MAC folds into
//             one DSP — holds worst-case magnitude up to ±256 in Q2.40
//             units, far above any real cabinet IR's L1 norm × sample)
//     out    = convergent-round + saturate back to sample_t
// =====================================================================
module ir_cab
    import jcm800_pkg::*;
#(
    parameter int    K_TAPS    = 1024,              // must be power of two
    parameter int    COEF_W    = 18,
    parameter int    COEF_Q    = 17,
    parameter string COEF_FILE = "ir_cab.mem"
)(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t in_sample,
    input  logic    in_valid,
    output sample_t out_sample,
    output logic    out_valid
);

    // ── Derived widths ──────────────────────────────────────────────
    localparam int AP_W      = $clog2(K_TAPS);      // 10 for K_TAPS=1024
    localparam int CNT_W     = $clog2(K_TAPS + 3);
    localparam int ACC_W     = 48;
    localparam int SAMPLE_W  = $bits(sample_t);

    localparam int SHIFT_AMT = COEF_Q;
    localparam logic signed [ACC_W-1:0] ROUND_BIT = ACC_W'(1) <<< (SHIFT_AMT - 1);

    // ── Storage ─────────────────────────────────────────────────────
    //   Both RAMs have no reset and synchronous read → BRAM18-inferrable
    //   (canonical Xilinx template).  sbuf_reg_r holds the registered
    //   read; coef_rom is read via coef_rom_reg_r.
    (* ram_style = "block" *) logic [COEF_W-1:0] coef_rom [0:K_TAPS-1];
    (* ram_style = "block" *) sample_t           sbuf     [0:K_TAPS-1];

    initial begin
        for (int i = 0; i < K_TAPS; i++) sbuf[i] = '0;
        $readmemh(COEF_FILE, coef_rom);
    end

    logic [AP_W-1:0] wp;
    logic [AP_W-1:0] wp_snap;

    // ── FSM ─────────────────────────────────────────────────────────
    typedef enum logic [1:0] {
        S_IDLE,
        S_SCHED,
        S_DRAIN
    } state_t;

    state_t           state;
    logic [CNT_W-1:0] cyc;
    logic [1:0]       drain_cnt;

    // ── Pipeline registers ──────────────────────────────────────────
    logic [COEF_W-1:0]        coef_r;
    sample_t                  samp_r;
    logic                     mac_en_r;
    logic signed [ACC_W-1:0]  acc;

    logic [AP_W-1:0]     samp_addr;
    logic                scheduling;

    assign scheduling = (state == S_SCHED);

    // tap index k=cyc sweeps 0..K_TAPS-1 ; sample read = x[n-k]
    assign samp_addr = (wp_snap - AP_W'(1) - cyc[AP_W-1:0]) & AP_W'(K_TAPS-1);

    // ── RAM ports (no reset → BRAM18 template) ──────────────────────
    always_ff @(posedge clk) begin
        if (in_valid)    sbuf[wp] <= in_sample;
        if (scheduling)  samp_r   <= sbuf[samp_addr];
    end

    always_ff @(posedge clk) begin
        if (scheduling)  coef_r   <= coef_rom[cyc[AP_W-1:0]];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            wp         <= '0;
            wp_snap    <= '0;
            cyc        <= '0;
            drain_cnt  <= '0;
            mac_en_r   <= 1'b0;
            acc        <= '0;
            out_sample <= '0;
            out_valid  <= 1'b0;
        end else begin
            out_valid <= 1'b0;

            if (in_valid) wp <= wp + AP_W'(1);

            // Pipeline stage 1 valid (coef_r / samp_r loaded in the RAM blocks)
            mac_en_r <= scheduling;

            // Pipeline stage 2: MAC (inferred DSP48, MREG + PREG)
            if (mac_en_r) begin
                acc <= acc + $signed(coef_r) * samp_r;
            end

            unique case (state)
                S_IDLE: begin
                    if (in_valid) begin
                        cyc       <= '0;
                        drain_cnt <= '0;
                        acc       <= ROUND_BIT;
                        wp_snap   <= wp + AP_W'(1);   // post-increment snapshot
                        state     <= S_SCHED;
                    end
                end

                S_SCHED: begin
                    if (cyc == CNT_W'(K_TAPS - 1)) begin
                        drain_cnt <= '0;
                        state     <= S_DRAIN;
                    end
                    cyc <= cyc + 1'b1;
                end

                S_DRAIN: begin
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

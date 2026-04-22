
// =====================================================================
// jcm800_pkg — shared types and arithmetic helpers
//
//   sample_t        — Q1.23 signed 24-bit audio word (matches codec)
//   round_conv96    — convergent (round-half-to-even) requantisation.
//                     Used by every IIR filter and the ×G_stage path to
//                     eliminate the DC bias that plain truncation introduces
//                     and that subsequent stages would otherwise amplify.
//   saturate_s24    — signed 24-bit saturating narrow on the LUT/ADC path.
//
// Convergent-rounding rule (IEEE 754 round-to-nearest-even):
//
//   y = x / 2^n
//       • if remainder < half     → truncate toward −∞ (arith shift)
//       • if remainder > half     → round up
//       • if remainder == half    → round to even (LSB cleared)
//
//   Implementation avoids runtime bit-select of x[n−1]: we detect the
//   exactly-half case with a mask compare, add a half-bias, arithmetic-
//   shift, and subtract 1 when the pre-rounded result is odd and we were
//   exactly at the boundary.  Works symmetrically for positive and
//   negative x (verified: x=-5, x=-2, x=-6, etc., all round correctly).
// =====================================================================
package jcm800_pkg;

    typedef logic signed [23:0] sample_t;

    // ------------------------------------------------------------------
    // Convergent rounding — 96-bit wide input, shift amount `n`.
    // Caller must size the input wide enough to hold the pre-shift value
    // without overflow.  96 bits covers every path in this project:
    //   shelf accumulator (Q12.68 ≤ 80 bits) + headroom
    //   HPF product (Q10.70 = 80 bits) + headroom
    //   LPF product / gain product (≤ 80 bits)
    // The return value is still 96-bit — caller slices and sign-extends.
    // ------------------------------------------------------------------
    function automatic logic signed [95:0] round_conv96(
        input logic signed [95:0] x,
        input int                 n
    );
        logic signed [95:0] half_val;
        logic signed [95:0] low_mask;
        logic signed [95:0] biased;
        logic signed [95:0] rounded_up;
        logic               was_half;
    begin
        if (n <= 0) begin
            round_conv96 = x;
        end else begin
            half_val   = 96'sd1 <<< (n - 1);
            low_mask   = (96'sd1 <<< n) - 96'sd1;
            biased     = x + half_val;
            rounded_up = biased >>> n;
            was_half   = ((x & low_mask) == half_val);
            if (was_half && rounded_up[0])
                round_conv96 = rounded_up - 96'sd1;
            else
                round_conv96 = rounded_up;
        end
    end
    endfunction

    // ------------------------------------------------------------------
    // Signed saturation to 24-bit sample_t.  Takes a wider signed value
    // (any width ≥ 25) and clamps out-of-range high/low rails.  Used on
    // the narrowed output of the gain / LUT / filter paths.
    // ------------------------------------------------------------------
    function automatic sample_t saturate_s24(input logic signed [95:0] x);
        logic signed [95:0] hi_lim;
        logic signed [95:0] lo_lim;
    begin
        hi_lim = 96'sh0000_0000_0000_0000_0000_0000 | 96'sd8388607;   // +2^23 - 1
        lo_lim = -96'sd8388608;                                        // -2^23
        if (x > hi_lim)      saturate_s24 = 24'sh7FFFFF;
        else if (x < lo_lim) saturate_s24 = 24'sh800000;
        else                 saturate_s24 = x[23:0];
    end
    endfunction

endpackage

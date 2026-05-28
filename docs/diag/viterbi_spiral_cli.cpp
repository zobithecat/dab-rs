// docs/diag/viterbi_spiral_cli.cpp
//
// Slice-8 standalone harness for the *oracle* (eti-stuff `viterbiSpiral`)
// side of the synthetic Viterbi unit test. Builds a tiny CLI that reads
// 2304 signed-byte soft bits from stdin, depunctures per the FIC
// puncture table (PI_16 × 21 + PI_15 × 3 + PI_X × 1), runs them through
// `viterbiSpiral::deconvolve`, and writes 768 hard bits (one bit per
// byte) to stdout. Same I/O protocol as the dab-rs `dab viterbi-cli`
// subcommand so a Python comparator can pipe identical inputs to both
// and bit-XOR the outputs.
//
// Build (macOS arm64 example — no SIMD intrinsics needed, the spiral-
// no-sse variant builds cleanly with stock clang):
//
//   ETI=/path/to/airspy-mini-dmb/eti-stuff/eti-cmdline
//   c++ -std=c++11 -O2                                                \
//       -I "$ETI/includes"                                            \
//       -I "$ETI/includes/eti-handling"                               \
//       -I "$ETI/includes/eti-handling/viterbi-spiral"                \
//       docs/diag/viterbi_spiral_cli.cpp                              \
//       "$ETI/src/eti-handling/viterbi-spiral/viterbi-spiral.cpp"     \
//       "$ETI/src/eti-handling/viterbi-spiral/spiral-no-sse.c"        \
//       "$ETI/src/eti-handling/protTables.cpp"                        \
//       -o /tmp/viterbi_spiral_cli

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "viterbi-spiral.h"
#include "protTables.h"

static const int FIC_IN_BITS      = 2304;
static const int FIC_VITERBI_LEN  = 3072 + 24;
static const int FIC_OUT_BITS     = 768;

int main () {
    // ---- Build the FIC puncture table (verbatim from ficHandler ctor) ----
    bool punctureTable [FIC_VITERBI_LEN];
    std::memset (punctureTable, 0, sizeof (punctureTable));
    int local = 0;
    int8_t *pi_16 = get_PCodes (16 - 1);
    int8_t *pi_15 = get_PCodes (15 - 1);
    int8_t *pi_x  = get_PCodes (8  - 1);
    for (int i = 0; i < 21; i++)
        for (int k = 0; k < 32 * 4; k++) {
            if (pi_16 [k % 32] != 0) punctureTable [local] = true;
            local++;
        }
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 32 * 4; k++) {
            if (pi_15 [k % 32] != 0) punctureTable [local] = true;
            local++;
        }
    for (int k = 0; k < 24; k++) {
        if (pi_x [k] != 0) punctureTable [local] = true;
        local++;
    }
    if (local != FIC_VITERBI_LEN) {
        std::fprintf (stderr,
                      "puncture table length mismatch: %d vs %d\n",
                      local, FIC_VITERBI_LEN);
        return 2;
    }

    // ---- Read 2304 signed bytes from stdin ----
    int8_t input [FIC_IN_BITS];
    size_t got = std::fread (input, 1, FIC_IN_BITS, stdin);
    if (got != (size_t) FIC_IN_BITS) {
        std::fprintf (stderr,
                      "expected %d bytes from stdin, got %zu\n",
                      FIC_IN_BITS, got);
        return 1;
    }

    // ---- Depuncture into 3096 i16 ----
    int16_t viterbiBlock [FIC_VITERBI_LEN];
    std::memset (viterbiBlock, 0, sizeof (viterbiBlock));
    int ic = 0;
    for (int i = 0; i < FIC_VITERBI_LEN; i++)
        if (punctureTable [i])
            viterbiBlock [i] = (int16_t) input [ic++];
    if (ic != FIC_IN_BITS) {
        std::fprintf (stderr,
                      "depuncture consumed %d / %d bytes — table broken\n",
                      ic, FIC_IN_BITS);
        return 3;
    }

    // ---- Decode via viterbiSpiral ----
    viterbiSpiral vs (FIC_OUT_BITS);
    uint8_t output [FIC_OUT_BITS];
    vs.deconvolve (viterbiBlock, output);

    // ---- Write 768 bytes (bit-per-byte) to stdout ----
    size_t wrote = std::fwrite (output, 1, FIC_OUT_BITS, stdout);
    if (wrote != (size_t) FIC_OUT_BITS) {
        std::fprintf (stderr, "short write: %zu / %d\n", wrote, FIC_OUT_BITS);
        return 4;
    }
    return 0;
}

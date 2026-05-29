# Slice 14 — Simultaneous Capture Procedure

## Why this is needed

Slice-14 Path A (analytic back-derivation from `k8b_v4.eti`) revealed that
`k8b_v4.iq` and `k8b_v4.eti` were captured **in separate sessions**:

| File | Command | Duration |
|------|---------|----------|
| `k8b_v4.iq`  | `airspy_rx -n 60000000`        | 20 s |
| `k8b_v4.eti` | `eti-cmdline-airspy -t 60 ...` | 60 s |

Two different RF moments. No frame-to-frame correspondence. The back-derived
expected soft bits cannot be compared against `dab fic-iq k8b_v4.iq`'s output
because they describe **different broadcasts**.

The infrastructure (`eti_to_expected_soft.py`, `compare_demap.py`,
`verify_expected_soft` example) is correct — verified by 100 % FIB round-trip
through dab-rs's `FicProtection`. But it needs simultaneously-captured
`*.iq` + `*.eti` to produce a meaningful comparison.

## What to capture

A single simultaneous capture of K8B (or any DAB channel with stable
reception):

```sh
cd /Users/zobithecat/Documents/projects/etc_projects/airspy-mini-dmb

# Output files (use a fresh tag)
TAG=k8b_v4_sync
mkdir -p data/captures/$TAG
IQ=data/captures/$TAG/$TAG.iq
ETI=data/captures/$TAG/$TAG.eti

# Build eti-cmdline-airspy if needed — the diagnostic dump hooks are already
# in-tree (DAB_RS_DIAG_DUMP / DAB_RS_ORACLE_VITERBI_IN / OUT).
( cd eti-stuff/eti-cmdline && cmake --build build -j )

# Run airspy_rx and eti-cmdline-airspy at the same time.
# airspy_rx records the raw I/Q, eti-cmdline-airspy decodes the LIVE
# stream simultaneously and dumps intermediate state via the env hooks.
AIRSPY_LNA=14 AIRSPY_MIXER=15 AIRSPY_VGA=12 \
DAB_RS_DIAG_DUMP=$TAG.ibits.bin \
DAB_RS_ORACLE_VITERBI_IN=$TAG.vitin.bin \
DAB_RS_ORACLE_VITERBI_OUT=$TAG.vitout.bin \
eti-cmdline-airspy -M 1 -C K8B -t 25 -O $ETI &
ETI_PID=$!

# Start I/Q recorder 1 s later so airspy lock has settled.
sleep 1
airspy_rx -f 183.008 -a 3000000 -t 2 \
          -l 14 -m 15 -v 12 -b 0 \
          -n 60000000 -r $IQ

wait $ETI_PID
```

(Adjust paths/flags to match the actual `airspy_rx` invocation that worked
for the previous `k8b_v4.iq` capture. The key constraint is that **both
processes record the exact same airspy stream at the same wall-clock
moment**. If only one device is present, run them sequentially with a hint
to the airspy driver to share the underlying stream — see your existing
notes.)

## What you'll get

| File | Contents | Format |
|------|----------|--------|
| `*.iq`       | raw INT16_IQ samples @ 3 MSPS | `airspy_rx` output |
| `*.eti`      | LIVE ETI(NI) byte-stream      | one ficBlock per 6144-byte frame |
| `*.ibits.bin`| per-symbol post-OFDM soft bits | u32 frame_idx + u32 ofdmSymCnt + 3072 × i16 |
| `*.vitin.bin`| per-ficBlock depunctured Viterbi input | u32 frame_idx + u32 ficno + 3096 × i16 |
| `*.vitout.bin`| per-ficBlock Viterbi output (pre-PRBS) | u32 frame_idx + u32 ficno + 768 × u8 |

## Comparing

After the simultaneous capture is on disk:

```sh
# 1. Back-derive expected soft bits from the LIVE ETI
python3 docs/diag/eti_to_expected_soft.py \
    /path/to/$TAG.eti /tmp/expected_soft.bin

# 2. Run dab-rs on the I/Q with the same dumps eti-stuff produced
DAB_RS_DUMP_DEMAP_OUT=/tmp/dab_demap.bin \
DAB_RS_DUMP_VITERBI_IN=/tmp/dab_vitin.bin \
DAB_RS_DUMP_VITERBI_OUT=/tmp/dab_vitout.bin \
    target/release/dab fic-iq /path/to/$TAG.iq

# 3. Sliding-offset comparison (search ±20 DAB frames in case of
# sync warmup differences):
python3 docs/diag/scan_alignment.py /tmp/expected_soft.bin /tmp/dab_demap.bin \
    --offset-range 20
# Try all 4 phases (which ficBlock the first ETI frame represents):
for p in 0 1 2 3; do
    python3 docs/diag/eti_to_expected_soft.py \
        $TAG.eti /tmp/expected_p${p}.bin --phase $p
    echo "--- phase $p ---"
    python3 docs/diag/scan_alignment.py /tmp/expected_p${p}.bin /tmp/dab_demap.bin \
        --offset-range 30
done

# 4. Per-stage cross-check at the existing oracle dump points
# (DAB_RS_DUMP_VITERBI_IN vs $TAG.vitin.bin; same for VITERBI_OUT).
# These compare LIVE eti-stuff intermediate state directly to dab-rs's.
```

## Expected outcome

If the OFDM chain is correct, the **best offset** at one of the phases should
show **≥ 90 % sign agreement** on the soft bits. The offset is the warmup
gap between airspy lock and the first dab-rs frame.

If sign agreement stays near 50 % across *all* phase × offset combinations,
the OFDM chain really is producing uncorrelated bits — point of divergence
is upstream of demap, and the per-symbol `ibits.bin` cross-check will narrow
it further (compare `DAB_RS_DUMP_VITERBI_IN` to the oracle `$TAG.vitin.bin`).

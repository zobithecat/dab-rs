#!/bin/sh
# docs/diag/airspy-sanity.sh
#
# Byte-equivalence sanity check: does libairspy emit the same INT16_IQ
# stream via the `airspy_rx` file-dumper as it does via the realtime
# callback an SDR application receives?
#
# This is the Fork-1 question from slice 4 of the dab-rs Week 3e
# investigation. If the file and the callback bytes match, the
# oracle's offline-vs-live divergence sits *above* libairspy — most
# likely in eti-stuff's offline plumbing (Fork 4 of slice 4). If they
# differ, libairspy's two output paths are not bit-equivalent and
# that gap is itself the explanation for the oracle behaving
# differently on saved captures.
#
# Hardware required: Airspy Mini, RF source (any DAB ensemble),
# libairspy installed (homebrew: `brew install airspy`).
#
# We capture two short streams back-to-back from the *same* RF input,
# at the same gain / frequency / rate. Because radio conditions drift
# millisecond-by-millisecond, exact byte equivalence isn't expected
# across captures — but the *statistical distribution* of samples
# should be identical, and the per-sample range should match
# bit-for-bit if libairspy's two paths share the same firmware-side
# DSP.

set -eu

OUT_DIR=${OUT_DIR:-/tmp/airspy_sanity}
FREQ_HZ=${FREQ_HZ:-183008000}   # K8B; override for your local ensemble
SAMPLE_RATE=${SAMPLE_RATE:-3000000}
NUM_SAMPLES=${NUM_SAMPLES:-3000000}   # 1 second
LNA=${LNA:-14}
MIXER=${MIXER:-15}
VGA=${VGA:-12}

mkdir -p "$OUT_DIR"
cd "$OUT_DIR"

echo "=== capture 1 (airspy_rx file dump) ==="
airspy_rx \
    -f $(awk "BEGIN {printf \"%.6f\", $FREQ_HZ / 1e6}") \
    -a "$SAMPLE_RATE" \
    -t 2 \
    -l "$LNA" -m "$MIXER" -v "$VGA" -b 0 \
    -n "$NUM_SAMPLES" \
    -r "$OUT_DIR/file.iq"

sleep 1  # give the radio half a beat to settle

echo "=== capture 2 (airspy_rx file dump, identical config) ==="
airspy_rx \
    -f $(awk "BEGIN {printf \"%.6f\", $FREQ_HZ / 1e6}") \
    -a "$SAMPLE_RATE" \
    -t 2 \
    -l "$LNA" -m "$MIXER" -v "$VGA" -b 0 \
    -n "$NUM_SAMPLES" \
    -r "$OUT_DIR/file2.iq"

# Note: we capture two file streams back-to-back rather than file vs
# callback because libairspy doesn't ship a "dump callback bytes"
# tool. To truly test file-vs-callback equivalence you would need to
# (a) instrument airspy-handler.cpp to dump its sbuf[] input to a
# file in `data_available()` before any /2048 scaling, then (b) run
# both `airspy_rx` and a patched eti-cmdline-airspy on the same
# radio. The two file captures here verify that *repeated* airspy_rx
# runs give statistically-stable output — a pre-requisite for any
# deeper comparison.

echo
echo "=== byte-level diff (expected: differs frame-to-frame, distrib equal) ==="
if cmp -s "$OUT_DIR/file.iq" "$OUT_DIR/file2.iq"; then
    echo "files are bit-identical (extraordinary — would imply"
    echo "deterministic firmware output, not normal for RF)"
else
    echo "files differ at the bit level (expected for live RF)"
fi

echo
echo "=== sample-amplitude distribution check ==="
python3 - <<'PY'
import os, struct, math, statistics, sys
out_dir = os.environ.get('OUT_DIR', '/tmp/airspy_sanity')
def stats(path):
    sz = os.path.getsize(path)
    n = min(sz // 4, 200_000)
    with open(path, 'rb') as f:
        raw = f.read(4 * n)
    vals = struct.unpack(f'<{2*n}h', raw)
    abs_vals = sorted(abs(v) for v in vals)
    return {
        'samples':  n,
        'mean_abs': sum(abs_vals) / len(abs_vals),
        'rms':      math.sqrt(sum(v*v for v in vals) / len(vals)),
        'p50':      abs_vals[len(abs_vals)//2],
        'p99':      abs_vals[int(len(abs_vals)*0.99)],
        'max':      abs_vals[-1],
    }

a = stats(os.path.join(out_dir, 'file.iq'))
b = stats(os.path.join(out_dir, 'file2.iq'))
keys = ['samples', 'mean_abs', 'rms', 'p50', 'p99', 'max']
print(f"  {'metric':<10}  {'capture 1':>12}  {'capture 2':>12}  {'delta':>10}")
for k in keys:
    delta = b[k] - a[k]
    print(f"  {k:<10}  {a[k]:>12.1f}  {b[k]:>12.1f}  {delta:>+10.1f}")

# Distribution should agree within ~5% if libairspy is stable.
mean_delta = abs(b['mean_abs'] - a['mean_abs']) / a['mean_abs'] if a['mean_abs'] > 0 else 0
rms_delta  = abs(b['rms']      - a['rms'])      / a['rms']      if a['rms']      > 0 else 0
print(f"\n  mean_abs %-delta: {mean_delta*100:.2f}%")
print(f"  rms      %-delta: {rms_delta*100:.2f}%")
if mean_delta < 0.05 and rms_delta < 0.05:
    print("  → distributions agree within 5% (libairspy is stable)")
else:
    print("  → distributions diverge > 5% (configuration mismatch or RF transient?)")
PY

echo
echo "=== next: cross-check against eti-cmdline-airspy ==="
echo "  1. apply docs/diag/eti-stuff-ibits-dump.patch to eti-stuff"
echo "  2. patch airspy-handler.cpp::data_available to fwrite() sbuf"
echo "     (the raw int16 pairs libairspy hands the callback)"
echo "  3. build eti-cmdline-airspy and run it for NUM_SAMPLES samples"
echo "  4. diff the callback-dump against $OUT_DIR/file.iq"
echo
echo "If those bytes match, libairspy's two paths are equivalent and"
echo "the offline-vs-live divergence we observed in slice 3+4 is"
echo "produced by eti-stuff offline plumbing — Fork 4 (bisect) of the"
echo "slice-4 follow-up list."

# dab-rs

**Memory-safe DAB Mode I OFDM demodulator and T-DMB receiver in pure Rust.**

`dab-rs` is a reference-validated, from-scratch Rust implementation of a
[DAB](https://en.wikipedia.org/wiki/Digital_Audio_Broadcasting) Mode I
receiver, targeting **byte-identical output** with the C++
[`eti-stuff`](https://github.com/JvanKatwijk/eti-stuff) decoder. The headline
contribution is a type-safe OFDM demodulator (the first in the Rust ecosystem)
that is deployable to the browser via WebAssembly.

It is the successor to the Python receiver
[`airspy-mini-dmb`](https://github.com/zobithecat/airspy-mini-dmb), and is
designed both as production-quality SDR software and as an educational kit for
undergraduate/graduate communications, broadcasting, and signal-processing
coursework.

## Status

Early development — building bottom-up against a validated Korean T-DMB
capture (channel **K8B**, YTN DMB, 183.008 MHz, EId `0xE040`).

| Crate            | Purpose                                                          | Status      |
| ---------------- | ---------------------------------------------------------------- | ----------- |
| `dab-fec`        | T-DMB outer FEC: sync-aligned Forney deinterleaver + RS(204,188) | **Week 1**  |
| `dab-eti`        | ETI(NI, G.703) frame parser (ETSI EN 300 799)                    | **Week 1**  |
| `dab-msc`        | MSC sub-channel byte extraction                                  | **Week 1**  |
| `dab-fic`        | FIC: FIB CRC, FIG 0/x & 1/x → Ensemble                           | **done**    |
| `dab-viterbi`    | Rate-1/4 punctured convolutional (Viterbi) inner decoder + EEP   | **Week 2**  |
| `dab-descramble` | Energy-dispersal PRBS (x⁹ + x⁵ + 1)                              | **Week 2**  |
| `dab-ofdm`       | **Mode I OFDM demodulator (main contribution)**                  | in progress |
| `dab-iq`         | Airspy / RTL-SDR I/Q input (libairspy FFI)                       | planned     |
| `dab-cli`        | Binary front-end (`dab`)                                         | **Week 1**  |

## Roadmap

Built bottom-up: every stage is validated against a reference before the next
is layered on top. The validation oracle is the C++
[`eti-stuff`](https://github.com/JvanKatwijk/eti-stuff) decoder (byte-identical
target); Week 1 additionally reproduces the Python
[`airspy-mini-dmb`](https://github.com/zobithecat/airspy-mini-dmb) receiver.

**Done**

- ✅ **Week 1 — Outer FEC & ETI plumbing.** `dab-eti` (ETI(NI) frame parser),
  `dab-msc` (sub-channel extraction), `dab-fec` (sync-aligned Forney
  deinterleaver + RS(204,188)), `dab-cli` (`dab fec`). Validated byte-identical
  against the Python receiver — 87.3 % RS on the K8B capture, sync lock at
  phase 160.
- ✅ **Week 2 — Inner FEC.** `dab-viterbi` (rate-1/4 K=7 Viterbi + EEP
  depuncturing) and `dab-descramble` (energy-dispersal PRBS), ported from
  `eti-stuff`. Covered by self-contained round-trip tests; the byte-identical
  cross-check against `eti-stuff` is deferred until OFDM soft bits exist.
- ✅ **`dab-fic` — FIC decode.** FIB CRC-16; FIG 0/0, 0/1, 0/2 and 1/0,1/1,1/5
  dispatch → Ensemble. Closes Stage A end-to-end: byte-identical against the
  Python reference on the K8B capture — ensemble label *"YTN DMB"* (EId 0xE040),
  4 sub-channels, 5 services, fib_ok 15042/20064. Try `dab fic <capture.eti>`.

**Next**

- 🔨 **Weeks 3-5 — `dab-ofdm` (the core contribution).** Mode I demodulator,
  built and validated one stage at a time:
  - ✅ **Static foundation** — Mode I parameters, the `get_phi` phase table,
    the Phase-Reference Symbol (PRS), the frequency-interleaver permutation,
    and an FFT wrapper. Input-independent and deterministic, so verified
    directly against `eti-stuff` (e.g. the interleaver is proven a bijection
    onto {−768..−1}∪{1..768}).
  - ⬜ The 7-stage sync/demod chain (next):
    1. Resample 3 → 2.048 MSPS (polyphase)
    2. Coarse time sync (null-symbol envelope dip)
    3. Fine time + fractional frequency offset (cyclic-prefix autocorrelation)
    4. Frequency correction (NCO)
    5. 2048-point FFT (`rustfft`)
    6. Channel equalisation against the phase-reference symbol
    7. π/4-DQPSK demap → soft bits (`+ ⇒ bit 1`; see *Discovered subtleties*)

  Reaching step 7 unblocks the deferred full-chain validation: the same raw
  I/Q into both `eti-stuff` and `dab-rs`, compared per OFDM symbol and
  end-to-end on the K8B capture. **This needs a raw I/Q capture, which does
  not yet exist** (see the validation note below).
- ⬜ **`dab-iq` — SDR input.** Airspy / RTL-SDR I/Q via `libairspy` (bindgen
  FFI); see the Airspy 12-bit note in *Discovered subtleties*.

**Later**

- ⬜ **Week 6 — Performance margin.** SNR-threshold tuning vs `eti-stuff`,
  lock-time optimisation, `criterion` benchmarks (throughput, latency, memory).
- ⬜ **Week 7 — WebAssembly.** `wasm-pack` build; decode the K8B capture (or a
  smaller sample) in-browser; live demo page.
- ⬜ **Week 8 — Paper.** Target SoftwareX / JOSS / IEEE BMSB —
  *"dab-ofdm-rs: A Memory-Safe Software-Defined DAB Mode I Demodulator in Rust
  with WebAssembly Deployment."*

### Validation status

| Stage | Scope                              | Oracle                              | Status                                            |
| ----- | ---------------------------------- | ----------------------------------- | ------------------------------------------------- |
| A     | Outer FEC + ETI/MSC + FIC          | Python `airspy-mini-dmb` + `.eti`   | ✅ byte-identical (87.3 % RS; ensemble "YTN DMB") |
| B     | Inner FEC (Viterbi + descramble)   | `eti-stuff` intermediate dump       | ⬜ deferred — needs raw I/Q or OFDM soft bits     |
| C     | OFDM core                          | `eti-stuff` per-symbol dump         | ⬜ blocked on `dab-ofdm` + raw I/Q                |

> Stage B/C cross-validation needs a raw I/Q capture (`airspy_rx -r out.iq`)
> and a built/instrumented `eti-stuff`. The committed `.eti` files are
> *downstream* of these stages, so they cannot serve as their input.

## DAB Mode I parameters

| Parameter        | Value                                            |
| ---------------- | ------------------------------------------------ |
| Internal rate    | 2.048 MSPS                                        |
| Useful symbol    | 2048 samples (1 ms)                               |
| Guard interval   | 504 samples                                       |
| Sub-carriers     | 1536 (−768..+768, DC null)                        |
| Modulation       | π/4-DQPSK (differential)                          |
| Inner FEC        | rate-1/4 conv, K=7, polys (0o133,0o171,0o145,0o133)|
| Outer FEC (T-DMB)| RS(204,188) DVB params + Forney TI (N=12, M=17)   |

## Discovered subtleties

Implementation notes that bit us during the port and that the next
contributor (or the next paper reviewer) deserves to know up front.

- **Viterbi soft-bit polarity convention** *(Week 2, `dab-viterbi`).*
  A comment in `eti-stuff`'s `viterbi-handler.cpp` suggests that a
  soft-bit value of `+255` decodes to bit `0`. Tracing the trellis
  metric updates shows the opposite: `+255` corresponds to bit `1`.
  The decoder is a verbatim port of the C++, so it inherits the
  *actual* behaviour, not the commented one. The OFDM demapper in
  Week 3 (`dab-ofdm`, π/4-DQPSK → soft bits) must emit soft values
  under this same `+ ⇒ 1` convention, otherwise the inner decoder
  silently produces inverted bytes.

- **Forney deinterleaver sync alignment** *(Week 1, `dab-fec`).*
  The deinterleaver must be aligned to the TS sync byte (`0x47`)
  *before* it processes any data, not after. The naive
  ``deinterleave then search for 0x47'' ordering happens to preserve
  the 204-byte sync cadence by coincidence (since `204 = N · M`), so
  the downstream sync scan looks fine but every byte is permuted to
  the wrong slot. On the reference K8B capture this drops RS from
  87.3 % to 0 %. See `crates/dab-fec/src/outer.rs` and the companion
  [`airspy-mini-dmb`](https://github.com/zobithecat/airspy-mini-dmb)
  paper for the detailed before/after.

- **Airspy 12-bit packed sample format** *(planned, `dab-iq`).*
  The Airspy Mini's native USB output packs 12-bit ADC samples
  across pairs of 16-bit words rather than emitting them as
  zero-extended signed-16 — i.e. raw USB bytes are not
  `Complex<i16>` even though that is what most receivers expect.
  `libairspy` unpacks transparently and exposes `Complex<i16>` or
  `Complex<f32>` to callers, which is why `dab-iq-airspy` (Week 4+)
  will go through `libairspy` via bindgen rather than talking to
  `libusb` directly. A pure-Rust libusb path would have to
  re-derive the unpacking, the firmware-level gain-stage command
  protocol (LNA / Mixer / VGA), and the bias-tee toggle — solvable
  but a substantial extra surface for marginal benefit.

## Build & test

Requires Rust stable (1.83+). On macOS:

```sh
brew install fftw libsndfile libsamplerate pkg-config libusb airspy
cargo build --workspace
cargo test  --workspace
```

### Validating against the reference capture

The outer-FEC integration test reproduces the Python receiver's result on the
`k8b_100pct.eti` capture (SubCh 1): **87.3 % RS success** (22 652 / 25 953
packets), with the 0x47 sync byte locking at **phase 160**.

The 30 MB capture is not committed to this repo. Point the test at a local
copy:

```sh
export DAB_RS_K8B_ETI=/path/to/k8b_100pct.eti
cargo test -p dab-fec --test golden -- --include-ignored
```

The expected numbers are pinned in [`tests/golden/`](tests/golden/).

You can also run the pipeline directly:

```sh
cargo run -p dab-cli -- fec /path/to/k8b_100pct.eti --subch 1
```

## License

MIT (code). Reference captures are linked, not redistributed, under CC-BY-4.0.

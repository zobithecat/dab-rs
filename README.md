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
| `dab-fic`        | FIC: FIB CRC, FIG 0/x & 1/x → Ensemble                           | planned     |
| `dab-viterbi`    | Rate-1/4 punctured convolutional (Viterbi) inner decoder + EEP   | **Week 2**  |
| `dab-descramble` | Energy-dispersal PRBS (x⁹ + x⁵ + 1)                              | **Week 2**  |
| `dab-ofdm`       | **Mode I OFDM demodulator (main contribution)**                  | planned     |
| `dab-iq`         | Airspy / RTL-SDR I/Q input (libairspy FFI)                       | planned     |
| `dab-cli`        | Binary front-end (`dab`)                                         | **Week 1**  |

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

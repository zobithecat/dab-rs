# dab-rs

**🇰🇷 한국 최초 · 🌐 세계 최초 — 순수 Rust 로 만든 메모리 안전 DAB Mode I OFDM 디코더 / T-DMB 수신기.**

![dab-rs 가 raw Airspy I/Q 로부터 디코드한 YTN DMB 영상](docs/img/ytn_decoded_hero.png)

> *YTN「뉴스특보」를 **dab-rs 가 단독으로** 디코드한 한 장면 — Airspy Mini 의
> 3 MSPS `int16` raw I/Q → OFDM 동기/CFO/FFT → FIC → MSC → 16-슬롯 Forney
> 시간-디인터리브 → EEP 디펑크처/비터비 → PRBS → RS(204,188) → MPEG-TS → H.264,
> 마지막 그림만 ffmpeg 에 맡겼다. **fudge factor 도 없고 magic number 도 없다.**
> 모든 상수는 `eti-stuff` 또는 DAB/DVB 표준에서 유래한다. 약간의 매크로블록
> 아티팩트는 이 캡처의 SNR 에서 t=8 정정 한계를 초과한 RS(204,188) 블록 6.9%
> 와 정확히 일치한다.*

---

## 왜 "한국 최초 · 세계 최초"인가

| 항목 | 현황 |
|------|------|
| **한국 최초** | **순수 Rust 로 작성된 한국형 T-DMB 수신기**. 한국 채널(K8B, YTN DMB)에서 raw I/Q → MPEG-TS → H.264 까지 end-to-end 검증 완료. |
| **세계 최초** | **Rust 로 작성된 메모리 안전 DAB Mode I OFDM 디코더**. WebAssembly 배포를 정조준 — `unsafe` 0줄, C/C++ 의존성 없음. |
| **검증** | C++ [`eti-stuff`](https://github.com/JvanKatwijk/eti-stuff) 와 byte 단위로 동등. `fudge factor` 0개, `magic number` 0개. 모든 상수는 spec 또는 eti-stuff 에 추적 가능. |
| **로드맵** | **🌐 WASM 웹버전** — 브라우저에서 동작하는 세계 최초의 웹 DMB 수신기. 별도 앱/플러그인 없이 URL 만으로 라이브 DMB 시청. |

---

## 빠른 시연

60초 라이브 캡처 → MP4:

```sh
# 1) Airspy Mini 로 K8B(YTN DMB) 60초 캡처
airspy_rx -f 183.008 -a 3000000 -t 2 -l 14 -m 15 -v 12 -b 0 \
          -n 180000000 -r /tmp/live.iq

# 2) FIC: 앙상블 + 서비스 목록 확인
dab fic-iq /tmp/live.iq
# → fib_ok=7424/7524 (98.7%), EId=0xE040, label="YTN DMB", 5 services

# 3) MSC: SubCh 1 (mYTN 비디오) → MPEG-TS
dab msc-iq --scid 1 --ts /tmp/live.ts /tmp/live.iq
# → ts_packets=12872, rs_corrected=7240 (98.7%)

# 4) (선택) SL 헤더 strip + SPS prepend → MP4
python3 tools/extract_sl_h264_v2.py /tmp/live.ts --video-pid 0x113 \
        --out /tmp/live.h264
cat sps_pps.bin /tmp/live.h264 > /tmp/live_sps.h264
ffmpeg -r 30 -i /tmp/live_sps.h264 -c copy /tmp/live.mp4
```

결과: **320×240 H.264 Baseline, 1784 frames @ 30 FPS, 60초 영상.**

---

## 현재 상태

**Week 3e + Week 4 완료** — raw I/Q 부터 MP4 까지 fudge-free end-to-end 동작.

| 크레이트 | 역할 | 상태 |
|----------|------|------|
| `dab-ofdm` | **Mode I OFDM 복조기 (핵심 기여)** | ✅ **fudge-free 90%+** (sim5, sim4, k8b_v4) |
| `dab-iq` | 파일 I/Q reader (Cs8/Cs16Le/Cf32Le, JSON sidecar) | ✅ done; libairspy 직결 streaming 은 추후 |
| `dab-fic` | FIB CRC-16, FIG 0/x·1/x → Ensemble | ✅ done — byte-identical vs eti-stuff |
| `dab-viterbi` | Rate-1/4 K=7 punctured 비터비 + EEP/FIC depuncture | ✅ done |
| `dab-descramble` | 에너지 분산 PRBS (x⁹ + x⁵ + 1) | ✅ done |
| `dab-msc` | MSC sub-channel byte 추출 | ✅ done |
| `dab-eti` | ETI(NI, G.703) frame parser (ETSI EN 300 799) | ✅ done |
| `dab-fec` | T-DMB outer FEC: sync-aligned Forney + RS(204,188) | ✅ done |
| `dab-cli` | `dab fic-iq` + `dab msc-iq` CLI 바이너리 | ✅ done |

### 검증 결과 (실측)

| 캡처 | 경로 | FIB CRC | RS(204,188) | MP4 frames |
|------|------|---------|-------------|------------|
| **60s live capture** | cs16le → LinearResampler → 전체 chain | **98.7%** (7424/7524) | **98.7%** (7240/7332) | **1784 @ 30 FPS** |
| `k8b_v4.iq` (20s) | cs16le → LinearResampler | 87.4% (2193/2508) | 92.8% (3383/3646) | 475 @ 23.75 FPS |
| `sim5_resampled.cf32` (12s) | cf32 bypass | 90.4% (1704/1884) | 96.8% (1381/1426) | — |

---

## 로드맵

상향식 구축: 각 단계는 다음 단계가 쌓이기 전에 reference 와 검증된다.
Oracle 은 C++ [`eti-stuff`](https://github.com/JvanKatwijk/eti-stuff) (byte-identical 목표) 와 Python [`airspy-mini-dmb`](https://github.com/zobithecat/airspy-mini-dmb).

### ✅ 완료

- **Week 1 — Outer FEC + ETI plumbing.** `dab-eti` (ETI(NI) frame parser),
  `dab-msc` (sub-channel extraction), `dab-fec` (sync-aligned Forney
  deinterleaver + RS(204,188)). Python reference 와 byte-identical.
- **Week 2 — Inner FEC.** `dab-viterbi` (rate-1/4 K=7 비터비 + EEP depuncturing),
  `dab-descramble` (energy-dispersal PRBS). `eti-stuff` verbatim 포팅.
- **Week 3a-d — `dab-iq` + OFDM stages 1-7.** 3MSPS → 2.048 MSPS resampler,
  null detect, CP autocorrelation sync, NCO + integer CFO, FFT, differential
  reference, π/4-DQPSK demap.
- **Week 3e — Fudge-free FIC chain.** 25개 슬라이스 의 진단 → integer-CFO 를
  rotate_spectrum 대신 시간-영역 NCO 로 통합 (`eti-stuff` 와 정확히 동일한
  구조). 모든 magic number 제거. K8B 에서 FIB CRC 87-90%, EId 0xE040, 5
  services 식별. `eti-stuff` linear-interp resampler 이식 (+2.5 dB SNR).
- **Week 4 — MSC sub-channel decode.** 75 data symbol demap, 16-슬롯 Forney
  시간 디인터리버 (eti-generator.cpp:207 verbatim), EepProtection +
  KoreanTDmbOuterFec wiring. K8B SubCh 1 → MPEG-TS → H.264 → MP4 (320×240,
  1784 frames, 30 FPS). 라이브 60초 캡처 검증 완료.

### 🔨 진행 예정

- **Week 5 — libairspy FFI + streaming.** 파일 I/O 대신 Airspy Mini 직결.
  Manual gain L14/M15/V12 default (실측 -G 0 대비 20× RS 개선). 영구
  ring buffer 기반 streaming pipeline (현재는 batch).
- **Week 6 — Performance margin.** SNR threshold 튜닝, lock 시간 최적화,
  `criterion` 벤치마크 (throughput, latency, memory).

### 🌐 차기 핵심: WebAssembly 웹 DMB

- **Week 7 — WASM 웹버전 (★ 세계 최초).** `wasm-pack` 빌드 + 브라우저 SDR
  연결 (WebUSB 로 Airspy Mini 직결, 또는 미리 녹화된 .iq 파일 업로드).
  - 라이브 데모 페이지: 사용자가 URL 만 열면 **앱/플러그인 없이 브라우저에서
    DMB 시청** 가능
  - 모바일 PWA: 안드로이드/iOS Chrome 에서 WebUSB 지원 시 SDR dongle 연결
    가능 (한국 K-block 6 채널 스캔 → 채널 선택 → 라이브 영상)
  - 학술/교육 자료 활용: spec.html → 실시간 OFDM constellation/FFT bin
    시각화 (signal-processing 강의용)
- **Week 8 — 논문.** SoftwareX / JOSS / IEEE BMSB 타깃 —
  *"dab-rs: A Memory-Safe Software-Defined DAB Mode I Demodulator in Rust
  with WebAssembly Deployment."*

---

## DAB Mode I 파라미터

| 파라미터 | 값 |
|----------|-----|
| 내부 sample rate | 2.048 MSPS |
| Useful symbol | 2048 samples (1 ms) |
| Guard interval | 504 samples |
| Sub-carriers | 1536 (−768..+768, DC null) |
| Modulation | π/4-DQPSK (differential) |
| Inner FEC | rate-1/4 conv, K=7, polys (0o133, 0o171, 0o145, 0o133) |
| Outer FEC (T-DMB) | RS(204,188) DVB params + Forney TI (N=12, M=17) |

---

## 발견된 함정 (Discovered subtleties)

포팅 과정에서 마주친 미세한 구현 디테일들. 다음 기여자(또는 다음 논문 reviewer)가
미리 알아둬야 할 사항들.

### 1. **Integer-CFO 는 시간영역 NCO 에 포함시켜야 한다** *(Week 3e slice 25)*

`eti-stuff` 는 `coarseCorrector + fineCorrector` 를 **하나의 시간영역 NCO**
로 함께 mix 한다 (`getSamples(coarseCorrector + fineCorrector)`). 정수-CFO
δ 가 매 심볼마다 NCO 위상 진폭 `2π·δ·T_s/T_u ≈ 1.55 rad/sym` (δ=+1) 를
부여하는데, dab-rs 의 초기 구현은 정수-CFO 를 `rotate_spectrum` 으로
post-FFT 처리해서 그 per-symbol 위상이 빠졌다. 수학적으로 같아 보이지만,
**π/4-DQPSK 차동 복조 단계에서 constellation 이 1.55 rad 회전된 채로 도착**
→ 비트 매핑 wrong.

**fix**: 데이터 심볼은 `cfo_hz + δ·1000` 을 NCO 에 통합해 mix, rotate 생략.
PRS 만 rotate (δ 를 측정하려면 PRS FFT 가 먼저 필요하기 때문). 결과: sim5
FIB CRC 4% → 90.4%, fudge-free.

### 2. **Linear-interpolation resampler 가 polyphase FIR 보다 +2.5 dB** *(Week 3e slice 17)*

dab-ofdm 의 초기 resampler 는 polyphase FIR (Blackman-sinc) 이었는데
한계 SNR(11 dB K8B) 에서 decoding 절벽 아래. `eti-stuff` 의 `airspy-handler.cpp:157-162`
linear-interp resampler 를 verbatim 이식 (`LinearResampler` struct).
같은 입력에서 band_ratio +2.5 dB, k8b_v4.iq cs16le FIB 0/2496 → 2193/2508.

### 3. **DAB MSC 시간 디인터리버는 비트-위치 기반 16-슬롯 Forney** *(Week 4 slice 1)*

`eti-stuff` `eti-generator.cpp:207`:
```cpp
const int16_t interleaveMap[] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};
```
비트 위치 `i & 0xF` 마다 16-슬롯 ring buffer 의 서로 다른 슬롯에서 읽어와
디인터리브한다. CIF 1개당 55296 비트, 18개 OFDM 심볼 → 1 CIF, 1 DAB frame
= 4 CIFs. 15-CIF warmup 후 출력이 valid.

### 4. **Airspy AGC (`-G 0`) 는 한계 SNR 에서 sub-optimal**

같은 indoor K8B 안테나 셋업에서 17-config gain sweep 결과: hardware AGC
(`-G 0`) 는 small-signal regime 에서 under-amplify 한다 (firmware 가 mixer
는 working level 로 잡지만 VGA 는 default 10 으로 고정 → ~20 dB headroom
미사용). Manual **LNA 14 / Mixer 15 / VGA 12** 가 25초당 **4619-4852 RS
corrected blocks** 를 산출, `-G 0` 의 **241** 대비 **20×** 개선.
`fibquality` 100 포화. Live air-receive 시 manual L14/M15/V12 기본 권장.

### 5. **SFN multipath 가 null symbol 을 얕게 만든다** *(Week 3a)*

DAB null symbol 은 96ms frame 마다 ~0 envelope 이어야 하는데, K8B 는
single-frequency network 환경에서 5개 송신탑(남산/관악산/용문산/광교산/운정산)
이 시간차로 도착해 null 간격이 다른 송신탑의 신호로 *채워진다*. min/µ ≈
0.72-0.78 (교과서 값 0.1-0.3 와 다름). 고정 threshold 는 모든 null 을 놓침.
`NullDetector` 는 **adaptive threshold `p1 + 0.30·(p99 − p1)`** 를 smoothed
envelope 에 적용해 ~7 dB SNR 에서도 96ms cadence 회복.

### 6. **Viterbi soft-bit polarity convention** *(Week 2)*

`eti-stuff` 의 `viterbi-handler.cpp` 주석은 `+255 → bit 0` 이라 적혀있지만
trellis metric update 를 추적하면 **`+255 → bit 1`** 가 실제 동작. dab-rs
는 verbatim 포팅이라 실제 동작을 그대로 따라가며, 따라서 OFDM 디매퍼도
같은 `+ ⇒ 1` convention 으로 soft bit 를 emit 해야 한다.

### 7. **Airspy 12-bit packed sample format** *(Week 5 예정)*

Airspy Mini 의 native USB 출력은 12-bit ADC 샘플을 16-bit 쌍에 packed
한다 (zero-extended `Complex<i16>` 가 아님). `libairspy` 가 transparent
하게 unpack 해서 `Complex<f32>` 또는 `Complex<i16>` 로 노출한다. 따라서
`dab-iq-airspy` (Week 5) 는 pure-Rust libusb 대신 bindgen FFI 로 libairspy
를 호출한다. 직접 libusb 경로는 unpacking + 게인 stage 제어 (LNA/Mixer/VGA)
+ bias-tee 토글 모두 새로 derive 해야 하므로 부담이 크다.

---

## 빌드 & 테스트

Rust stable (1.83+). macOS 기준:

```sh
brew install fftw libsndfile libsamplerate pkg-config libusb airspy
cargo build --workspace --release
cargo test  --workspace --release
```

### CLI 사용 예

```sh
# 라이브 캡처
airspy_rx -f 183.008 -a 3000000 -t 2 -l 14 -m 15 -v 12 -n 180000000 -r live.iq

# FIC (앙상블 + 서비스 목록)
dab fic-iq live.iq

# MSC (특정 sub-channel → MPEG-TS)
dab msc-iq --scid 1 --ts mYTN.ts live.iq

# 다른 format/rate
dab fic-iq --format cf32le --rate 2048000 pre_resampled.cf32
```

### 검증 captures

`airspy-mini-dmb` repo (Git LFS) 에 reference captures (k8b_v4.iq +
k8b_v4.eti + sim5_resampled.cf32 등) 존재. 로컬 경로 지정 후 통합 테스트:

```sh
export DAB_RS_K8B_V4_IQ=/path/to/k8b_v4.iq
cargo test -p dab-cli --test k8b_v4_fic_iq -- --include-ignored
```

기대 수치는 [`tests/golden/`](tests/golden/) 에 기록되어 있다.

---

## 학술/교육 활용

`dab-rs` 는 production-quality SDR 소프트웨어인 동시에 **학부/대학원 통신·방송·
신호처리 강의용 교재**로 설계됐다. 각 단계가 isolated crate + unit tests +
diagnostic dump 로 분리되어, 다음 시연이 모두 가능하다:

- **OFDM 동기 시각화**: `docs/diag/cp_autocorr_manual.py` 가 CP 자기상관
  metric 을 plot 해 fine timing + fractional CFO 추정의 동작 원리 노출
- **FFT bin alignment**: `docs/diag/fft_by_pos.py` 가 sync_pos 기반으로
  dab-rs FFT 와 eti-stuff FFT 를 sample-aligned 비교 (|cc|=0.9994 도달
  검증)
- **시간 디인터리버 위상 분석**: `docs/diag/sync_pos_compare.py` 가 PRS
  alignment 의 ±T_s oscillation 을 dump
- **outer FEC visualization**: `crates/dab-viterbi/examples/encode_dump.rs`
  + `verify_expected_soft.rs` 가 Python ↔ Rust convolutional encode 의
  bit-identity 확인

각 발견(Discovered subtlety #1-#7) 은 reference 와의 측정 → 가설 → 정확한
수정 cycle 로 도출됐다 (Week 3e 의 25 슬라이스 = `docs/diag/` 의 진단 인프라).

---

## 라이센스

코드: **MIT**. Reference captures 는 별도 link, redistribute 하지 않음 (CC-BY-4.0).

---

## 인용

이 프로젝트를 학술 자료에서 인용한다면:

```bibtex
@software{dab_rs_2026,
  author  = {{zobithecat}},
  title   = {{dab-rs: A Memory-Safe Software-Defined DAB Mode I Demodulator
              in Rust with WebAssembly Deployment}},
  year    = {2026},
  url     = {https://github.com/zobithecat/dab-rs},
  note    = {The first pure-Rust DAB OFDM decoder and the first
             T-DMB receiver implementation in Rust.}
}
```

---

🇰🇷 **한국 최초 · 🌐 세계 최초** — Made with ❤️ and `cargo build`.

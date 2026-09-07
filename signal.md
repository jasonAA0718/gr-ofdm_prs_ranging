# OFDM PRS-Like Signal Mathematics

This document describes the current OFDM/PRS-like ranging burst implemented in
`gr-ofdm_prs_ranging`. It is a custom deterministic ranging waveform, not a
standards-complete LTE/NR PRS signal.

## Burst Layout

The transmitted complex baseband burst is

```text
[zero guard]
[repeated QPSK acquisition preamble]
[coarse ZC sync]
[MC-DS OFDM PRS/data symbols]
[tail guard]
```

Using the common defaults:

```text
zero_guard_len    = 1000 samples
preamble_len      = 256 samples
preamble_repeats  = 8
coarse_sync_len   = 839 samples
fft_len           = 512
cp_len            = 64
active_bins       = 512
prs_symbols       = 127
Gold code ID      = 2
```

The OFDM PRS-like section length is

```math
N_{\text{PRS}} = N_{\text{sym}}(N_{\text{FFT}} + N_{\text{CP}})
```

For the default values:

```math
N_{\text{PRS}} = 127(512 + 64) = 73152
```

## Repeated QPSK Acquisition Preamble

The short acquisition preamble is one deterministic QPSK period repeated
`preamble_repeats` times. Let the period length be `L_p`. The generated period is

```math
p[n] \in \left\{\frac{1+j}{\sqrt{2}}, \frac{1-j}{\sqrt{2}},
\frac{-1+j}{\sqrt{2}}, \frac{-1-j}{\sqrt{2}}\right\},
\quad 0 \le n < L_p
```

The transmitted repeated preamble is

```math
x_p[n + rL_p] = p[n],
\quad 0 \le n < L_p,\quad 0 \le r < R
```

where `R = preamble_repeats`.

The detector uses repeated-period self-correlation:

```math
C_p[m] =
\sum_{n=0}^{L_p(R-1)-1} y[m+n]^* y[m+n+L_p]
```

with normalized metric

```math
M_p[m] =
\frac{|C_p[m]|}
{\sqrt{
\left(\sum_n |y[m+n]|^2\right)
\left(\sum_n |y[m+n+L_p]|^2\right)}}
```

This is a cheap first-stage acquisition gate. It does not need to know the exact
QPSK values, only that adjacent preamble periods are repeated.

## Coarse ZC Sync

After the repeated preamble, the burst carries a Zadoff-Chu-like coarse sync
sequence. For length `N_z` and root `u`, the current implementation generates

```math
z_u[n] = \exp\left(-j\pi u\frac{n(n+1)}{N_z}\right),
\quad 0 \le n < N_z
```

The receiver computes the normalized coarse correlation

```math
C_z[m] = \sum_{n=0}^{N_z-1} y[m+n] z_u[n]^*
```

```math
M_z[m] =
\frac{|C_z[m]|}
{\sqrt{N_z \sum_{n=0}^{N_z-1}|y[m+n]|^2}}
```

The repeated preamble first gates a proposed frame location using the preamble
threshold. The detector then evaluates `M_z` at offsets `-2...+2` around the
predicted ZC boundary, selects the strongest valid offset, and accepts it only
when it passes the separate ZC threshold. The selected offset corrects both the
frame start and coarse-sync index. A threshold-passing edge maximum recenters
the five-point window once before acceptance. Configurable roots continue to
separate channels/responders.

Example channel plan:

```text
Initiator POLL TX:       coarse_zc_root = 25
Responder RX detector:   coarse_zc_root = 25
Responder RESPONSE TX:   coarse_zc_root = 29
Initiator RX detector:   coarse_zc_root = 29, channel_id = 1
```

## MC-DS OFDM Payload and PRS

The current prototype carries the existing 120-bit SS-TWR packet inside the
OFDM symbols. There is no standalone time-domain payload. Native FFT bins use
a fixed three-pilot/one-data comb:

```math
\mathcal{D}=\{k: k\bmod 4=3\},\qquad |\mathcal{D}|=128
```

The other 384 bins form the pilot set `P`. Data-bin indices `q=0...119`
contain the serialized packet bits and `q=120...127` contain zero padding.
Serialization remains LSB-first and the CRC16 initial value and polynomial
remain `0xffff` and `0x1021`.

| Payload field | Bits | POLL | RESPONSE |
|---|---:|---|---|
| `packet_type` | 8 | `1` | `2` |
| `poll_frame_id` | 32 | New poll ID | Echoed poll ID |
| `response_frame_id` | 32 | Zero/unused | New response ID |
| `reply_delay_samples` | 32 | Zero/unused | Configured delay |
| CRC16 | 16 | Enabled | Enabled |

After serialization, CRC generation, and padding, TX XORs this exact fixed
128-bit scrambler:

```text
00100101001010011100110011110110010101100100000010010111100110111111010100001000001100001101101100111100000111111110011100100011
```

It has 64 zeros and 64 ones. If `u_q` is the logical payload or padding bit and
`s_q` is the scrambler bit, `b_q=2(u_q\mathbin{\mathrm{XOR}}s_q)-1` under the
project's `0 -> -1`, `1 -> +1` BPSK convention. CRC covers only the original
120 logical bits.

Let `c_i[m]` be chip `m` from row `i` of
`lib/DSP/gold127_family_bipolar.csv`, where row index equals `code_id`. The
default is `i=2`; each row has 127 real chips in `{-1,+1}`. Let `G_m[k]` be
Golay A512 for even `m` and B512 for odd `m`. The native-IFFT input is

```math
X_m[k]=
\begin{cases}
c_i[m]b_q, & k=4q+3 \\
c_i[m]G_m[k], & k\in\mathcal{P}.
\end{cases}
```

The one chip `c_i[m]` multiplies the complete 512-bin symbol. It is not applied
per subcarrier or per time sample. Every symbol repeats the same 128 scrambled
data values. RX removes the Gold chip and Golay sign before inter-symbol CFO
estimation. After CFO phase alignment, it averages channel estimates across
all 127 symbols on the 384 pilot bins. Data bins are Gold-despread, coherently
combined across 127 symbols, equalized by adjacent-pilot interpolation, and
hard-decided. RX XORs the same scrambler, discards positions `120...127`, and
parses/checks the unchanged 120-bit packet and CRC.

A mismatched Gold row suppresses coherent amplitude according to its
cross-correlation with the transmitted row, but `code_id` is not an identity or
authentication check. In a noiseless single-user case, a common residual scale
can cancel when the despread data is divided by the despread channel. Reliable
multi-anchor association must therefore also use packet identity and scheduled
or otherwise separable resources.

The OFDM payload quality metric is the mean normalized BPSK decision margin:

```math
m_q=\frac{|\Re\{\hat D_q\}|}{|\hat D_q|+\epsilon},\qquad
\text{payload\_metric}=\frac{1}{120}\sum_{q=0}^{119}m_q.
```

This metric is relative and dimensionless. CRC remains authoritative. The
channel estimator publishes `failure_reason=NONE` or `PAYLOAD_CRC` after the
OFDM payload decision; acquisition failures still originate in the detector.

## Historical Standalone BPSK Payload

The following section documents the waveform used for the earlier attenuation
experiments. It is not generated or decoded by the current production path.

Both POLL and RESPONSE frames contain the complete fixed-length BPSK payload:

```text
[16 reference samples]
[8-bit packet type, repeated 280 samples/bit]
[32-bit poll frame ID, repeated 280 samples/bit]
[32-bit response frame ID, repeated 280 samples/bit]
[32-bit reply delay, repeated 280 samples/bit]
[16-bit CRC, repeated 280 samples/bit]
```

The payload geometry is:

| Part | Information bits | Samples per bit | Payload samples | Sample offsets |
|---|---:|---:|---:|---:|
| Known BPSK reference | N/A | 1 per reference sign | 16 | 0-15 |
| Packet type | 8 | 280 | 2240 | 16-2255 |
| Poll frame ID | 32 | 280 | 8960 | 2256-11215 |
| Response frame ID | 32 | 280 | 8960 | 11216-20175 |
| Reply delay samples | 32 | 280 | 8960 | 20176-29135 |
| CRC16 | 16 | 280 | 4480 | 29136-33615 |
| **Total** | **120 protected bits** |  | **33616 samples** | **0-33615** |

At `Fs = 30 MS/s`:

```text
one BPSK sample                    = 33.333 ns
one repeated information bit      = 280 samples = 9.333 us
16-sample reference duration      = 0.533 us
33600 repeated data/CRC samples   = 1120.000 us
complete 33616-sample payload     = 1120.533 us
uncoded information-bit rate      = 30 MS/s / 280 = 107.143 kbit/s
```

There is no separate pulse-shaping stage inside the payload encoder. One real
BPSK value is written into each complex baseband sample:

```text
bit 0 -> -amplitude + j0
bit 1 -> +amplitude + j0
```

Every information bit is encoded as 280 identical consecutive samples. After
CFO and common-phase correction, the decoder sums the real values of all 280
samples and decides the bit from the sign of that coherent sum. Sample
magnitude is retained instead of being discarded by hard voting.

The 16 known reference signs are:

```text
+ + + - - + - + - - - + - + + -
```

They estimate one common payload phase. All integer fields, including the
transmitted CRC value, are serialized least-significant bit first. CRC16 uses
initial value `0xffff` and polynomial `0x1021`. It covers these decoded fields:

```text
packet_type
poll_frame_id
response_frame_id
reply_delay_samples
```

The three 32-bit fields enter the CRC one byte at a time, least-significant byte
first.

### POLL and RESPONSE Content

Both packet types occupy all 33616 samples. An unused field is transmitted as
zero; it is not physically omitted.

| Payload field | Initiator POLL | Responder RESPONSE |
|---|---|---|
| `packet_type` | `1` (`POLL`), enabled | `2` (`RESPONSE`), enabled |
| `poll_frame_id` | New initiator poll ID, enabled | Echo of the decoded POLL ID, enabled |
| `response_frame_id` | `0`, **not used for POLL** | New responder response ID, enabled |
| `reply_delay_samples` | `0`, **not used for POLL** | Configured responder delay, enabled |
| `CRC16` | Enabled; covers type, poll ID and the two zero fields | Enabled; covers all four populated fields |

The responder only schedules a RESPONSE after the received POLL payload passes
CRC and has `packet_type = POLL`. Therefore:

```text
POLL payload CRC failure at responder
    -> responder sends no RESPONSE
    -> initiator can later report NO_PREAMBLE for that attempt

RESPONSE payload CRC failure at initiator
    -> initiator reports PAYLOAD_CRC
    -> no valid SS-TWR result is produced
```

### Payload Metric

The packet payload starts with `Nref = 16` known reference symbols. The
remaining `B = 120` data and CRC bits are each repeated `R = 280` times.
The repeated-preamble CFO estimate is initially converted to phase increment
`\hat{\omega}=2\pi\hat{f}_e/F_s` and applied to every payload sample. If this
decode fails CRC, the conditional CP-CFO retry is performed as described in
the consolidated CFO chapter below.

```math
\tilde{y}[i] = y[i]\exp(-j\hat{\omega}i)
```

For derotated reference samples `yref[i]` and expected real BPSK signs `a[i]`,
the decoder calculates

```math
C_{\text{ref}} = \sum_{i=0}^{N_{\text{ref}}-1} y_{\text{ref}}[i]a[i]
```

```math
M_{\text{ref}} =
\min\left(
1,
\frac{|C_{\text{ref}}|}
{\sqrt{N_{\text{ref}}\sum_i|y_{\text{ref}}[i]|^2}}
\right)
```

It uses `conj(Cref) / |Cref|` to remove the remaining common payload phase.
Let `z[b,r]` be corrected sample `r` of bit `b`. The coherent decision is

```math
S[b] = \sum_{r=0}^{R-1}\Re\{z[b,r]\}
```

The decoded bit is one when `S[b] >= 0` and zero otherwise. Its normalized
decision margin is

```math
m[b] =
\frac{|S[b]|}
{\sum_{r=0}^{R-1}|\Re\{z[b,r]\}|}
```

The average decision metric and reported payload metric are

```math
M_{\text{decision}} = \frac{1}{B}\sum_{b=0}^{B-1}m[b]
```

```math
\text{payload metric} = \min(M_{\text{ref}}, M_{\text{decision}})
```

`payload_metric` is in `[0, 1]` and measures reference correlation plus
coherent-decision consistency. It is not SNR, dBFS, or CRC probability. Payload
validity is a separate condition:

```text
frame_id_valid = received CRC16 matches the CRC16 recomputed from decoded fields
```

There is no payload-metric acceptance threshold in the current decoder. A high
`payload_metric` does not override a CRC failure.

The decision metric measures consistency, not correctness. A coherently
combined bit can still have a high margin and the wrong sign; CRC remains the
validity check.

### Attenuation Result Interpretation

The reported attenuation experiment produced approximately:

```text
75 dB attenuation: payload CRC failure reaches about 50% packet failure
95 dB attenuation: repeated-preamble/ZC acquisition begins to fail
```

This ordering is consistent with the implementation. Acquisition integrates a
768-pair repeated-preamble correlation and then a 419-sample coherent ZC
correlation. The payload now coherently combines 280 samples per bit, and CRC
detects errors across 120 decoded bits.

At unchanged per-sample amplitude, increasing from 5 to 280 samples per bit
transmits 56 times as much energy per information bit:

```math
10\log_{10}(280/5) = 17.48\ \text{dB}
```

This is additional transmitted energy and airtime, not coding gain at fixed
`Eb/N0`.

The attenuation result was recorded before section RMS equalization. The
current transmitter now applies:

```text
Payload RMS      = tx_amp
OFDM-symbol RMS  = tx_amp
Preamble RMS     = tx_amp + 3 dB
ZC RMS           = tx_amp + 3 dB
Final peak       <= 0.9
```

The dynamic payload is written before this normalization and one common final
peak scale is applied to the complete burst. Both current examples use
`tx_amp = 0.6`. The attenuation experiment must therefore be repeated before
attributing the earlier 20 dB separation to the updated waveform.

Assuming independent decoded-bit errors and that CRC detects them, a 50% packet
failure rate across 120 bits corresponds approximately to:

```math
P_b \simeq 1 - (1 - 0.5)^{1/120} \simeq 0.00576
```

Thus only about `0.58%` post-combining decoded-bit error probability can already
produce `50%` packet failure. This estimate is illustrative: burst errors,
phase-estimation errors, clipping, and correlated interference violate the
independent-error assumption.

The measured `20 dB` separation is an experimental relative attenuation result,
not an absolute receiver sensitivity or calibrated link-budget value. It does,
however, identify the current BPSK payload/CRC stage as the first observed
digital failure layer before acquisition loss.

## OFDM PRS-Like Symbol

Let:

```text
N      = fft_len
Ncp    = cp_len
K      = active_bins = N
M      = prs_symbols
Fs     = sample rate
Deltaf = Fs / N
```

Every native FFT bin is occupied. The exact native indexing passed to the IFFT
is:

```text
fft_bin 0 ... 255   -> signed bins 0 ... +255
fft_bin 256 ... 511 -> signed bins -256 ... -1
```

Measured over each useful 512-sample IFFT output before CP for 50,000
sequential POLL IDs, the production scrambler gives PAPR
mean/P99.9/maximum of `7.1019 / 8.9338 / 9.5733 dB`. Multiplication by a real
Gold chip does not alter a symbol's PAPR.

The Gold-code source of truth and its production representation are:

```text
lib/DSP/gold127_family_bipolar.csv
lib/DSP/gold127_codes.h
```

No CSV is read at GNU Radio runtime. `tools/generate_gold127_header.py`
validates exactly 129 rows by 127 columns of `+1/-1` and generates the header.
CSV row `i` maps to code ID `i`, and column `m` maps to OFDM symbol `m`.

The length-512 Golay pair is generated at compile time using:

```text
A0 = [1]
B0 = [1]
A_next = [A, B]
B_next = [A, -B]
```

The 127 symbols assign:

```text
even symbols 0,2,...,126 -> A512
odd symbols 1,3,...,125  -> B512
```

TX uses native `k=0...511` without fftshift. Pilot positions use the Golay
entry multiplied by the known Gold chip; positions `k mod 4 = 3` use the
spread BPSK data value. There is no OFDM PRS MT19937 generation.

The time-domain OFDM symbol before cyclic prefix is

```math
s_m[n] =
\frac{1}{N}
\sum_{k=0}^{N-1}
X_m[k]\exp\left(j2\pi\frac{kn}{N}\right),
\quad 0 \le n < N
```

The transmitted CP-OFDM symbol is

```math
x_m[n] =
\begin{cases}
s_m[n + N - N_{\text{CP}}], & 0 \le n < N_{\text{CP}} \\
s_m[n - N_{\text{CP}}], & N_{\text{CP}} \le n < N + N_{\text{CP}}
\end{cases}
```

The receiver removes the CP and computes the native FFT. For phase-slope
processing it flattens bins in monotonic signed-frequency order:

```text
native 256 ... 511, then native 0 ... 255
signed -256 ... -1, then signed 0 ... +255
```

The channel estimator fetches Golay references in this same flattened order,
while each reference is still indexed from the native table. TX and RX
therefore use identical `(symbol, fft_bin)` entries without reordering the IFFT
input.

## Channel Estimation

For pilot subcarrier `k` of OFDM symbol `m`, the received value is modeled as

```math
Y_m[k] = H[k]c[m]G_m[k] + W_m[k]
```

The per-symbol channel estimate is

```math
\hat{H}_m[k] = \frac{Y_m[k]}{c[m]G_m[k]}
```

The implementation averages over all PRS symbols:

```math
\hat{H}[k] =
\frac{1}{M}
\sum_{m=0}^{M-1}
\frac{Y_m[k]}{c[m]G_m[k]}
```

Only the 384 pilot estimates are emitted to the phase-slope estimator. The
implementation also computes a relative SNR-like metric from average pilot
channel power and residual error; it is not calibrated RF power.

## CFO Estimation and Correction

Three CFO methods are relevant to the design, but the current production path
only produces the CP and pilot-channel estimates:

```text
repeated preamble -> method documented below; currently disabled
PRS cyclic prefix -> prs_cp_cfo_hz
adjacent PRS channel estimates -> prs_channel_cfo_hz
```

Let

```text
Fs    = sample rate
Lp    = repeated-preamble period length
R     = number of preamble repetitions
NFFT  = FFT length
NCP   = cyclic-prefix length
M     = number of PRS symbols
K     = number of active PRS bins
```

The current USRP examples use:

```text
Fs = 30 MHz, Lp = 256, R = 8
NFFT = 512, NCP = 64, M = 127, K = 384 pilot bins
```

### Repeated-Preamble CFO

This method is not calculated by either current frame detector. The repeated
preamble remains an acquisition gate only, and `preamble_cfo_hz` is no longer
published or written to current CSV files. The equations remain here to record
the evaluated alternative.

For constant CFO `f_e`, samples in adjacent copies of the repeated preamble are
approximately related by

```math
y[n+L_p] \approx y[n]
\exp\left(j2\pi f_e\frac{L_p}{F_s}\right)
```

After frame extraction, the detector accumulates

```math
C_p^{(f)} =
\sum_{n=0}^{L_p(R-1)-1} y[n]^*y[n+L_p]
```

and estimates

```math
\hat f_p =
\frac{F_s}{2\pi L_p}\angle C_p^{(f)}
```

This calculation uses `L_p(R-1)` complex correlation products, covers all
`L_pR` preamble samples, and has correlation lag `L_p`. With the current USRP
geometry:

```text
correlation products = 256(8-1) = 1792
unique samples covered = 256(8) = 2048
observation duration = 2048 / 30 MHz = 68.267 us
correlation lag = 256 / 30 MHz = 8.533 us
unambiguous CFO range = +/-Fs/(2Lp) = +/-58.594 kHz
```

The preamble method has the widest acquisition range and would be available
first. A receiver using it would convert the estimate to phase increment

```math
\hat\omega_p = 2\pi\frac{\hat f_p}{F_s}
```

The current MC-DS payload decoder does not use this path.

### PRS Cyclic-Prefix CFO

The CP of PRS symbol `m` duplicates the final `N_CP` samples of its useful
`N_FFT`-sample waveform. Corresponding samples are separated by exactly
`N_FFT` samples. If `s_m` is the start of the CP, the detector calculates

```math
C_{\text{CP}} =
\sum_{m=0}^{M-1}
\sum_{n=0}^{N_{\text{CP}}-1}
y[s_m+n]^*y[s_m+N_{\text{FFT}}+n]
```

The raw phase is ambiguous by integer multiples of `2\pi`. The standard frame
detector currently selects the principal CP-CFO branch around zero. The FFT-ZC
detector unwraps around its acquisition-bin CFO estimate. In the general form,
let that reference be `hat f_ref`:

```math
\theta_{\text{ref}} =
2\pi\hat f_{\text{ref}}\frac{N_{\text{FFT}}}{F_s}
```

```math
\tilde\theta_{\text{CP}} =
\angle C_{\text{CP}} +
2\pi\operatorname{round}\left(
\frac{\theta_{\text{ref}}-\angle C_{\text{CP}}}{2\pi}
\right)
```

The CP estimate is

```math
\hat f_{\text{CP}} =
\frac{F_s}{2\pi N_{\text{FFT}}}
\tilde\theta_{\text{CP}}
```

and its normalized coherence is

```math
\rho_{\text{CP}} =
\frac{|C_{\text{CP}}|}
{\sqrt{
\left(\sum |y_{\text{CP}}|^2\right)
\left(\sum |y_{\text{tail}}|^2\right)}}
```

For the current geometry:

```text
correlation products = M NCP = 127(64) = 8128
selected input samples = 16256
correlation lag = 512 / 30 MHz = 17.067 us
complete PRS span = 127(512+64) = 73152 samples = 2.4384 ms
raw unambiguous CFO range = +/-Fs/(2NFFT) = +/-29.297 kHz
CFO alias interval = Fs/NFFT = 58.594 kHz
```

The CP estimate is attached to the extracted frame and used as the unwrap
reference for channel CFO. It is available only after a complete frame has
been extracted, so it cannot rescue failed preamble or ZC acquisition.

### PRS Channel CFO

After the FFT and pilot division, the channel estimator retains every
per-symbol channel estimate:

```math
\hat H_m[k] = \frac{Y_m[k]}{c[m]G_m[k]},\quad k\in\mathcal{P}
```

Constant CFO appears primarily as common phase evolution between adjacent PRS
symbols:

```math
\hat H_{m+1}[k] \approx
\hat H_m[k]\exp(j2\pi f_e T_{\text{sym}})
```

where

```math
T_{\text{sym}} =
\frac{N_{\text{FFT}}+N_{\text{CP}}}{F_s}
```

The estimator accumulates

```math
C_H =
\sum_{m=0}^{M-2}
\sum_{k=0}^{K-1}
\hat H_m[k]^*\hat H_{m+1}[k]
```

It unwraps `angle C_H` relative to `hat f_CP`, or relative to the detector's
acquisition CFO when the CP field is unavailable, and calculates

```math
\hat f_H =
\frac{\widetilde{\angle C_H}}
{2\pi T_{\text{sym}}}
```

The channel coherence is the magnitude of `C_H` divided by the square root of
the accumulated powers of the two members of every adjacent-symbol pair. The
current geometry gives:

```text
per-symbol pilot-channel estimates = MK = 127(384) = 48768
adjacent-symbol products = (M-1)K = 126(384) = 48384
adjacent-symbol separation = 576 samples = 19.2 us
complete PRS span = 73152 samples = 2.4384 ms
raw unambiguous CFO range = +/-Fs/[2(NFFT+NCP)] = +/-26.042 kHz
CFO alias interval = Fs/(NFFT+NCP) = 52.083 kHz
```

Every symbol channel is then rotated to the time reference of symbol zero:

```math
\tilde H_m[k] =
\hat H_m[k]
\exp(-j2\pi\hat f_H mT_{\text{sym}})
```

and the CFO-aligned channel used by the phase-slope estimator is

```math
\hat H[k] = \frac{1}{M}\sum_{m=0}^{M-1}\tilde H_m[k]
```

### Comparison and Current Roles

| Estimate | Products | Correlation separation | Observation span | Raw CFO range | Current role |
|---|---:|---:|---:|---:|---|
| Preamble `hat f_p` | 1792 | 256 samples | 2048 samples | +/-58.594 kHz | Disabled; acquisition gate only |
| PRS CP `hat f_CP` | 8128 | 512 samples | 73152-sample PRS span | +/-29.297 kHz | Channel-CFO unwrap reference |
| PRS channel `hat f_H` | 48384 | 576 samples | 73152-sample PRS span | +/-26.042 kHz | Pilot/data inter-symbol phase alignment |

The two active estimates should be approximately equal for a stable oscillator
and a correctly extracted frame. Differences close to `Fs/N_FFT` or
`Fs/(N_FFT+N_CP)` indicate a likely phase-ambiguity branch error. Low CP
coherence points toward low SNR, incorrect timing, interference, or delay spread
beyond the CP. High CP coherence with low channel coherence instead points
toward channel variation, SFO, ICI, or an FFT-window problem.

The metadata used for comparison is:

```text
prs_cp_cfo_hz
prs_cp_cfo_coherence
selected_cfo_hz
prs_channel_cfo_hz
residual_cfo_hz
channel_coherence
```

`residual_cfo_hz` currently means `prs_channel_cfo_hz` minus the unwrap
reference. It is not a fourth independent CFO measurement made after channel
correction.

### Warning: Current PRS ICI Is Not Removed

> **Warning:** The current FFT receiver does not derotate raw PRS time samples
> before the FFT. Preamble and CP CFO are used for payload decoding and phase
> unwrapping, while `hat f_H` is applied only to channel estimates after the
> FFT.

Post-FFT channel rotation removes common phase evolution between OFDM symbols,
but it cannot reverse leakage that CFO has already produced between subcarriers.
For normalized CFO

```math
\epsilon_f = \frac{f_e}{\Delta f},
\qquad
\Delta f = \frac{F_s}{N_{\text{FFT}}}
```

the received bin contains both its desired term and CFO-dependent contributions
from other bins. This ICI can bias channel phase and therefore bias the final
phase-slope delay. The present correction also does not remove SFO, phase noise,
multipath distortion, or an incorrect integer FFT-window position.

`selected_cfo_hz` records the detector-side CFO selection; MC-DS payload
decoding uses the downstream pilot-channel CFO estimate. The channel estimator always calculates
and applies its inter-symbol estimate without a coherence threshold. A
low-coherence channel CFO can therefore rotate channel symbols using an
unreliable estimate. In addition, an invalid CP estimate is currently exported
as finite value zero, so downstream code cannot distinguish invalid CP from a
true zero-Hz estimate using `prs_cp_cfo_hz` alone.

### Recommended First Implementation

The first implementation should use two correction stages:

```text
reliable CP CFO, otherwise detector acquisition CFO or zero
-> continuous time-domain PRS derotation
-> FFT and pilot division
-> residual PRS channel CFO estimation
-> residual inter-symbol channel rotation
-> channel averaging
-> phase-slope delay estimation
```

Select the pre-FFT estimate as

```math
\hat f_0 =
\begin{cases}
\hat f_{\text{CP}}, & \rho_{\text{CP}} \ge \rho_{\min} \\
\hat f_{\text{det}}, & \text{otherwise}
\end{cases}
```

with an initial `rho_min` in the range `0.3...0.5`. Before each FFT, derotate
the useful PRS samples using a continuous sample index measured from the start
of the PRS section:

```math
\tilde y[n] =
y[n]\exp\left(-j2\pi\hat f_0\frac{n}{F_s}\right)
```

The phase index must continue across CP-OFDM symbols rather than restart at
zero for each useful symbol. This removes within-symbol rotation and most
inter-symbol common phase before the FFT. It requires only `M NFFT = 8192`
complex multiplications per frame and can use a recursively updated complex
phasor instead of one sine/cosine evaluation per sample.

After pre-FFT correction, the adjacent-channel estimator measures residual CFO,
not total CFO. Its unwrap reference must therefore change from `hat f_CP` to
zero. Define

```math
\hat f_{\text{total}} = \hat f_0 + \hat f_{\text{res}}
```

and use `hat f_res` for the remaining inter-symbol channel rotation. Suggested
metadata is:

```text
prs_time_cfo_applied_hz
prs_residual_cfo_hz
prs_total_cfo_hz
prs_cp_cfo_valid
```

If the residual is still large enough to cause meaningful ICI, an optional
second pass can repeat the PRS derotation and FFT using `hat f_total`. The
recommended first implementation is the single pre-FFT correction plus
residual channel rotation; it addresses the existing ICI problem with much
less computation than a two-pass receiver.

## Phase-Slope Delay Estimation

For a single path with propagation delay `tau`, the frequency-domain channel has
approximately linear phase:

```math
H(f) = A(f)\exp(-j2\pi f\tau)
```

Therefore:

```math
\phi(f) = \angle H(f) \approx \phi_0 - 2\pi f\tau
```

The implementation unwraps the phase of the averaged channel estimates:

```math
\phi_k = \operatorname{unwrap}(\angle \hat{H}[k])
```

Then it performs weighted linear regression:

```math
\phi_k \approx a + b f_k
```

with weight

```math
w_k = |\hat{H}[k]|^2
```

The estimated fine delay is

```math
\hat{\tau} = -\frac{b}{2\pi}
```

The residual RMS is

```math
\epsilon_{\text{rms}} =
\sqrt{\frac{1}{K}\sum_k(\phi_k - a - bf_k)^2}
```

The implementation outputs:

```text
fine_delay
fine_delay_samples = fine_delay * Fs
phase_slope
phase_residual
quality
```

## Why Phase-Slope Does Not Currently Improve `range_m`

The present SS-RTT `range_m` is computed from timestamps:

```math
RTT = T_4 - T_1 - T_{\text{reply}}
```

```math
ToF = \frac{RTT - T_{\text{cal}}}{2}
```

```math
range = c \cdot ToF
```

The `fine_delay` from phase slope is measured and logged, but it is not currently
fused into the SS-RTT range calculation.

Even if fused, phase-slope delay is not automatically better in the current
hardware setup because it includes more than propagation delay:

```text
measured phase slope =
  propagation delay
+ TX/RX RF group delay
+ analog filter group delay
+ antenna/cable delay
+ residual coarse timing offset
+ residual CFO/SFO phase error
+ multipath phase distortion
+ USRP channel-dependent phase behavior
```

The issue is not simply that sample range resolution is larger than carrier
phase resolution. A sample at `Fs = 10 MHz` is

```math
T_s = 100 ns
```

For two-way RTT, one sample corresponds to

```math
\Delta R_{\text{sample}} = \frac{cT_s}{2} \approx 15 m
```

For `Fs = 30 MHz`, this becomes about `5 m`.

Phase slope can in principle estimate sub-sample delay because it uses phase
across bandwidth, not only the nearest time sample. However, the useful
resolution depends on bandwidth, SNR, calibration, and model validity. The 384
pilot bins are interleaved across the complete 512-bin FFT, so their frequency
support spans approximately

```math
B \approx N\Delta f = F_s
```

The current 512-bin configuration therefore gives:

```text
Fs = 10 MHz: B ≈ 10 MHz
Fs = 30 MHz: B ≈ 30 MHz
```

The rough delay discrimination scale is on the order of `1/B`, which corresponds
to tens of meters unless SNR and calibration are good enough for sub-bin
super-resolution. In this project, the uncalibrated RF/clock/group-delay terms
are currently larger than the fine propagation phase term being sought.

Therefore, phase-slope is useful today as a quality/channel diagnostic, but the
stable absolute range is still dominated by SS-RTT timestamping and empirical
calibration.

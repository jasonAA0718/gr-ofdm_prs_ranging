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
[BPSK packet payload]
[OFDM PRS-like pilot symbols]
[tail guard]
```

Using the common defaults:

```text
zero_guard_len    = 1000 samples
preamble_len      = 128 samples
preamble_repeats  = 16
coarse_sync_len   = 839 samples
payload_len       = 616 samples
fft_len           = 1024
cp_len            = 128
active_bins       = 1024
prs_symbols       = 16
```

The OFDM PRS-like section length is

```math
N_{\text{PRS}} = N_{\text{sym}}(N_{\text{FFT}} + N_{\text{CP}})
```

For the default values:

```math
N_{\text{PRS}} = 16(1024 + 128) = 18432
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

The repeated preamble first proposes a frame location. The ZC metric then
confirms the coarse sync position and, with configurable roots, can separate
channels/responders.

Example channel plan:

```text
Initiator POLL TX:       coarse_zc_root = 25
Responder RX detector:   coarse_zc_root = 25
Responder RESPONSE TX:   coarse_zc_root = 29
Initiator RX detector:   coarse_zc_root = 29, channel_id = 1
```

## BPSK Packet Payload

Both POLL and RESPONSE frames contain the complete fixed-length BPSK payload:

```text
[16 reference samples]
[8-bit packet type, repeated 5 samples/bit]
[32-bit poll frame ID, repeated 5 samples/bit]
[32-bit response frame ID, repeated 5 samples/bit]
[32-bit reply delay, repeated 5 samples/bit]
[16-bit CRC, repeated 5 samples/bit]
```

The payload geometry is:

| Part | Information bits | Samples per bit | Payload samples | Sample offsets |
|---|---:|---:|---:|---:|
| Known BPSK reference | N/A | 1 per reference sign | 16 | 0-15 |
| Packet type | 8 | 5 | 40 | 16-55 |
| Poll frame ID | 32 | 5 | 160 | 56-215 |
| Response frame ID | 32 | 5 | 160 | 216-375 |
| Reply delay samples | 32 | 5 | 160 | 376-535 |
| CRC16 | 16 | 5 | 80 | 536-615 |
| **Total** | **120 protected bits** |  | **616 samples** | **0-615** |

At `Fs = 30 MS/s`:

```text
one BPSK sample                    = 33.333 ns
one repeated information bit      = 5 samples = 166.667 ns
16-sample reference duration      = 0.533 us
600 repeated data/CRC samples     = 20.000 us
complete 616-sample payload       = 20.533 us
uncoded information-bit rate      = 30 MS/s / 5 = 6 Mbit/s
```

There is no separate pulse-shaping stage inside the payload encoder. One real
BPSK value is written into each complex baseband sample:

```text
bit 0 -> -amplitude + j0
bit 1 -> +amplitude + j0
```

Every information bit is encoded as five identical consecutive samples. The
decoder makes a hard majority decision; at least three of the five signs
determine the decoded bit.

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

Both packet types occupy all 616 samples. An unused field is transmitted as
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
remaining `B = 120` data and CRC bits are each repeated `R = 5` times.

For received reference samples `yref[i]` and expected real BPSK signs `a[i]`,
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

It uses `conj(Cref) / |Cref|` to remove the common payload phase. For bit `b`,
let `q[b]` be the number of its five corrected samples whose real part is
nonnegative. The hard-decision vote margin is

```math
v[b] = \frac{|2q[b]-R|}{R}
```

For `R = 5`, one bit contributes:

```text
5-0 or 0-5 vote: 1.0
4-1 or 1-4 vote: 0.6
3-2 or 2-3 vote: 0.2
```

The average vote metric and reported payload metric are

```math
M_{\text{vote}} = \frac{1}{B}\sum_{b=0}^{B-1}v[b]
```

```math
\text{payload_metric} = \min(M_{\text{ref}}, M_{\text{vote}})
```

`payload_metric` is in `[0, 1]` and measures reference correlation plus
hard-vote consistency. It is not SNR, dBFS, or CRC probability. Payload
validity is a separate condition:

```text
frame_id_valid = received CRC16 matches the CRC16 recomputed from decoded fields
```

There is no payload-metric acceptance threshold in the current decoder. A high
`payload_metric` does not override a CRC failure.

The vote metric measures decision agreement, not correctness. Five consistently
wrong samples produce a unanimous `5-0` vote and contribute `1.0`. The decoder
also discards sample magnitude after common-phase correction; it counts positive
and negative real parts instead of soft-combining their amplitudes.

### Attenuation Result Interpretation

The reported attenuation experiment produced approximately:

```text
75 dB attenuation: payload CRC failure reaches about 50% packet failure
95 dB attenuation: repeated-preamble/ZC acquisition begins to fail
```

This ordering is consistent with the implementation. Acquisition integrates a
768-pair repeated-preamble correlation and then a 419-sample coherent ZC
correlation. Each payload bit has only a five-sample hard-decision majority
vote, and CRC detects errors across 120 decoded bits.

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

Thus only about `0.58%` post-majority decoded-bit error probability can already
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

Every native FFT bin is occupied. The CSV uses the exact native indexing passed
to the IFFT:

```text
fft_bin 0 ... 511    -> signed bins 0 ... +511
fft_bin 512 ... 1023 -> signed bins -512 ... -1
```

Measured over each useful 1024-sample IFFT output before CP, the 16 fixed Golay
symbols have minimum/mean/maximum PAPR of
`3.0062 / 3.0062 / 3.0062 dB`. The former 600-bin Random-QPSK construction,
retained only in QA for comparison, measures
`7.4209 / 8.9281 / 10.4139 dB`.

The exact frequency-domain source of truth is:

```text
lib/golay_ofdm_1024x16.csv
```

It is converted offline into the compile-time table:

```text
lib/golay_prs_table.h
```

No CSV is read at GNU Radio runtime. The current entries are real BPSK
`+1+j0` or `-1+j0`, but both real and imaginary components are retained in the
compiled representation.

The table contains the recursively generated length-1024 Golay pair:

```text
A0 = [1]
B0 = [1]
A_next = [A, B]
B_next = [A, -B]
```

and assigns:

```text
symbols 0,2,4,...,14 -> A
symbols 1,3,5,...,15 -> B
```

TX uses `X_m[k] = table[m][k]` directly for native `k=0...1023`; there is no
fftshift and no PRS MT19937 generation.

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
native 512 ... 1023, then native 0 ... 511
signed -512 ... -1, then signed 0 ... +511
```

The channel estimator fetches Golay references in this same flattened order,
while each reference is still indexed from the native table. TX and RX
therefore use identical `(symbol, fft_bin)` entries without reordering the IFFT
input.

## Channel Estimation

For active subcarrier `k` of OFDM symbol `m`, the received value is modeled as

```math
Y_m[k] = H[k]X_m[k] + W_m[k]
```

The per-symbol channel estimate is

```math
\hat{H}_m[k] = \frac{Y_m[k]}{X_m[k]}
```

The implementation averages over all PRS symbols:

```math
\hat{H}[k] =
\frac{1}{M}
\sum_{m=0}^{M-1}
\frac{Y_m[k]}{X_m[k]}
```

It also computes a simple SNR-like metric from the average channel power and
residual error around the average channel estimate.

## Coarse CFO Estimation

The repeated preamble also provides a coarse carrier frequency offset estimate.
If the received repeated preamble has CFO `f_e`, adjacent periods are related by

```math
y[n+L_p] \approx y[n]\exp\left(j2\pi f_e \frac{L_p}{F_s}\right)
```

The detector accumulates

```math
C_{\text{CFO}} =
\sum_n y[n]^*y[n+L_p]
```

Then

```math
\hat{f}_e =
\frac{\angle C_{\text{CFO}}}{2\pi}
\frac{F_s}{L_p}
```

This estimate is reported as metadata. In the current receiver chain it is not
used to derotate the OFDM PRS symbols before phase-slope estimation.

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
resolution depends on bandwidth, SNR, calibration, and model validity. With
active bandwidth approximately

```math
B \approx K\Delta f = K\frac{F_s}{N}
```

the full-band Golay configuration `K=N=1024` gives:

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

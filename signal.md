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
active_bins       = 600
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

## OFDM PRS-Like Symbol

Let:

```text
N      = fft_len
Ncp    = cp_len
K      = active_bins
M      = prs_symbols
Fs     = sample rate
Deltaf = Fs / N
```

The active bins exclude DC and are split equally around DC:

```math
\mathcal{K} =
\left\{-\frac{K}{2}, \ldots, -1\right\}
\cup
\left\{1, \ldots, \frac{K}{2}\right\}
```

For OFDM symbol index `m`, deterministic QPSK pilots are generated:

```math
X_m[k] \in
\left\{\frac{1+j}{\sqrt{2}}, \frac{1-j}{\sqrt{2}},
\frac{-1+j}{\sqrt{2}}, \frac{-1-j}{\sqrt{2}}\right\},
\quad k \in \mathcal{K}
```

Inactive bins and DC are zero:

```math
X_m[0] = 0,\quad X_m[k] = 0 \quad k \notin \mathcal{K}
```

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

The receiver removes the CP, computes an FFT, and extracts active bins in this
order:

```text
negative active bins, then positive active bins
```

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

the default `K=600`, `N=1024` gives:

```text
Fs = 10 MHz: B ≈ 5.86 MHz
Fs = 30 MHz: B ≈ 17.58 MHz
```

The rough delay discrimination scale is on the order of `1/B`, which corresponds
to tens of meters unless SNR and calibration are good enough for sub-bin
super-resolution. In this project, the uncalibrated RF/clock/group-delay terms
are currently larger than the fine propagation phase term being sought.

Therefore, phase-slope is useful today as a quality/channel diagnostic, but the
stable absolute range is still dominated by SS-RTT timestamping and empirical
calibration.

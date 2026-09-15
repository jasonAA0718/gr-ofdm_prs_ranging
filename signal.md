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

At 30 MS/s, the enforced current OFDM geometry and default code selection are:

```text
fft_len           = 512
cp_len            = 64
active_bins       = 512
prs_symbols       = 127
Gold code ID      = 2
```

The code ID is configurable; 512 bins, CP 64, and 127 symbols are enforced by
the current transmitter. The sample rate is configurable as well; numerical
examples in this document use 30 MS/s.

The OFDM PRS-like section length is

```math
N_{\text{PRS}} = N_{\text{sym}}(N_{\text{FFT}} + N_{\text{CP}})
```

For these OFDM values:

```math
N_{\text{PRS}} = 127(512 + 64) = 73152
```

Acquisition geometry is configurable and must match the peer transmitter.
The working-tree main examples differ: initiator period/repeats/ZC length
are 512/33/2001, responder values are 512/16/1999. The FFT-ZC profiling pair
uses 256/4/419. All four use 30 MS/s and carrier 1060 MHz; these are example
settings, not immutable waveform requirements. The main initiator uses
100-sample zero and tail guards, while the responder and profiling examples
use 1000 samples each. These differences also affect the timestamp reference.

For zero guard length `N_g`, preamble period `L_p`, repetitions `R`,
ZC length `N_z`, and tail guard `N_t`:

```math
N_{frame}=N_g+L_pR+N_z+M(N+N_{CP})+N_t.
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
x_p[n + rL_p] = 0.95p[n],
\quad 0 \le n < L_p,\quad 0 \le r < R
```

where `R = preamble_repeats`. TX seeds `std::mt19937` with
`seed ^ 0x5a17U`, takes one generator output per period sample, and maps bits
0 and 1 to the real/imaginary signs of unit QPSK. The same period is copied
`R` times; normalization then multiplies it by 0.95.

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
QPSK values, only that adjacent preamble periods are repeated. It is not an
accurate boundary estimator: partial overlap can pass the gate.

The local detector updates correlation and the two energies as a rolling
window. It compares squared correlation against `threshold^2 * E1 * E2`
before taking the square root. With nonpositive denominator the metric is
zero; there is no added epsilon floor in this normalization. The gate threshold
is configurable (0.35 in the supplied examples), distinct from ZC threshold.
Uniform amplitude scaling cancels ideally, but noise and distortion do not.

## Coarse ZC Sync

After the repeated preamble, the burst carries a Zadoff-Chu-like coarse sync
sequence. For length `N_z` and root `u`, the current implementation generates

```math
z_u[n] = \exp\left(-j\pi u\frac{n(n+1)}{N_z}\right),
\quad 0 \le n < N_z
```

The transmitted ZC is multiplied by 0.95; the receiver uses the unit-magnitude
reference above. It computes the normalized coarse correlation

```math
C_z[m] = \sum_{n=0}^{N_z-1} y[m+n] z_u[n]^*
```

```math
M_z[m] =
\frac{|C_z[m]|}
{\sqrt{N_z \sum_{n=0}^{N_z-1}|y[m+n]|^2}}
```

In the local `prs_frame_detector`, the repeated preamble first gates a proposed frame location using the preamble
threshold. The detector then evaluates `M_z` at offsets `-2...+2` around the
predicted ZC boundary, selects the strongest valid offset, and accepts it only
when it passes the separate ZC threshold. The selected offset corrects both the
frame start and coarse-sync index. A threshold-passing edge maximum recenters
the five-point window once before acceptance. Configurable roots continue to
separate channels/responders.

A historical single-responder root convention is:

```text
Initiator POLL TX:       coarse_zc_root = 25
Responder RX detector:   coarse_zc_root = 25
Responder RESPONSE TX:   coarse_zc_root = 29
Initiator RX detector:   coarse_zc_root = 29, channel_id = 1
```


### FFT-ZC Detector

All supplied GRCs currently select the separate `prs_fft_zc_frame_detector`.
It implements linear matched filtering with a reversed conjugate ZC kernel:

```math
h[n]=z_u[N_z-1-n]^*,\qquad
r=\operatorname{IFFT}(\operatorname{FFT}(y)\operatorname{FFT}(h)).
```

For correlation FFT size `F`, each block supplies at most `F-N_z+1`
candidate positions and overlaps the next input by `N_z-1` samples.
Only outputs at `N_z-1+i` are used, with IFFT scaling `1/F`.
Each candidate has rolling input-energy normalization using the same
`M_z` formula; this avoids circular wrap contamination of accepted outputs.

In continuous mode, a preamble gate predicts ZC start at gate index plus
`L_p R`. `zc_search_before/after` bound the search around that prediction.
In time-gated mode, the ZC search spans the buffered scheduled response
interval; the global preamble maximum is diagnostic and does not restrict
the ZC search. `zc_gate_offset_samples` is selected ZC index minus predicted
ZC index, and need not be near zero.

Candidates are visited in time order. The accepted point is the maximum
within the first contiguous lobe at or above `zc_threshold`, not the global
maximum over all paths. There is no peak-ratio threshold. A lobe reaching the
end of the available search can be accepted without observing its falling edge.
Example ZC thresholds are 0.4 at the initiator and 0.5 at the responder.

Optional CFO compensation applies only to matched-filter input copies.
With no tracked CFO it searches -300 through +300 Hz in 100 Hz steps.
A finite channel-CFO feedback value with `channel_coherence >= 0.2` becomes
the next acquisition reference. Metadata uses `INITIAL_BIN`, `PRS_FLL`,
or `DISABLED`. This feedback does not derotate the raw PRS frame sent to
the FFT receiver. Within the bootstrap bank, accepted threshold peaks take
priority, then earlier candidate indices, then larger metric at the same index.
For the profiling ZC length 419, the observation lasts only 13.967 us at
30 MS/s; 100 Hz hypotheses are very close compared with its reciprocal span
of approximately 71.6 kHz. The bank is an acquisition aid, not a precise
first-frame frequency measurement.

### Frame and Timestamp Reference

For a selected ZC start sample `n_z`, both detectors derive:

```math
n_{frame}=n_z-N_g-L_pR,\qquad
n_{PRS}=n_z+N_z.
```

The extracted vector includes the configured zero guard, preamble, ZC, OFDM
section, and tail guard. `frame_start` and `coarse_peak` are absolute
sample indices; `coarse_peak` denotes ZC start, not the convolution-output
index `N_z-1` samples later. `prs_start_rel` is the OFDM CP start relative
to the extracted frame. `rx_time` is reconstructed for that frame start
from UHD tags and sample offset. Metadata includes `preamble_metric`,
`coarse_metric`, and CP-CFO diagnostics. `peak_metric` is a legacy alias
accepted by some downstream readers, not a separate estimator.

Detectors require a complete configured frame before publishing `frame_out`.
Minimum-frame-gap checks and consumed-frame buffer drops suppress duplicates.
Time-gated expiry can instead publish acquisition failure metadata on
`event_out`. Successful raw frames proceed to the channel estimator for CRC;
detector success alone does not imply payload success.

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
project's `0 -> -1`, `1 -> +1` BPSK convention. CRC is computed over the first 104 logical information bits; its 16-bit
value completes the 120-bit packet. Scrambling and padding are outside CRC
coverage. The CRC update consumes packet type followed by each 32-bit field
in least-significant-byte order; the transmitted fields and CRC are LSB-first.

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
m_q=\frac{|\Re\{\hat D_q\}|}{\max(10^{-12},|\hat D_q|)},\qquad
\text{payload\_metric}=\frac{1}{120}\sum_{q=0}^{119}m_q.
```

This metric is relative and dimensionless. CRC remains authoritative. The
channel estimator publishes `failure_reason=NONE` or `PAYLOAD_CRC` after the
OFDM payload decision; acquisition failures still originate in the detector.

## Section Normalization

`prs_frame_builder::normalize_sections()` multiplies the generated unit
preamble and ZC by 0.95. For each CP-plus-OFDM symbol it measures

```math
r_m=\sqrt{\frac{1}{N+N_{CP}}\sum_n |x_m[n]|^2},
\qquad a_m=\frac{\text{tx\_amp}}{r_m}.
```

It scales each symbol by `a_m`, measures the largest resulting OFDM
magnitude `A_peak`, and scales only the complete OFDM section by

```math
g_{OFDM}=\min\left(1,\frac{0.9}{A_{peak}}\right).
```

The effective CP-plus-symbol RMS is `tx_amp * g_OFDM`; preamble and ZC
remain at 0.95. Guards remain zero. This is uniform gain reduction, not
sample clipping. TX amplitude is not calibrated dBm. There is no standalone
payload amplitude setting. Positive real scaling preserves CP copying and
phase, but symbol-dependent normalization is included in the effective channel
seen by RX because pilot division does not explicitly remove `a_m`.

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

## Channel Estimation and Payload Decoding

On pilot bins, RX divides the FFT output by the unit known reference:

```math
\hat H_m[k]=\frac{Y_m[k]}{c_i[m]G_m[k]},\quad k\in\mathcal P.
```

This effective channel includes transmit scaling, RF response, residual timing
and CFO effects. Under a static single-path model it is approximately channel
gain times a linear delay phase, but the implementation does not fit a
multipath tap model or the paper-style OPA low/high-frequency estimator.

The channel CFO calculation below gives one symbol rotation
`r_m = exp(-j 2 pi hat f_H m T_sym)`. RX multiplies each pilot estimate by
that rotation, then averages all 127 symbols:

```math
\tilde H_m[k]=r_m\hat H_m[k],\qquad
\bar H[k]=\frac{1}{M}\sum_m\tilde H_m[k].
```

Only 384 pilot bins, in increasing signed-frequency order, are sent to the
phase estimator. No single OFDM symbol is privileged as the only regression
input; symbol zero is the phase reference for alignment.

For data bin `k=4q+3`, the decoder computes

```math
D_q=\frac{1}{M}\sum_m Y_m[k]c_i[m]r_m.
```

Its interpolated channel is `(bar H[k-1]+bar H[k+1])/2`, except native
bin 511 uses only bin 510. With channel power above `1e-12`, equalization
is `D_q * conj(H_interp) / |H_interp|^2`; otherwise the equalized value is
zero. The hard bit is one if the equalized real part is nonnegative, zero
otherwise. RX descrambles, removes eight padding bits, and compares the
received CRC with the recomputed CRC over the 104 information bits.

`frame_id_valid` reports that CRC comparison, not a phase-fit test or packet
type check. The channel estimator publishes payload outcomes on `event_out`
and the channel on `channel_out`. SS-TWR then checks POLL/RESPONSE type.
Legacy retry fields are populated with false/zero; there is no current
CP-based payload retry.

### SNR and Coherence

The logged `snr` is derived from the same pilot samples after phase alignment:

```math
P_s=\frac{1}{K_p}\sum_{k\in\mathcal P}|\bar H[k]|^2,
```

```math
P_e=\frac{1}{K_pM}\sum_{k\in\mathcal P}
\max\left(0,\sum_m|\tilde H_m[k]|^2-M|\bar H[k]|^2\right),
\quad K_p=384,
```

```math
\mathrm{snr}=10\log_{10}\frac{P_s+10^{-12}}{P_e+10^{-12}}.
```

The residual equals symbol-to-symbol error around the averaged channel, up to
roundoff. No noise-only guard samples or payload bins enter this estimate.
It approximates pilot SNR under a static channel and independent additive
noise; channel variation, residual CFO/ICI, reference mismatch and amplitude
variation also count as error. It is distinct from adjacent-symbol
`channel_coherence`, and neither is calibrated absolute RX power.

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
K     = number of pilot bins used for channel CFO
```

The shared OFDM geometry is:

```text
Fs = 30 MHz
NFFT = 512, NCP = 64, M = 127, K = 384 pilot bins
```

Preamble counts below use the matched profiling pair: Lp=256, R=4.

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

The historical estimator accumulated

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
`L_pR` preamble samples, and has correlation lag `L_p`. For the profiling preamble geometry (if this disabled method were used):

```text
correlation products = 256(4-1) = 768
unique samples covered = 256(4) = 1024
observation duration = 1024 / 30 MHz = 34.133 us
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
| Preamble `hat f_p` | 768 | 256 samples | 1024 samples | +/-58.594 kHz | Disabled; acquisition gate only |
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
> before the FFT. CP CFO supplies the channel-CFO unwrap reference; `hat f_H`
> aligns pilot channels and data after FFT. Preamble CFO is disabled.

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
w_k = \max(10^{-12},|\hat{H}[k]|^2)
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

The fitted slope is the weighted covariance divided by weighted frequency
variance. The fit residual is unweighted across the 384 bins. The code defines
quality using coarse ZC metric, residual limit, and the pilot residual SNR:

```math
Q=\max(0,M_z)
\max\left(0,1-\frac{\epsilon_{rms}}{\max(10^{-6},\epsilon_{max})}\right)
\operatorname{clip}\left(\frac{\mathrm{snr}+10}{40},0,1\right).
```

`valid` is set by `phase_residual <= max_residual_rms` only.
It does not enforce CRC, an SNR threshold, or a half-sample delay bound.
`coarse_delay=(coarse_peak-frame_start)/Fs` is the configured internal
frame-to-ZC offset, not a propagation-delay estimate.

## Integer Timing and Fine Delay

The detector-selected frame start sets the FFT window. For effective arrival
delay `tau_arrival` and the corresponding coarse timing selection
`tau_selected`, the remaining phase slope represents approximately

```math
\tau_{fine}\simeq\tau_{arrival}-\tau_{selected}.
```

Integer delay is removed by shifting the extracted time window, not by
subtracting a large delay inside the regression. A one-sample change of the
selected boundary can change the fine estimate by roughly the opposite sample.
Multipath, RF group delay, CFO/ICI and FFT-window errors can spoil this model;
the implementation does not guarantee `fine_delay` lies within +/-0.5 sample.

The live SS-TWR solver calculates

```math
\mathrm{RTT}=T_4-T_1-T_{reply},\qquad
R_{integer}=\frac{c}{2}(\mathrm{RTT}-T_{cal}),
```

with `T_cal = 1.159952546762614e-05 s` hard-coded in the solver.
It publishes this as both `range_m` and `integer_range_m`. The RESPONSE
fine-delay contribution `c * fine_delay / 2` is logged separately.
The code does not receive the responder's POLL fine estimate in the control
payload, so it cannot form a live bidirectional fine correction.

For matched directional logs, the offline correction under consistent
timestamp/reference conventions is

```math
R_{corrected}=R_{integer}
+\frac{c}{2}(\tau_{IR,fine}+\tau_{RI,fine}).
```

Here IR is POLL reception at the responder and RI is RESPONSE reception at
the initiator. Each link is estimated separately; the live receiver does not
jointly fit both channels. Historical offline measurements are recorded in
README.md, not evidence of an implemented online fusion block.

At 30 MS/s, one sample is 33.333 ns and one integer RTT range increment is
approximately 4.997 m. A phase fit can resolve a residual below one sample,
but low standard deviation does not establish absolute accuracy. Full-band
pilot support spans approximately 30 MHz; the rough 1/bandwidth scale is not
a hard lower bound on delay precision for a known waveform.

## Implementation References

| Processing | Source |
|---|---|
| Waveform construction and section scaling | [prs_frame_builder.cc](lib/DSP/prs_frame_builder.cc) |
| Bin comb, scrambler, Golay and Gold indexing | [mc_ds_prs.h](lib/DSP/mc_ds_prs.h) |
| Packet serialization and CRC | [prs_payload_codec.cc](lib/DSP/prs_payload_codec.cc) |
| Local / FFT-ZC acquisition | [local detector](lib/DSP/prs_frame_detector_impl.cc), [FFT-ZC detector](lib/DSP/prs_fft_zc_frame_detector_impl.cc) |
| CP CFO, frequency axes, phase unwrap | [prs_receiver_utils.cc](lib/DSP/prs_receiver_utils.cc) |
| FFT, channel/payload, phase fit | [FFT receiver](lib/DSP/prs_fft_receiver_impl.cc), [channel estimator](lib/DSP/prs_channel_estimator_impl.cc), [phase estimator](lib/DSP/prs_phase_slope_estimator_impl.cc) |
| Timestamp range | [prs_ssrtt_solver_impl.cc](lib/USRP/prs_ssrtt_solver_impl.cc) |

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
| **Total** | **104 information + 16 CRC bits** |  | **33616 samples** | **0-33615** |

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
In the earlier preamble-CFO variant, the estimate was initially converted to phase increment
`\hat{\omega}=2\pi\hat{f}_e/F_s` and applied to every payload sample. If this
decode failed CRC, a CP-CFO retry could be attempted. Neither operation is
part of the current OFDM payload decoder.

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

This historical decoder had no payload-metric acceptance threshold. A high
`payload_metric` does not override a CRC failure.

The decision metric measures consistency, not correctness. A coherently
combined bit can still have a high margin and the wrong sign; CRC remains the
validity check.

### Historical Energy and CRC Interpretation

At unchanged per-sample amplitude, increasing from 5 to 280 samples per bit
transmits 56 times as much energy per information bit:

```math
10\log_{10}(280/5) = 17.48\ \text{dB}
```

This is additional transmitted energy and airtime, not coding gain at fixed
`Eb/N0`.

The attenuation results and their interpretation are recorded in README.md's
Experimental History. They predate the current integrated payload and scaling.

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
however, identify the historical BPSK payload/CRC stage as the first observed
digital failure layer before acquisition loss.

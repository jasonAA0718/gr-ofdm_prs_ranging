# MC-DS-CDMA Prototype Migration Plan for `gr-ofdm_prs_ranging`

Updated: 2026-09-05

## Purpose

This document defines the first implementation step for converting the current standalone BPSK payload + OFDM PRS waveform into a simplified MC-DS-CDMA OFDM waveform.

The implementation goal is deliberately limited:

```text
Current:

[zero guard]
[repeated QPSK acquisition preamble]
[coarse ZC sync]
[standalone BPSK payload]
[16-symbol OFDM Golay PRS]
[tail guard]

Target prototype:

[zero guard]
[repeated QPSK acquisition preamble]
[coarse ZC sync]
[8-symbol MC-DS-CDMA OFDM block]
[tail guard]
```

This first prototype must:

1. Remove the standalone 33616-sample BPSK payload section.
2. Preserve the existing repeated QPSK acquisition preamble.
3. Preserve the existing coarse ZC synchronization section.
4. Reduce the OFDM block from 16 symbols to 8 symbols.
5. Put the existing 120-bit packet payload into OFDM data bins.
6. Fill the remaining OFDM bins with Golay pilots using a 7-pilot / 1-data pattern.
7. Spread the complete OFDM frequency-domain vector across 8 OFDM symbols using an 8-chip PRN/Gold code.
8. Preserve the existing SS-TWR protocol and metadata semantics as much as possible.
9. Do not add multi-responder grouping, simultaneous multi-code reception, or multi-anchor scheduling yet.

The purpose of this version is only to validate that the current ranging system can operate with a full MC-DS-CDMA-style OFDM block before multi-responder extensions are added.

---

# 1. Existing Signal State

The current burst is:

```text
[zero guard]
[repeated QPSK acquisition preamble]
[coarse ZC sync]
[BPSK packet payload]
[OFDM Golay PRS-like symbols]
[tail guard]
```

Current default/reference OFDM parameters:

```text
fft_len      = 1024
cp_len       = 128
active_bins  = 1024
prs_symbols  = 16
```

The current standalone BPSK payload occupies:

```text
33616 samples
```

and contains:

```text
packet_type
poll_frame_id
response_frame_id
reply_delay_samples
CRC16
```

with a total of:

```text
120 protected bits
```

Current payload field allocation:

| Field | Bits |
|---|---:|
| packet_type | 8 |
| poll_frame_id | 32 |
| response_frame_id | 32 |
| reply_delay_samples | 32 |
| CRC16 | 16 |
| Total | 120 |

The current implementation serializes integer fields least-significant-bit first.

The current CRC behavior must remain unchanged.

The current production OFDM PRS uses:

```text
Golay A on even OFDM symbols
Golay B on odd OFDM symbols
```

over all 1024 native FFT bins.

The transmitter uses native FFT indexing:

```text
fft_bin = 0 ... 1023
```

with no `fftshift` before IFFT.

The receiver later reorders bins for monotonic signed-frequency phase-slope processing, but the transmitter mapping itself remains native FFT order.

---

# 2. Target MC-DS-CDMA Parameters

The first prototype is fixed to:

```text
NFFT = 1024
NCP  = 128

MC-DS spreading length M = 8

OFDM symbols per burst = 8

spreading code type = PRN / Gold

data modulation = BPSK

pilot/data mapping:
    7 Golay pilot bins
    1 BPSK data bin
    repeated over all 1024 native FFT bins
```

Therefore:

```text
1024 / 8 = 128 data-bin positions
```

The existing payload contains 120 bits, so the remaining 8 data positions are fixed padding.

---

# 3. Exact Frequency-Domain Bin Mapping

Use native FFT indexing.

For:

```text
k = 0 ... 1023
```

define:

```text
if (k % 8) <= 6:
    bin k is a Golay pilot bin

if (k % 8) == 7:
    bin k is a BPSK data bin
```

Exact pattern:

```text
bin 0   pilot
bin 1   pilot
bin 2   pilot
bin 3   pilot
bin 4   pilot
bin 5   pilot
bin 6   pilot
bin 7   data[0]

bin 8   pilot
...
bin 14  pilot
bin 15  data[1]

...

bin 1016 ... 1022 pilot
bin 1023            data[127]
```

Counts:

```text
pilot bins = 896
data bins  = 128
```

Of the 128 data positions:

```text
data[0 ... 119]   = existing serialized 120-bit payload
data[120 ... 127] = fixed zero padding
```

The 8 padding bits:

```text
must not be included in CRC
must not alter the existing payload field structure
must be ignored by the decoder
```

---

# 4. BPSK Mapping

Keep the existing binary-to-BPSK convention:

```text
bit 0 -> -1 + j0
bit 1 -> +1 + j0
```

Padding bits are logical zero:

```text
padding bit 0 -> -1 + j0
```

The standalone BPSK time-domain repetition code is removed.

There is no longer:

```text
280 samples per bit
```

The payload is now carried directly in 120 OFDM data bins.

Reliability in the first prototype comes from:

```text
MC-DS spreading across 8 OFDM symbols
+
frequency-domain channel estimation/equalization
+
CRC validation
```

Do not add a new FEC scheme in this revision.

---

# 5. Golay Pilot Mapping

Retain the existing compiled Golay A/B pilot tables.

Do not regenerate random pilots.

Do not change the source-of-truth Golay table.

Continue using:

```text
Golay A on even OFDM symbol index
Golay B on odd OFDM symbol index
```

For OFDM symbol `m`:

```text
m = 0,2,4,6 -> Golay A
m = 1,3,5,7 -> Golay B
```

Only the pilot bins use the Golay table entry.

For a pilot bin `k`:

```math
S_m[k] = G_m[k]
```

where `G_m[k]` is the existing Golay A/B value for the same native FFT bin.

For a data bin `k`:

```math
S_m[k] = D[k]
```

where `D[k]` is the BPSK value corresponding to the assigned payload or padding bit.

The payload vector is held constant across the 8 spreading symbols.

---

# 6. MC-DS Spreading Model

The complete 1024-bin OFDM vector is spread.

This includes:

```text
896 Golay pilot bins
120 payload bins
8 padding bins
```

Let:

```math
c[m], \quad m = 0,\ldots,7
```

be the selected 8-chip PRN/Gold spreading sequence.

For each OFDM symbol:

```math
X_m[k] = c[m] S_m[k],
\qquad
k=0,\ldots,1023
```

where:

```math
S_m[k] =
\begin{cases}
G_m[k], & k \bmod 8 \in \{0,\ldots,6\}\\
D[q],   & k \bmod 8 = 7
\end{cases}
```

and:

```math
q = \left\lfloor k/8 \right\rfloor
```

This means every OFDM symbol carries the same 120-bit payload and the same 8 padding bits, while the Golay pilot pattern still alternates A/B.

The complete OFDM symbol is multiplied by one spreading chip `c[m]`.

Because IFFT is linear:

```math
IFFT\{c[m]S_m[k]\}
=
c[m]\,IFFT\{S_m[k]\}
```

so the spreading code also appears as one common complex sign/phase factor per OFDM symbol in the time domain.

This is symbol-domain direct-sequence spreading.

It is not the earlier rejected scheme where each individual OFDM time sample is expanded into many high-rate chips.

Therefore the original:

```text
NFFT = 1024
NCP  = 128
sample rate
subcarrier spacing
```

remain unchanged.

---

# 7. PRN / Gold Code Requirements

The first version uses one fixed 8-chip spreading sequence.

Do not implement multi-responder code assignment yet.

Requirements:

1. Code length must be exactly 8 chips for this prototype.
2. The same code must be used by transmitter and receiver.
3. The code should be deterministic and fixed by configuration or compile-time definition.
4. The spreading code must not be regenerated differently between frames.
5. Initial acquisition must not depend on this code.
6. The code is only used for the 8-symbol OFDM block.

Implementation note:

A conventional Gold-code family is usually defined for lengths of the form:

```text
2^n - 1
```

so an exact length-8 "Gold code" needs an explicit design choice.

For the first prototype, do not silently invent a standards-based Gold sequence.

Instead, implement an explicit deterministic 8-chip PRN sequence and name it clearly in code, for example:

```text
mc_ds_prn_code
```

If the existing project already has a preferred PRN/Gold generator suitable for an 8-chip sequence, reuse it.

Otherwise use a fixed documented bipolar sequence:

```text
c[m] in {+1, -1}
```

and keep the implementation structured so that the code can later be replaced by a proper multi-user code family.

Do not call an arbitrary 8-chip bipolar vector "Gold code" in comments unless it is actually generated from a documented Gold-code construction.

---

# 8. Acquisition and Coarse Synchronization

Do not change the current acquisition chain in this revision.

Keep:

```text
[repeated QPSK acquisition preamble]
[coarse ZC sync]
```

The MC-DS spreading code does not replace the initial acquisition mechanism.

Reason:

The 8-chip MC-DS code varies once per OFDM symbol.

It does not appear as a contiguous 8-sample code sequence in the raw ADC stream.

The receiver should therefore continue to locate the burst using:

```text
repeated QPSK preamble detection
+
existing coarse ZC correlation/refinement
```

After the frame boundary is known, the receiver extracts the 8 OFDM symbols and performs MC-DS despreading.

---

# 9. Updated Burst Geometry

Old:

```text
[zero guard]
[QPSK preamble]
[coarse ZC]
[33616-sample BPSK payload]
[16 × (1024+128) OFDM]
[tail guard]
```

New:

```text
[zero guard]
[QPSK preamble]
[coarse ZC]
[8 × (1024+128) MC-DS OFDM]
[tail guard]
```

The standalone payload length is removed entirely.

New OFDM section length:

```math
N_{\text{OFDM,new}}
=
8(1024+128)
=
9216 \text{ samples}
```

Old OFDM section length:

```math
N_{\text{OFDM,old}}
=
16(1024+128)
=
18432 \text{ samples}
```

Therefore the frame becomes substantially shorter because both:

```text
33616 standalone payload samples
```

and:

```text
8 of the former 16 OFDM symbols
```

are removed.

Any frame-length constants, extraction offsets, expected response-window sizes, and QA synthetic-frame lengths must be updated.

---

# 10. Receiver Processing Order

The receiver should use this conceptual order:

```text
frame detector
    |
    +-- repeated preamble acquisition
    +-- coarse ZC synchronization
    |
extract 8 MC-DS OFDM symbols
    |
remove CP
    |
1024-point FFT for each symbol
    |
known spreading-code despreading
    |
pilot processing
    |
channel estimation
    |
phase-slope fine delay
    |
data-bin equalization
    |
BPSK payload decode
    |
CRC validation
```

The exact placement of despreading relative to Golay removal must preserve the known per-symbol A/B pattern.

For pilot bin `k`:

```math
Y_m[k]
=
H[k]\,c[m]\,G_m[k] + W_m[k]
```

The recommended estimator is:

```math
\hat H[k]
=
\frac{1}{M}
\sum_{m=0}^{M-1}
Y_m[k]\,
c^*[m]\,
G_m^*[k]
```

because current Golay values have unit magnitude.

Equivalently:

```math
\hat H[k]
=
\frac{1}{M}
\sum_m
\frac{Y_m[k]}{c[m]G_m[k]}
```

for nonzero unit-magnitude spreading/pilot values.

This preserves the full frequency-dependent channel:

```math
\hat H[k]
```

needed by the phase-slope fine-delay estimator.

Do not collapse the 896 pilot bins into one scalar correlation result.

The phase-slope stage still requires phase versus frequency.

---

# 11. Data Recovery

For data bins:

```math
Y_m[k]
=
H[k]\,c[m]\,D[k] + W_m[k]
```

where the BPSK symbol `D[k]` is constant across all 8 OFDM spreading symbols.

First despread:

```math
Z[k]
=
\frac{1}{8}
\sum_{m=0}^{7}
Y_m[k]c^*[m]
```

giving approximately:

```math
Z[k] \approx H[k]D[k]
```

The data symbol must then be channel-equalized.

Because a data bin itself is not a Golay pilot, the receiver needs a channel estimate at that frequency.

First prototype recommendation:

interpolate the complex channel estimate from neighboring pilot bins.

Since the data pattern is:

```text
7 pilot bins + 1 data bin
```

every data bin has nearby pilot support.

For data bin:

```text
k = 8q + 7
```

estimate its channel from adjacent pilot estimates.

Recommended simple first implementation:

```text
use the nearest pilot bin on the left
```

or:

```text
linear complex interpolation between nearest pilot bins on both sides
```

Prefer linear interpolation if implementation cost is small.

Boundary case:

```text
k = 1023
```

has no pilot bin to the right in native index order.

For the first version use the nearest left pilot estimate for that final data bin, or implement circular/native-FFT-aware interpolation only if the existing channel estimator already treats the frequency boundary correctly.

Do not invent a complicated interpolation filter in the first prototype.

Then:

```math
\hat D[k] = Z[k]/\hat H_{\text{data}}[k]
```

BPSK hard decision:

```text
real(D_hat) >= 0 -> bit 1
real(D_hat) < 0  -> bit 0
```

Collect only the first 120 decoded data bits.

Ignore decoded bits 120...127.

Reconstruct the existing fields exactly as before.

Run the existing CRC16 logic.

Set:

```text
frame_id_valid = 1
```

only if CRC passes.

---

# 12. CFO Handling

Do not remove the existing CFO framework.

The project currently has:

```text
prs_cp_cfo_hz
prs_channel_cfo_hz
```

and inter-symbol channel phase alignment.

Changing from 16 to 8 OFDM symbols changes the observation length but does not eliminate the need for CFO correction.

The MC-DS despreading code itself introduces a known symbol-to-symbol sign/phase sequence.

Therefore any inter-symbol CFO estimator must remove or account for the PRN spreading chip before estimating common phase evolution.

Otherwise code-chip transitions can be misinterpreted as CFO.

For example, before adjacent-symbol channel CFO estimation, remove:

```math
c[m]
```

and the known Golay A/B pilot modulation.

Conceptually:

```math
\tilde H_m[k]
=
Y_m[k] c^*[m] G_m^*[k]
```

Then estimate CFO from the phase evolution of:

```math
\tilde H_m[k]
```

rather than directly from the raw per-symbol channel values that still contain the spreading code.

The exact existing CFO implementation should be modified minimally.

Do not redesign all CFO estimation in this task.

The key rule is:

```text
known PRN spreading phase must not contaminate the CFO estimator
```

---

# 13. Phase-Slope Fine Delay

The phase-slope estimator should continue to operate on the despread and averaged pilot-channel vector.

Only pilot bins are valid direct channel observations.

Therefore the phase-slope estimator must not assume 1024 valid pilot bins anymore.

It should receive or derive the 896 pilot frequency positions:

```text
all k where k % 8 != 7
```

Use the actual signed frequency corresponding to each pilot bin.

Do not insert fake channel values for data bins into the phase regression unless those values are explicitly interpolated and intentionally supported.

Preferred first prototype:

```text
phase-slope regression uses only the 896 true pilot bins
```

This still preserves almost the full 1024-bin frequency span, so fine-delay performance should remain close to the existing full-band design, subject to the new pilot pattern and reduced symbol count.

---

# 14. Payload Semantics

Do not change SS-TWR packet semantics.

POLL:

```text
packet_type          = POLL
poll_frame_id        = new initiator poll ID
response_frame_id    = 0
reply_delay_samples  = 0
CRC16                = existing calculation
```

RESPONSE:

```text
packet_type          = RESPONSE
poll_frame_id        = echoed POLL ID
response_frame_id    = new response ID
reply_delay_samples  = configured responder delay
CRC16                = existing calculation
```

The only change is where those 120 bits are physically carried.

Old:

```text
standalone repeated-sample BPSK payload
```

New:

```text
128 interleaved OFDM data bins
```

---

# 15. SS-TWR Protocol

Do not change the SS-TWR transaction.

Keep:

```text
T1: Initiator TX POLL
T2: Responder RX POLL
T3: Responder TX RESPONSE after reply delay
T4: Initiator RX RESPONSE
```

Keep the existing range formula and calibration behavior unless a frame-length-dependent constant must be updated elsewhere.

Do not add:

```text
DS-TWR
FINAL packet
multi-anchor scheduler
multi-responder simultaneous response
```

in this revision.

---

# 16. Multi-Responder Features Explicitly Deferred

Do not implement yet:

```text
anchor groups
two responders transmitting simultaneously
multiple PRN codes
PRN code assignment by responder ID
multiple despreading branches
multi-code acquisition
multi-anchor positioning
WLS position solver changes
TDMA/CDMA grouping scheduler
```

The first target is a single-link functional regression:

```text
Initiator <-> one Responder
```

using the new MC-DS-CDMA OFDM payload/ranging block.

---

# 17. Suggested Code Changes

Inspect the current project before editing.

Likely affected areas include:

```text
lib/DSP/prs_frame_builder.cc
lib/DSP/prs_frame_detector_impl.cc
lib/DSP/prs_fft_receiver_impl.cc
lib/DSP/prs_channel_estimator_impl.cc
lib/DSP/prs_phase_slope_estimator_impl.cc
payload serialization/CRC helpers
GNU Radio block constructors only if necessary
QA tests
example GRC flowgraphs if frame-length parameters are explicit
```

The exact filenames must be verified from the repository.

Do not rename public blocks unnecessarily.

Do not introduce a new block unless the current architecture makes the change impossible to keep clean.

Prefer extending the existing frame builder and receiver chain.

---

# 18. Backward Compatibility

Where possible:

1. Keep existing public constructor arguments.
2. Append new parameters only if necessary.
3. Provide defaults.
4. Avoid breaking Python bindings or GRC files unnecessarily.
5. If `prs_symbols` is currently configurable, set the relevant example/default for this prototype to 8 instead of hard-coding 8 throughout the implementation.
6. If a new `mc_ds_enabled` option is introduced, keep the old mode available only if doing so does not make the implementation significantly more complex.

For this project branch, functionality of the new waveform is more important than maintaining obsolete standalone-payload behavior, but public API breakage should still be minimized.

---

# 19. Metadata

Preserve current important metadata fields where their meaning remains valid.
Remove preamble_cfo_hz

Examples include:

```text
packet_type
poll_frame_id
response_frame_id
reply_delay_samples
frame_id_valid
payload_metric
prs_cp_cfo_hz
prs_cp_cfo_coherence
selected_cfo_hz
prs_channel_cfo_hz
residual_cfo_hz
channel_coherence
phase_slope_rad_per_hz
fine_delay_s
fine_delay_samples
```

If useful, add:

```text
mc_ds_enabled
mc_ds_code_length
mc_ds_code_id
mc_ds_despread_metric
```

but do not add metadata merely for completeness.

For this first single-code prototype, `mc_ds_code_id` may be fixed.

---

# 20. Payload Metric

The current payload metric was designed around:

```text
16-sample known BPSK reference
+
280-sample coherent combining per bit
```

That metric is no longer mathematically appropriate once the standalone BPSK payload is removed.

Do not silently reuse the old formula.

Define a new payload quality metric based on OFDM data symbols.

A simple first version may use:

```text
average normalized BPSK decision margin
```

after channel equalization:

```math
m_q =
\frac{|\Re\{\hat D_q\}|}
{|\hat D_q|+\epsilon}
```

and:

```math
payload\_metric
=
\frac{1}{120}
\sum_{q=0}^{119} m_q
```

Keep:

```text
CRC result
```

as the authoritative payload validity condition.

A high payload metric must not override CRC failure.

---

# 21. Scaling and PAPR

The current waveform has explicit section RMS scaling.

Because the standalone BPSK payload is removed, update the section scaling logic accordingly.

Do not apply the old payload-section RMS normalization to a section that no longer exists.

The new MC-DS OFDM symbols contain:

```text
Golay pilot bins
+
BPSK data bins
+
PRN spreading
```

The old approximately 3 dB Golay PAPR result is no longer guaranteed.

Therefore add or update QA to measure:

```text
minimum PAPR
mean PAPR
maximum PAPR
```

for the new 8-symbol MC-DS waveform.

Do not assume that Golay low-PAPR properties remain unchanged after inserting 128 data bins.

The PRN chip itself is a common symbol-level sign/phase factor and therefore does not change the PAPR within a given OFDM symbol.

The data-bin replacement pattern can change PAPR.

---

# 22. Frame Detector Updates

The frame detector currently knows where the standalone BPSK payload and OFDM section are located.

All sample offsets after coarse ZC must be updated.

Old layout:

```text
coarse ZC
    |
33616-sample payload
    |
OFDM
```

New layout:

```text
coarse ZC
    |
OFDM starts immediately
```

Therefore update:

```text
frame length
OFDM start offset
expected complete-frame boundary
response acquisition window assumptions
synthetic QA frame construction
payload decode location
```

The frame detector no longer decodes a standalone time-domain BPSK payload.

Payload decoding should occur after OFDM FFT/despread/channel-equalization.

If the current architecture requires `packet_type` or IDs inside the frame detector before FFT processing, refactor the message flow carefully.

Do not fake payload values in the detector.

The new correct dependency is:

```text
frame detector
    -> raw accepted frame / timing metadata
FFT / MC-DS receiver
    -> payload decode + channel
```

If moving payload decode downstream affects responder triggering, preserve the responder behavior by forwarding the decoded payload metadata to the SS-TWR responder only after CRC-valid OFDM payload decoding.

---

# 23. Receiver Architecture Concern

The existing responder currently decides whether to transmit a RESPONSE only after:

```text
POLL payload CRC valid
packet_type == POLL
```

Because payload decoding moves later in the chain, verify that:

```text
frame detector
-> FFT
-> MC-DS despread
-> channel/data equalization
-> payload decode
-> SS-TWR responder
```

still meets the configured reply-delay timing.

Do not reduce the reply delay in this revision.

If needed, retain or increase the current reply delay during prototype testing.

Do not bypass CRC validation merely to preserve timing.

---

# 24. QA Requirements

The implementation is not complete until focused QA covers the new waveform.

Minimum tests:

## 24.1 Frequency Mapping

Verify exactly:

```text
896 pilot bins
128 data bins
```

and:

```text
k % 8 == 7
```

for every data bin.

## 24.2 Payload Mapping

Verify:

```text
120 payload bits
8 zero padding bits
```

and correct LSB-first serialization.

## 24.3 CRC

Verify identical CRC generation and validation to the old packet format.

## 24.4 MC-DS Spreading

For every OFDM symbol:

```math
X_m[k] = c[m]S_m[k]
```

must hold.

## 24.5 Despread Recovery

In an ideal/noiseless channel:

```text
TX
-> IFFT
-> CP
-> RX
-> CP removal
-> FFT
-> despread
```

must recover the original:

```text
pilot values
payload bits
```

with zero CRC errors.

## 24.6 Golay A/B

Verify:

```text
m even -> Golay A on pilot bins
m odd  -> Golay B on pilot bins
```

## 24.7 AWGN

Add a moderate-SNR test confirming reliable payload recovery.

Do not choose an unrealistically high SNR only.

## 24.8 CFO

Verify that the known MC-DS code does not bias the existing inter-symbol CFO estimate after code removal.

## 24.9 Fractional Delay

Verify the phase-slope fine-delay estimator still recovers a known injected fractional delay using only the 896 pilot bins.

## 24.10 Frame Length

Verify the new frame length exactly matches the new geometry.

## 24.11 PAPR

Record new OFDM PAPR statistics.

---

# 25. Experimental Validation Sequence

After build and QA pass, validate in this order:

```text
1. software loopback / synthetic frame
2. wired B210 test
3. single-link initiator/responder SS-TWR
4. payload CRC reliability
5. CFO comparison
6. fine-delay stability
7. ranging RMS
```

Do not add multi-responder CDMA until the single-link waveform is stable.

Important comparisons against the old waveform:

```text
frame duration
packet success rate
payload CRC failure rate
preamble CFO
PRS CP CFO
channel CFO
channel coherence
fine_delay_samples
range RMS
PAPR
```

---

# 26. First Prototype Success Criteria

The prototype is successful if all of the following are true:

1. Standalone 33616-sample BPSK payload is completely removed.
2. Current 120-bit packet data is recovered from OFDM data bins.
3. CRC behavior is unchanged.
4. 8-symbol PRN spreading and despreading work.
5. Acquisition preamble and coarse ZC continue to work unchanged.
6. Responder only replies to a valid CRC-confirmed POLL.
7. Initiator correctly identifies a valid RESPONSE.
8. The phase-slope estimator still returns a usable fine delay.
9. Existing SS-TWR produces valid range output.
10. Wired test does not introduce a new systematic frame-acquisition failure.

---

# 27. Explicit Non-Goals

Do not implement any of the following in this task:

```text
multi-responder grouping
two simultaneous responders
TDMA group scheduler
per-anchor PRN assignment
Walsh comparison
ZC-vs-PRN comparison
pilot-only spreading
higher data-rate payload
QPSK payload
FEC
DS-TWR
position solver changes
fine-delay tracking
multi-burst averaging
```

Those are later stages.

---

# 28. Recommended Implementation Order

Use small, reviewable steps.

## Step 1

Create reusable payload serialization helpers that output the existing 120-bit payload as a bit vector.

Verify CRC and bit order.

## Step 2

Change the OFDM frame builder:

```text
16 symbols -> 8 symbols
```

Implement:

```text
7 pilot + 1 data
```

mapping.

Do not add spreading yet.

Verify a noiseless receiver can recover the payload.

## Step 3

Add the fixed 8-chip PRN sequence.

Apply:

```math
X_m[k] = c[m]S_m[k]
```

to all 1024 bins.

## Step 4

Modify the FFT/channel path to remove the known spreading code.

Verify pilot recovery.

## Step 5

Implement OFDM payload channel equalization and BPSK decode.

Verify CRC.

## Step 6

Remove the old standalone BPSK payload construction and decoding path.

Update all frame offsets and frame lengths.

## Step 7

Update CFO handling so spreading-code phase does not contaminate inter-symbol CFO estimation.

## Step 8

Update phase-slope processing to use only the 896 true pilot bins.

## Step 9

Update QA, PAPR tests, examples, and profiling flowgraphs.

## Step 10

Run wired SS-TWR validation.

---

# 29. Engineering Constraints

1. Do not modify the repeated QPSK acquisition preamble.
2. Do not modify the coarse ZC acquisition design.
3. Do not change the SS-TWR packet field definitions.
4. Do not change CRC16 semantics.
5. Keep native FFT indexing on TX.
6. Do not add fftshift to the transmitter.
7. Keep Golay A/B alternating on pilot bins.
8. Use exactly 8 MC-DS OFDM symbols.
9. Use exactly 7 pilot bins followed by 1 data bin.
10. Use the complete 1024-bin vector for spreading.
11. Do not add multi-responder functionality yet.
12. Do not call an arbitrary 8-chip sequence a Gold code without documenting how it is generated.
13. Preserve current CFO metadata where possible.
14. Preserve current fine-delay and SS-TWR output fields where possible.
15. Build and run focused tests after every meaningful step.

---

# 30. Expected Final Signal

The first prototype target is:

```text
[zero guard]

[repeated QPSK acquisition preamble]

[coarse ZC sync]

[MC-DS OFDM symbol 0]
    PRN chip c[0]
    Golay A pilot bins
    120 BPSK payload bits + 8 padding bits on data bins

[MC-DS OFDM symbol 1]
    PRN chip c[1]
    Golay B pilot bins
    same payload/data vector

...

[MC-DS OFDM symbol 7]
    PRN chip c[7]
    Golay B pilot bins
    same payload/data vector

[tail guard]
```

The receiver must perform:

```text
acquisition
-> coarse ZC sync
-> extract 8 OFDM symbols
-> FFT
-> remove PRN spreading
-> estimate channel from 896 Golay pilot bins
-> estimate fine delay from pilot-channel phase slope
-> interpolate channel onto 128 data bins
-> equalize data
-> decode first 120 BPSK bits
-> ignore 8 padding bits
-> verify CRC
-> continue existing SS-TWR logic
```

This is the complete scope for the first MC-DS-CDMA prototype.

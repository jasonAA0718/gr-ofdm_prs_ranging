# Project Memory: OFDM PRS / SS-TWR Ranging

This file is a handoff note for future AI agents and developers working on
`/home/cnsl/Desktop/gr-ofdm_prs_ranging`. 
It records the current project architecture, recent implementation state, known problems, and recommended future work.

## Table of Contents

- [Engineering Rules for Future Agents](#engineering-rules-for-future-agents)
- [Project Goal](#project-goal)
- [Current OFDM PRS Burst](#current-ofdm-prs-burst)
- [Current SS-TWR State](#current-ss-twr-state)
- [Payload State](#payload-state)
- [Changes on 2026-07-24](#changes-on-2026-07-24)
- [Current Receiver Chain](#current-receiver-chain)
- [Receiver Data-Path Efficiency](#receiver-data-path-efficiency)
- [Decoupled RX Timing](#decoupled-rx-timing)
- [Recent Coarse ZC Root / Channel Separation Support](#recent-coarse-zc-root--channel-separation-support)
- [Known Experimental State](#known-experimental-state)
- [Phase-Slope Estimator Status](#phase-slope-estimator-status)
- [Known Test Status](#known-test-status)
- [Installation Reminder](#installation-reminder)
- [2026-07-27 Acquisition Attempt Logging](#2026-07-27-acquisition-attempt-logging)
- [2026-07-28 BPSK Payload Documentation](#2026-07-28-bpsk-payload-documentation)
- [2026-07-28 Golay PRS and Section Scaling](#2026-07-28-golay-prs-and-section-scaling)
- [2026-08-05 PRS CFO Refinement and Phase Diagnostics](#2026-08-05-prs-cfo-refinement-and-phase-diagnostics)
- [Future Work](#future-work)
  - [1. Computational Cost and Processing Latency](#1-computational-cost-and-processing-latency)
  - [2. End-to-End Update-Rate Budget](#2-end-to-end-update-rate-budget)
  - [3. Integrate Communication Payload into OFDM](#3-integrate-communication-payload-into-ofdm)
  - [4. Multi-Responder Observation Separation](#4-multi-responder-observation-separation)
  - [5. Fine-Ranging Validation and Calibration](#5-fine-ranging-validation-and-calibration)
  - [6. Positioning Algorithms](#6-positioning-algorithms)

## Engineering Rules for Future Agents

- Do not modify the repeated QPSK acquisition preamble unless explicitly asked.
- Keep constructor changes backward-compatible by appending parameters with
  defaults.
- After changing public block constructors, update:

```text
include headers
lib impl headers
lib impl cc
Python bindings
GRC YAML
examples
QA tests
```

- Build after each meaningful change.
- Run focused tests after changing receiver/transmitter behavior.


## Project Goal

The long-term goal is real-time long-distance positioning using multiple
responders/anchors. The current architecture is moving toward:

```text
Initiator
  -> sends POLL PRS burst

Responder/Anchor i
  -> detects POLL
  -> sends RESPONSE PRS burst after fixed reply delay

Initiator
  -> detects RESPONSE from each responder
  -> computes pseudorange/range per responder
  -> later feeds least-squares or Kalman positioning
```
## Current OFDM PRS Burst

The current burst is a custom OFDM/PRS-like signal:

```text
[zero guard]
[repeated QPSK acquisition preamble]
[coarse ZC sync]
[BPSK SS-TWR payload]
[OFDM PRS-like pilot symbols]
[tail guard]
```

Default/reference frame geometry:

```text
zero guard:          1000 samples
short preamble:      preamble_len * preamble_repeats
coarse ZC sync:      coarse_sync_len samples, historically 839
BPSK payload:        33616 samples
OFDM PRS block:      prs_symbols * (fft_len + cp_len)
tail guard:          1000 samples
```

Default OFDM parameters:

```text
fft_len      = 1024
cp_len       = 128
active_bins  = 1024
prs_symbols  = 16
pilot table  = lib/DSP/golay_prs_table.h
```

All native FFT bins are occupied:

```text
fft_bin 0 ... 1023
signed bins -512 ... +511
```

The fixed table is generated from `lib/DSP/golay_ofdm_1024x16.csv`. Even-numbered
symbols use Golay A and odd-numbered symbols use Golay B. The `seed` parameter
is retained for the repeated QPSK acquisition preamble, not for OFDM PRS.

The repeated acquisition preamble is deterministic QPSK and should remain
unchanged unless explicitly requested. It is the cheap first-stage detector.

The coarse ZC sync is the second-stage confirmation and channel-separation
sequence.

## Current SS-TWR State

The current system implements minimal two-message SS-TWR:

```text
T1: initiator TX POLL
T2: responder RX POLL
T3: responder TX RESPONSE after fixed reply_delay_samples
T4: initiator RX RESPONSE
```

Range computation currently uses timestamp SS-TWR:

```text
RTT = T4 - T1 - reply_delay
ToF = (RTT - calibration_delay) / 2
range_m = ToF * 299792458
```

There is no DS-TWR, no FINAL packet, no clock-skew correction, and no
positioning filter yet.

## Payload State

The packet payload is BPSK with CRC and repetition. It is currently reliable and
should be preserved.

Payload fields:

```text
packet_type
poll_frame_id
response_frame_id
reply_delay_samples
CRC-16
280 samples per information bit
```

`frame_id_valid=1` must only be set when CRC passes.

`recv_id` is a local receiver counter and is not the transmitted frame ID.

## Changes on 2026-07-24

The following performance and scheduling changes were completed without
changing the PRS frame purpose, signal-processing equations, metadata contract,
or SS-TWR range formula.

### Signal-Processing Efficiency

```text
lib/DSP/prs_receiver_utils.cc
    Added zero-copy access to PMT complex vectors.

lib/DSP/prs_frame_detector_impl.cc
    Replaced per-scheduler-call sample shifting with logical buffer drops and
    batched compaction.

lib/DSP/prs_fft_receiver_impl.cc
    Reused FFT input, output, and symbol buffers between frames.

lib/DSP/prs_channel_estimator_impl.cc
    Precomputed pilot reciprocals and calculated channel residual statistics
    in one input pass.

lib/DSP/prs_frame_builder.cc
    Replaced the quadratic transmitter DFT with GNU Radio's FFT backend.
```

At the default geometry, these changes avoid approximately 191 KB, 77 KB, and
4.8 KB of input copying per frame across the receiver stages. PRS construction
changed from approximately `O(16 * 1024^2)` to
`O(16 * 1024 * log2(1024))`.

### RX and TX Scheduling

A new `prs_rx_timekeeper` block was added in:

```text
include/gnuradio/ofdm_prs_ranging/prs_rx_timekeeper.h
lib/USRP/prs_rx_timekeeper_impl.h
lib/USRP/prs_rx_timekeeper_impl.cc
python/ofdm_prs_ranging/bindings/prs_rx_timekeeper_python.cc
grc/ofdm_prs_ranging_prs_rx_timekeeper.block.yml
```

It continuously consumes the UHD RX branch, tracks the most recent `rx_time`
tag and absolute sample offset, and converts a strobe into an explicit timed
trigger. `prs_timed_burst_source` now accepts zero or one stream input, so it
does not need to consume the RX stream when the trigger already contains
`tx_time_secs` and `tx_time_frac`.


### Scheduled Correlation Windows

`prs_frame_detector` now has an optional `tx_time_in` message port and these
backward-compatible parameters:

```text
time_gating=False
reply_delay_s=0.05
window_before_s=0.0002
window_after_s=0.004
```

When gating is enabled, a transmit-time message schedules:

```text
window_start = tx_time + reply_delay_s - window_before_s
window_end   = tx_time + reply_delay_s + window_after_s
```

The detector continues consuming all UHD samples and tracking `rx_time` while
disarmed. It skips sample buffering and preamble/coarse correlation outside the
window, and resets correlation state across inactive sample gaps.

The initiator connects `prs_timed_burst_source.tx_time_out` to both
`prs_frame_detector.tx_time_in` and `prs_ssrtt_solver.tx_time_in`. Its current
50 ms reply delay, 0.2 ms early margin, and 2 ms late window cover the expected
response from a target below 100 km, including the complete configured frame.
With the current 500 ms strobe period, correlation is active for about 2.2 ms,
or approximately 0.44% of each cycle.

The responder remains in continuous correlation mode because it cannot predict
the first poll without shared radio time or an established slot schedule.


### Initiator and Responder Text UI

The message-only `prs_text_ui` block was added for compact live terminal
status. It does not consume or copy the UHD sample stream.


Initiator display:

```text
[INITIATOR] RX RECEIVING | responses/polls 49/50 (98.0%) | loss 2.0% |
pending 0 | SNR 22.4 dB | range 84.25 m
```

Responder display:

```text
[RESPONDER] RX RECEIVING | replies/polls 50/50 (100.0%) | SNR 21.8 dB
```

Poll and response counts use unique `poll_frame_id` values. Initiator polls
remain pending until a matched response arrives or `response_timeout_s`
expires. `loss` counts only expired polls, while the response percentage uses
all transmitted polls. RX status becomes `NO SIGNAL` after
`stale_timeout_s` without a valid frame. SNR is the current channel-residual
estimate, and range remains subject to the configured SSRTT calibration delay.

The implementation and integration files are:

```text
python/ofdm_prs_ranging/prs_text_ui.py
python/ofdm_prs_ranging/qa_prs_text_ui.py
```


## Current Receiver Chain

The OFDM PRS receiver chain is:

```text
UHD Source
-> prs_frame_detector
-> prs_fft_receiver
-> prs_channel_estimator
-> prs_phase_slope_estimator
-> prs_ssrtt_solver or logger
```

`prs_frame_detector` does:

```text
1. repeated-preamble rolling metric
2. five-point coarse ZC correlation refinement at offsets -2...+2
3. frame extraction
4. rx_time preservation
5. payload decode
6. metadata publication
```

The repeated preamble is the continuous acquisition gate and uses `threshold`.
After that gate passes, the detector evaluates normalized ZC correlation at the
predicted boundary and offsets `-2`, `-1`, `+1`, and `+2`. It selects the
strongest valid offset, applies the separate `zc_threshold`, and corrects both
the frame start and coarse-sync index by the selected offset. If a
threshold-passing maximum lies at `-2` or `+2`, the five-point window is
recentered once before acceptance so a still-rising edge is not mistaken for a
local peak.

## Receiver Data-Path Efficiency

The receiver implementation keeps the signal-processing equations and PDU
interfaces unchanged while avoiding avoidable work in the hot path:

```text
PMT complex-vector inputs are read through const views instead of copied.
The frame-detector buffer drops samples logically and compacts in batches.
FFT and channel-estimation output buffers are reused between messages.
Golay pilot reciprocals are precomputed once.
Channel residual energy is computed from first- and second-order sums.
TX PRS symbols use GNU Radio's FFT backend instead of a quadratic reference DFT.
```

These changes reduce scheduler stalls but do not change the acquisition
thresholds, frame geometry, metadata, phase-slope model, or SS-TWR range
formula. Compare `O`/`L` counts, CPU load, valid frames per second, and
measurement statistics before changing signal parameters.

## Decoupled RX Timing

The timed burst source no longer needs to consume the continuous UHD RX stream.
This prevents a scheduled TX burst from temporarily backpressuring RX and
causing an overflow at the trigger period.

Initiator timing uses:

```text
UHD Source -> prs_frame_detector
UHD Source -> prs_rx_timekeeper
message_strobe -> prs_rx_timekeeper -> prs_timed_burst_source -> UHD Sink
```

`prs_rx_timekeeper` consumes its RX branch without copying samples. It tracks
the latest UHD `rx_time` tag and absolute sample offset, then converts each
strobe into a trigger containing explicit `tx_time_secs` and `tx_time_frac`.

The responder already calculates an explicit response TX time from the detected
POLL timestamp:

```text
UHD Source -> receiver chain -> prs_ssrtt_responder
prs_ssrtt_responder -> prs_timed_burst_source -> UHD Sink
```

`prs_timed_burst_source` accepts zero or one stream input for compatibility with
older flowgraphs. New SS-TWR flowgraphs must leave that stream input
unconnected.

### Time-Gated Correlation

The initiator frame detector can restrict acquisition work to the expected
response interval. Its `tx_time_in` message input accepts the
`prs_timed_burst_source` `tx_time_out` dictionary and schedules:

```text
window_start = tx_time + reply_delay_s - window_before_s
window_end   = tx_time + reply_delay_s + window_after_s
```

The 30 MHz initiator example uses a 50 ms responder delay, 0.2 ms early margin,
and 2 ms late window. The detector continues consuming every UHD sample and
tracking `rx_time` while disarmed, but it does not copy samples or run preamble
correlation outside the scheduled window. The responder remains in continuous
acquisition mode because it cannot predict the first poll without a shared time
or an established slot schedule.

## Recent Coarse ZC Root / Channel Separation Support

The repeated QPSK preamble was not changed.

Current one-channel separation plan:

```text
Initiator POLL TX root:       25
Responder RX detector root:   25
Responder RESPONSE TX root:   29
Initiator RX detector root:   29
Initiator RX channel_id:      1
```

For multiple responders later:

```text
Responder 1 RESPONSE root: 29, channel_id 1
Responder 2 RESPONSE root: 31, channel_id 2
Responder 3 RESPONSE root: 37, channel_id 3
Responder 4 RESPONSE root: 41, channel_id 4
```

Expected future initiator receive structure:

```text
UHD Source
  -> prs_frame_detector(coarse_zc_root=29, channel_id=1)
  -> prs_frame_detector(coarse_zc_root=31, channel_id=2)
  -> prs_frame_detector(coarse_zc_root=37, channel_id=3)
```

Then each branch feeds the same OFDM receiver/measurement path, and the
positioning layer consumes `channel_id`, responder ID, `range_m`, and quality.

## Known Experimental State

Outdoor LOS test at 48 m ground truth produced roughly 50.8 m mean range in one
accepted run. This is acceptable for the current SS-TWR/calibration state.

The main operational problem is measurement update rate and SDR stability:

```text
O = RX overflow
L = TX late command
```

## Phase-Slope Estimator Status

The receiver estimates fine delay from phase slope:

```text
H(k) = Y(k) / X(k)
phase unwrap
weighted linear regression phase vs frequency
tau = -slope / (2*pi)
```

But current `range_m` does not use this fine delay. Range is dominated by SS-TWR
timestamps and empirical calibration.

Phase slope is currently best treated as a diagnostic/quality metric because it
contains:

```text
propagation delay
RF group delay
cable/antenna delay
analog filter delay
residual coarse timing offset
residual CFO/SFO
multipath phase distortion
USRP channel phase behavior
```

Do not make phase slope the main range estimator until calibration and fusion
are designed.

See `signal.md` for the current signal equations.

## Known Test Status

Build command:

```bash
cmake --build build -j$(nproc)
```

Focused test command:

```bash
ctest --test-dir build -R 'prs_timed|prs_receiver' --output-on-failure
```

Current state:

```text
Build passes.
prs_timed tests pass.
New coarse_zc_root/channel_id tests pass.
New time-gated correlation test passes.
New initiator/responder text UI tests pass.
qa_prs_receiver still fails only at the older synthetic SS-RTT timestamp test.
```

Known failure:

```text
test_ssrtt_solver_computes_range_from_synthetic_timestamps
```

The failure is due to the current hard-coded SS-TWR calibration delay and is not
caused by coarse ZC root/channel separation.

## Installation Reminder

After source changes or git pull on another PC, rebuild and reinstall locally.
Do not copy installed `.so` files between PCs.

```bash
cmake --build build -j$(nproc)
sudo cmake --build build --target install
sudo ldconfig
```

Check import:

```bash
python3 -c "from gnuradio import ofdm_prs_ranging; print('OOT import OK')"
```





## 2026-07-27 Acquisition Attempt Logging

Add the `failure_reason` to distingulish the fail reason of ranging.

CSV Keeps `attempt_id`, `poll_frame_id`, and `response_frame_id` to find the loss packet rate.

`prs_timed_burst_source` publishes `attempt_id` with the TX tags and
`tx_time_out` metadata. 

The attempt ID is the poll frame ID, if the initiator could not find response.
The CSV would record this attempt ID with `failure_reason` `NO_PREAMBLE`

`prs_frame_detector` has a separate optional `event_out` message port. The
initiator and responder acquisition loggers use this port; `frame_out` remains
the successful detector output used by the FFT, channel estimator, UI, and
ranging blocks. This prevents a failed acquisition event from entering the DSP
chain.

The current `failure_reason` values are:

```text
NONE            frame detected and payload CRC valid
PAYLOAD_CRC     repeated preamble and ZC passed, payload CRC invalid
NO_PREAMBLE     gated attempt expired without crossing the repeated-preamble threshold
ZC_SYNC         repeated preamble crossed threshold but ZC confirmation did not
FRAME_BOUNDARY  ZC passed but a complete frame was not published before window expiry
UNKNOWN         logger received metadata without a detector failure reason
```

`NO_PREAMBLE` does not prove that RF samples were absent. It combines RF below
the usable level, wrong timing/window, and repeated-preamble threshold failure.
The detector cannot observe UHD overflow state, so it does not emit a distinct
UHD failure code.

## 2026-07-28 BPSK Payload Documentation

`signal.md` now documents the complete 33616-sample BPSK payload, its 16-sample
reference, 280 samples per information bit, all field offsets, bit order, CRC
coverage, and the different POLL/RESPONSE field meanings.

The attenuation observation is recorded as:

```text
about 75 dB: 50% packet/CRC failure
about 95 dB: acquisition failure
```

This identifies payload decode as the first observed digital failure stage.
The payload now uses CFO-corrected coherent combining over 280 samples per bit.
Section scaling has also changed, so this experiment should be repeated with
the updated waveform.

## 2026-07-28 Golay PRS and Section Scaling

The production OFDM PRS pilots now come only from the compiled table generated
from:

```text
lib/DSP/golay_ofdm_1024x16.csv
```

The table occupies all `1024` native FFT bins for all `16` symbols. TX writes
CSV `fft_bin` directly into the IFFT input without fftshift. RX extracts full
band in monotonic signed-frequency order (`512...1023`, then `0...511`) and
uses the same native-bin table entries for channel division.

Random-QPSK MT19937 generation remains only for the repeated acquisition
preamble. The channel-estimator seed argument is retained for API compatibility
but is unused.

Transmit scaling now uses:

```text
Payload RMS:      tx_amp
OFDM-symbol RMS:  tx_amp
Preamble RMS:     tx_amp + 3 dB
ZC RMS:           tx_amp + 3 dB
Final burst peak: <= 0.9 through one common scale
```

Dynamic BPSK payload contents are written at unit amplitude before section
normalization, so payload insertion can no longer undo the amplitude policy.

The QA measurement over the useful 1024-sample IFFT portion (before CP) is:

```text
Pilot                         Minimum PAPR   Mean PAPR   Maximum PAPR
Golay A/B, 1024 active bins      3.0062 dB    3.0062 dB      3.0062 dB
Old Random-QPSK, 600 bins        7.4209 dB    8.9281 dB     10.4139 dB
```

The Random-QPSK implementation used for this comparison exists only in
`lib/DSP/qa_golay_prs.cc`; production OFDM PRS generation does not use it.

## 2026-08-05 PRS CFO Refinement and Phase Diagnostics

The detector now estimates CFO in two stages. The repeated QPSK preamble still
provides the original coarse estimate. A second estimate combines the cyclic
prefix correlation from all 16 Golay OFDM symbols (`16 * 128` CP pairs). The
preamble estimate resolves the CP estimator's phase ambiguity.

Payload decoding first uses the preamble CFO. When CRC fails and PRS CP
coherence is at least `0.2`, the detector retries once with the PRS CP CFO.
There is no multi-candidate search in this implementation. Detector metadata
now includes:

```text
preamble_cfo_hz
prs_cp_cfo_hz
prs_cp_cfo_coherence
selected_cfo_hz
payload_retry_used
```

The channel estimator retains all `16 x 1024` per-symbol channel estimates. It
estimates CFO from their inter-symbol common-phase rotation, derotates every
symbol to the first PRS-symbol time, and only then averages the channel. Added
metadata is:

```text
prs_channel_cfo_hz
residual_cfo_hz
channel_coherence
```

The measurement CSV also records `phase_slope_rad_per_hz`, `fine_delay_s`,
`fine_delay_samples`, and `phase_range_contribution_m`. The contribution is
`c * fine_delay / 2`; it is one directional contribution, not a calibrated
phase-corrected range. A complete SS-RTT phase correction requires matched poll
and response rows plus RF-chain group-delay calibration.

New output files avoid mixing the expanded schema with earlier corridor data:

```text
CSV/initiator_acquisition_v2.csv
CSV/responder_acquisition_v2.csv
CSV/initiator_measurements.csv
CSV/responder_measurements.csv
```

The initiator measurement file contains received RESPONSE measurements after
the SS-RTT solver. The responder measurement file contains received POLL
measurements before response scheduling. Join them using `poll_frame_id`.

Verification completed with GNU Radio 3.10.11:

```text
cmake --build build -j4
HOME=/tmp XDG_CACHE_HOME=/tmp ctest --test-dir build --output-on-failure
7/7 tests passed
```

The receiver QA covers positive and negative CFO, CRC recovery from a biased
preamble CFO, inter-symbol channel CFO compensation, combined CFO plus
fractional delay, CSV schema, and SS-RTT diagnostic fields.

## Future Work

The next phase should quantify the engineering cost of OFDM/PRS fine ranging, reduce avoidable waveform overhead, and extend the single-responder result toward stable multi-anchor measurements. The priority is no longer only to reduce ranging variance, but to measure what computation time, airtime, and system complexity are required to obtain that improvement.

### 1. Computational Cost and Processing Latency

Benchmark the processing time of each receive stage on the target PC using a monotonic high-resolution timer such as `std::chrono::steady_clock`. At minimum, record:

```text
frame detector
FFT receiver
channel estimator and PRS CFO correction
phase unwrap and phase-slope estimator
SS-RTT solver/logger
total per-frame RX processing time
```

For each block, report:

```text
mean
median
95th percentile
99th percentile
maximum
```

Two costs must be kept separate:

```text
1. correlation/timestamp ranging -> complete OFDM/PRS ranging cost
2. existing OFDM receiver       -> incremental fine-ranging cost
```

The second comparison is important because an OFDM communication modem already pays for FFT processing. In that case, the incremental ranging cost is mainly CFR estimation, CFO refinement, phase extraction/unwrapping, and the weighted phase-slope fit.

The current weighted phase-slope fit is a one-pass linear regression over 1024 frequency bins. It should be implemented and benchmarked as direct accumulated sums rather than a general matrix least-squares solver. Measure whether the FFT, channel estimator, complex rotations, phase extraction, or the regression itself is the actual bottleneck.

Use the measured total processing time to estimate the CPU-side ceiling:

```text
f_DSP,max ~= 1 / T_processing
```

This value must be distinguished from the complete over-the-air ranging update rate.

A useful comparison for reporting is a simple DLL-style timing tracker or correlation-only timing baseline. The comparison should focus on the additional computation and latency required by OFDM phase-based fine ranging, not assume that the OFDM method is computationally cheaper.

### 2. End-to-End Update-Rate Budget

Build an explicit timing budget for one complete ranging transaction:

```text
T_cycle =
    POLL waveform airtime
  + responder reply delay
  + RESPONSE waveform airtime
  + RX/TX processing latency
  + scheduling / guard margin
```

Then estimate:

```text
f_update,max = 1 / T_cycle
```

At the current 30 MS/s geometry, the waveform already contains large non-PRS overhead:

```text
BPSK payload: 33616 samples ~= 1.1205 ms
PRS block:     18432 samples ~= 0.6144 ms
```

The current examples also use an approximately 50 ms responder reply delay. Therefore, do not attribute the present update-rate limit to the phase-slope estimator until the complete timing budget has been measured.

Evaluate the effect of:

```text
shorter reply delay
shorter payload airtime
fewer PRS symbols
reduced guard time
DSP optimization
multi-responder scheduling
```

The final report should show which component limits the update rate before and after each optimization.

### 3. Integrate Communication Payload into OFDM

The current waveform transmits the SS-TWR fields in a separate long repeated-BPSK section before the OFDM PRS block. This is reliable but inefficient in airtime. Evaluate moving the control information into one or more OFDM symbols while preserving known pilot resources required for channel and fine-delay estimation.

Current control fields that must remain protected are:

```text
packet_type
poll_frame_id
response_frame_id
reply_delay_samples
CRC-16
```

A first implementation should avoid redesigning the entire PRS resource map. A practical intermediate waveform is:

```text
[preamble]
[coarse ZC]
[1-2 OFDM control/data symbols]
[Golay PRS symbols]
[tail guard]
```

Compare the current repeated-BPSK payload against the OFDM-integrated version using:

```text
payload airtime
BER / packet error rate
CRC success rate
ranging bias and standard deviation
phase residual
CPU processing time
maximum update rate
```

A later design may frequency-multiplex known pilots and data within the same OFDM symbols, but the receiver must retain enough known `X_m[k]` values to estimate `H_m[k] = Y_m[k] / X_m[k]` reliably.

### 4. Multi-Responder Observation Separation

The next system-level objective is to verify that one initiator can obtain stable and correctly associated observations from multiple responders.

Start with time-separated responses rather than simultaneous OFDM transmissions. Use unique response ZC roots / `channel_id` values together with staggered reply delays, for example:

```text
Responder 1: unique response root, reply slot 1
Responder 2: unique response root, reply slot 2
Responder 3: unique response root, reply slot 3
Responder 4: unique response root, reply slot 4
```

For every responder, verify independently:

```text
anchor/responder identity
poll_frame_id association
packet loss rate
integer range
phase-corrected range
SNR
coarse_metric
payload_metric
phase_residual
quality
```

Unique ZC roots only separate acquisition identities. They do not by themselves separate two fully overlapping OFDM PRS signals. If two responders transmit the same pilots at the same time, the initiator observes a superposition of their channels. Therefore, after TDMA is stable, evaluate orthogonal OFDM resource separation such as:

```text
different time slots
different subcarrier groups / PRS combs
muting patterns
other orthogonal pilot allocations
```

Test scalability from one to two, three, and four responders. Include near-far cases and intentional response collisions to determine the conditions under which per-anchor measurements remain identifiable and stable.

### 5. Fine-Ranging Validation and Calibration

Continue validating the phase-based correction across distance rather than only at one static point. Use the same calibration across a distance sweep and fit:

```text
measured_range = a * ground_truth_range + b
```

The main goals are to determine whether the remaining error is primarily a fixed bias `b`, a scale error `a`, or environment-dependent multipath.

Recommended tests:

```text
multiple LOS distances
small distance increments below one integer RTT range bin
repeatability across power cycles
SNR sweep / attenuation sweep
controlled multipath and NLOS cases
```

Record fine-delay behavior together with:

```text
phase_residual
channel_coherence
prs_cp_cfo_coherence
preamble_cfo_hz
prs_cp_cfo_hz
prs_channel_cfo_hz
residual_cfo_hz
```

Before treating phase slope as a calibrated propagation-delay measurement, implement the recommended pre-FFT time-domain PRS CFO derotation, then estimate only the residual inter-symbol CFO after FFT. Compare the existing full-band weighted regression with the paper-style low/high-frequency OPA estimator on the same frames.

### 6. Positioning Algorithms

Positioning should begin only after multi-responder measurement separation and update rate are stable. The positioning layer should consume:

```text
timestamp
channel_id
responder_id / anchor_id
anchor_position
range_m or calibrated phase-corrected range
quality
coarse_metric
payload_metric
phase_residual
snr
```

Start with weighted least squares:

```text
minimize sum_i w_i (||x - a_i|| - r_i)^2
```

where:

```text
x   = unknown initiator position
a_i = known anchor position
r_i = measured range
w_i = quality-based weight
```

Do not add a Kalman filter until the system has:

```text
at least 3 stable anchors for 2D or 4 for 3D
consistent update rate
known anchor coordinates
calibrated range bias
outlier rejection
reliable anchor identity and observation association
```

A Kalman filter can then be evaluated for moving-platform tracking after the raw multi-anchor ranging performance is characterized.


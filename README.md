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
- [2026-08-20 Processing-Time Profiling](#2026-08-20-processing-time-profiling)
- [2026-09-05 MC-DS OFDM Payload Prototype](#2026-09-05-mc-ds-ofdm-payload-prototype)
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
[127 Gold-code MC-DS OFDM PRS/data symbols]
[tail guard]
```

Default/reference frame geometry:

```text
zero guard:          1000 samples
short preamble:      preamble_len * preamble_repeats
coarse ZC sync:      coarse_sync_len samples, historically 839
standalone payload:  0 samples
MC-DS OFDM block:    127 * (fft_len + cp_len) = 73152 samples
tail guard:          1000 samples
```

Default OFDM parameters:

```text
fft_len               = 512
cp_len                = 64
active_bins            = 512
prs_symbols            = 127
mc_ds_gold_code_id     = 2
Gold code source       = lib/DSP/gold127_family_bipolar.csv
compiled Gold table    = lib/DSP/gold127_codes.h
```

All native FFT bins are occupied. Bins with `k % 4 == 3` carry 128 scrambled
BPSK data/padding values; the other 384 bins carry known pilots:

```text
fft_bin 0 ... 511
signed bins -256 ... +255
```

A length-512 Golay pair is generated at compile time by the same recursive
construction as the earlier table: even symbols use A and odd symbols use B.
Every complete frequency-domain symbol is multiplied by chip `m` from Gold
row `mc_ds_gold_code_id`; the default is authoritative CSV row 2. The `seed`
parameter remains only for the repeated QPSK acquisition preamble.

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

The 120-bit BPSK packet and CRC format is preserved, but it is carried on 120
OFDM data bins and repeated across 127 symbols with the selected Gold code.
Eight additional positions are zero padding. Before BPSK mapping, all 128
positions are XORed with the fixed balanced scrambler defined in
`lib/DSP/mc_ds_prs.h`; RX descrambles after its hard decisions and discards the
padding before the unchanged CRC check.
The old 280-samples-per-bit standalone section is no longer present.

Payload fields:

```text
packet_type
poll_frame_id
response_frame_id
reply_delay_samples
CRC-16
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
5. raw-frame metadata publication
```

`prs_channel_estimator` removes the known spreading chip and Golay pilots,
estimates channel/CFO from 384 pilot bins, despreads and equalizes the 128 data
bins, and publishes payload CRC metadata. Only a CRC-valid POLL reaches
responder scheduling.

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

This section records the earlier standalone-payload implementation. The
2026-09-05 MC-DS prototype supersedes its preamble-CFO payload retry and its
16-symbol counts.

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

## 2026-08-20 Processing-Time Profiling

Two hardware flowgraphs preserve the current SS-RTT waveform and receiver chain
while enabling per-stage monotonic-clock profiling:

```text
examples/prs_ssrtt_initiator_profiling.grc
examples/prs_ssrtt_responder_profiling.grc
```

The production blocks retain profiling disabled by default. The profiling
flowgraphs enable the optional `timing_out` ports on the frame detector, FFT
receiver, channel estimator, phase-slope estimator, SS-RTT responder/solver,
and measurement CSV logger. All reports feed one `PRS Timing Collector` block.
The text UI is omitted so terminal refresh work does not contaminate the timing
distribution.

The following blocks are timed:

| Timed block | Timed processing boundary |
| --- | --- |
| `frame_detector` | Repeated-preamble scanning, local ZC refinement, frame extraction, preamble/CP CFO estimation, payload decoding, metadata construction, and detected-frame publication. |
| `fft_receiver` | Entry to `handle_frame()` through CP removal, all 16 FFTs, active-bin reordering, output-vector construction, and `symbols_out` publication. |
| `channel_estimator` | Entry to `handle_symbols()` through pilot removal, inter-symbol CFO estimation, CFO rotation, channel averaging, residual/SNR calculation, and `channel_out` publication. |
| `phase_slope_estimator` | Entry to `handle_channel()` through phase extraction/unwrapping, weighted regression, residual/quality calculation, and `measurement_out` publication. |
| `ssrtt_responder` | Processing of an accepted POLL measurement through response-trigger construction and `trigger_out` publication. |
| `ssrtt_solver` | Storage of a POLL transmit timestamp, or processing of an accepted RESPONSE measurement through SS-RTT calculation and `ssrtt_out` publication. |
| `csv_logger` | Measurement formatting, file write, and file flush. This is reported separately and is not OFDM estimator cost. |

Each measurement uses `std::chrono::steady_clock`. A start timestamp is taken
when the relevant handler or code region is entered and a stop timestamp is
taken immediately after that region completes. Repeated operations, such as
the 16 FFT executions, are timed individually and accumulated into one stage
duration for the frame.

`handler_total` is wall-clock elapsed time from entry to the block's message
handler until completion of its normal output publication:

```text
handler_total = handler_end_time - handler_start_time
```

It includes input validation, DSP work, metadata/output construction, output
message publication, and any operating-system preemption that occurs while the
handler is running. It excludes time waiting in a GNU Radio message queue,
upstream and downstream block execution, and construction/publication of the
separate timing report. It is therefore processing latency observed on the
handler thread, not a hardware CPU-cycle count.

The stream-based frame detector is a special case because acquisition scanning
can span several scheduler calls before a frame or failed acquisition is
reported. `preamble_zc_scan` accumulates only the wall-clock time actually spent
inside the repeated-preamble and local-ZC search. `handler_total` covers the
subsequent frame/failure publication path. Use:

```text
frame_detector.acquisition_total = preamble_zc_scan + publication-handler time
```

for detector computational cost. This is not the over-the-air acquisition
window duration because idle time between scheduler calls is excluded.

An experimental `PRS FFT-ZC Frame Detector` is available as a separate block;
the production `PRS Frame Detector` remains unchanged. It uses the repeated
preamble only as a cheap signal-presence gate, then searches a bounded ZC
candidate region with overlap-save FFT matched filtering. It does not estimate
or apply CFO from the repeated preamble. The responder
defaults to 1,024 candidates before and after the gate-predicted ZC boundary.
The time-gated initiator searches ZC candidates across its scheduled response
window and again runs a global repeated-preamble scan inside that window for
experimental comparison. The repeated-metric maximum supplies only
`preamble_metric`, a predicted ZC boundary, and `zc_gate_offset_samples`; it does
not constrain the ZC interval or supply CFO correction. Because this maximum can
occur in the payload or OFDM section, `frame_start` still comes from the selected
ZC correlation lobe rather than from the preamble prediction.

With `PRS FLL CFO Compensation` enabled, the first acquisition evaluates the
fixed CFO hypotheses `-300, -200, -100, 0, 100, 200, 300 Hz` and records the
selected value as `detection_cfo_hz` with `cfo_source=INITIAL_BIN`. The channel
estimator's `channel_out` is fed back to the detector's optional `prs_cfo_in`
port. A finite `prs_channel_cfo_hz` with `channel_coherence >= 0.2` replaces the
bootstrap value for the next ZC acquisition and initial BPSK payload decode;
those frames report `cfo_source=PRS_FLL`. Disabling the option uses zero CFO and
reports `cfo_source=DISABLED`. The current-frame PRS CP estimate remains the
unwrap reference for channel CFO and the conditional payload retry.

The seven-bin search is experimental rather than a precise first-frame CFO
estimator. A 419-sample ZC at 30 MS/s spans about 14 microseconds and has a
nominal frequency resolution near 71.6 kHz, so ZC magnitudes at hypotheses only
100 Hz apart are almost identical. Its main cost is seven matched-filter passes
on the first acquisition. The longer PRS observation provides the useful CFO
feedback; subsequent acquisitions require one matched-filter pass.

Use these A/B profiling flowgraphs to compare it with the existing detector:

```text
examples/prs_ssrtt_initiator_fft_zc_profiling.grc
examples/prs_ssrtt_responder_fft_zc_profiling.grc
```

Its `fft_zc_frame_detector` timing report retains the comparable aggregate
`preamble_zc_scan` and `acquisition_total` stages. It additionally separates
`preamble_gate_scan`, `zc_input_prepare`, `zc_fft_forward`,
`zc_spectrum_multiply`, `zc_ifft`, and `zc_normalize_peak`. Therefore the
existing timing collector can compare acquisition cost without schema changes.
The first acquisition includes all seven CFO hypotheses in these accumulated
ZC stages; later acquisitions include the single tracked-CFO pass.

The experimental detector no longer uses a peak-ratio threshold or selects the
global maximum. Candidates are examined in increasing sample order. Once the
normalized ZC metric crosses `zc_threshold`, the detector returns the local
maximum within that first contiguous above-threshold correlation lobe and does
not compare it with later, stronger paths. The threshold must be high enough to
exclude partial-overlap sidelobes; the profiling flowgraphs use practical
values of `0.4` or `0.5`, and clean time-gated QA uses `0.5`.

The FFT-ZC profiling flowgraphs write acquisition diagnostics to version-4 CSV
files. In addition to the existing metrics, they record
`zc_gate_offset_samples` and the validity/metric of both the initial
tracked/initial-bin CFO payload decode and the CP-CFO retry. Use these fields to
separate a weak or displaced ZC peak from a payload-only failure. Existing
version-2 and version-3 profiling captures remain unchanged. Some version-4
captures were made while the time-gated initiator's global preamble scan was
disabled; compare the logged `preamble_metric` (`0` when disabled) when combining
those captures with current results.

### Known FFT-ZC Ranging Limitation

The FFT-ZC detector is experimental and must not currently be used as the
accuracy baseline. A same-environment 12.5 m wireless A/B test produced the
following results:

| Detector | Paired phase-corrected result | Directional fine-delay behavior |
|---|---:|---|
| Legacy local-ZC detector | `13.519 m` mean, `0.084 m` standard deviation | Both directions remained approximately within `+/-0.5` sample |
| FFT-ZC detector | `15.596 m` mean, `3.254 m` standard deviation before rejection | Poll-link estimates reached `+3.63` samples and produced corrections up to `+18.1 m` |

The unchanged Golay channel estimator and phase-slope estimator return to
normal behavior when the legacy detector is restored. This isolates the
regression to the detector/extraction path rather than the propagation channel
or the phase regression itself. Payload CRC and high CP/channel coherence do
not prove exact PRS alignment: the 280-sample BPSK integration and OFDM CP can
tolerate a small frame-boundary error that appears as a linear phase slope.

The leading diagnosis is peak-selection policy. The legacy detector examines
candidates in time order and accepts the first local ZC peak that passes its
threshold after narrow refinement. The FFT-ZC detector searches a wider region
and previously derived `frame_start` from the global maximum. In multipath, the
strongest correlation path need not be the first arriving path. The observed
bad fine-delay groups are consistent with approximately two- and four-sample
boundary changes. The experimental branch now selects the local maximum of the
earliest above-threshold correlation lobe. Synthetic two-path QA verifies that
a weaker credible first path is selected ahead of a stronger path delayed by
eight samples. Wireless validation and same-IQ comparison against the legacy
detector are still required.

Until that comparison is complete:

- Use the legacy detector for ranging accuracy, calibration, multi-anchor, and
  waveform experiments.
- Use the FFT-ZC detector only for acquisition-cost profiling and matched-filter
  development.
- Reject rather than clip a fine correction when the phase fit is invalid,
  either direction has inadequate quality, or the residual fine delay lies
  outside the expected ambiguity interval. Fall back to integer SS-RTT for that
  attempt.
- Do not claim FFT-ZC ranging accuracy from rows that include invalid phase
  fits.

Before enabling FFT-ZC for ranging, replay identical IQ through both detectors
and record the legacy ZC index, FFT global-maximum index, earliest FFT local
peak above threshold, top candidate offsets/metrics, and extracted-frame index.
The FFT detector should agree with the stable reference boundary within one
sample for at least `99.9%` of accepted frames and must not regress the
fine-delay outlier rate. The earliest-credible-peak change is an experimental
candidate fix, not yet an accuracy result.

The matched-filter development is contained in the four consecutive commits
after profiling baseline `ceef9e8`: `140240c`, `ea89e9a`, `a243170`, and
`f1110aa`. Preserve these commits on an experimental branch if the stable UHD
branch is returned to the legacy detector.

The timing collector buffers reports and calculates count, mean, median, p95,
p99, and maximum after normal flowgraph shutdown. The UHD source/sink, RX
timekeeper, timed burst source, acquisition logger, and timing collector itself
are not included in the current timed-block set.

## 2026-09-05 MC-DS OFDM Payload Prototype

The original eight-symbol prototype has now been replaced by the production
configuration specified in `CODEX_512_SCRAMBLER_GOLD127_IMPLEMENTATION.md`.
It retains the integrated 120-bit packet and removes the 33,616-sample
standalone BPSK section, but uses 127 true 512-bin CP-OFDM symbols. Native bins
`k % 4 == 3` carry 120 serialized payload bits plus eight zero-padding bits;
the other 384 bins carry the length-512 Golay A/B pilots.

```text
NFFT = 512, CP = 64, M = 127
pilot bins = 384, data bins = 128
default Gold code row = 2
```

The exact 128-bit balanced scrambler is applied after serialization, CRC, and
zero padding and before BPSK mapping. TX multiplies the entire symbol by one
chip from the selected row of `lib/DSP/gold127_family_bipolar.csv`. Runtime code
uses the generated self-contained `lib/DSP/gold127_codes.h`; regenerate it with
`tools/generate_gold127_header.py`. CSV row index maps directly to `code_id`,
and chip column maps directly to OFDM symbol index.

TX keeps native FFT indexing and uses no fftshift. RX removes the Gold chip and
Golay sign before estimating inter-symbol CFO, phase-aligns all 127 symbols,
and averages only the 384 pilot-bin channel estimates. It separately despreads
and coherently combines each data bin, interpolates its channel from adjacent
pilots, hard-decides all 128 values, descrambles, discards padding, and checks
the unchanged CRC16 over the logical 120-bit packet.

`mc_ds_gold_code_id` selects coherent processing gain; it is not packet
authentication. A wrong row has low cross-correlation and therefore does not
combine with the desired 127-symbol gain. In a noiseless single-signal case,
however, the same residual scalar can appear in both channel and data paths and
cancel during equalization, so code mismatch alone is not required to force a
CRC failure. Packet fields, ZC/channel association, and later TDMA/resource
allocation still provide responder association.

Payload success/failure is now published by `prs_channel_estimator`; the frame
detectors only publish acquisition failures and accepted raw frames. The phase
slope estimator receives a compact 384-bin channel vector. The responder is
therefore triggered only after downstream OFDM payload CRC success.

The current OFDM block airtime at 30 MS/s is:

```text
one OFDM symbol:          576 samples = 0.01920 ms
127-symbol MC-DS block: 73152 samples = 2.43840 ms
```

The production scrambler QA over 50,000 sequential POLL IDs reports useful
symbol PAPR mean/P99.9/maximum of
`7.1019 / 8.9338 / 9.5733 dB`. The Gold sign does not alter PAPR.

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

The current weighted phase-slope fit is a one-pass linear regression over 384
pilot frequency bins. Measure whether the FFT, channel estimator, complex
rotations, MC-DS despreading/equalization, phase extraction, or the regression
itself is the actual bottleneck.

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

At the current 30 MS/s prototype geometry:

```text
standalone BPSK payload: 0 samples
MC-DS OFDM block:       73152 samples = 2.4384 ms
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

The first MC-DS integrated-payload prototype is implemented. The next work is
experimental validation against the prior repeated-BPSK baseline, not another
waveform redesign.

Current control fields that must remain protected are:

```text
packet_type
poll_frame_id
response_frame_id
reply_delay_samples
CRC-16
```

A deployed prototype burst is:

```text
[preamble]
[coarse ZC]
[127 Gold-code MC-DS OFDM PRS/data symbols]
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

The receiver retains 384 known pilot bins for channel and phase-slope
estimation. Multi-code and simultaneous-responder behavior remain future work.

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

# Project Memory: OFDM PRS / SS-TWR Ranging

This file is a handoff note for future AI agents and developers working on
`/home/cnsl/gnuradio-zc-twr`. It records the current project architecture,
recent implementation state, known problems, and recommended future work.

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

Do not implement positioning filters until the multi-responder range streams are
stable.

## Repository Layout

Main OFDM PRS OOT:

```text
gr-ofdm_prs_ranging/
```

Important examples:

```text
gr-ofdm_prs_ranging/examples/prs_ssrtt_initiator_10M_1399.py
gr-ofdm_prs_ranging/examples/prs_ssrtt_responder_10M_1399.py
gr-ofdm_prs_ranging/examples/prs_ssrtt_initiator_10M_1399.grc
gr-ofdm_prs_ranging/examples/prs_ssrtt_responder_10M_1399.grc
```

Legacy ZC/BPSK TW-RTT system:

```text
tx/
rx/
common/
```

Do not break or rewrite the legacy ZC/BPSK flowgraphs when working on OFDM PRS.

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
BPSK payload:        616 samples
OFDM PRS block:      prs_symbols * (fft_len + cp_len)
tail guard:          1000 samples
```

Default OFDM parameters:

```text
fft_len      = 1024
cp_len       = 128
active_bins  = 600
prs_symbols  = 16
pilot seed   = 13990001
```

The active OFDM carriers exclude DC and use:

```text
negative bins: -active_bins/2 ... -1
positive bins: +1 ... +active_bins/2
```

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
5x repetition
```

`frame_id_valid=1` must only be set when CRC passes.

`recv_id` is a local receiver counter and is not the transmitted frame ID.

## Changes on 2026-07-24

The following performance and scheduling changes were completed without
changing the PRS frame purpose, signal-processing equations, metadata contract,
or SS-TWR range formula.

### Signal-Processing Efficiency

```text
lib/prs_receiver_utils.cc
    Added zero-copy access to PMT complex vectors.

lib/prs_frame_detector_impl.cc
    Replaced per-scheduler-call sample shifting with logical buffer drops and
    batched compaction.

lib/prs_fft_receiver_impl.cc
    Reused FFT input, output, and symbol buffers between frames.

lib/prs_channel_estimator_impl.cc
    Precomputed pilot reciprocals and calculated channel residual statistics
    in one input pass.

lib/prs_frame_builder.cc
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
lib/prs_rx_timekeeper_impl.h
lib/prs_rx_timekeeper_impl.cc
python/ofdm_prs_ranging/bindings/prs_rx_timekeeper_python.cc
grc/ofdm_prs_ranging_prs_rx_timekeeper.block.yml
```

It continuously consumes the UHD RX branch, tracks the most recent `rx_time`
tag and absolute sample offset, and converts a strobe into an explicit timed
trigger. `prs_timed_burst_source` now accepts zero or one stream input, so it
does not need to consume the RX stream when the trigger already contains
`tx_time_secs` and `tx_time_frac`.

The updated initiator topology is:

```text
UHD Source -> prs_frame_detector
UHD Source -> prs_rx_timekeeper
message_strobe -> prs_rx_timekeeper -> prs_timed_burst_source -> UHD Sink
```

The updated responder topology is:

```text
UHD Source -> prs_frame_detector
prs_frame_detector -> prs_ssrtt_responder
prs_ssrtt_responder -> prs_timed_burst_source -> UHD Sink
```

The direct `UHD Source -> prs_timed_burst_source` connection was removed from
both SSRTT flowgraphs. This prevents timed transmission from backpressuring the
RX stream at the strobe or response period.

The scheduling change also updated the timed burst source implementation,
public header, Python binding, GRC definition, CMake registration, initiator
flowgraph, responder flowgraph, and timing QA tests.

### Scheduled Correlation Windows

`prs_frame_detector` now has an optional `tx_time_in` message port and these
backward-compatible parameters:

```text
time_gating=False
reply_delay_s=0.05
window_before_s=0.0002
window_after_s=0.002
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

Correlation gating updated:

```text
include/gnuradio/ofdm_prs_ranging/prs_frame_detector.h
lib/prs_frame_detector_impl.h
lib/prs_frame_detector_impl.cc
python/ofdm_prs_ranging/bindings/prs_frame_detector_python.cc
python/ofdm_prs_ranging/qa_prs_receiver.py
grc/ofdm_prs_ranging_prs_frame_detector.block.yml
examples/prs_ssrtt_initiator_10M_1399.grc
examples/prs_ssrtt_initiator_10M_1399.py
README.md
```

### Initiator and Responder Text UI

The message-only `prs_text_ui` block was added for compact live terminal
status. It does not consume or copy the UHD sample stream.

Initiator inputs:

```text
prs_timed_burst_source.tx_time_out -> prs_text_ui.tx_in
prs_frame_detector.frame_out -> prs_text_ui.frame_in
prs_ssrtt_solver.ssrtt_out -> prs_text_ui.measurement_in
```

Initiator display:

```text
[INITIATOR] RX RECEIVING | responses/polls 49/50 (98.0%) | loss 2.0% |
pending 0 | SNR 22.4 dB | range 84.25 m
```

Responder inputs:

```text
prs_frame_detector.frame_out -> prs_text_ui.frame_in
prs_phase_slope_estimator.measurement_out -> prs_text_ui.measurement_in
prs_timed_burst_source.tx_time_out -> prs_text_ui.tx_in
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
grc/ofdm_prs_ranging_prs_text_ui.block.yml
examples/prs_ssrtt_initiator_10M_1399.grc
examples/prs_ssrtt_initiator_10M_1399.py
examples/prs_ssrtt_responder_10M_1399.grc
examples/prs_ssrtt_responder_10M_1399.py
```

### Validation

```text
Clean Release build: passed
Initiator GRC validation and generation: passed
Responder GRC validation and generation: passed
Time-gating inside/outside-window test: passed
Initiator/responder text UI tests: passed
Timed burst and ZC focused tests: passed
git diff --check: passed
```

`qa_prs_receiver` passes six of seven cases. Its older synthetic SSRTT timestamp
case still fails because of the existing hard-coded calibration delay; the
failure is unrelated to the scheduling and correlation-window changes.
Hardware overflow and CPU measurements must still be repeated after installing
the updated OOT module.

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
2. coarse ZC normalized correlation confirmation
3. frame extraction
4. rx_time preservation
5. payload decode
6. metadata publication
```

The repeated preamble is the continuous acquisition gate. The coarse ZC is only
checked after the repeated preamble passes.

## Receiver Data-Path Efficiency

The receiver implementation keeps the signal-processing equations and PDU
interfaces unchanged while avoiding avoidable work in the hot path:

```text
PMT complex-vector inputs are read through const views instead of copied.
The frame-detector buffer drops samples logically and compacts in batches.
FFT and channel-estimation output buffers are reused between messages.
QPSK pilot reciprocals are precomputed once.
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

Coarse ZC root configurability has been added for pre-positioning channel
separation.

Added parameters:

```text
prs_timed_burst_source(..., coarse_zc_root=25)
prs_frame_detector(..., coarse_zc_root=25, channel_id=0)
```

Behavior:

```text
TX coarse ZC root is set by prs_timed_burst_source coarse_zc_root.
RX coarse ZC correlation reference is set by prs_frame_detector coarse_zc_root.
Detector metadata includes coarse_zc_root and channel_id.
Acquisition CSV includes channel_id and coarse_zc_root.
```

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

At 30 MS/s full duplex, both initiator and responder can overload the GNU Radio
scheduler. When RX processing stalls, UHD source overflows; when timed TX
packets arrive late, UHD reports `L`.

Current examples have been tuned during experiments and may use:

```text
samp_rate = 30e6
preamble_len = 256
preamble_repeats = 4 or 8 depending on side/test
center_freq = 1060e6 in recent examples
```

Always verify initiator and responder use matching frame geometry:

```text
samp_rate
fft_len
cp_len
active_bins
prs_symbols
preamble_len
preamble_repeats
coarse_sync_len
payload_len
zero/tail guard
```

If these do not match, acquisition or payload decode will fail.

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

## Future Work 1: Reduce Coarse ZC Length

Motivation:

The current acquisition is two-stage. Since the repeated preamble gates the
coarse ZC correlation, the long ZC sequence is not the main continuous
long-range detector. It is mainly for:

```text
frame confirmation
coarse timing anchor
channel/root separation
false-alarm rejection
quality metric
```

Therefore `coarse_sync_len` may be reducible.

Recommended experiment order:

```text
839  current/reference
419  first reduction candidate
257  second candidate
127  probably too short for robust multi-responder outdoor use
```

Use prime lengths where possible because ZC root properties are cleaner.

Important:

If `coarse_sync_len` changes, both TX and RX examples must change together:

```text
prs_timed_burst_source coarse_sync_len
prs_frame_detector coarse_sync_len
```

Also confirm the payload and PRS offsets still propagate correctly through
metadata:

```text
payload_start = zero_guard + preamble_total + coarse_sync_len
prs_start_rel = zero_guard + preamble_total + coarse_sync_len + payload_len
```

Metrics to compare:

```text
O count
L count
detected frames/sec
valid payload frames/sec
preamble_metric
coarse_metric
payload_metric
wrong channel detections
range_m mean/std
CPU load
```

Do not reduce below 257 until root/channel separation is proven stable.

## Future Work 2: Long-Distance Positioning Preparation

Before implementing positioning:

1. Stabilize one responder at long range.
2. Confirm no persistent `O`/`L`.
3. Confirm acquisition CSV shows expected root/channel only.
4. Confirm payload CRC pass rate is high.
5. Confirm `range_m` bias and standard deviation over many samples.
6. Record responder anchor position and initiator ground truth position.

Recommended logging for each long-distance run:

```text
measurement CSV
initiator acquisition CSV
responder acquisition CSV
USRP serials
sample rate
center frequency
TX/RX gains
preamble_len
preamble_repeats
coarse_sync_len
coarse_zc_root mapping
reply_delay_samples
ground truth distance
environment notes
```

Use LOS tests first. Multipath tests should come later.

## Future Work 3: Multiple Responders

Recommended multi-responder architecture:

```text
Initiator UHD Source
  -> detector(root=29, channel_id=1)
  -> detector(root=31, channel_id=2)
  -> detector(root=37, channel_id=3)
```

Each responder:

```text
RX detector root = 25       # detects POLL
TX response root = unique   # identifies responder/channel
payload responder/frame fields confirm identity
```

The acquisition root should separate channels before payload decode. Payload
metadata should confirm identity, not be the first channel separator.

Future payload extension:

```text
responder_id or anchor_id
```

Do not add this until the root/channel separation is stable.

Possible scheduling options:

1. Fixed reply delay per responder:

```text
Responder 1 reply_delay = 50 ms
Responder 2 reply_delay = 80 ms
Responder 3 reply_delay = 110 ms
```

This avoids response collisions.

2. Same reply delay with different roots:

This is more spectrally efficient but more difficult. It risks overlap and
heavier receiver load from multiple detector branches.

Start with staggered reply delays.

## Future Work 4: Positioning Algorithms

Positioning should consume stable range measurements:

```text
timestamp
channel_id
responder_id
anchor_position
range_m
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
a_i = known anchor/responder position
r_i = measured range to anchor i
w_i = quality-based weight
```

Only after least squares works should a Kalman filter be added.

Kalman state could later be:

```text
[x, y, vx, vy]
```

or for 3D:

```text
[x, y, z, vx, vy, vz]
```

Do not begin Kalman work until:

```text
at least 3 stable 2D anchors or 4 stable 3D anchors
consistent update rate
known anchor coordinates
range outlier rejection
```

## Engineering Rules for Future Agents

- Do not modify the legacy ZC/BPSK TW-RTT flowgraphs unless explicitly asked.
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
- If changing `.grc`, either also update generated `.py` or explicitly state
  that GRC regeneration is required.
- If acquisition CSV schema changes, warn that old appended CSV files have stale
  headers.

## Current Practical Advice

For SDR stability:

```text
Start at 10 MS/s when debugging.
Use 30 MS/s only after O/L are gone.
Increase responder reply delay if L appears.
Disable acquisition logger temporarily if testing raw overflow.
Keep initiator/responder frame geometry identical.
Use different coarse ZC roots only for channel separation, not for timing fixes.
```

For current one-responder channel separation:

```text
Initiator TX POLL:       root 25
Responder RX POLL:       root 25
Responder TX RESPONSE:   root 29
Initiator RX RESPONSE:   root 29, channel_id 1
```

Delete or rotate old acquisition CSV files before testing new channel metadata:

```bash
rm -f gr-ofdm_prs_ranging/examples/CSV/initiator_acquisition.csv
rm -f gr-ofdm_prs_ranging/examples/CSV/responder_acquisition.csv
```

## 2026-07-27 Acquisition Attempt Logging

The acquisition CSV now begins with:

```text
log_time_unix,node,attempt_id,failure_reason,...
```

`prs_timed_burst_source` publishes `attempt_id` with the TX tags and
`tx_time_out` metadata. For SS-TWR, the attempt ID is the poll frame ID, so the
initiator poll and its received response share one identifier.

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

The initiator can emit one timeout result for every scheduled poll because its
detector receives the poll `tx_time_out` message. The responder has no knowledge
of remote polls that produce no detector candidate, so it cannot assign an
attempt ID to a completely missed remote poll. For a decoded poll it uses the
payload poll frame ID; a payload-CRC failure without a gating window leaves
`attempt_id` empty.

The CSV schema changed. Delete or rotate existing acquisition CSV files before
running the updated flowgraphs, otherwise append mode will retain the old
header.

## Metric Naming

`coarse_metric` is the canonical name for the normalized ZC confirmation
correlation. The former PRS metadata field `peak_metric` contained exactly the
same value and is no longer emitted or written as a separate CSV column.
Downstream C++ blocks retain read-only fallback support for old metadata that
contains `peak_metric`.

The exact `payload_metric` reference-correlation and repeated-bit vote equations
are documented in `signal.md`. CRC validity remains separate in
`frame_id_valid`.

## 2026-07-28 BPSK Payload Documentation

`signal.md` now documents the complete 616-sample BPSK payload, its 16-sample
reference, five samples per information bit, all field offsets, bit order, CRC
coverage, and the different POLL/RESPONSE field meanings.

The attenuation observation is recorded as:

```text
about 75 dB: 50% packet/CRC failure
about 95 dB: acquisition failure
```

This identifies payload decode as the first observed digital failure stage.
The separation is not purely processing gain: the payload uses hard
five-sample majority votes, and timed burst generation overwrites payload
samples with `d_tx_amp` after frame-wide normalization. Current POLL and
RESPONSE payload amplitudes are `0.4` and `0.2`, respectively.

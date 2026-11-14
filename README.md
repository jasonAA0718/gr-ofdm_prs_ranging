# OFDM PRS / SS-TWR Ranging

GNU Radio OOT blocks for USRP/UHD burst ranging using a custom OFDM PRS-like
waveform. The current system supports statically scheduled 1..8-anchor ranging
cycles and complete positioning epochs from finalized cycles with at least three
configured anchors. Fixed-height 2D position calculation and CSV result logging
are available. It is not a standards-complete LTE/NR PRS implementation.

Packet, geometry and one-anchor ranging sections were updated for Phase B.
The historical sections retain their original provenance. [signal.md](docs/signal.md) is the detailed waveform and DSP reference.
Roadmap, proposed changes, and outstanding experiments are in local
`Agenda.md`, intentionally excluded from Git.

## Current Implementation

The current waveform combines 127 Gold-spread OFDM symbols with a repeated
QPSK acquisition preamble and coarse ZC synchronization. Every OFDM symbol
has 512 bins and a 64-sample CP: 384 pilot bins and 128 data/padding bins.
The 80-bit packet v2, including CRC, is carried inside OFDM. There is
no standalone time-domain BPSK payload in the current receiver chain.

| Capability | Implementation status |
|---|---|
| Timed POLL/RESPONSE SS-TWR | Implemented using UHD sample timestamps and timed TX |
| Integrated OFDM payload | Implemented; 512 bins, 127 Gold chips, fixed data scrambler |
| Local-ZC detector | Available as `prs_frame_detector` |
| FFT-ZC detector | Available as `prs_fft_zc_frame_detector`; selected by the main GRCs |
| CFO handling | Per-anchor TDMA CFO feedback with a 5 s default lifetime; managed pre-FFT compensation plus residual pilot/data phase alignment; legacy `prs_cfo_in` retained |
| Fine delay | Weighted phase-slope estimate from the averaged 384 pilot bins |
| Live range | Integer timestamp SS-TWR plus separate bidirectional corrected range when both fine estimates are valid |
| Anchor identity | Packet-level anchor ID 0–7; responder POLL fine delay transported in RESPONSE |
| Multi-anchor ranging cycles | Implemented for 1..8 configured anchors with static windows, packet identity, duplicate suppression and bounded cycle completion |
| RF validation | One-anchor Phase B and two-anchor Phase C reported successful by the user; three/four/eight-anchor RF operation is not claimed |
| Complete positioning epochs | Implemented on `epoch_out`; requires every configured anchor, at least three anchors, and normal cycle finalization; synthetic QA verified |
| Online position calculation | Fixed-height 2D least squares in `prs_position_solver`; accepts partial observations with at least three usable anchors |
| Position CSV | `prs_position_csv_logger` records successful fixes and solver failures, with cycle association and residual summaries |
| DS-TWR, clock-skew correction | Not implemented |

The FFT-ZC algorithm remains experimental in accuracy terms even though the
examples select it. Availability, example selection, and experimental validation
are separate facts.

Source ownership is `lib/DSP` for waveform/receiver processing,
`lib/USRP` for UHD-oriented timing and SS-TWR. The older standalone ZC ranging
blocks remain separate from this OFDM pipeline. This branch does not build a
Pluto/IIO path.

## Architecture and Message Flow

```text
UHD Source -> frame detector -> FFT receiver -> channel estimator
                                                -> phase-slope estimator
                                                   -> initiator SS-RTT solver
                                                   or responder SS-RTT responder

detector.event_out -----------+
channel_estimator.event_out -+-> acquisition CSV logger

initiator: phase_estimator.measurement_out -> cycle.measurement_in
           cycle.window_out (per-anchor CFO) -> FFT-ZC detector.tx_time_in
legacy continuous path: channel_estimator.channel_out -> detector.prs_cfo_in (optional)
```

The detector extracts a complete raw complex frame with timing and acquisition
metadata. It measures CP CFO but does not decode the OFDM payload. The FFT
receiver removes CPs, applies a managed frame's requested time-domain CFO
correction, performs 127 FFTs, and reorders bins into signed-frequency order. The channel estimator removes known pilots/Gold chips, estimates
inter-symbol CFO, aligns and averages pilot channels, and despreads/equalizes
the data before CRC checking. The phase estimator receives the compact
384-bin channel vector.

Only CRC/reserved-valid packets with the appropriate packet type reach SS-TWR processing.
The responder additionally requires a finite receive timestamp. Phase-fit
`valid` and payload `frame_id_valid` are distinct. Integer ranging requires
packet validity; bidirectional correction additionally requires both directional
fine estimates to be valid and finite.

Initiator TX timing is decoupled from receiver processing:

```text
UHD Source -> prs_rx_timekeeper
message_strobe -> prs_rx_timekeeper -> prs_timed_burst_source -> UHD Sink
burst_source.tx_time_out -> detector.tx_time_in and solver.tx_time_in
```

The timekeeper consumes samples, tracks UHD `rx_time` and sample offsets, and
creates explicit timed triggers. The burst source's optional complex stream
input remains for older flowgraphs; current examples do not need that input.
Responder scheduling uses decoded POLL receive time plus `reply_delay_samples`
and sends an explicit TX-time trigger to the burst source.

Initiator acquisition is time-gated around expected responses; responder
acquisition is continuous. Gate windows limit receive search/buffering, not RF
transmission or the full transaction latency. Their size must accommodate the
configured frame and scheduling assumptions.

## Waveform and Configuration

See [signal.md](docs/signal.md) for exact bin indexing, spreading, CRC, normalization,
detector equations, and CFO observation lengths. The current shared OFDM
geometry is 512/64/127 at 30 MS/s, giving 73,152 OFDM samples or 2.4384 ms.
The Gold CSV is compiled into `lib/DSP/gold127_codes.h`; no CSV is loaded at
runtime. The length-512 Golay A/B pilots are generated at compile time in
`lib/DSP/mc_ds_prs.h`. MT19937 remains only in the repeated QPSK preamble.

Repeated-preamble and ZC RMS are fixed at 0.95. Each CP-plus-OFDM symbol is
normalized to requested RMS `tx_amp`, then the complete OFDM section is
uniformly reduced if needed to limit its peak magnitude to 0.9. That limiter
does not scale acquisition sections. Payload and pilots share the OFDM scale.

Leading and tail guards are internal fixed 1000-sample PHY constants shared by
TX and RX. Guard constructor arguments and GRC controls have been removed;
external positional callers must remove the former ninth and tenth arguments.
No ignored compatibility arguments are retained. Rebuild/reinstall consumers.

The main examples use 128/17/819 preamble period/repeats/ZC, 31 ms replies,
and 30 MS/s at 1060 MHz.
Runtime gain, thresholds, delays, receive margins and strobe settings remain
configurable. The user's main initiator threshold selections are preserved.
The split acquisition example and committed responder Python use
`window_capture -> zc_frame_sync`; its preamble CFO corrects ZC-search scratch
samples while the downstream FFT receives original samples. Monolithic paths
retain disabled preamble CFO. No acquisition redesign was made in Phase B.

Responder `anchor_id` follows `reply_delay_samples` in the constructor.
Packet identity is authoritative; channel labels, ZC roots and Gold codes do
not substitute for anchor identity or establish overlapping-response separation.
All modified measurement CSV identity columns now use `poll_id` and
`anchor_id`; use new output files rather than appending to old-schema CSVs.

## Ranging and Diagnostics

The live solver uses POLL TX time T1, RESPONSE RX time T4, and the responder's
reported reply delay. It retains the current hard-coded calibration and publishes integer
range alongside a separate corrected range. The RESPONSE carries the responder's
POLL phase estimate, and the initiator adds both directional residuals only when
their validity/finite checks pass. Invalid correction produces NaN, leaving
integer range available. Exact packet, Q10 and range equations are in
[signal.md](docs/signal.md). No clock-skew compensation or FINAL packet is present.

Fine delay is relative to the detector-selected FFT window, not absolute ToF,
and is not restricted to half a sample. Software QA does not validate RF accuracy.

Acquisition logging joins detector failures with downstream payload outcomes.
The principal IDs are `attempt_id`, `poll_id`, and
`anchor_id`. Internal metadata still contains compatibility IDs;
CRC-failed decoded IDs should not be trusted as packet identities.

| Failure reason | Meaning |
|---|---|
| `NONE` | Packet CRC and reserved bits passed |
| `PAYLOAD_CRC` | Downstream payload decode failed; also used for short symbol input |
| `NO_PREAMBLE` | Gated attempt expired without a recorded preamble gate |
| `ZC_SYNC` | Recorded preamble candidate without accepted ZC confirmation |
| `FRAME_BOUNDARY` | Recorded ZC candidate but no complete published frame |
| `UNKNOWN` | Logger metadata lacks a recognized reason |

For the time-gated FFT-ZC detector, the preamble scan is diagnostic and does
not constrain the ZC search; failure labels must be interpreted with that
distinction. `NO_PREAMBLE` does not prove absence of RF samples. UHD overflow
and late/underflow events are not separate detector failure enums.

`snr` is a pilot-channel residual ratio in dB, not calibrated dBm.
`channel_coherence`, `preamble_metric`, `coarse_metric`, and
`payload_metric` are relative quantities. No absolute RX power calibration
is implemented. Current payload-retry fields are compatibility placeholders:
retry validity/use are false and retry metric is zero.

The optional Python text UI counts unique poll IDs. Initiator loss counts
expired unanswered polls; response percentage includes all transmitted polls.
A stale valid-frame timestamp produces `NO SIGNAL`. It displays the same
relative SNR and live integer range, not an offline phase-corrected result.

## Profiling Removal

Development processing-time instrumentation and `prs_timing_collector` have
been removed. Blocks no longer accept `enable_profiling` or publish `timing_out`.
Split acquisition diagnostics no longer include `block_a_handler_time_ns`,
`cfo_processing_time_ns`, `zc_processing_time_ns`, `frame_extraction_time_ns`,
or `block_b_total_time_ns`. Acquisition outcomes and sample/UHD timestamps
remain available.

External C++ and Python callers must remove the former profiling argument.
In particular, `prs_channel_estimator` now takes `mc_ds_gold_code_id` immediately
after `seed`, and `prs_ssrtt_responder` takes `anchor_id` immediately after
`reply_delay_samples`. Update external GRC files to remove the parameter and
any connections to `timing_out`; remove timing collector blocks. Rebuild and
reinstall before running flowgraphs. UHD timing, timed TX, ranging calibration,
and the waveform are unchanged by this removal.

CMake installation does not prune files from previous installations. If an old
installation exists, also remove its obsolete `prs_timing_collector.h` and
`ofdm_prs_ranging_prs_timing_collector.block.yml` from that installation prefix.

## Per-Anchor CFO Feedback and Pre-FFT Correction

Managed initiators use the existing ranging-cycle feedback connection.
`measurement_in` already carries `prs_channel_cfo_hz` and `channel_coherence`
from channel estimation. After CRC, cycle, slot, identity, reply-delay and
first-valid observation checks, the coordinator stores the newest finite CFO
with coherence at least 0.2 under the decoded anchor ID. Phase-fit `valid` and
fine-delay validity do not gate this CFO update. Weak/nonfinite, duplicate,
stale-generation and inconsistent observations do not replace or refresh a
trusted reference. Older receive timestamps cannot overwrite newer feedback.

Each `window_out` snapshots its own anchor's reference. Age is calculated in
UHD seconds from the reference RESPONSE T4 to the upcoming window start;
`cfo_max_age_s` is a new final constructor/GRC argument, finite and positive,
with default 5.0 s. Missing, future-dated or expired references produce no
window CFO. Cache entries survive cycle finalization and are cleared on block
start; memory is bounded by the configured anchors. No smoothing is added.

The detector retains the snapshot with its window across scheduler calls.
Managed windows never use the global `prs_cfo_in` fallback: missing/invalid
references use the existing -300..+300 Hz bootstrap bank. Continuous and
legacy un-managed windows preserve their prior global-feedback behavior.
The `cfo_source` values are `ANCHOR_FLL`, `PRS_FLL`, `INITIAL_BIN`, or `DISABLED`.
With compensation enabled, managed frames carry `pre_fft_cfo_hz` equal to the
selected acquisition CFO, including bootstrap selection. Disabling detector
CFO compensation also disables its managed pre-FFT request. Responder/split
acquisition examples keep their previous wiring and processing path.

`frame_out` samples remain raw. The FFT receiver derotates only its FFT input
copy, with phase referenced to the original first frame sample and continuous
across CPs and OFDM symbols. `pre_fft_cfo_hz` on FFT output reports the actual
applied correction; absent/nonfinite requests apply zero. Original PMT samples,
frame/FFT boundaries and UHD timestamps are unchanged.

The raw CP CFO minus the applied correction supplies the residual channel-CFO
unwrap reference. Pilot/data symbol rotations use the estimated residual;
`prs_channel_cfo_hz` reports applied correction plus residual, so feedback
remains a total CFO rather than accumulating or feeding back residual alone.
The historical `residual_cfo_hz` definition stays total channel CFO minus the
raw CP/detector unwrap reference. Actual remaining FFT-input CFO is
`prs_channel_cfo_hz - pre_fft_cfo_hz`.

Measurement CSV appends `pre_fft_cfo_hz` to distinguish actual pre-FFT
correction from the reconstructed total. Start a new measurement CSV for
this schema; existing log files are not converted. Position CSV is unchanged.
Both initiator examples expose the new lifetime default and retain their
experimental RF/gain/threshold/timing settings. Rebuild/reinstall the OOT
module before running updated GRCs; the appended default preserves source-level
calls, but the compiled constructor signature has changed. Detailed equations and the
remaining ICI limitation are in [signal.md](docs/signal.md#managed-pre-fft-cfo-correction).

Software verification (2026-11-13): Release build and all 13 CTest targets pass,
including 11 new per-anchor CFO cases. All seven example GRCs were regenerated
in a temporary directory using workspace block definitions for validation;
committed initiator Python snapshots were updated only for the lifetime argument
to preserve their existing experimental settings. Synthetic checks cover cache
quality/identity/lifetime, window isolation, large positive/negative CFO unwrap,
raw-frame preservation, packet CRC, fine delay, disabled compensation and the
legacy continuous path. No installation or RF validation was performed.


## Experimental History

These are reported historical results, not freshly rerun verification of the
current 512-bin waveform or the working-tree example settings.

| Experiment | Recorded result | Interpretation |
|---|---|---|
| Outdoor LOS, 48 m laser/reference distance | About 50.8 m mean in an accepted run | Historical absolute bias remained |
| Earlier attenuation sweep, standalone BPSK | About 50% CRC failure at 75 dB attenuation; acquisition loss around 95 dB | Payload failed first; relative attenuation, not calibrated sensitivity |
| 12.5 m wireless, local-ZC | Paired corrected mean 13.519 m, std 0.084 m | Historical stable fine-delay reference |
| 12.5 m wireless, FFT-ZC | Mean 15.596 m, std 3.254 m before rejection | Historical outliers; POLL fine delay reached +3.63 samples |

The 12.5 m A/B comparison associated the regression with the changed receive
path, but did not uniquely establish its cause. The earlier global-maximum
policy has since been replaced by the earliest above-threshold lobe maximum.
A synthetic two-path test was recorded as selecting an earlier weaker path
ahead of a stronger path eight samples later; that is not wireless validation.

The project discussion also reported a 68.5 m corridor run: 503 valid ranges,
integer mean/std 67.8274/2.4823 m and offline bidirectional corrected mean/std
68.0188/0.0921 m (about -0.48 m bias). Those workbooks and the cited
`Report_OFDMlowRMS.xlsx` comparison are not present in this checkout, so their
statistics and exact waveform/gain settings cannot be independently rechecked
here. Lower OFDM RMS was reported to suppress wired-test jumps and CFO
fluctuations; this remains evidence to investigate amplitude/dynamic range,
not proof of a specific clipping location.

## Implementation History

- **2026-07-24:** PMT complex-vector views, batched detector compaction, reused
  FFT/channel buffers, and FFT-based TX replaced avoidable copying and a
  quadratic DFT. The reported 191 KB/77 KB/4.8 KB copy savings and
  `O(16 * 1024^2)` to `O(16 * 1024 * log2(1024))` comparison apply to that
  geometry. The current channel estimator divides by generated unit pilots
  directly; the old precomputed-reciprocal description is not current.
- **2026-07-24:** RX timekeeper, optional burst-source stream input, time gates,
  and terminal UI separated TX scheduling from continuous RX consumption.
- **2026-07-27:** Attempt IDs and acquisition failure reporting were added.
- **2026-07-28:** The earlier Golay CSV held 16 full-band 1024-bin symbols,
  with CP 128. Useful-symbol PAPR min/mean/max was 3.0062/3.0062/3.0062 dB,
  versus 7.4209/8.9281/10.4139 dB for the earlier 600-bin random QPSK.
  Those pilots are superseded; the table remains as an artifact.
- **2026-08-05:** Standalone BPSK used initial preamble CFO and a conditional
  CP-CFO CRC retry. Inter-symbol PRS CFO alignment and phase diagnostics were
  added. These payload retry operations are absent from the current path.
- **2026-08-20 onward:** Optional timing instrumentation and the separate
  FFT-ZC detector were introduced. Historical branch points were profiling
  baseline `ceef9e8` followed by `140240c`, `ea89e9a`, `a243170`,
  and `f1110aa`; this records provenance, not a branch-switch instruction.
- **2026-09-05 onward:** An eight-symbol MC-DS prototype was superseded by
  512-bin/127-chip Gold spreading with the fixed balanced scrambler.
  Recorded QA over 50,000 sequential POLL IDs gave useful-symbol PAPR
  mean/P99.9/max 7.1019/8.9338/9.5733 dB.
- **2026-09-10 normalization:** Acquisition amplitude became fixed at 0.95;
  `tx_amp` controls requested OFDM RMS with an OFDM-only 0.9 peak limit.

Detailed historical standalone-BPSK equations remain in
[signal.md](docs/signal.md#historical-standalone-bpsk-payload).
Previously referenced `MC_DS_CDMA_PLAN.md` and
`CODEX_512_SCRAMBLER_GOLD127_IMPLEMENTATION.md` are absent from this checkout;
the present description is grounded in source, not unavailable plan files.

## Build and Verification
Build the environments
```bash
sudo apt update

sudo apt install -y \
    xterm \
    clang-format \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-dev \
    python3-pip \
    pybind11-dev \
    libboost-all-dev \
    gnuradio-dev \
    libvolk-dev \
    linux-tools-common \
    linux-tools-generic

sudo cpupower frequency-set -g performance

sudo sysctl -w net.core.rmem_max=50000000
sudo sysctl -w net.core.wmem_max=50000000
sudo sysctl -w net.core.rmem_default=50000000
sudo sysctl -w net.core.wmem_default=50000000
```

After source changes, rebuild and reinstall on the target PC rather than
copying installed shared libraries between machines:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$(nproc)"
ctest --test-dir build --output-on-failure
sudo cmake --install build
sudo ldconfig
python3 -c "from gnuradio import ofdm_prs_ranging; print('OOT import OK')"
```

A historical August 5 record reported 7/7 tests passing with GNU Radio 3.10.11.
An older note reported a synthetic SS-RTT calibration test failure. Neither is
a statement of current test status; use a fresh build/test log for that.
This documentation review does not certify the current binary or hardware run.
Source inspection found that `test_frame_section_levels` still expects a
whole-frame peak of at most 0.900001 and a fixed 3 dB acquisition/OFDM ratio.
Those assertions conflict with the implemented 0.95 acquisition amplitude and
independent OFDM limiter; the test needs a separate implementation/QA update.
Public constructor changes require corresponding headers, bindings, GRC YAML,
examples, and focused QA updates.

Historical Phase B software verification before the later scrambler/calibration
updates: the build passed and the full registered suite
passes 8/11 targets. Packet v2, Q10 bounds/validity, PMT trigger rejection, fixed
guards, synthetic one-anchor transport and bidirectional correction pass. The
old whole-frame 0.9 amplitude assertions still conflict with unchanged 0.95
acquisition sections. The required v2 POLL bit pattern also changes the existing
PAPR regression: P99.9 9.7167 dB and maximum 10.1913 dB exceed its old 9.6/10.0
limits. An isolated re-run of the original v1 codec/PAPR benchmark gave
P99.9 8.9338 dB and maximum 9.5733 dB, confirming this is a v2 regression.
Those limits and waveform scaling were not changed to make QA green.
The obsolete subtractive-calibration/1-us synthetic assertions were reconciled
with the unchanged additive source expression in the updated ranging QA.
That historical software run did not constitute RF validation. The user has
since reported successful one-anchor Phase-B RF validation.

## Phase D Milestone 1: Complete Observation Integration

The SS-TWR Solver still calculates one anchor-specific range. The cycle manager
owns schedule validation, association, duplicate handling, finalization and
complete-observation storage. It does not recalculate or filter ranges.

The constructor retains its argument order and two-anchor defaults. Configure
between one and eight unique packet anchor IDs in 0..7 with matching delays:

```python
cycle = ofdm_prs_ranging.prs_multi_anchor_cycle(
    samp_rate=30e6,
    anchor_ids=[0, 1, 2, 3],
    time_delays=[0.031, 0.081, 0.131, 0.181],  # seconds
    preamble_len=128, preamble_repeats=17, coarse_sync_len=819,
    window_before_s=0.002, window_after_s=0.005,
    processing_grace_s=0.1)
```

Empty arrays, more than eight anchors, unequal lengths, duplicate/out-of-range
IDs, invalid delays, 24-bit sample-delay overflow and overlapping complete
receive windows are rejected. The POLL does not distribute the schedule;
responders must be statically configured with corresponding identities/delays.

Each active cycle stores at most eight complete accepted metadata dictionaries,
keyed by decoded `anchor_id`, under its local `cycle_sequence` and `poll_id`.
An observation contains the unchanged solver metadata plus a stored
`measurement_timestamp` alias for `t4_rx_time`. Individual `measurement_out`
PDUs remain unchanged. A finite integer range is sufficient for availability;
invalid fine correction retains its flag and NaN corrected range. There is no
range-magnitude screening, phase-quality rejection, weighting or averaging.

At the normal finalized completion point (last capture-window end plus the
configured hardware-time processing grace), `cycle_out` reports expected,
received and missing anchors. Only then, if **all** configured anchors have
accepted ranges and there are at least three anchors, `epoch_out` emits one
complete positioning epoch. Missing anchors, one/two-anchor configurations and
shutdown finalization never produce epochs. Early, partial and revised epochs
are not implemented. Completion cleans both stored observations and solver TX
state; duplicate and stale-generation checks remain active.

`epoch_out` is a `(metadata_dictionary, PMT_NIL)` PDU. All arrays/vectors use
the configured `anchor_ids` order, independently of response arrival order:

| Field | PMT type / meaning |
|---|---|
| `poll_id`, `cycle_sequence` | unsigned integers; wire POLL identity and local generation |
| `reference_timestamp` | double; POLL T1 in initiator UHD seconds |
| `cycle_completion_timestamp` | double; final capture-window end, before grace |
| `summary_emission_timestamp` | double; hardware RX watermark used to finalize |
| `expected_anchor_ids`, `observed_anchor_ids` | u32 vectors; identical for a complete epoch |
| `integer_ranges_m`, `corrected_ranges_m` | f64 vectors; original solver results in metres |
| `fine_correction_valid_flags` | u8 vector; 0/1 flags, without replacing invalid corrected values |
| `measurement_timestamps` | f64 vector; per-anchor RESPONSE T4 values in seconds |
| `per_anchor_quality_diagnostics` | PMT vector of dictionaries with available validity, SNR, phase and acquisition/CFO diagnostics |
| `measurements` | PMT vector of complete accepted metadata dictionaries, preserving reply delays, directional fine delays and all other diagnostics |

The new `examples/prs_ssrtt_initiator_complete_epoch.grc` and its generated
Python demonstrate four configurable anchors and print complete epochs. The
known-working single-link and two-anchor examples are preserved. At 30 MS/s
the complete RESPONSE is 78147 samples / 2.6049 ms. The example uses 50 ms
start spacing, leaving 47.3951 ms between nominal bursts and 40.3951 ms between
the 2 ms-early / 5 ms-late receive windows. Its final window ends 188.6049 ms
after T1; the 100 ms grace reaches 288.6049 ms. The inherited 500 ms POLL
strobe leaves ample separation. These are configured margins, not a new RF
validation claim. See [the schedule contract](docs/signal.md#static-multi-anchor-response-schedule)
for timing semantics and epoch eligibility.

Focused QA includes 1/2-anchor ranging-only compatibility, complete 3/4/8-anchor
epochs, no early/partial/shutdown publication, invalid fine correction, ordered
metadata preservation, duplicates, missing anchors, sequential reuse and cleanup.
The complete epoch remains an observation set. The positioning and logging
path described below additionally accepts partial observations; supplied anchor
coordinates must be surveyed for meaningful RF position accuracy.

Milestone verification (fresh build, October 28): all 14 focused cycle-manager
tests pass, including the existing complete-packet/two-window acquisition cases.
Full CTest passes 11/12 targets. `qa_prs_receiver` retains three failures already
present in the pre-edit baseline: its CSV header expectation differs from the
current logger, one timestamp test reads the removed `rtt_s` alias, and one fine
correction test expects the older additive calibration. Those tests and the
current SS-TWR equation/calibration were not changed. The Phase-C acquisition
regression checks the same synthetic RTT from canonical T1/T4/decoded delay
instead of requiring that removed alias. Existing mismatch flags and complete
summary fields are published again so diagnostics remain machine-readable.

## Position Results and CSV Logging

The cycle manager's `observation_out` publishes every finalized cycle, including
partial/empty observations and aborted cycles. Connect it to
`prs_position_solver.epoch_in`. The legacy `epoch_out` continues to publish
complete, normally finalized observations with at least three configured
anchors. Do not connect both outputs to the same solver: a complete cycle
would otherwise be processed twice.

The position solver uses a fixed vehicle height and estimates local X/Y in
metres. It attempts a fix with at least three usable, distinct configured
anchors, even when other anchors did not respond. Finite positive selected
ranges are usable; a valid finite corrected range takes priority, otherwise
the integer range is selected. There is no new outlier filter or RF accuracy
claim. Unknown/duplicate anchor IDs and malformed layouts produce diagnostics.

Each input produces either `position_out` on success or `diagnostic_out` on
failure. Both carry `success`, `reason`, `poll_id`, `cycle_sequence`, available
UHD reference/finalization timestamps, `observed_anchor_count`,
`used_anchor_count`, selected IDs/ranges, fine-selection flags and residuals.
Failures omit `position_m`; the last successful fix remains the next initial
guess. Reference time is POLL TX T1, not solver execution or CSV write time.

Connect the two solver outputs to `prs_position_csv_logger.position_in` and
its optional `diagnostic_in`. The complete-epoch example now demonstrates
this path and writes `examples/CSV/complete_epoch_positions.csv`.
The CSV columns are:

```text
cycle_sequence,poll_id,reference_timestamp,success,reason,x_m,y_m,z_m,
observed_anchor_count,used_anchor_count,anchor_ids,residual_rms_m,residual_max_abs_m
```

Each result is one row. XYZ is empty on failure; `z_m` on success is the fixed
vehicle height. `anchor_ids` lists usable selected IDs separated by semicolons.
Counts distinguish received observations from usable selected ranges, even
when too few remain to solve. No expected count, missing IDs or per-anchor
ranges are written. Residual summaries measure consistency with the selected
ranges, not position error; unavailable summaries are empty.

Individual ranges remain in the measurement CSV, which now also includes
`cycle_sequence`. Join within a run by `(cycle_sequence, poll_id)` and then
`anchor_id`; the 16-bit POLL ID alone can repeat. Use separate files per run
and retain the anchor-coordinate configuration alongside them. Start a new
measurement CSV after this schema change. Position CSV append checks its
header and rejects incompatible files. The logger flushes each row and
reports file-open/write failures.

Shutdown observation publication does not guarantee downstream processing
after the whole flowgraph has already stopped. To record `CYCLE_ABORTED`,
finalize the cycle manager while solver/logger handlers are still running,
then allow their messages to drain before stopping the flowgraph. Normal
expired cycles require no special shutdown handling.

Software verification for the positioning logger change (2026-11-12): build
and generation of all seven GRC examples pass. All ten new positioning/logging
cases and all fourteen cycle-manager cases pass. Full CTest passes 11/12 targets;
`qa_prs_receiver` still has the pre-existing removed-`rtt_s` and calibration
expectation failures. Its measurement CSV test now checks the current schema
including `cycle_sequence`. No RF positioning validation or system installation
was performed as part of this change.

Receiver QA update (2026-11-13): synthetic RTT is reconstructed as
`t4_rx_time - t1_tx_time - reply_delay_samples / samp_rate` using current
measurement CSV fields. Hard-coded absolute calibration expectations and
obsolete output-alias assertions were removed; fine-correction sign/validity
and invalid-value fallback checks remain. The solver calibration/formula is
unchanged. Fresh full CTest passes all 12/12 targets (24 receiver cases).

## Historical Phase C Implementation and Software Verification

The following records the original two-anchor delivery. Phase D above
supersedes its anchor-count and epoch limitations. The user subsequently
reported successful two-anchor Phase-C RF validation.

`prs_multi_anchor_cycle` is a native message-level coordinator between the
phase-slope receiver and the unchanged SS-TWR calculation. It creates two
independent FFT-ZC receive windows per POLL, checks the decoded packet's
`(poll_id, anchor_id)` against local schedule metadata, forwards consistent
responses to the solver, and publishes each accepted anchor-specific range.
The solver returns results to the coordinator; first-valid-wins suppression
prevents a second accepted range for the same anchor in one cycle. Invalid
fine correction still leaves the Phase-B integer range available.

The coordinator additionally emits `cycle_out` summaries and `diagnostic_out`
PDUs. A cycle summary is not a positioning epoch. No ranges are averaged and
no coordinates, position solver, three/four-anchor mode or overlapping-response
separation are implemented. This constructor currently requires exactly two
unique IDs in 0–7; the list/index representation and packet have room for later
extension without a protocol redesign.

New RF preparation examples (existing single-anchor GRCs/Python are preserved):

| Role | GRC / generated Python basename | Anchor / delay |
|---|---|---|
| Initiator | `prs_ssrtt_initiator_multi_anchor` | `anchor_ids=[0,1]`, `time_delays=[0.031,0.081]` seconds |
| Anchor 0 | `prs_ssrtt_responder_anchor0` | ID 0, 0.031 s = 930000 samples |
| Anchor 1 | `prs_ssrtt_responder_anchor1` | ID 1, 0.081 s = 2430000 samples |

All three use 30 MS/s, 1060 MHz and 128/17/819 acquisition geometry. Both
RESPONSES retain root 29, Gold code 2 and the same verified waveform. Anchor
examples reuse the existing split-acquisition responder structure; select the
correct UHD device/address on each target PC. The new initiator uses the
existing FFT-ZC gated path; optional pilot-CFO feedback is left unconnected
so one anchor's tracked CFO is not reused as the next anchor's reference.
The existing bootstrap/CP/channel CFO mathematics are unchanged.

The complete response is 78147 samples / 2.6049 ms. Start spacing is 50 ms;
physical burst gap is 47.3951 ms. The configured 2 ms early and 5 ms late
margins produce 9.6049 ms receive windows with 40.3951 ms between them.
The new initiator retains the main example's conservative 400 ms POLL strobe
and 100 ms TX lead. These are initial experimental settings, not measured
multi-anchor timing guarantees. Detailed schedule semantics are in signal.md.

Wire the timekeeper's optional `clock_out` to coordinator `clock_in`. This
hardware-time heartbeat advances expiration even with no detected packets.
The coordinator records cycle completion at the final full-window end and
allows a configured 100 ms hardware-time DSP-drain grace before emitting the
summary/cleaning solver state. Results whose RX timestamp lies outside their
window, or which arrive after that finite grace, are rejected diagnostically.
Missing anchors appear in `missing_anchor_ids`; no zero/previous ranges are
fabricated. If RX time stops advancing, normal expiration cannot advance;
flowgraph shutdown flushes active state as `STOPPED_INCOMPLETE`, not success.

Local `cycle_sequence` plus RX timestamp and slot bounds reject delayed results
from an expired generation after POLL-ID wrap. This does not authenticate a
bit-identical RF replay physically received inside a newly matching slot:
packet v2 has no on-air generation/nonce. The coordinator holds at most 16
active cycles, detector explicit windows at most 32, and pending/accepted sets
at most two anchors per cycle. Overlapping cycles, duplicate/out-of-order TX
notifications and capacity violations are rejected on `diagnostic_out`.

Responder duplicate-POLL suppression uses the POLL RX timestamp: a repeat ID
within `max(0.2 s, configured_reply_delay + 0.05 s)` does not trigger another
RESPONSE. Older queued POLLs beyond the RX high-water/hold interval are also
rejected. Entries expire by hardware time, allowing legitimate future 16-bit
ID reuse; the cache is bounded by that 16-bit domain.

Run `qa_prs_multi_anchor_cycle` for focused schedule, association, missing,
duplicate, cleanup, reuse and complete packet/acquisition tests. The existing
v2 quantization QA now uses distinct POLL IDs for its independent cases so
it does not deliberately trigger the new duplicate-POLL policy. No Phase-B
packet/fine-delay/PAPR assertions were relaxed.

Before Phase C edits, a fresh baseline build/CTest passed 10/11 targets. Two
`qa_prs_receiver` assertions still expect the earlier additive calibration;
current source uses `rtt_s/2 - 5.808278798530661e-06`. This is a pre-existing
source/test conflict. Phase C preserves the current source equation exactly
and does not alter those two expectations to make the suite green.

Software tests do not establish two-anchor RF success. Rebuild/reinstall on
all three target PCs, restart all flowgraphs and verify the loaded library.
Use fresh `CSV/multi_anchor_*` and `CSV/anchor0_*` / `CSV/anchor1_*` logs.
The initiator prints cycle summaries/diagnostics and writes accepted ranges
with decoded anchor identity to its measurement CSV. First demonstrate two
physical responders with one shared POLL ID, then evaluate RF availability,
association and range behavior before any positioning work.

Final Phase C software verification: build successful; the eight focused
`qa_prs_multi_anchor_cycle` test functions pass, including five complete
packet/FFT-ZC reception scenarios (both anchors, either missing, first CRC
invalid, and production 30 MS/s with 31/81 ms delays). Per-anchor fine-valid/invalid behavior, ID reuse, capacity rejection
and solver cleanup are covered. Full CTest passes 11/12 targets; the same two
pre-existing calibration expectations fail within `qa_prs_receiver`. There
are no new regression failures. Scrambler/PAPR checks pass unchanged
(mean/P99.9/max 6.5775/7.9880/8.5061 dB). All three new GRCs were compiled
using workspace block definitions and their generated Python is included.
Two-anchor RF validation remains pending.

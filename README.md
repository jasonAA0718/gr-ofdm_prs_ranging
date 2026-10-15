# OFDM PRS / SS-TWR Ranging

GNU Radio OOT blocks for USRP/UHD burst ranging using a custom OFDM PRS-like
waveform. The current system supports one-anchor operation and statically scheduled
two-anchor ranging cycles. Positioning remains future work. It is not a
standards-complete LTE/NR PRS implementation.

Packet, geometry and one-anchor ranging sections were updated for Phase B.
The historical sections retain their original provenance. [signal.md](signal.md) is the detailed waveform and DSP reference.
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
| FFT-ZC detector | Available as `prs_fft_zc_frame_detector`; selected by the main and profiling GRCs |
| CFO handling | CP estimate plus post-FFT pilot/data phase alignment; optional FFT-ZC CFO feedback |
| Fine delay | Weighted phase-slope estimate from the averaged 384 pilot bins |
| Live range | Integer timestamp SS-TWR plus separate bidirectional corrected range when both fine estimates are valid |
| Profiling | Optional instrumentation in existing blocks and a timing collector |
| Anchor identity | Packet-level anchor ID 0–7; responder POLL fine delay transported in RESPONSE |
| Two-anchor ranging cycles | Implemented with static windows, packet identity, duplicate suppression and bounded cycle completion; synthetic QA verified |
| RF validation | One-anchor Phase B reported successful by the user; two-anchor Phase C RF validation pending |
| Positioning epochs / online positioning | Not implemented |
| DS-TWR, clock-skew correction, pre-FFT PRS CFO correction | Not implemented |

The FFT-ZC algorithm remains experimental in accuracy terms even though the
examples select it. Availability, example selection, and experimental validation
are separate facts.

Source ownership is `lib/DSP` for waveform/receiver processing,
`lib/USRP` for UHD-oriented timing and SS-TWR, and `lib/Profiling` for timing
collection. The older standalone ZC ranging blocks remain separate from this
OFDM pipeline. This branch does not build a Pluto/IIO path.

## Architecture and Message Flow

```text
UHD Source -> frame detector -> FFT receiver -> channel estimator
                                                -> phase-slope estimator
                                                   -> initiator SS-RTT solver
                                                   or responder SS-RTT responder

detector.event_out -----------+
channel_estimator.event_out -+-> acquisition CSV logger

channel_estimator.channel_out -> FFT-ZC detector.prs_cfo_in (optional feedback)
```

The detector extracts a complete raw complex frame with timing and acquisition
metadata. It measures CP CFO but does not decode the OFDM payload. The FFT
receiver removes CPs, performs 127 FFTs, and reorders bins into signed-frequency
order. The channel estimator removes known pilots/Gold chips, estimates
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

See [signal.md](signal.md) for exact bin indexing, spreading, CRC, normalization,
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
and 30 MS/s at 1060 MHz. The profiling pair uses 256/4/419 and 50 ms replies.
Runtime gain, thresholds, delays, receive margins and strobe settings remain
configurable. The user's main initiator threshold selections are preserved.
The split acquisition example and committed responder Python use
`window_capture -> zc_frame_sync`; its preamble CFO corrects ZC-search scratch
samples while the downstream FFT receives original samples. Monolithic paths
retain disabled preamble CFO. No acquisition redesign was made in Phase B.

Responder `anchor_id` is appended after the existing profiling argument.
Packet identity is authoritative; channel labels, ZC roots and Gold codes do
not substitute for anchor identity or establish overlapping-response separation.
All modified measurement/profiling CSV identity columns now use `poll_id` and
`anchor_id`; use new output files rather than appending to old-schema CSVs.

## Ranging and Diagnostics

The live solver uses POLL TX time T1, RESPONSE RX time T4, and the responder's
reported reply delay. It retains the current hard-coded calibration and publishes integer
range alongside a separate corrected range. The RESPONSE carries the responder's
POLL phase estimate, and the initiator adds both directional residuals only when
their validity/finite checks pass. Invalid correction produces NaN, leaving
integer range available. Exact packet, Q10 and range equations are in
[signal.md](signal.md). No clock-skew compensation or FINAL packet is present.

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

## Processing-Time Profiling

The available profiling flowgraphs are:

```text
examples/prs_ssrtt_initiator_fft_zc_profiling.grc
examples/prs_ssrtt_responder_fft_zc_profiling.grc
```

The older `prs_ssrtt_initiator_profiling.grc` and responder counterpart are
not present in this checkout. Existing blocks contain optional instrumentation;
there is no copied parallel DSP implementation. Reports go through
`timing_out` to `prs_timing_collector`.

| Block | Timed work |
|---|---|
| Local/FFT-ZC detector | Accumulated acquisition scanning; extraction, CP CFO, metadata and publication |
| FFT receiver | CP removal/copy, 127 FFTs, bin reordering, output construction/publication |
| Channel estimator | Pilot removal, channel CFO, phase alignment, averaging/SNR, payload decode and publications |
| Phase-slope estimator | Phase extraction/unwrap, weighted fit, residual/quality, publication |
| SS-RTT responder | POLL validation, response-trigger construction/publication |
| SS-RTT solver | POLL timestamp storage or RESPONSE range processing/publication |
| Measurement CSV logger | Formatting, writing, flushing |

The helper uses `std::chrono::steady_clock` elapsed wall time.
`handler_total` runs from timing-report construction near handler entry to
entry into `finish()`, after normal output publication. It includes OS
preemption and instrumentation overhead within that interval; it excludes
queue waiting, downstream execution, and building/publishing the timing report.
Repeated operations are accumulated into a stage duration.

Detector scanning spans scheduler calls. `preamble_zc_scan` sums timed search
regions; detector `handler_total` describes the publication handler.
`acquisition_total` combines scan time and that publication path. It is not
elapsed RF acquisition time. Stage subtotals and handler totals overlap and
must not be summed as independent costs.

The FFT detector additionally separates gate scanning, input preparation,
forward FFT, spectrum multiplication, inverse FFT, and normalization/peak
selection. The collector buffers reports and calculates count, mean, median,
p95, p99, and maximum on normal shutdown. UHD source/sink, RX timekeeper,
timed burst source, acquisition logger, and collector are not instrumented by
this set. These results alone do not measure complete end-to-end latency.

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
[signal.md](signal.md#historical-standalone-bpsk-payload).
Previously referenced `MC_DS_CDMA_PLAN.md` and
`CODEX_512_SCRAMBLER_GOLD127_IMPLEMENTATION.md` are absent from this checkout;
the present description is grounded in source, not unavailable plan files.

## Build and Verification

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

## Phase C: Static Two-Anchor Ranging Cycles

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

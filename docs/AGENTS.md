# Coding Agent Operating Rules

## Scope

- Work within the custom OFDM PRS / USRP-UHD SS-TWR ranging system. Do not
  assume LTE/NR PRS compliance or extend the task into a PHY redesign.
- Respect ownership: `lib/DSP` for waveform/receiver processing, `lib/USRP`
  for timing and SS-TWR, and `lib/Profiling` for timing collection. Keep the
  historical standalone ZC blocks separate; do not introduce Pluto/IIO work
  into this branch as an incidental change.
- Inspect the working tree before editing and preserve unrelated user changes.

## Authority and Documentation Ownership

- Follow the user's task and constraints first. These rules govern agent work;
  they do not authorize implementing the whole roadmap.
- Use [signal.md](signal.md) for current waveform mathematics and DSP contracts,
  [README.md](README.md) for architecture, implementation status, and experimental
  history, and local `Agenda.md` for priorities and proposals, in that order
  for their respective subjects. A proposal does not override current behavior.
- Verify relevant source, bindings, and GRC settings before changing behavior.
  Source establishes what runs, not necessarily what was intended. Report
  source/document conflicts with evidence; do not silently select a calibration,
  geometry, or architecture. Resolve intent before making a dependent change.
- Treat labeled historical sections as provenance, not current specifications.
  Distinguish code availability, example selection, synthetic QA, and RF validation.
- Update implemented equations/parameters in signal.md, architecture/status and
  experimental history in README.md, and future work in Agenda.md. Avoid copying
  detailed mathematics into README.md or maintaining a second specification here.
- Agenda.md is intentionally Git-ignored and may be absent in another checkout.
  Do not force-add it or assume its contents were shared. Report missing sources
  when relevant. Keep useful experimental history labeled and do not invent results.

## Waveform and Receiver Constraints

- Preserve `fft_len=512`, `cp_len=64`, `active_bins=512`, and `prs_symbols=127`
  unless the task explicitly changes the waveform. Use 30 MS/s for documented
  calculations, while verifying the actual run's configurable sample rate/carrier.
- Preserve the 384-pilot/128-data comb (`k % 4 == 3` selects data), fixed
  scrambler, 104 information bits plus 16 CRC bits, and eight padding bits.
  The current payload is integrated into OFDM; do not restore standalone BPSK.
- Preserve native TX FFT indexing and signed-frequency RX ordering. Keep TX/RX
  Gold/Golay references identical. Gold CSV rows map to code IDs and columns to
  symbols; use the generated header at runtime. MT19937 is for the repeated
  QPSK preamble only, not current OFDM pilots.
- Keep repeated-preamble and ZC RMS at 0.95, requested CP-plus-OFDM-symbol RMS
  at `tx_amp`, and peak reduction confined to the OFDM section at limit 0.9.
  Do not replace uniform scaling with clipping or assume requested RMS is final RMS.
- Preserve separate preamble/ZC thresholds and detector-specific search behavior.
  Detector changes must preserve frame boundaries, sample indices and `rx_time`.
  OFDM payload CRC belongs to the channel estimator, not the frame detector.
- Preamble CFO is disabled. CP CFO supplies an unwrap reference; channel CFO
  aligns pilots/data after FFT. Pre-FFT PRS CFO correction is proposed work,
  not an existing feature or a prerequisite for RF validation.

## SS-TWR Timing and Metadata

- Preserve the two-message POLL/RESPONSE protocol, packet type/CRC checks,
  `poll_frame_id` association, and sample-based `reply_delay_samples` semantics.
  Do not conflate payload `frame_id_valid` with phase-fit `valid` or `quality`.
- Preserve UHD timestamp reference points and explicit timed TX triggers.
  Keep RX timekeeping decoupled from TX; the optional burst-source complex input
  remains a compatibility interface, not a reason to reconnect continuous RX.
- Check peer TX/RX preamble, ZC, guards, extraction offsets and acquisition
  windows before interpreting timing. Supplied GRCs are experiment snapshots,
  not a guaranteed matched pair. Strobe interval and `burst_period` differ.
- Do not silently change the hard-coded SS-TWR calibration. Verify the current
  expression in source and its timestamp/reference convention before modifying
  it. Historical calibration values may not remain valid after waveform,
  detector, or timestamp-reference changes. Report uncertainty instead of
  silently recalibrating.
  inspect and report the current expression and confirm its intended reference.
- Keep live integer `range_m` distinct from offline bidirectional phase
  correction. `fine_delay` is relative to the selected FFT window, not absolute
  ToF and not guaranteed within half a sample. Do not add DS-TWR, clock-skew
  correction, or online fine-delay fusion incidentally.
- Preserve PDU ports, metadata units, ID semantics and CSV failure reasons.
  Do not trust CRC-failed IDs or treat `NO_PREAMBLE` as proof of absent RF.

## Multi-Responder Work

- Validate 1, 2, 3, then 4 responders using unique response ZC roots,
  `channel_id`, staggered reply delays/TDMA slots and packet association.
  Keep an explicit channel/slot-to-anchor mapping.
- Do not assume ZC roots separate fully overlapping OFDM signals or Gold codes
  authenticate packets. Overlapping-response experiments follow stable TDMA;
  do not claim an implemented/validated multi-anchor scheduler without evidence.
- When the assigned task reaches positioning, begin with static 2D weighted
  least squares after stable multi-responder ranges have been demonstrated.
  

## Code and API Changes

- Keep changes scoped and preserve existing buffer reuse, PMT views and profiling
  interfaces. Do not copy complete DSP blocks merely to add instrumentation.
- For public constructor/API changes, update public/implementation headers,
  C++ definitions, Python bindings, GRC YAML, affected examples and QA together.
  Preserve backward compatibility where applicable; append defaulted parameters
  rather than silently reordering existing arguments.
- Check CMake source/install entries when adding, moving or removing files.
  Keep generated tables tied to their source/generator and verify exact indexing.
- Regenerate affected Python flowgraphs from GRC when needed and report which
  were regenerated; do not overwrite unrelated experimental settings.

## Build and Test

For implementation changes, configure/build and run focused QA appropriate to
the change; run the full suite for shared waveform, API or timing changes:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$(nproc)"
ctest --test-dir build --output-on-failure
```

- Rebuild/reinstall on the target PC for hardware testing; do not copy installed
  shared libraries between PCs. Use README.md's install/import workflow.
- Verify TX/RX consistency for waveform changes and synthetic timestamp/fine-delay
  behavior for ranging changes. Preserve profiling compatibility where affected.
- Report commands, failures, unavailable dependencies/hardware, and unrun checks.
  Historical pass counts are not current verification. Do not change expected
  results merely to hide calibration or normalization failures; the documented
  `test_frame_section_levels` conflict requires explicit reconciliation.
- For documentation-only edits, check links, formatting and consistency; do not
  imply that a documentation check validates binaries or RF performance.
## Experimental Data Rules
- Do not merge or compare RF experiment results unless waveform geometry,
  sample rate, carrier frequency, TX/RX gain, antenna setup, detector type,
  calibration and test environment are known to be compatible.
- Preserve raw measurement files. Write derived analysis to new files rather
  than overwriting the original data.
- Distinguish wired, attenuator, corridor, outdoor LOS, multipath and NLOS
  experiments. Do not generalize a result from one environment to another.
- For link-budget experiments, distinguish configured SDR gain and normalized
  waveform amplitude from calibrated conducted or radiated RF power.
## Priorities and Assumptions to Avoid

Use the following order when choosing among otherwise-unassigned roadmap work:

1. Link-budget and operational-range validation.
2. Multi-responder ranging validation.
3. Fine-ranging calibration across distance.
4. Multi-anchor positioning.
5. Integrated payload robustness validation.
6. Update-rate and processing-latency optimization.

- Do not label `snr`, `tx_amp`, normalized gain or uncalibrated FFT power as dBm.
  Separate calibrated RF measurements from relative DSP metrics and record
  effective waveform amplitude when interpreting link-budget experiments.
- Do not equate theoretical link budget, CRC success, high coherence, or low
  range standard deviation with validated range accuracy or availability.
- Do not assume FFT/phase-slope computation is the bottleneck. Profile measured
  faults; handler timings exclude queue wait, overlap stage subtotals, and are
  not complete transaction latency. Avoid double-counting processing already
  contained within reply delay.
- Do not treat current GRC selection of experimental FFT-ZC as proof of accuracy,
  or missing workbooks/plan files as evidence of verified behavior. State what
  could not be established and keep future estimator improvements labeled.

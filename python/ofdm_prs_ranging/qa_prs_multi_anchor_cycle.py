#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Focused static two-anchor lifecycle and real packet/acquisition regression."""
import math
import time
import numpy as np
import pmt
from gnuradio import blocks, gr, gr_unittest, ofdm_prs_ranging as prs


def meta(**values):
    out = pmt.make_dict()
    for key, value in values.items():
        if isinstance(value, bool): encoded = pmt.PMT_T if value else pmt.PMT_F
        elif isinstance(value, int): encoded = pmt.from_uint64(value) if value >= 0 else pmt.from_long(value)
        elif isinstance(value, float): encoded = pmt.from_double(value)
        else: encoded = value
        out = pmt.dict_add(out, pmt.intern(key), encoded)
    return out


def get(message, key):
    value = message if pmt.is_dict(message) else pmt.car(message)
    return pmt.to_python(pmt.dict_ref(value, pmt.intern(key), pmt.PMT_NIL))


def stamp(t):
    return pmt.make_tuple(pmt.from_uint64(math.floor(t)), pmt.from_double(t - math.floor(t)))


class qa_prs_multi_anchor_cycle(gr_unittest.TestCase):
    FS = 30e6
    DELAYS = [0.031, 0.081]
    FRAME_S = 78147 / FS

    def setUp(self):
        self.tb = gr.top_block()
        self.running = False

    def tearDown(self):
        if self.running:
            self.tb.stop()
            self.tb.wait()

    def wait_count(self, debug, count):
        deadline = time.monotonic() + 4
        while debug.num_messages() < count and time.monotonic() < deadline:
            time.sleep(0.002)
        self.assertEqual(debug.num_messages(), count)

    def post(self, block, port, message):
        block.to_basic_block()._post(pmt.intern(port), message)

    def setup_cycle(self, **kwargs):
        self.cycle = prs.prs_multi_anchor_cycle(**kwargs)
        self.solver = prs.prs_ssrtt_solver(self.FS)
        self.ranges, self.summaries, self.diag, self.windows = [blocks.message_debug() for _ in range(4)]
        self.solver_results = blocks.message_debug()
        for source, port, target, target_port in (
            (self.cycle, 'solver_tx_out', self.solver, 'tx_time_in'),
            (self.cycle, 'solver_measurement_out', self.solver, 'measurement_in'),
            (self.solver, 'ssrtt_out', self.cycle, 'range_in'),
            (self.cycle, 'measurement_out', self.ranges, 'store'),
            (self.cycle, 'cycle_out', self.summaries, 'store'),
            (self.cycle, 'diagnostic_out', self.diag, 'store'),
            (self.cycle, 'window_out', self.windows, 'store')):
            self.tb.msg_connect(source, port, target, target_port)
        self.tb.msg_connect(self.solver, 'ssrtt_out', self.solver_results, 'store')
        self.tb.start(); self.running = True

    def open_cycle(self, poll=42, t1=100.0):
        previous = self.windows.num_messages()
        self.post(self.cycle, 'tx_time_in', meta(packet_type=1, poll_id=poll,
                  tx_time_secs=math.floor(t1), tx_time_frac=t1 - math.floor(t1)))
        self.wait_count(self.windows, previous + 2)
        return [self.windows.get_message(previous + i) for i in range(2)]

    def response(self, window, anchor=None, delay=None, crc=True, fine=True):
        anchor = int(get(window, 'expected_anchor_id')) if anchor is None else anchor
        delay = int(get(window, 'expected_reply_delay_samples')) if delay is None else delay
        t4 = float(get(window, 'window_start')) + 0.002 + 13.6e-6
        m = meta(packet_type=2, frame_id_valid=crc, poll_id=int(get(window, 'poll_id')),
                 anchor_id=anchor, reply_delay_samples=delay,
                 cycle_sequence=int(get(window, 'cycle_sequence')),
                 response_slot_index=int(get(window, 'response_slot_index')),
                 fine_delay_valid=fine, valid=True, fine_delay_samples=-0.125,
                 poll_fine_delay_samples=0.25 if fine else float('nan'), rx_time=stamp(t4))
        return pmt.cons(m, pmt.PMT_NIL)

    def close_cycle(self, windows, count):
        end = max(float(get(w, 'window_end')) for w in windows)
        self.post(self.cycle, 'clock_in', meta(clock_time=end - 0.000001))
        # Completing both packets must not end the scheduled interval early.
        self.assertEqual(self.summaries.num_messages(), count - 1)
        self.post(self.cycle, 'clock_in', meta(clock_time=end + 0.101))
        self.wait_count(self.summaries, count)
        self.assertEqual(self.cycle.active_cycle_count(), 0)
        return self.summaries.get_message(count - 1)

    def test_schedule_validation_and_rounding(self):
        for ids, delays in (([0], [.031]), ([0, 0], self.DELAYS), ([0, 8], self.DELAYS),
                            ([0, 256], self.DELAYS), ([0, 1], [0, .081]),
                            ([0, 1], [float('nan'), .081]), ([0, 1], [float('inf'), .081]),
                            ([0, 1], [.031, .032]), ([0, 1], [.031, 1.0]),
                            ([0, 1], [.031])):
            with self.assertRaises(ValueError): prs.prs_multi_anchor_cycle(self.FS, ids, delays)
        self.setup_cycle(time_delays=[0.03100002, 0.08100002])
        w = self.open_cycle()
        self.assertEqual(get(w[0], 'expected_reply_delay_samples'), 930001)
        self.assertEqual(get(w[1], 'expected_reply_delay_samples'), 2430001)
        self.assertAlmostEqual(get(w[0], 'window_end') - get(w[0], 'window_start'),
                               .002 + self.FRAME_S + .005)

    def test_two_anchors_arrival_order_and_first_valid_duplicate(self):
        self.setup_cycle(); w = self.open_cycle()
        # Queue arrival order does not assign identity.
        self.post(self.cycle, 'measurement_in', self.response(w[1]))
        self.post(self.cycle, 'measurement_in', self.response(w[0], fine=False))
        self.post(self.cycle, 'measurement_in', self.response(w[0], fine=False))
        self.wait_count(self.ranges, 2); self.wait_count(self.diag, 1)
        self.assertEqual({get(self.ranges.get_message(i), 'anchor_id') for i in range(2)}, {0, 1})
        self.assertTrue(all(get(self.ranges.get_message(i), 'poll_id') == 42 for i in range(2)))
        by_anchor = {get(self.ranges.get_message(i), 'anchor_id'): self.ranges.get_message(i) for i in range(2)}
        self.assertFalse(get(by_anchor[0], 'fine_correction_valid'))
        self.assertTrue(math.isnan(get(by_anchor[0], 'corrected_range_m')))
        self.assertTrue(math.isfinite(get(by_anchor[0], 'integer_range_m')))
        self.assertTrue(get(by_anchor[1], 'fine_correction_valid'))
        self.assertTrue(math.isfinite(get(by_anchor[1], 'corrected_range_m')))
        self.assertEqual(self.cycle.active_cycle_count(), 1)  # no early completion
        summary = self.close_cycle(w, 1)
        self.assertEqual(list(get(summary, 'received_anchor_ids')), [0, 1])
        self.assertEqual(list(get(summary, 'missing_anchor_ids')), [])
        self.assertEqual(get(summary, 'duplicate_response_count'), 1)

    def test_missing_each_anchor_crc_and_fine_invalid(self):
        self.setup_cycle()
        for index, missing in enumerate((0, 1)):
            w = self.open_cycle(poll=42 + index, t1=100.0 + index)
            self.post(self.cycle, 'measurement_in', self.response(w[missing], crc=False))
            self.post(self.cycle, 'measurement_in', self.response(w[1 - missing], fine=False))
            self.wait_count(self.ranges, index + 1)
            row = self.ranges.get_message(index)
            self.assertTrue(math.isfinite(get(row, 'integer_range_m')))
            self.assertFalse(get(row, 'fine_correction_valid'))
            self.assertTrue(math.isnan(get(row, 'corrected_range_m')))
            summary = self.close_cycle(w, index + 1)
            self.assertEqual(list(get(summary, 'missing_anchor_ids')), [missing])
            self.assertEqual(list(get(summary, 'received_anchor_ids')), [1 - missing])
            self.assertEqual(get(summary, 'crc_failure_count'), 1)

    def test_slot_and_delay_mismatch_are_rejected_without_relabeling(self):
        self.setup_cycle(); w = self.open_cycle()
        self.post(self.cycle, 'measurement_in', self.response(w[0], anchor=1))
        self.post(self.cycle, 'measurement_in', self.response(w[1], delay=2430001))
        self.wait_count(self.diag, 2)
        self.assertEqual(get(self.diag.get_message(0), 'anchor_id'), 1)
        self.assertTrue(get(self.diag.get_message(0), 'slot_identity_mismatch'))
        self.assertEqual(get(self.diag.get_message(1), 'reply_delay_samples'), 2430001)
        self.assertTrue(get(self.diag.get_message(1), 'reply_delay_mismatch'))
        summary = self.close_cycle(w, 1)
        self.assertEqual(get(summary, 'accepted_measurement_count'), 0)
        self.assertEqual(get(summary, 'slot_identity_mismatch_count'), 1)
        self.assertEqual(get(summary, 'reply_delay_mismatch_count'), 1)

    def test_sequential_wrap_reuse_cleanup_and_stale_response(self):
        self.setup_cycle()
        old = None
        for i, poll in enumerate((65535, 0, 65535, 1, 2, 3)):
            w = self.open_cycle(poll=poll, t1=100.0 + i)
            if i == 2:
                self.post(self.cycle, 'measurement_in', old)
                # Even an old packet with a forged current generation fails RX-time coverage.
                m = pmt.dict_add(pmt.car(old), pmt.intern('cycle_sequence'),
                                 pmt.from_uint64(int(get(w[0], 'cycle_sequence'))))
                self.post(self.cycle, 'measurement_in', pmt.cons(m, pmt.PMT_NIL))
                self.wait_count(self.diag, 2)
            current = self.response(w[0]); old = current if i == 0 else old
            self.post(self.cycle, 'measurement_in', current)
            self.wait_count(self.ranges, i + 1)
            summary = self.close_cycle(w, i + 1)
            self.assertEqual(get(summary, 'poll_id'), poll)
            self.assertEqual(get(summary, 'accepted_measurement_count'), 1)
            # Check actual solver cleanup, not only coordinator bookkeeping.
            deadline = time.monotonic() + 1
            while self.solver.to_basic_block().nmsgs(pmt.intern('tx_time_in')) and time.monotonic() < deadline:
                time.sleep(.002)
            self.post(self.solver, 'measurement_in', current)
            deadline = time.monotonic() + 1
            while self.solver.to_basic_block().nmsgs(pmt.intern('measurement_in')) and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertEqual(self.solver_results.num_messages(), i + 1)
        self.assertEqual(self.ranges.num_messages(), 6)
        # Completed cycles leave no active coordinator state.

    def test_responder_duplicate_poll_and_time_limited_reuse(self):
        responder = prs.prs_ssrtt_responder(self.FS, 930000, False, 0)
        debug, diag = blocks.message_debug(), blocks.message_debug()
        self.tb.msg_connect(responder, 'trigger_out', debug, 'store')
        self.tb.msg_connect(responder, 'diagnostic_out', diag, 'store')
        self.tb.start(); self.running = True
        for t in (100.0, 100.0, 100.001, 101.0):
            self.post(responder, 'measurement_in', pmt.cons(meta(packet_type=1,
                poll_id=65535, frame_id_valid=True, valid=True, fine_delay_samples=.25,
                rx_time=stamp(t)), pmt.PMT_NIL))
        self.wait_count(debug, 2); self.wait_count(diag, 2)

    def test_active_capacity_is_bounded_and_expiration_cleans_all(self):
        self.setup_cycle()
        for i in range(16):
            self.open_cycle(poll=i, t1=100.0 + i)
        self.assertEqual(self.cycle.active_cycle_count(), 16)
        self.post(self.cycle, 'tx_time_in', meta(packet_type=1, poll_id=16,
                  tx_time_secs=116, tx_time_frac=0.0))
        self.wait_count(self.diag, 1)
        self.assertEqual(self.cycle.active_cycle_count(), 16)
        self.post(self.cycle, 'clock_in', meta(clock_time=117.0))
        self.wait_count(self.summaries, 16)
        self.assertEqual(self.cycle.active_cycle_count(), 0)

    def render_response(self, anchor, poll, delay, fs=1e6):
        tx = prs.prs_timed_burst_source(samp_rate=fs, preamble_repeats=17,
                                      coarse_sync_len=819, coarse_zc_root=29,
                                      attach_tx_time=False)
        head, sink = blocks.head(gr.sizeof_gr_complex, tx.frame_len()), blocks.vector_sink_c()
        tb = gr.top_block(); tb.connect(tx, head, sink); tb.start()
        self.post(tx, 'trigger', meta(packet_type=2, anchor_id=anchor, poll_id=poll,
            reply_delay_samples=delay, poll_fine_delay_q10=256, fine_delay_valid=True,
            tx_time_secs=0, tx_time_frac=0.0))
        deadline = time.monotonic() + 3
        while len(sink.data()) < tx.frame_len() and time.monotonic() < deadline: time.sleep(.002)
        tb.stop(); tb.wait()
        self.assertEqual(len(sink.data()), tx.frame_len())
        return np.asarray(sink.data(), dtype=np.complex64)

    def test_two_scheduled_windows_complete_phy_crc_and_missing_cases(self):
        fs, delays = 1e6, [.12, .25]
        excess_samples = [12, 14]  # Different known synthetic RTT residuals.
        frames = [self.render_response(i, 42, round(delays[i] * fs)) for i in range(2)]
        production_frames = [self.render_response(i, 42, round(delay * self.FS), self.FS)
                             for i, delay in enumerate(self.DELAYS)]
        slow_frames = frames
        bad_first = frames[0].copy()
        start = 1000 + 128 * 17 + 819
        section = bad_first[start:start + 127 * 576].reshape(127, 576)
        bins = np.fft.fft(section[:, 64:], axis=1); bins[:, 3] *= -1
        useful = np.fft.ifft(bins, axis=1).astype(np.complex64)
        section[:, 64:] = useful; section[:, :64] = useful[:, -64:]
        for case, expected in (('both', [0, 1]), ('missing0', [1]),
                               ('missing1', [0]), ('crc0', [1]), ('production', [0, 1])):
            with self.subTest(case=case):
                fs, delays = (self.FS, self.DELAYS) if case == 'production' else (1e6, [.12, .25])
                frames = production_frames if case == 'production' else slow_frames
                tb = gr.top_block()
                duration = .1 + max(delays) + frames[0].size / fs + .005 + .1 + .05
                samples = np.zeros(math.ceil(duration * fs), np.complex64)
                for anchor in expected if case != 'crc0' else [0, 1]:
                    frame = bad_first if case == 'crc0' and anchor == 0 else frames[anchor]
                    offset = round((.1 + delays[anchor]) * fs) + excess_samples[anchor]
                    samples[offset:offset + frame.size] = frame
                tag = gr.tag_utils.python_to_tag((0, pmt.intern('rx_time'), stamp(100.0), pmt.intern('qa')))
                source = blocks.vector_source_c(samples, False, 1, [tag])
                throttle = blocks.throttle(gr.sizeof_gr_complex, fs)
                detector = prs.prs_fft_zc_frame_detector(samp_rate=fs, preamble_repeats=17,
                    coarse_sync_len=819, coarse_zc_root=29, threshold=.3, zc_threshold=.5,
                    time_gating=True, cfo_compensation=(case == 'production'))
                cycle = prs.prs_multi_anchor_cycle(fs, [0, 1], delays)
                keeper = prs.prs_rx_timekeeper(fs)
                fft, channel, phase = prs.prs_fft_receiver(fs), prs.prs_channel_estimator(fs), prs.prs_phase_slope_estimator(fs)
                solver = prs.prs_ssrtt_solver(fs)
                ranges, summaries, captured = [blocks.message_debug() for _ in range(3)]
                tb.connect(source, throttle); tb.connect(throttle, detector); tb.connect(throttle, keeper)
                for src, port, dst, inp in (
                    (cycle, 'window_out', detector, 'tx_time_in'), (keeper, 'clock_out', cycle, 'clock_in'),
                    (cycle, 'solver_tx_out', solver, 'tx_time_in'),
                    (detector, 'frame_out', fft, 'frame_in'), (detector, 'frame_out', captured, 'store'),
                    (fft, 'symbols_out', channel, 'symbols_in'), (channel, 'channel_out', phase, 'channel_in'),
                    (phase, 'measurement_out', cycle, 'measurement_in'),
                    (cycle, 'solver_measurement_out', solver, 'measurement_in'),
                    (solver, 'ssrtt_out', cycle, 'range_in'), (cycle, 'measurement_out', ranges, 'store'),
                    (cycle, 'cycle_out', summaries, 'store')):
                    tb.msg_connect(src, port, dst, inp)
                tb.start()
                try:
                    self.post(cycle, 'tx_time_in', meta(packet_type=1, poll_id=42, tx_time_secs=100, tx_time_frac=.1))
                    self.wait_count(summaries, 1)
                    self.assertEqual(ranges.num_messages(), len(expected))
                    self.assertEqual(list(get(summaries.get_message(0), 'received_anchor_ids')), expected)
                    self.assertEqual(get(summaries.get_message(0), 'crc_failure_count'), 1 if case == 'crc0' else 0)
                    self.assertEqual(captured.num_messages(), 2 if case in ('both', 'crc0', 'production') else 1)
                    for i in range(ranges.num_messages()):
                        row = ranges.get_message(i)
                        self.assertEqual(get(row, 'poll_id'), 42)
                        self.assertIn(get(row, 'anchor_id'), expected)
                        self.assertEqual(get(row, 'anchor_id'), get(row, 'expected_anchor_id'))
                        anchor = int(get(row, 'anchor_id'))
                        self.assertAlmostEqual(get(row, 'rtt_s'), excess_samples[anchor] / fs, delta=1e-10)
                        self.assertTrue(math.isfinite(get(row, 'integer_range_m')))
                        self.assertTrue(get(row, 'fine_correction_valid'))
                        self.assertAlmostEqual(get(row, 'poll_fine_delay_samples'), .25)
                        self.assertTrue(math.isfinite(get(row, 'corrected_range_m')))
                    self.assertEqual(cycle.active_cycle_count(), 0)
                finally:
                    tb.stop(); tb.wait()


if __name__ == '__main__':
    gr_unittest.run(qa_prs_multi_anchor_cycle)

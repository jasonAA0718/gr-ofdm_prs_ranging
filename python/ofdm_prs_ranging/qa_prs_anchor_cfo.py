#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Per-anchor CFO association, scheduled acquisition and residual/total domains."""
import math
import time

import numpy as np
import pmt
from gnuradio import blocks, gr, gr_unittest, ofdm_prs_ranging as prs


def meta(**fields):
    result = pmt.make_dict()
    for key, item in fields.items():
        if isinstance(item, bool): item = pmt.from_bool(item)
        elif isinstance(item, int): item = pmt.from_uint64(item) if item >= 0 else pmt.from_long(item)
        elif isinstance(item, float): item = pmt.from_double(item)
        elif isinstance(item, str): item = pmt.intern(item)
        result = pmt.dict_add(result, pmt.intern(key), item)
    return result


def field(message, key):
    data = message if pmt.is_dict(message) else pmt.car(message)
    return pmt.dict_ref(data, pmt.intern(key), pmt.PMT_NIL)


def get(message, key):
    return pmt.to_python(field(message, key))


def stamp(t):
    return pmt.make_tuple(pmt.from_uint64(math.floor(t)), pmt.from_double(t-math.floor(t)))


class qa_prs_anchor_cfo(gr_unittest.TestCase):
    FS = 30e6

    def setUp(self):
        self.tb = gr.top_block()
        self.running = False

    def tearDown(self):
        if self.running:
            self.tb.stop()
            self.tb.wait()

    def start(self, limit=8192):
        self.tb.start(max_noutput_items=limit)
        self.running = True

    def stop(self):
        self.tb.stop()
        self.tb.wait()
        self.running = False

    def post(self, block, port, message):
        block.to_basic_block()._post(pmt.intern(port), message)

    def wait_count(self, debug, count):
        deadline = time.monotonic() + 4
        while debug.num_messages() < count and time.monotonic() < deadline:
            time.sleep(.002)
        self.assertEqual(debug.num_messages(), count)

    def setup_cache(self, **kwargs):
        self.cycle = prs.prs_multi_anchor_cycle(anchor_ids=[2, 0], time_delays=[.031, .081], **kwargs)
        self.windows, self.forwarded, self.summaries, self.diag = [blocks.message_debug() for _ in range(4)]
        for port, target in [('window_out', self.windows), ('solver_measurement_out', self.forwarded),
                             ('cycle_out', self.summaries), ('diagnostic_out', self.diag)]:
            self.tb.msg_connect(self.cycle, port, target, 'store')
        self.start()

    def open_cycle(self, t1=100., poll=42):
        count = self.windows.num_messages()
        self.post(self.cycle, 'tx_time_in', meta(packet_type=1, poll_id=poll,
                  tx_time_secs=math.floor(t1), tx_time_frac=t1-math.floor(t1)))
        self.wait_count(self.windows, count+2)
        return [self.windows.get_message(count+i) for i in range(2)]

    def packet(self, window, cfo, coherence=.9, **changes):
        t4 = get(window, 'window_start') + .002 + .00002
        fields = dict(packet_type=2, frame_id_valid=True, poll_id=int(get(window, 'poll_id')),
                      cycle_sequence=int(get(window, 'cycle_sequence')),
                      anchor_id=int(get(window, 'expected_anchor_id')),
                      response_slot_index=int(get(window, 'response_slot_index')),
                      reply_delay_samples=int(get(window, 'expected_reply_delay_samples')),
                      rx_time=stamp(t4), prs_channel_cfo_hz=float(cfo), channel_coherence=float(coherence),
                      valid=False, fine_delay_valid=False)
        fields.update(changes)
        return pmt.cons(meta(**fields), pmt.PMT_NIL)

    def close_cycle(self, windows):
        count = self.summaries.num_messages()
        self.post(self.cycle, 'clock_in', meta(clock_time=max(get(w, 'window_end') for w in windows)+.101))
        self.wait_count(self.summaries, count+1)

    def test_cache_is_per_anchor_persistent_and_expires_without_refresh_from_bad_cfo(self):
        self.setup_cache()
        first = self.open_cycle()
        for window in first:
            self.assertTrue(pmt.is_null(field(window, 'prs_channel_cfo_hz')))
        for window, cfo in zip(first, (4500., -7000.)):
            self.post(self.cycle, 'measurement_in', self.packet(window, cfo))
        self.wait_count(self.forwarded, 2)
        # Duplicate or CRC-invalid observations must not replace either cache entry.
        self.post(self.cycle, 'measurement_in', self.packet(first[0], 12345.))
        self.post(self.cycle, 'measurement_in', self.packet(first[1], 12345., frame_id_valid=False))
        self.wait_count(self.diag, 2)
        self.close_cycle(first)
        second = self.open_cycle(101., 43)
        self.assertEqual([get(w, 'prs_channel_cfo_hz') for w in second], [4500., -7000.])
        for window in first:
            self.assertTrue(pmt.is_null(field(window, 'prs_channel_cfo_hz')))
        self.post(self.cycle, 'measurement_in', self.packet(second[0], 9999., coherence=.1))
        self.post(self.cycle, 'measurement_in', self.packet(second[1], float('nan')))
        self.wait_count(self.forwarded, 4)
        self.close_cycle(second)
        # Snapshot remains unchanged even after another observation was received.
        third = self.open_cycle(102., 44)
        self.assertEqual([get(w, 'prs_channel_cfo_hz') for w in third], [4500., -7000.])
        self.close_cycle(third)
        expired = self.open_cycle(105.5, 45)
        self.assertTrue(all(pmt.is_null(field(w, 'prs_channel_cfo_hz')) for w in expired))
        self.close_cycle(expired)
        # One-anchor reacquisition must not refresh another anchor's stale reference.
        recovery = self.open_cycle(106.5, 46)
        self.post(self.cycle, 'measurement_in', self.packet(recovery[0], -1200.))
        self.wait_count(self.forwarded, 5)
        self.close_cycle(recovery)
        last = self.open_cycle(107.5, 47)
        self.assertEqual(get(last[0], 'prs_channel_cfo_hz'), -1200.)
        self.assertTrue(pmt.is_null(field(last[1], 'prs_channel_cfo_hz')))

    def test_rejected_identity_slot_delay_and_out_of_order_updates(self):
        self.setup_cache()
        old = self.open_cycle(100., 10)
        new = self.open_cycle(101., 11)
        for changes in ({'anchor_id': 0}, {'response_slot_index': 99},
                        {'reply_delay_samples': 930001}):
            self.post(self.cycle, 'measurement_in', self.packet(new[0], 9999., **changes))
        self.wait_count(self.diag, 3)
        self.post(self.cycle, 'measurement_in', self.packet(new[0], 5000.))
        self.wait_count(self.forwarded, 1)
        self.post(self.cycle, 'measurement_in', self.packet(old[0], 1000.))
        self.wait_count(self.forwarded, 2)
        newest = self.open_cycle(102., 12)
        self.assertEqual(get(newest[0], 'prs_channel_cfo_hz'), 5000.)
        self.assertTrue(pmt.is_null(field(newest[1], 'prs_channel_cfo_hz')))

    def test_lifetime_validation_and_cache_cleared_on_start(self):
        for bad in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError): prs.prs_multi_anchor_cycle(cfo_max_age_s=bad)
        self.setup_cache(cfo_max_age_s=.5)
        first = self.open_cycle()
        self.post(self.cycle, 'measurement_in', self.packet(first[0], 3000.))
        self.wait_count(self.forwarded, 1)
        self.close_cycle(first)
        stale = self.open_cycle(101., 43)
        self.assertTrue(pmt.is_null(field(stale[0], 'prs_channel_cfo_hz')))
        self.post(self.cycle, 'measurement_in', self.packet(stale[0], 4000.))
        self.wait_count(self.forwarded, 2)
        self.cycle.stop()
        self.wait_count(self.summaries, 2)
        self.cycle.start()
        fresh = self.open_cycle(102., 44)
        self.assertTrue(pmt.is_null(field(fresh[0], 'prs_channel_cfo_hz')))

    def render_response(self, anchor=0):
        tx = prs.prs_timed_burst_source(samp_rate=self.FS, preamble_repeats=17,
                                      coarse_sync_len=819, coarse_zc_root=29, attach_tx_time=False)
        tb, head, sink = gr.top_block(), blocks.head(gr.sizeof_gr_complex, tx.frame_len()), blocks.vector_sink_c()
        tb.connect(tx, head, sink)
        tb.start()
        try:
            self.post(tx, 'trigger', meta(packet_type=2, anchor_id=anchor, poll_id=42,
                      reply_delay_samples=930000, tx_time_secs=0, tx_time_frac=0.))
            deadline = time.monotonic()+4
            while len(sink.data()) < tx.frame_len() and time.monotonic() < deadline: time.sleep(.002)
            self.assertEqual(len(sink.data()), tx.frame_len())
        finally:
            tb.stop(); tb.wait()
        return np.asarray(sink.data(), np.complex64), tx.prs_start()

    def fft_pipeline(self):
        fft, channel, phase = prs.prs_fft_receiver(self.FS), prs.prs_channel_estimator(self.FS), prs.prs_phase_slope_estimator(self.FS)
        symbols, decoded, estimates = [blocks.message_debug() for _ in range(3)]
        for source, port, target, input_port in (
                (fft, 'symbols_out', symbols, 'store'), (fft, 'symbols_out', channel, 'symbols_in'),
                (channel, 'channel_out', decoded, 'store'), (channel, 'channel_out', phase, 'channel_in'),
                (phase, 'measurement_out', estimates, 'store')):
            self.tb.msg_connect(source, port, target, input_port)
        return fft, symbols, decoded, estimates

    def test_pre_fft_exact_correction_preserves_raw_samples_and_fft_window(self):
        frame, start = self.render_response()
        fft, symbols, decoded, estimates = self.fft_pipeline()
        self.start()
        expected = np.fft.fft(frame[start:start+127*576].reshape(127, 576)[:, 64:], axis=1)
        expected = np.concatenate((expected[:, 256:], expected[:, :256]), axis=1).ravel()
        for index, total in enumerate((70000., -65000.)):
            injected = (frame*np.exp(2j*np.pi*total*np.arange(frame.size)/self.FS)).astype(np.complex64)
            packet = pmt.cons(meta(prs_start_rel=start, prs_cp_cfo_hz=total, pre_fft_cfo_hz=total),
                              pmt.init_c32vector(injected.size, injected))
            self.post(fft, 'frame_in', packet)
            self.wait_count(estimates, index+1)
            np.testing.assert_allclose(np.asarray(pmt.c32vector_elements(pmt.cdr(symbols.get_message(index)))),
                                       expected, atol=2e-5, rtol=1e-5)
            np.testing.assert_array_equal(np.asarray(pmt.c32vector_elements(pmt.cdr(packet))), injected)
            self.assertTrue(get(decoded.get_message(index), 'frame_id_valid'))
            self.assertAlmostEqual(get(decoded.get_message(index), 'prs_channel_cfo_hz'), total, delta=.1)
            self.assertAlmostEqual(get(estimates.get_message(index), 'fine_delay_samples'), 0., delta=1e-4)

    def test_rejected_short_frame_never_reports_requested_correction_as_applied(self):
        fft, symbols, decoded, estimates = self.fft_pipeline()
        self.start()
        self.post(fft, 'frame_in', pmt.cons(meta(prs_start_rel=0, pre_fft_cfo_hz=70000.),
                                          pmt.init_c32vector(8, [0j]*8)))
        self.wait_count(decoded, 1)
        self.assertEqual(get(symbols.get_message(0), 'fft_error'), 'short_frame')
        self.assertEqual(get(decoded.get_message(0), 'pre_fft_cfo_hz'), 0.)
        self.assertFalse(get(decoded.get_message(0), 'frame_id_valid'))

    def test_residual_unwrap_total_reconstruction_and_no_repeated_accumulation(self):
        frame, start = self.render_response()
        fft, symbols, decoded, estimates = self.fft_pipeline()
        self.start()
        cases = [(70000., 69000.), (70000., 70000.), (-65000., -64900.)]
        for index, (total, applied) in enumerate(cases):
            injected = (frame*np.exp(2j*np.pi*total*np.arange(frame.size)/self.FS)).astype(np.complex64)
            packet = pmt.cons(meta(prs_start_rel=start, prs_cp_cfo_hz=total, pre_fft_cfo_hz=applied),
                              pmt.init_c32vector(injected.size, injected))
            self.post(fft, 'frame_in', packet)
            self.wait_count(estimates, index+1)
            result = decoded.get_message(index)
            self.assertTrue(get(result, 'frame_id_valid'))
            self.assertEqual(get(result, 'pre_fft_cfo_hz'), applied)
            self.assertAlmostEqual(get(result, 'prs_channel_cfo_hz'), total, delta=.2)
            # Historical residual field is total-minus-raw-CP, not remaining FFT-input CFO.
            self.assertAlmostEqual(get(result, 'residual_cfo_hz'), 0., delta=.2)
            self.assertAlmostEqual(get(result, 'prs_channel_cfo_hz')-applied, total-applied, delta=.2)

    def check_detector_windows(self, chunk, references, actual, compensate=True):
        frames = [self.render_response(anchor)[0] for anchor in (2, 0)]
        starts = [4096, 4096+frames[0].size+128]
        samples = np.zeros(starts[-1]+frames[-1].size+512, np.complex64)
        for start, frame, cfo in zip(starts, frames, actual):
            samples[start:start+frame.size] = frame*np.exp(2j*np.pi*cfo*np.arange(frame.size)/self.FS)
        tag = gr.tag_utils.python_to_tag((0, pmt.intern('rx_time'), stamp(100.), pmt.intern('qa')))
        source = blocks.vector_source_c(samples, False, 1, [tag])
        source.set_min_output_buffer(len(samples))
        detector = prs.prs_fft_zc_frame_detector(samp_rate=self.FS, preamble_repeats=17,
            coarse_sync_len=819, coarse_zc_root=29, time_gating=True,
            correlation_fft_len=2048, zc_threshold=.5, cfo_compensation=compensate)
        fft, symbols, decoded, estimates = self.fft_pipeline()
        captured = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect(detector, 'frame_out', fft, 'frame_in')
        self.tb.msg_connect(detector, 'frame_out', captured, 'store')
        # A deliberately unrelated global reference must never affect managed slots.
        self.post(detector, 'prs_cfo_in', meta(prs_channel_cfo_hz=13000., channel_coherence=.99))
        for slot, (start, frame, reference) in enumerate(zip(starts, frames, references)):
            window = dict(cycle_sequence=9, expected_anchor_id=(2, 0)[slot], poll_id=42,
                          response_slot_index=slot, expected_reply_delay_samples=930000,
                          window_start=100.+(start-64)/self.FS,
                          window_end=100.+(start+frame.size+64)/self.FS)
            if reference is not None:
                window.update(prs_channel_cfo_hz=reference, channel_coherence=.95)
            self.post(detector, 'tx_time_in', meta(**window))
        self.start(chunk)
        self.wait_count(estimates, 2)
        for i, (start, total) in enumerate(zip(starts, actual)):
            raw = captured.get_message(i)
            self.assertEqual(get(raw, 'frame_start'), start)
            self.assertEqual(get(raw, 'expected_anchor_id'), (2, 0)[i])
            np.testing.assert_array_equal(np.asarray(pmt.c32vector_elements(pmt.cdr(raw))),
                                          samples[start:start+frames[i].size])
            result = decoded.get_message(i)
            self.assertTrue(get(result, 'frame_id_valid'))
            self.assertEqual(get(result, 'anchor_id'), (2, 0)[i])
            self.assertAlmostEqual(get(result, 'prs_channel_cfo_hz'), total, delta=.2)
            if not compensate:
                self.assertEqual(get(raw, 'cfo_source'), 'DISABLED')
                self.assertEqual(get(result, 'pre_fft_cfo_hz'), 0.)
            elif references[i] is None:
                self.assertEqual(get(raw, 'cfo_source'), 'INITIAL_BIN')
                self.assertLessEqual(abs(get(result, 'pre_fft_cfo_hz')), 300.)
            else:
                self.assertEqual(get(raw, 'cfo_source'), 'ANCHOR_FLL')
                self.assertEqual(get(result, 'pre_fft_cfo_hz'), references[i])
        self.stop()

    def test_window_references_are_isolated_across_work_calls(self):
        self.check_detector_windows(1024, [70000., -65000.], [70000., -65000.])

    def test_adjacent_windows_queued_together_keep_their_own_reference(self):
        self.check_detector_windows(1048576, [70000., -65000.], [70000., -65000.])

    def test_unknown_anchor_bootstraps_without_using_global_feedback(self):
        self.check_detector_windows(1024, [70000., None], [70000., 900.])

    def test_disabled_compensation_remains_disabled(self):
        self.check_detector_windows(1024, [900., -900.], [900., -900.], compensate=False)

    def test_legacy_continuous_feedback_keeps_raw_fft_input(self):
        frame, start = self.render_response()
        cfo = 900.
        injected = (frame*np.exp(2j*np.pi*cfo*np.arange(frame.size)/self.FS)).astype(np.complex64)
        samples = np.concatenate((np.zeros(2048, np.complex64), injected, np.zeros(4096, np.complex64)))
        source = blocks.vector_source_c(samples, False)
        detector = prs.prs_fft_zc_frame_detector(samp_rate=self.FS, preamble_repeats=17,
            coarse_sync_len=819, coarse_zc_root=29, correlation_fft_len=2048)
        fft, symbols, decoded, estimates = self.fft_pipeline()
        captured = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect(detector, 'frame_out', captured, 'store')
        self.tb.msg_connect(detector, 'frame_out', fft, 'frame_in')
        self.post(detector, 'prs_cfo_in', meta(prs_channel_cfo_hz=cfo, channel_coherence=.95))
        self.start()
        self.wait_count(estimates, 1)
        raw, result = captured.get_message(0), decoded.get_message(0)
        self.assertEqual(get(raw, 'cfo_source'), 'PRS_FLL')
        self.assertTrue(pmt.is_null(field(raw, 'pre_fft_cfo_hz')))
        self.assertEqual(get(result, 'pre_fft_cfo_hz'), 0.)
        self.assertTrue(get(result, 'frame_id_valid'))
        self.assertAlmostEqual(get(result, 'prs_channel_cfo_hz'), cfo, delta=.2)


if __name__ == '__main__':
    gr_unittest.run(qa_prs_anchor_cfo)

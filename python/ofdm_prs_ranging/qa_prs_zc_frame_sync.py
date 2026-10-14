#!/usr/bin/env python3

from gnuradio import blocks, gr, gr_unittest, ofdm_prs_ranging
import numpy
import pmt
import time


class qa_prs_zc_frame_sync(gr_unittest.TestCase):
    FS = 1e6
    FFT = 32
    CP = 8
    ACTIVE = 32
    NSYM = 3
    L = 16
    R = 5
    NZ = 31
    NG = ofdm_prs_ranging.PHY_LEADING_GUARD_SAMPLES
    NT = ofdm_prs_ranging.PHY_TAIL_GUARD_SAMPLES
    ROOT = 7
    CHANNEL = 4
    PREAMBLE_THRESHOLD = 0.8
    ZC_THRESHOLD = 0.8

    @property
    def coarse_rel(self):
        return self.NG + self.L * self.R

    @property
    def frame_len(self):
        return self.coarse_rel + self.NZ + self.NSYM * (self.FFT + self.CP) + self.NT

    def _sync(self):
        return ofdm_prs_ranging.prs_zc_frame_sync(
            self.FS,
            self.FFT,
            self.CP,
            self.ACTIVE,
            self.NSYM,
            self.L,
            self.R,
            self.NZ,
            self.PREAMBLE_THRESHOLD,
            self.ZC_THRESHOLD,
            self.ROOT,
            self.CHANNEL,
            True,
        )

    def _period(self):
        rng = numpy.random.default_rng(17)
        values = rng.integers(0, 4, self.L)
        return numpy.asarray(
            [complex(1 if value & 1 else -1, 1 if value & 2 else -1)
             for value in values], dtype=numpy.complex64) / numpy.sqrt(2.0)

    def _zc(self):
        n = numpy.arange(self.NZ, dtype=numpy.float64)
        root = self.ROOT % self.NZ
        if root <= 0:
            root += self.NZ
        return numpy.exp(-1j * numpy.pi * root * n * (n + 1) / self.NZ).astype(
            numpy.complex64)

    def _frame(self, valid_preamble=True, valid_zc=True):
        frame = numpy.zeros(self.frame_len, dtype=numpy.complex64)
        if valid_preamble:
            frame[self.NG:self.coarse_rel] = numpy.tile(self._period(), self.R) * 0.95
        if valid_zc:
            frame[self.coarse_rel:self.coarse_rel + self.NZ] = self._zc() * 0.95
        rng = numpy.random.default_rng(23)
        prs_start = self.coarse_rel + self.NZ
        for symbol in range(self.NSYM):
            useful = (rng.standard_normal(self.FFT) +
                      1j * rng.standard_normal(self.FFT)).astype(numpy.complex64) * 0.05
            start = prs_start + symbol * (self.FFT + self.CP)
            frame[start:start + self.CP] = useful[-self.CP:]
            frame[start + self.CP:start + self.CP + self.FFT] = useful
        return frame

    def _candidate(self, candidate_id=1, frame_start=100, cfo_hz=0.0,
                   amplitude=1.0, first_crossing_offset=None,
                   valid_preamble=True, valid_zc=True, fixed_search=None):
        frame = self._frame(valid_preamble, valid_zc)
        after = 100
        samples = numpy.zeros(frame_start + len(frame) + after, dtype=numpy.complex64)
        samples[frame_start:frame_start + len(frame)] = frame
        if cfo_hz:
            phase = numpy.exp(1j * 2.0 * numpy.pi * cfo_hz *
                              numpy.arange(len(samples)) / self.FS)
            samples = (samples * phase).astype(numpy.complex64)
        samples = (samples * amplitude).astype(numpy.complex64)

        candidate_start = 10000 + candidate_id * 1000
        zc_local = frame_start + self.coarse_rel
        if first_crossing_offset is None:
            first_crossing_offset = frame_start + self.NG
        search_start = self.coarse_rel
        search_end = len(samples) - (self.NZ + self.NSYM * (self.FFT + self.CP) + self.NT)
        if fixed_search is not None:
            search_start, search_end = fixed_search
        meta = pmt.make_dict()
        values = {
            "candidate_id": candidate_id,
            "attempt_id": 70 + candidate_id,
            "window_id": 90 + candidate_id,
            "candidate_window_start": candidate_start,
            "candidate_window_end": candidate_start + len(samples),
            "candidate_window_length": len(samples),
            "absolute_sample_index": candidate_start,
            "first_preamble_crossing_index": candidate_start + first_crossing_offset,
            "zc_search_start_offset": search_start,
            "zc_search_end_offset": search_end,
            "fft_len": self.FFT,
            "cp_len": self.CP,
            "active_bins": self.ACTIVE,
            "prs_symbols": self.NSYM,
            "preamble_len": self.L,
            "preamble_repeats": self.R,
            "coarse_sync_len": self.NZ,
            "zero_guard_len": self.NG,
            "tail_guard_len": self.NT,
        }
        for key, value in values.items():
            meta = pmt.dict_add(meta, pmt.intern(key), pmt.from_uint64(value))
        meta = pmt.dict_add(
            meta, pmt.intern("candidate_rx_time"),
            pmt.make_tuple(pmt.from_uint64(100), pmt.from_double(0.25)))
        meta = pmt.dict_add(
            meta, pmt.intern("rx_time_tag_offset"), pmt.from_uint64(9876))
        return pmt.cons(meta, pmt.init_c32vector(len(samples), samples)), samples, frame, zc_local

    @staticmethod
    def _meta(message):
        return pmt.car(message) if pmt.is_pair(message) else message

    @staticmethod
    def _uint(meta, key):
        return pmt.to_uint64(pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL))

    @staticmethod
    def _double(meta, key):
        return pmt.to_double(pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL))

    @staticmethod
    def _symbol(meta, key):
        return pmt.symbol_to_string(pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL))

    def _process(self, candidate):
        tb = gr.top_block()
        sync = self._sync()
        frames = blocks.message_debug()
        diagnostics = blocks.message_debug()
        releases = blocks.message_debug()
        timing = blocks.message_debug()
        tb.msg_connect(sync, "synchronized_frame_out", frames, "store")
        tb.msg_connect(sync, "diagnostics_out", diagnostics, "store")
        tb.msg_connect(sync, "slot_release", releases, "store")
        tb.msg_connect(sync, "timing_out", timing, "store")
        tb.start()
        sync.to_basic_block()._post(pmt.intern("candidate_in"), candidate)
        deadline = time.monotonic() + 2.0
        while releases.num_messages() == 0 and time.monotonic() < deadline:
            time.sleep(0.001)
        tb.stop()
        tb.wait()
        self.assertEqual(releases.num_messages(), 1)
        self.assertEqual(diagnostics.num_messages(), 1)
        self.assertEqual(timing.num_messages(), 1)
        return frames, self._meta(diagnostics.get_message(0)), releases.get_message(0)

    def test_zero_cfo_frame_and_metadata_contract(self):
        candidate, samples, expected_frame, zc_local = self._candidate()
        frames, diag, release = self._process(candidate)
        self.assertEqual(frames.num_messages(), 1)
        self.assertTrue(pmt.to_bool(pmt.dict_ref(diag, pmt.intern("success"), pmt.PMT_F)))
        self.assertEqual(self._symbol(diag, "failure_reason"), "NONE")
        self.assertEqual(self._uint(diag, "selected_local_zc_index"), zc_local)
        self.assertAlmostEqual(self._double(diag, "coarse_metric"), 1.0, places=5)
        self.assertAlmostEqual(self._double(diag, "preamble_cfo_hz"), 0.0, delta=0.2)

        frame_pdu = frames.get_message(0)
        frame_meta = self._meta(frame_pdu)
        observed = numpy.asarray(pmt.c32vector_elements(pmt.cdr(frame_pdu)),
                                 dtype=numpy.complex64)
        numpy.testing.assert_array_equal(observed, samples[100:100 + self.frame_len])
        numpy.testing.assert_array_equal(observed, expected_frame)
        candidate_start = self._uint(self._meta(candidate), "candidate_window_start")
        self.assertEqual(self._uint(frame_meta, "absolute_sample_index"),
                         candidate_start + 100)
        self.assertEqual(self._uint(frame_meta, "frame_start"), candidate_start + 100)
        self.assertEqual(self._uint(frame_meta, "coarse_peak"),
                         candidate_start + zc_local)
        self.assertEqual(self._uint(frame_meta, "rx_time_tag_offset"), 9876)
        for key in ("recv_id", "prs_cp_cfo_hz", "prs_cp_cfo_coherence",
                    "selected_cfo_hz", "cfo", "prs_start_rel", "prs_len",
                    "attempt_id", "window_id"):
            self.assertFalse(pmt.is_null(
                pmt.dict_ref(frame_meta, pmt.intern(key), pmt.PMT_NIL)), key)
        rx_time = pmt.dict_ref(frame_meta, pmt.intern("rx_time"), pmt.PMT_NIL)
        observed_time = (pmt.to_uint64(pmt.tuple_ref(rx_time, 0)) +
                         pmt.to_double(pmt.tuple_ref(rx_time, 1)))
        self.assertAlmostEqual(observed_time, 100.25 + 100 / self.FS, places=9)
        self.assertEqual(self._uint(release, "candidate_id"), 1)

    def test_positive_and_negative_cfo(self):
        for index, injected in enumerate((3500.0, -4200.0), start=10):
            with self.subTest(cfo_hz=injected):
                candidate, _, _, zc_local = self._candidate(
                    candidate_id=index, cfo_hz=injected)
                frames, diag, _ = self._process(candidate)
                self.assertEqual(frames.num_messages(), 1)
                self.assertAlmostEqual(
                    self._double(diag, "preamble_cfo_hz"), injected, delta=2.0)
                self.assertEqual(self._uint(diag, "selected_local_zc_index"), zc_local)

    def test_first_crossing_does_not_control_zc(self):
        selected = []
        for index, crossing in enumerate((80, 120, 165), start=20):
            candidate, _, _, _ = self._candidate(
                candidate_id=index, first_crossing_offset=crossing)
            frames, diag, _ = self._process(candidate)
            self.assertEqual(frames.num_messages(), 1)
            selected.append(self._uint(diag, "selected_local_zc_index"))
            candidate_start = self._uint(self._meta(candidate), "candidate_window_start")
            expected_delta = (candidate_start + selected[-1] -
                              (candidate_start + crossing))
            self.assertEqual(
                pmt.to_long(pmt.dict_ref(
                    diag,
                    pmt.intern("zc_selected_minus_first_preamble_crossing_samples"),
                    pmt.PMT_NIL)),
                expected_delta)
        self.assertEqual(len(set(selected)), 1)

    def test_low_amplitude_preserves_normalized_zc_metric(self):
        metrics = []
        for index, amplitude in enumerate((1.0, 1e-4), start=30):
            candidate, _, _, _ = self._candidate(
                candidate_id=index, amplitude=amplitude)
            frames, diag, _ = self._process(candidate)
            self.assertEqual(frames.num_messages(), 1)
            metrics.append(self._double(diag, "coarse_metric"))
        self.assertAlmostEqual(metrics[0], metrics[1], places=5)

    def test_zc_failure_releases_credit(self):
        candidate, _, _, _ = self._candidate(candidate_id=40, valid_zc=False)
        frames, diag, release = self._process(candidate)
        self.assertEqual(frames.num_messages(), 0)
        self.assertEqual(self._symbol(diag, "failure_reason"), "ZC_SYNC")
        self.assertFalse(pmt.is_null(
            pmt.dict_ref(diag, pmt.intern("frame_start"), pmt.PMT_NIL)))
        self.assertEqual(self._uint(release, "candidate_id"), 40)

    def test_invalid_preamble_releases_credit(self):
        candidate, _, _, _ = self._candidate(candidate_id=41, valid_preamble=False)
        frames, diag, release = self._process(candidate)
        self.assertEqual(frames.num_messages(), 0)
        self.assertEqual(self._symbol(diag, "failure_reason"), "PREAMBLE_INVALID")
        self.assertGreater(self._uint(diag, "zc_search_width"), 0)
        self.assertEqual(self._uint(release, "candidate_id"), 41)

    def test_earliest_and_latest_frame_geometry(self):
        early, _, _, early_zc = self._candidate(
            candidate_id=50, frame_start=0,
            fixed_search=(self.coarse_rel, self.coarse_rel))
        frames, diag, _ = self._process(early)
        self.assertEqual(frames.num_messages(), 1)
        self.assertEqual(self._uint(diag, "selected_local_zc_index"), early_zc)

        late_frame_start = 150
        late_zc = late_frame_start + self.coarse_rel
        late, samples, _, _ = self._candidate(
            candidate_id=51, frame_start=late_frame_start,
            fixed_search=(late_zc, late_zc))
        frames, diag, _ = self._process(late)
        self.assertEqual(frames.num_messages(), 1)
        self.assertEqual(self._uint(diag, "selected_local_zc_index"), late_zc)
        observed = numpy.asarray(
            pmt.c32vector_elements(pmt.cdr(frames.get_message(0))),
            dtype=numpy.complex64)
        numpy.testing.assert_array_equal(
            observed, samples[late_frame_start:late_frame_start + self.frame_len])

    def test_invalid_geometry_releases_credit(self):
        candidate, _, _, _ = self._candidate(candidate_id=60)
        meta = self._meta(candidate)
        meta = pmt.dict_add(meta, pmt.intern("fft_len"), pmt.from_uint64(64))
        malformed = pmt.cons(meta, pmt.cdr(candidate))
        frames, diag, release = self._process(malformed)
        self.assertEqual(frames.num_messages(), 0)
        self.assertEqual(self._symbol(diag, "failure_reason"), "INVALID_GEOMETRY")
        self.assertEqual(self._uint(release, "candidate_id"), 60)

        short_meta = self._meta(candidate)
        short_meta = pmt.dict_add(
            short_meta, pmt.intern("candidate_id"), pmt.from_uint64(61))
        short_candidate = pmt.cons(
            short_meta, pmt.init_c32vector(10, numpy.zeros(10, numpy.complex64)))
        frames, diag, release = self._process(short_candidate)
        self.assertEqual(frames.num_messages(), 0)
        self.assertEqual(self._symbol(diag, "failure_reason"), "INVALID_GEOMETRY")
        self.assertEqual(self._uint(release, "candidate_id"), 61)

    def test_window_capture_to_zc_sync_path(self):
        frame_start = 500
        frame = self._frame()
        stream = numpy.zeros(frame_start + len(frame) + 2000, dtype=numpy.complex64)
        stream[frame_start:frame_start + len(frame)] = frame
        rx_tag = gr.tag_t()
        rx_tag.offset = 0
        rx_tag.key = pmt.intern("rx_time")
        rx_tag.value = pmt.make_tuple(pmt.from_uint64(5), pmt.from_double(0.5))
        rx_tag.srcid = pmt.intern("qa")
        source = blocks.vector_source_c(stream, False, 1, [rx_tag])
        throttle = blocks.throttle(gr.sizeof_gr_complex, self.FS, True)
        capture = ofdm_prs_ranging.prs_window_capture(
            self.FS, self.FFT, self.CP, self.ACTIVE, self.NSYM, self.L, self.R,
            self.NZ, self.PREAMBLE_THRESHOLD, 300, 64, 64,
            1, True, False)
        sync = self._sync()
        legacy = ofdm_prs_ranging.prs_frame_detector(
            self.FS, self.FFT, self.CP, self.ACTIVE, self.NSYM, self.L, self.R,
            self.NZ, self.PREAMBLE_THRESHOLD, 300, self.ROOT,
            self.CHANNEL, False, 0.05, 0.0002, 0.004, self.ZC_THRESHOLD, False)
        frames = blocks.message_debug()
        legacy_frames = blocks.message_debug()
        diagnostics = blocks.message_debug()
        capture_events = blocks.message_debug()
        tb = gr.top_block()
        tb.connect(source, throttle)
        tb.connect(throttle, capture)
        tb.connect(throttle, legacy)
        tb.msg_connect(capture, "candidate_out", sync, "candidate_in")
        tb.msg_connect(sync, "slot_release", capture, "slot_release")
        tb.msg_connect(sync, "synchronized_frame_out", frames, "store")
        tb.msg_connect(sync, "diagnostics_out", diagnostics, "store")
        tb.msg_connect(capture, "event_out", capture_events, "store")
        tb.msg_connect(legacy, "frame_out", legacy_frames, "store")
        tb.start()
        deadline = time.monotonic() + 3.0
        while ((frames.num_messages() == 0 or legacy_frames.num_messages() == 0 or
                diagnostics.num_messages() == 0) and
               time.monotonic() < deadline):
            time.sleep(0.002)
        tb.stop()
        tb.wait()
        self.assertEqual(frames.num_messages(), 1)
        self.assertEqual(legacy_frames.num_messages(), 1)
        self.assertEqual(diagnostics.num_messages(), 1)
        output = frames.get_message(0)
        legacy_output = legacy_frames.get_message(0)
        output_meta = self._meta(output)
        legacy_meta = self._meta(legacy_output)
        self.assertEqual(self._uint(output_meta, "frame_start"), frame_start)
        self.assertEqual(self._uint(output_meta, "coarse_peak"),
                         frame_start + self.coarse_rel)
        self.assertEqual(self._uint(output_meta, "frame_start"),
                         self._uint(legacy_meta, "frame_start"))
        self.assertEqual(self._uint(output_meta, "coarse_peak"),
                         self._uint(legacy_meta, "coarse_peak"))
        numpy.testing.assert_array_equal(
            numpy.asarray(pmt.c32vector_elements(pmt.cdr(output)), dtype=numpy.complex64),
            frame)
        numpy.testing.assert_array_equal(
            numpy.asarray(pmt.c32vector_elements(pmt.cdr(output)), dtype=numpy.complex64),
            numpy.asarray(pmt.c32vector_elements(pmt.cdr(legacy_output)),
                          dtype=numpy.complex64))
        self.assertGreaterEqual(capture_events.num_messages(), 1)


if __name__ == '__main__':
    gr_unittest.run(qa_prs_zc_frame_sync)
